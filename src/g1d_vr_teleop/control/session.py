"""One explicit dual-arm VR session using the existing sole DDS publisher.

No startup/column/exit RPC is issued. Normal completion returns to the saved
joint pose, then pauses. Fault/disconnect/TERM pauses without an automatic path.
Recording starts only after A; task clips start at measured motion. Default is
read-only preparation.
"""

from g1d_vr_teleop.config import URDF, ROBOT_HOST, DDS_INTERFACE, STATE_ROOT
import argparse
from dataclasses import replace, dataclass
import json
import math
from pathlib import Path
import signal
import threading
import time
import uuid

import numpy as np

from g1d_vr_teleop.core.dual_arm import (
    DualArmState, DualVRControl, DualVRDecision, XRInput, VRControlError,
    build_payloads, build_return_trajectory, parse_urdf_limits, validate_ready_status,
)
from g1d_vr_teleop.control.profile import SiteDualVRControl


C = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def robot_poses(frame):
    # Fixed-session controller world deltas: head motion does not move an arm.
    # This permits event-driven head updates without using an old head pose.
    result = []
    for side in ('left', 'right'):
        raw = np.asarray(frame[side], float).reshape(4, 4, order='F')
        out = np.eye(4)
        out[:3, :3] = C @ raw[:3, :3] @ C.T
        out[:3, 3] = C @ raw[:3, 3]
        result.append(out)
    return tuple(result)


class StartButtonLatch:
    """Require a new A edge after observing release in this XR session.

    Event counters preserve quick clicks across 50 Hz transport sampling.
    Already-held A and edges that preceded this listener never start motion.
    """
    def __init__(self):
        self.source = None
        self.baseline = None

    def update(self, frame):
        source = frame['source_id']
        pressed = frame['right_state']['a_button']
        count = frame['right_a_press_count']
        if source != self.source:
            self.source, self.baseline = source, None
        if self.baseline is None:
            if not pressed:
                self.baseline = count
            return False
        return count > self.baseline


class FinishButtonHold:
    """A new, continuously held A ends normally; quick confirmation taps do not."""
    def __init__(self, start_count):
        self.start_count = start_count
        self.held_since_ns = None
        self.held_count = None

    def update(self, frame):
        if (not frame['right_state']['a_button'] or
                frame['right_a_press_count'] <= self.start_count):
            self.held_since_ns = None
            self.held_count = None
            return False
        stamp = frame['controller_monotonic_ns']
        if self.held_since_ns is None or self.held_count != frame['right_a_press_count']:
            self.held_since_ns = stamp
            self.held_count = frame['right_a_press_count']
        return stamp-self.held_since_ns >= 1_000_000_000


def rotation_angle(rotation):
    return math.acos(float(np.clip((np.trace(rotation) - 1.) / 2., -1., 1.)))


def anchored_targets(frame, reference_xr, reference_fk):
    import pinocchio as pin
    current = robot_poses(frame)
    targets, translations, rotvecs = [], [], []
    for pose, anchor, fk in zip(current, reference_xr, reference_fk):
        delta = pose[:3, 3] - anchor[:3, 3]
        rotation = pose[:3, :3] @ anchor[:3, :3].T
        rotation_vector=np.asarray(pin.log3(rotation))
        rotation=np.asarray(pin.exp3(rotation_vector))
        target = fk.copy()
        target[:3, 3] += delta
        target[:3, :3] = rotation @ fk[:3, :3]
        targets.append(target)
        translations.append(tuple(float(x) for x in delta))
        # Only the vector norm is consumed by the numerical range guard.
        rotvecs.append(tuple(float(x) for x in rotation_vector))
    return tuple(targets), tuple(translations), tuple(rotvecs)


def make_xr_input(frame, delta, rotations):
    """Use the identical event representation for first and later commands."""
    left, right = frame['left_state'], frame['right_state']
    trigger = tuple(float(np.clip(13.5 - 27. * side['trigger_value'], .28, 5.38))
                    for side in (left, right))
    return XRInput(frame['controller_seq'], frame['controller_monotonic_ns'],
                   delta[0], delta[1], rotations[0], rotations[1],
                   bool(left['squeeze']), bool(right['squeeze']), trigger,
                   session_enabled=True)


@dataclass(frozen=True)
class SourceArmState(DualArmState):
    source_tick: int
    source_timestamp_ns: int


class DirectFeedbackClient:
    """Public RPC client with an independent passive, latest-only state feed."""
    def __init__(self, rpc_client, source):
        self.rpc_client, self.source = rpc_client, source

    def __getattr__(self, name):
        return getattr(self.rpc_client, name)

    def read_arm_state(self):
        sample = self.source.snapshot(max_age_s=.2)
        return SourceArmState(sample.q, sample.received_monotonic_ns,
                              sample.mode_machine, sample.tick, sample.source_timestamp_ns)


def read_state(client):
    if isinstance(client, DirectFeedbackClient):
        return client.read_arm_state()
    for _ in range(4):
        before = client.state_received_at
        state = client.latest_state
        after = client.state_received_at
        if before == after:
            q = [float(m.q) for m in state.motor_state[:29]]
            q[13] = 0.  # Invalid G1D slot; no physical encoder is inferred.
            return DualArmState(tuple(q), round(after * 1e9), int(state.mode_machine))
    raise VRControlError('Cannot acquire a consistent fresh robot state')


def capture_ready_start(client):
    """Acquire a fresh READY baseline; a prior waiting pose is not the goal."""
    status = client.status()
    validate_ready_status(status)
    return status, read_state(client)


class XRReceiver(threading.Thread):
    # Transport and original event use the same freshness budget. A separate
    # 150 ms transport guard incorrectly rejected still-valid <200 ms events.
    MAX_EVENT_AGE_NS = 200_000_000

    def __init__(self):
        super().__init__(name='vr-event-receiver', daemon=True)
        self.lock, self.stop_event = threading.Lock(), threading.Event()
        self.latest = None
        self.error = None

    def run(self):
        import zmq
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.SUBSCRIBE, b'')
        sock.setsockopt(zmq.RCVTIMEO, 50)
        sock.connect('tcp://127.0.0.1:6014')
        try:
            while not self.stop_event.is_set():
                try:
                    value = sock.recv_json()
                except zmq.Again:
                    continue
                with self.lock:
                    self.latest = (value, time.monotonic_ns())
        except Exception as exc:
            self.error = str(exc)
        finally:
            sock.close(0)
            ctx.term()

    def snapshot(self):
        if self.error:
            raise VRControlError('XR receiver failed: ' + self.error)
        with self.lock:
            item = self.latest
        if item is None:
            raise VRControlError('Waiting for XR input transport')
        value, received = item
        now = time.monotonic_ns()
        if not 0 <= now - received <= self.MAX_EVENT_AGE_NS:
            raise VRControlError('XR input transport is stale')
        if value.get('valid') is not True:
            raise VRControlError(value.get('reason', 'Invalid XR input'))
        frame = value['frame']
        if frame.get('pose_reference') != 'controller_world':
            raise VRControlError('XR publisher must supply validated controller-world poses')
        stamps = [frame['controller_monotonic_ns']]
        if any(type(s) is not int or not 0 <= now - s <= self.MAX_EVENT_AGE_NS for s in stamps):
            raise VRControlError('Original controller events are stale')
        return frame


class IKWorker(threading.Thread):
    def __init__(self, ik, client, receiver, anchor_xr, anchor_fk):
        super().__init__(name='vr-cpu-ik', daemon=True)
        self.ik, self.client, self.receiver = ik, client, receiver
        self.anchor_xr, self.anchor_fk = anchor_xr, anchor_fk
        self.lock, self.stop_event = threading.Lock(), threading.Event()
        self.result, self.error = None, None
        self.takeover_started = None
        self.takeover_initial_q = None

    def solve(self, frame):
        import pinocchio as pin
        targets, _, _ = anchored_targets(frame, self.anchor_xr, self.anchor_fk)
        state = read_state(self.client)
        q = np.array(state.q[15:29])
        ik = self.ik
        ik.opti.set_initial(ik.var_q, q)
        ik.opti.set_value(ik.var_q_last, q)
        ik.opti.set_value(ik.param_tf_l, targets[0])
        ik.opti.set_value(ik.param_tf_r, targets[1])
        solution = ik.opti.solve()  # No legacy catch-and-return-current fallback.
        if solution.stats().get('success') is not True:
            raise VRControlError('IK solver did not report success')
        candidate = np.asarray(solution.value(ik.var_q), float).reshape(14)
        if not np.isfinite(candidate).all():
            raise VRControlError('IK returned nonfinite joints')
        # Smooth IK solutions across four frames.
        ik.smooth_filter.add_data(candidate)
        candidate = np.asarray(ik.smooth_filter.filtered_data,float).reshape(14)
        if self.takeover_started is not None:
            u = float(np.clip((time.monotonic()-self.takeover_started)/2.,0.,1.))
            blend = u*u*(3.-2.*u)
            candidate = (1.-blend)*self.takeover_initial_q + blend*candidate
        return tuple(candidate)

    def run(self):
        while not self.stop_event.is_set():
            try:
                frame = self.receiver.snapshot()
                candidate = self.solve(frame)
                with self.lock:
                    self.result = (candidate, time.monotonic_ns(), frame['controller_monotonic_ns'])
            except Exception as exc:
                self.error = f'{type(exc).__name__}: {exc}'
                return
            self.stop_event.wait(.025)

    def snapshot(self):
        if self.error:
            raise VRControlError('IK stopped: ' + self.error)
        with self.lock:
            result = self.result
        if result is None:
            raise VRControlError('IK has not produced a solution')
        # Retain the last solved target while
        # the worker computes its next solution. Live controller transport,
        # original controller events and robot/Dex feedback are checked by the
        # sender independently; computation time alone does not abort a task.
        return result[0]


class Session:
    def __init__(self, client, root, limits, *, execute=False, duration=60., recording=None):
        self.client, self.root, self.limits = client, root, limits
        self.execute, self.duration = execute, duration
        self.abort = threading.Event()
        self.sender, self.context, self.ik_worker = None, None, None
        self.policy_attempted, self.paused = False, False
        self.sequence, self.session_id = 0, uuid.uuid4().hex
        self.last_sent, self.sent = None, 0
        self.command_timeout_s = .5
        self.phase = 'preparing'
        self.recording = recording
        self.motion_start = None
        self.motion_candidates = []
        self.motion_last_state_ns = None
        self.trace=[]
        self.pending_ik=None
        self.b_press_baseline = None
        self.grippers_opened = False
        self.final_gripper_positions = None

    def check_abort_button(self, frame):
        count = frame['b_press_count']
        if (any(frame[side]['b_button'] for side in ('left_state','right_state')) or
                (self.b_press_baseline is not None and count != self.b_press_baseline)):
            raise VRControlError('VR B button abort')

    def report(self, phase, **extra):
        self.phase = phase
        data = {'phase': phase, 'robot_control_enabled': self.policy_attempted and not self.paused,
                'commands_sent': self.sent,
                'recording': bool(self.recording and self.recording.processes and not self.recording.stopped),
                'recording_directory': str(self.recording.output) if self.recording else None,
                'monotonic_s': time.monotonic(),
                'arm_feedback_source':'direct_DDS_latest_only' if isinstance(self.client,DirectFeedbackClient) else 'ZMQ_transport',
                'grippers_opened':self.grippers_opened,
                **extra}
        tmp = self.root / 'control-status.tmp'
        tmp.write_text(json.dumps(data, indent=2) + '\n')
        tmp.replace(self.root / 'control-status.json')

    def open_sender(self):
        import zmq
        self.context = zmq.Context()
        self.sender = self.context.socket(zmq.PUSH)
        for key, value in ((zmq.LINGER, 0), (zmq.SNDHWM, 4), (zmq.IMMEDIATE, 1), (zmq.SNDTIMEO, 30)):
            self.sender.setsockopt(key, value)
        self.sender.connect(f'tcp://{ROBOT_HOST}:6000')
        self.abort.wait(.15)  # Connection setup only, no packet.

    def send(self, decision):
        from ial_g1d.robot.protocol import encode_command_envelope, make_lowcmd, make_gripper_command
        if self.abort.is_set():
            raise VRControlError('Operator abort requested')
        now = time.monotonic()
        observed=read_state(self.client)
        if self.last_sent is not None and now - self.last_sent > self.command_timeout_s:
            raise VRControlError(f'Command gap {now-self.last_sent:.6f}s exceeds bridge watchdog {self.command_timeout_s:g}s')
        for topic, data in build_payloads(decision, make_lowcmd=make_lowcmd,
                                          make_gripper_command=make_gripper_command):
            self.sender.send(encode_command_envelope(topic, data,
                sequence=self.sequence, session_id=self.session_id))
            self.sequence += 1
        self.last_sent, self.sent = time.monotonic(), self.sent + 1
        self.trace.append({'sent_monotonic_s':self.last_sent,'frame':self.sent,
            'state_monotonic_ns':observed.received_monotonic_ns,'measured_arm':observed.q[15:29],
            'state_source':'direct_DDS_latest_only' if isinstance(observed,SourceArmState) else 'ZMQ_transport',
            'source_tick':getattr(observed,'source_tick',None),
            'source_timestamp_ns':getattr(observed,'source_timestamp_ns',None),
            'command_arm':decision.arm_q,'command_velocity':decision.arm_velocity,
            'ik_target':self.pending_ik,'phase':self.phase})

    def pause_once(self):
        if self.ik_worker is not None:
            self.ik_worker.stop_event.set()
        # No producer remains after closing the sole sender. RPC does not flush
        # old queues, so this process never automatically reattaches.
        if self.sender is not None:
            self.sender.close(0)
            self.sender = None
        if self.policy_attempted and not self.paused:
            self.paused = True  # Latch before request; never retry an ACK loss.
            # Reuse the deployed sampler's public protocol-v3 RPC. This site's
            # old G1DClient rejects pause_policy before sending any request.
            from g1d_vr_teleop.core.pause import pause_once
            return pause_once(ROBOT_HOST,6002)
        return None

    def open_grippers(self, core, previous, arm_q):
        """Open both Dex1 only during an explicit, healthy normal reset."""
        self.report('opening_grippers_after_return')
        began,stable=time.monotonic(),None
        while True:
            tick=time.monotonic()
            raw=[reader.snapshot() for reader in self.grippers]
            if any(not math.isfinite(r.q) or not .18 <= r.q <= 5.48 for r in raw):
                raise VRControlError('Gripper feedback outside calibrated endpoint tolerance')
            decision=self.return_decision(core,previous,arm_q,(0.,)*14)
            decision=replace(decision,grippers=tuple(max(.28,min(5.38,r.q+.18)) for r in raw))
            self.send(decision)
            if all(r.q>=5.33 for r in raw):
                stable=tick if stable is None else stable
                if tick-stable>=.2:
                    self.final_gripper_positions=tuple(r.q for r in raw)
                    self.grippers_opened=True
                    return self.final_gripper_positions
            else:stable=None
            if tick-began>8.:
                raise VRControlError('Opening grippers did not finish')
            self.abort.wait(max(0.,.02-(time.monotonic()-tick)))

    def return_saved(self, core, previous):
        if getattr(core,'site_profile',False):
            # Site targets are a PD lead about measured q, not a trajectory
            # endpoint or measured velocity. Begin the normal return by holding
            # the measured joint positions; waiting for the old lead target
            # cannot settle under static bias.
            self.report('returning_saved_pose')
            measured = tuple(read_state(self.client).q[15:29])
            previous = self.return_decision(core,previous,measured,(0.,)*14)
            self.send(previous)
        self.report('returning_saved_pose')
        # Brake the last commanded velocity before constructing a zero-endpoint
        # quintic. Preserve grip position and fixed body throughout.
        q = np.array(previous.arm_q, float)
        velocity = np.array(previous.arm_velocity, float)
        old = time.monotonic()
        while np.max(np.abs(velocity)) > 1e-8:
            tick = time.monotonic()
            dt = min(.04, max(.001, tick - old))
            velocity -= np.clip(velocity, -core.acceleration_limit*dt, core.acceleration_limit*dt)
            q += velocity * dt
            decision = self.return_decision(core, previous, q, velocity)
            self.send(decision)
            old = tick
            self.abort.wait(max(0., .02 - (time.monotonic() - tick)))
        trajectory = build_return_trajectory(q,self.saved_arm_q,self.limits,
            velocity_limit=min(core.velocity_limit,.10),acceleration_limit=core.acceleration_limit,minimum_duration_s=1.,
            **({'start_joint_margin':0.} if getattr(core,'site_profile',False) else {}))
        began, stable = time.monotonic(), None
        while True:
            tick = time.monotonic()
            sample = trajectory.sample(tick - began)
            decision = self.return_decision(core, previous, sample.q, sample.velocity)
            self.send(decision)
            if tick - began >= trajectory.duration_s:
                measured = np.array(read_state(self.client).q[15:29])
                if np.max(np.abs(measured - np.array(trajectory.saved_q))) <= .05:
                    stable = tick if stable is None else stable
                    if tick - stable > .5:
                        self.open_grippers(core,decision,trajectory.saved_q)
                        return
                else:
                    stable = None
                if tick - began > trajectory.duration_s + 10.:
                    raise VRControlError('Saved-pose return did not settle')
            self.abort.wait(max(0., .02 - (time.monotonic() - tick)))

    def return_decision(self, core, previous, arm_q, velocity, *, tracking_limit=.08):
        if hasattr(self,'receiver'):
            try:
                frame=self.receiver.snapshot()
            except VRControlError:
                pass
            else:
                self.check_abort_button(frame)
        state = read_state(self.client)
        now_ns = time.monotonic_ns()
        if now_ns - state.received_monotonic_ns > 200_000_000 or state.mode_machine != core.mode_machine:
            raise VRControlError('Return state is stale or mode changed')
        body = np.array(core.body_q)
        measured = np.array(state.q)
        if max(abs(measured[12] - body[12]), abs(measured[14] - body[14])) > .01:
            raise VRControlError('Body moved during saved-pose return')
        arm_q = np.array(arm_q)
        if np.max(np.abs(measured[15:29] - arm_q)) > tracking_limit:
            raise VRControlError(f'Return tracking error exceeds {tracking_limit:g} rad')
        core._check_arm(arm_q)
        body[15:29] = arm_q
        # Require fresh, healthy original gripper feedback even while holding.
        for reader in self.grippers:
            reader.snapshot()
        return replace(previous, body_q=tuple(body), arm_q=tuple(arm_q),
                       arm_velocity=tuple(velocity), first_target=False)

    def observe_motion(self, state, command):
        if self.motion_start is not None or state.received_monotonic_ns == self.motion_last_state_ns:
            return
        self.motion_last_state_ns = state.received_monotonic_ns
        origin=self.motion_baseline_q
        moved = max(abs(a-b) for a,b in zip(state.q[15:29],origin)) > .001
        requested = max(abs(a-b) for a,b in zip(command.arm_q,origin)) > .001
        if moved and requested:
            self.motion_candidates.append(state.received_monotonic_ns / 1e9)
            if len(self.motion_candidates) == 3:
                self.motion_start = self.motion_candidates[0]
        else:
            self.motion_candidates.clear()

    def run(self, ik, receiver, grippers):
        import pinocchio as pin
        self.grippers = grippers
        self.receiver = receiver
        initial_status = self.client.status()
        validate_ready_status(initial_status)
        self.saved = read_state(self.client)
        reference_path=STATE_ROOT/'initial-pose.json'
        reference=json.loads(reference_path.read_text())
        if reference['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise VRControlError('Fixed initial pose belongs to another boot; explicit posture initialization required')
        self.saved_arm_q=tuple(float(x) for x in reference['arm_q'])
        build_return_trajectory(self.saved.q[15:29],self.saved_arm_q,self.limits)
        (self.root / 'saved-initial-pose.json').write_text(json.dumps({
            'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'state': self.saved.__dict__, 'status': initial_status,
            'fixed_initial_reference':str(reference_path),'fixed_initial_arm_q':self.saved_arm_q,
            'start': 'attach at measured current pose without initial restoration',
            'normal_end': 'return to fixed saved arm pose within 0.05 rad, then open both grippers; preserve body',
            'fault_end': 'pause and hold; no automatic return', 'recording': False}, indent=2)+'\n')
        self.report('waiting_right_A_click',start_mode='right_A_click',return_pose_saved=True)
        start_button, last_report = StartButtonLatch(), 0.
        while not self.abort.is_set():
            try:
                frame = receiver.snapshot()
                left, right = frame['left_state'], frame['right_state']
                pressed = right['a_button']
                start_clicked = start_button.update(frame)
                if time.monotonic()-last_report>.5:
                    self.report('waiting_right_A_click',right_A=pressed,start_released_seen=start_button.baseline is not None,
                                right_a_press_count=frame['right_a_press_count'],
                                left_trigger=left['trigger_value'],right_trigger=right['trigger_value'])
                    last_report=time.monotonic()
                if start_clicked:
                    break
            except VRControlError as exc:
                if time.monotonic() - last_report > .5:
                    self.report('waiting_right_A_click',reason=str(exc),start_released_seen=start_button.baseline is not None)
                    last_report = time.monotonic()
            self.abort.wait(.02)
        if self.abort.is_set():
            return
        if not self.execute:
            self.report('read_only_inputs_verified', command_gate_opened=False)
            return
        source_id = frame['source_id']
        self.b_press_baseline = frame['b_press_count']
        start_a_count = frame['right_a_press_count']
        self.check_abort_button(frame)
        initial_status, current = capture_ready_start(self.client)
        core = SiteDualVRControl(initial_status,current,self.limits,now_ns=time.monotonic_ns(),
                                 activation_mode='latched')
        self.command_timeout_s = core.command_timeout_s
        anchor_xr = robot_poses(frame)
        model, data = ik.reduced_robot.model, model_data(ik)
        pin.framesForwardKinematics(model, data, np.array(current.q[15:29]))
        anchor_fk = tuple(data.oMf[fid].homogeneous.copy() for fid in (ik.L_hand_id, ik.R_hand_id))
        self.ik_worker = IKWorker(ik, self.client, receiver, anchor_xr, anchor_fk)
        # Solve once before opening the policy gate; failure leaves READY hold.
        self.ik_worker.result = (self.ik_worker.solve(frame), time.monotonic_ns(), frame['controller_monotonic_ns'])
        if self.recording:
            self.report('preparing_three_video_action_recording')
            self.recording.prepare(self.abort)
        self.open_sender()
        # Recording preparation may take seconds. Revalidate all inputs and
        # bridge ownership before opening the command gate.
        frame = receiver.snapshot()
        self.check_abort_button(frame)
        if frame['source_id'] != source_id:
            raise VRControlError('XR session changed during preparation')
        # Before activating, wait for both grippers' genuinely fresh samples.
        # Once active, brief gaps hold their last targets without ending VR.
        while True:
            raw_start = tuple(reader.snapshot() for reader in grippers)
            now_start = time.monotonic_ns()
            if all(0 <= now_start-r.received_monotonic_ns <= 100_000_000 for r in raw_start):
                break
            self.report('waiting_fresh_gripper_feedback')
            frame = receiver.snapshot()
            self.check_abort_button(frame)
            if frame['source_id'] != source_id:
                raise VRControlError('XR session changed while awaiting gripper feedback')
            if self.abort.wait(.02):
                raise VRControlError('Start interrupted while awaiting gripper feedback')
        start_status, current = capture_ready_start(self.client)
        (self.root/'session-start-measured-pose.json').write_text(json.dumps({
            'state':current.__dict__,'status':start_status,
            'exit_reference':str(reference_path),
            'start':'latest measured pose immediately before policy activation'},indent=2)+'\n')
        anchor_xr = robot_poses(frame)
        pin.framesForwardKinematics(model,data,np.array(current.q[15:29]))
        anchor_fk = tuple(data.oMf[fid].homogeneous.copy() for fid in (ik.L_hand_id,ik.R_hand_id))
        self.ik_worker.anchor_xr,self.ik_worker.anchor_fk = anchor_xr,anchor_fk
        # The first accepted target remains the newest measured q; do not wait
        # for an IK solve after the gate ACK.
        self.ik_worker.result = (tuple(current.q[15:29]),time.monotonic_ns(),frame['controller_monotonic_ns'])
        self.policy_attempted = True
        response = self.client.request_safety('start_policy')
        if response.get('success') is not True or response.get('policy_active') is not True:
            raise VRControlError('Policy gate did not acknowledge activation')
        # The first packet is always the newest measured position. Start here;
        # the fixed initial reference is used only on normal exit, per operator.
        frame=receiver.snapshot()
        current=read_state(self.client)
        self.ik_worker.takeover_initial_q = np.array(current.q[15:29])
        self.ik_worker.takeover_started = time.monotonic()
        _, first_delta, first_rotations = anchored_targets(frame,anchor_xr,anchor_fk)
        first_xr = make_xr_input(frame,first_delta,first_rotations)
        raw=tuple(reader.snapshot() for reader in grippers)
        previous=core.step(current,first_xr,current.q[15:29],*raw,now_ns=time.monotonic_ns())
        self.send(previous)
        began, report_at = time.monotonic(), 0.
        self.motion_baseline_q=tuple(read_state(self.client).q[15:29])
        finish_button = FinishButtonHold(start_a_count)
        self.ik_worker.start()
        while not self.abort.is_set():
            tick = time.monotonic()
            frame = receiver.snapshot()
            if frame['source_id'] != source_id:
                raise VRControlError('XR session changed; manual restart required')
            left, right = frame['left_state'], frame['right_state']
            self.check_abort_button(frame)
            sample_elapsed=tick-(self.motion_start if self.motion_start is not None else began)
            finish_held = finish_button.update(frame)
            normal_end = (finish_held or
                          (self.motion_start is not None and sample_elapsed >= self.duration) or
                          (self.motion_start is None and tick-began >= self.duration) or
                          (self.root / 'finish.request').exists())
            if normal_end and previous is not None:
                self.ik_worker.stop_event.set()
                self.normal_finish_reason = ('right_A_held_1s' if finish_held else
                    ('finish_request_file' if (self.root/'finish.request').exists() else
                    ('duration_60s' if self.motion_start is not None else 'no_motion_60s')))
                if self.recording:
                    end = min(tick, self.motion_start+self.duration) if self.motion_start else None
                    self.recording.request_stop(self.motion_start,end)
                self.return_saved(core, previous)
                self.report('returned_saved_pose')
                return
            targets, delta, rotations = anchored_targets(frame, anchor_xr, anchor_fk)
            xr = make_xr_input(frame,delta,rotations)
            state = read_state(self.client)
            raw = tuple(reader.snapshot() for reader in grippers)
            self.pending_ik=self.ik_worker.snapshot()
            decision = core.step(state,xr,self.pending_ik,*raw,now_ns=time.monotonic_ns())
            if decision.pause_required:
                raise VRControlError('Episode ended before first command')
            self.send(decision)
            self.observe_motion(state,decision)
            previous = decision
            if tick >= report_at:
                if self.recording:
                    self.recording.check()
                self.report('vr_active',elapsed_s=tick-began,sample_elapsed_s=None if self.motion_start is None else tick-self.motion_start,
                            control_profile=core.profile_metadata,mapping_scale=1.,right_A=right['a_button'],
                            right_a_press_count=frame['right_a_press_count'],
                            normal_finish='hold right A 1 second or 60-second duration')
                report_at = tick + .5
            self.abort.wait(max(0., .02 - (time.monotonic() - tick)))
        raise VRControlError('Session interrupted')


def model_data(ik):
    return ik.reduced_robot.model.createData()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--duration', type=float, default=60.)
    p.add_argument('--record-output', type=Path)
    a = p.parse_args(argv)
    if not math.isfinite(a.duration) or not 0 < a.duration <= 60.:
        p.error('duration must be within (0,60]')
    root = a.directory.resolve()
    root.mkdir(parents=True, exist_ok=True)
    import fcntl
    STATE_ROOT.mkdir(parents=True,exist_ok=True)
    owner_lock=(STATE_ROOT/'control-owner.lock').open('a+')
    fcntl.flock(owner_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    from ial_g1d.robot.client import G1DClient, G1DClientConfig
    from g1d_vr_teleop.feedback.grippers import LatestDex1Subscriber
    from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
    limits = parse_urdf_limits(URDF.read_text())
    client = G1DClient(G1DClientConfig(robot_ip=ROBOT_HOST, control_mode='dual_arm',
                        enable_agv=False, max_state_age_s=.2, safety_timeout_s=3.))
    receiver, grippers, arm_source = XRReceiver(), [], None
    from g1d_vr_teleop.recording.manager import Recording
    recording = Recording(a.record_output.resolve()) if a.record_output else None
    session = Session(client,root,limits,execute=a.execute,duration=a.duration,recording=recording)
    signal.signal(signal.SIGINT, lambda *_: session.abort.set())
    signal.signal(signal.SIGTERM, lambda *_: session.abort.set())
    error = None
    try:
        client.connect(command_socket=False)
        validate_ready_status(client.status())
        # Constructor is CPU-only; run from model-cache cwd, never a controller.
        ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0,DDS_INTERFACE)
        for side in ('left','right'):
            grippers.append(LatestDex1Subscriber(side=side,max_age_s=.5).start())
        from g1d_vr_teleop.feedback.arms import LatestArmState
        arm_source = LatestArmState().start()
        deadline = time.monotonic() + 3.
        for reader in grippers:
            while True:
                try:
                    reader.snapshot()
                    break
                except Exception:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.02)
        while True:
            try:
                arm_source.snapshot(max_age_s=.2)
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        session.client = DirectFeedbackClient(client, arm_source)
        receiver.start()
        session.run(ik, receiver, grippers)
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
        print(error, flush=True)
    finally:
        if recording and recording.processes:
            end=time.monotonic()
            recording.request_stop(session.motion_start,
                min(end,session.motion_start+a.duration) if session.motion_start else None)
        try:
            pause = session.pause_once()
            if pause is not None and (pause.get('success') is not True or pause.get('ready_hold_active') is not True):
                raise VRControlError('Pause did not confirm READY hold')
        except Exception as exc:
            error = (error or '') + f'; pause unconfirmed: {exc}'
        receiver.stop_event.set()
        for reader in grippers:
            reader.close()
        if arm_source is not None:
            arm_source.close()
        if session.context is not None:
            session.context.term()
        client.close(require_safe_exit=False)
        with (root/'control-trace.jsonl').open('x') as trace:
            for row in session.trace:
                trace.write(json.dumps(row,allow_nan=False)+'\n')
        returned = error is None and session.phase == 'returned_saved_pose'
        session.report('fault_hold' if error else 'completed', error=error,
                       returned_to_initial=returned,
                       normal_finish_reason=getattr(session,'normal_finish_reason',None))
        if recording and recording.processes:
            try:
                result=recording.finalize()
                (root/'recording-result.json').write_text(json.dumps(result,indent=2)+'\n')
            except Exception as exc:
                error=(error or '')+f'; recording finalization: {exc}'
                session.report('recording_finalize_failed',error=error,returned_to_initial=returned)
    if error:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
