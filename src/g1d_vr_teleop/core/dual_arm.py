"""Offline numerical core for one bounded, dual-arm VR episode.

No sockets, SDK, DDS, threads, clock reads or lifecycle calls. The runtime owns
the reviewed READY attach, exclusive sender, original timestamp acquisition,
IK/collision checks and pause RPC. A decision is a candidate, never an execution
acknowledgement. Release, invalid input or feedback ends this instance; there is
deliberately no reset/resume API. XR deltas are relative to ONE fixed episode
reference, not consecutive-frame deltas.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from numbers import Real
import xml.etree.ElementTree as ET

from .contact import Dex1ContactController


ARM_MOTORS = tuple(range(15, 29))
ARM_NAMES = tuple(f"{side}_{joint}_joint" for side in ("left", "right") for joint in
                  ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                   "wrist_roll", "wrist_pitch", "wrist_yaw"))
RAW_TIMESTAMP_KIND = "original_DDS_callback_host_monotonic_ns_not_producer_timestamp"
MAX_VELOCITY = .04
MAX_ACCELERATION = .05
MAX_STATE_AGE_NS = 200_000_000
MAX_XR_AGE_NS = 200_000_000
MAX_COMMAND_GAP_NS = 150_000_000
JOINT_MARGIN = .05
MAX_TRACKING_ERROR = .08
MAX_BODY_DRIFT = .01


class VRControlError(RuntimeError):
    """No command is returned; the runtime must stop its sender and pause once."""


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise VRControlError(f"{label} must be finite numeric data")
    return float(value)


def _vector(values, size, label):
    try:
        result = tuple(values)
    except TypeError as exc:
        raise VRControlError(f"{label} must contain {size} coordinates") from exc
    if len(result) != size:
        raise VRControlError(f"{label} must contain {size} coordinates")
    return tuple(_number(value, label) for value in result)


def _stamp_age(stamp, now, maximum, label):
    if type(stamp) is not int or type(now) is not int or stamp <= 0 or now < stamp:
        raise VRControlError(f"{label} timestamp is invalid or in the future")
    if now - stamp > maximum:
        raise VRControlError(f"{label} is stale")


def _body_q(values):
    try:
        q = list(values)
    except TypeError as exc:
        raise VRControlError("Body state must be 29D") from exc
    if len(q) != 29:
        raise VRControlError("Body state must be 29D")
    # Slot13 is invalid telemetry on this G1D; this is not an encoder reading.
    q[13] = 0.
    return _vector(q, 29, "body q")


def validate_joint_limits(limits):
    rows = tuple(_vector(row, 3, "URDF limit") for row in limits)
    if len(rows) != 14 or any(lo + 2 * JOINT_MARGIN >= hi or velocity <= 0
                              for lo, hi, velocity in rows):
        raise VRControlError("Fourteen ordered URDF lower/upper/velocity triples required")
    return rows


def parse_urdf_limits(xml_text):
    """Parse the exact left7/right7 URDF order without loading a robot SDK."""
    try:
        root = ET.fromstring(xml_text)
        limits = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            if name in ARM_NAMES:
                if name in limits:
                    raise VRControlError(f"Duplicate URDF arm joint: {name}")
                element = joint.find("limit")
                if element is None:
                    raise VRControlError(f"Missing URDF limit: {name}")
                limits[name] = tuple(float(element.attrib[key]) for key in ("lower", "upper", "velocity"))
        return validate_joint_limits(tuple(limits[name] for name in ARM_NAMES))
    except (ET.ParseError, KeyError, TypeError, ValueError) as exc:
        raise VRControlError(f"Invalid or incomplete dual-arm URDF: {exc}") from exc


def validate_ready_status(status):
    """Require an existing healthy READY hold; no body alignment is requested."""
    if not isinstance(status, dict):
        raise VRControlError("Bridge status must be an object")
    required = {"success": True, "protocol_version": 3, "bridge_implementation": "ial_g1d",
                "control_mode": "dual_arm", "lifecycle": "ready", "ready_hold_active": True,
                "policy_active": False, "pause_policy_supported": True}
    for key, expected in required.items():
        value = status.get(key)
        if value != expected or type(value) is not type(expected):
            raise VRControlError(f"READY status requires {key}={expected!r}")
    for key in ("last_error", "hold_error", "active_sequence"):
        if key not in status or status[key] is not None:
            raise VRControlError(f"READY status has missing/nonempty {key}")
    if status.get("column_position_known") is False or status.get("_column_position_known") is False:
        raise VRControlError("Column position is unknown")
    standard=(status.get("teleop_home_mode") is False and status.get("column_lowered") is True)
    measured_hold=(status.get("teleop_home_mode") is True and status.get("column_lowered") is False
                   and status.get("teleop_home_mode_supported") is True)
    if not (standard or measured_hold):
        raise VRControlError("Expected completed standard startup or explicit no-motion measured-pose startup")
    if not {"upper_body", "robot_state"}.issubset(set(status.get("capabilities", []))):
        raise VRControlError("Missing upper_body/robot_state capability")
    alignment = status.get("body_alignment", {})
    if (not isinstance(alignment, dict) or alignment.get("failure_latched")
            or alignment.get("error") or 13 not in status.get("invalid_motor_slots", [])):
        raise VRControlError("Body alignment is faulted or G1D slot13 protection is missing")
    backend = status.get("bridge", {})
    activation = backend.get("activation", {}) if isinstance(backend, dict) else {}
    if (not isinstance(backend, dict) or not isinstance(activation, dict)
            or backend.get("control_activated") is not True or backend.get("dry_run") is not False
            or backend.get("shutdown_requested") or backend.get("shutdown_error")
            or activation.get("upper_body_active") is not True or activation.get("error")):
        raise VRControlError("Bridge backend is inactive, faulted or shutting down")
    body = status.get("body_hold_target_rad")
    if body is None:
        return None
    if not isinstance(body, dict) or set(body) != {"waist_yaw", "waist_pitch"}:
        raise VRControlError("Exact yaw12/pitch14 hold targets are required")
    yaw, pitch = (_number(body[key], key) for key in ("waist_yaw", "waist_pitch"))
    return {"waist_yaw": yaw, "waist_pitch": pitch}


@dataclass(frozen=True)
class DualArmState:
    q: tuple
    received_monotonic_ns: int
    mode_machine: int


@dataclass(frozen=True)
class XRInput:
    sequence: int
    received_monotonic_ns: int
    left_translation: tuple
    right_translation: tuple
    left_rotvec: tuple
    right_rotvec: tuple
    left_squeeze: bool
    right_squeeze: bool
    # Physical Dex1 raw target coordinates, NOT raw or wrapper trigger values.
    desired_grippers: tuple
    session_enabled: bool = False


@dataclass(frozen=True)
class DualVRDecision:
    phase: str
    pause_required: bool
    reason: str | None = None
    body_q: tuple | None = None
    arm_q: tuple | None = None
    arm_velocity: tuple | None = None
    grippers: tuple | None = None
    contact_evidence: tuple | None = None
    mode_machine: int | None = None
    first_target: bool = False


class DualVRControl:
    """One numerical episode; construction and step never communicate.

    The runtime must use a single thread, establish stationary READY, bind the
    robot boot/source/lifecycle evidence, and independently validate the swept
    path. It must stop sending on VRControlError or pause_required. Creating a
    fresh object is NOT authorization to restart after a fault or release.
    """
    def __init__(self, ready_status, state: DualArmState, joint_limits, *, now_ns,
                 activation_mode="hold",velocity_limit=MAX_VELOCITY,
                 acceleration_limit=MAX_ACCELERATION,tracking_error_limit=MAX_TRACKING_ERROR):
        if activation_mode not in {"hold", "latched"}:
            raise VRControlError("Activation mode must be hold or latched")
        self.activation_mode = activation_mode
        self.velocity_limit=_number(velocity_limit,"velocity limit")
        self.acceleration_limit=_number(acceleration_limit,"acceleration limit")
        self.tracking_error_limit=_number(tracking_error_limit,"tracking error limit")
        if not 0 < self.tracking_error_limit <= .10:
            raise VRControlError("Tracking error limit must be within (0,0.10] rad")
        if not 0 < self.velocity_limit <= .20 or not 0 < self.acceleration_limit <= .25:
            raise VRControlError("VR speed/acceleration exceeds bounded 0.20/0.25 limits")
        self.phase, self.reason = "armed", None
        self.limits = validate_joint_limits(joint_limits)
        body = validate_ready_status(ready_status)
        q = self._state(state, now_ns)
        self._check_arm(tuple(q[index] for index in ARM_MOTORS))
        body = body or {"waist_yaw": q[12], "waist_pitch": q[14]}
        if abs(q[12] - body["waist_yaw"]) > MAX_BODY_DRIFT or abs(q[14] - body["waist_pitch"]) > MAX_BODY_DRIFT:
            raise VRControlError("Measured torso differs from READY hold")
        baseline = list(q)
        baseline[12], baseline[14] = body["waist_yaw"], body["waist_pitch"]
        self.body_q = tuple(baseline)
        self.mode_machine = state.mode_machine
        self.command_q = None
        self.velocity = (0.,) * 14
        self.last_command_ns = None
        self._last_state = (state.received_monotonic_ns, q)
        self._last_xr = None
        self._raw_samples = [None, None]
        self._grippers = (Dex1ContactController(), Dex1ContactController())

    @staticmethod
    def _state(state, now_ns):
        if not isinstance(state, DualArmState):
            raise VRControlError("Expected DualArmState")
        _stamp_age(state.received_monotonic_ns, now_ns, MAX_STATE_AGE_NS, "robot state")
        if type(state.mode_machine) is not int or not 0 <= state.mode_machine <= 255:
            raise VRControlError("Invalid robot mode_machine")
        return _body_q(state.q)

    def _check_arm(self, q):
        q = _vector(q, 14, "arm q")
        if any(value < lo + JOINT_MARGIN or value > hi - JOINT_MARGIN
               for value, (lo, hi, _) in zip(q, self.limits)):
            raise VRControlError("Arm target violates URDF limits or 0.05 rad margin")
        return q

    def _xr(self, xr, now_ns):
        if not isinstance(xr, XRInput) or type(xr.sequence) is not int or xr.sequence <= 0:
            raise VRControlError("Invalid atomic XR frame/sequence")
        _stamp_age(xr.received_monotonic_ns, now_ns, MAX_XR_AGE_NS, "XR input")
        components = []
        for name, maximum in (("left_translation", .10), ("right_translation", .10),
                              ("left_rotvec", .35), ("right_rotvec", .35)):
            value = _vector(getattr(xr, name), 3, name)
            if math.sqrt(sum(coordinate ** 2 for coordinate in value)) > maximum:
                raise VRControlError(f"XR {name} exceeds fixed-reference range {maximum}")
            components.append(value)
        if type(xr.left_squeeze) is not bool or type(xr.right_squeeze) is not bool:
            raise VRControlError("Both squeeze inputs must be explicit booleans")
        desired = _vector(xr.desired_grippers, 2, "physical Dex1 targets")
        if any(value < .28 or value > 5.38 for value in desired):
            raise VRControlError("Physical Dex1 targets must be in [0.28,5.38]")
        signature = (xr.sequence, xr.received_monotonic_ns, *components,
                     xr.left_squeeze, xr.right_squeeze, desired)
        previous = self._last_xr
        if previous is not None:
            if signature[0] < previous[0] or signature[1] < previous[1]:
                raise VRControlError("XR sequence/timestamp moved backwards")
            if signature[0] == previous[0]:
                if signature != previous:
                    raise VRControlError("Duplicate XR sequence has conflicting data")
            elif signature[1] <= previous[1]:
                raise VRControlError("New XR sequence requires a new original timestamp")
        self._last_xr = signature
        return desired

    def _raw_feedback(self, raw, side, now_ns):
        topic = f"rt/dex1/{'left' if side == 0 else 'right'}/state"
        if (getattr(raw, "topic", None) != topic
                or getattr(raw, "timestamp_kind", None) != RAW_TIMESTAMP_KIND):
            raise VRControlError(f"{topic} requires original DDS callback feedback")
        sequence, stamp = getattr(raw, "sequence", None), getattr(raw, "received_monotonic_ns", None)
        if type(sequence) is not int or sequence <= 0:
            raise VRControlError("Invalid raw DDS feedback sequence")
        _stamp_age(stamp, now_ns, getattr(self, 'raw_feedback_max_age_ns', 100_000_000), "raw Dex1 feedback")
        q, dq, tau = (_number(getattr(raw, field, None), f"raw Dex1 {field}") for field in ("q", "dq", "tau_est"))
        if type(getattr(raw, "mode", None)) is not int or not 0 <= raw.mode <= 255:
            raise VRControlError("Invalid raw Dex1 mode")
        if type(getattr(raw, "lost", None)) is not int or raw.lost != 0:
            raise VRControlError("Raw Dex1 lost must be integer zero")
        lower_tolerance = _number(getattr(self, 'raw_feedback_lower_tolerance', 0.),
                                  'raw Dex1 lower feedback tolerance')
        if not 0 <= lower_tolerance <= .1:
            raise VRControlError('Raw Dex1 lower feedback tolerance must be within [0,0.1]')
        lower, upper = round(.28-lower_tolerance, 12), 5.48
        if not lower <= q <= upper:
            raise VRControlError(f'{topic} feedback q={q:.9g} outside accepted [{lower:g},{upper:g}]')
        signature = (sequence, stamp, q, dq, tau, raw.mode, raw.lost)
        previous = self._raw_samples[side]
        if previous is not None:
            if sequence < previous[0] or stamp < previous[1]:
                raise VRControlError("Raw DDS feedback moved backwards")
            if sequence == previous[0]:
                if signature != previous:
                    raise VRControlError("Duplicate raw DDS sequence has conflicting data")
            elif stamp <= previous[1]:
                raise VRControlError("New raw DDS sequence requires a new original timestamp")
        self._raw_samples[side] = signature
        return q, stamp

    def step(self, state, xr, ik_target_q14, left_raw_feedback, right_raw_feedback, *, now_ns):
        if self.phase == "ended":
            return DualVRDecision("ended", True, self.reason)
        if self.phase == "faulted":
            raise VRControlError(f"Episode fault is latched: {self.reason}")
        try:
            # Release is acted on before stale-state or IK work; no target can
            # escape this branch. The input still must identify boolean gates.
            if isinstance(xr, XRInput) and self.activation_mode == "latched" and type(xr.session_enabled) is not bool:
                raise VRControlError("Latched session enable must be boolean")
            if isinstance(xr, XRInput) and (
                    (self.activation_mode == "hold" and (xr.left_squeeze is False or xr.right_squeeze is False))
                    or (self.activation_mode == "latched" and xr.session_enabled is False)):
                self.phase, self.reason = "ended", "dual_squeeze_released"
                return DualVRDecision("ended", True, self.reason)
            q = self._state(state, now_ns)
            if state.mode_machine != self.mode_machine:
                raise VRControlError("Robot mode_machine changed during the episode")
            if (state.received_monotonic_ns < self._last_state[0]
                    or (state.received_monotonic_ns == self._last_state[0] and q != self._last_state[1])):
                raise VRControlError("Robot state timestamp is backwards or conflicting")
            if abs(q[12] - self.body_q[12]) > MAX_BODY_DRIFT or abs(q[14] - self.body_q[14]) > MAX_BODY_DRIFT:
                raise VRControlError("Fixed yaw12/pitch14 drift exceeds 0.01 rad")
            measured = self._check_arm(tuple(q[index] for index in ARM_MOTORS))
            desired = self._xr(xr, now_ns)
            target = self._check_arm(ik_target_q14)
            first = self.command_q is None
            if first:
                command, velocity = measured, (0.,) * 14
            else:
                gap = now_ns - self.last_command_ns
                if gap <= 0 or gap > MAX_COMMAND_GAP_NS:
                    raise VRControlError("Command interval must be positive and at most 0.15 s")
                errors=tuple(abs(a-b) for a,b in zip(measured,self.command_q))
                worst=max(range(14),key=errors.__getitem__)
                if errors[worst] > self.tracking_error_limit:
                    raise VRControlError(f"Tracking error exceeds {self.tracking_error_limit:g} rad: motor={15+worst}, measured={measured[worst]:.6f}, previous_command={self.command_q[worst]:.6f}, gap_s={gap/1e9:.6f}")
                dt = gap / 1e9
                velocities = []
                for goal, old_q, old_v, actual, (_, _, urdf_v) in zip(target,self.command_q,self.velocity,measured,self.limits):
                    vmax = min(self.velocity_limit, urdf_v)
                    error = goal - old_q
                    # Leave a discrete braking step before reaching the goal.
                    # On a moving/reversing target we may brake through its
                    # location; never snap q or velocity to the new target.
                    acceleration=self.acceleration_limit
                    braking = (math.sqrt((acceleration * dt) ** 2 + 8 * acceleration * abs(error))
                               - acceleration * dt) / 2
                    wanted = math.copysign(min(vmax, abs(error) / dt, braking), error)
                    # Feedback governs progress before the hard tracking stop.
                    # Reserve braking room inside a 0.04-rad soft lead; the
                    # 0.08-rad rejection remains unchanged. Never clip position
                    # or instantly change velocity to hide a feedback problem.
                    lead=old_q-actual
                    if wanted*lead>0:
                        remaining=max(0.,.04-abs(lead))
                        lead_speed=max(0.,math.sqrt((acceleration*dt)**2+2*acceleration*remaining)-acceleration*dt)
                        wanted=math.copysign(min(abs(wanted),lead_speed),wanted)
                    v = max(old_v - acceleration * dt, min(old_v + acceleration * dt, wanted))
                    velocities.append(max(-vmax, min(vmax, v)))
                velocity = tuple(velocities)
                command = tuple(old_q + v * dt for old_q, v in zip(self.command_q, velocity))
                # Reject infeasible limit/reversal combinations. A last-stage
                # position clip would silently break the acceleration bound.
                self._check_arm(command)
                errors=tuple(abs(a-b) for a,b in zip(measured,command))
                worst=max(range(14),key=errors.__getitem__)
                if errors[worst] > self.tracking_error_limit:
                    raise VRControlError(f"Next target tracking error exceeds {self.tracking_error_limit:g} rad: motor={15+worst}, measured={measured[worst]:.6f}, proposed_command={command[worst]:.6f}, previous_command={self.command_q[worst]:.6f}, dt_s={dt:.6f}")
            outputs = []
            for side, raw in enumerate((left_raw_feedback, right_raw_feedback)):
                feedback, stamp = self._raw_feedback(raw, side, now_ns)
                request = max(.28, min(5.38, feedback)) if first else desired[side]
                outputs.append(self._grippers[side].update(request, feedback,
                    feedback_stamp_ns=stamp, now_ns=now_ns, lost=raw.lost))
            full = list(self.body_q)
            full[15:29] = command
            self.command_q, self.velocity = command, velocity
            self.last_command_ns = now_ns
            self._last_state = (state.received_monotonic_ns, q)
            self.phase = "active"
            return DualVRDecision("active", False, body_q=tuple(full), arm_q=command,
                arm_velocity=velocity, grippers=tuple(output.target for output in outputs),
                contact_evidence=tuple(output.evidence for output in outputs),
                mode_machine=state.mode_machine, first_target=first)
        except Exception as exc:
            self.phase, self.reason = "faulted", f"{type(exc).__name__}: {exc}"
            raise VRControlError(self.reason) from exc


def build_lowcmd(decision, *, make_lowcmd):
    """Call the public protocol builder, then preserve READY gains/body targets.

    Dependency injection prevents importing hardware packages. Returned torque
    is zero: the existing bridge alone supplies dual-arm gravity compensation.
    """
    if (not isinstance(decision, DualVRDecision) or decision.phase != "active"
            or decision.pause_required or decision.body_q is None):
        raise VRControlError("Only an active accepted decision can produce a payload")
    q = _body_q(decision.body_q)
    command = deepcopy(make_lowcmd(list(q), decision.mode_machine))
    motors = command.get("motor_cmd") if isinstance(command, dict) else None
    if not isinstance(motors, list) or len(motors) != 29 or any(not isinstance(item, dict) for item in motors):
        raise VRControlError("Public make_lowcmd did not return 29 motor dictionaries")
    weak = {4, 10, 15, 16, 17, 18, 22, 23, 24, 25}
    wrists = {19, 20, 21, 26, 27, 28}
    for index, item in enumerate(motors):
        kp, kd = (40., 1.5) if index in wrists else ((80., 3.) if index in weak else (300., 3.))
        item.update(mode=1, q=q[index], dq=0., tau=0., kp=kp, kd=kd)
    motors[13].update(mode=0, q=0., dq=0., tau=0., kp=0., kd=0.)
    command.update(mode_pr=0, mode_machine=decision.mode_machine)
    return command


def build_payloads(decision, *, make_lowcmd, make_gripper_command):
    """Return left grip, right grip, body payloads; no envelope or transmission."""
    body = build_lowcmd(decision, make_lowcmd=make_lowcmd)
    grippers = _vector(decision.grippers, 2, "protected Dex1 targets")
    if any(value < .28 or value > 5.38 for value in grippers):
        raise VRControlError("Protected Dex1 target is outside its command range")
    return (("rt/dex1/left/cmd", make_gripper_command(grippers[0])),
            ("rt/dex1/right/cmd", make_gripper_command(grippers[1])),
            ("rt/lowcmd", body))


@dataclass(frozen=True)
class ReturnSample:
    q: tuple
    velocity: tuple
    acceleration: tuple


@dataclass(frozen=True)
class SavedPoseReturn:
    """One fixed return trajectory, independent of XR and episode state.

    Construct only after a normal end and a fresh, stationary measured pose.
    This type does not authorize a return after lost input/fault, start any
    lifecycle, move the column, or send commands. Runtime guards remain needed
    throughout the return. An abnormal interruption must retain hold.
    """
    start_q: tuple
    saved_q: tuple
    duration_s: float

    def sample(self, elapsed_s):
        elapsed = _number(elapsed_s, "return elapsed time")
        if elapsed < 0:
            raise VRControlError("Return elapsed time cannot be negative")
        u = min(1., elapsed / self.duration_s)
        blend = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
        d_blend = (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4) / self.duration_s
        dd_blend = (60 * u - 180 * u ** 2 + 120 * u ** 3) / self.duration_s ** 2
        if u == 1.:
            return ReturnSample(self.saved_q, (0.,) * 14, (0.,) * 14)
        delta = tuple(target - start for start, target in zip(self.start_q, self.saved_q))
        return ReturnSample(tuple(start + blend * d for start, d in zip(self.start_q, delta)),
                            tuple(d_blend * d for d in delta), tuple(dd_blend * d for d in delta))


def build_return_trajectory(current_q14,saved_q14,joint_limits,*,velocity_limit=MAX_VELOCITY,
                            acceleration_limit=MAX_ACCELERATION,minimum_duration_s=4.,
                            start_joint_margin=JOINT_MARGIN):
    """Freeze a zero-endpoint-velocity/acceleration quintic return to saved q.

    Exact maxima: speed=1.875*abs(delta)/T; acceleration=(10/sqrt(3))*abs(delta)/T².
    Body/column/grippers are deliberately outside this 14D numerical interface.
    """
    limits = validate_joint_limits(joint_limits)
    velocity_limit=_number(velocity_limit,"return velocity limit")
    acceleration_limit=_number(acceleration_limit,"return acceleration limit")
    minimum_duration_s=_number(minimum_duration_s,"return minimum duration")
    start_joint_margin=_number(start_joint_margin,"return start joint margin")
    if not 0 <= start_joint_margin <= JOINT_MARGIN:
        raise VRControlError("Return start joint margin is invalid")
    if not 0 < velocity_limit <= .20 or not 0 < acceleration_limit <= .25 or minimum_duration_s<.5:
        raise VRControlError("Return trajectory bounds are invalid")
    start = _vector(current_q14, 14, "return start q")
    saved = _vector(saved_q14, 14, "saved original q")
    # A measured manual pose can lie inside the hard bounds but within the
    # planning margin. A site return may start there and move monotonically
    # inward; its saved endpoint still requires the full planning margin.
    for values,margin in ((start,start_joint_margin),(saved,JOINT_MARGIN)):
        if any(value < lo + margin or value > hi - margin
               for value, (lo, hi, _) in zip(values, limits)):
            raise VRControlError("Return endpoint violates URDF limits or margin")
    duration = minimum_duration_s
    for a, b, (_, _, urdf_speed) in zip(start, saved, limits):
        delta = abs(b - a)
        duration = max(duration,1.875*delta/min(velocity_limit,urdf_speed),
                       math.sqrt((10/math.sqrt(3))*delta/acceleration_limit))
    return SavedPoseReturn(start, saved, duration)
