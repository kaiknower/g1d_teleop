"""Pure teleoperation profile math; no SDK, network, motion or lifecycle side effects.

Clips target lead relative to measured joint positions. A 20..30 parameter
at 250 Hz means .08..12 rad target lead. The runtime owns the two-second
takeover blend, filtered IK, lifecycle ownership and bounded normal return.
"""
from g1d_vr_teleop.core.dual_arm import (
    ARM_MOTORS, MAX_BODY_DRIFT, MAX_XR_AGE_NS,
    DualVRControl, DualVRDecision, XRInput, VRControlError,
    _number, _stamp_age, _vector,
)
from g1d_vr_teleop.control.grippers import ManualDex1Controller
from g1d_vr_teleop.core.contact import ContactOutput


class SiteDualVRControl(DualVRControl):
    site_profile = True
    raw_feedback_max_age_ns = 500_000_000
    raw_feedback_lower_tolerance = .1
    profile_metadata = {
        'site_profile': True,
        'name': 'measured_position_lead',
        'reference_control_hz': 250.,
        'reference_driver_parameter_start': 20.,
        'reference_driver_parameter_end': 30.,
        'lead_start_rad': .08,
        'lead_end_rad': .12,
        'lead_ramp_seconds': 5.,
        'arm_velocity_semantics': 'finite_difference_of_targets_not_physical_velocity',
        'normal_return_velocity_limit_rad_s': .10,
        'normal_return_acceleration_limit_rad_s2': .25,
        'gripper_mode': 'manual_endpoint_hold',
        'gripper_closing_speed_raw_s': 6.4,
        'gripper_fresh_update_max_age_s': .1,
        'gripper_outage_hold_timeout_s': .5,
        'gripper_feedback_position_range': [.18, 5.48],
        'gripper_command_position_range': [.28, 5.38],
    }

    def __init__(self, ready_status, state, joint_limits, *, now_ns,
                 activation_mode='latched'):
        # These properties are retained ONLY for bounded normal-return helpers;
        # the teleop step below uses the original measured-lead algorithm.
        super().__init__(ready_status, state, joint_limits, now_ns=now_ns,
                         activation_mode=activation_mode, velocity_limit=.10,
                         acceleration_limit=.25, tracking_error_limit=.10)
        # This site sends measured-position targets, not the generic integrated
        # trajectory. Honor the existing bridge watchdog without accepting old
        # XR, robot, or gripper observations as new data after a scheduling gap.
        self.command_timeout_s = _number(
            ready_status['bridge'].get('command_timeout_s', .5),
            'bridge command timeout')
        if not 0 < self.command_timeout_s <= .5:
            raise VRControlError('Bridge command timeout must be within (0,0.5] seconds')
        self.max_command_gap_ns = round(self.command_timeout_s*1e9)
        if self.max_command_gap_ns <= 0:
            raise VRControlError('Bridge command timeout must be at least one nanosecond')
        self.profile_metadata = {**type(self).profile_metadata,
                                 'command_timeout_s': self.command_timeout_s}
        self._first_command_ns = None
        self._grippers = (ManualDex1Controller(),ManualDex1Controller())
        self._last_grip_outputs = [None, None]
        self.current_lead_limit_rad = .08

    def lead_limit_rad_at(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            raise VRControlError('Lead profile timestamp must be positive integer nanoseconds')
        if self._first_command_ns is None:
            return .08
        if now_ns < self._first_command_ns:
            raise VRControlError('Lead profile timestamp moved backwards')
        elapsed_s = (now_ns-self._first_command_ns)/1e9
        return (20. + 10.*min(1., elapsed_s/5.))/250.

    def _check_arm(self, q):
        q = _vector(q, 14, 'arm q')
        if any(value < lo or value > hi for value, (lo, hi, _) in zip(q, self.limits)):
            raise VRControlError('Arm target violates URDF joint limits')
        return q

    def _xr(self, xr, now_ns):
        if not isinstance(xr, XRInput) or type(xr.sequence) is not int or xr.sequence <= 0:
            raise VRControlError('Invalid atomic XR frame/sequence')
        _stamp_age(xr.received_monotonic_ns, now_ns, MAX_XR_AGE_NS, 'XR input')
        components = tuple(_vector(getattr(xr, name), 3, name) for name in
            ('left_translation', 'right_translation', 'left_rotvec', 'right_rotvec'))
        if type(xr.left_squeeze) is not bool or type(xr.right_squeeze) is not bool:
            raise VRControlError('Both squeeze inputs must be explicit booleans')
        if type(xr.session_enabled) is not bool:
            raise VRControlError('Session enable must be boolean')
        desired = _vector(xr.desired_grippers, 2, 'physical Dex1 targets')
        if any(value < .28 or value > 5.38 for value in desired):
            raise VRControlError('Physical Dex1 targets must be in [0.28,5.38]')
        signature = (xr.sequence, xr.received_monotonic_ns, *components,
                     xr.left_squeeze, xr.right_squeeze, desired, xr.session_enabled)
        previous = self._last_xr
        if previous is not None:
            if signature[0] < previous[0] or signature[1] < previous[1]:
                raise VRControlError('XR sequence/timestamp moved backwards')
            if signature[0] == previous[0]:
                if signature != previous:
                    raise VRControlError('Duplicate XR sequence has conflicting data')
            elif signature[1] <= previous[1]:
                raise VRControlError('New XR sequence requires a new original timestamp')
        self._last_xr = signature
        return desired

    def step(self, state, xr, ik_target_q14, left_raw_feedback, right_raw_feedback, *, now_ns):
        if self.phase == 'ended':
            return DualVRDecision('ended', True, self.reason)
        if self.phase == 'faulted':
            raise VRControlError(f'Episode fault is latched: {self.reason}')
        try:
            if isinstance(xr, XRInput):
                if type(xr.session_enabled) is not bool:
                    raise VRControlError('Session enable must be boolean')
                if ((self.activation_mode == 'latched' and xr.session_enabled is False)
                    or (self.activation_mode == 'hold' and
                        (xr.left_squeeze is False or xr.right_squeeze is False))):
                    self.phase, self.reason = 'ended', 'operator_session_disabled'
                    return DualVRDecision('ended', True, self.reason)
            q = self._state(state, now_ns)
            if state.mode_machine != self.mode_machine:
                raise VRControlError('Robot mode_machine changed during the episode')
            if (state.received_monotonic_ns < self._last_state[0] or
                (state.received_monotonic_ns == self._last_state[0] and q != self._last_state[1])):
                raise VRControlError('Robot state timestamp is backwards or conflicting')
            if abs(q[12]-self.body_q[12]) > MAX_BODY_DRIFT or abs(q[14]-self.body_q[14]) > MAX_BODY_DRIFT:
                raise VRControlError('Fixed yaw12/pitch14 drift exceeds 0.01 rad')
            measured = self._check_arm(tuple(q[index] for index in ARM_MOTORS))
            desired = self._xr(xr, now_ns)
            target = self._check_arm(ik_target_q14)
            first = self.command_q is None
            if first:
                command, velocity = measured, (0.,)*14
            else:
                gap = now_ns-self.last_command_ns
                if gap <= 0 or gap > self.max_command_gap_ns:
                    raise VRControlError(f'Site command interval invalid: gap_s={gap/1e9:.6f}, '
                                         f'limit_s={self.command_timeout_s:g}')
                self.current_lead_limit_rad = self.lead_limit_rad_at(now_ns)
                delta = tuple(goal-actual for goal, actual in zip(target, measured))
                # One global scale preserves the original 14-joint direction.
                motion_scale = max(abs(value) for value in delta)/self.current_lead_limit_rad
                command = tuple(actual+value/max(motion_scale, 1.)
                                for actual, value in zip(measured, delta))
                self._check_arm(command)
                velocity = tuple((new-old)/(gap/1e9) for new, old in zip(command, self.command_q))
            outputs = []
            for side, raw in enumerate((left_raw_feedback, right_raw_feedback)):
                feedback, stamp = self._raw_feedback(raw, side, now_ns)
                gripper = self._grippers[side]
                if not gripper.feedback_min_position <= feedback <= gripper.feedback_max_position:
                    raise VRControlError(f'Raw Dex1 side={side} feedback q={feedback:.9g} outside accepted [{gripper.feedback_min_position:g},{gripper.feedback_max_position:g}]')
                age_ns = now_ns-stamp
                if age_ns > 100_000_000:
                    prior = self._last_grip_outputs[side]
                    if first or prior is None:
                        raise VRControlError('First Dex1 command requires feedback within 0.1 seconds')
                    # A transient receive gap holds the exact last target. It
                    # cannot advance compression or contribute contact evidence.
                    # Never rewrite the original sample timestamp or run the
                    # manual controller using a pretend fresh observation.
                    outputs.append(ContactOutput(prior.target, 'feedback_wait', False, False,
                        {'phase': 'feedback_wait', 'feedback_stamp_ns': stamp,
                         'now_ns': now_ns, 'feedback_age_s': age_ns/1e9,
                         'feedback_q': feedback, 'requested_q': desired[side],
                         'target_q': prior.target, 'accepted_new_feedback': False,
                         'closing_advance_raw': 0., 'contact_candidate': False,
                         'grasp_success': False, 'lift_allowed': False,
                         'manual_control': True, 'outage_timeout_s': .5,
                         'note': 'Hold previous target while waiting for fresh original DDS feedback'}))
                    continue
                request = max(.28, min(5.38, feedback)) if first else desired[side]
                outputs.append(gripper.update(request, feedback,
                    feedback_stamp_ns=stamp, now_ns=now_ns, lost=raw.lost))
            full = list(self.body_q)
            full[15:29] = command
            self.command_q, self.velocity = command, velocity
            self._last_grip_outputs = outputs
            self.last_command_ns = now_ns
            if first:
                self._first_command_ns = now_ns
            self._last_state = (state.received_monotonic_ns, q)
            self.phase = 'active'
            return DualVRDecision('active', False, body_q=tuple(full), arm_q=command,
                arm_velocity=velocity, grippers=tuple(output.target for output in outputs),
                contact_evidence=tuple(output.evidence for output in outputs),
                mode_machine=state.mode_machine, first_target=first)
        except Exception as exc:
            self.phase, self.reason = 'faulted', f'{type(exc).__name__}: {exc}'
            raise VRControlError(self.reason) from exc
