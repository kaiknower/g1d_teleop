"""Pure manual Dex1 targets; endpoint stalls are holds, never grasp results.

This isolated VR controller does not change the autonomous contact controller.
It consumes original feedback timestamps and creates no hardware connections.
"""
from collections import deque
import math
from numbers import Real

from g1d_vr_teleop.core.contact import ContactOutput, Dex1ContactError


class ManualDex1Controller:
    min_position = .28
    max_position = 5.38
    feedback_upper_tolerance = .10
    # Encoder readback may pass either command endpoint slightly. This is a
    # sensor acceptance envelope only; _clip keeps all outputs in [.28,5.38].
    feedback_min_position = .18
    feedback_max_position = 5.48
    closing_speed_raw_s = 6.4
    close_lead = .20
    open_lead = .18
    hold_lead = .04
    feedback_max_age_s = .10
    direction_epsilon = .02
    contact_window_s = .25
    open_hold_window_s = .40
    contact_motion_epsilon = .01
    response_error = .06
    # Only classifies diagnostics; never blocks a manual close or permits lift.
    endpoint_diagnostic_position = .45

    def __init__(self):
        self.mode = 'tracking'
        self._samples = deque()
        self._direction = None
        self._last_stamp_ns = self._last_feedback = self._last_output = None
        self._fault_evidence = None

    def _fault(self, reason, evidence):
        self.mode = 'fault'
        self._fault_evidence = {**evidence, 'reason': reason, 'phase': 'fault',
            'contact_candidate': False, 'grasp_success': False, 'lift_allowed': False}
        raise Dex1ContactError(reason, self._fault_evidence)

    def _clip(self, value):
        return max(self.min_position, min(self.max_position, float(value)))

    def _stall(self, direction, stamp, feedback, window):
        if direction != self._direction:
            self._samples.clear()
            self._direction = direction
        self._samples.append((stamp, feedback))
        cutoff = stamp - round(window * 1e9)
        while len(self._samples) >= 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        span = (stamp - self._samples[0][0]) / 1e9
        values = [q for _, q in self._samples]
        extent = max(values) - min(values)
        return dict(direction=direction, sample_count=len(values), span_s=span,
            position_range_raw=extent,
            stalled=len(values) >= 2 and span >= window and extent <= self.contact_motion_epsilon)

    def update(self, desired_position, feedback, *, feedback_stamp_ns, now_ns, lost=0):
        if self.mode == 'fault':
            raise Dex1ContactError('Manual controller is faulted; new episode required', self._fault_evidence)
        evidence = dict(feedback_stamp_ns=feedback_stamp_ns, now_ns=now_ns,
            feedback_q=feedback, requested_q=desired_position, lost=lost,
            grasp_success=False, lift_allowed=False, manual_control=True)
        if (type(feedback_stamp_ns) is not int or type(now_ns) is not int
                or feedback_stamp_ns <= 0 or now_ns < feedback_stamp_ns):
            self._fault('Invalid or future original feedback timestamp', evidence)
        if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v)
               for v in (desired_position, feedback)):
            self._fault('Nonfinite or nonnumeric target/feedback', evidence)
        if type(lost) is not int or lost != 0:
            self._fault('DDS feedback lost must be integer zero', evidence)
        age = (now_ns - feedback_stamp_ns) / 1e9
        if age > self.feedback_max_age_s:
            self._fault('Feedback is older than 0.1 seconds', evidence)
        if not self.feedback_min_position <= feedback <= self.feedback_max_position:
            self._fault(f'Feedback q={feedback:.9g} outside accepted [{self.feedback_min_position:g},{self.feedback_max_position:g}]', evidence)
        if self._last_stamp_ns is not None and feedback_stamp_ns < self._last_stamp_ns:
            self._fault('Original feedback timestamp moved backwards', evidence)
        if feedback_stamp_ns == self._last_stamp_ns:
            if feedback != self._last_feedback:
                self._fault('Same original timestamp has conflicting feedback', evidence)
            prior = self._last_output
            return ContactOutput(prior.target, prior.phase, False, prior.contact_candidate,
                {**prior.evidence, **evidence, 'accepted_new_feedback': False,
                 'duplicate_feedback': True, 'closing_advance_raw': 0.})

        desired, feedback = self._clip(desired_position), float(feedback)
        dt = 0. if self._last_stamp_ns is None else (feedback_stamp_ns - self._last_stamp_ns) / 1e9
        gap = self._last_stamp_ns is not None and dt > self.feedback_max_age_s
        if gap:
            self._samples.clear()  # An unobserved gap is not stall evidence.
        previous = self._clip(feedback) if self._last_output is None else self._last_output.target
        advance_limit = self.closing_speed_raw_s * min(dt, self.feedback_max_age_s)
        stall = dict(stalled=False, sample_count=0, span_s=0.)
        contact = False
        if desired < feedback - self.direction_epsilon:
            holding = self._direction == 'closing' and self.mode in {'contact', 'limit_hold'}
            stall = self._stall('closing', feedback_stamp_ns, feedback, self.contact_window_s)
            holding |= stall['stalled'] and feedback - desired >= self.response_error
            if holding:
                contact = feedback > self.endpoint_diagnostic_position
                self.mode = 'contact' if contact else 'limit_hold'
                target = max(desired, feedback - self.hold_lead)
            else:
                self.mode = 'tracking'
                target = max(desired, feedback - self.close_lead)
        elif desired > feedback + self.direction_epsilon:
            stall = self._stall('opening', feedback_stamp_ns, feedback, self.open_hold_window_s)
            # A manual end stop does not imply task failure. Keep a bounded
            # preload while the operator can release/reverse the trigger.
            stopped = stall['stalled'] and desired - feedback >= self.response_error
            self.mode = 'limit_hold' if stopped else 'tracking'
            target = min(desired, feedback + (self.hold_lead if stopped else self.open_lead))
        else:
            self._samples.clear()
            self._direction = None
            self.mode = 'limit_hold' if desired in (self.min_position, self.max_position) else 'tracking'
            target = desired
        # The first observation primes at feedback. Fresh observations alone
        # advance compression; opening and contact preload release are immediate.
        target = self._clip(max(target, previous - advance_limit))
        evidence.update(target_q=target, desired_clipped_q=desired, phase=self.mode,
            feedback_age_s=age, feedback_interval_s=dt,
            accepted_new_feedback=True, duplicate_feedback=False,
            closing_advance_raw=max(0., previous - target),
            maximum_closing_advance_raw=advance_limit,
            closing_speed_limit_raw_s=self.closing_speed_raw_s,
            observation_gap_reset_stall=gap, stall=stall, contact_candidate=contact,
            contact_note='Stall diagnostic only; no grasp success or lift authorization')
        output = ContactOutput(target, self.mode, True, contact, evidence)
        self._last_stamp_ns, self._last_feedback, self._last_output = feedback_stamp_ns, feedback, output
        return output
