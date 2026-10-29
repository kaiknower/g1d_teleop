"""Pure numerical reuse of the site's Dex1ContactController.

No SDK, DDS, sockets, clock reads, robot commands, or automatic lifting. Caller
supplies the original DDS feedback callback timestamp and current timestamp in
the SAME monotonic domain. A relay/poll timestamp is not a new observation.

Original lead/stall logic is preserved. The 0.8 raw-unit/s bound applies only to
closing compression advance. Contact releases the old .20 preload to .04
immediately, matching the old controller, without a bidirectional speed claim.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from numbers import Real


class Dex1ContactError(RuntimeError):
    """Latched rejection: adapter must stop closure, not retry or lift."""
    def __init__(self, reason, evidence):
        super().__init__(reason)
        self.reason, self.evidence = reason, evidence


@dataclass(frozen=True)
class ContactOutput:
    target: float
    phase: str
    accepted_new_feedback: bool
    contact_candidate: bool
    evidence: dict
    grasp_success: bool = False
    lift_allowed: bool = False


@dataclass
class Dex1ContactController:
    # Parameters from the existing site Dex1ContactController.
    min_position: float = .28
    max_position: float = 5.38
    close_lead: float = .20
    open_lead: float = .18
    hold_lead: float = .04
    contact_window_s: float = .25
    contact_motion_epsilon: float = .01
    response_error: float = .06
    open_fault_window_s: float = .40
    direction_epsilon: float = .02
    closing_speed_raw_s: float = .8
    feedback_max_age_s: float = .1
    feedback_upper_tolerance: float = .1
    minimum_contact_position: float = .45
    mode: str = field(default="tracking", init=False)
    _samples: deque = field(default_factory=deque, init=False, repr=False)
    _direction: str | None = field(default=None, init=False, repr=False)
    _last_stamp_ns: int | None = field(default=None, init=False, repr=False)
    _last_feedback: float | None = field(default=None, init=False, repr=False)
    _last_output: ContactOutput | None = field(default=None, init=False, repr=False)
    _closing_started_ns: int | None = field(default=None, init=False, repr=False)
    _fault_evidence: dict | None = field(default=None, init=False, repr=False)

    def __post_init__(self):
        for name in ("min_position", "max_position", "close_lead", "open_lead", "hold_lead",
                     "contact_window_s", "contact_motion_epsilon", "response_error",
                     "open_fault_window_s", "direction_epsilon", "closing_speed_raw_s",
                     "feedback_max_age_s", "feedback_upper_tolerance", "minimum_contact_position"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not self.min_position < self.minimum_contact_position < self.max_position:
            raise ValueError("Contact threshold must lie within the command range")
        if self.hold_lead > self.close_lead:
            raise ValueError("hold_lead must not exceed close_lead")
        if (self.min_position < .28 or self.max_position > 5.38 or self.closing_speed_raw_s > .8
                or self.feedback_max_age_s > .1 or self.feedback_upper_tolerance > .1
                or self.minimum_contact_position < .45):
            raise ValueError("Command, compression-speed, freshness or contact limits cannot be relaxed")

    @property
    def close_timeout_s(self):
        # Existing grasp-script budget: full close travel plus five seconds.
        return max(8., (self.max_position-self.min_position)/self.closing_speed_raw_s + 5.)

    def reset(self):
        """Explicit offline episode reset, never performed automatically."""
        self.mode = "tracking"
        self._samples.clear()
        self._direction = None
        self._last_stamp_ns = self._last_feedback = self._last_output = None
        self._closing_started_ns = self._fault_evidence = None

    def _fault(self, reason, evidence):
        self.mode = "fault"
        def json_value(value):
            if isinstance(value, dict):
                return {key: json_value(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [json_value(item) for item in value]
            if isinstance(value, Real):
                return value if math.isfinite(value) else repr(value)
            return value if value is None or isinstance(value, (str, bool)) else repr(value)
        self._fault_evidence = {**json_value(evidence), "reason": reason, "phase": "fault",
            "contact_candidate": False, "grasp_success": False, "lift_allowed": False}
        raise Dex1ContactError(reason, self._fault_evidence)

    def _clip(self, q):
        return min(self.max_position, max(self.min_position, float(q)))

    def _record_motion(self, direction, stamp_ns, feedback, window_s):
        if direction != self._direction:
            self._samples.clear()
            self._direction = direction
        self._samples.append((stamp_ns, feedback))
        cutoff = stamp_ns-round(window_s*1e9)
        while len(self._samples) >= 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        span = (self._samples[-1][0]-self._samples[0][0])/1e9
        positions = [q for _, q in self._samples]
        extent = max(positions)-min(positions)
        return {"direction": direction, "sample_count": len(self._samples),
            "first_feedback_stamp_ns": self._samples[0][0], "last_feedback_stamp_ns": stamp_ns,
            "span_s": span, "window_required_s": window_s, "position_range_raw": extent,
            "motion_epsilon_raw": self.contact_motion_epsilon,
            "stalled": bool(len(self._samples) >= 2 and span >= window_s and extent <= self.contact_motion_epsilon)}

    def update(self, desired_position, feedback, *, feedback_stamp_ns, now_ns, lost=0):
        """Consume one raw DDS observation; duplicates cannot advance state.

        First closing observation primes at clipped feedback. Invalid/stale/backwards
        feedback, lost != 0, timeout, unsuccessful opening, or a low-position
        stall latch fault and yield no new target. Contact is only a stall-based
        candidate, never grasp success or permission to lift.
        """
        if self.mode == "fault":
            raise Dex1ContactError("Controller is faulted; explicit reset required", self._fault_evidence or {})
        evidence = {"feedback_stamp_ns": feedback_stamp_ns, "now_ns": now_ns,
            "feedback_q": feedback, "requested_q": desired_position, "lost": lost,
            "grasp_success": False, "lift_allowed": False}
        if (type(feedback_stamp_ns) is not int or type(now_ns) is not int
                or feedback_stamp_ns <= 0 or now_ns < feedback_stamp_ns):
            self._fault("Invalid or future original feedback timestamp", evidence)
        for value in (desired_position, feedback, lost):
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
                self._fault("Nonfinite or nonnumeric target/feedback/lost", evidence)
        age_s = (now_ns-feedback_stamp_ns)/1e9
        evidence["feedback_age_s"] = age_s
        if age_s > self.feedback_max_age_s:
            self._fault("Feedback is older than 0.1 seconds", evidence)
        if lost != 0:
            self._fault("DDS feedback reports lost != 0", evidence)
        if not self.min_position <= feedback <= self.max_position+self.feedback_upper_tolerance:
            self._fault("Feedback outside accepted position range", evidence)
        if self._last_stamp_ns is not None and feedback_stamp_ns < self._last_stamp_ns:
            self._fault("Original DDS feedback timestamp moved backwards", evidence)
        if feedback_stamp_ns == self._last_stamp_ns:
            if feedback != self._last_feedback:
                self._fault("Same original feedback timestamp has conflicting position", evidence)
            prior = self._last_output
            return ContactOutput(prior.target, prior.phase, False, prior.contact_candidate,
                {**prior.evidence, **evidence, "accepted_new_feedback": False,
                 "duplicate_feedback": True, "closing_advance_raw": 0.,
                 "note": "Target, mode, timeout and stall samples unchanged; not a new observation"})

        desired, feedback = self._clip(desired_position), float(feedback)
        dt = 0. if self._last_stamp_ns is None else (feedback_stamp_ns-self._last_stamp_ns)/1e9
        # Unobserved intervals cannot be counted as a continuous stall.
        # Resumed compression uses at most one freshness interval.
        discontinuity = self._last_stamp_ns is not None and dt > self.feedback_max_age_s
        if discontinuity:
            self._samples.clear()
        max_advance = self.closing_speed_raw_s*min(dt, self.feedback_max_age_s)
        previous_target = self._clip(feedback) if self._last_output is None else self._last_output.target
        stall = {"stalled": False, "sample_count": 0, "span_s": 0.}
        releasing_contact_preload = False
        if desired < feedback-self.direction_epsilon:
            if self._closing_started_ns is None:
                self._closing_started_ns = feedback_stamp_ns
            elapsed = (feedback_stamp_ns-self._closing_started_ns)/1e9
            evidence["closing_elapsed_s"] = elapsed
            if self.mode != "contact" and elapsed >= self.close_timeout_s:
                self._fault("Closing timed out without contact", {**evidence, "timeout_s": self.close_timeout_s})
            stall = self._record_motion("closing", feedback_stamp_ns, feedback, self.contact_window_s)
            lead = self.hold_lead if self.mode == "contact" else self.close_lead
            target = max(desired, feedback-lead)
            if self.mode != "contact" and stall["stalled"] and feedback-desired >= self.response_error:
                if feedback <= self.minimum_contact_position:
                    self._fault("Stalled at or below minimum contact position; no successful grasp", {**evidence, "stall": stall})
                self.mode = "contact"
                target = max(desired, feedback-self.hold_lead)
                releasing_contact_preload = target > previous_target
            if self.mode == "contact" and feedback <= self.minimum_contact_position:
                self._fault("Contact feedback fell below required position", {**evidence, "stall": stall})
        elif desired > feedback+self.direction_epsilon:
            self._closing_started_ns = None
            if self.mode == "contact":
                self.mode = "tracking"
            target = min(desired, feedback+self.open_lead)
            stall = self._record_motion("opening", feedback_stamp_ns, feedback, self.open_fault_window_s)
            if stall["stalled"] and target-feedback >= self.response_error:
                self._fault("Dex1 did not respond to an opening command", {**evidence, "stall": stall})
        else:
            self.mode = "tracking"
            self._closing_started_ns = None
            self._samples.clear()
            self._direction = None
            target = desired
        # Opening and contact preload release keep the old immediate behavior.
        target = self._clip(max(target, previous_target-max_advance))
        advance = max(0., previous_target-target)
        evidence.update(desired_clipped_q=desired, target_q=target, phase=self.mode,
            accepted_new_feedback=True, duplicate_feedback=False,
            feedback_interval_s=dt, observation_gap_reset_stall=discontinuity,
            closing_advance_raw=advance, maximum_closing_advance_raw=max_advance,
            closing_speed_limit_raw_s=self.closing_speed_raw_s,
            contact_preload_release_exception=releasing_contact_preload,
            release_note="Opening and immediate .20-to-.04 preload release preserve old semantics; only compression advance has a speed bound",
            feedback_upper_tolerance_note="Feedback above commanded OPEN has been observed (5.392); tolerance does not identify its cause, enlarge command range, or calibrate opening",
            stall=stall, contact_candidate=self.mode == "contact",
            contact_condition_min_q_exclusive=self.minimum_contact_position)
        output = ContactOutput(target, self.mode, True, self.mode == "contact", evidence)
        self._last_stamp_ns, self._last_feedback, self._last_output = feedback_stamp_ns, feedback, output
        return output
