"""Atomic, fail-closed receiver freshness for the audited TeleVuer API.

No Vuer/SDK import or server is started by importing this module. Construct
``make_fresh_televuer(TeleVuer)(...)`` only in the authorized application.
Consume ``fresh_snapshot()`` directly; reading the inherited individual shared
properties afterwards loses the atomicity guarantee.

Poses remain OpenXR column-major flat matrices; analog trigger/squeeze remain
0..1. This module deliberately does not repeat TeleVuerWrapper transforms.
Timestamps and sequence numbers describe actual server event receipts, not
device capture times (the upstream protocol supplies no capture timestamp).
Head and controllers are separate events with a bounded receipt-time skew.
The subclass factory targets the audited Linux/fork TeleVuer implementation.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
import multiprocessing as mp
from numbers import Real
import time


class FreshnessError(RuntimeError):
    """The caller must hold/disable motion; no usable frame was returned."""


@dataclass(frozen=True)
class ControllerState:
    trigger: bool
    trigger_value: float
    squeeze: bool
    squeeze_value: float
    thumbstick: bool
    thumbstick_value: tuple
    a_button: bool
    b_button: bool


@dataclass(frozen=True)
class RawXRFrame:
    head: tuple
    left: tuple
    right: tuple
    left_state: ControllerState
    right_state: ControllerState
    head_seq: int
    controller_seq: int
    head_monotonic_ns: int
    controller_monotonic_ns: int
    snapshot_monotonic_ns: int
    source_id: int
    right_a_press_count: int = 0
    b_press_count: int = 0
    pose_reference: str = "head_relative"

    @property
    def sequence(self):
        return (self.head_seq, self.controller_seq)

    @property
    def age_s(self):
        return (self.snapshot_monotonic_ns - min(
            self.head_monotonic_ns, self.controller_monotonic_ns)) / 1e9


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("expected a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("expected a finite number")
    return value


def _pose(value):
    """Validate the actual flat column-major Vuer SE(3) representation."""
    if isinstance(value, (str, bytes, Mapping)) or len(value) != 16:
        raise ValueError("expected 16 matrix elements")
    pose = tuple(_finite_number(x) for x in value)
    if any(abs(pose[i] - expected) > 1e-5
           for i, expected in ((3, 0), (7, 0), (11, 0), (15, 1))):
        raise ValueError("invalid homogeneous matrix row")
    columns = [pose[start:start + 3] for start in (0, 4, 8)]
    for i in range(3):
        for j in range(3):
            dot = sum(columns[i][k] * columns[j][k] for k in range(3))
            if abs(dot - float(i == j)) > 0.02:
                raise ValueError("rotation is not orthonormal")
    a, b, c = columns
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - b[0] * (a[1] * c[2] - a[2] * c[1])
                   + c[0] * (a[1] * b[2] - a[2] * b[1]))
    if abs(determinant - 1.0) > 0.02:
        raise ValueError("rotation is not proper")
    return pose


def _tracking_flags(value):
    # These flags are optional in the audited API; a present negative flag is
    # never overridden merely because the last pose matrix is still valid.
    for name in ("connected", "tracked", "valid"):
        if name in value and value[name] is not True:
            raise ValueError("tracking flag is false")


def _controller(value):
    if not isinstance(value, Mapping):
        raise ValueError("missing controller state")
    _tracking_flags(value)
    bools = {}
    for key in ("trigger", "squeeze", "thumbstick", "aButton", "bButton"):
        if type(value[key]) is not bool:
            raise ValueError("missing/non-boolean controller button")
        bools[key] = value[key]
    trigger = _finite_number(value["triggerValue"])
    squeeze = _finite_number(value["squeezeValue"])
    if not (0.0 <= trigger <= 1.0 and 0.0 <= squeeze <= 1.0):
        raise ValueError("controller analog value outside 0..1")
    axes = value["thumbstickValue"]
    if isinstance(axes, (str, bytes, Mapping)) or len(axes) != 2:
        raise ValueError("missing thumbstick axes")
    axes = tuple(_finite_number(x) for x in axes)
    if any(abs(x) > 1.0 for x in axes):
        raise ValueError("thumbstick axis outside -1..1")
    return (float(bools["trigger"]), trigger, float(bools["squeeze"]),
            squeeze, float(bools["thumbstick"]), axes[0], axes[1],
            float(bools["aButton"]), float(bools["bButton"]))


def _unpack_controller(values):
    return ControllerState(bool(values[0]), values[1], bool(values[2]),
                           values[3], bool(values[4]), tuple(values[5:7]),
                           bool(values[7]), bool(values[8]))


class FreshXRState:
    """Shared state allocated before TeleVuer forks its server process.

    ``record_*`` are receiver hooks, not control-loop refresh methods. Invalid
    events immediately invalidate their entire stream, including prior data.
    Snapshot lock acquisition is bounded so a dead writer cannot block control.
    ``clock_ns`` is injectable for deterministic offline tests only.
    """

    def __init__(self, *, context=None, clock_ns=time.monotonic_ns):
        context = context or mp.get_context()
        self._lock = context.RLock()
        self._data = context.RawArray("d", 66)  # head, L, R, L-state, R-state
        self._meta = context.RawArray("Q", 15)
        # seq[0:2], event_ns[2:4], valid[4:6], source_id[6:8], rejected[8:10]
        # right-A/B rising counts[10:12], previous right-A/left-B/right-B[12:15]
        self._clock_ns = clock_ns

    def _commit(self, stream, values, event_ns, source_id):
        if not self._lock.acquire(timeout=0.02):
            raise FreshnessError("XR event state lock unavailable")
        try:
            if stream == 1 and values is not None:
                buttons = (int(values[48]), int(values[40]), int(values[49]))
                # A new/invalid session cannot fabricate a rising edge. A valid
                # release followed by a press is needed to increment a counter.
                if self._meta[5] and self._meta[7] == source_id:
                    self._meta[10] += int(buttons[0] and not self._meta[12])
                    self._meta[11] += int(any(buttons[i] and not self._meta[12+i]
                                              for i in (1, 2)))
                self._meta[12:15] = buttons
            self._meta[stream] += 1
            self._meta[2 + stream] = event_ns
            self._meta[4 + stream] = int(values is not None)
            self._meta[6 + stream] = source_id
            if values is None:
                self._meta[8 + stream] += 1
                start, end = (0, 16) if stream == 0 else (16, 66)
                self._data[start:end] = [0.0] * (end - start)
            elif stream == 0:
                self._data[0:16] = values
            else:
                self._data[16:66] = values
        finally:
            self._lock.release()
        return values is not None

    def record_head(self, payload, *, source_id=0):
        event_ns = self._clock_ns()
        try:
            if not isinstance(payload, Mapping):
                raise ValueError("missing camera event")
            camera = payload["camera"]
            if not isinstance(camera, Mapping):
                raise ValueError("missing camera")
            _tracking_flags(payload)
            _tracking_flags(camera)
            values = _pose(camera["matrix"])
        except (KeyError, TypeError, ValueError, OverflowError):
            values = None
        return self._commit(0, values, event_ns, source_id)

    def record_controllers(self, payload, *, source_id=0):
        event_ns = self._clock_ns()
        try:
            if not isinstance(payload, Mapping):
                raise ValueError("missing controller event")
            _tracking_flags(payload)
            # Both poses AND both complete states are validated before one
            # commit. No optional button defaults or partial-side updates.
            values = (_pose(payload["left"]) + _pose(payload["right"])
                      + _controller(payload["leftState"])
                      + _controller(payload["rightState"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            values = None
        return self._commit(1, values, event_ns, source_id)

    def diagnostics(self):
        if not self._lock.acquire(timeout=0.02):
            raise FreshnessError("XR snapshot lock unavailable")
        try:
            meta = tuple(self._meta)
            now_ns = self._clock_ns()
        finally:
            self._lock.release()
        return dict(head_seq=meta[0], controller_seq=meta[1],
                    head_valid=bool(meta[4]), controller_valid=bool(meta[5]),
                    head_rejected=meta[8], controller_rejected=meta[9],
                    head_age_s=None if not meta[0] else (now_ns-meta[2])/1e9,
                    controller_age_s=None if not meta[1] else (now_ns-meta[3])/1e9,
                    right_A=bool(meta[12]) if meta[5] else None,
                    left_B=bool(meta[13]) if meta[5] else None,
                    right_B=bool(meta[14]) if meta[5] else None,
                    right_a_press_count=meta[10], b_press_count=meta[11])

    def snapshot(self, *, max_age_s=0.25, max_skew_s=0.10):
        return self._snapshot(max_age_s=max_age_s, max_skew_s=max_skew_s,
                              controller_world=False)

    def controller_world_snapshot(self, *, max_age_s=0.25):
        """Fresh controller world poses; head is initialization evidence only.

        Consumers MUST use controller world deltas, never subtract this head
        pose. Head validation and same-session identity still apply, but a
        stationary head's change-driven event need not act as a heartbeat.
        """
        return self._snapshot(max_age_s=max_age_s, max_skew_s=0.10,
                              controller_world=True)

    def _snapshot(self, *, max_age_s, max_skew_s, controller_world):
        max_age_s = _finite_number(max_age_s)
        max_skew_s = _finite_number(max_skew_s)
        if max_age_s <= 0 or max_skew_s < 0:
            raise ValueError("max_age_s must be positive; max_skew_s nonnegative")
        if not self._lock.acquire(timeout=0.02):
            raise FreshnessError("XR snapshot lock unavailable")
        try:
            meta, values = tuple(self._meta), tuple(self._data)
            now_ns = self._clock_ns()
        finally:
            self._lock.release()
        if not meta[0] or not meta[1]:
            raise FreshnessError("waiting for head and both controller events")
        if not meta[4] or not meta[5]:
            raise FreshnessError("latest head/controller event is invalid")
        if meta[6] != meta[7]:
            raise FreshnessError("head and controllers belong to different XR sessions")
        ages = (now_ns - meta[2], now_ns - meta[3])
        relevant_age = ages[1] if controller_world else max(ages)
        if min(ages) < 0 or relevant_age > max_age_s * 1e9:
            raise FreshnessError("head/controller event stream is stale")
        if not controller_world and abs(meta[2] - meta[3]) > max_skew_s * 1e9:
            raise FreshnessError("head/controller event skew is too large")
        return RawXRFrame(values[0:16], values[16:32], values[32:48],
                          _unpack_controller(values[48:57]),
                          _unpack_controller(values[57:66]), meta[0], meta[1],
                          meta[2], meta[3], now_ns, meta[6], meta[10], meta[11],
                          "controller_world" if controller_world else "head_relative")


def make_fresh_televuer(televuer_class):
    """Wrap an explicitly supplied installed TeleVuer class without patching it.

    The returned class starts the normal upstream server only when constructed.
    Pass ``use_hand_tracking=False``. Hooks use a shared state created before
    ``super().__init__``; the fork therefore shares it with the control process.
    Invalid events are not forwarded to legacy per-field storage.
    """
    class FreshTeleVuer(televuer_class):
        def __init__(self, *args, **kwargs):
            use_hand_tracking = args[0] if args else kwargs.get("use_hand_tracking")
            if use_hand_tracking is not False:
                raise ValueError("FreshTeleVuer supports controller tracking only")
            self.fresh_state = FreshXRState()
            super().__init__(*args, **kwargs)

        async def on_cam_move(self, event, session, fps=60):
            if self.fresh_state.record_head(getattr(event, "value", None),
                                            source_id=id(session)):
                await super().on_cam_move(event, session, fps=fps)

        async def on_controller_move(self, event, session, fps=60):
            if self.fresh_state.record_controllers(getattr(event, "value", None),
                                                   source_id=id(session)):
                await super().on_controller_move(event, session, fps=fps)

        def fresh_snapshot(self, **kwargs):
            return self.fresh_state.snapshot(**kwargs)

        def fresh_controller_world_snapshot(self, **kwargs):
            return self.fresh_state.controller_world_snapshot(**kwargs)

    return FreshTeleVuer
