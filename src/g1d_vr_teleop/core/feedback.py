"""Passive Dex1 DDS feedback, with original callback receive timestamps.

Constructing this adapter does not import the SDK or open a channel. start()
creates exactly one ChannelSubscriber; no publisher, robot controller, bridge,
RPC, or lifecycle operation is constructed. MotorStates_ has no producer
timestamp: received_monotonic_ns is host callback time, not exposure/firmware
time, and snapshot() never refreshes it.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time


TOPIC = "rt/dex1/right/state"
MAX_FEEDBACK_AGE_NS = 100_000_000


class RawDex1FeedbackError(RuntimeError):
    pass


class RawDex1FeedbackUnavailable(RawDex1FeedbackError):
    """Initial readiness only; no sample has arrived yet."""


@dataclass(frozen=True)
class RawDex1Feedback:
    sequence: int
    received_monotonic_ns: int
    q: float
    dq: float
    tau_est: float
    mode: int
    lost: int
    topic: str = TOPIC
    timestamp_kind: str = "original_DDS_callback_host_monotonic_ns_not_producer_timestamp"


class RawDex1Subscriber:
    def __init__(self, *, side="right", interface="eth0", domain_id=0, clock_ns=time.monotonic_ns,
                 channel_initialize=None, subscriber_factory=None, message_type=None):
        if side not in {"left", "right"}:
            raise ValueError("Dex1 side must be left or right")
        self.topic = f"rt/dex1/{side}/state"
        injected = (channel_initialize, subscriber_factory, message_type)
        if any(value is not None for value in injected) and not all(value is not None for value in injected):
            raise ValueError("Inject all three SDK boundaries together")
        self.interface, self.domain_id, self.clock_ns = interface, domain_id, clock_ns
        self._initialize, self._factory, self._message_type = injected
        self._lock = threading.Lock()
        self._latest = self._subscriber = None
        self._error = None
        self._callbacks = 0
        self._closed = self._started = False

    def start(self):
        if self._closed or self._error:
            raise RawDex1FeedbackError("Closed/faulted subscriber cannot be restarted implicitly")
        if self._started:
            raise RawDex1FeedbackError("Subscriber already started")
        if self._factory is None:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
            self._initialize, self._factory, self._message_type = ChannelFactoryInitialize, ChannelSubscriber, MotorStates_
        self._initialize(self.domain_id, self.interface)
        self._subscriber = self._factory(self.topic, self._message_type)
        self._started = True
        try:
            self._subscriber.Init(self._on_message, 100)
        except BaseException:
            self.close()
            raise
        return self

    def _on_message(self, message):
        # Timestamp immediately at callback entry, before the lock or packing.
        stamp = self.clock_ns()
        with self._lock:
            if self._closed or self._error:
                return
            self._callbacks += 1
            try:
                if type(stamp) is not int or stamp <= 0:
                    raise ValueError("Invalid callback monotonic timestamp")
                if self._latest is not None and stamp <= self._latest.received_monotonic_ns:
                    raise ValueError("Callback timestamps must increase strictly")
                if len(message.states) != 1:
                    raise ValueError("Right Dex1 must provide exactly one motor state")
                state = message.states[0]
                q, dq, tau = float(state.q), float(state.dq), float(state.tau_est)
                if not all(math.isfinite(x) for x in (q, dq, tau)):
                    raise ValueError("Nonfinite right-Dex1 feedback")
                if type(state.lost) is not int or state.lost != 0:
                    raise ValueError(f"Right-Dex1 feedback lost != 0 or malformed: {state.lost!r}")
                if type(state.mode) is not int:
                    raise ValueError("Malformed right-Dex1 mode")
                self._latest = RawDex1Feedback(self._callbacks, stamp, q, dq, tau, state.mode, state.lost,
                                              topic=self.topic)
            except BaseException as exc:
                # SDK callback thread must not silently swallow a bad reading
                # and leave the old good value visible to a closing controller.
                self._error = {"callback_sequence": self._callbacks,
                    "received_monotonic_ns": stamp, "error": f"{type(exc).__name__}: {exc}"}

    def snapshot(self, *, now_ns=None):
        now = self.clock_ns() if now_ns is None else now_ns
        with self._lock:
            if self._closed:
                raise RawDex1FeedbackError("Feedback subscriber is closed")
            if self._error:
                raise RawDex1FeedbackError(f"Feedback fault latched: {self._error}")
            sample = self._latest
        if sample is None:
            raise RawDex1FeedbackUnavailable("No original right-Dex1 DDS callback received")
        if type(now) is not int or not 0 <= now-sample.received_monotonic_ns <= MAX_FEEDBACK_AGE_NS:
            raise RawDex1FeedbackError("Original right-Dex1 callback is future-dated or older than 0.1 seconds")
        return sample

    def status(self):
        with self._lock:
            return {"topic": self.topic, "started": self._started, "closed": self._closed,
                    "callbacks": self._callbacks, "fault": self._error,
                    "latest_received_monotonic_ns": None if self._latest is None else self._latest.received_monotonic_ns,
                    "publisher_created": False, "motion_commands_sent": 0}

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            subscriber = self._subscriber
        if subscriber is not None:
            subscriber.Close()  # Existing SDK CloseReader path only.
