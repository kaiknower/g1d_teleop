"""Passive, latest-only LowState feedback. Importing this module opens nothing.

Start after RawDex1Subscriber has initialized DDS domain 0 on the site network.
Unlike the SDK listener, the DDS reader retains only the newest sample and is
polled at 100 Hz. Source timestamps establish ordering, not cross-host age.
"""
from dataclasses import dataclass
import math
from numbers import Real
import threading
import time

DDS_ENTITY_CREATION_LOCK = threading.Lock()


class ArmStateError(RuntimeError):
    """Feedback cannot be used; the caller must stop sending and retain hold."""


class ArmStateUnavailable(ArmStateError):
    """No valid, advancing sample has arrived yet."""


@dataclass(frozen=True)
class LatestArmSample:
    q: tuple
    mode_machine: int
    host_monotonic_ns: int
    tick: int
    source_timestamp_ns: int
    sequence: int

    @property
    def received_monotonic_ns(self):
        return self.host_monotonic_ns


class ArmStateAccumulator:
    """Pure accumulator; all clocks and message fields are supplied by caller."""
    def __init__(self):
        self.latest = None
        self.error = None

    def ingest(self, q, mode_machine, tick, source_timestamp_ns, *, now_ns):
        if self.error is not None:
            raise ArmStateError(self.error)
        try:
            values = tuple(q)
            if len(values) not in (29, 35):
                raise ArmStateError('LowState must contain 29 or 35 motors')
            # G1D slot 13 is invalid telemetry, not a physical waist joint.
            values = tuple(0. if i == 13 else value for i, value in enumerate(values[:29]))
            if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v)
                   for v in values):
                raise ArmStateError('LowState contains nonfinite or nonnumeric joint positions')
            values = tuple(float(v) for v in values)
            if type(mode_machine) is not int or not 0 <= mode_machine <= 255:
                raise ArmStateError('Invalid LowState mode_machine')
            if type(tick) is not int or not 0 <= tick <= 0xffffffff:
                raise ArmStateError('Invalid LowState tick')
            if type(source_timestamp_ns) is not int or source_timestamp_ns <= 0:
                raise ArmStateError('Invalid DDS source timestamp')
            if type(now_ns) is not int or now_ns <= 0:
                raise ArmStateError('Invalid host receive timestamp')
            old = self.latest
            if old is not None:
                if (tick < old.tick or source_timestamp_ns < old.source_timestamp_ns
                        or now_ns <= old.host_monotonic_ns):
                    raise ArmStateError('LowState counter or timestamp moved backwards; restart required')
                if tick == old.tick or source_timestamp_ns == old.source_timestamp_ns:
                    return False  # Duplicate input never renews freshness.
            self.latest = LatestArmSample(values, mode_machine, now_ns, tick,
                                          source_timestamp_ns, 1 if old is None else old.sequence + 1)
            return True
        except Exception as exc:
            self.error = str(exc)
            if isinstance(exc, ArmStateError):
                raise
            raise ArmStateError(self.error) from exc

    def snapshot(self, *, now_ns, max_age_s=.2):
        if self.error is not None:
            raise ArmStateError(self.error)
        if self.latest is None:
            raise ArmStateUnavailable('Waiting for an original LowState sample')
        if (type(now_ns) is not int or isinstance(max_age_s, bool)
                or not isinstance(max_age_s, Real) or not math.isfinite(max_age_s) or max_age_s <= 0):
            raise ArmStateError('Invalid feedback age check')
        age = now_ns - self.latest.host_monotonic_ns
        if not 0 <= age <= max_age_s * 1e9:
            raise ArmStateError('Original latest-only LowState is stale or future-dated')
        return self.latest


class LatestArmState:
    """One passive reader thread; never creates publishers or lifecycle RPCs."""
    def __init__(self, *, max_age_s=.2, poll_interval_s=.01):
        if not 0 < max_age_s <= .2 or not 0 < poll_interval_s <= .02:
            raise ValueError('Expected max_age_s <= .2 and poll_interval_s <= .02')
        self.max_age_s, self.poll_interval_s = max_age_s, poll_interval_s
        self._core = ArmStateAccumulator()
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._thread, self._error = None, None

    def start(self):
        if self._thread is not None or self._stop.is_set():
            raise ArmStateError('LatestArmState instances cannot be restarted')
        self._thread = threading.Thread(target=self._run, name='latest-only-lowstate', daemon=True)
        self._thread.start()
        return self

    def _run(self):
        participant = topic = reader = None
        try:
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.topic import Topic
            from cyclonedds.sub import DataReader
            from cyclonedds.qos import Qos, Policy
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            # Cyclone's Python IDL type cache initializes lazily; creating
            # readers concurrently can race while resolving nested types.
            with DDS_ENTITY_CREATION_LOCK:
                participant = DomainParticipant(0)
                topic = Topic(participant, 'rt/lowstate', LowState_)
                reader = DataReader(participant, topic, qos=Qos(
                    Policy.History.KeepLast(1), Policy.Reliability.BestEffort))
            while not self._stop.is_set():
                samples = reader.take(64)
                received = time.monotonic_ns()
                valid = [message for message in samples if message.sample_info.valid_data]
                if valid:
                    # Also coalesce a batch if samples arrive during take().
                    # Only its newest sample receives this host timestamp.
                    message = valid[-1]
                    info = message.sample_info
                    with self._lock:
                        self._core.ingest(tuple(m.q for m in message.motor_state),
                            message.mode_machine, message.tick, info.source_timestamp,
                            now_ns=received)
                self._stop.wait(self.poll_interval_s)
        except Exception as exc:
            with self._lock:
                self._error = f'Passive latest-only LowState reader failed: {exc}'
        finally:
            # Cyclone Python entities release their DDS handles on destruction.
            # They are owned only by this thread, after its final take().
            del reader
            del topic
            del participant

    def snapshot(self, *, max_age_s=None, now_ns=None):
        maximum = self.max_age_s if max_age_s is None else max_age_s
        if (isinstance(maximum, bool) or not isinstance(maximum, Real)
                or not math.isfinite(maximum) or not 0 < maximum <= self.max_age_s):
            raise ArmStateError('Snapshot age must not exceed the configured feedback limit')
        with self._lock:
            now_ns = time.monotonic_ns() if now_ns is None else now_ns
            if self._error is not None:
                raise ArmStateError(self._error)
            if self._stop.is_set():
                raise ArmStateError('Latest-only LowState reader is closed')
            return self._core.snapshot(now_ns=now_ns, max_age_s=maximum)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.)
            if self._thread.is_alive():
                raise ArmStateError('Latest-only LowState reader did not stop')
