"""Passive latest-only Dex1 feedback; no SDK listener queue or motion API.

Initialize the site's DDS domain 0 before start(). Source timestamps establish
message ordering only. Snapshot freshness is measured from the original
successful take on this host; reading a snapshot never renews it.
"""
from dataclasses import dataclass
import math
from numbers import Real
import threading
import time
from g1d_vr_teleop.feedback.arms import DDS_ENTITY_CREATION_LOCK

from g1d_vr_teleop.core.feedback import (
    RawDex1Feedback, RawDex1FeedbackError,
    RawDex1FeedbackUnavailable,
)


def _max_age_ns(max_age_s):
    if (isinstance(max_age_s, bool) or not isinstance(max_age_s, Real)
            or not math.isfinite(max_age_s) or not 0 < max_age_s <= .5):
        raise ValueError('Dex1 feedback max_age_s must be within (0,0.5]')
    return round(max_age_s * 1e9)


@dataclass(frozen=True)
class LatestDex1Feedback(RawDex1Feedback):
    source_timestamp_ns: int = 0


class Dex1StateAccumulator:
    """Pure validation and ordering core with caller-supplied receive clock."""
    def __init__(self, *, side='right'):
        if side not in {'left', 'right'}:
            raise ValueError('Dex1 side must be left or right')
        self.topic = f'rt/dex1/{side}/state'
        self.latest = None
        self.error = None

    def ingest(self, message, source_timestamp_ns, *, now_ns):
        if self.error is not None:
            raise RawDex1FeedbackError(self.error)
        try:
            if len(message.states) != 1:
                raise RawDex1FeedbackError('Dex1 must provide exactly one motor state')
            motor = message.states[0]
            values = (motor.q, motor.dq, motor.tau_est)
            if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v)
                   for v in values):
                raise RawDex1FeedbackError('Nonfinite or nonnumeric Dex1 feedback')
            if type(motor.lost) is not int or motor.lost != 0:
                raise RawDex1FeedbackError('Dex1 feedback lost must be integer zero')
            if type(motor.mode) is not int or not 0 <= motor.mode <= 255:
                raise RawDex1FeedbackError('Malformed Dex1 mode')
            if type(source_timestamp_ns) is not int or source_timestamp_ns <= 0:
                raise RawDex1FeedbackError('Invalid original Dex1 DDS source timestamp')
            if type(now_ns) is not int or now_ns <= 0:
                raise RawDex1FeedbackError('Invalid Dex1 host receive timestamp')
            old = self.latest
            if old is not None:
                if source_timestamp_ns < old.source_timestamp_ns or now_ns <= old.received_monotonic_ns:
                    raise RawDex1FeedbackError('Dex1 source or receive time moved backwards; restart required')
                if source_timestamp_ns == old.source_timestamp_ns:
                    return False
            # The protocol tag is retained for compatibility with the existing
            # contact/core API; timestamp is original host acquisition time,
            # now from take() rather than an SDK callback or snapshot polling.
            self.latest = LatestDex1Feedback(
                sequence=1 if old is None else old.sequence + 1,
                received_monotonic_ns=now_ns,
                q=float(values[0]), dq=float(values[1]), tau_est=float(values[2]),
                mode=motor.mode, lost=motor.lost, topic=self.topic,
                source_timestamp_ns=source_timestamp_ns)
            return True
        except Exception as exc:
            self.error = str(exc)
            if isinstance(exc, RawDex1FeedbackError):
                raise
            raise RawDex1FeedbackError(self.error) from exc

    def snapshot(self, *, now_ns, max_age_s=.1):
        maximum_age_ns = _max_age_ns(max_age_s)
        if self.error is not None:
            raise RawDex1FeedbackError('Feedback fault latched: ' + self.error)
        if self.latest is None:
            raise RawDex1FeedbackUnavailable('No original latest-only Dex1 sample received')
        if (type(now_ns) is not int
                or not 0 <= now_ns - self.latest.received_monotonic_ns <= maximum_age_ns):
            raise RawDex1FeedbackError(f'Original Dex1 sample is future-dated or older than {max_age_s:g} seconds')
        return self.latest


class LatestDex1Subscriber:
    """Drop-in passive per-side reader. DDS domain 0 must already be configured."""
    def __init__(self, *, side='right', clock_ns=time.monotonic_ns, max_age_s=.1):
        _max_age_ns(max_age_s)
        self.max_age_s = float(max_age_s)
        self._core = Dex1StateAccumulator(side=side)
        self.topic, self.clock_ns = self._core.topic, clock_ns
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._thread, self._error = None, None

    def start(self):
        if self._thread is not None or self._stop.is_set():
            raise RawDex1FeedbackError('Latest-only Dex1 reader cannot be restarted')
        self._thread = threading.Thread(target=self._run,
            name='latest-only-' + self.topic.replace('/', '-'), daemon=True)
        self._thread.start()
        return self

    def _run(self):
        participant = topic = reader = None
        try:
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.topic import Topic
            from cyclonedds.sub import DataReader
            from cyclonedds.qos import Qos, Policy
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
            with DDS_ENTITY_CREATION_LOCK:
                participant = DomainParticipant(0)
                topic = Topic(participant, self.topic, MotorStates_)
                reader = DataReader(participant, topic, qos=Qos(
                    Policy.History.KeepLast(1), Policy.Reliability.BestEffort))
            while not self._stop.is_set():
                samples = reader.take(64)
                received = self.clock_ns()
                valid = [sample for sample in samples if sample.sample_info.valid_data]
                if valid:
                    newest = valid[-1]
                    with self._lock:
                        self._core.ingest(newest, newest.sample_info.source_timestamp,
                                          now_ns=received)
                self._stop.wait(.01)
        except Exception as exc:
            with self._lock:
                self._error = f'Latest-only Dex1 reader failed: {type(exc).__name__}: {exc}'
        finally:
            del reader
            del topic
            del participant

    def snapshot(self, *, now_ns=None):
        with self._lock:
            # Observe the clock after acquiring the sample lock. A writer can
            # otherwise commit a newer sample between the clock and this lock.
            now = self.clock_ns() if now_ns is None else now_ns
            if self._stop.is_set():
                raise RawDex1FeedbackError('Latest-only Dex1 reader is closed')
            if self._error is not None:
                raise RawDex1FeedbackError(self._error)
            return self._core.snapshot(now_ns=now, max_age_s=self.max_age_s)

    def status(self):
        with self._lock:
            sample = self._core.latest
            return {'topic': self.topic, 'started': self._thread is not None,
                'closed': self._stop.is_set(), 'fault': self._error or self._core.error,
                'callbacks': 0 if sample is None else sample.sequence,
                'acquisition': 'original_latest_only_DDS_take',
                'max_age_s': self.max_age_s,
                'latest_received_monotonic_ns': None if sample is None else sample.received_monotonic_ns,
                'source_timestamp_ns': None if sample is None else sample.source_timestamp_ns,
                'publisher_created': False, 'motion_commands_sent': 0}

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.)
            if self._thread.is_alive():
                raise RawDex1FeedbackError('Latest-only Dex1 reader did not stop')
