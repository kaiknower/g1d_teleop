"""No DDS dependency: original advancing samples alone renew freshness."""
from dataclasses import FrozenInstanceError
import unittest

from g1d_vr_teleop.feedback.arms import ArmStateAccumulator, ArmStateError, ArmStateUnavailable


class LatestArmTests(unittest.TestCase):
    def ingest(self, core, *, tick=10, source=100, now=1_000_000_000, q=None, mode=5):
        return core.ingest([0.] * 35 if q is None else q, mode, tick, source, now_ns=now)

    def test_empty_reader_has_no_fresh_sample(self):
        with self.assertRaises(ArmStateUnavailable):
            ArmStateAccumulator().snapshot(now_ns=1_000_000_000)

    def test_accepts_29_and_35_and_neutralizes_invalid_slot(self):
        for size in (29, 35):
            core = ArmStateAccumulator()
            q = [.1] * size
            q[13] = float('nan')
            self.assertTrue(self.ingest(core, q=q))
            sample = core.snapshot(now_ns=1_010_000_000)
            self.assertEqual(len(sample.q), 29)
            self.assertEqual(sample.q[13], 0.)
            self.assertEqual(sample.sequence, 1)
            self.assertEqual(sample.received_monotonic_ns, sample.host_monotonic_ns)
            with self.assertRaises(FrozenInstanceError):
                sample.tick = 11

    def test_repeated_tick_or_source_does_not_refresh(self):
        for tick, source in ((10, 100), (10, 101), (11, 100)):
            core = ArmStateAccumulator()
            self.ingest(core)
            self.assertFalse(self.ingest(core, tick=tick, source=source, now=1_190_000_000))
            self.assertEqual(core.latest.host_monotonic_ns, 1_000_000_000)
            self.assertEqual(core.latest.sequence, 1)
            with self.assertRaisesRegex(ArmStateError, 'stale'):
                core.snapshot(now_ns=1_200_000_001)

    def test_advancing_sample_restores_freshness_without_clock_comparison(self):
        core = ArmStateAccumulator()
        self.ingest(core, source=900_000_000_000_000)
        self.assertTrue(self.ingest(core, tick=11, source=900_000_000_000_001, now=1_250_000_000))
        self.assertEqual(core.snapshot(now_ns=1_260_000_000).sequence, 2)

    def test_backwards_or_wrapped_counters_latch_fault(self):
        for tick, source, now in ((9, 101, 1_010_000_000),
                                  (11, 99, 1_010_000_000),
                                  (11, 101, 999_999_999),
                                  (0, 101, 1_010_000_000)):
            core = ArmStateAccumulator()
            self.ingest(core)
            with self.assertRaises(ArmStateError):
                self.ingest(core, tick=tick, source=source, now=now)
            with self.assertRaises(ArmStateError):
                core.snapshot(now_ns=1_010_000_000)

    def test_bad_payloads_fail_closed(self):
        bad = [dict(q=[0.] * 28), dict(q=[0.] * 34), dict(mode=True), dict(mode=256),
               dict(tick=-1), dict(source=0), dict(now=0)]
        for value in (float('nan'), float('inf'), True, '0'):
            q = [0.] * 29
            q[18] = value
            bad.append(dict(q=q))
        for kwargs in bad:
            core = ArmStateAccumulator()
            with self.assertRaises(ArmStateError):
                self.ingest(core, **kwargs)
            with self.assertRaises(ArmStateError):
                self.ingest(core, tick=20, source=200, now=2_000_000_000)

    def test_age_is_checked_from_original_successful_take(self):
        core = ArmStateAccumulator()
        self.ingest(core)
        self.assertEqual(core.snapshot(now_ns=1_200_000_000).sequence, 1)
        for now in (999_999_999, 1_200_000_001):
            with self.assertRaises(ArmStateError):
                core.snapshot(now_ns=now)


if __name__ == '__main__':
    unittest.main()
