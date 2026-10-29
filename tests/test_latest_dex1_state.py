"""Pure checks: no DDS, robot publisher, network, or lifecycle dependency."""
from types import SimpleNamespace
import unittest

from g1d_vr_teleop.core.feedback import RawDex1Feedback, RawDex1FeedbackError, RawDex1FeedbackUnavailable
from g1d_vr_teleop.core.contact import Dex1ContactController
from g1d_vr_teleop.feedback.grippers import Dex1StateAccumulator, LatestDex1Subscriber


def message(**kwargs):
    values = dict(q=4.08, dq=.02, tau_est=.01, mode=1, lost=0)
    values.update(kwargs)
    return SimpleNamespace(states=[SimpleNamespace(**values)])


class LatestDexTests(unittest.TestCase):
    def test_constructor_is_inert_and_empty_has_no_sample(self):
        reader = LatestDex1Subscriber(side='left')
        self.assertFalse(reader.status()['started'])
        self.assertFalse(reader.status()['publisher_created'])
        with self.assertRaises(RawDex1FeedbackUnavailable):
            reader.snapshot()
        reader.close()
        with self.assertRaises(RawDex1FeedbackError):
            reader.snapshot()

    def test_both_topics_keep_existing_raw_sample_contract(self):
        for side in ('left', 'right'):
            core = Dex1StateAccumulator(side=side)
            core.ingest(message(), 100, now_ns=1_000_000_000)
            sample = core.snapshot(now_ns=1_010_000_000)
            self.assertIsInstance(sample, RawDex1Feedback)
            self.assertEqual(sample.topic, f'rt/dex1/{side}/state')
            self.assertEqual(sample.timestamp_kind, 'original_DDS_callback_host_monotonic_ns_not_producer_timestamp')
            self.assertEqual((sample.q, sample.dq, sample.tau_est, sample.mode, sample.lost), (4.08, .02, .01, 1, 0))

    def test_repeated_source_never_refreshes_and_empty_gap_expires(self):
        core = Dex1StateAccumulator()
        core.ingest(message(), 100, now_ns=1_000_000_000)
        self.assertFalse(core.ingest(message(), 100, now_ns=1_099_000_000))
        self.assertEqual(core.snapshot(now_ns=1_100_000_000).received_monotonic_ns, 1_000_000_000)
        with self.assertRaises(RawDex1FeedbackError):
            core.snapshot(now_ns=1_100_000_001)
        self.assertEqual(core.latest.sequence, 1)

    def test_optional_half_second_reader_keeps_original_stamp_and_default_is_strict(self):
        core=Dex1StateAccumulator()
        core.ingest(message(),100,now_ns=1_000_000_000)
        with self.assertRaises(RawDex1FeedbackError):
            core.snapshot(now_ns=1_150_000_000)
        sample=core.snapshot(now_ns=1_150_000_000,max_age_s=.5)
        self.assertEqual(sample.received_monotonic_ns,1_000_000_000)
        self.assertEqual(sample.sequence,1)
        core.snapshot(now_ns=1_500_000_000,max_age_s=.5)
        with self.assertRaises(RawDex1FeedbackError):
            core.snapshot(now_ns=1_500_000_001,max_age_s=.5)
        reader=LatestDex1Subscriber(max_age_s=.5,clock_ns=lambda:1_150_000_000)
        reader._core.ingest(message(),100,now_ns=1_000_000_000)
        self.assertEqual(reader.snapshot().received_monotonic_ns,1_000_000_000)
        self.assertEqual(reader.status()['max_age_s'],.5)

    def test_feedback_window_is_finite_positive_and_at_most_watchdog(self):
        for value in (0,-1,.500001,True,float('nan'),float('inf'),'0.5'):
            with self.assertRaises(ValueError):LatestDex1Subscriber(max_age_s=value)
            with self.assertRaises(ValueError):Dex1StateAccumulator().snapshot(now_ns=1,max_age_s=value)

    def test_ordering_does_not_compare_source_and_host_clocks(self):
        core = Dex1StateAccumulator()
        core.ingest(message(), 900_000_000_000_000, now_ns=1_000_000_000)
        core.ingest(message(), 900_000_000_000_001, now_ns=1_050_000_000)
        self.assertEqual(core.snapshot(now_ns=1_060_000_000).sequence, 2)

    def test_backwards_source_or_host_latches(self):
        for source, host in ((99, 1_010_000_000), (101, 999_999_999)):
            core = Dex1StateAccumulator()
            core.ingest(message(), 100, now_ns=1_000_000_000)
            with self.assertRaises(RawDex1FeedbackError):
                core.ingest(message(), source, now_ns=host)
            with self.assertRaises(RawDex1FeedbackError):
                core.snapshot(now_ns=1_010_000_000)

    def test_bad_data_and_lost_fail_closed(self):
        for msg in (message(lost=1), message(lost=False), message(q=float('nan')),
                    message(dq=float('inf')), message(tau_est=True), message(mode=256),
                    SimpleNamespace(states=[]), SimpleNamespace(states=[message(), message()])):
            core = Dex1StateAccumulator()
            with self.assertRaises(RawDex1FeedbackError):
                core.ingest(msg, 100, now_ns=1_000_000_000)
            with self.assertRaises(RawDex1FeedbackError):
                core.ingest(message(), 101, now_ns=1_010_000_000)

    def test_repeated_snapshot_is_not_new_contact_evidence(self):
        core, contact = Dex1StateAccumulator(), Dex1ContactController()
        core.ingest(message(), 100, now_ns=1_000_000_000)
        sample = core.snapshot(now_ns=1_000_000_000)
        first = contact.update(.28, sample.q, feedback_stamp_ns=sample.received_monotonic_ns,
                               now_ns=1_000_000_000, lost=sample.lost)
        sample = core.snapshot(now_ns=1_050_000_000)
        repeat = contact.update(.28, sample.q, feedback_stamp_ns=sample.received_monotonic_ns,
                                now_ns=1_050_000_000, lost=sample.lost)
        self.assertFalse(repeat.accepted_new_feedback)
        self.assertEqual(repeat.target, first.target)
        core.ingest(message(), 101, now_ns=1_050_000_000)
        sample = core.snapshot(now_ns=1_050_000_000)
        fresh = contact.update(.28, sample.q, feedback_stamp_ns=sample.received_monotonic_ns,
                               now_ns=1_050_000_000, lost=sample.lost)
        self.assertTrue(fresh.accepted_new_feedback)


if __name__ == '__main__':
    unittest.main()
