"""Offline manual-gripper behavior and feedback boundary tests."""
import unittest
from g1d_vr_teleop.core.contact import Dex1ContactController, Dex1ContactError
from g1d_vr_teleop.control.grippers import ManualDex1Controller


BASE = 1_000_000_000


def step(control, q=4.08, desired=.28, stamp=BASE, now=None, lost=0):
    return control.update(desired, q, feedback_stamp_ns=stamp,
                          now_ns=stamp if now is None else now, lost=lost)


class ManualDexTests(unittest.TestCase):
    def test_fresh_closing_advance_is_eight_times_autonomous_and_twice_previous(self):
        manual, autonomous = ManualDex1Controller(), Dex1ContactController()
        a, b = step(manual), step(autonomous)
        a2, b2 = step(manual, stamp=BASE+10_000_000), step(autonomous, stamp=BASE+10_000_000)
        self.assertAlmostEqual(a.target-a2.target, .064)
        self.assertAlmostEqual(a.target-a2.target, 8*(b.target-b2.target))
        previous=ManualDex1Controller();previous.closing_speed_raw_s=3.2
        p=step(previous);p2=step(previous,stamp=BASE+10_000_000)
        self.assertAlmostEqual(a.target-a2.target,2*(p.target-p2.target))
        self.assertEqual(autonomous.closing_speed_raw_s, .8)

    def test_empty_close_stalls_hold_without_fault_or_grasp_claim(self):
        for q in (.26839295, .28, .35, .45):
            control = ManualDex1Controller()
            for i in range(600):  # No autonomous closure timeout.
                result = step(control, q=q, stamp=BASE+i*50_000_000)
                self.assertGreaterEqual(result.target, .28)
                self.assertFalse(result.grasp_success)
                self.assertFalse(result.lift_allowed)
                self.assertFalse(result.contact_candidate)
            self.assertEqual(result.phase, 'limit_hold')

    def test_release_opens_from_empty_or_object_hold(self):
        for q in (.35, 4.08):
            control = ManualDex1Controller()
            for i in range(8):
                result = step(control, q=q, stamp=BASE+i*50_000_000)
            self.assertAlmostEqual(result.target, max(.28, q-.04))
            opened = step(control, q=q, desired=5.38, stamp=BASE+400_000_000)
            self.assertAlmostEqual(opened.target, min(5.38, q+.18))
            self.assertFalse(opened.contact_candidate)

    def test_opening_endstop_holds_without_abort(self):
        control = ManualDex1Controller()
        for i in range(30):
            output = step(control, q=5.15, desired=5.38, stamp=BASE+i*50_000_000)
        self.assertEqual(output.phase, 'limit_hold')
        self.assertAlmostEqual(output.target, 5.19)
        self.assertFalse(output.grasp_success)

    def test_duplicate_feedback_cannot_move_even_if_trigger_changes(self):
        control = ManualDex1Controller()
        first = step(control)
        duplicate = step(control, desired=5.38, now=BASE+50_000_000)
        self.assertEqual(duplicate.target, first.target)
        self.assertFalse(duplicate.accepted_new_feedback)
        with self.assertRaises(Dex1ContactError):
            step(control, q=4.09, now=BASE+60_000_000)

    def test_boundaries_clip_targets_and_allow_only_feedback_tolerance(self):
        for q, request in ((.18, -100), (.26839295, -100), (.28, -100), (5.48, 100), (4., 100)):
            result = step(ManualDex1Controller(), q=q, desired=request)
            self.assertTrue(.28 <= result.target <= 5.38)
        for q in (.179, 5.481, float('nan'), float('inf')):
            with self.assertRaises(Dex1ContactError):
                step(ManualDex1Controller(), q=q)

    def test_low_endpoint_readback_can_hold_and_reopen_without_lowering_command_limit(self):
        control=ManualDex1Controller()
        q=.26839295
        for i in range(8):
            closed=step(control,q=q,stamp=BASE+i*50_000_000)
            self.assertEqual(closed.target,.28)
            self.assertFalse(closed.contact_candidate)
            self.assertFalse(closed.grasp_success)
        opened=step(control,q=q,desired=5.38,stamp=BASE+400_000_000)
        self.assertAlmostEqual(opened.target,q+.18)
        self.assertTrue(.28<=opened.target<=5.38)
        with self.assertRaises(Dex1ContactError):
            step(Dex1ContactController(),q=q)

    def test_stale_lost_backwards_and_invalid_timestamps_latch(self):
        for kwargs in (dict(now=BASE+100_000_001), dict(lost=1), dict(lost=False),
                       dict(now=BASE-1), dict(stamp=0)):
            control = ManualDex1Controller()
            with self.assertRaises(Dex1ContactError):
                step(control, **kwargs)
            with self.assertRaises(Dex1ContactError):
                step(control, stamp=BASE+200_000_000)
        control = ManualDex1Controller()
        step(control)
        with self.assertRaises(Dex1ContactError):
            step(control, stamp=BASE-1)


if __name__ == '__main__':
    unittest.main()
