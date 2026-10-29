"""Offline tests only: never import Vuer, an SDK, or create network listeners."""

import asyncio
import copy
from dataclasses import FrozenInstanceError
import math
import multiprocessing as mp
import threading
import time
from types import SimpleNamespace
import unittest

from g1d_vr_teleop.input.state import FreshnessError, FreshXRState, make_fresh_televuer


def pose(x=0.0):
    return [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., x, 1., 0., 1.]


def state(trigger=0.0):
    return dict(trigger=trigger > 0.5, triggerValue=trigger,
                squeeze=False, squeezeValue=0.0, thumbstick=False,
                thumbstickValue=[0., 0.], aButton=False, bButton=False)


def head():
    return dict(camera=dict(matrix=pose()))


def controllers(x=0.0, trigger=0.0):
    return dict(left=pose(x), right=pose(-x),
                leftState=state(trigger), rightState=state(trigger))


class Clock:
    def __init__(self):
        self.ns = 10_000_000_000

    def __call__(self):
        return self.ns

    def advance(self, seconds):
        self.ns += int(seconds * 1e9)


def child_write(shared, complete):
    shared.record_head(head(), source_id=123)
    shared.record_controllers(controllers(0.37, 0.6), source_id=123)
    complete.set()


class FreshXRTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.xr = FreshXRState(clock_ns=self.clock)

    def fill(self):
        self.assertTrue(self.xr.record_head(head()))
        self.assertTrue(self.xr.record_controllers(controllers(.3, .8)))

    def test_starts_uninitialized_and_needs_all_streams(self):
        with self.assertRaises(FreshnessError):
            self.xr.snapshot()
        self.xr.record_head(head())
        with self.assertRaises(FreshnessError):
            self.xr.snapshot()

    def test_raw_frame_and_states_are_immutable_atomic_copies(self):
        self.fill()
        frame = self.xr.snapshot()
        self.assertEqual(frame.sequence, (1, 1))
        self.assertEqual(frame.left[12], .3)
        self.assertEqual(frame.right[12], -.3)
        self.assertEqual(frame.left_state.trigger_value, .8)
        self.assertTrue(frame.left_state.trigger)
        with self.assertRaises(FrozenInstanceError):
            frame.head_seq = 99
        with self.assertRaises(FrozenInstanceError):
            frame.left_state.trigger_value = 0.
        self.xr.record_controllers(controllers(.4))
        self.assertEqual(frame.left[12], .3)
        self.assertEqual(self.xr.snapshot().left[12], .4)

    def test_reads_do_not_refresh_event_timestamps_or_sequence(self):
        self.fill()
        first = self.xr.snapshot()
        self.clock.advance(.20)
        second = self.xr.snapshot()
        self.assertEqual(second.sequence, first.sequence)
        self.assertEqual(second.head_monotonic_ns, first.head_monotonic_ns)
        self.assertAlmostEqual(second.age_s, .2)
        self.clock.advance(.051)
        with self.assertRaisesRegex(FreshnessError, "stale"):
            self.xr.snapshot()

    def test_one_stream_cannot_keep_the_other_alive(self):
        self.fill()
        self.clock.advance(.30)
        self.xr.record_controllers(controllers())
        with self.assertRaisesRegex(FreshnessError, "stale"):
            self.xr.snapshot()
        self.xr.record_head(head())
        self.assertEqual(self.xr.snapshot().sequence, (2, 2))

    def test_head_controller_skew_is_rejected(self):
        self.fill()
        self.clock.advance(.11)
        self.xr.record_controllers(controllers())
        with self.assertRaisesRegex(FreshnessError, "skew"):
            self.xr.snapshot()

    def test_world_snapshot_accepts_stationary_head_with_fresh_controllers(self):
        self.fill()
        self.clock.advance(10.)
        self.xr.record_controllers(controllers())
        frame = self.xr.controller_world_snapshot(max_age_s=.2)
        self.assertEqual(frame.pose_reference, 'controller_world')
        self.assertEqual(frame.head_seq, 1)
        self.assertEqual(frame.controller_seq, 2)
        self.assertEqual(self.xr.diagnostics()['head_age_s'], 10.)
        self.assertEqual(self.xr.diagnostics()['controller_age_s'], 0.)
        with self.assertRaisesRegex(FreshnessError, 'stale'):
            self.xr.snapshot()
        self.clock.advance(.201)
        with self.assertRaisesRegex(FreshnessError, 'stale'):
            self.xr.controller_world_snapshot(max_age_s=.2)

    def test_world_snapshot_keeps_initialization_session_and_validation_guards(self):
        self.xr.record_controllers(controllers(), source_id=1)
        with self.assertRaises(FreshnessError):
            self.xr.controller_world_snapshot()
        self.xr.record_head(head(), source_id=2)
        with self.assertRaisesRegex(FreshnessError, 'sessions'):
            self.xr.controller_world_snapshot()
        self.xr.record_head(head(), source_id=1)
        self.xr.controller_world_snapshot()
        bad = head()
        bad['camera']['tracked'] = False
        self.xr.record_head(bad, source_id=1)
        with self.assertRaisesRegex(FreshnessError, 'invalid'):
            self.xr.controller_world_snapshot()
        self.xr.record_head(head(), source_id=1)
        self.xr.record_controllers({}, source_id=1)
        with self.assertRaisesRegex(FreshnessError, 'invalid'):
            self.xr.controller_world_snapshot()

    def test_button_edges_survive_press_release_between_snapshots(self):
        self.fill()
        press = controllers()
        press['rightState']['aButton'] = True
        press['leftState']['bButton'] = True
        self.xr.record_controllers(press)
        self.xr.record_controllers(press)
        self.xr.record_controllers(controllers())
        frame = self.xr.controller_world_snapshot()
        self.assertFalse(frame.right_state.a_button)
        self.assertEqual(frame.right_a_press_count, 1)
        self.assertEqual(frame.b_press_count, 1)
        press['rightState']['bButton'] = True
        self.xr.record_controllers(press)
        frame = self.xr.snapshot()
        self.assertEqual(frame.right_a_press_count, 2)
        self.assertEqual(frame.b_press_count, 2)
        self.assertTrue(self.xr.diagnostics()['right_A'])

    def test_new_or_invalid_session_does_not_fabricate_press_edges(self):
        press = controllers()
        press['rightState']['aButton'] = True
        press['rightState']['bButton'] = True
        self.xr.record_head(head(), source_id=1)
        self.xr.record_controllers(press, source_id=1)
        self.assertEqual(self.xr.snapshot().right_a_press_count, 0)
        self.xr.record_controllers({}, source_id=1)
        self.xr.record_controllers(press, source_id=1)
        self.assertEqual(self.xr.snapshot().b_press_count, 0)
        self.xr.record_controllers(controllers(), source_id=1)
        self.xr.record_head(head(), source_id=2)
        self.xr.record_controllers(press, source_id=2)
        self.assertEqual(self.xr.snapshot().right_a_press_count, 0)

    def test_one_bad_controller_invalidates_whole_pair_immediately(self):
        self.fill()
        bad = controllers(.9)
        del bad["rightState"]["squeezeValue"]
        self.assertFalse(self.xr.record_controllers(bad))
        with self.assertRaisesRegex(FreshnessError, "invalid"):
            self.xr.snapshot()
        diagnostic = self.xr.diagnostics()
        self.assertFalse(diagnostic["controller_valid"])
        self.assertEqual(diagnostic["controller_rejected"], 1)
        self.assertEqual(diagnostic["controller_seq"], 2)
        self.xr.record_controllers(controllers(.2))
        self.assertEqual(self.xr.snapshot().left[12], .2)

    def test_invalid_head_never_reuses_old_pose(self):
        self.fill()
        self.assertFalse(self.xr.record_head(dict(camera=dict(matrix=[0.] * 16))))
        with self.assertRaises(FreshnessError):
            self.xr.snapshot()

    def test_matrix_rejects_nan_inf_reflection_scale_and_bad_shape(self):
        matrices = [[0.] * 16, [1.] * 15, "1234567890123456", None]
        for index, value in ((0, math.nan), (12, math.inf), (0, -1.),
                             (0, 2.), (3, .1), (15, 0.)):
            matrix = pose()
            matrix[index] = value
            matrices.append(matrix)
        for matrix in matrices:
            with self.subTest(matrix=matrix):
                self.fill()
                self.assertFalse(self.xr.record_head(dict(camera=dict(matrix=matrix))))
                with self.assertRaises(FreshnessError):
                    self.xr.snapshot()

    def test_required_buttons_analog_and_axes_are_validated(self):
        for key, value in (("triggerValue", math.nan), ("triggerValue", 1.1),
                           ("squeezeValue", -.1), ("triggerValue", "0.2"),
                           ("triggerValue", True), ("squeeze", "false"),
                           ("aButton", None), ("thumbstickValue", [0]),
                           ("thumbstickValue", [0, 1.2])):
            with self.subTest(key=key, value=value):
                self.fill()
                bad = controllers()
                bad["leftState"][key] = value
                self.assertFalse(self.xr.record_controllers(bad))
                with self.assertRaises(FreshnessError):
                    self.xr.snapshot()

    def test_missing_optional_in_legacy_button_is_not_defaulted(self):
        self.fill()
        bad = controllers()
        del bad["rightState"]["bButton"]
        self.assertFalse(self.xr.record_controllers(bad))
        with self.assertRaises(FreshnessError):
            self.xr.snapshot()

    def test_negative_tracking_flag_rejects_valid_old_matrix(self):
        for flag in ("connected", "tracked", "valid"):
            self.fill()
            bad = controllers()
            bad["leftState"][flag] = False
            self.assertFalse(self.xr.record_controllers(bad))
            with self.assertRaises(FreshnessError):
                self.xr.snapshot()

    def test_sessions_cannot_mix(self):
        self.xr.record_head(head(), source_id=10)
        self.xr.record_controllers(controllers(), source_id=20)
        with self.assertRaisesRegex(FreshnessError, "sessions"):
            self.xr.snapshot()
        self.xr.record_head(head(), source_id=20)
        self.assertEqual(self.xr.snapshot().source_id, 20)

    def test_future_timestamps_rejected(self):
        self.fill()
        self.clock.advance(-.1)
        with self.assertRaises(FreshnessError):
            self.xr.snapshot()

    def test_invalid_thresholds_rejected(self):
        self.fill()
        for kwargs in (dict(max_age_s=0), dict(max_age_s=math.nan),
                       dict(max_age_s=math.inf), dict(max_skew_s=-.1)):
            with self.assertRaises(ValueError):
                self.xr.snapshot(**kwargs)

    def test_lock_timeout_fails_closed_without_hanging(self):
        locked, release = threading.Event(), threading.Event()
        def hold_lock():
            with self.xr._lock:
                locked.set()
                release.wait(2.)
        thread = threading.Thread(target=hold_lock)
        thread.start()
        self.assertTrue(locked.wait(1.))
        started = time.monotonic()
        try:
            with self.assertRaisesRegex(FreshnessError, "lock unavailable"):
                self.xr.snapshot()
            self.assertLess(time.monotonic() - started, .5)
        finally:
            release.set()
            thread.join(2.)

    @unittest.skipUnless("fork" in mp.get_all_start_methods(), "Linux/fork target")
    def test_state_written_in_child_is_visible_to_parent(self):
        context = mp.get_context("fork")
        xr = FreshXRState(context=context)
        complete = context.Event()
        child = context.Process(target=child_write, args=(xr, complete))
        child.start()
        self.assertTrue(complete.wait(3.))
        child.join(3.)
        self.assertEqual(child.exitcode, 0)
        frame = xr.snapshot(max_age_s=5.)
        self.assertEqual(frame.source_id, 123)
        self.assertEqual(frame.left[12], .37)
        self.assertEqual(frame.left_state.trigger_value, .6)

    def test_concurrent_reader_does_not_mix_controller_fields(self):
        self.fill()
        done = threading.Event()
        def write():
            for n in range(1000):
                self.xr.record_controllers(controllers(float(n), (n % 101) / 100.))
            done.set()
        thread = threading.Thread(target=write)
        thread.start()
        while not done.is_set():
            frame = self.xr.snapshot()
            if frame.controller_seq > 1:
                self.assertEqual(frame.left[12], -frame.right[12])
                self.assertEqual(frame.left_state.trigger_value,
                                 (int(frame.left[12]) % 101) / 100.)
                self.assertEqual(frame.left_state, frame.right_state)
        thread.join(2.)


class HookTests(unittest.TestCase):
    def test_hooks_allocate_before_base_init_and_forward_only_valid_events(self):
        class FakeTeleVuer:
            def __init__(self, **kwargs):
                self.state_existed_at_fork_point = hasattr(self, "fresh_state")
                self.forwarded = []

            async def on_cam_move(self, event, session, fps=60):
                self.forwarded.append("head")

            async def on_controller_move(self, event, session, fps=60):
                self.forwarded.append("controllers")

        fresh_class = make_fresh_televuer(FakeTeleVuer)
        tv = fresh_class(use_hand_tracking=False)
        session = object()
        asyncio.run(tv.on_cam_move(SimpleNamespace(value=head()), session))
        asyncio.run(tv.on_controller_move(SimpleNamespace(value=controllers()), session))
        self.assertTrue(tv.state_existed_at_fork_point)
        self.assertEqual(tv.fresh_snapshot().sequence, (1, 1))
        self.assertEqual(tv.forwarded, ["head", "controllers"])
        asyncio.run(tv.on_controller_move(SimpleNamespace(value={}), session))
        self.assertEqual(tv.forwarded, ["head", "controllers"])
        with self.assertRaises(FreshnessError):
            tv.fresh_snapshot()
        with self.assertRaises(ValueError):
            fresh_class(use_hand_tracking=True)


if __name__ == "__main__":
    unittest.main()
