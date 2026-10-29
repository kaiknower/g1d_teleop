"""Input-thread tests with fake XR and transport; no Vuer, ZMQ or robot SDK."""
from contextlib import contextmanager
from dataclasses import dataclass
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from g1d_vr_teleop.input.state import FreshnessError
from g1d_vr_teleop.input.server import VideoFreshness,XRPublisher


@dataclass
class Frame:
    controller_monotonic_ns:int
    controller_seq:int
    source_id:int=23
    right_a_press_count:int=7
    b_press_count:int=4
    pose_reference:str='controller_world'


class FakeXR:
    def __init__(self):
        self.count=0
        self.failure=None
        self.last_frame=None
        self.fresh_state=SimpleNamespace(diagnostics=lambda:{'controller_seq':self.count,'right_a_press_count':7})
    def fresh_controller_world_snapshot(self,*,max_age_s):
        assert max_age_s==.2
        if self.failure:raise FreshnessError(self.failure)
        self.count+=1
        self.last_frame=Frame(time.monotonic_ns()-1_000_000,self.count)
        return self.last_frame


class Wire:
    def __init__(self,fail=False):
        self.fail=fail
        self.rows=[]
        self.owner=self.closed=None
        self.threads=[]
    @contextmanager
    def factory(self):
        self.owner=threading.get_ident()
        try:yield self
        finally:self.closed=threading.get_ident()
    def send_json(self,value):
        self.threads.append(threading.get_ident())
        if self.fail:raise RuntimeError('simulated transport failure')
        self.rows.append(value)


def wait_for(predicate,timeout=1.):
    deadline=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>deadline:raise AssertionError('condition timed out')
        time.sleep(.002)


class PublisherTests(unittest.TestCase):
    def test_video_snapshot_clock_is_acquired_under_sample_lock(self):
        video=VideoFreshness()
        video.record_frame(100)
        def clock():
            self.assertTrue(video.lock.locked())
            return 150
        with patch('g1d_vr_teleop.input.server.time.monotonic_ns',side_effect=clock):
            self.assertEqual(video.snapshot(),(1,50/1e9))

    def start(self,*,video=None,xr=None,wire=None):
        video=video or VideoFreshness();video.record_frame(time.monotonic_ns())
        xr=xr or FakeXR();wire=wire or Wire()
        publisher=XRPublisher(xr,video,publisher_factory=wire.factory)
        self.addCleanup(publisher.close)
        publisher.start()
        return publisher,video,xr,wire

    def test_video_work_block_does_not_block_fresh_input_publication(self):
        publisher,_,_,wire=self.start()
        wait_for(lambda:len(wire.rows)>=2)
        baseline=len(wire.rows)
        # Stand in for a blocked image decoder/status writer on the main thread.
        # The only activity here is the independent input worker.
        threading.Event().wait(.18)
        self.assertGreaterEqual(len(wire.rows)-baseline,5)
        self.assertTrue(wire.rows[-1]['valid'])
        status=publisher.snapshot()
        self.assertGreater(status['publisher_frames'],baseline)
        self.assertGreater(status['publisher_max_gap_s'],0.)
        self.assertGreaterEqual(status['publisher_max_loop_s'],status['publisher_loop_s'])

    def test_worker_owns_creation_sends_and_close_and_preserves_original_stamp(self):
        publisher,_,xr,wire=self.start()
        wait_for(lambda:len(wire.rows)>=2)
        publisher.close()
        self.assertNotEqual(wire.owner,threading.get_ident())
        self.assertEqual(wire.owner,wire.closed)
        self.assertTrue(all(t==wire.owner for t in wire.threads))
        frame=wire.rows[-1]['frame']
        self.assertEqual(frame['controller_monotonic_ns'],xr.last_frame.controller_monotonic_ns)
        self.assertEqual(frame['source_id'],23)
        self.assertEqual(frame['right_a_press_count'],7)
        self.assertEqual(frame['b_press_count'],4)

    def test_old_video_fails_closed_while_fresh_controller_stream_continues(self):
        publisher,video,_,wire=self.start()
        wait_for(lambda:len(wire.rows)>=1)
        video.record_frame(time.monotonic_ns()-310_000_000)
        wait_for(lambda:any(row.get('reason')=='Head camera is stale' for row in wire.rows))
        self.assertFalse(publisher.snapshot()['payload']['valid'])
        video.record_frame(time.monotonic_ns())
        wait_for(lambda:publisher.snapshot()['payload']['valid'])

    def test_invalid_source_is_published_as_invalid_not_last_good_frame(self):
        publisher,_,xr,wire=self.start()
        wait_for(lambda:len(wire.rows)>=1)
        xr.failure='latest controller tracking flag invalid'
        wait_for(lambda:wire.rows[-1].get('reason')==xr.failure)
        self.assertNotIn('frame',wire.rows[-1])
        self.assertFalse(wire.rows[-1]['valid'])
        self.assertIn('controller_seq',publisher.snapshot()['diagnostics'])

    def test_transport_failure_is_visible_to_main_and_worker_closes(self):
        publisher,_,_,wire=self.start(wire=Wire(fail=True))
        wait_for(lambda:not publisher.is_alive())
        with self.assertRaisesRegex(RuntimeError,'simulated transport failure'):
            publisher.snapshot()
        self.assertEqual(wire.owner,wire.closed)
        self.assertEqual(len(wire.threads),1)

    def test_constructing_worker_does_not_create_transport(self):
        wire=Wire()
        publisher=XRPublisher(FakeXR(),VideoFreshness(),publisher_factory=wire.factory)
        self.assertIsNone(wire.owner)
        self.assertIsNone(publisher.snapshot())
        publisher.close()


if __name__=='__main__':unittest.main()
