"""Independent stereo/XR frontend. No robot command or DDS publisher."""

from g1d_vr_teleop.config import CERT_FILE, KEY_FILE, STATE_ROOT, INPUT_ROOT
import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import os
from pathlib import Path
import signal
import threading
import time

from g1d_vr_teleop.input.state import make_fresh_televuer, FreshnessError


class VideoFreshness:
    """Small shared metadata; no image, disk operation or decoder under lock."""
    def __init__(self):
        self.lock=threading.Lock()
        self.last_image_ns=None
        self.frames=0

    def record_frame(self, now_ns):
        with self.lock:
            self.last_image_ns=now_ns
            self.frames+=1

    def snapshot(self):
        with self.lock:
            stamp,frames=self.last_image_ns,self.frames
            now_ns=time.monotonic_ns()
        return frames,None if stamp is None else (now_ns-stamp)/1e9


@contextmanager
def json_publisher():
    """ZMQ socket and context belong exclusively to the calling worker."""
    import zmq
    ctx=zmq.Context()
    output=ctx.socket(zmq.PUB)
    try:
        output.setsockopt(zmq.LINGER,0)
        output.setsockopt(zmq.SNDHWM,2)
        output.setsockopt(zmq.SNDTIMEO,50)
        output.bind('tcp://127.0.0.1:6014')
        yield output
    finally:
        output.close(0)
        ctx.term()


class XRPublisher(threading.Thread):
    """50 Hz input publication independent of video decoding and status I/O."""
    def __init__(self,tv,video,*,publisher_factory=json_publisher,
                 clock_ns=time.monotonic_ns,period_s=.02):
        super().__init__(name='xr-input-publisher',daemon=True)
        self.tv,self.video,self.factory=tv,video,publisher_factory
        self.clock_ns,self.period_s=clock_ns,period_s
        self.stop_event=threading.Event()
        self.lock=threading.Lock()
        self.error=None
        self.latest=None

    def run(self):
        count,last_sent,max_gap,max_loop=0,None,0.,0.
        try:
            with self.factory() as output:
                while not self.stop_event.is_set():
                    begin=self.clock_ns()
                    frames,camera_age=self.video.snapshot()
                    diagnostics={}
                    try:
                        diagnostics=self.tv.fresh_state.diagnostics()
                        frame=self.tv.fresh_controller_world_snapshot(max_age_s=.2)
                        frames,camera_age=self.video.snapshot()
                        if camera_age is None or not 0 <= camera_age <= .3:
                            raise FreshnessError('Head camera is stale')
                        payload={'valid':True,'frame':asdict(frame)}
                    except FreshnessError as exc:
                        payload={'valid':False,'reason':str(exc)}
                    output.send_json(payload)
                    sent=self.clock_ns()
                    gap=0. if last_sent is None else (sent-last_sent)/1e9
                    elapsed=(sent-begin)/1e9
                    count+=1;max_gap=max(max_gap,gap);max_loop=max(max_loop,elapsed)
                    last_sent=sent
                    latest={'payload':payload,'diagnostics':diagnostics,
                            'camera_frames':frames,'camera_age_s':camera_age,
                            'monotonic_s':sent/1e9,'publisher_frames':count,
                            'publisher_gap_s':gap,'publisher_max_gap_s':max_gap,
                            'publisher_loop_s':elapsed,'publisher_max_loop_s':max_loop}
                    with self.lock:self.latest=latest
                    self.stop_event.wait(max(0.,self.period_s-(self.clock_ns()-begin)/1e9))
        except Exception as exc:
            with self.lock:self.error=f'{type(exc).__name__}: {exc}'

    def snapshot(self):
        with self.lock:
            error,latest=self.error,self.latest
        if error is not None:
            raise RuntimeError('XR publication failed: '+error)
        if self.ident is not None and not self.is_alive() and not self.stop_event.is_set():
            raise RuntimeError('XR publication thread exited')
        return latest

    def close(self):
        self.stop_event.set()
        if self.ident is not None:
            self.join(timeout=1.)
            if self.is_alive():
                raise RuntimeError('XR publication thread did not stop')


def control_overlay_status(base):
    """Read the session receipt, never infer control readiness from video."""
    try:
        current = json.loads((base / 'current.json').read_text())
        status = json.loads((Path(current['control_directory']) / 'control-status.json').read_text())
        alive = False
        pid = current.get('control_pid')
        if type(pid) is int and pid > 1:
            try:
                os.kill(pid, 0)
                alive = True
            except ProcessLookupError:
                pass
        phase = status.get('phase', 'UNKNOWN')
        if not alive and phase not in ('fault_hold', 'completed', 'recording_finalize_failed'):
            phase = 'CONTROL OFF'
        return phase, status.get('error') or status.get('reason') or ''
    except (OSError, ValueError, KeyError):
        return 'CONTROL OFF', 'No live control status'


def draw_status(image, phase, reason, diagnostics, xr_valid):
    import cv2
    image = image.copy()
    color = (60, 230, 60) if phase == 'vr_active' and xr_valid else (40, 200, 255)
    for x in (8, 648):
        cv2.rectangle(image, (x-3, 0), (x+625, 82), (0, 0, 0), -1)
        cv2.putText(image, str(phase)[:44], (x, 22), cv2.FONT_HERSHEY_SIMPLEX, .52, color, 1)
        line = ('A=%s clicks=%s | tap A:start hold A:finish B:stop' %
                (diagnostics.get('right_A'), diagnostics.get('right_a_press_count')))
        cv2.putText(image, line, (x, 45), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
        detail = str(reason)[:75] if reason else ('XR READY' if xr_valid else 'XR WAITING')
        cv2.putText(image, detail, (x, 67), cv2.FONT_HERSHEY_SIMPLEX, .40, color, 1)
    return image


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, default=INPUT_ROOT)
    a = p.parse_args(argv)
    import cv2
    import numpy as np
    import zmq
    from televuer import TeleVuer
    root = a.directory.resolve()
    root.mkdir(parents=True, exist_ok=True)
    public = root / 'public'
    public.mkdir(exist_ok=True)
    os.chdir(public)
    ctx = zmq.Context()
    camera = ctx.socket(zmq.SUB)
    camera.setsockopt(zmq.LINGER, 0)
    camera.setsockopt(zmq.RCVHWM, 2)
    camera.setsockopt(zmq.SUBSCRIBE, b'')
    camera.connect('tcp://127.0.0.1:55555')
    FreshTV = make_fresh_televuer(TeleVuer)
    tv = FreshTV(use_hand_tracking=False, binocular=True, img_shape=(480, 1280),
                 display_fps=20., display_mode='immersive', zmq=True, webrtc=False,
                 cert_file=str(CERT_FILE),
                 key_file=str(KEY_FILE))
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    video=VideoFreshness()
    publisher=XRPublisher(tv,video)
    publisher.start()
    report_at=0.
    phase, control_reason = 'CONTROL OFF', ''
    try:
        while running:
            begin = time.monotonic()
            # Drain to latest camera frame; original XR event timestamps are
            # preserved separately and never refreshed by this publisher.
            wire = None
            for _ in range(4):
                try:
                    wire = camera.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break
            img = None
            if wire is not None:
                img = cv2.imdecode(np.frombuffer(wire, np.uint8), cv2.IMREAD_COLOR)
                if img is not None and img.shape == (480, 1280, 3):
                    video.record_frame(time.monotonic_ns())
                else:
                    img = None
            now = time.monotonic()
            published=publisher.snapshot()
            payload=published['payload'] if published else {'valid':False,'reason':'XR publisher preparing'}
            diagnostics=published['diagnostics'] if published else {}
            _,camera_age=video.snapshot()
            if now >= report_at:
                phase, control_reason = control_overlay_status(STATE_ROOT)
            if img is not None:
                tv.render_to_xr(draw_status(img, phase, control_reason or payload.get('reason'),
                                           diagnostics, payload['valid']))
            if camera_age is None or camera_age > .6:
                blank = np.zeros((480, 1280, 3), dtype=np.uint8)
                for x in (20, 660):
                    cv2.putText(blank, 'CAMERA STALE', (x, 220), cv2.FONT_HERSHEY_SIMPLEX,
                                1., (0, 0, 255), 2)
                tv.render_to_xr(blank)
            if not tv.process.is_alive():
                raise RuntimeError('XR web child exited')
            if now >= report_at:
                status = {'robot_control_enabled': False, 'camera_frames': video.snapshot()[0],
                          'camera_age_s': camera_age, 'xr_valid': payload['valid'],
                          'xr_reason': payload.get('reason'), 'monotonic_s': now,
                          'web_pid': tv.process.pid, 'recording': False,
                          'pose_reference': 'controller_world',
                          'control_phase': phase, 'control_reason': control_reason,
                          'diagnostics': diagnostics,
                          'publisher':None if published is None else {
                              k:v for k,v in published.items() if k.startswith('publisher_') or k=='monotonic_s'}}
                tmp = root / 'input-status.tmp'
                tmp.write_text(json.dumps(status, indent=2) + '\n')
                tmp.replace(root / 'input-status.json')
                report_at = now + .5
            time.sleep(max(0., .02 - (time.monotonic() - begin)))
    finally:
        try:
            publisher.close()
        finally:
            camera.close(0)
            ctx.term()
            tv.close()


if __name__ == '__main__':
    main()
