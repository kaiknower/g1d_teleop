"""Bounded passive recorder reuse; no robot publisher or lifecycle requests."""
import json
import os
from pathlib import Path
import subprocess
import time


from g1d_vr_teleop.config import PYTHON as TV, CAMERA_PYTHON as CAMERA, child_environment


def read(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


class Recording:
    def __init__(self, output):
        self.output, self.raw = output, output / 'raw'
        self.processes, self.logs = [], []
        self.stopped, self.interval = False, None

    def prepare(self, abort):
        self.raw.mkdir(parents=True, exist_ok=False)
        env = child_environment()
        specifications = (
            ('video', CAMERA, 'g1d_vr_teleop.recording.video'),
            ('action-state', TV, 'g1d_vr_teleop.recording.actions'))
        for name, python, module in specifications:
            log = (self.raw/f'{name}.log').open('x')
            self.logs.append(log)
            argv = [python, '-m', module, '--output', str(self.raw/name),
                    '--duration', '180', '--stop-file', str(self.raw/f'{name}.stop')]
            self.processes.append(subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, env=env))
        deadline = time.monotonic() + 8.
        while not abort.is_set():
            self.check()
            video, action = (read(self.raw/name/'recording.json') for name in ('video','action-state'))
            if (all(video.get('receive_stats',{}).get(name,{}).get('received_frames',0)>0
                    for name in ('head_stereo','left_wrist','right_wrist'))
                    and all(action.get('received',{}).get(topic,0)>0 for topic in
                            ('rt/lowstate','rt/dex1/left/state','rt/dex1/right/state'))):
                return
            if time.monotonic() > deadline:
                raise RuntimeError('Three-video/action recorders did not acquire all streams')
            abort.wait(.1)
        raise RuntimeError('Recording preparation interrupted')

    def check(self):
        if self.stopped:
            return
        for name, process in zip(('video','action-state'), self.processes):
            report = read(self.raw/name/'recording.json')
            if process.poll() is not None or report.get('status') == 'failed' or report.get('errors'):
                raise RuntimeError(f'{name} recorder failed or ended unexpectedly')

    def request_stop(self, start=None, end=None):
        if self.stopped:
            return
        self.stopped = True
        if start is not None and end is not None:
            self.interval = {'start_monotonic_s':start,'end_monotonic_s':end,
                'start_definition':'first of 3 distinct measured states displaced >0.001 rad, with corresponding nonzero arm command',
                'end_definition':'60 seconds after measured motion onset, or earlier deliberate operator finish',
                'clock':'robot host monotonic; video frame receive timeline, not hardware synchronization'}
        for name in ('video','action-state'):
            (self.raw/f'{name}.stop').touch()

    def finalize(self):
        self.request_stop()
        for process in self.processes:
            process.wait(timeout=20.)
            if process.returncode != 0:
                raise RuntimeError('Recorder returned failure while finalizing')
        for log in self.logs:
            log.close()
        reports = {name:read(self.raw/name/'recording.json') for name in ('video','action-state')}
        if any(report.get('status') != 'completed' or report.get('errors') for report in reports.values()):
            # Existing recorders use their own terminal status spelling.
            for name, report in reports.items():
                if report.get('status') in ('failed','starting','recording') or report.get('errors'):
                    raise RuntimeError(f'Unfinalized/failed recorder: {name}')
        receipt = {'raw':str(self.raw),'raw_is_delivery':False,'interval':self.interval,
                   'recorders':{name:report.get('status') for name,report in reports.items()}}
        if self.interval:
            from g1d_vr_teleop.recording.trim import trim_episode
            receipt['task'] = trim_episode(self.output,self.interval,phase='dual_arm_dex1_vr_teleoperation')
        else:
            receipt['task'] = None
            receipt['reason'] = 'No sustained task arm motion was observed; raw remains diagnostic only'
        (self.output/'episode.json').write_text(json.dumps(receipt,indent=2)+'\n')
        return receipt
