"""Rearm another A-gated VR episode only after verified normal completion.

No motion, startup, return or recovery is issued by this manager. The one child
owns its normal return and finalization. Fault, B, signal or unknown receipts
stop rearming; a new manager requires explicit operator action.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import threading
import time

from g1d_vr_teleop.control.launcher import BASE, launch_session, require_ready, write_json_atomic


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def verify_normal_completion(meta, returncode, expected_boot, current_boot):
    """Pure receipt validation; callers MUST reap the child before this call."""
    if current_boot != expected_boot:
        raise RuntimeError('Robot boot changed; automatic rearm stopped')
    if returncode != 0:
        raise RuntimeError(f'VR child did not exit normally: returncode={returncode}')
    directory=Path(meta['control_directory'])
    status=json.loads((directory/'control-status.json').read_text())
    if (status.get('phase')!='completed' or status.get('returned_to_initial') is not True
            or status.get('grippers_opened') is not True
            or 'error' not in status or status['error'] is not None
            or status.get('robot_control_enabled') is not False
            or status.get('recording') is not False):
        raise RuntimeError('Normal return/control-stop receipt is missing or invalid')
    output=Path(meta['recording_directory'])
    if (output/'raw').exists():
        receipt=json.loads((directory/'recording-result.json').read_text())
        episode=json.loads((output/'episode.json').read_text())
        if receipt != episode or receipt.get('raw')!=str(output/'raw'):
            raise RuntimeError('Recording finalization receipts disagree')
        if set(receipt.get('recorders',{}))!={'video','action-state'}:
            raise RuntimeError('Both recorder finalization receipts are required')
        task=receipt.get('task')
        if task is not None and (not isinstance(task,dict) or task.get('status')!='finalized_verified'):
            raise RuntimeError('Task recording did not pass final verification')
        for name in ('video','action-state'):
            report=json.loads((output/'raw'/name/'recording.json').read_text())
            if (receipt['recorders'][name] not in ('completed','finalized')
                    or report.get('status') not in ('completed','finalized')
                    or report.get('errors')):
                raise RuntimeError(f'{name} recording did not finalize successfully')
    return status


class SessionManager:
    def __init__(self, *, base=BASE, launch=launch_session, ready=require_ready,
                 read_boot=boot_id, stop_event=None):
        self.base,self.launch,self.ready,self.read_boot=Path(base),launch,ready,read_boot
        self.stop=stop_event or threading.Event()
        self.child=None
        self.meta=None
        self.terminated=False
        self.episodes_completed=0
        self.start_boot=read_boot()

    def report(self,phase,**extra):
        data={'manager_pid':os.getpid(),'phase':phase,'boot_id':self.start_boot,
              'monotonic_s':time.monotonic(),'episodes_completed':self.episodes_completed,
              'child_pid':None if self.child is None else self.child.pid,
              'control_directory':None if self.meta is None else self.meta.get('control_directory'),
              'automatic_motion':False,**extra}
        write_json_atomic(self.base/'manager-status.json',data)
        current_path=self.base/'current.json'
        if current_path.exists():
            current=json.loads(current_path.read_text())
            current.update(manager_pid=os.getpid(),manager_phase=phase)
            write_json_atomic(current_path,current)

    def request_stop(self,*_):
        self.stop.set()

    def run(self):
        try:
            while not self.stop.is_set():
                if self.read_boot()!=self.start_boot:
                    raise RuntimeError('Robot boot changed; automatic rearm stopped')
                self.ready()
                if self.stop.is_set():break
                self.meta,self.child=self.launch(execute=True,manager_pid=os.getpid())
                if self.child is None:raise RuntimeError('Manager launch returned no child')
                self.terminated=False
                self.report('child_waiting_for_operator')
                while self.child.poll() is None:
                    if self.stop.is_set():
                        if not self.terminated:
                            self.child.terminate()
                            self.terminated=True
                            self.report('stopping_child_hold')
                        # Do not spawn or kill another process if a child takes
                        # time to finalize; retain ownership until it exits.
                        time.sleep(.1)
                    else:self.stop.wait(.2)
                code=self.child.wait()  # Reap only after all child finalization.
                if self.stop.is_set():
                    self.report('stopped',child_returncode=code)
                    return 0
                self.report('verifying_normal_completion',child_returncode=code)
                verify_normal_completion(self.meta,code,self.start_boot,self.read_boot())
                self.ready()  # Healthy READY must be restored by the child.
                self.episodes_completed+=1
                self.child=None
                self.report('rearming_wait_for_new_A')
            self.report('stopped')
            return 0
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
            if self.child is not None and self.child.poll() is None:
                # Even a metadata/reporting error must not orphan a controller.
                # The child handles SIGTERM by holding and finalizing, never by
                # attempting a return trajectory on this fault path.
                self.stop.set()
                if not self.terminated:
                    self.child.terminate()
                    self.terminated=True
                self.child.wait()
            self.report('stopped_requires_operator',error=error)
            return 1


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    a=p.parse_args(argv)
    if not a.execute:
        print(json.dumps({'execute_required':True,'motion_commands_sent':0,
                          'normal_end':'verify return and recording, then wait for a NEW A press',
                          'fault_end':'stop rearming; no automatic recovery or return'}))
        return 0
    BASE.mkdir(parents=True,exist_ok=True)
    with (BASE/'manager-owner.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        lock.seek(0);lock.truncate();lock.write(str(os.getpid()));lock.flush()
        manager=SessionManager()
        signal.signal(signal.SIGINT,manager.request_stop)
        signal.signal(signal.SIGTERM,manager.request_stop)
        return manager.run()


if __name__=='__main__':raise SystemExit(main())
