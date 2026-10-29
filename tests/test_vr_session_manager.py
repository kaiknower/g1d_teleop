"""Manager lifecycle tests with inert fake children; no robot/network calls."""
import fcntl
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import g1d_vr_teleop.control.launcher as launcher
from g1d_vr_teleop.control.manager import SessionManager, verify_normal_completion


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value))


def receipt(base,index=1,*,valid=True,finalized=True):
    directory=base/f'session-{index}';output=base/f'recording-{index}'
    status=dict(phase='completed' if valid else 'fault_hold',returned_to_initial=valid,
                grippers_opened=valid,
                error=None if valid else 'B stop',robot_control_enabled=False,recording=False)
    save(directory/'control-status.json',status)
    final=dict(raw=str(output/'raw'),recorders={'video':'finalized','action-state':'finalized'},task=None)
    save(directory/'recording-result.json',final);save(output/'episode.json',final)
    for name in ('video','action-state'):
        save(output/'raw'/name/'recording.json',dict(status='finalized' if finalized else 'recording',errors=[]))
    return dict(control_directory=str(directory),recording_directory=str(output),control_pid=101)


class Child:
    def __init__(self,pid=101,code=0,on_wait=None):
        self.pid,self.code,self.on_wait=pid,code,on_wait
        self.reaped=False;self.terminations=0
    def poll(self):return self.code
    def wait(self):
        self.reaped=True
        if self.on_wait:self.on_wait()
        return self.code
    def terminate(self):self.terminations+=1;self.code=-15


class ManagerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.base=Path(self.tmp.name)
    def tearDown(self):self.tmp.cleanup()

    def test_normal_rearms_only_after_wait_and_finalization_then_fault_stops(self):
        children=[];calls=[]
        def launch(**kwargs):
            if children:self.assertTrue(children[-1].reaped)
            i=len(children)+1;calls.append(kwargs)
            meta=receipt(self.base,i,finalized=False)
            child=Child(100+i,0 if i==1 else 1,
                        on_wait=lambda:receipt(self.base,i,finalized=True))
            children.append(child);return meta,child
        manager=SessionManager(base=self.base,launch=launch,ready=lambda:None,read_boot=lambda:'boot')
        self.assertEqual(manager.run(),1)
        self.assertEqual(len(children),2)
        self.assertEqual(manager.episodes_completed,1)
        self.assertTrue(all(c.reaped for c in children))
        self.assertTrue(all(c['execute'] for c in calls))

    def test_fault_invalid_return_and_unfinalized_recording_do_not_rearm(self):
        for kind in ('exit','fault','return','closed_grippers','recording','missing'):
            with self.subTest(kind=kind):
                meta=receipt(self.base,kind,valid=kind!='fault',finalized=kind!='recording')
                p=Path(meta['control_directory'])/'control-status.json'
                if kind=='return':
                    v=json.loads(p.read_text());v['returned_to_initial']=False;save(p,v)
                if kind=='closed_grippers':
                    v=json.loads(p.read_text());v['grippers_opened']=False;save(p,v)
                if kind=='missing':p.unlink()
                calls=[];child=Child(code=1 if kind=='exit' else 0)
                def launch(**kwargs):calls.append(kwargs);return meta,child
                manager=SessionManager(base=self.base,launch=launch,ready=lambda:None,read_boot=lambda:'boot')
                self.assertEqual(manager.run(),1)
                self.assertEqual(len(calls),1)
                self.assertEqual(manager.episodes_completed,0)

    def test_boot_change_after_child_exit_stops_rearm(self):
        boots=iter(('one','one','two'))
        calls=[];meta=receipt(self.base)
        def launch(**kwargs):calls.append(kwargs);return meta,Child()
        manager=SessionManager(base=self.base,launch=launch,ready=lambda:None,read_boot=lambda:next(boots))
        self.assertEqual(manager.run(),1);self.assertEqual(len(calls),1)

    def test_post_exit_unhealthy_ready_stops_rearm(self):
        reads=[];calls=[];meta=receipt(self.base)
        def ready():
            reads.append(1)
            if len(reads)>1:raise RuntimeError('Bridge no longer READY')
        def launch(**kwargs):calls.append(kwargs);return meta,Child()
        manager=SessionManager(base=self.base,launch=launch,ready=ready,read_boot=lambda:'boot')
        self.assertEqual(manager.run(),1);self.assertEqual(len(calls),1)

    def test_stop_terminates_one_child_once_and_never_rearms(self):
        event=threading.Event();meta=receipt(self.base);child=Child(code=None);calls=[]
        def launch(**kwargs):
            calls.append(kwargs);event.set();return meta,child
        manager=SessionManager(base=self.base,launch=launch,ready=lambda:None,read_boot=lambda:'boot',stop_event=event)
        self.assertEqual(manager.run(),0)
        self.assertEqual(child.terminations,1)
        self.assertTrue(child.reaped);self.assertEqual(len(calls),1)

    def test_no_motion_finalized_receipt_is_valid_without_claiming_task_dataset(self):
        meta=receipt(self.base)
        self.assertTrue(verify_normal_completion(meta,0,'boot','boot')['returned_to_initial'])

    def test_present_task_requires_finalized_verified_status(self):
        meta=receipt(self.base)
        p=Path(meta['control_directory'])/'recording-result.json'
        data=json.loads(p.read_text());data['task']={'status':'partial'}
        save(p,data);save(Path(meta['recording_directory'])/'episode.json',data)
        with self.assertRaisesRegex(RuntimeError,'final verification'):
            verify_normal_completion(meta,0,'boot','boot')

    def test_manager_error_terminates_and_reaps_live_child_once(self):
        meta=receipt(self.base);child=Child(code=None)
        manager=SessionManager(base=self.base,launch=lambda **kw:(meta,child),
                               ready=lambda:None,read_boot=lambda:'boot')
        with patch.object(manager,'report',side_effect=[RuntimeError('report write failed'),None]):
            self.assertEqual(manager.run(),1)
        self.assertTrue(child.reaped);self.assertEqual(child.terminations,1)

    def test_manual_launch_refused_while_manager_lock_is_held(self):
        with (self.base/'manager-owner.lock').open('a+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeError,'manager owns'):
                with launcher._manager_guard(self.base,None):pass

    def test_dry_run_api_returns_metadata_without_spawning(self):
        save(self.base/'current.json',{})
        save(self.base/'initial-pose.json',{'boot_id':'boot'})
        with patch.object(launcher,'BASE',self.base),patch.object(launcher,'require_ready'),\
             patch.object(launcher,'current_boot_id',return_value='boot'),\
             patch.object(launcher.subprocess,'Popen') as spawn:
            meta,proc=launcher.launch_session()
        self.assertIsNone(proc);spawn.assert_not_called()
        self.assertEqual(meta['motion_commands_sent'],0)

    def test_initial_reference_wrong_boot_is_rejected_before_child_spawn(self):
        save(self.base/'current.json',{})
        save(self.base/'initial-pose.json',{'boot_id':'old'})
        with patch.object(launcher,'BASE',self.base),patch.object(launcher,'require_ready'),\
             patch.object(launcher,'current_boot_id',return_value='current'),\
             patch.object(launcher.subprocess,'Popen') as spawn:
            with self.assertRaisesRegex(RuntimeError,'another robot boot'):
                launcher.launch_session(execute=True)
        spawn.assert_not_called()


if __name__=='__main__':unittest.main()
