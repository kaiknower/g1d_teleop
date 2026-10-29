"""Explicit normal return from healthy READY, reusing the VR session sender."""

from g1d_vr_teleop.config import URDF, ROBOT_HOST, DDS_INTERFACE, STATE_ROOT
import argparse
from dataclasses import replace
from datetime import datetime,timezone
import fcntl
import json
from pathlib import Path
import signal
import time

from g1d_vr_teleop.core.dual_arm import DualVRControl,DualVRDecision,parse_urdf_limits,validate_ready_status
from g1d_vr_teleop.feedback.arms import LatestArmState
from g1d_vr_teleop.feedback.grippers import LatestDex1Subscriber
from g1d_vr_teleop.control.session import Session,DirectFeedbackClient,XRReceiver,read_state,VRControlError
from g1d_vr_teleop.control.profile import SiteDualVRControl


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true')
    p.add_argument('--open-grippers-only',action='store_true');a=p.parse_args(argv)
    base=STATE_ROOT
    base.mkdir(parents=True,exist_ok=True)
    lock=(base/'control-owner.lock').open('a+');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    reference=json.loads((base/'initial-pose.json').read_text())
    if reference['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        raise VRControlError('Fixed initial pose belongs to another boot')
    if not a.execute:
        print(json.dumps({'motion_commands_sent':0,'goal':reference['arm_q']}));return
    from ial_g1d.robot.client import G1DClient,G1DClientConfig
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    operation='open-grippers' if a.open_grippers_only else 'restore-initial'
    root=base/(operation+'-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));root.mkdir()
    client=G1DClient(G1DClientConfig(robot_ip=ROBOT_HOST,control_mode='dual_arm',enable_agv=False))
    readers=[];source=None;receiver=XRReceiver();session=None;error=None;final=None;final_grippers=None
    try:
        client.connect(command_socket=False);status=client.status();validate_ready_status(status)
        ChannelFactoryInitialize(0,DDS_INTERFACE)
        readers=[LatestDex1Subscriber(side=s).start() for s in ['left','right']]
        source=LatestArmState().start();view=DirectFeedbackClient(client,source)
        deadline=time.monotonic()+3.
        while True:
            try:state=read_state(view);raw=[r.snapshot() for r in readers];break
            except Exception:
                if time.monotonic()>deadline:raise
                time.sleep(.02)
        limits=parse_urdf_limits(URDF.read_text())
        session=Session(view,root,limits,execute=True);session.grippers=readers
        session.saved_arm_q=tuple(reference['arm_q'])
        signal.signal(signal.SIGINT,lambda *_:session.abort.set())
        signal.signal(signal.SIGTERM,lambda *_:session.abort.set())
        receiver.start();session.receiver=receiver;session.open_sender()
        try:frame=receiver.snapshot()
        except VRControlError:pass
        else:
            session.b_press_baseline=frame['b_press_count'];session.check_abort_button(frame)
        validate_ready_status(view.status());state=read_state(view)
        core=SiteDualVRControl(status,state,limits,now_ns=time.monotonic_ns(),activation_mode='latched')
        session.policy_attempted=True;response=view.request_safety('start_policy')
        if response.get('success') is not True or response.get('policy_active') is not True:
            raise VRControlError('Normal return policy gate was not acknowledged')
        state=read_state(view);now=time.monotonic_ns();raw=[r.snapshot() for r in readers]
        # An explicit arm return holds both grippers; it is not a grasp/contact
        # update. Original fresh, finite, healthy DDS feedback is acquired above;
        # retain the mechanical command bounds even for endpoint sensor bias.
        first=DualVRDecision('active',False,body_q=core.body_q,
            arm_q=tuple(state.q[15:29]),arm_velocity=(0.,)*14,
            grippers=tuple(max(.28,min(5.38,r.q)) for r in raw),
            mode_machine=state.mode_machine,first_target=True)
        session.send(first)
        if a.open_grippers_only:
            final_grippers=session.open_grippers(core,first,state.q[15:29])
            final=read_state(view);session.report('grippers_opened')
        else:
            session.report('returning_saved_pose');session.return_saved(core,first)
            final_grippers=session.final_gripper_positions
            final=read_state(view);session.report('returned_saved_pose')
    except Exception as exc:error=f'{type(exc).__name__}: {exc}'
    finally:
        if session:
            try:
                pause=session.pause_once()
                if pause is not None and (pause.get('success') is not True or pause.get('ready_hold_active') is not True):
                    raise VRControlError('Normal return pause did not confirm READY hold')
            except Exception as exc:error=(error or '')+f'; pause: {exc}'
        receiver.stop_event.set()
        for r in readers:r.close()
        if source:source.close()
        if session and session.context is not None:session.context.term()
        client.close(require_safe_exit=False)
        receipt={'completed':error is None,'error':error,'directory':str(root),
            'operation':operation,'final_gripper_positions':final_grippers,
            'grippers_opened':bool(session and session.grippers_opened),
            'body_height_motion_requested':False,'recording_started':False,
            'commands_sent':0 if session is None else session.sent,
            'final_max_arm_error_rad':None if final is None else max(abs(a-b) for a,b in zip(final.q[15:29],reference['arm_q']))}
        if session:
            session.report('completed' if error is None else 'fault_hold',error=error,
                           returned_to_initial=error is None and not a.open_grippers_only,purpose=operation)
        (root/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        meta_path=base/'current.json'
        if meta_path.exists():
            meta=json.loads(meta_path.read_text())
            if '/vr-session-' in meta.get('control_directory',''):
                meta['last_vr_control_directory']=meta['control_directory']
            meta.update(control_directory=str(root),control_pid=None,
                robot_control_enabled=False,recording=False,
                control_purpose=operation,operation_receipt=str(root/'receipt.json'))
            if not a.open_grippers_only:meta['restore_receipt']=str(root/'receipt.json')
            meta_path.write_text(json.dumps(meta,indent=2)+'\n')
        if session:
            with (root/'control-trace.jsonl').open('x') as f:
                for row in session.trace:f.write(json.dumps(row)+'\n')
        print(json.dumps(receipt),flush=True)
    if error:raise SystemExit(1)


if __name__=='__main__':main()
