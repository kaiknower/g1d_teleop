"""Bind a new boot's completed standard startup pose; never sends motion."""

from g1d_vr_teleop.config import ROBOT_HOST, DDS_INTERFACE, STATE_ROOT
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import shutil
import time


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--startup-receipt',type=Path,required=True)
    p.add_argument('--execute',action='store_true');a=p.parse_args(argv)
    base=STATE_ROOT;boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    receipt=json.loads(a.startup_receipt.read_text())
    if (receipt.get('boot_id')!=boot or receipt.get('response',{}).get('success') is not True
            or receipt['response'].get('lifecycle')!='ready'):
        raise RuntimeError('A completed standard startup receipt for this boot is required')
    from ial_g1d.robot.client import G1DClient,G1DClientConfig
    from g1d_vr_teleop.core.dual_arm import validate_ready_status
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from g1d_vr_teleop.feedback.arms import LatestArmState,ArmStateUnavailable
    c=G1DClient(G1DClientConfig(robot_ip=ROBOT_HOST,control_mode='dual_arm',enable_agv=False))
    source=None
    try:
        c.connect(command_socket=False);status=c.status();validate_ready_status(status)
        if status['teleop_home_mode'] or not status['column_lowered']:
            raise RuntimeError('Standard raised-arm/lowered-body posture is required')
        ChannelFactoryInitialize(0,DDS_INTERFACE);source=LatestArmState().start();deadline=time.monotonic()+3.
        while True:
            try:sample=source.snapshot();break
            except ArmStateUnavailable:
                if time.monotonic()>deadline:raise
                time.sleep(.01)
        arm=sample.q[15:29]
        if max(map(abs,arm))>.05:
            raise RuntimeError('Measured arms have not reached the standard startup endpoint')
        result={'boot_id':boot,'arm_q':arm,'body_hold_target_rad':status['body_hold_target_rad'],
            'source':'completed standard startup, original latest-only DDS sample',
            'startup_receipt':str(a.startup_receipt.resolve()),
            'created_utc':datetime.now(timezone.utc).isoformat(),
            'start':'current measured pose; no initial restoration',
            'normal_exit':'return to this fixed arm pose; preserve body height'}
        if a.execute:
            base.mkdir(parents=True,exist_ok=True)
            path=base/'initial-pose.json'
            if path.exists():
                old=json.loads(path.read_text())
                if old.get('boot_id')==boot:
                    print(json.dumps({'existing_reference_preserved':True,'path':str(path)}));return
                backup=base/('initial-pose-boot-'+old['boot_id']+'.json')
                if not backup.exists():shutil.copy2(path,backup)
            tmp=base/'initial-pose.tmp';tmp.write_text(json.dumps(result,indent=2)+'\n');tmp.replace(path)
        print(json.dumps({'saved':a.execute,'reference':result}),flush=True)
    finally:
        if source:source.close()
        c.close(require_safe_exit=False)


if __name__=='__main__':main()
