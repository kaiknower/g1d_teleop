"""Explicit recovery of ONLY a stopped policy-watchdog session.

Reuse the site's supported abort_hold -> measured-pose startup. No column or
arm waypoint sequence; never edits private controller fields or restarts DDS.
Unrelated errors, unknown ownership and shutdown remain blocked. Never retries.
"""
from g1d_vr_teleop.config import ROBOT_HOST
import argparse
import json
from pathlib import Path
import time
from g1d_vr_teleop.control.session import read_state
from g1d_vr_teleop.core.dual_arm import validate_ready_status


def rpc(request):
    import zmq
    ctx=zmq.Context();sock=ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER,0)
    sock.setsockopt(zmq.SNDTIMEO,2000);sock.setsockopt(zmq.RCVTIMEO,2000)
    sock.connect(f'tcp://{ROBOT_HOST}:6002')
    try:
        sock.send_json(request);result=sock.recv_json()
        if result.get('success') is not True or result.get('protocol_version')!=3:
            raise RuntimeError(f'Public lifecycle request failed: {result}')
        return result
    finally:sock.close(0);ctx.term()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true');a=p.parse_args(argv)
    from ial_g1d.robot.client import G1DClient,G1DClientConfig
    if a.output.exists():raise FileExistsError(a.output)
    c=G1DClient(G1DClientConfig(robot_ip=ROBOT_HOST,control_mode='dual_arm',enable_agv=False,max_state_age_s=.2))
    receipt={'automatic_retry':False,'execute':a.execute,'requests':[]}
    try:
        c.connect(command_socket=False);before=c.status();state=read_state(c)
        receipt.update(before=before,initial_q=state.q)
        backend=before.get('bridge',{})
        if (before.get('lifecycle')!='ready' or before.get('policy_active') is not False
                or before.get('ready_hold_active') is not True or before.get('hold_error')
                or not str(before.get('last_error')).startswith('policy command stream timed out')
                or before.get('body_alignment',{}).get('failure_latched')
                or before.get('abort_hold_supported') is not True
                or before.get('teleop_home_mode_supported') is not True
                or backend.get('shutdown_requested') or backend.get('shutdown_error')
                or backend.get('control_activated') is not True):
            raise RuntimeError('Not the supported stopped-policy watchdog state; no recovery attempted')
        if not a.execute:
            receipt['status']='dry_run_supported_hold_recovery'
            return
        for request in ({'command':'abort_hold'},
                {'command':'startup','teleop_home_mode':True,'control_mode':'dual_arm',
                 'max_safe_height_mm':400.,'lower_height_mm':400.,'column_speed':.35}):
            receipt['requests'].append({'request':request,'sent_monotonic_s':time.monotonic()})
            receipt['requests'][-1]['response']=rpc(request)
        after=c.status();validate_ready_status(after)
        receipt.update(after=after,final_q=read_state(c).q)
        before_count=backend['diagnostics']['forwarded'];after_count=after['bridge']['diagnostics']['forwarded']
        if after_count!=before_count:raise RuntimeError('Unexpected external policy commands during recovery')
        delta=max(abs(a-b) for i,(a,b) in enumerate(zip(state.q,receipt['final_q'])) if i!=13)
        receipt['maximum_valid_joint_change_rad']=delta
        if delta>.015:raise RuntimeError('Joint state changed unexpectedly during measured-pose recovery')
        # Exercise the exact existing public pause RPC while already READY.
        # It is idempotent here and leaves the persistent hold active.
        from g1d_vr_teleop.core.pause import pause_once
        receipt['pause_rpc_verified']=pause_once(ROBOT_HOST,6002)
        receipt['status']='completed_ready_hold_no_waypoints_or_column_requests'
    except Exception as exc:
        receipt.update(status='failed_or_unconfirmed',error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        c.close(require_safe_exit=False)
        a.output.write_text(json.dumps(receipt,indent=2)+'\n')
        print(json.dumps({'receipt':str(a.output),'status':receipt.get('status')}))


if __name__=='__main__':main()
