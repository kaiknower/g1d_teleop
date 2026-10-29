"""Reusable entry: A starts at current pose, 60 seconds, saved pose on exit."""

from g1d_vr_teleop.config import RECORD_ROOT, PYTHON, XR_ROOT, ROBOT_HOST, STATE_ROOT, child_environment
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import subprocess
import re
import fcntl
import time
from contextlib import contextmanager

from g1d_vr_teleop.core.dual_arm import validate_ready_status


BASE=STATE_ROOT


def is_manager_command(argv):
    """Recognize only supported manager entrypoints, not another command."""
    if not argv:
        return False
    executable=Path(argv[0]).name
    if executable == 'g1d-vr':
        return argv[1:2] == ['run']
    if not re.fullmatch(r'(?:python|pypy)(?:\d+(?:\.\d+)*)?(?:\.exe)?',executable):
        return False
    tail=argv[1:]
    while tail and tail[0] in ('-u','-B','-E','-I','-O','-OO','-s','-S','-q'):
        tail=tail[1:]
    if tail[:1] == ['-m']:
        return (tail[1:2] == ['g1d_vr_teleop.control.manager'] or
                tail[1:3] == ['g1d_vr_teleop','run'])
    return len(tail)>1 and Path(tail[0]).name == 'g1d-vr' and tail[1] == 'run'


def require_ready():
    from ial_g1d.robot.client import G1DClient,G1DClientConfig
    c=G1DClient(G1DClientConfig(robot_ip=ROBOT_HOST,control_mode='dual_arm',enable_agv=False))
    try:c.connect(command_socket=False);validate_ready_status(c.status())
    finally:c.close(require_safe_exit=False)


def write_json_atomic(path, value):
    tmp=path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(value,indent=2)+'\n')
    tmp.replace(path)


@contextmanager
def _manager_guard(base, manager_pid):
    with (base/'manager-owner.lock').open('a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            if manager_pid != os.getpid():
                raise RuntimeError('VR session manager owns this controller; manual parallel launch refused')
            cmdline=Path(f'/proc/{manager_pid}/cmdline').read_bytes()
            argv=[part.decode() for part in cmdline.split(b'\0') if part]
            if not is_manager_command(argv):
                raise RuntimeError('Manager PID does not identify the VR session manager')
            lock.seek(0)
            if lock.read().strip()!=str(manager_pid):
                raise RuntimeError('VR manager lock owner identity does not match')
        else:
            if manager_pid is not None:
                raise RuntimeError('Manager launch requires an already-held manager lock')
        yield


def current_boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def launch_session(execute=False, manager_pid=None):
    """Return launch metadata and its one child (None for read-only planning)."""
    base=BASE
    base.mkdir(parents=True,exist_ok=True)
    # Serialize a manual launch racing with manager startup. The manager lock
    # remains held throughout its lifetime; this lock spans each child launch.
    with (base/'launch-owner.lock').open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with _manager_guard(base,manager_pid):
            return _launch_session(base,execute,manager_pid)


def _launch_session(base,execute,manager_pid):
    meta=json.loads((base/'current.json').read_text()) if (base/'current.json').exists() else {}
    old=meta.get('control_pid')
    if old:
        try:os.kill(old,0)
        except ProcessLookupError:pass
        else:raise RuntimeError('Existing control process is still running; no competing sender is permitted')
    require_ready()
    if not (base/'initial-pose.json').exists():
        raise RuntimeError('A reviewed fixed initial-pose reference is required')
    if json.loads((base/'initial-pose.json').read_text()).get('boot_id')!=current_boot_id():
        raise RuntimeError('Fixed initial-pose reference belongs to another robot boot')
    suffix=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    session=base/('vr-session-'+suffix)
    recording=RECORD_ROOT/('vr-dual-60s-'+suffix)
    argv=[PYTHON,'-m','g1d_vr_teleop.control.session',
          '--directory',str(session),'--execute','--duration','60','--record-output',str(recording)]
    if not execute:
        return {'argv':argv,'motion_commands_sent':0},None
    session.mkdir();recording.mkdir(parents=True)
    write_json_atomic(session/'control-status.json',{
        'phase':'preparing_next_episode','robot_control_enabled':False,
        'commands_sent':0,'recording':False,'recording_directory':str(recording),
        'monotonic_s':time.monotonic(),'error':None})
    env=child_environment(state_root=base)
    with (session/'control.log').open('x') as log:
        proc=subprocess.Popen(argv,env=env,
            cwd=str(XR_ROOT/'teleop'),
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    meta.update(control_pid=proc.pid,control_argv=argv,control_directory=str(session),
        control_purpose='dual_arm_vr_teleoperation',
        recording_directory=str(recording),start_mode='right_A_click',controller_grip_required=False,
        requested_duration_s=60,normal_return_joint_velocity_limit_rad_s=.10,
        normal_return_joint_acceleration_limit_rad_s2=.25,
        teleop_profile='measured_position_lead',mapping_scale=1.,
        experimental_workspace_limit=False,takeover_blend_s=2.,
        gripper_mode='manual_endpoint_hold',gripper_closing_speed_raw_s=6.4,
        finish_mode='right_A_hold_1s_or_60s',
        restore_before_start=False,normal_exit_pose_tolerance_rad=.05,
        fixed_initial_pose=str(base/'initial-pose.json'),robot_control_enabled=False,recording=False,
        normal_end='stop task recording, return initial arms, open both grippers, pause READY')
    meta.update(manager_pid=manager_pid,manager_phase='child_waiting_for_operator' if manager_pid else 'manual_session')
    meta.pop('joint_velocity_limit_rad_s',None)
    meta.pop('joint_acceleration_limit_rad_s2',None)
    write_json_atomic(base/'current.json',meta)
    return meta,proc
