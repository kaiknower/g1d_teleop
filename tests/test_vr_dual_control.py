"""CPU-only tests of the command and saved-pose boundaries."""
from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from g1d_vr_teleop.core.dual_arm import (
    DualArmState,DualVRControl,XRInput,VRControlError,build_lowcmd,build_return_trajectory)
from g1d_vr_teleop.core.feedback import RawDex1Feedback


def status():
    return dict(success=True,protocol_version=3,bridge_implementation='ial_g1d',
        control_mode='dual_arm',lifecycle='ready',ready_hold_active=True,policy_active=False,
        pause_policy_supported=True,teleop_home_mode=False,column_lowered=True,last_error=None,
        hold_error=None,active_sequence=None,capabilities=['upper_body','robot_state'],
        body_alignment={'state':'never_started','failure_latched':False},invalid_motor_slots=[13],
        body_hold_target_rad={'waist_yaw':-.016,'waist_pitch':0.},
        bridge={'control_activated':True,'dry_run':False,'shutdown_requested':False,
                'activation':{'upper_body_active':True}})


LIMITS=((-2.,2.,3.),)*14
BASE=10_000_000_000


def state(ns=BASE,arm=None):
    q=[0.]*29;q[12]=-.016;q[13]=float('nan')
    q[15:29]=arm or [0.]*14
    return DualArmState(tuple(q),ns,1)


def xr(ns=BASE,seq=1):
    return XRInput(seq,ns,(0.,)*3,(0.,)*3,(0.,)*3,(0.,)*3,True,True,(5.38,5.38))


def raw(side,ns=BASE,seq=1):
    return RawDex1Feedback(seq,ns,5.38,0.,0.,1,0,topic=f'rt/dex1/{side}/state')


class CoreTests(unittest.TestCase):
    def step(self,core,ns=BASE,seq=1,goal=None,measured=None,input=None):
        return core.step(state(ns,measured),input or xr(ns,seq),goal or (0.,)*14,
                         raw('left',ns,seq),raw('right',ns,seq),now_ns=ns)

    def test_first_command_is_measured_and_invalid13_is_neutralized(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE)
        decision=self.step(core,goal=(1.,)*14)
        self.assertEqual(decision.arm_q,(0.,)*14)
        self.assertTrue(decision.first_target)
        def builder(q,mode):
            return {'motor_cmd':[dict(q=x) for x in q]}
        packet=build_lowcmd(decision,make_lowcmd=builder)
        self.assertEqual(packet['motor_cmd'][13],dict(mode=0,q=0.,dq=0.,tau=0.,kp=0.,kd=0.))
        self.assertEqual(packet['motor_cmd'][12]['q'],-.016)
        self.assertEqual(packet['motor_cmd'][15]['kp'],80.)
        self.assertEqual(packet['motor_cmd'][19]['kp'],40.)
        self.assertTrue(all(m['tau']==0 for m in packet['motor_cmd']))

    def test_slew_and_acceleration_bound_even_when_goal_reverses(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE)
        prior=self.step(core)
        for i in range(1,300):
            goal=(.2 if i<150 else -.2,)*14
            decision=self.step(core,BASE+i*20_000_000,i+1,goal,prior.arm_q)
            self.assertLessEqual(max(map(abs,decision.arm_velocity)),.04+1e-12)
            self.assertLessEqual(max(abs(a-b) for a,b in zip(prior.arm_velocity,decision.arm_velocity)),.001+1e-12)
            prior=decision

    def test_release_latches_end_and_cannot_resume(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE)
        self.step(core)
        released=replace(xr(BASE+20_000_000,2),left_squeeze=False)
        self.assertTrue(self.step(core,BASE+20_000_000,2,input=released).pause_required)
        self.assertTrue(self.step(core,BASE+40_000_000,3).pause_required)

    def test_latched_mode_does_not_require_grips_but_cannot_resume_after_stop(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,activation_mode='latched')
        value=replace(xr(),left_squeeze=False,right_squeeze=False,session_enabled=True)
        self.assertFalse(self.step(core,input=value).pause_required)
        value=replace(value,sequence=2,received_monotonic_ns=BASE+20_000_000,session_enabled=False)
        self.assertTrue(self.step(core,BASE+20_000_000,2,input=value).pause_required)
        value=replace(value,sequence=3,received_monotonic_ns=BASE+40_000_000,session_enabled=True)
        self.assertTrue(self.step(core,BASE+40_000_000,3,input=value).pause_required)

    def test_requested_faster_speed_still_has_acceleration_and_tracking_bounds(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,
                           velocity_limit=.15,acceleration_limit=.20)
        previous=self.step(core)
        largest=0.
        for i in range(1,100):
            current=self.step(core,BASE+i*20_000_000,i+1,(.5,)*14,previous.arm_q)
            largest=max(largest,max(map(abs,current.arm_velocity)))
            self.assertLessEqual(max(map(abs,current.arm_velocity)),.15+1e-12)
            self.assertLessEqual(max(abs(a-b) for a,b in zip(current.arm_velocity,previous.arm_velocity)),.004+1e-12)
            previous=current
        self.assertGreater(largest,.10)

    def test_explicit_measured_pose_start_does_not_require_column_motion(self):
        config=status();config.update(teleop_home_mode=True,column_lowered=False,teleop_home_mode_supported=True)
        core=DualVRControl(config,state(),LIMITS,now_ns=BASE)
        self.assertEqual(self.step(core).arm_q,(0.,)*14)

    def test_feedback_governor_handles_delayed_measurement_without_raising_tracking_limit(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,
                           velocity_limit=.15,acceleration_limit=.20)
        previous=self.step(core);history=[previous.arm_q]*25
        max_error=0.
        for i in range(1,500):
            # Half-second transport/plant lag: feedback is not replaced with
            # the current command. The command must slow to measured progress.
            measured=history[-25]
            result=self.step(core,BASE+i*20_000_000,i+1,(.7,)*14,measured)
            max_error=max(max_error,max(abs(a-b) for a,b in zip(result.arm_q,measured)))
            self.assertLessEqual(max(abs(a-b) for a,b in zip(result.arm_velocity,previous.arm_velocity)),.004+1e-12)
            history.append(result.arm_q);previous=result
        self.assertLess(max_error,.08)
        self.assertGreater(previous.arm_q[0],.2)

    def test_explicit_010_tracking_limit_remains_a_hard_bound(self):
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,tracking_error_limit=.10)
        self.step(core)
        moved=[0.]*14;moved[0]=.095
        self.step(core,BASE+20_000_000,2,measured=moved)
        moved[0]=.101
        with self.assertRaisesRegex(VRControlError,'0.1 rad'):
            self.step(core,BASE+40_000_000,3,measured=moved)
        with self.assertRaises(VRControlError):
            DualVRControl(status(),state(),LIMITS,now_ns=BASE,tracking_error_limit=.11)

    def test_stale_state_raw_feedback_and_body_motion_fault(self):
        for kind in ('state','raw','body','tracking'):
            core=DualVRControl(status(),state(),LIMITS,now_ns=BASE)
            self.step(core)
            ns=BASE+20_000_000
            s=state(ns)
            l=raw('left',ns,2)
            if kind=='state':s=replace(s,received_monotonic_ns=BASE-300_000_000)
            if kind=='raw':l=raw('left',BASE-100_000_001,2)
            if kind in ('body','tracking'):
                q=list(s.q);q[12 if kind=='body' else 15]+=.1;s=replace(s,q=tuple(q))
            with self.assertRaises(VRControlError):
                core.step(s,xr(ns,2),(0.,)*14,l,raw('right',ns,2),now_ns=ns)
            with self.assertRaises(VRControlError):self.step(core,ns+20_000_000,3)

    def test_measured_return_start_can_move_inward_from_planning_margin(self):
        start=(LIMITS[0][1]-.019,)+(0.,)*13
        with self.assertRaisesRegex(VRControlError,'endpoint'):
            build_return_trajectory(start,(0.,)*14,LIMITS)
        plan=build_return_trajectory(start,(0.,)*14,LIMITS,start_joint_margin=0.)
        samples=[plan.sample(plan.duration_s*i/100) for i in range(101)]
        self.assertEqual(samples[0].q,start)
        self.assertEqual(samples[-1].q,(0.,)*14)
        self.assertTrue(all(0.<=s.q[0]<=start[0] for s in samples))
        outside=(LIMITS[0][1]+.001,)+(0.,)*13
        with self.assertRaisesRegex(VRControlError,'endpoint'):
            build_return_trajectory(outside,(0.,)*14,LIMITS,start_joint_margin=0.)
        with self.assertRaisesRegex(VRControlError,'endpoint'):
            build_return_trajectory((0.,)*14,start,LIMITS,start_joint_margin=0.)

    def test_return_has_exact_saved_endpoint_and_continuous_bounded_derivatives(self):
        plan=build_return_trajectory((.3,)*14,(0.,)*14,LIMITS)
        for i in range(1001):
            sample=plan.sample(plan.duration_s*i/1000)
            self.assertLessEqual(max(map(abs,sample.velocity)),.04+1e-10)
            self.assertLessEqual(max(map(abs,sample.acceleration)),.05+1e-10)
        end=plan.sample(plan.duration_s+1)
        self.assertEqual(end.q,(0.,)*14)
        self.assertEqual(end.velocity,(0.,)*14)
        self.assertEqual(end.acceleration,(0.,)*14)


if __name__=='__main__':unittest.main()
