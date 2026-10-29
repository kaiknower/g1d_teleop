"""Pure tests of the teleoperation profile; no robot/network imports."""
from dataclasses import replace
import unittest
from g1d_vr_teleop.control.profile import SiteDualVRControl
from g1d_vr_teleop.core.dual_arm import VRControlError
from tests.test_vr_dual_control import BASE, LIMITS, status, state, xr, raw


class SiteProfileTests(unittest.TestCase):
    def core(self):
        return SiteDualVRControl(status(), state(), LIMITS, now_ns=BASE)

    def step(self, core, offset=0, *, goal=None, measured=None, value=None, left=None, right=None):
        ns=BASE+offset
        value=value or replace(xr(ns, offset+1),session_enabled=True)
        return core.step(state(ns, measured),value,goal or (1.,)*14,
                         left or raw('left',ns,offset+1),right or raw('right',ns,offset+1),now_ns=ns)

    def test_first_command_is_measured_and_body_is_preserved(self):
        core=self.core();d=self.step(core,20_000_000,measured=(.01,)*14)
        self.assertTrue(d.first_target)
        self.assertEqual(d.arm_q,(.01,)*14)
        self.assertEqual(d.body_q[12],-.016)
        self.assertEqual(d.body_q[13],0.)
        self.assertEqual(d.body_q[14],0.)

    def test_original_global_lead_clip_and_five_second_ramp(self):
        core=self.core();self.step(core)
        goal=(1.,-.5)+(.25,)*12
        d=self.step(core,20_000_000,goal=goal)
        lead=(20.+10.*.02/5.)/250.
        self.assertAlmostEqual(d.arm_q[0],lead)
        self.assertAlmostEqual(d.arm_q[1],-.5*lead)
        self.assertAlmostEqual(d.arm_q[2],.25*lead)
        for offset in range(120_000_000,5_020_000_001,100_000_000):
            d=self.step(core,offset,goal=goal)
        self.assertAlmostEqual(d.arm_q[0],.12)
        self.assertAlmostEqual(core.current_lead_limit_rad,.12)
        self.assertGreater(d.arm_q[0],.10)

    def test_targets_inside_lead_are_not_rescaled(self):
        core=self.core();self.step(core)
        d=self.step(core,20_000_000,goal=(.03,)*14)
        self.assertEqual(d.arm_q,(.03,)*14)

    def test_no_experimental_workspace_or_joint_margin(self):
        core=self.core()
        value=replace(xr(),session_enabled=True,left_translation=(.5,0.,0.),right_rotvec=(0.,0.,1.))
        self.step(core,value=value,goal=(2.,)*14)
        d=self.step(core,20_000_000,measured=(1.99,)*14,goal=(2.,)*14)
        self.assertEqual(d.arm_q,(2.,)*14)
        with self.assertRaisesRegex(VRControlError,'URDF'):
            self.step(core,40_000_000,goal=(2.001,)*14)

    def test_stale_xr_and_fault_latch(self):
        core=self.core();self.step(core)
        value=replace(xr(BASE-300_000_000,2),session_enabled=True)
        with self.assertRaisesRegex(VRControlError,'stale'):
            self.step(core,20_000_000,value=value)
        with self.assertRaisesRegex(VRControlError,'latched'):
            self.step(core,40_000_000)

    def test_each_raw_side_is_required_and_lost_faults(self):
        for side in ('left','right'):
            core=self.core();self.step(core)
            ns=BASE+20_000_000
            with self.assertRaises(VRControlError):
                self.step(core,20_000_000,**{side:replace(raw(side,ns,2),lost=1)})
        core=self.core()
        with self.assertRaises(VRControlError):
            self.step(core,right=raw('left'))

    def test_mode_body_and_stale_state_guards(self):
        for kind in ('mode','body','stale'):
            core=self.core();self.step(core);ns=BASE+20_000_000;s=state(ns)
            if kind=='mode':s=replace(s,mode_machine=2)
            if kind=='body':
                q=list(s.q);q[12]+=.02;s=replace(s,q=tuple(q))
            if kind=='stale':s=replace(s,received_monotonic_ns=BASE-300_000_000)
            with self.assertRaises(VRControlError):
                core.step(s,replace(xr(ns,2),session_enabled=True),(1.,)*14,
                          raw('left',ns,2),raw('right',ns,2),now_ns=ns)

    def test_explicit_end_cannot_resume(self):
        core=self.core();self.step(core)
        d=self.step(core,20_000_000,value=replace(xr(BASE+20_000_000,2),session_enabled=False))
        self.assertTrue(d.pause_required)
        self.assertTrue(self.step(core,40_000_000).pause_required)

    def test_bad_or_backwards_xr_and_command_gap_rejected(self):
        for kind in ('nonfinite','backwards','gap'):
            core=self.core();self.step(core);offset=20_000_000
            value=replace(xr(BASE+offset,2),session_enabled=True)
            if kind=='nonfinite':value=replace(value,left_translation=(float('nan'),0.,0.))
            if kind=='backwards':value=replace(value,received_monotonic_ns=BASE-1)
            if kind=='gap':offset=500_000_001;value=replace(value,received_monotonic_ns=BASE+offset)
            with self.assertRaises(VRControlError):self.step(core,offset,value=value)

    def test_site_scheduler_gap_uses_bridge_watchdog_with_fresh_inputs(self):
        core=self.core();self.step(core)
        decision=self.step(core,250_000_000)
        self.assertFalse(decision.pause_required)
        self.assertAlmostEqual(max(map(abs,decision.arm_q)),(20.+10.*.25/5.)/250.)
        self.assertEqual(core.max_command_gap_ns,500_000_000)
        self.assertEqual(core.profile_metadata['command_timeout_s'],.5)
        with self.assertRaisesRegex(VRControlError,r'gap_s=0.500000, limit_s=0.5'):
            self.step(core,750_000_001)

    def test_site_uses_explicit_shorter_bridge_timeout(self):
        ready=status();ready['bridge']['command_timeout_s']=.2
        core=SiteDualVRControl(ready,state(),LIMITS,now_ns=BASE)
        self.step(core)
        self.assertEqual(core.max_command_gap_ns,200_000_000)
        with self.assertRaisesRegex(VRControlError,r'limit_s=0.2'):
            self.step(core,250_000_000)

    def test_invalid_bridge_timeouts_are_rejected(self):
        for timeout in (0,-1,.50001,float('nan'),float('inf'),True,'0.5',None):
            ready=status();ready['bridge']['command_timeout_s']=timeout
            with self.subTest(timeout=timeout),self.assertRaises(VRControlError):
                SiteDualVRControl(ready,state(),LIMITS,now_ns=BASE)

    def test_generic_trajectory_still_rejects_gap_over_150ms(self):
        from g1d_vr_teleop.core.dual_arm import DualVRControl
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,activation_mode='latched')
        self.step(core)
        with self.assertRaisesRegex(VRControlError,'0.15 s'):
            self.step(core,250_000_000)

    def test_transient_feedback_holds_gripper_while_arms_continue_then_resumes(self):
        core=self.core();self.step(core)
        ns=BASE+20_000_000
        close=replace(xr(ns,2),session_enabled=True,desired_grippers=(.28,.28))
        old_left=raw('left',ns,2);old_right=raw('right',ns,2)
        previous=self.step(core,20_000_000,value=close,left=old_left,right=old_right)
        held=self.step(core,170_000_000,left=old_left,right=old_right)
        self.assertEqual(held.grippers,previous.grippers)
        self.assertGreater(held.arm_q[0],previous.arm_q[0])
        for evidence in held.contact_evidence:
            self.assertEqual(evidence['phase'],'feedback_wait')
            self.assertEqual(evidence['feedback_stamp_ns'],ns)
            self.assertFalse(evidence['accepted_new_feedback'])
            self.assertFalse(evidence['contact_candidate'])
            self.assertFalse(evidence['grasp_success'])
            self.assertFalse(evidence['lift_allowed'])
            self.assertEqual(evidence['closing_advance_raw'],0.)
        self.assertEqual(core._grippers[0]._last_stamp_ns,ns)
        resumed_ns=BASE+190_000_000
        close=replace(close,sequence=3,received_monotonic_ns=resumed_ns)
        # Previous test helper XR sequence at170ms is larger; retain monotonic
        # receiver sequence without changing the original DDS stamp.
        close=replace(close,sequence=190_000_001)
        resumed=self.step(core,190_000_000,value=close,
            left=replace(raw('left',resumed_ns,3),q=5.2),
            right=replace(raw('right',resumed_ns,3),q=5.2))
        for old,new,evidence in zip(held.grippers,resumed.grippers,resumed.contact_evidence):
            self.assertLessEqual(old-new,core._grippers[0].closing_speed_raw_s*.1+1e-12)
            self.assertTrue(evidence['accepted_new_feedback'])
            self.assertTrue(evidence['observation_gap_reset_stall'])

    def test_feedback_outage_over_half_second_faults_but_first_requires_fresh(self):
        core=self.core();self.step(core)
        for offset in range(100_000_000,500_000_001,100_000_000):
            self.step(core,offset,left=raw('left'),right=raw('right'))
        with self.assertRaisesRegex(VRControlError,'stale'):
            self.step(core,500_000_001,left=raw('left'),right=raw('right'))
        with self.assertRaisesRegex(VRControlError,'latched'):
            self.step(core,520_000_000)
        core=self.core()
        with self.assertRaisesRegex(VRControlError,'First Dex1'):
            self.step(core,150_000_000,left=raw('left'),right=raw('right'))

    def test_delayed_feedback_still_rejects_invalid_range_and_default_core_stays_strict(self):
        core=self.core();self.step(core)
        with self.assertRaises(VRControlError):
            self.step(core,150_000_000,left=replace(raw('left'),q=9.))
        from g1d_vr_teleop.core.dual_arm import DualVRControl
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,activation_mode='latched')
        self.step(core)
        with self.assertRaisesRegex(VRControlError,'stale'):
            self.step(core,150_000_000,left=raw('left'),right=raw('right'))

    def test_site_encoder_tolerance_is_separate_from_command_limits(self):
        for low in (.18,.26839295):
            core=self.core()
            initial=self.step(core,left=replace(raw('left'),q=low),
                              right=replace(raw('right'),q=5.3894))
            self.assertEqual(initial.grippers,(.28,5.38))
            ns=BASE+20_000_000
            opened=self.step(core,20_000_000,left=replace(raw('left',ns,2),q=low),
                             right=replace(raw('right',ns,2),q=5.3894))
            self.assertGreater(opened.grippers[0],.28)
            self.assertTrue(all(.28<=q<=5.38 for q in opened.grippers))
        core=self.core()
        with self.assertRaisesRegex(VRControlError,r'rt/dex1/left/state.*0.179.*0.18,5.48'):
            self.step(core,left=replace(raw('left'),q=.179))
        from g1d_vr_teleop.core.dual_arm import DualVRControl
        core=DualVRControl(status(),state(),LIMITS,now_ns=BASE,activation_mode='latched')
        with self.assertRaisesRegex(VRControlError,r'rt/dex1/left/state.*0.28,5.48'):
            self.step(core,left=replace(raw('left'),q=.26839295))


if __name__=='__main__':unittest.main()
