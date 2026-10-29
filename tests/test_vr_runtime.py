"""Numerical and mocked lifecycle checks. No DDS or robot calls."""
from pathlib import Path
import sys
import importlib.util
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from dataclasses import replace
import numpy as np
from g1d_vr_teleop.control.session import anchored_targets,robot_poses,rotation_angle,Session,StartButtonLatch,FinishButtonHold,XRReceiver,VRControlError,DirectFeedbackClient,SourceArmState,read_state,IKWorker
import time


class RuntimeTests(unittest.TestCase):
    def test_receiver_transport_shares_original_event_freshness_budget(self):
        receiver=XRReceiver()
        now=10_000_000_000
        frame=dict(pose_reference='controller_world',controller_monotonic_ns=now-180_000_000)
        receiver.latest=({'valid':True,'frame':frame},now-170_000_000)
        with patch('g1d_vr_teleop.control.session.time.monotonic_ns',return_value=now):
            self.assertIs(receiver.snapshot(),frame)
            receiver.latest=({'valid':True,'frame':frame},now-201_000_000)
            with self.assertRaisesRegex(VRControlError,'transport is stale'):
                receiver.snapshot()
            receiver.latest=({'valid':True,'frame':frame},now)
            frame['controller_monotonic_ns']=now-201_000_000
            with self.assertRaisesRegex(VRControlError,'controller events are stale'):
                receiver.snapshot()
            frame['controller_monotonic_ns']=now+1
            with self.assertRaisesRegex(VRControlError,'controller events are stale'):
                receiver.snapshot()

    def test_sender_uses_bridge_watchdog_not_extra_150ms_abort(self):
        session=Session(Mock(),Path('/unused'),(),execute=True)
        session.sender=Mock();session.last_sent=99.75;session.command_timeout_s=.5
        decision=SimpleNamespace(arm_q=(0.,)*14,arm_velocity=(0.,)*14)
        observed=SimpleNamespace(q=(0.,)*29,received_monotonic_ns=1)
        protocol=SimpleNamespace(encode_command_envelope=lambda *a,**k:b'packet',
                                 make_lowcmd=Mock(),make_gripper_command=Mock())
        with patch.dict(sys.modules,{'ial_g1d.robot.protocol':protocol}), \
             patch('g1d_vr_teleop.control.session.time.monotonic',return_value=100.), \
             patch('g1d_vr_teleop.control.session.read_state',return_value=observed), \
             patch('g1d_vr_teleop.control.session.build_payloads',return_value=[('rt/lowcmd',{})]):
            session.send(decision)
            session.sender.send.assert_called_once_with(b'packet')
            session.last_sent=99.49
            with self.assertRaisesRegex(VRControlError,'bridge watchdog'):
                session.send(decision)
            self.assertEqual(session.sent,1)

    def test_site_return_reanchors_measured_pose_without_waiting_for_static_pd_bias(self):
        from g1d_vr_teleop.core.dual_arm import DualArmState,DualVRDecision,build_return_trajectory
        now=[100.]
        q=[0.]*29;q[23]=.13065
        measured=list(q)
        previous_arm=[0.]*14;previous_arm[8]=.23456
        previous=DualVRDecision('active',False,body_q=tuple(q),arm_q=tuple(previous_arm),
            arm_velocity=(8.,)*14,grippers=(2.,3.),mode_machine=1)
        core=SimpleNamespace(site_profile=True,body_q=tuple(q),mode_machine=1,
                             velocity_limit=.20,acceleration_limit=.25,_check_arm=Mock())
        session=Session(Mock(),Path('/unused'),((-2.,2.,3.),)*14,execute=True)
        session.saved_arm_q=(0.,)*14
        session.grippers=[Mock(),Mock()]
        for reader in session.grippers:
            reader.snapshot.return_value=SimpleNamespace(q=5.38)
        session.report=Mock()
        session.abort=Mock()
        session.abort.wait.side_effect=lambda delay:now.__setitem__(0,now[0]+max(delay,.001))
        sent=[]
        def send(decision):
            sent.append(decision)
            measured[15:29]=decision.arm_q
        session.send=send
        def read(_):return DualArmState(tuple(measured),int(now[0]*1e9),1)
        with patch('g1d_vr_teleop.control.session.read_state',side_effect=read),\
             patch('g1d_vr_teleop.control.session.time.monotonic',side_effect=lambda:now[0]),\
             patch('g1d_vr_teleop.control.session.time.monotonic_ns',side_effect=lambda:int(now[0]*1e9)),\
             patch('g1d_vr_teleop.control.session.build_return_trajectory',wraps=build_return_trajectory) as build:
            session.return_saved(core,previous)
        self.assertEqual(sent[0].arm_q,tuple(q[15:29]))
        self.assertEqual(sent[0].arm_velocity,(0.,)*14)
        self.assertEqual(sent[0].grippers,(2.,3.))
        self.assertTrue(np.allclose(build.call_args.args[0],q[15:29]))
        self.assertEqual(build.call_args.kwargs['velocity_limit'],.10)
        self.assertEqual(build.call_args.kwargs['acceleration_limit'],.25)
        self.assertEqual(sent[-1].arm_q,(0.,)*14)
        self.assertLess(now[0]-100.,10.)
        self.assertNotIn('settling_before_return',[call.args[0] for call in session.report.call_args_list])
        self.assertTrue(all(max(map(abs,d.arm_velocity))<=.10+1e-9 for d in sent))
        self.assertTrue(all(reader.snapshot.called for reader in session.grippers))
        self.assertTrue(session.grippers_opened)
        self.assertEqual(sent[-1].grippers,(5.38,5.38))

    def test_normal_reset_opens_both_grippers_without_moving_arms_or_body(self):
        from g1d_vr_teleop.core.dual_arm import DualArmState,DualVRDecision
        now=[100.];positions=[.28,1.]
        q=(0.,)*29
        core=SimpleNamespace(body_q=q,mode_machine=1,_check_arm=Mock())
        previous=DualVRDecision('active',False,body_q=q,arm_q=(0.,)*14,
            arm_velocity=(0.,)*14,grippers=tuple(positions),mode_machine=1)
        session=Session(Mock(),Path('/unused'),(),execute=True)
        session.report=Mock();session.abort=Mock()
        session.abort.wait.side_effect=lambda delay:now.__setitem__(0,now[0]+max(delay,.001))
        session.grippers=[Mock(),Mock()]
        for side,reader in enumerate(session.grippers):
            reader.snapshot.side_effect=lambda side=side:SimpleNamespace(q=positions[side])
        sent=[]
        def send(decision):
            sent.append(decision);positions[:]=decision.grippers
        session.send=send
        with patch('g1d_vr_teleop.control.session.read_state',side_effect=lambda _:DualArmState(q,int(now[0]*1e9),1)),\
             patch('g1d_vr_teleop.control.session.time.monotonic',side_effect=lambda:now[0]),\
             patch('g1d_vr_teleop.control.session.time.monotonic_ns',side_effect=lambda:int(now[0]*1e9)):
            self.assertEqual(session.open_grippers(core,previous,(0.,)*14),(5.38,5.38))
        self.assertTrue(session.grippers_opened)
        self.assertTrue(all(d.body_q==q and d.arm_q==(0.,)*14 and d.arm_velocity==(0.,)*14 for d in sent))
        self.assertTrue(all(.28<=g<=5.38 for d in sent for g in d.grippers))

    def test_normal_reset_feedback_loss_does_not_open_or_claim_success(self):
        session=Session(Mock(),Path('/unused'),(),execute=True)
        session.report=Mock();session.send=Mock()
        session.grippers=[Mock(),Mock()]
        session.grippers[0].snapshot.side_effect=RuntimeError('fresh Dex1 feedback lost')
        with self.assertRaisesRegex(RuntimeError,'feedback lost'):
            session.open_grippers(Mock(),Mock(),(0.,)*14)
        session.send.assert_not_called()
        self.assertFalse(session.grippers_opened)

    def test_completed_ik_target_has_no_extra_computation_age_abort(self):
        worker=IKWorker(None,None,None,(),())
        worker.result=((0.,)*14,1,1)
        self.assertEqual(worker.snapshot(),(0.,)*14)
        worker.error='solver failed'
        with self.assertRaisesRegex(VRControlError,'solver failed'):
            worker.snapshot()

    def test_quick_a_confirmation_never_ends_and_new_continuous_hold_does(self):
        button=FinishButtonHold(5)
        def frame(seconds,count,pressed):
            return dict(controller_monotonic_ns=int(seconds*1e9),right_a_press_count=count,
                        right_state={'a_button':pressed})
        self.assertFalse(button.update(frame(1,5,True)))
        self.assertFalse(button.update(frame(2,6,True)))
        self.assertFalse(button.update(frame(2.1,6,False)))
        self.assertFalse(button.update(frame(3,7,True)))
        self.assertFalse(button.update(frame(3.99,7,True)))
        self.assertTrue(button.update(frame(4,7,True)))

    def test_unobserved_release_between_new_a_edges_resets_finish_hold(self):
        button=FinishButtonHold(2)
        def frame(seconds,count):
            return dict(controller_monotonic_ns=int(seconds*1e9),right_a_press_count=count,
                        right_state={'a_button':True})
        self.assertFalse(button.update(frame(1,3)))
        self.assertFalse(button.update(frame(1.8,4)))
        self.assertFalse(button.update(frame(2.1,4)))
        self.assertTrue(button.update(frame(2.8,4)))

    def test_control_reads_original_latest_dds_sample_and_propagates_staleness(self):
        rpc=Mock();source=Mock()
        sample=SimpleNamespace(q=(0.,)*29,received_monotonic_ns=100,mode_machine=5,
                               tick=12,source_timestamp_ns=300)
        source.snapshot.return_value=sample
        client=DirectFeedbackClient(rpc,source)
        state=read_state(client)
        self.assertIsInstance(state,SourceArmState)
        self.assertEqual((state.received_monotonic_ns,state.source_tick,state.source_timestamp_ns),(100,12,300))
        rpc.status.return_value={'lifecycle':'ready'}
        self.assertEqual(client.status(),{'lifecycle':'ready'})
        source.snapshot.side_effect=RuntimeError('raw state stale')
        with self.assertRaisesRegex(RuntimeError,'raw state stale'):
            read_state(client)

    @unittest.skipUnless(importlib.util.find_spec('pinocchio'), 'Optional IK numerical dependency: pinocchio')
    def test_head_motion_does_not_change_controller_world_targets(self):
        identity=np.eye(4)
        original={key:identity.flatten(order='F').tolist() for key in ('head','left','right')}
        initial=robot_poses(original)
        moved=dict(original)
        head=identity.copy();head[:3,3]=[1.,-2.,3.]
        moved['head']=head.flatten(order='F').tolist()
        targets,delta,_=anchored_targets(moved,initial,(identity,identity))
        self.assertTrue(all(np.allclose(t,identity) for t in targets))
        self.assertTrue(all(np.allclose(d,0.) for d in delta))

    def test_a_latch_requires_new_edge_but_catches_short_released_click(self):
        latch=StartButtonLatch()
        def frame(count,pressed=False,source=1):
            return dict(source_id=source,right_a_press_count=count,right_state={'a_button':pressed})
        self.assertFalse(latch.update(frame(3,True)))
        self.assertFalse(latch.update(frame(3,False)))
        self.assertTrue(latch.update(frame(4,False)))
        self.assertFalse(latch.update(frame(9,False,source=2)))
        self.assertTrue(latch.update(frame(10,True,source=2)))

    def test_world_receiver_preserves_controller_freshness_without_using_old_head(self):
        receiver=XRReceiver()
        now=time.monotonic_ns()
        frame=dict(pose_reference='controller_world',head_monotonic_ns=now-10_000_000_000,
                   controller_monotonic_ns=now)
        receiver.latest=({'valid':True,'frame':frame},now)
        self.assertIs(receiver.snapshot(),frame)
        frame['controller_monotonic_ns']=now-201_000_000
        with self.assertRaisesRegex(VRControlError,'controller events are stale'):
            receiver.snapshot()
        frame['controller_monotonic_ns']=time.monotonic_ns()
        frame['pose_reference']='head_relative'
        with self.assertRaisesRegex(VRControlError,'controller-world'):
            receiver.snapshot()

    def test_b_edge_remains_abort_even_after_release(self):
        session=Session(Mock(),Path('/unused'),(),execute=True)
        session.b_press_baseline=3
        frame=dict(b_press_count=4,left_state={'b_button':False},right_state={'b_button':False})
        with self.assertRaisesRegex(VRControlError,'B button abort'):
            session.check_abort_button(frame)

    @unittest.skipUnless(importlib.util.find_spec('pinocchio'), 'Optional IK numerical dependency: pinocchio')
    def test_site_mapping_has_original_amplitude_and_preserves_anchor(self):
        import pinocchio as pin
        identity=np.eye(4)
        original={key:identity.flatten(order='F').tolist() for key in ('head','left','right')}
        initial=robot_poses(original)
        moved=dict(original)
        pose=identity.copy();pose[:3,:3]=pin.exp3(np.array([1.2,0.,0.]));pose[0,3]=.8
        moved['left']=pose.flatten(order='F').tolist()
        targets,delta,rotvec=anchored_targets(moved,initial,(identity,identity))
        self.assertAlmostEqual(np.linalg.norm(delta[0]),.8)
        self.assertAlmostEqual(rotation_angle(targets[0][:3,:3]),1.2)
        self.assertTrue(np.allclose(targets[1],identity))
        again,_,_=anchored_targets(original,initial,(identity,identity))
        self.assertTrue(np.allclose(again[0],identity))

    def test_pause_uses_public_rpc_once_instead_of_incompatible_client(self):
        client=Mock();client.request_safety.side_effect=AssertionError('Old client must not receive pause_policy')
        pause=Mock(return_value={'success':True,'ready_hold_active':True})
        session=Session(client,Path('/unused'),(),execute=True)
        session.policy_attempted=True
        session.sender=Mock()
        with patch.dict(sys.modules,{'g1d_vr_teleop.core.pause':SimpleNamespace(pause_once=pause)}):
            self.assertTrue(session.pause_once()['success'])
            self.assertIsNone(session.pause_once())
        from g1d_vr_teleop.config import ROBOT_HOST
        pause.assert_called_once_with(ROBOT_HOST,6002)
        client.request_safety.assert_not_called()

    @unittest.skipUnless(importlib.util.find_spec('ial_g1d'), 'Optional external bridge compatibility test')
    def test_timeout_recovery_uses_supported_hold_and_no_motion_start(self):
        from ial_g1d.robot.advanced_safety import AdvancedSafetyController,RobotStateBuffer
        from ial_g1d.robot.protocol import RobotState,MotorState
        buffer=RobotStateBuffer()
        buffer.update(RobotState(tuple(MotorState(0.,0.,0.,30.) for _ in range(35)),{},b'',1,5.38,5.38))
        column=Mock();publish=Mock()
        ctrl=AdvancedSafetyController(state_buffer=buffer,publish_lowcmd=publish,
            safety_active=threading.Event(),policy_gate_engaged=threading.Event(),
            policy_active=threading.Event(),shutdown_event=threading.Event(),column_mover=column)
        ctrl.lifecycle='running';ctrl.policy_active.set()
        with patch.object(ctrl,'_start_hold'),patch.object(ctrl,'_stop_hold'),patch.object(ctrl,'_run_sequence') as sequence:
            ctrl.policy_timeout(.6)
            self.assertIn('timed out',ctrl.last_error)
            aborted=ctrl.handle_request({'command':'abort_hold'})
            self.assertEqual(aborted['lifecycle'],'exited')
            response=ctrl.handle_request({'command':'startup','teleop_home_mode':True,
                'control_mode':'dual_arm','max_safe_height_mm':400,'lower_height_mm':400,'column_speed':.35})
            self.assertTrue(response['success']);self.assertEqual(response['lifecycle'],'ready')
            self.assertIsNone(response['last_error']);column.assert_not_called();sequence.assert_not_called()


if __name__=='__main__':unittest.main()
