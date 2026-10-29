"""A-gated start baseline tests with passive fake state; no robot commands."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from g1d_vr_teleop.core.dual_arm import VRControlError
from g1d_vr_teleop.control.profile import SiteDualVRControl
from tests.test_vr_dual_control import BASE, LIMITS, status, raw, xr
from g1d_vr_teleop.control.session import DirectFeedbackClient, capture_ready_start


def sample(arm_q, stamp=BASE, tick=1):
    q=[0.]*29
    q[12]=-.016
    q[15:29]=arm_q
    return SimpleNamespace(q=tuple(q),received_monotonic_ns=stamp,mode_machine=1,
                           tick=tick,source_timestamp_ns=1_000_000+tick)


class StartBaselineTests(unittest.TestCase):
    def setup_client(self,*samples):
        rpc=Mock();rpc.status.return_value=status()
        source=Mock();source.snapshot.side_effect=list(samples)
        return DirectFeedbackClient(rpc,source),rpc,source

    def test_waiting_drift_over_five_milliradians_becomes_new_start(self):
        launch_pose=sample((0.,)*14)
        after_wait=sample((.04,)*14,BASE+20_000_000,2)
        client,rpc,source=self.setup_client(launch_pose,after_wait)
        _,old=capture_ready_start(client)
        ready,current=capture_ready_start(client)
        self.assertGreater(max(abs(a-b) for a,b in zip(current.q[15:29],old.q[15:29])),.005)
        self.assertEqual(current.q[15:29],(.04,)*14)
        self.assertEqual(current.received_monotonic_ns,BASE+20_000_000)
        self.assertEqual(ready['lifecycle'],'ready')
        self.assertEqual(source.snapshot.call_count,2)
        source.snapshot.assert_called_with(max_age_s=.2)
        rpc.request_safety.assert_not_called()

    def test_first_site_packet_is_current_measured_pose_despite_far_ik_target(self):
        measured=sample((.06,)*14)
        client,rpc,_=self.setup_client(measured)
        ready,current=capture_ready_start(client)
        core=SiteDualVRControl(ready,current,LIMITS,now_ns=BASE)
        decision=core.step(current,replace(xr(),session_enabled=True),(1.5,)*14,
                           raw('left'),raw('right'),now_ns=BASE)
        self.assertTrue(decision.first_target)
        self.assertEqual(decision.arm_q,(.06,)*14)
        self.assertEqual(decision.arm_velocity,(0.,)*14)
        self.assertEqual(decision.body_q[12],-.016)
        self.assertEqual(decision.body_q[14],0.)
        rpc.request_safety.assert_not_called()

    def test_recording_preparation_rechecks_ready_and_reads_new_measurement(self):
        at_a=sample((.02,)*14)
        after_prepare=sample((.035,)*14,BASE+1_000_000_000,2)
        client,rpc,source=self.setup_client(at_a,after_prepare)
        _,first=capture_ready_start(client)
        _,newest=capture_ready_start(client)
        self.assertEqual(first.q[15:29],(.02,)*14)
        self.assertEqual(newest.q[15:29],(.035,)*14)
        self.assertEqual(newest.source_tick,2)
        self.assertEqual(rpc.status.call_count,2)
        self.assertEqual(source.snapshot.call_count,2)
        rpc.request_safety.assert_not_called()

    def test_invalid_ready_rejected_before_reading_state_or_opening_policy(self):
        client,rpc,source=self.setup_client(sample((0.,)*14))
        rpc.status.return_value={**status(),'lifecycle':'running','policy_active':True}
        with self.assertRaises(VRControlError):capture_ready_start(client)
        source.snapshot.assert_not_called()
        rpc.request_safety.assert_not_called()

    def test_stale_passive_state_failure_propagates_without_policy_rpc(self):
        client,rpc,source=self.setup_client()
        source.snapshot.side_effect=VRControlError('Original arm state is stale')
        with self.assertRaisesRegex(VRControlError,'state is stale'):
            capture_ready_start(client)
        rpc.request_safety.assert_not_called()


if __name__=='__main__':unittest.main()
