"""Deadline admission, standard camera geometry and measured completion contracts."""
import asyncio
import copy
import math
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock
import httpx
from ros2_agent_interfaces.srv import RobotRequest
from sensor_msgs.msg import CameraInfo
from helpers import RosFixture, config, eventually, arm_state
import pixels_test
from ros2_agent_server.api import create_app
from ros2_agent_server.completion import CompletionMonitor, CompletionPolicy
from ros2_agent_server.numerics import calibration_key, same_calibration, rectified_intrinsics
from ros2_agent_server.protocol import Reply, BridgeError
from ros2_agent_server.robot_node import RobotBridgeNode, Pending
from ros2_agent_server.state import StateStore
from embodiment.ros2.timing import Timing

POSE=dict(frame_id='world',position=[.1,0.,.3],orientation=[0.,0.,0.,1.])

class DeadlineGeometryTest(unittest.IsolatedAsyncioTestCase):
    async def fixture(self):
        f=RosFixture(config('minimal.json'))
        self.addCleanup(f.close)
        await f.ready()
        return f

    def test_late_success_cannot_win_before_timer(self):
        future=mock.Mock()
        future.done.return_value=False
        p=Pending(future,time.monotonic()-.01)
        bridge=SimpleNamespace(_lock=threading.RLock(),_pending=[p],get_logger=lambda:mock.Mock())
        RobotBridgeNode._complete(bridge,p,Reply(payload={'success':True}))
        result=future.set_result.call_args.args[0]
        self.assertEqual(504,result.status)
        self.assertFalse(result.payload['success'])
        RobotBridgeNode._complete(bridge,p,Reply())
        future.set_result.assert_called_once()

    async def test_queued_request_expired_before_dispatch_never_moves(self):
        f=await self.fixture()
        import json
        future=f.gateway.client.call_async(RobotRequest.Request(operation='move_arm',resource_id='arm',
            payload_json=json.dumps(POSE),timeout_sec=10.,deadline_ns=f.robot.get_clock().now().nanoseconds-1))
        await eventually(future.done)
        result=Reply.from_ros(future.result())
        self.assertEqual('request_expired',result.payload['code'])
        self.assertEqual('not_started',result.payload['outcome'])
        self.assertFalse(f.driver.calls)

    def test_shared_budgets_include_settling_measurement_and_delivery(self):
        c=config('minimal.json')
        timing=Timing.from_server(c.server.model_dump())
        for op,payload in [('move_arm',dict(duration=60.)),('move_arms',dict(moves=[dict(duration=60.)])),
            ('set_gripper',{}),('execute_plan',{}),('reset_arms',{}),('recover_arms',{}),('stop',{}),('capture',{}),('state',{})]:
            self.assertEqual(c.server.operation_timeout(op,payload),timing.operation_timeout(op,payload))
        self.assertGreater(c.server.operation_timeout('move_arms',dict(moves=[dict(duration=60.)])),60.+c.server.state_completion_timeout)
        self.assertEqual(c.server.stop_timeout,timing.operation_timeout('stop'))
        self.assertGreater(timing.observation_timeout,2*timing.camera_timeout)

    async def test_post_ack_motion_can_settle_within_dwell_budget(self):
        f=await self.fixture()
        f.config.server.settling_dwell=.1
        f.config.server.state_completion_timeout=1.
        f.driver.hold_moves=True
        task=asyncio.create_task(f.gateway.request('move_arm','arm',POSE,2.))
        await eventually(lambda:bool(f.driver.held))
        f.driver.states['arm']['moving']=True
        f.driver.release(True)
        await asyncio.sleep(.15)
        self.assertFalse(task.done())
        f.driver.states['arm']['moving']=False
        result=await task
        self.assertTrue(result.payload['success'],result.payload)

    async def test_unsettled_motion_times_out_and_stop_remains_available(self):
        f=await self.fixture()
        f.config.server.state_completion_timeout=.15
        f.driver.hold_moves=True
        task=asyncio.create_task(f.gateway.request('move_arm','arm',POSE,.5))
        await eventually(lambda:bool(f.driver.held))
        f.driver.states['arm']['moving']=True
        f.driver.release(True)
        result=await task
        self.assertEqual('post_command_state_timeout',result.payload['code'])
        self.assertEqual('unknown',result.payload['outcome'])
        self.assertTrue(f.robot._recovery_required)
        self.assertTrue((await f.gateway.request('stop','',{'arm_id':None},1.)).payload['success'])

    def test_plan_and_capture_lifetimes_are_configured_not_refreshed(self):
        g=pixels_test.PixelProjectionTest();g.setUp()
        g.config.server.capture_ttl=300.
        g.config.server.plan_ttl=600.
        cap=g.capture()
        plan=g.store.create(cap['capture_id'],[dict(arm_id='left',grasp=[1,1],release=[2,2])])
        g.store.plans[plan['plan_id']]['state']='approached'
        g.now=250.
        self.assertEqual('approached',g.store.get(plan['plan_id'])['state'])
        g.now=301.
        with self.assertRaises(BridgeError):g.store.get_capture(cap['capture_id'])
        g.now=601.
        with self.assertRaises(BridgeError):g.store.get(plan['plan_id'])
        self.assertEqual(0.,g.store.plans[plan['plan_id']]['created'])

    def test_standard_rectified_P_is_used_and_unsupported_geometry_rejected(self):
        g=pixels_test.PixelProjectionTest();g.setUp()
        g.config.cameras[0].camera_info_mode='ros_rectified'
        g.info.p=[4.,0.,1.,0.,0.,4.,1.,0.,0.,0.,1.,0.]
        g.info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        g.info.d=[.1,-.01]
        capture=g.capture()
        self.assertAlmostEqual(1.5,g.store.project(capture['capture_id'],[2,1])['position'][0],delta=1e-6)
        good=copy.deepcopy(g.info)
        for field,change in [('p.translation',lambda:setattr(g.info,'p',good.p[:3]+[.1]+good.p[4:])),
                            ('r',lambda:g.info.r.__setitem__(0,.9)),
                            ('roi.dimensions',lambda:setattr(g.info.roi,'width',2)),
                            ('binning_x',lambda:setattr(g.info,'binning_x',2))]:
            g.info=copy.deepcopy(good)
            change()
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,field):g.capture()
        g.info=copy.deepcopy(good)
        g.info.d=[]
        g.config.cameras[0].camera_info_mode='rectified_k'
        with self.assertRaisesRegex(ValueError,'camera_info.p'):g.capture()
        g.info.p=[0.]*12
        g.info.r[0]=.9
        with self.assertRaisesRegex(ValueError,'camera_info.r'):g.capture()

    def test_projection_matrix_change_invalidates_calibration_key(self):
        a=CameraInfo(k=[100.,0.,24.,0.,100.,16.,0.,0.,1.],p=[100.,0.,24.,0.,0.,100.,16.,0.,0.,0.,1.,0.])
        b=copy.deepcopy(a);b.p[0]+=.01
        self.assertFalse(same_calibration(calibration_key(a),calibration_key(b),config().cameras[0]))
        camera=config().cameras[0]
        camera.camera_info_mode='ros_rectified'
        a.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        for field,index in [('p',3),('r',1)]:
            valid=copy.deepcopy(a)
            getattr(valid,field)[index]=.9*camera.rectification_tolerance
            invalid=copy.deepcopy(valid)
            getattr(invalid,field)[index]=1.1*camera.rectification_tolerance
            rectified_intrinsics(valid,camera)
            with self.assertRaisesRegex(ValueError,'camera_info.'+field):rectified_intrinsics(invalid,camera)
            self.assertFalse(same_calibration(calibration_key(valid),calibration_key(invalid),camera))

    def test_stationary_history_rejects_late_motion_exposure_and_sample_gaps(self):
        store=StateStore(config('minimal.json'))
        for stamp,moving in [(1_000_000_000,False),(1_100_000_000,True),(1_200_000_000,False),(1_300_000_000,False)]:
            state=arm_state();state['moving']=moving
            store.update_arm('arm',state,stamp)
        self.assertFalse(store.stationary_interval('arm',1_050_000_000,1_250_000_000))
        self.assertTrue(store.stationary_interval('arm',1_210_000_000,1_290_000_000))
        self.assertFalse(store.stationary_interval('arm',1_210_000_000,1_400_000_000))
        store.update_arm('arm',arm_state(),1_800_000_000)
        self.assertFalse(store.stationary_interval('arm',1_400_000_000,1_700_000_000))
        store.reset()
        self.assertFalse(store.stationary_interval('arm',1_210_000_000,1_290_000_000))

    def test_completion_requires_tolerance_dwell_and_known_opening(self):
        policy=CompletionPolicy(dwell_sec=.25,max_sample_gap_sec=.2)
        monitor=CompletionMonitor(POSE,opening=.5,policy=policy)
        state=arm_state();state['gripper']['opening']=.51
        state['flange_pose']['position'][0]+=.001
        state['flange_pose']['orientation']=[0.,0.,0.,-1.]
        self.assertFalse(monitor.update(state,1.))
        self.assertFalse(monitor.update(state,1.125))
        self.assertTrue(monitor.update(state,1.25))
        for mutation in ('position','opening','moving','frame','angle'):
            bad=copy.deepcopy(state)
            if mutation=='position':bad['flange_pose']['position'][0]+=.01
            if mutation=='opening':bad['gripper']['opening']=None
            if mutation=='moving':bad['moving']=True
            if mutation=='frame':bad['flange_pose']['frame_id']='other'
            if mutation=='angle':bad['flange_pose']['orientation']=[0.,0.,math.sin(.1),math.cos(.1)]
            monitor=CompletionMonitor(POSE,opening=.5,policy=policy)
            self.assertFalse(monitor.update(bad,1.))
            self.assertFalse(monitor.update(bad,1.125))
            self.assertFalse(monitor.update(bad,1.25))
        monitor=CompletionMonitor(POSE,opening=.5,policy=policy)
        self.assertFalse(monitor.update(state,1.))
        self.assertFalse(monitor.update(state,1.5))
        self.assertFalse(monitor.update(state,1.5))
        self.assertFalse(monitor.update(state,1.625))

    async def test_measured_driver_completion_success_failure_and_recovery(self):
        for reaches_target in (True,False):
            f=await self.fixture()
            f.driver.hold_moves=True
            monitor=CompletionMonitor(POSE,policy=CompletionPolicy(dwell_sec=.08,max_sample_gap_sec=.2))
            task=asyncio.create_task(f.gateway.request('move_arm','arm',POSE,2.))
            await eventually(lambda:bool(f.driver.held))
            completed=False
            for _ in range(4):
                state=arm_state()
                state['flange_pose']['position'][0]+=.001 if reaches_target else .01
                f.driver.states['arm']=state
                stamp=f.driver.get_clock().now().nanoseconds/1e9
                if monitor.update(state,stamp):
                    completed=True
                    break
                await asyncio.sleep(.05)
            f.driver.release(completed)
            result=await task
            self.assertEqual(reaches_target,result.payload['success'])
            if not reaches_target:
                self.assertTrue(f.robot._recovery_required)
                f.driver.hold_moves=False
                self.assertTrue((await f.gateway.request('stop','',{'arm_id':None},1.)).payload['success'])
                recovered=await f.gateway.request('recover_arms','',{},1.)
                self.assertTrue(recovered.payload['success'],recovered.payload)
                self.assertFalse(f.robot._recovery_required)

    def test_gradual_clock_drift_preserves_stamps_and_rejects_excess_skew(self):
        import json
        c=config('minimal.json')
        store=StateStore(c)
        now=1_000_000_000
        bridge=SimpleNamespace(config=c,store=store,get_logger=lambda:mock.Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now)))
        accepted=None
        for lead_ms in range(9):
            now+=30_000_000
            measured=now+lead_ms*1_000_000
            RobotBridgeNode._arm_state(bridge,'arm',SimpleNamespace(data=json.dumps(dict(arm_state(),stamp_ns=measured))))
            if lead_ms <= 5:accepted=measured
            self.assertEqual(accepted,store.state(now)['arms'][0]['measurement_stamp_ns'])
        # Corrected source clock advances beyond the last accepted measurement.
        now+=30_000_000
        RobotBridgeNode._arm_state(bridge,'arm',SimpleNamespace(data=json.dumps(dict(arm_state(),stamp_ns=now))))
        self.assertEqual(now,store.state(now)['arms'][0]['measurement_stamp_ns'])

    async def test_final_deadline_check_invalidates_late_motion_success(self):
        import json
        c=config('minimal.json')
        now=[100.]
        async def delayed_workflow(*args,**kwargs):
            now[0]=102.
            return Reply(payload={'success':True})
        bridge=SimpleNamespace(config=c,get_logger=lambda:mock.Mock(),_workflow=delayed_workflow,
            pixels=mock.Mock(),_motion_uncertain=False,_recovery_required=False,_all_stopped=True)
        request=RobotRequest.Request(operation='move_arm',resource_id='arm',payload_json=json.dumps(POSE),timeout_sec=1.)
        with mock.patch('ros2_agent_server.robot_node.time.monotonic',side_effect=lambda:now[0]):
            response=await RobotBridgeNode._request(bridge,request,RobotRequest.Response())
        result=Reply.from_ros(response)
        self.assertEqual(504,result.status)
        self.assertEqual('unknown',result.payload['outcome'])
        self.assertTrue(bridge._motion_uncertain and bridge._recovery_required)
        self.assertFalse(bridge._all_stopped)
        bridge.pixels.invalidate.assert_called_once()
