"""Asynchronous hardware inputs through real ROS services and the HTTP agent."""

import asyncio
import copy
import json
import struct
import unittest
from unittest import mock

import httpx
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage, CameraInfo, Image

from helpers import ROOT, RosFixture, config, eventually, jpeg
from ros2_agent_server.api import create_app
from ros2_agent_server.robot_node import Pending
from scenario_test import ScenarioStream


class HardwareIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def make_fixture(self, topology='minimal.json'):
        c = config(topology)
        for camera in c.cameras:
            camera.sync_tolerance_sec = .01
            camera.depth_frame = camera.id + '_registered_depth'
        c.server.state_completion_timeout = .3
        f = RosFixture(c)
        self.addCleanup(f.close)
        await f.ready()
        return f

    async def test_buffered_delayed_depth_with_different_frame_and_stamp(self):
        f = await self.make_fixture()
        f.driver.depth_offset_ns = -5_000_000
        f.driver.depth_delay_ticks = 2
        result = await f.gateway.request('capture', 'overhead', {}, 1.)
        self.assertEqual(200, result.status, result.payload)
        self.assertEqual(-5_000_000, result.payload['sync_delta_ns'])
        self.assertEqual('overhead_registered_depth', result.payload['depth_frame_id'])
        self.assertEqual(result.payload['stamp_ns'], result.payload['pose_stamp_ns'])
        point = f.robot.pixels.project(result.payload['capture_id'], [24,16])
        self.assertEqual([0.,0.,1.], point['position'])
        self.assertFalse(f.driver.calls)

    async def test_timeout_reports_matching_reason_and_observation_needs_no_geometry(self):
        f = await self.make_fixture()
        f.driver.publish_geometry = False
        f.robot._depth.clear()
        f.robot._calibration.clear()
        with mock.patch.object(f.robot, 'get_logger', return_value=mock.Mock()) as logger:
            result = await f.gateway.request('capture', 'overhead', {}, .15)
            self.assertEqual(504, result.status)
            self.assertEqual('camera_info_unavailable', result.payload['details']['code'])
            self.assertTrue(logger.return_value.warning.called)
        image = await f.gateway.request('observation', 'overhead', {}, .3)
        self.assertEqual(200, image.status)
        self.assertEqual('rgb', image.payload['kind'])
        self.assertNotIn('camera_pose', image.payload)
        self.assertFalse(f.driver.calls)

    async def test_outside_tolerance_fails_and_parallel_capture_keeps_identity(self):
        f = await self.make_fixture()
        f.driver.depth_offset_ns = -15_000_000
        f.robot._depth.clear()
        result = await f.gateway.request('capture', 'overhead', {}, .2)
        self.assertEqual(504, result.status)
        self.assertEqual('depth_sync_unavailable', result.payload['details']['code'])
        f.driver.depth_offset_ns = -4_000_000
        results = await asyncio.gather(*(f.gateway.request('capture','overhead',{},.8) for _ in range(3)))
        self.assertTrue(all(r.status == 200 for r in results))
        self.assertEqual(3, len({r.payload['capture_id'] for r in results}))
        self.assertTrue(all(r.payload['sync_delta_ns'] == -4_000_000 for r in results))

    async def test_new_state_after_response_is_required_for_every_arm(self):
        f = await self.make_fixture('dual_arm.json')
        f.driver.skip_states = {'left', 'right'}
        call = asyncio.create_task(f.gateway.request('reset_arms', '', {}, 1.))
        await eventually(lambda: any(p.check is not None for p in f.robot._pending))
        self.assertFalse(call.done())
        f.driver.skip_states = {'right'}
        stamp = f.robot.store._arms['left'][2]
        await eventually(lambda: f.robot.store._arms['left'][2] > stamp)
        self.assertFalse(call.done())
        f.driver.skip_states.clear()
        result = await call
        self.assertEqual(200, result.status, result.payload)
        self.assertTrue(result.payload['success'])

    async def test_stale_republication_after_completion_latches_unknown(self):
        f = await self.make_fixture()
        f.driver.state_stamp_ns = f.robot.store._arms['arm'][2]
        result = await f.gateway.request('reset_arms', '', {}, 1.)
        self.assertEqual(504, result.status)
        self.assertEqual('post_command_state_timeout', result.payload['code'])
        self.assertEqual('unknown', result.payload['outcome'])
        blocked = await f.gateway.request('reset_arms', '', {}, 1.)
        self.assertEqual(409, blocked.status)
        stopped = await f.gateway.request('stop', '', {'arm_id':None}, .5)
        self.assertTrue(stopped.payload['success'])

    async def test_standard_string_state_requires_measurement_time_and_same_fault_contract(self):
        f = await self.make_fixture()
        result = await f.gateway.request('state', '', {}, .5)
        self.assertGreater(result.payload['arms'][0]['measurement_stamp_ns'], 0)
        self.assertEqual('/robotics/arms/arm/state', f.config.state_topic('arm'))
        self.assertIn((f.config.state_topic('arm'), ['std_msgs/msg/String']), f.robot.get_topic_names_and_types())
        result = await f.gateway.request('reset_arms', '', {}, 1.)
        self.assertTrue(result.payload['success'])

    async def test_tf_interpolates_at_image_time_and_reports_missing_transform(self):
        f = await self.make_fixture('single_arm.json')
        f.driver.publish_images = False
        f.robot.tf_buffer.clear()
        f.robot._depth.clear()
        task = asyncio.create_task(f.gateway.request('capture', 'wrist', {}, .8))
        # This configuration calls the wrist camera 'wrist'.
        await eventually(lambda: any(p.camera and p.camera[0]=='wrist' for p in f.robot._pending))
        camera = next(c for c in f.config.cameras if c.id == 'wrist')
        stamp = f.driver.get_clock().now().nanoseconds
        rgb = CompressedImage(format='jpeg', data=jpeg())
        rgb.header.frame_id = camera.optical_frame
        rgb.header.stamp.sec, rgb.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        info = CameraInfo(header=copy.deepcopy(rgb.header), width=48, height=32,
                          k=[100.,0.,24.,0.,100.,16.,0.,0.,1.])
        depth = Image(header=copy.deepcopy(rgb.header), width=48, height=32,
                      encoding='16UC1', step=96, data=struct.pack('<H',1000)*48*32)
        depth.header.frame_id = camera.depth_frame
        f.driver.info_publishers['wrist'].publish(info)
        f.driver.image_publishers['wrist'].publish(rgb)
        f.driver.depth_publishers['wrist'].publish(depth)
        await eventually(lambda: any(p.reason.get('code') == 'capture_tf_unavailable' for p in f.robot._pending))
        for delta, x in ((-1_000_000,1.), (1_000_000,3.)):
            for frame in (camera.optical_frame, camera.parent_frame):
                tf = TransformStamped()
                tf.header.frame_id = f.config.world_frame
                tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(stamp+delta, 1_000_000_000)
                tf.child_frame_id = frame
                tf.transform.rotation.w = 1.
                tf.transform.translation.x = x
                f.driver.tf_broadcaster.sendTransform(tf)
        result = await task
        self.assertEqual(200, result.status, result.payload)
        self.assertAlmostEqual(2., result.payload['camera_pose']['position'][0])
        self.assertAlmostEqual(2., result.payload['flange_pose']['position'][0])
        self.assertEqual([2.,0.,1.], f.robot.pixels.project(result.payload['capture_id'],[24,16])['position'])

    async def test_calibration_change_invalidates_old_geometry_and_restarts_capture(self):
        f = await self.make_fixture()
        result = await f.gateway.request('capture', 'overhead', {}, 1.)
        old = result.payload['capture_id']
        f.driver.publish_geometry = False
        info = copy.deepcopy(f.robot._calibration['overhead'])
        info.k[0] = 200.
        f.driver.info_publishers['overhead'].publish(info)
        await eventually(lambda: old not in f.robot.pixels.captures)
        self.assertNotIn(old, f.robot.pixels.captures)
        await eventually(lambda: f.robot._calibration['overhead'].k[0] == 200.)
        f.driver.publish_geometry = True
        # Wait for the fixture's calibration change before issuing a new request.
        await eventually(lambda: f.robot._calibration['overhead'].k[0] == 100.)
        result = await f.gateway.request('capture', 'overhead', {}, 1.)
        self.assertEqual(200, result.status)

    async def test_response_driven_agent_with_delayed_depth_and_rgb_only_inspection(self):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        f = await self.make_fixture()
        f.driver.depth_offset_ns = -5_000_000
        f.driver.depth_delay_ticks = 1
        original = f.driver.execute
        async def execute(request, response):
            result = await original(request, response)
            if request.operation == 'execute_plan':
                stage = json.loads(request.payload_json)['stage']
                f.driver.images['overhead'] = jpeg('green' if stage == 'pick' else 'blue')
                if stage == 'pick':
                    f.driver.publish_geometry = False
                    f.driver.geometry_queue.clear()
                    f.robot._depth.clear()
            return result
        f.driver.execute = execute
        stream = ScenarioStream('success', ['arm'], lambda *args: None)
        def er(request):
            body = dict(targets=[dict(arm_id='arm', grasp=[500,500], release=[750,750])])
            return httpx.Response(200, json={'candidates':[{'finishReason':'STOP',
                'content':{'parts':[{'text':json.dumps(body)}]}}]})
        with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
            client.return_value.create_stream.return_value = stream
            result = await run_application(AgentConfig.load(ROOT/'agent/configs/minimal.json'),
                'Move block to tray', model='mock', api_key='mock', timeout=20,
                transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),
                robotics_transport=httpx.MockTransport(er))
        if stream.failure:
            raise stream.failure
        self.assertTrue(result['success'])
        self.assertTrue(stream.completed)
        self.assertTrue(any(name=='inspect_grasp' and r['success'] for name,r in stream.results))

    async def test_stop_interrupts_waiting_for_post_command_measurement(self):
        f = await self.make_fixture()
        f.driver.publish_states = False
        motion = asyncio.create_task(f.gateway.request('reset_arms', '', {}, 1.))
        await eventually(lambda: any(p.check is not None for p in f.robot._pending))
        stopped = await f.gateway.request('stop', '', {'arm_id':None}, .5)
        self.assertTrue(stopped.payload['success'])
        result = await motion
        self.assertEqual(409, result.status)
        self.assertFalse(f.robot._motion_uncertain)
        self.assertTrue(f.robot._recovery_required)
        self.assertEqual(['reset_arms', 'stop'], [c[0] for c in f.driver.calls])

    async def test_clock_rollback_rejects_pending_capture_and_drops_old_evidence(self):
        f = await self.make_fixture()
        first = await f.gateway.request('observation', 'overhead', {}, .5)
        f.driver.publish_images = False
        pending = asyncio.create_task(f.gateway.request('capture', 'overhead', {}, .5))
        await eventually(lambda: any(p.camera for p in f.robot._pending))
        # Exercise the production clock-jump detector without changing the machine clock.
        f.robot._last_ros_ns = f.robot.get_clock().now().nanoseconds + 10_000_000_000
        result = await pending
        self.assertEqual(409, result.status)
        self.assertEqual('clock_reset', result.payload['code'])
        self.assertNotIn(first.payload['capture_id'], f.robot.pixels.captures)
        f.driver.publish_images = True
        fresh = await f.gateway.request('capture', 'overhead', {}, .8)
        self.assertEqual(200, fresh.status)

    async def test_request_id_correlates_http_gateway_bridge_and_driver(self):
        f = await self.make_fixture()
        traces = []
        original = f.driver.execute
        async def execute(request, response):
            traces.append(request.request_id)
            return await original(request, response)
        f.driver.execute = execute
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),
                                     base_url='http://test') as http:
            response = await http.post('/v1/arms/reset', json={})
        self.assertEqual(200, response.status_code)
        self.assertEqual([response.headers['x-request-id']], traces)

    async def test_wrist_rgb_verification_works_without_tf_or_depth(self):
        f = await self.make_fixture('single_arm.json')
        capture = (await f.gateway.request('capture','overhead',{},.8)).payload
        plan = (await f.gateway.request('create_plan','',dict(capture_id=capture['capture_id'],
            targets=[dict(arm_id='arm',grasp=[24,16],release=[24,24])]),.5)).payload
        picked = await f.gateway.request('execute_plan','',dict(plan_id=plan['plan_id'],stage='pick'),1.)
        self.assertTrue(picked.payload['success'])
        f.driver.publish_geometry = False
        f.robot.tf_buffer.clear()
        f.robot._depth.clear()
        observation = await f.gateway.request('observation','wrist',{},.5)
        self.assertEqual(200,observation.status)
        verified = await f.gateway.request('verify_grasp','',dict(plan_id=plan['plan_id'],observations=[
            dict(arm_id='arm',capture_id=observation.payload['capture_id'],success=True)]),.5)
        self.assertTrue(verified.payload['success'])
