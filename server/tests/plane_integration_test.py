"""Planar capture over ROS/HTTP, including a response-driven Gemini application."""

import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import httpx

from helpers import RosFixture, eventually, jpeg
from plane_projection_test import plane_config
from scenario_test import ScenarioStream
from ros2_agent_server.api import create_app


class PlaneIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def fixture(self, name='minimal.json'):
        f=RosFixture(plane_config(name))
        self.addCleanup(f.close)
        f.driver.publish_geometry=False
        f.driver.geometry_queue.clear()
        await f.ready()
        f.robot._depth.clear()
        f.robot._calibration.clear()
        f.robot.tf_buffer.clear()
        return f

    async def test_http_capture_without_depth_calibration_or_tf(self):
        f=await self.fixture()
        with mock.patch.object(f.robot, '_pose_at', side_effect=AssertionError('Plane must not query TF')):
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)), base_url='http://test') as client:
                capture=await client.get('/v1/cameras/overhead/capture')
                self.assertEqual(200,capture.status_code,capture.text)
                self.assertEqual('plane',capture.json()['kind'])
                self.assertNotIn('camera_pose',capture.json())
                self.assertIn('x-request-id',capture.headers)
                bad=await client.post('/v1/plans',json={'capture_id':capture.json()['capture_id'],
                    'targets':[dict(arm_id='arm',grasp=[0,0],release=[10,10])]})
                self.assertEqual(422,bad.status_code,bad.text)
                self.assertIn('valid_region',bad.text)
                self.assertFalse(f.driver.calls)
                f.driver.publish_images=False
                f.config.server.camera_timeout=.1
                missing=await client.get('/v1/cameras/overhead/capture')
                self.assertEqual(504,missing.status_code)
                self.assertEqual('fresh_rgb_unavailable',missing.json()['details']['code'])

    async def test_camera_info_change_latches_plane_calibration_invalid(self):
        from sensor_msgs.msg import CameraInfo
        f=await self.fixture()
        info=CameraInfo(width=48,height=32,k=[100.,0.,24.,0.,100.,16.,0.,0.,1.])
        info.header.frame_id='overhead_optical'
        f.driver.info_publishers['overhead'].publish(info)
        await eventually(lambda:'overhead' in f.robot._calibration)
        capture=await f.gateway.request('capture','overhead',{},.8)
        self.assertEqual(200,capture.status)
        changed=copy.deepcopy(info)
        changed.k[0]=200.
        f.driver.info_publishers['overhead'].publish(changed)
        await eventually(lambda:'overhead' in f.robot._invalid_plane_cameras)
        self.assertNotIn(capture.payload['capture_id'],f.robot.pixels.captures)
        result=await f.gateway.request('capture','overhead',{},.5)
        self.assertEqual(409,result.status)
        self.assertEqual('plane_calibration_invalidated',result.payload['code'])
        # Restoring the topic does not silently revive the old offline calibration.
        f.driver.info_publishers['overhead'].publish(info)
        result=await f.gateway.request('capture','overhead',{},.5)
        self.assertEqual(409,result.status)

    async def test_plane_detection_then_depth_wrist_refinement(self):
        f=await self.fixture('single_arm.json')
        captured=await f.gateway.request('capture','overhead',{},.8)
        self.assertEqual(200,captured.status)
        plan=await f.gateway.request('create_plan','',{'capture_id':captured.payload['capture_id'],
            'targets':[dict(arm_id='arm',grasp=[10,20],release=[20,10])]},.8)
        self.assertEqual(200,plan.status,plan.payload)
        plan_id=plan.payload['plan_id']
        release=copy.deepcopy(plan.payload['targets'][0]['release'])
        approach=await f.gateway.request('execute_plan','',dict(plan_id=plan_id,stage='approach'),1.)
        self.assertEqual(200,approach.status,approach.payload)
        f.driver.wrist_offset=.25
        f.driver.publish_geometry=True
        wrist=await f.gateway.request('capture','wrist',{},1.)
        self.assertEqual(200,wrist.status,wrist.payload)
        self.assertEqual('rgbd',wrist.payload['kind'])
        refined=await f.gateway.request('refine_plan','',dict(plan_id=plan_id,arm_id='arm',
            capture_id=wrist.payload['capture_id'],pixel=[24,16]),.8)
        self.assertEqual(200,refined.status,refined.payload)
        self.assertEqual([.25,0.,1.],refined.payload['targets'][0]['grasp']['position'])
        self.assertEqual(release,refined.payload['targets'][0]['release'])

    async def test_gemini_scenario_completes_with_only_rgb_and_offline_plane(self):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        f=await self.fixture()
        original=f.driver.execute
        async def execute(request,response):
            result=await original(request,response)
            if request.operation=='move' and json.loads(request.payload_json).get('phase') in ('lift','retreat'):
                stage=json.loads(request.payload_json)['phase']
                f.driver.images['overhead']=jpeg('green' if stage=='lift' else 'blue')
            return result
        f.driver.execute=execute
        stream=ScenarioStream('success',['arm'],lambda *args:None)
        prompts=[]
        def er(request):
            prompts.append(json.loads(request.content)['contents'][0]['parts'][1]['text'])
            return httpx.Response(200,json={'candidates':[{'finishReason':'STOP','content':{'parts':[
                {'text':json.dumps({'targets':[dict(arm_id='arm',grasp=[500,500],release=[750,750])]})}]}}]})
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            path.write_text(f.config.model_dump_json())
            with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
                client.return_value.create_stream.return_value=stream
                result=await run_application(AgentConfig.load(path),'Move the thin part on the calibrated plane',
                    model='mock',api_key='mock',timeout=20,
                    transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),
                    robotics_transport=httpx.MockTransport(er))
        if stream.failure:
            raise stream.failure
        self.assertTrue(result['success'])
        self.assertTrue(stream.completed)
        self.assertTrue(prompts)
        self.assertIn('calibrated plane, not measured depth',prompts[0])
        self.assertIn('test-table-v1',prompts[0])
        stages=[payload for op,_,payload in f.driver.calls if op=='prepare' and payload['steps'][-1].get('phase')]
        self.assertEqual(['lift','retreat'],[p['steps'][-1]['phase'] for p in stages])
        plan=next(iter(f.robot.pixels.plans.values()))
        self.assertEqual('plane',plan['targets'][0]['grasp']['projection'])
        self.assertAlmostEqual(3.,plan['targets'][0]['grasp']['position'][2])
        contact=stages[0]['steps'][2]['targets'][0]
        self.assertAlmostEqual(3.,contact['position'][2])
        self.assertEqual('tcp',contact['reference'])
        self.assertFalse(f.robot._depth)
