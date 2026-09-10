import asyncio
import base64
import json
import unittest
from unittest import mock

import httpx

from ros2_agent_server.api import create_app
from helpers import ROOT, RosFixture, MockGeminiStream, config, eventually


class WorkflowStream(MockGeminiStream):
    def __init__(self, assessment_success=True):
        self.assessment_success = assessment_success
        self.plan_id = None
        self.observations = {}
        super().__init__(self.script())

    def Send(self, message):
        if 'toolResponse' in message:
            response = message['toolResponse']['functionResponses'][0]['response']
            if 'plan_id' in response:
                self.plan_id = response['plan_id']
            if 'observation_id' in response:
                self.observations[response['arm_id']] = response['observation_id']
        super().Send(message)

    def script(self):
        yield dict(id='reset', name='reset_arms', args={})
        for cycle in range(2 if self.assessment_success else 1):
            yield dict(id=f'detect{cycle}', name='detect_targets', args=dict(
                camera_id='overhead', instruction='Move both ends of the bar to the tray', arm_ids=['left','right']))
            if cycle == 0:
                yield dict(id='approach', name='approach_targets', args=dict(plan_id=self.plan_id))
                yield dict(id='refine', name='refine_grasp', args=dict(plan_id=self.plan_id,
                    arm_id='left', camera_id='left_wrist', instruction='Left end of the bar'))
            yield dict(id=f'pick{cycle}', name='pick_targets', args=dict(plan_id=self.plan_id))
            for arm in ('left', 'right'):
                yield dict(id=f'inspect-{arm}-{cycle}', name='inspect_grasp', args=dict(plan_id=self.plan_id,arm_id=arm))
            yield dict(id=f'verify{cycle}', name='verify_grasp', args=dict(plan_id=self.plan_id, observations=[
                dict(arm_id=arm,observation_id=self.observations[arm],success=self.assessment_success or arm == 'left',
                     reason='The bar end is visible in the gripper.' if self.assessment_success or arm == 'left' else 'The right gripper appears empty.')
                for arm in ('left','right')]))
            yield dict(id=f'place{cycle}', name='place_targets', args=dict(plan_id=self.plan_id))
            yield dict(id=f'observe{cycle}', name='get_robot_state', args={})
        yield dict(id='done', name='finish_task', args=dict(success=self.assessment_success,
            summary='Both cycles visually checked' if self.assessment_success else 'Right grasp failed; stopped.'))


class PixelIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.fixture = RosFixture(config('dual_arm.json'))
        self.addCleanup(self.fixture.close)
        await self.fixture.ready()
        self.app = create_app(self.fixture.gateway, self.fixture.config)
        self.http = httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app), base_url='http://test')
        self.addAsyncCleanup(self.http.aclose)

    async def capture(self, camera='overhead'):
        response = await self.http.get(f'/v1/cameras/{camera}/capture')
        self.assertEqual(200, response.status_code, response.text)
        self.assertEqual('no-store', response.headers['cache-control'])
        return response.json()

    async def plan(self):
        capture = await self.capture()
        response = await self.http.post('/v1/plans', json=dict(capture_id=capture['capture_id'], targets=[
            dict(arm_id='left', grasp=[12,16], release=[12,24]),
            dict(arm_id='right', grasp=[36,16], release=[36,24])]))
        self.assertEqual(200,response.status_code,response.text)
        return response.json()

    async def test_agent_can_reobserve_after_depth_quality_failure(self):
        import struct
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from embodiment.ros2.manipulation import Manipulation
        from embodiment.ros2.robot_client import Ros2RobotClient
        agent_config = AgentConfig.load(ROOT / 'agent/configs/dual_arm.json')
        robot = Ros2RobotClient(agent_config, transport=httpx.ASGITransport(app=self.app))
        self.addAsyncCleanup(robot.close)
        reasoning = mock.Mock()
        reasoning.reason = mock.AsyncMock(return_value={'targets': [
            dict(arm_id='left', grasp=[500,500], release=[500,750])]})
        tools = Manipulation(agent_config, robot, reasoning)
        # An in-range outlier cannot be detected by min/max bounds alone.
        depth = [1000]*(48*32)
        depth[16*48+24] = 2000
        self.fixture.driver.depth_data = struct.pack('<1536H', *depth)
        with self.assertRaises(httpx.HTTPStatusError) as error:
            await tools.detect_targets('overhead', 'Move the workpiece', ['left'])
        self.assertEqual(422, error.exception.response.status_code)
        self.assertEqual('depth_quality_invalid', error.exception.response.json()['code'])
        self.assertFalse(tools.plans)
        first_capture = reasoning.reason.call_args.args[1]['capture_id']
        self.fixture.driver.depth_data = struct.pack('<1536H', *[
            1000+(i % 3-1)*5 for i in range(48*32)])
        result = await tools.detect_targets('overhead', 'Reobserve and locate the workpiece', ['left'])
        self.assertTrue(result['success'], result)
        self.assertNotEqual(first_capture, reasoning.reason.call_args.args[1]['capture_id'])
        self.assertAlmostEqual(1., result['targets'][0]['grasp']['position'][2], delta=.005001)
        self.assertFalse(self.fixture.driver.calls)

    async def test_capture_binds_tf_at_exposure_not_current_arm_pose(self):
        capture = await self.capture('left_wrist')
        self.assertAlmostEqual(1.,capture['camera_pose']['position'][0])
        self.assertAlmostEqual(1.,capture['flange_pose']['position'][0])
        self.fixture.driver.wrist_offset = 9.
        later = await self.capture('left_wrist')
        self.assertAlmostEqual(9.,later['camera_pose']['position'][0])
        point = self.fixture.robot.pixels.project(capture['capture_id'],[24,16])
        self.assertEqual([1.,0.,1.],point['position'])
        self.assertEqual(capture['stamp_ns'],point['stamp_ns'])

    async def test_missing_capture_geometry_fails_without_driver_motion(self):
        self.fixture.driver.publish_geometry = False
        await asyncio.sleep(.1)
        reply = await self.fixture.gateway.request('capture','overhead',{},.1)
        self.assertEqual(504,reply.status)
        self.assertEqual([], self.fixture.driver.calls)

    async def test_place_requires_post_pick_wrist_verification_and_no_replay(self):
        plan = await self.plan()
        before = await self.capture('left_wrist')
        body = dict(plan_id=plan['plan_id'], stage='place')
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=body)).status_code)
        picked = await self.http.post('/v1/plans/execute',json=dict(body,stage='pick'))
        self.assertEqual('picked',picked.json()['state'])
        left, right = await self.capture('left_wrist'), await self.capture('right_wrist')
        obs = [dict(arm_id='left',capture_id=left['capture_id'],success=True),
               dict(arm_id='right',capture_id=right['capture_id'],success=True)]
        stale = [dict(obs[0],capture_id=before['capture_id']),obs[1]]
        self.assertIn((await self.http.post('/v1/plans/verify',json=dict(plan_id=plan['plan_id'],observations=stale))).status_code, (409,422))
        failed = await self.http.post('/v1/plans/verify',json=dict(plan_id=plan['plan_id'],observations=[dict(obs[0],success=False),obs[1]]))
        self.assertFalse(failed.json()['success'])
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=body)).status_code)
        verified = await self.http.post('/v1/plans/verify',json=dict(plan_id=plan['plan_id'],observations=obs))
        self.assertEqual('verified',verified.json()['state'])
        self.assertEqual('placed',(await self.http.post('/v1/plans/execute',json=body)).json()['state'])
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=body)).status_code)
        self.assertEqual(['lift','retreat'],[c[2]['steps'][-1]['phase'] for c in self.fixture.driver.calls if c[0]=='prepare'])
        self.assertEqual(['open','approach','descend','close','lift'],[step['phase'] for step in self.fixture.driver.calls[0][2]['steps']])
        self.assertEqual(['left','right'],self.fixture.driver.calls[0][2]['arm_ids'])

    async def test_stop_interrupts_group_and_prevents_late_success_or_retry(self):
        plan = await self.plan()
        self.fixture.driver.hold_moves = True
        body = dict(plan_id=plan['plan_id'],stage='pick')
        pending = asyncio.create_task(self.http.post('/v1/plans/execute',json=body))
        await eventually(lambda: bool(self.fixture.driver.held))
        competing = await self.http.post('/v1/move', json=dict(all_arms=True, targets=[dict(kind='named',name='home')]))
        self.assertEqual(409,competing.status_code)
        self.assertTrue((await self.http.post('/v1/stop',json={})).json()['success'])
        self.assertFalse((await pending).json()['success'])
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=body)).status_code)
        self.assertEqual(['prepare','gripper','move','stop'],[c[0] for c in self.fixture.driver.calls])

    async def test_group_timeout_and_missing_coordination_ack_invalidate_plan(self):
        plan = await self.plan()
        self.fixture.driver.confirm_coordination = False
        body = dict(plan_id=plan['plan_id'],stage='pick')
        response = await self.http.post('/v1/plans/execute',json=body)
        self.assertEqual(502,response.status_code)
        self.assertEqual('unknown',response.json()['outcome'])
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=body)).status_code)
        self.assertEqual(409,(await self.http.post('/v1/move', json=dict(all_arms=True, targets=[dict(kind='named',name='home')]))).status_code)
        await self.http.post('/v1/stop',json={'arm_ids':['left']})
        self.assertEqual(409,(await self.http.post('/v1/move', json=dict(all_arms=True, targets=[dict(kind='named',name='home')]))).status_code)
        await self.http.post('/v1/stop',json={})
        self.fixture.driver.confirm_coordination = True
        self.assertTrue((await self.http.post('/v1/arms/recover')).json()['success'])
        plan = await self.plan()
        self.fixture.driver.hold_moves = True
        reply = await self.fixture.gateway.request('execute_plan','',dict(plan_id=plan['plan_id'],stage='pick'),.1)
        self.assertEqual(504,reply.status)
        self.assertEqual('invalid',self.fixture.robot.pixels.plans[plan['plan_id']]['state'])
        await self.http.post('/v1/stop',json={})

    async def test_reset_and_explicit_dual_pose_are_single_group_requests(self):
        plan = await self.plan()
        reset = await self.http.post('/v1/move', json=dict(all_arms=True, targets=[dict(kind='named',name='home')]))
        self.assertTrue(reset.json()['success'])
        self.assertEqual(['left','right'],self.fixture.driver.calls[0][2]['arm_ids'])
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=dict(plan_id=plan['plan_id'],stage='pick'))).status_code)
        moves = [dict(arm_id=arm,frame_id='world',position=[x,0.,1.],orientation=[0.,0.,0.,1.])
                 for arm,x in [('left',-.2),('right',.2)]]
        response = await self.http.post('/v1/move', json=dict(arm_ids=['left','right'],
            targets=[dict(kind='pose',**{k:v for k,v in m.items() if k!='arm_id'}) for m in moves]))
        self.assertTrue(response.json()['success'])
        self.assertEqual(['prepare','move','prepare','move'],[c[0] for c in self.fixture.driver.calls])
        self.assertEqual(2,len(self.fixture.driver.calls[-1][2]['targets']))

    async def test_refinement_rejects_wrong_wrist_and_preserves_release(self):
        plan = await self.plan()
        await self.http.post('/v1/plans/execute',json=dict(plan_id=plan['plan_id'],stage='approach'))
        wrong = await self.capture('right_wrist')
        body = dict(plan_id=plan['plan_id'],arm_id='left',capture_id=wrong['capture_id'],pixel=[24,16])
        self.assertEqual(422,(await self.http.post('/v1/plans/refine',json=body)).status_code)
        correct = await self.capture('left_wrist')
        response = await self.http.post('/v1/plans/refine',json=dict(body,capture_id=correct['capture_id']))
        self.assertEqual(200,response.status_code)
        target = response.json()['targets'][0]
        self.assertEqual(plan['targets'][0]['release'],target['release'])
        self.assertEqual([1.,0.,1.],target['grasp']['position'])

    async def test_live_agent_failed_assessment_blocks_real_server_place(self):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        def er(request):
            prompt = json.loads(request.content)['contents'][0]['parts'][1]['text']
            if 'Locate grasp' in prompt:
                result = {'targets':[dict(arm_id=arm,grasp=[500,x],release=[750,x])
                                     for arm,x in [('left',250),('right',750)]]}
            elif 'Refine the grasp' in prompt:
                result = {'point':[500,500]}
            else:
                raise AssertionError('ER cannot assess grasp')
            return httpx.Response(200,json={'candidates':[{'finishReason':'STOP',
                'content':{'parts':[{'text':json.dumps(result)}]}}]})
        stream = WorkflowStream(assessment_success=False)
        with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
            client.return_value.create_stream.return_value = stream
            result = await run_application(AgentConfig.load(ROOT/'agent/configs/dual_arm.json'),
                'Move the bar only after confirming both grasps.',model='mock-live',api_key='mock-key',
                timeout=30,transport=httpx.ASGITransport(app=self.app),robotics_transport=httpx.MockTransport(er))
        self.assertFalse(result['success'])
        responses = {r['name']:r['response'] for m in stream.messages if 'toolResponse' in m
                     for r in m['toolResponse']['functionResponses']}
        self.assertFalse(responses['verify_grasp']['success'])
        self.assertFalse(responses['place_targets']['success'])
        stages = [payload['steps'][-1].get('phase') for op,_,payload in self.fixture.driver.calls if op == 'prepare' and payload['steps'][-1].get('phase')]
        self.assertEqual(['approach','lift'],stages)
        self.assertEqual('stop',self.fixture.driver.calls[-1][0])
        self.assertEqual('invalid',next(iter(self.fixture.robot.pixels.plans.values()))['state'])

    async def test_live_agent_er_http_and_ros_complete_two_dual_arm_cycles(self):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        er_requests = []
        def er(request):
            body = json.loads(request.content)
            er_requests.append(body)
            prompt = body['contents'][0]['parts'][1]['text']
            if 'Locate grasp' in prompt:
                result = {'targets':[dict(arm_id='left',grasp=[500,250],release=[750,250]),
                                     dict(arm_id='right',grasp=[500,750],release=[750,750])]}
            elif 'Refine the grasp' in prompt:
                result = {'point':[500,500]}
            else:
                raise AssertionError('Grasp assessment must not call Robotics ER')
            return httpx.Response(200,json={'candidates':[{'finishReason':'STOP','content':{'parts':[{'text':json.dumps(result)}]}}]})
        stream = WorkflowStream()
        with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
            client.return_value.create_stream.return_value = stream
            result = await run_application(AgentConfig.load(ROOT/'agent/configs/dual_arm.json'),
                'Move the bar with both arms, then inspect and repeat.',model='mock-live',api_key='mock-key',
                timeout=30, transport=httpx.ASGITransport(app=self.app), robotics_transport=httpx.MockTransport(er))
        self.assertTrue(result['success'])
        responses = [m['toolResponse']['functionResponses'][0] for m in stream.messages if 'toolResponse' in m]
        self.assertTrue(all(r['response'].get('success',True) for r in responses), responses)
        self.assertEqual(3,len(er_requests))
        inspected = 0
        for i, message in enumerate(stream.messages):
            if "toolResponse" not in message:
                continue
            response = message["toolResponse"]["functionResponses"][0]
            if response["name"] == "inspect_grasp":
                inspected += 1
                capture = response["response"]
                raw = base64.b64decode(stream.messages[i-1]["realtimeInput"]["video"]["data"])
                self.assertEqual(self.fixture.driver.images[capture["camera_id"]],raw)
                self.assertNotIn("image_base64",capture)
        self.assertEqual(4,inspected)
        prepared = [payload for operation,_,payload in self.fixture.driver.calls if operation=='prepare']
        self.assertEqual([None,'approach','lift','retreat','lift','retreat'],[p['steps'][-1].get('phase') for p in prepared])
        for payload in prepared:
            self.assertEqual(['left','right'],payload['arm_ids'])
            if payload['steps'][-1].get('phase'):
                self.assertTrue(payload['coupled'])
        plans = list(self.fixture.robot.pixels.plans.values())
        self.assertEqual(['placed','placed'],[p['state'] for p in plans])
        refined = plans[0]['targets'][0]['grasp']
        self.assertGreater(refined['position'][0],.9)
        self.assertNotEqual(plans[0]['targets'][0]['grasp']['capture_id'],plans[0]['targets'][0]['release']['capture_id'])
