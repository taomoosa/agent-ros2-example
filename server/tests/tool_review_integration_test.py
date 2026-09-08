import json
import tempfile
from pathlib import Path
import unittest
from unittest import mock

import httpx

from helpers import ROOT, RosFixture, MockGeminiStream, config, eventually
from ros2_agent_server.api import create_app


class MinimalStream(MockGeminiStream):
    def __init__(self, retry=False):
        self.plan_id = None
        self.observation_id = None
        self.retry = retry
        super().__init__(self.script())

    def Send(self, message):
        if 'toolResponse' in message:
            result = message['toolResponse']['functionResponses'][0]['response']
            self.plan_id = result.get('plan_id',self.plan_id)
            self.observation_id = result.get('observation_id',self.observation_id)
        super().Send(message)

    def script(self):
        yield dict(id='reset',name='reset_arms',args={})
        for cycle in range(2 if self.retry else 1):
            yield dict(id=f'detect{cycle}',name='detect_targets',args=dict(
                camera_id='overhead',instruction='Move the blue block onto the tray',arm_ids=['arm']))
            yield dict(id=f'pick{cycle}',name='pick_targets',args=dict(plan_id=self.plan_id))
            if self.retry and cycle == 0:
                yield dict(id='recover',name='recover_arms',args={})
                yield dict(id='state',name='get_robot_state',args={})
                continue
            yield dict(id='inspect',name='inspect_grasp',args=dict(plan_id=self.plan_id,arm_id='arm'))
            yield dict(id='verify',name='verify_grasp',args=dict(plan_id=self.plan_id,observations=[
                dict(arm_id='arm',observation_id=self.observation_id,success=True,
                     reason='The fixed-camera view shows the block held above the table.')]))
            yield dict(id='place',name='place_targets',args=dict(plan_id=self.plan_id))
        yield dict(id='state-final',name='get_robot_state',args={})
        yield dict(id='finish',name='finish_task',args=dict(success=True,summary='Block placed and scene checked'))


class ToolReviewIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.fixture = RosFixture(config('minimal.json'))
        self.addCleanup(self.fixture.close)
        await self.fixture.ready()
        self.app = create_app(self.fixture.gateway,self.fixture.config)
        self.http = httpx.AsyncClient(transport=httpx.ASGITransport(app=self.app),base_url='http://test')
        self.addAsyncCleanup(self.http.aclose)

    async def run_minimal(self, *, retry=False):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        er_requests = []
        def er(request):
            prompt = json.loads(request.content)['contents'][0]['parts'][1]['text']
            er_requests.append(prompt)
            result = {'targets':[dict(arm_id='arm',grasp=[500,500],release=[750,750])]}
            return httpx.Response(200,json={'candidates':[{'finishReason':'STOP',
                'content':{'parts':[{'text':json.dumps(result)}]}}]})
        if retry:
            original = self.fixture.driver.execute
            failed = False
            async def fail_once(request,response):
                nonlocal failed
                if request.operation == 'execute_plan' and json.loads(request.payload_json)['stage'] == 'pick' and not failed:
                    failed = True
                    self.fixture.driver.reply_status = 409
                    self.fixture.driver.reply_payload = dict(success=False,error='Transient jaw jam',
                        failed_phase='close',failed_arm_ids=['arm'],recoverable=True)
                    try:
                        return await original(request,response)
                    finally:
                        self.fixture.driver.reply_status = 200
                        self.fixture.driver.reply_payload = {'success':True}
                return await original(request,response)
            self.fixture.driver.execute = fail_once
        stream = MinimalStream(retry=retry)
        with tempfile.TemporaryDirectory() as directory:
            prompt_file = Path(directory)/'custom.md'
            prompt_file.write_text('Choose the textured face of the blue block.')
            with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
                client.return_value.create_stream.return_value = stream
                result = await run_application(AgentConfig.load(ROOT/'agent/configs/minimal.json'),
                    'Move block onto tray.',model='mock-live',api_key='mock-key',timeout=30,
                    transport=httpx.ASGITransport(app=self.app),robotics_transport=httpx.MockTransport(er),
                    er_prompt_files={'detect':prompt_file})
        self.assertTrue(result['success'])
        self.assertEqual(2 if retry else 1,len(er_requests))
        self.assertTrue(all('Choose the textured face' in p for p in er_requests))
        responses = {r['id']:r['response'] for m in stream.messages if 'toolResponse' in m
                     for r in m['toolResponse']['functionResponses']}
        self.assertEqual(['pick_targets'],responses['detect0']['next_actions'])
        self.assertEqual('overhead',responses['inspect']['camera_id'])
        self.assertEqual('placed',responses['place']['state'])
        self.assertEqual(1,len(self.fixture.config.cameras))
        declarations = {d['name'] for d in stream.messages[0]['setup']['tools'][0]['functionDeclarations']}
        self.assertNotIn('refine_grasp',declarations)
        if retry:
            self.assertFalse(responses['pick0']['success'])
            self.assertEqual('close',responses['pick0']['failed_phase'])
            self.assertTrue(responses['recover']['success'])
            self.assertNotEqual(responses['detect0']['plan_id'],responses['detect1']['plan_id'])
            self.assertEqual(['reset_arms','execute_plan','stop','recover_arms','execute_plan','execute_plan'],
                             [c[0] for c in self.fixture.driver.calls])
        else:
            self.assertTrue(all(r.get('success',True) for r in responses.values()))

    async def test_one_fixed_camera_one_arm_completes_without_wrist(self):
        await self.run_minimal()

    async def test_gripper_failure_recovers_and_retries_with_new_plan(self):
        await self.run_minimal(retry=True)

    async def test_fault_telemetry_blocks_motion_and_unrecoverable_recovery(self):
        for component in ('arm','gripper'):
            fault = dict(code='drive_fault',message='Mock component fault',recoverable=False)
            state = self.fixture.driver.states['arm']
            state['fault'] = fault if component == 'arm' else None
            state['gripper']['fault'] = fault if component == 'gripper' else None
            self.fixture.driver.publish()
            await eventually(lambda: (self.fixture.robot.store._arms['arm'][0].get('fault') if component == 'arm'
                else self.fixture.robot.store._arms['arm'][0]['gripper'].get('fault')) == fault)
            response = await self.http.post('/v1/arms/arm/pose',json=dict(frame_id='world',position=[0.,0.,1.],orientation=[0.,0.,0.,1.]))
            self.assertEqual(409,response.status_code)
            await self.http.post('/v1/stop',json={})
            recovered = await self.http.post('/v1/arms/recover')
            self.assertEqual(409,recovered.status_code)
            self.assertFalse(recovered.json()['recoverable'])
            self.assertEqual(component,recovered.json()['failures'][0]['component'])
        self.assertTrue(all(c[0]=='stop' for c in self.fixture.driver.calls))

    async def test_recovery_requires_stop_and_known_recoverable_fault_can_clear(self):
        self.assertEqual(409,(await self.http.post('/v1/arms/recover')).status_code)
        fault = dict(code='transient',message='Mock recoverable arm fault',recoverable=True)
        self.fixture.driver.states['arm']['fault'] = fault
        self.fixture.driver.publish()
        await eventually(lambda: self.fixture.robot.store._arms['arm'][0].get('fault') == fault)
        await self.http.post('/v1/stop',json={})
        result = await self.http.post('/v1/arms/recover')
        self.assertTrue(result.json()['success'])
        self.assertEqual(['secure_or_support_payload','release','retreat','home'],self.fixture.driver.calls[-1][2]['phases'])
        await eventually(lambda: self.fixture.robot.store._arms['arm'][0].get('fault') is None)
        self.assertFalse((await self.http.get('/v1/state')).json()['recovery_required'])

    async def test_negative_object_sensor_overrides_positive_visual_assessment(self):
        capture = (await self.http.get('/v1/cameras/overhead/capture')).json()
        plan = (await self.http.post('/v1/plans',json=dict(capture_id=capture['capture_id'],
            targets=[dict(arm_id='arm',grasp=[24,16],release=[30,24])]))).json()
        await self.http.post('/v1/plans/execute',json=dict(plan_id=plan['plan_id'],stage='pick'))
        image = (await self.http.get('/v1/cameras/overhead/capture')).json()
        self.fixture.driver.states['arm']['gripper']['object_detected'] = False
        self.fixture.driver.publish()
        await eventually(lambda: self.fixture.robot.store._arms['arm'][0]['gripper'].get('object_detected') is False)
        response = await self.http.post('/v1/plans/verify',json=dict(plan_id=plan['plan_id'],observations=[
            dict(arm_id='arm',capture_id=image['capture_id'],success=True)]))
        self.assertEqual(409,response.status_code)
        self.assertEqual('no_object',response.json()['failures'][0]['code'])
        self.assertEqual(409,(await self.http.post('/v1/plans/execute',json=dict(plan_id=plan['plan_id'],stage='place'))).status_code)
