"""Response-driven Gemini scenarios through the real agent, HTTP and ROS bridge."""

import asyncio
import base64
import copy
import io
import json
import unittest
from unittest import mock

import httpx
from PIL import Image

from helpers import ROOT, RosFixture, MockGeminiStream, config, jpeg
from ros2_agent_server.api import create_app
from ros2_agent_server.protocol import Reply


class ScenarioStream(MockGeminiStream):
    """A bounded model script that must inspect each real response before advancing."""
    def __init__(self, scenario, arm_ids, hook):
        super().__init__([])
        self.scenario, self.arms, self.hook = scenario, arm_ids, hook
        self.flow = self.script()
        self.steps = 0
        self.pending = None
        self.results = []
        self.plan_ids = []
        self.evidence_ids = []
        self.color = None
        self.failure = None
        self.completed = False

    def Start(self, on_message, on_done):
        super().Start(on_message, on_done)
        self.on_done = on_done

    def Send(self, message):
        self.messages.append(copy.deepcopy(message))
        if 'realtimeInput' in message and 'video' in message['realtimeInput']:
            raw = base64.b64decode(message['realtimeInput']['video']['data'])
            with Image.open(io.BytesIO(raw)) as image:
                self.color = image.convert('RGB').getpixel((image.width//2,image.height//2))
        if 'setup' in message:
            self.on_message({'setupComplete': {}})
        elif 'clientContent' in message or 'toolResponse' in message:
            try:
                result = None
                if 'toolResponse' in message:
                    responses = message['toolResponse']['functionResponses']
                    assert len(responses) == 1
                    response = responses[0]
                    assert (response['id'], response['name']) == (self.pending['id'], self.pending['name'])
                    result = response['response']
                    assert isinstance(result, dict)
                    self.results.append((response['name'], copy.deepcopy(result)))
                    self.hook(response['name'], result)
                call = self.flow.send(result)
                self.steps += 1
                assert self.steps <= 40, 'Scenario exceeded its step budget'
                self.pending = call
                if self.scenario == 'disconnect' and call['name'] == 'detect_targets':
                    self.on_done()
                elif self.scenario == 'duplicate' and call['name'] == 'detect_targets':
                    duplicate = dict(call, id='step1')
                    self.on_message({'toolCall': {'functionCalls': [duplicate]}})
                else:
                    self.on_message({'toolCall': {'functionCalls': [call]}})
            except StopIteration:
                self.completed = True
            except BaseException as exc:
                self.failure = exc
                self.on_done()

    def ask(self, name, **args):
        result = yield dict(id=f'step{self.steps+1}', name=name, args=args)
        assert isinstance(result, dict), f'{name}: missing response'
        return result

    def finish(self, success, reason):
        result = yield from self.ask('finish_task', success=success, summary=reason)
        assert result['success'] is success, result

    def recover(self, failure):
        assert failure.get('success') is False, failure
        assert failure.get('error'), failure
        if self.scenario in ('phase_failure', 'unrecoverable', 'recovery_limit', 'unsupported_recovery', 'stop_failure'):
            assert failure['failed_phase'] == 'close' and failure['failed_arm_ids'] == self.arms
        if self.scenario == 'motion_timeout':
            assert failure['outcome'] == 'unknown'
        result = yield from self.ask('recover_arms')
        if result['success']:
            assert result['recovery_attempts_remaining'] >= 0
            observed = yield from self.ask('get_robot_state')
            assert observed['success'] and not observed['recovery_required']
            return True
        assert result.get('error'), result
        yield from self.finish(False, 'Recovery unavailable: ' + result['error'])
        return False

    def script(self):
        initial = yield from self.ask('get_robot_state')
        assert initial['success'] and initial['post_action_observation']
        reset = yield from self.ask('reset_arms', **({'arm_id': 'missing'} if self.scenario=='invalid_reset' else {}))
        if not reset['success']:
            assert self.scenario == 'invalid_reset'
            yield from self.finish(False, 'Rejected invalid reset arguments')
            return
        if self.scenario == 'missing_state':
            state = yield from self.ask('get_robot_state')
            assert state['success'] is False and state['http_status'] == 503
            yield from self.finish(False, 'Current state unavailable')
            return
        for cycle in range(2):
            detected = yield from self.ask('detect_targets', camera_id='overhead', instruction='Move the block onto the tray', arm_ids=self.arms)
            if detected['success'] is False:
                assert detected.get('error')
                if cycle == 0 and self.scenario in ('er_invalid', 'er_coordinates', 'er_json', 'er_timeout'):
                    continue
                yield from self.finish(False, 'Detection unavailable')
                return
            assert detected['state'] == 'detected'
            plan_id = detected['plan_id']
            assert plan_id not in self.plan_ids, 'A retry reused an old plan'
            self.plan_ids.append(plan_id)
            picked = yield from self.ask('pick_targets', plan_id=plan_id)
            if not picked['success']:
                if (yield from self.recover(picked)):
                    continue
                return
            assert picked['state'] == 'picked'
            if self.scenario == 'manual_held' and cycle == 0:
                moved = yield from self.ask('move', arm_ids=self.arms, targets=[dict(kind='named',name='ready')])
                assert moved['success'] is False
                premature = yield from self.ask('finish_task', success=True, summary='Incorrect early success')
                assert premature['success'] is False
                if (yield from self.recover(moved)):
                    continue
                return
            for assessment_attempt in range(2):
                observations = []
                for arm in self.arms:
                    inspected = yield from self.ask('inspect_grasp', plan_id=plan_id, arm_id=arm)
                    assert inspected['success'], inspected
                    assert inspected['observation_id'] not in self.evidence_ids
                    self.evidence_ids.append(inspected['observation_id'])
                    assert inspected['stamp_ns'] > picked['motion_stamp_ns']
                    # The mock uses an explicit image fixture as its visual evidence.
                    red, green, blue = self.color
                    held = green > red+40 and green > blue+40
                    observations.append(dict(arm_id=arm, observation_id=inspected['observation_id'],
                        success=held, reason='Object visibly held' if held else 'Occluded; grasp is uncertain'))
                verified = yield from self.ask('verify_grasp', plan_id=plan_id, observations=observations)
                if verified['success']:
                    assert all(o['success'] for o in observations)
                    break
                assert verified.get('error')
                if self.scenario == 'reinspection' and assessment_attempt == 0:
                    continue
                if self.scenario == 'sensor_conflict':
                    assert verified['failures'][0]['code'] == 'no_object'
                if (yield from self.recover(verified)):
                    break
                return
            if not verified['success']:
                continue
            placed = yield from self.ask('place_targets', plan_id=plan_id)
            assert placed['success'] and placed['state'] == 'placed', placed
            premature = yield from self.ask('finish_task', success=True, summary='No final observation yet')
            assert premature['success'] is False
            state = yield from self.ask('get_robot_state')
            if not state.get('success'):
                assert self.scenario == 'image_failure'
                yield from self.finish(False, 'Final scene could not be observed')
                return
            assert state['post_action_observation']
            red, green, blue = self.color
            achieved = blue > red+40 and blue > green+40
            yield from self.finish(achieved, 'Object observed on tray' if achieved else 'Object is outside destination')
            return
        yield from self.finish(False, 'Scenario retry budget exhausted')


class ScenarioTest(unittest.IsolatedAsyncioTestCase):
    async def run_scenario(self, scenario):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        topology = 'dual_arm.json' if scenario=='group_partial' else 'minimal.json'
        fixture = RosFixture(config(topology))
        arms = [a.id for a in fixture.config.arms]
        er_calls = []
        injected = False
        try:
            await fixture.ready()
            original = fixture.driver.execute
            async def execute(request, response):
                nonlocal injected
                body = json.loads(request.payload_json)
                if request.operation == 'stop' and scenario == 'stop_failure':
                    fixture.driver.calls.append((request.operation, request.resource_id, body))
                    return Reply(payload=dict(success=False,error='Stop not confirmed')).to_ros(response)
                if request.operation == 'recover' and scenario == 'unsupported_recovery':
                    fixture.driver.calls.append((request.operation, request.resource_id, body))
                    return Reply(status=501,payload=dict(success=False,error='Recovery unsupported',recoverable=False)).to_ros(response)
                pick = request.operation == 'gripper' and body.get('phase') == 'close'
                if pick and scenario == 'disconnect_motion':
                    fixture.driver.hold_moves = True
                    stream.on_done()
                if pick and not injected and scenario in ('phase_failure','unrecoverable','unsupported_recovery','stop_failure','recovery_limit','group_partial','motion_timeout'):
                    injected = True
                    if scenario == 'motion_timeout':
                        fixture.driver.hold_moves = True
                    else:
                        fixture.driver.reply_status = 409 if scenario!='group_partial' else 200
                        fixture.driver.reply_payload = dict(success=False,error='Jaw jam',code='jam',failed_phase='close',
                            failed_arm_ids=arms,recoverable=scenario!='unrecoverable')
                        if scenario == 'unrecoverable':
                            fixture.driver.states[arms[0]]['fault'] = dict(code='drive_fault',message='Operator required',recoverable=False)
                            fixture.driver.publish()
                        if scenario == 'group_partial':
                            fixture.driver.confirm_coordination = False
                            fixture.driver.reply_payload = dict(success=True,coordinated=True,completed_arm_ids=[arms[0]])
                        try:
                            return await original(request,response)
                        finally:
                            fixture.driver.reply_status=200
                            fixture.driver.reply_payload={'success':True}
                            fixture.driver.confirm_coordination=True
                if request.operation == 'stop':
                    fixture.driver.hold_moves = False
                reply = await original(request,response)
                if request.operation == 'move' and body.get('phase') == 'lift':
                    color = 'gray' if scenario in ('reinspection','uncertain') else 'green'
                    for camera in fixture.config.cameras: fixture.driver.images[camera.id] = jpeg(color)
                if request.operation=='move' and body.get('phase')=='retreat':
                    for camera in fixture.config.cameras:
                        fixture.driver.images[camera.id] = jpeg('red' if scenario=='placement_miss' else 'blue')
                return reply
            fixture.driver.execute = execute
            if scenario == 'motion_timeout':
                original_request = fixture.gateway.request
                async def short_deadline(operation,resource_id,payload,timeout):
                    nonlocal injected
                    if operation=='execute_plan' and not injected:
                        injected=True
                        fixture.driver.hold_moves=True
                        timeout=.15
                    return await original_request(operation,resource_id,payload,timeout)
                fixture.gateway.request = short_deadline
            def hook(name,result):
                if name=='reset_arms':
                    if scenario=='missing_depth': fixture.driver.publish_geometry=False; fixture.robot._depth.clear()
                    if scenario=='missing_state': fixture.driver.publish_states=False; fixture.robot.store._arms.clear()
                if name=='inspect_grasp' and scenario=='sensor_conflict':
                    fixture.driver.states[arms[0]]['gripper']['object_detected']=False
                    fixture.driver.publish()
                if name=='verify_grasp' and scenario=='reinspection' and result.get('success') is False:
                    fixture.driver.images['overhead']=jpeg('green')
                if name=='place_targets' and scenario=='image_failure': fixture.driver.publish_images=False
            stream = ScenarioStream(scenario,arms,hook)
            events = []
            async def er(request):
                er_calls.append(json.loads(request.content))
                if scenario=='er_timeout' and len(er_calls)==1:
                    raise httpx.ReadTimeout('Mock ER timeout',request=request)
                result={'targets':[dict(arm_id=a,grasp=[500,500],release=[750,750]) for a in arms]}
                if scenario=='er_invalid' and len(er_calls)==1: result={'targets':[]}
                if scenario=='er_coordinates' and len(er_calls)==1: result['targets'][0]['grasp']=[-1,500]
                return httpx.Response(200,json={'candidates':[{'finishReason':'STOP','content':{'parts':[{'text':'invalid json' if scenario=='er_json' and len(er_calls)==1 else json.dumps(result)}]}}]})
            with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
                client.return_value.create_stream.return_value=stream
                kwargs=dict(model='mock-live',api_key='mock-key',timeout=20,on_event=events.append,
                    transport=httpx.ASGITransport(app=create_app(fixture.gateway,fixture.config)),
                    robotics_transport=httpx.MockTransport(er),max_recovery_attempts=0 if scenario=='recovery_limit' else 1)
                if scenario in ('disconnect','duplicate','disconnect_motion'):
                    with self.assertRaises(RuntimeError):
                        await run_application(AgentConfig.load(ROOT/'agent/configs'/topology),'Move block to tray',**kwargs)
                    self.assertEqual(1,client.return_value.create_stream.call_count)
                    self.assertTrue(any(c[0]=='stop' for c in fixture.driver.calls))
                    if scenario != 'disconnect_motion': self.assertFalse(er_calls)
                    else: self.assertEqual(1, sum(c[0]=='gripper' and c[2].get('phase')=='close' for c in fixture.driver.calls))
                else:
                    result = await run_application(AgentConfig.load(ROOT/'agent/configs'/topology),'Move block to tray',**kwargs)
                    if stream.failure: raise stream.failure
                    expected = scenario in ('success','reinspection','phase_failure','er_invalid','er_coordinates','er_json','er_timeout','manual_held','motion_timeout','group_partial')
                    self.assertEqual(expected,result['success'],(scenario,result))
                    self.assertTrue(stream.completed)
                    if scenario=='stop_failure':
                        self.assertTrue(result['operator_required'])
                        self.assertFalse(result['stop_result']['success'])
            if stream.failure: raise stream.failure
            wire = {r['id']: r['response'] for m in stream.messages if 'toolResponse' in m
                    for r in m['toolResponse']['functionResponses']}
            for event in events:
                if event.get('type') == 'tool_call':
                    self.assertEqual(wire[event['id']],event['result'])
            self.assertTrue(stream.closed)
            self.assertLessEqual(stream.steps,40)
            calls=fixture.driver.calls
            if scenario in ('missing_depth','missing_state','invalid_reset','disconnect','duplicate'):
                self.assertFalse(any(c[0]=='prepare' and c[2]['steps'][-1].get('phase')=='lift' for c in calls))
            if scenario in ('stop_failure','recovery_limit'):
                self.assertFalse(any(c[0]=='recover' for c in calls))
            if scenario in ('uncertain','sensor_conflict','unrecoverable','unsupported_recovery'):
                self.assertFalse(any(c[0]=='move' and c[2].get('phase')=='retreat' for c in calls))
            if scenario=='group_partial':
                self.assertEqual(2,len(stream.plan_ids))
                self.assertFalse(any(c[0]=='move' and c[2].get('phase')=='retreat' and calls.index(c) < next(i for i,v in enumerate(calls) if v[0]=='recover') for c in calls))
                stops=[c for c in calls if c[0]=='stop']
                self.assertTrue(stops)
                self.assertEqual(arms,stops[0][2]['arm_ids'])
            if scenario=='manual_held':
                self.assertEqual(1,sum(c[0]=='move' and 'phase' not in c[2] for c in calls))
            if scenario=='motion_timeout':
                self.assertEqual(2,sum(c[0]=='prepare' and c[2]['steps'][-1].get('phase')=='lift' for c in calls))
            if scenario in ('er_invalid','er_coordinates','er_json','er_timeout'):
                self.assertEqual(2,len(er_calls))
                self.assertNotEqual(er_calls[0],er_calls[1])
        finally:
            fixture.close()

    async def test_success_failure_and_recovery_scenarios(self):
        for scenario in ('success','reinspection','uncertain','phase_failure','unrecoverable',
                         'unsupported_recovery','stop_failure','recovery_limit','er_invalid','er_coordinates','er_json','er_timeout',
                         'missing_depth','missing_state','image_failure','sensor_conflict','placement_miss',
                         'group_partial','motion_timeout','disconnect','duplicate','disconnect_motion','manual_held','invalid_reset'):
            with self.subTest(scenario=scenario):
                await self.run_scenario(scenario)
