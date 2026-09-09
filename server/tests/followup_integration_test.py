"""ROS/HTTP regression tests for input and physical outcome admission."""

import json
import unittest

import httpx

from helpers import RosFixture, config, eventually
from ros2_agent_server.api import create_app
from ros2_agent_server.protocol import Reply


class FollowupIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def test_manual_motion_cannot_forget_held_plan_with_or_without_sensor(self):
        for topology in ('minimal.json', 'dual_arm.json'):
            fixture = RosFixture(config(topology))
            try:
                await fixture.ready()
                async with httpx.AsyncClient(transport=httpx.ASGITransport(
                        app=create_app(fixture.gateway, fixture.config)), base_url='http://test') as http:
                    arms = [a.id for a in fixture.config.arms]
                    for sensor in (True, None):
                        for operation in ('poses', 'reset', 'gripper', 'pose'):
                            with self.subTest(topology=topology, sensor=sensor, operation=operation):
                                capture = (await http.get('/v1/cameras/overhead/capture')).json()
                                body = dict(capture_id=capture['capture_id'], targets=[
                                    dict(arm_id=arm, grasp=[12+i*12,16], release=[12+i*12,24])
                                    for i,arm in enumerate(arms)])
                                plan = (await http.post('/v1/plans', json=body)).json()
                                picked = await http.post('/v1/plans/execute', json=dict(plan_id=plan['plan_id'], stage='pick'))
                                self.assertTrue(picked.json()['success'])
                                for arm in arms:
                                    fixture.driver.states[arm]['gripper']['object_detected'] = sensor
                                fixture.driver.publish()
                                await eventually(lambda: all(fixture.robot.store._arms[a][0]['gripper'].get('object_detected') is sensor for a in arms))
                                pose = dict(frame_id='world', position=[0.,0.,.3], orientation=[0.,0.,0.,1.])
                                path = '/v1/arms/' + (operation if operation in ('poses','reset') else arms[0]+'/'+operation)
                                body = dict(moves=[dict(pose,arm_id=a) for a in arms]) if operation=='poses' else (
                                    {} if operation=='reset' else {'opening':1.} if operation=='gripper' else pose)
                                count = len(fixture.driver.calls)
                                denied = await http.post(path, json=body)
                                self.assertEqual(409, denied.status_code, denied.text)
                                self.assertEqual(count, len(fixture.driver.calls))
                                self.assertTrue((await http.get('/v1/state')).json()['recovery_required'])
                                forbidden = await http.post('/v1/plans', json=dict(
                                    capture_id=capture['capture_id'], targets=[dict(arm_id=arms[0], grasp=[24,16], release=[24,24])]))
                                self.assertEqual(409, forbidden.status_code)
                                await http.post('/v1/stop', json={})
                                recovered = await http.post('/v1/arms/recover', json={})
                                self.assertTrue(recovered.json()['success'], recovered.text)
                                self.assertFalse((await http.get('/v1/state')).json()['recovery_required'])
            finally:
                fixture.close()

    async def test_invalid_empty_commands_never_reach_driver_over_http_or_ros(self):
        fixture = RosFixture(config('dual_arm.json'))
        try:
            await fixture.ready()
            async with httpx.AsyncClient(transport=httpx.ASGITransport(
                    app=create_app(fixture.gateway,fixture.config)),base_url='http://test') as http:
                for operation in ('reset_arms','recover_arms'):
                    for payload in ({'arm_id':'left'}, None, [], False, 42, ''):
                        raw = json.dumps(payload)
                        route = '/v1/arms/' + ('reset' if operation=='reset_arms' else 'recover')
                        self.assertEqual(422,(await http.post(route,content=raw)).status_code)
                        result = await fixture.gateway.request(operation,'',payload,1.)
                        self.assertEqual(422,result.status)
                self.assertEqual([],fixture.driver.calls)
        finally:
            fixture.close()

    async def test_place_or_recover_cannot_succeed_with_a_positive_held_sensor(self):
        fixture = RosFixture(config('minimal.json'))
        try:
            await fixture.ready()
            async with httpx.AsyncClient(transport=httpx.ASGITransport(
                    app=create_app(fixture.gateway,fixture.config)),base_url='http://test') as http:
                capture=(await http.get('/v1/cameras/overhead/capture')).json()
                plan=(await http.post('/v1/plans',json=dict(capture_id=capture['capture_id'],targets=[
                    dict(arm_id='arm',grasp=[24,16],release=[24,24])]))).json()
                await http.post('/v1/plans/execute',json=dict(plan_id=plan['plan_id'],stage='pick'))
                image=(await http.get('/v1/cameras/overhead/capture')).json()
                await http.post('/v1/plans/verify',json=dict(plan_id=plan['plan_id'],observations=[
                    dict(arm_id='arm',capture_id=image['capture_id'],success=True)]))
                await eventually(lambda: fixture.robot.store._arms['arm'][0]['gripper'].get('object_detected') is True)
                original=fixture.driver.execute
                async def false_completion(request,response):
                    if request.operation in ('execute_plan','recover_arms'):
                        return Reply(payload=dict(success=True,coordinated=True,completed_arm_ids=['arm'])).to_ros(response)
                    return await original(request,response)
                fixture.driver.execute=false_completion
                result=await http.post('/v1/plans/execute',json=dict(plan_id=plan['plan_id'],stage='place'))
                self.assertEqual(409,result.status_code)
                self.assertEqual('object_not_released',result.json()['failures'][0]['code'])
                await http.post('/v1/stop',json={})
                result=await http.post('/v1/arms/recover',json={})
                self.assertEqual(409,result.status_code)
                self.assertTrue((await http.get('/v1/state')).json()['recovery_required'])
        finally:
            fixture.close()

    async def test_late_all_arm_stop_cannot_override_a_newer_failed_stop(self):
        import asyncio
        fixture = RosFixture(config('dual_arm.json'))
        try:
            await fixture.ready()
            entered, release = asyncio.Event(), asyncio.Event()
            async def command(operation,resource,payload,timeout, **kwargs):
                if payload['arm_id'] is None:
                    entered.set()
                    await release.wait()
                    return Reply(payload={'success':True})
                return Reply(payload={'success':False,'error':'Stop failed'})
            fixture.robot._command = command
            old = asyncio.create_task(fixture.robot._workflow('stop','',{'arm_id':None},1.))
            await asyncio.wait_for(entered.wait(),1.)
            newer = await fixture.robot._workflow('stop','',{'arm_id':'left'},1.)
            self.assertFalse(newer.payload['success'])
            release.set()
            late = await asyncio.wait_for(old,1.)
            self.assertEqual(409,late.status)
            self.assertTrue(fixture.robot._motion_uncertain)
            self.assertFalse(fixture.robot._all_stopped)
        finally:
            fixture.close()

    async def test_moving_telemetry_prevents_new_motion(self):
        fixture = RosFixture(config('minimal.json'))
        try:
            await fixture.ready()
            fixture.driver.states['arm']['moving']=True
            fixture.driver.publish()
            await eventually(lambda: fixture.robot.store._arms['arm'][0]['moving'])
            reply = await fixture.gateway.request('move_arm','arm',dict(frame_id='world',
                position=[0.,0.,.3],orientation=[0.,0.,0.,1.]),1.)
            self.assertEqual(409,reply.status)
            self.assertEqual('arm_moving',reply.payload['failures'][0]['code'])
            self.assertEqual([],fixture.driver.calls)
        finally:
            fixture.close()

    async def test_successful_single_arm_stop_preserves_known_idle_outcome(self):
        fixture = RosFixture(config('dual_arm.json'))
        try:
            await fixture.ready()
            result = await fixture.gateway.request('stop','',{'arm_id':'left'},1.)
            self.assertTrue(result.payload['success'])
            state = await fixture.gateway.request('state','',{},1.)
            self.assertFalse(state.payload['motion_outcome_unknown'])
            self.assertFalse(state.payload['recovery_required'])
            self.assertFalse(fixture.robot._all_stopped)
        finally:
            fixture.close()
