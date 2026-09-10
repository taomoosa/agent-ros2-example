"""Primitive adapter template exercised through real ROS and the HTTP bridge."""
import asyncio
import copy
import dataclasses
import json
import math
import unittest
from unittest import mock
import httpx
from rclpy.task import Future
from helpers import FakeDriver, RosFixture, config, eventually, arm_state, MockGeminiStream
from ros2_agent_server.api import create_app
from ros2_agent_server.motion import MotionCompiler, Move, Hardware
from ros2_agent_server.pixels import PixelPlans
from ros2_agent_server.protocol import BridgeError, Reply, validate_request
from ros2_agent_server.primitive_driver import PrimitiveAdapter, HardwareBackend
from ros2_agent_interfaces.srv import RobotRequest
from embodiment.ros2.config import RobotConfig as AgentConfig
from embodiment.ros2.tools import ros2_tools
from run_ros2 import run_application
import pixels_test
from plane_projection_test import plane_config


def primitive_config(name='minimal.json'):
    c=config(name)
    c.server.hardware={
        'profiles':{a.id:dict(orientation=[0.,0.,0.,1.],
            approach_m=.1,lift_m=.12,transfer_height_m=5.) for a in c.arms},
        'position_names':{a.id:{'home':'Initial position.', 'ready':'Ready position.'} for a in c.arms}}
    return c


class PrimitiveTest(unittest.IsolatedAsyncioTestCase):
    async def fixture(self,name='minimal.json'):
        f=RosFixture(primitive_config(name),driver_factory=FakeDriver)
        self.addCleanup(f.close)
        for arm in f.config.arms:
            f.driver.backend.named_positions[arm.id]['home']=dict(kind='joints',names=['joint1','joint2'],positions=[.2,-.4])
        await f.ready()
        http=httpx.AsyncClient(transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),base_url='http://test')
        self.addAsyncCleanup(http.aclose)
        return f,http

    async def plan(self,http,ids):
        cap=(await http.get('/v1/cameras/overhead/capture')).json()
        result=await http.post('/v1/plans',json=dict(capture_id=cap['capture_id'],targets=[dict(arm_id=a,grasp=[24,16],release=[25,16]) for a in ids]))
        self.assertEqual(200,result.status_code,result.text)
        return result.json()['plan_id']

    async def test_three_targets_and_singleton_group_use_one_execution(self):
        f,http=await self.fixture()
        pose=dict(kind='pose',reference='tcp',frame_id='world',position=[.3,0.,.4],orientation=[0.,0.,0.,1.])
        result=await http.post('/v1/move',json=dict(arm_ids=['arm'],targets=[pose]))
        self.assertTrue(result.json()['success'],result.text)
        self.assertAlmostEqual(.4,f.driver.backend.motions[-1][0]['position'][2])
        self.assertEqual('tcp',f.driver.backend.motions[-1][0]['reference'])
        cap=(await http.get('/v1/cameras/overhead/capture')).json()
        result=await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='pixel',capture_id=cap['capture_id'],pixel=[24,16],profile='tabletop')]))
        self.assertTrue(result.json()['success'],result.text)
        result=await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='named',name='home')]))
        self.assertTrue(result.json()['success'],result.text)
        self.assertEqual({'kind':'named','name':'home'},f.driver.backend.motions[-1][0])
        self.assertEqual([.2,-.4],f.driver.backend.resolved_steps[0]['targets'][0]['positions'])
        before=len(f.driver.backend.motions)
        result=await http.post('/v1/move', json=dict(arm_ids=['arm'], targets=[dict(kind='pose', **{k:pose[k] for k in ('frame_id','position','orientation')})]))
        self.assertTrue(result.json()['success'],result.text)
        self.assertEqual(before+1,len(f.driver.backend.motions))
        self.assertTrue((await http.post('/v1/move', json=dict(all_arms=True,targets=[dict(kind='named',name='home')]))).json()['success'])
        self.assertTrue((await http.post('/v1/gripper',json=dict(all_arms=True,opening=1.))).json()['success'])

    async def test_invalid_selection_targets_and_unknown_names_never_move(self):
        f,http=await self.fixture()
        for payload in [dict(arm_ids=[],targets=[dict(kind='named',name='home')]),
            dict(arm_ids=['arm'],all_arms=True,targets=[dict(kind='named',name='home')]),
            dict(arm_ids=['arm','arm'],targets=[dict(kind='named',name='home')]),
            dict(arm_ids=['missing'],targets=[dict(kind='named',name='home')]),
            dict(all_arms=True,targets=[dict(kind='named',name='missing')]),
            dict(all_arms=True,targets=[dict(kind='pixel',capture_id='missing',pixel=[1,1],profile='tabletop')]),
            dict(all_arms=True,targets=[dict(kind='named',name='home',position=[0.,0.,0.])])]:
            response=await http.post('/v1/move',json=payload)
            self.assertGreaterEqual(response.status_code,400,response.text)
        self.assertFalse(f.driver.backend.motions)

    async def test_pick_verify_place_and_group_partial_stop(self):
        for topology in ('minimal.json','dual_arm.json'):
            f,http=await self.fixture(topology)
            ids=[a.id for a in f.config.arms]
            plan=await self.plan(http,ids)
            result=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
            self.assertTrue(result.json()['success'],result.text)
            self.assertEqual(['open','approach','descend','close','lift'],[s['phase'] for s in f.driver.backend.prepared])
            self.assertEqual(409,(await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='named',name='home')]))).status_code)
            # A rejected manual move intentionally invalidates held plans; recover.
            stop=await http.post('/v1/stop',json=dict(arm_ids=[ids[0]]))
            self.assertEqual(ids,stop.json()['stopped_arm_ids'])
            self.assertTrue((await http.post('/v1/arms/recover',json={})).json()['success'])
            plan=await self.plan(http,ids)
            self.assertTrue((await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))).json()['success'])
            observations=[]
            for arm in ids:
                cap=(await http.get('/v1/cameras/overhead/observation')).json()
                observations.append(dict(arm_id=arm,capture_id=cap['capture_id'],success=True))
            self.assertTrue((await http.post('/v1/plans/verify',json=dict(plan_id=plan,observations=observations))).json()['success'])
            result=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='place'))
            self.assertTrue(result.json()['success'],result.text)
            self.assertEqual(['transfer','descend','open','retreat'],[s['phase'] for s in f.driver.backend.prepared])

    async def test_partial_stop_interrupts_active_group_and_late_success(self):
        f,http=await self.fixture('dual_arm.json')
        plan=await self.plan(http,['left','right'])
        f.driver.hold_moves=True
        task=asyncio.create_task(http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick')))
        await eventually(lambda:bool(f.driver.held))
        stop=await http.post('/v1/stop',json=dict(arm_ids=['left']))
        self.assertEqual(['left','right'],stop.json()['stopped_arm_ids'])
        result=await task
        self.assertFalse(result.json()['success'])
        self.assertEqual(1,len(f.driver.backend.motions))
        self.assertFalse(f.driver.adapter.sequence)

    async def test_capability_preflight_failure_and_phase_failure_stop_group(self):
        f,http=await self.fixture('dual_arm.json')
        f.driver.backend.coupled_transfer=False
        plan=await self.plan(http,['left','right'])
        result=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
        self.assertFalse(result.json()['success'])
        self.assertFalse(f.driver.backend.motions)
        self.assertEqual(['left','right'],f.driver.backend.stop_ids[-1])
        f.driver.backend.coupled_transfer=True
        await http.post('/v1/stop',json={})
        await http.post('/v1/arms/recover',json={})
        f.driver.backend.fail_phase='descend'
        plan=await self.plan(http,['left','right'])
        result=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
        self.assertFalse(result.json()['success'])
        self.assertEqual('descend',result.json()['failed_phase'])
        self.assertEqual(['left','right'],f.driver.backend.stop_ids[-1])
        self.assertEqual(['prepare','gripper','move','move','stop'],[c[0] for c in f.driver.calls[-5:]])

    async def test_sequence_deadline_replay_and_unsupported_recovery(self):
        f,http=await self.fixture()
        f.driver.hold_moves=True
        result=await f.gateway.request('move','',dict(all_arms=True,targets=[dict(kind='named',name='home')]),.25)
        self.assertEqual(504,result.status)
        self.assertEqual('unknown',result.payload['outcome'])
        self.assertFalse(f.driver.held)
        f.driver.hold_moves=False
        await http.post('/v1/stop',json={})
        f.driver.backend.recovery=False
        result=await http.post('/v1/arms/recover',json={})
        self.assertFalse(result.json()['success'])
        # Direct primitive replays cannot bypass preparation or expected step index.
        last=next(c for c in f.driver.calls if c[0]=='move')
        request=RobotRequest.Request(operation='move',payload_json=json.dumps(last[2]),timeout_sec=1.)
        result=Reply.from_ros(await f.driver.adapter.handle(request,RobotRequest.Response()))
        self.assertEqual(409,result.status)

    def test_pixel_resolver_uses_plane_calibration_and_rejects_expired_evidence(self):
        c=plane_config()
        c.server.hardware=primitive_config('dual_arm.json').server.hardware
        c.server.hardware['profiles']={a.id:next(iter(c.server.hardware['profiles'].values())) for a in c.arms}
        c.server.hardware['position_names']={}
        from ros2_agent_server.state import CameraFrame
        from helpers import jpeg
        now=[0.]
        pixels=PixelPlans(c,clock=lambda:now[0])
        cap=pixels.capture_plane('overhead',CameraFrame(jpeg(),'overhead_optical',100,1))
        compiler=MotionCompiler(c,pixels)
        arm=c.arms[0].id
        point=pixels.project(cap['capture_id'],[24,16])
        target=dict(kind='pixel',capture_id=cap['capture_id'],pixel=[24,16],profile='tabletop',offset_m=0.)
        resolved=compiler.resolve(arm,target)
        self.assertAlmostEqual(point['position'][2],resolved['position'][2])
        self.assertEqual('tcp',resolved['reference'])
        now[0]=c.server.capture_ttl+1
        with self.assertRaises(BridgeError):compiler.resolve(arm,target)

    async def test_agent_gemini_mock_uses_common_move_and_pick_place_sequence(self):
        for topology,selected,fail_once in [('minimal.json',None,False),('dual_arm.json',None,False),
                ('dual_arm.json',['left'],False),('dual_arm.json',['right'],False),('minimal.json',None,True)]:
            f,http=await self.fixture(topology)
            ids=selected or [a.id for a in f.config.arms]
            if selected:
                f.driver.backend.coupled_transfer=False
            if fail_once:f.driver.backend.fail_phase='descend'
            from helpers import ROOT
            cfg=AgentConfig.load(ROOT/'agent/configs'/topology)
            class Flow(MockGeminiStream):
                def __init__(self):
                    self.plan_id=None
                    self.observations={}
                    super().__init__(self.script())
                def Send(self,message):
                    if 'toolResponse' in message:
                        response=message['toolResponse']['functionResponses'][0]['response']
                        self.last_response=response
                        if 'plan_id' in response:self.plan_id=response['plan_id']
                        if 'observation_id' in response:self.observations[response['arm_id']]=response['observation_id']
                    super().Send(message)
                def script(self):
                    yield dict(id='home',name='move',args=dict(all_arms=True,targets=[dict(kind='named',name='home')]))
                    for attempt in range(2 if fail_once else 1):
                        yield dict(id=f'detect-{attempt}',name='detect_targets',args=dict(camera_id='overhead',instruction='Move the block',arm_ids=ids))
                        if any(c.mount=='flange' for c in cfg.cameras):
                            yield dict(id=f'approach-{attempt}',name='approach_targets',args=dict(plan_id=self.plan_id))
                        yield dict(id=f'pick-{attempt}',name='pick_targets',args=dict(plan_id=self.plan_id))
                        if self.last_response.get('success') is True:break
                        yield dict(id=f'recover-{attempt}',name='recover_arms',args={})
                    for arm in ids:
                        yield dict(id='inspect-'+arm,name='inspect_grasp',args=dict(plan_id=self.plan_id,arm_id=arm))
                    yield dict(id='verify',name='verify_grasp',args=dict(plan_id=self.plan_id,observations=[dict(arm_id=a,observation_id=self.observations[a],success=True,reason='Object is held in the image') for a in ids]))
                    yield dict(id='place',name='place_targets',args=dict(plan_id=self.plan_id))
                    yield dict(id='observe',name='get_robot_state',args={})
                    yield dict(id='done',name='finish_task',args=dict(success=True,summary='Placement observed'))
            def er(request):
                return httpx.Response(200,json={'candidates':[{'finishReason':'STOP','content':{'parts':[{'text':json.dumps({'targets':[dict(arm_id=a,grasp=[500,500],release=[500,600]) for a in ids]})}]}}]})
            stream=Flow()
            with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
                client.return_value.create_stream.return_value=stream
                result=await run_application(cfg,'Move the block',model='mock',api_key='mock',timeout=30,
                    transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),robotics_transport=httpx.MockTransport(er))
            self.assertTrue(result['success'])
            responses=[m['toolResponse']['functionResponses'][0]['response'] for m in stream.messages if 'toolResponse' in m]
            self.assertEqual(1 if fail_once else 0,sum(r.get('success') is not True for r in responses),responses)
            self.assertNotIn('execute_plan',[c[0] for c in f.driver.calls])
            phases=[payload for op,_,payload in f.driver.calls if op in {'move','gripper'} and 'phase' in payload]
            self.assertTrue(phases)
            self.assertTrue(all(payload['arm_ids']==ids for payload in phases),phases)
            prepared=[payload for op,_,payload in f.driver.calls if op=='prepare' and 'phase' in payload['steps'][0]]
            self.assertTrue(prepared)
            self.assertTrue(all(payload['coupled']==(len(ids)>1) for payload in prepared))
            declarations={d['name']:d for d in ros2_tools(cfg)[0]['functionDeclarations']}
            self.assertTrue({'move','gripper','stop'} <= declarations.keys())
            self.assertFalse({'move_arm','move_arms','set_gripper'} & declarations.keys())
            self.assertNotIn('behavior',declarations['stop'])

    async def test_wrist_pixel_targets_bind_the_arm_and_old_capture_is_not_reused(self):
        f,http=await self.fixture('dual_arm.json')
        cap=(await http.get('/v1/cameras/left_wrist/capture')).json()
        target=dict(kind='pixel',capture_id=cap['capture_id'],pixel=[24,16],profile='tabletop')
        wrong=await http.post('/v1/move',json=dict(arm_ids=['right'],targets=[target]))
        self.assertEqual(422,wrong.status_code,wrong.text)
        self.assertFalse(f.driver.backend.motions)
        valid=await http.post('/v1/move',json=dict(arm_ids=['left'],targets=[target]))
        self.assertTrue(valid.json()['success'],valid.text)
        old=await http.post('/v1/move',json=dict(arm_ids=['left'],targets=[target]))
        self.assertGreaterEqual(old.status_code,400)
        self.assertEqual(1,len(f.driver.backend.motions))

    async def test_stop_empty_duplicates_and_conflicting_scopes_are_rejected(self):
        f,http=await self.fixture('dual_arm.json')
        for payload in ({'arm_ids':[]},{'arm_ids':['left','left']},{'arm_ids':['missing']},
                        {'arm_ids':['left'],'all_arms':True},{'arm_id':'left','all_arms':True}):
            response=await http.post('/v1/stop',json=payload)
            self.assertEqual(422,response.status_code,response.text)
        self.assertFalse(f.driver.backend.stop_ids)
        response=await http.post('/v1/stop',json=dict(arm_ids=['left']))
        self.assertEqual(['left'],response.json()['stopped_arm_ids'])
        self.assertFalse(f.robot._all_stopped)
        response=await http.post('/v1/stop',json=dict(all_arms=True))
        self.assertEqual(['left','right'],response.json()['stopped_arm_ids'])
        self.assertTrue(f.robot._all_stopped)

    async def test_missing_profile_joint_capability_and_malformed_completion_fail(self):
        f,http=await self.fixture()
        f.driver.backend.joint_targets=False
        response=await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='named',name='home')]))
        self.assertEqual(501,response.status_code,response.text)
        self.assertFalse(f.driver.backend.motions)
        await http.post('/v1/stop',json={})
        await http.post('/v1/arms/recover',json={})
        f.robot.motion.hardware.profiles.clear()
        plan=await self.plan(http,['arm'])
        response=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
        self.assertEqual(422,response.status_code,response.text)
        self.assertFalse(f.driver.backend.motions)
        f.driver.backend.move=mock.AsyncMock(return_value={'success':'yes'})
        response=await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='named',name='ready')]))
        self.assertEqual(502,response.status_code,response.text)
        self.assertEqual('unknown',response.json()['outcome'])
        self.assertTrue(f.driver.backend.stop_ids)

    async def test_driver_stop_during_preflight_cannot_reserve_or_dispatch_late_work(self):
        c=primitive_config('dual_arm.json')
        entered,release=asyncio.Event(),asyncio.Event()
        class Backend(HardwareBackend):
            coordinated_motion=True
            async def prepare(self,steps,arm_ids,coupled,context):
                entered.set()
                await release.wait()
                return dict(success=True)
            async def stop(self,arm_ids,context):
                return dict(success=True)
        adapter=PrimitiveAdapter(c,Backend(),lambda:100)
        _,steps=MotionCompiler(c,PixelPlans(c)).move(dict(all_arms=True,targets=[dict(kind='named',name='ready')]))
        request=RobotRequest.Request(operation='prepare',timeout_sec=2.,payload_json=json.dumps(dict(
            sequence_id='sequence',steps=steps,arm_ids=['left','right'],coupled=False)))
        task=asyncio.create_task(adapter.handle(request,RobotRequest.Response()))
        await entered.wait()
        stop=RobotRequest.Request(operation='stop',timeout_sec=1.,payload_json=json.dumps(dict(arm_ids=['left'])))
        stopped=Reply.from_ros(await adapter.handle(stop,RobotRequest.Response()))
        self.assertEqual(['left','right'],stopped.payload['completed_arm_ids'])
        release.set()
        result=Reply.from_ros(await task)
        self.assertFalse(result.payload['success'])
        self.assertIsNone(adapter.sequence)

    async def test_dwell_is_required_at_intermediate_phases_and_stop_unblocks_wait(self):
        f,http=await self.fixture()
        f.config.server.settling_dwell=.15
        f.config.server.state_completion_timeout=.5
        plan=await self.plan(http,['arm'])
        task=asyncio.create_task(http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick')))
        await eventually(lambda:any(c[0]=='gripper' for c in f.driver.calls))
        self.assertFalse(f.driver.backend.motions)
        await asyncio.sleep(.05)
        self.assertFalse(f.driver.backend.motions)
        stopped=await http.post('/v1/stop',json=dict(all_arms=True))
        self.assertTrue(stopped.json()['success'])
        self.assertFalse((await task).json()['success'])
        self.assertFalse(f.driver.backend.motions)

    def test_tcp_pose_is_preserved_without_server_tool_calibration(self):
        c=primitive_config()
        c.server.hardware['profiles']={}
        compiler=MotionCompiler(c,PixelPlans(c))
        pose=dict(kind='pose',reference='tcp',frame_id='world',position=[.3,.1,.4],
            orientation=[0.,0.,math.sqrt(.5),math.sqrt(.5)])
        _,steps=compiler.move(dict(all_arms=True,targets=[pose]))
        self.assertEqual(pose,steps[0]['targets'][0])
        for field,value in [('tcp_offset',[.1,0.,0.]),('tcp_orientation',[0.,0.,0.,1.])]:
            profile=dict(primitive_config().server.hardware['profiles']['arm'],**{field:value})
            with self.assertRaises(ValueError):
                Hardware(profiles={'arm':profile})

    async def test_named_home_rejects_measured_payload_even_without_a_plan(self):
        for path,body in [('/v1/move',dict(all_arms=True,targets=[dict(kind='named',name='home')])),('/v1/move',dict(arm_ids=['arm'],targets=[dict(kind='named',name='home')]))]:
            f,http=await self.fixture()
            f.driver.states['arm']['gripper']['object_detected']=True
            await eventually(lambda:f.robot.store.state(f.robot.get_clock().now().nanoseconds)['arms'][0]['gripper']['object_detected'] is True)
            response=await http.post(path,json=body)
            self.assertEqual(409,response.status_code,response.text)
            self.assertFalse(f.driver.calls)
            self.assertTrue(f.robot._recovery_required)

    async def test_negative_grasp_sensor_aborts_before_lift(self):
        f,http=await self.fixture()
        original=f.driver.backend.gripper
        async def empty_gripper(openings,ids,context):
            result=await original(openings,ids,context)
            for arm in ids:f.driver.states[arm]['gripper']['object_detected']=False
            f.driver.publish()
            return result
        f.driver.backend.gripper=empty_gripper
        plan=await self.plan(http,['arm'])
        response=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
        self.assertFalse(response.json()['success'])
        self.assertEqual('close',response.json()['failed_phase'])
        self.assertEqual(2,len(f.driver.backend.motions))
        self.assertEqual(['arm'],f.driver.backend.stop_ids[-1])

    async def test_missing_hardware_config_rejects_home_and_plan_before_dispatch(self):
        f,http=await self.fixture()
        f.robot.motion.hardware=Hardware()
        result=await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='named',name='home')]))
        self.assertEqual(422,result.status_code)
        self.assertIn('Unknown named position',result.json()['error'])
        plan=await self.plan(http,['arm'])
        result=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
        self.assertEqual(422,result.status_code)
        self.assertIn('hardware.profiles.arm',result.json()['error'])
        self.assertFalse(f.driver.calls)
        result=await http.post('/v1/move',json=dict(arm_ids=['arm'],targets=[dict(kind='pose',frame_id='world',
            position=[.2,0.,.3],orientation=[0.,0.,0.,1.])]))
        self.assertTrue(result.json()['success'],result.text)
        self.assertEqual(['prepare','move'],[c[0] for c in f.driver.calls])

    def test_removed_config_and_driver_operations_are_rejected(self):
        from ros2_agent_server.models import Settings
        for mode in ('legacy','primitives'):
            with self.assertRaises(ValueError):
                Settings(driver_mode=mode)
        for operation in ('move_arm','move_arms','set_gripper','reset_arms'):
            with self.assertRaises(BridgeError) as error:
                validate_request(config(),operation,'',{})
            self.assertEqual(404,error.exception.status)

    async def test_group_named_goals_are_resolved_only_in_backend_before_any_motion(self):
        f,http=await self.fixture('dual_arm.json')
        del f.driver.backend.named_positions['right']['ready']
        result=await http.post('/v1/move',json=dict(all_arms=True,targets=[dict(kind='named',name='ready')]))
        self.assertEqual(422,result.status_code,result.text)
        self.assertIn("'right'",result.json()['error'])
        self.assertFalse(f.driver.backend.motions)
        self.assertEqual(['prepare','stop'],[c[0] for c in f.driver.calls])
        targets=f.driver.calls[0][2]['steps'][0]['targets']
        self.assertEqual([{'kind':'named','name':'ready'}]*2,targets)

    async def test_backend_named_goal_snapshot_is_not_reloaded_during_execution(self):
        f,http=await self.fixture()
        f.driver.hold_moves=True
        task=asyncio.create_task(http.post('/v1/move',json=dict(arm_ids=['arm'],targets=[dict(kind='named',name='ready')])))
        await eventually(lambda:bool(f.driver.held))
        f.driver.backend.named_positions['arm']['ready']['position'][2]=9.
        f.driver.release(True)
        result=await task
        self.assertTrue(result.json()['success'],result.text)
        self.assertEqual({'kind':'named','name':'ready'},f.driver.backend.motions[-1][0])
        self.assertAlmostEqual(.4,f.driver.backend.resolved_steps[0]['targets'][0]['position'][2])
        self.assertAlmostEqual(.35,f.driver.states['arm']['flange_pose']['position'][2])

    async def test_pixel_pick_phases_deliver_tcp_positions_to_backend(self):
        f,http=await self.fixture()
        plan=await self.plan(http,['arm'])
        result=await http.post('/v1/plans/execute',json=dict(plan_id=plan,stage='pick'))
        self.assertTrue(result.json()['success'],result.text)
        steps=f.driver.calls[0][2]['steps']
        poses=[s['targets'][0] for s in steps if s['operation']=='move']
        self.assertEqual(['tcp']*3,[p['reference'] for p in poses])
        for expected,pose in zip([1.1,1.,1.12],poses):
            self.assertAlmostEqual(expected,pose['position'][2])
        self.assertAlmostEqual(1.07,f.driver.states['arm']['flange_pose']['position'][2])

    async def test_hardware_template_owns_name_pairs_and_does_not_fake_preflight_or_recovery(self):
        import importlib.util
        import time
        from helpers import ROOT
        from ros2_agent_server.primitive_driver import CommandContext
        spec=importlib.util.spec_from_file_location('robot_backend_example',ROOT/'server/examples/primitive_driver.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        hardware=json.loads((ROOT/'server/examples/robot_hardware.json').read_text())
        backend=module.RobotBackend(hardware)
        target={'kind':'named','name':'home'}
        hardware['named_positions']['arm']['home']['xyz'][2]=9.
        self.assertAlmostEqual(1.2,backend.resolve_targets([target],['arm'])[0]['xyz'][2])
        self.assertEqual({'xyz','quaternion'},set(backend.resolve_targets([target],['arm'])[0]))
        backend.named_positions['arm']['preset']='controller-native-preset-7'
        self.assertEqual(['controller-native-preset-7'],backend.resolve_targets([dict(kind='named',name='preset')],['arm']))
        context=CommandContext(time.monotonic()+1.)
        step=dict(operation='move',arm_ids=['arm'],duration=3.,targets=[target])
        with self.assertRaises(BridgeError) as error:
            await backend.prepare([step],['arm'],False,context)
        self.assertEqual(501,error.exception.status)
        with self.assertRaises(BridgeError) as error:
            await backend.recover(['arm'],context)
        self.assertEqual(501,error.exception.status)


    def test_named_metadata_accepts_descriptions_but_never_coordinate_schemas(self):
        catalog={'arm':{'home':'Initial position.','ready':'Observation position.'}}
        self.assertEqual(catalog,Hardware(position_names=catalog).position_names)
        for entries in (['home'],{'home':{}},{'home':{'xyz':[0.,0.,0.]}},{'home':''},{'':'Empty name'}):
            with self.assertRaises(ValueError):
                Hardware(position_names={'arm':entries})
