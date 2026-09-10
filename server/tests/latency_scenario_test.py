"""Delayed model/controller work with real HTTP/ROS and capture-bound evidence."""
import asyncio
import dataclasses
import json
import unittest
from unittest import mock

import httpx
from rclpy.task import Future
from helpers import ROOT, RosFixture, config, jpeg, eventually
from scenario_test import ScenarioStream
from ros2_agent_server.api import create_app
from embodiment.ros2.config import RobotConfig as AgentConfig
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from embodiment.ros2.timing import Timing
from run_ros2 import run_application


def er_response():
    return httpx.Response(200,json={'candidates':[{'finishReason':'STOP','content':{'parts':[
        {'text':json.dumps({'targets':[dict(arm_id='arm',grasp=[500,500],release=[750,750])]})}]}}]})


class LatencyScenarioTest(unittest.IsolatedAsyncioTestCase):
    async def fixture(self):
        f=RosFixture(config('minimal.json'))
        self.addCleanup(f.close)
        await f.ready()
        f.config.server.settling_dwell=.06
        f.driver.depth_offset_ns=-6_000_000
        f.driver.depth_delay_ticks=2
        return f

    def agent(self,f,er):
        cfg=AgentConfig.load(ROOT/'agent/configs/minimal.json')
        e=Ros2Embodiment(cfg,api_key='mock',
            transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),
            robotics_transport=httpx.MockTransport(er))
        self.addAsyncCleanup(e.close)
        return e

    async def test_slow_er_expired_capture_retries_with_new_evidence_without_motion(self):
        f=await self.fixture()
        clock=[0.]
        f.robot.pixels.clock=lambda:clock[0]
        f.config.server.capture_ttl=1.
        requests=[]
        async def er(request):
            requests.append(json.loads(request.content))
            await asyncio.sleep(.05)
            # Advance only evidence time, avoiding a wall-clock race at expiry.
            clock[0]+=1.2 if len(requests)==1 else .1
            return er_response()
        e=self.agent(f,er)
        result=await e.execute_action('detect_targets',camera_id='overhead',instruction='Move block',arm_ids=['arm'])
        self.assertFalse(result['success'],result)
        self.assertIn('Capture expired',result['error'])
        self.assertFalse(e.manipulation.plans)
        self.assertFalse(f.driver.calls)
        self.assertFalse(e.manipulation.needs_recovery)
        result=await e.execute_action('detect_targets',camera_id='overhead',instruction='Move block',arm_ids=['arm'])
        self.assertTrue(result['success'],result)
        self.assertNotEqual(requests[0],requests[1])
        self.assertEqual(1,len(f.robot.pixels.plans))
        self.assertFalse(f.driver.calls)

    async def test_model_thinking_past_plan_lifetime_never_dispatches_stale_pick(self):
        f=await self.fixture()
        clock=[0.]
        f.robot.pixels.clock=lambda:clock[0]
        f.config.server.plan_ttl=1.
        e=self.agent(f,lambda request:er_response())
        plan=await e.execute_action('detect_targets',camera_id='overhead',instruction='Move block',arm_ids=['arm'])
        self.assertTrue(plan['success'],plan)
        clock[0]=1.2
        result=await e.execute_action('pick_targets',plan_id=plan['plan_id'])
        self.assertFalse(result['success'],result)
        self.assertFalse(f.driver.calls)
        self.assertTrue((await e.execute_action('recover_arms'))['success'])
        fresh=await e.execute_action('detect_targets',camera_id='overhead',instruction='Move block',arm_ids=['arm'])
        self.assertTrue(fresh['success'],fresh)
        self.assertNotEqual(plan['plan_id'],fresh['plan_id'])
        self.assertTrue((await e.execute_action('pick_targets',plan_id=fresh['plan_id']))['success'])

    async def test_stop_while_real_er_request_pending_discards_late_detection(self):
        f=await self.fixture()
        entered,release=asyncio.Event(),asyncio.Event()
        async def er(request):
            entered.set()
            await release.wait()
            return er_response()
        e=self.agent(f,er)
        pending=asyncio.create_task(e.execute_action('detect_targets',camera_id='overhead',instruction='Move block',arm_ids=['arm']))
        try:
            await asyncio.wait_for(entered.wait(),2.)
            self.assertTrue((await e.execute_action('stop'))['success'])
        finally:
            release.set()
        result=await pending
        self.assertFalse(result['success'],result)
        self.assertFalse(f.robot.pixels.plans)
        self.assertEqual(['stop'],[op for op,_,_ in f.driver.calls])

    async def test_slow_live_er_motion_settling_and_skew_complete_full_workflow(self):
        f=await self.fixture()
        loop=asyncio.get_running_loop()
        class DelayedStream(ScenarioStream):
            def Start(self,on_message,on_done):
                def later(message):
                    loop.call_soon_threadsafe(lambda:loop.call_later(.1,on_message,message))
                super().Start(later,on_done)
        stream=DelayedStream('success',['arm'],lambda *args:None)
        original=f.driver.execute
        async def execute(request,response):
            if request.operation in {'move','gripper'}:
                done=Future(executor=f.executor)
                def arrived():
                    if not done.done():done.set_result(True)
                timer=f.driver.create_timer(.11,arrived)
                try:await done
                finally:f.driver.destroy_timer(timer)
            result=await original(request,response)
            phase=json.loads(request.payload_json).get('phase')
            if request.operation=='move' and phase in {'lift','retreat'}:
                f.driver.images['overhead']=jpeg('green' if phase=='lift' else 'blue')
            return result
        f.driver.execute=execute
        async def er(request):
            # Model idle must exclude ER/tool execution, even with long reasoning.
            await asyncio.sleep(.8)
            return er_response()
        cfg=dataclasses.replace(AgentConfig.load(ROOT/'agent/configs/minimal.json'),
            timing=dataclasses.replace(Timing(),model_idle_timeout=.5,er_timeout=3.))
        with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
            client.return_value.create_stream.return_value=stream
            result=await run_application(cfg,'Move block to tray',model='mock',api_key='mock',timeout=25.,
                transport=httpx.ASGITransport(app=create_app(f.gateway,f.config)),robotics_transport=httpx.MockTransport(er))
        if stream.failure:raise stream.failure
        self.assertTrue(result['success'],result)
        self.assertTrue(stream.completed)
        self.assertEqual(1,sum(op=='gripper' and p.get('phase')=='close' for op,_,p in f.driver.calls))
        self.assertFalse(any(op=='recover' for op,_,p in f.driver.calls))
        failures=[(name,r) for name,r in stream.results if not r['success']]
        self.assertEqual(['finish_task'],[name for name,_ in failures])
        self.assertIn('Observe current state',failures[0][1]['error'])

    async def test_dual_arm_completion_tolerates_independent_state_cadence_and_pose_noise(self):
        import copy
        from std_msgs.msg import String
        f=RosFixture(config('dual_arm.json'))
        self.addCleanup(f.close)
        await f.ready()
        f.config.server.settling_dwell=.12
        f.config.server.state_completion_timeout=2.
        f.driver.publish_states=False
        samples={'left':[],'right':[]}
        async def publish(arm,period,lag_ns):
            index=0
            while True:
                with f.driver._lock:
                    state=copy.deepcopy(f.driver.states[arm])
                stamp=f.driver.get_clock().now().nanoseconds-lag_ns
                state['stamp_ns']=stamp
                state['flange_pose']['position'][0]+=(index%3-1)*.0003
                # Equivalent quaternion signs and small numerical norm residuals.
                state['flange_pose']['orientation']=[v*(-1 if index%2 else 1)*1.00001
                    for v in state['flange_pose']['orientation']]
                f.driver.state_publishers[arm].publish(String(data=json.dumps(state)))
                samples[arm].append(stamp)
                await asyncio.sleep(period+(index%3)*.003)
                index+=1
        tasks=[asyncio.create_task(publish('left',.027,7_000_000)),
               asyncio.create_task(publish('right',.047,11_000_000))]
        try:
            await eventually(lambda:all(len(v)>=3 for v in samples.values()))
            result=await f.gateway.request('move','',dict(all_arms=True,targets=[dict(kind='named',name='ready')]),3.)
            self.assertTrue(result.payload['success'],result.payload)
            self.assertEqual(['left','right'],result.payload['completed_arm_ids'])
            self.assertNotEqual(samples['left'][-1],samples['right'][-1])
            self.assertTrue(all(len(v)>=5 for v in samples.values()))
        finally:
            for task in tasks:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
