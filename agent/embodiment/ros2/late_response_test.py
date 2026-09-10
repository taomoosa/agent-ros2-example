"""Late Gemini and HTTP responses must not resurrect expired work."""
import asyncio
import dataclasses
import json
import time
import unittest
from unittest import mock

import httpx
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.robot_client import Ros2RobotClient
from embodiment.ros2.robotics_er import RoboticsER
from embodiment.ros2.ros2_test import CONFIGS, MockRobot, MockGeminiStream
from embodiment.ros2.timing import Timing
from run_ros2 import run_application


class LateResponseTest(unittest.IsolatedAsyncioTestCase):
  def config(self, **timing):
    return dataclasses.replace(RobotConfig.load(CONFIGS/'minimal.json'),
        timing=dataclasses.replace(Timing(), **timing))

  async def test_er_rejects_late_success_even_when_cancellation_is_suppressed(self):
    async def late(request):
      try:
        await asyncio.Event().wait()
      except asyncio.CancelledError:
        return httpx.Response(200,json={'candidates':[{'finishReason':'STOP',
            'content':{'parts':[{'text':'{"targets":[]}'}]}}]})
    er=RoboticsER('mock',transport=httpx.MockTransport(late),timeout=.03)
    self.addAsyncCleanup(er.close)
    with self.assertRaisesRegex(httpx.ReadTimeout,'elapsed'):
      await er.reason('detect',dict(camera_id='c',capture_id='id',image_base64=''),'Locate',['arm'])

  async def test_http_late_success_is_rejected_after_cancellation_or_slow_decode(self):
    for mode in ('suppressed_cancel','slow_decode'):
      class SlowResponse(httpx.Response):
        def json(self):
          time.sleep(.08)
          return super().json()
      async def late(request):
        if mode=='suppressed_cancel':
          try: await asyncio.Event().wait()
          except asyncio.CancelledError: pass
          return httpx.Response(200,json={'success':True})
        return SlowResponse(200,json={'success':True})
      robot=Ros2RobotClient(self.config(request_timeout=.02,ros_response_margin=.01,http_response_margin=.01),
          transport=httpx.MockTransport(late))
      self.addAsyncCleanup(robot.close)
      with self.subTest(mode=mode),self.assertRaisesRegex(httpx.ReadTimeout,'elapsed'):
        await robot.get_robot_state()

  async def test_delayed_live_model_turn_completes_within_idle_budget(self):
    loop=asyncio.get_running_loop()
    backend=MockRobot()
    class DelayedStream(MockGeminiStream):
      def Start(self,on_message,on_done):
        def delayed(message):
          loop.call_soon_threadsafe(lambda:loop.call_later(.12,on_message,message))
        super().Start(delayed,on_done)
    stream=DelayedStream(backend.history,[
        dict(id='state',name='get_robot_state',args={}),
        dict(id='finish',name='finish_task',args=dict(success=True,summary='Observed'))])
    with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
      client.return_value.create_stream.return_value=stream
      result=await run_application(self.config(model_idle_timeout=1.),'Observe',model='mock',api_key='mock',
          transport=httpx.MockTransport(backend),timeout=5.)
    self.assertTrue(result['success'])
    self.assertTrue(stream.closed)
    self.assertEqual(2,len([m for m in stream.messages if 'toolResponse' in m]))

  async def test_live_reply_arriving_after_idle_timeout_cannot_move(self):
    backend=MockRobot()
    class StalledStream(MockGeminiStream):
      def Send(self,message):
        self.messages.append(message)
        if 'setup' in message:self.on_message({'setupComplete':{}})
        # No reply to the instruction until after application shutdown.
    stream=StalledStream(backend.history,[])
    with mock.patch('model.live_api_client.GeminiLiveApiClient') as client:
      client.return_value.create_stream.return_value=stream
      with self.assertRaisesRegex(TimeoutError,'Model response'):
        await run_application(self.config(model_idle_timeout=.15),'Move',model='mock',api_key='mock',
            transport=httpx.MockTransport(backend),timeout=5.)
    self.assertTrue(stream.closed)
    self.assertTrue(any(r.url.path=='/v1/stop' for r in backend.requests))
    stream.on_message({'toolCall':{'functionCalls':[dict(id='late',name='move',
        args=dict(all_arms=True,targets=[dict(kind='named',name='home')]))]}})
    await asyncio.sleep(.05)
    self.assertFalse(any(r.url.path=='/v1/move' for r in backend.requests))

  async def test_er_synchronous_decode_overrun_cannot_create_a_result(self):
    class SlowResponse(httpx.Response):
      def json(self):
        time.sleep(.08)
        return super().json()
    er=RoboticsER('mock',transport=httpx.MockTransport(lambda request:SlowResponse(200,
        json={'candidates':[{'finishReason':'STOP','content':{'parts':[{'text':'{"targets":[]}'}]}}]})),timeout=.03)
    self.addAsyncCleanup(er.close)
    with self.assertRaisesRegex(httpx.ReadTimeout,'elapsed'):
      await er.reason('detect',dict(camera_id='c',capture_id='id',image_base64=''),'Locate',['arm'])

  async def test_late_motion_success_stops_and_never_automatically_replays(self):
    from embodiment.ros2.ros2_embodiment import Ros2Embodiment
    paths=[]
    async def late(request):
      paths.append(request.url.path)
      if request.url.path=='/v1/move':
        try:await asyncio.Event().wait()
        except asyncio.CancelledError:pass
      return httpx.Response(200,json={'success':True})
    e=Ros2Embodiment(self.config(motion_timeout=.01,settling_timeout=.01,state_completion_timeout=.01,
        bridge_processing_margin=.01,ros_response_margin=.01,http_response_margin=.01),transport=httpx.MockTransport(late))
    self.addAsyncCleanup(e.close)
    args=dict(all_arms=True,targets=[dict(kind='named',name='home')],duration=.1)
    result=await e.execute_action('move',**args)
    self.assertFalse(result['success'])
    self.assertEqual('unknown',result['outcome'])
    self.assertTrue(result['stop_result']['success'])
    self.assertTrue(e.manipulation.needs_recovery)
    self.assertFalse((await e.execute_action('move',**args))['success'])
    self.assertEqual(['/v1/move','/v1/stop'],paths)
