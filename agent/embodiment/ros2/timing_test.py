"""Elapsed deadlines, delayed transport work and stop-before-cleanup ordering."""
import asyncio
import dataclasses
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock
import httpx
from embodiment.ros2.bounded_io import thread_call
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.timing import Timing
from embodiment.ros2.robot_client import Ros2RobotClient
from embodiment.ros2.robotics_er import RoboticsER
from embodiment.ros2.live_session import Ros2SessionManager
from embodiment.ros2.camera_poller import Ros2CameraPoller
from embodiment.ros2.ros2_test import CONFIGS, MockRobot, jpeg
from run_ros2 import run_application

class TimingTest(unittest.IsolatedAsyncioTestCase):
  def config(self, **values):
    return dataclasses.replace(RobotConfig.load(CONFIGS/'minimal.json'), timing=dataclasses.replace(Timing(), **values))

  async def test_http_and_er_elapsed_deadlines_with_mock_transport(self):
    calls=[]
    async def slow(request):
      calls.append(request)
      await asyncio.sleep(.2)
      return httpx.Response(200,json={'success':True})
    robot=Ros2RobotClient(self.config(request_timeout=.02,ros_response_margin=.01,http_response_margin=.01),transport=httpx.MockTransport(slow))
    er=RoboticsER('mock',transport=httpx.MockTransport(slow),timeout=.02)
    self.addAsyncCleanup(robot.close)
    self.addAsyncCleanup(er.close)
    with self.assertRaises(httpx.ReadTimeout): await robot.get_robot_state()
    with self.assertRaises(httpx.ReadTimeout):
      await er.reason('detect',dict(camera_id='c',capture_id='id',image_base64=''),'Locate',['arm'])
    self.assertEqual(2,len(calls))

  async def test_stop_precedes_delayed_cleanup_and_idle_model_has_own_limit(self):
    for idle in (False,True):
      order=[]
      backend=MockRobot()
      async def http(request):
        if request.url.path=='/v1/stop': order.append('stop')
        return backend(request)
      class Session:
        def __init__(self,**kwargs):
          if idle: self._last_activity=asyncio.get_running_loop().time()
          self.observation=SimpleNamespace(stop_input_tasks=mock.AsyncMock(),close=mock.Mock())
          self.bus=SimpleNamespace(shutdown=mock.AsyncMock())
          self.stream=None
        async def start_session(self,**kwargs):
          try:
            await asyncio.Event().wait()
            yield {}
          finally:
            order.append('cleanup')
            await asyncio.sleep(.2)
      with self.assertRaises(TimeoutError):
        await run_application(self.config(cleanup_timeout=.03,model_idle_timeout=.03),'Observe',model='mock',api_key='mock',
            transport=httpx.MockTransport(http),session_factory=Session,timeout=2. if idle else .1)
      self.assertEqual(['stop','cleanup'],order)

  async def test_late_connection_is_closed_and_slow_send_interrupts(self):
    closed=threading.Event()
    def connect():
      time.sleep(.15)
      return SimpleNamespace(Shutdown=closed.set)
    with self.assertRaises(TimeoutError):
      await thread_call(connect,.02,abandoned=lambda stream:stream.Shutdown())
    for _ in range(50):
      if closed.is_set(): break
      await asyncio.sleep(.01)
    self.assertTrue(closed.is_set())
    session=object.__new__(Ros2SessionManager)
    session.loop=asyncio.get_running_loop()
    session.embodiment=SimpleNamespace(config=self.config(live_io_timeout=.02),interrupt=mock.Mock())
    session.stream=SimpleNamespace(Send=lambda message:time.sleep(.1))
    with self.assertRaises(TimeoutError): await session._send_stream({'toolResponse':{}})
    session.embodiment.interrupt.assert_called_once_with(session_lost=True)

  async def test_serial_captures_fit_observation_budget_and_stay_fresh(self):
    entered=asyncio.Event()
    calls=[]
    async def capture(camera):
      calls.append(camera)
      entered.set()
      await asyncio.sleep(1.1)
      return jpeg()
    poller=Ros2CameraPoller(SimpleNamespace(config=SimpleNamespace(cameras=[SimpleNamespace(id='c')]),get_camera_snapshot=capture),asyncio.Queue())
    background=asyncio.create_task(poller.wait_for_next_frame())
    await entered.wait()
    session=object.__new__(Ros2SessionManager)
    session.embodiment=SimpleNamespace(poller=poller,observation_timeout=self.config().timing.observation_timeout,
        _stop_generation=0,observation_revision=0,manipulation=SimpleNamespace(inspections={}))
    session.observation=SimpleNamespace(dump_frame=mock.AsyncMock())
    session.send_message=mock.AsyncMock()
    session._frame_results={}
    self.assertTrue(await session.send_latest_video_frame())
    await background
    self.assertEqual(2,len(calls))

  async def test_tcp_total_deadline_even_while_chunks_keep_arriving(self):
    workers=set()
    async def serve(reader,writer):
      workers.add(asyncio.current_task())
      try:
        await reader.readuntil(b'\r\n\r\n')
        writer.write(b'HTTP/1.1 200 OK\r\nContent-Length: 1000\r\n\r\n')
        await writer.drain()
        for _ in range(20):
          await asyncio.sleep(.01)
          writer.write(b' ')
          await writer.drain()
      except (ConnectionError,asyncio.IncompleteReadError): pass
      finally:
        writer.close()
        try: await writer.wait_closed()
        except ConnectionError: pass
    server=await asyncio.start_server(serve,'127.0.0.1',0)
    cfg=dataclasses.replace(self.config(request_timeout=.02,ros_response_margin=.01,http_response_margin=.01),
        robot_url=f'http://127.0.0.1:{server.sockets[0].getsockname()[1]}')
    robot=Ros2RobotClient(cfg)
    try:
      with self.assertRaisesRegex(httpx.ReadTimeout,'elapsed'): await robot.get_robot_state()
    finally:
      await robot.close()
      server.close()
      await server.wait_closed()
      await asyncio.gather(*workers,return_exceptions=True)

  async def test_active_tool_can_outlast_model_idle_window(self):
    class Session:
      def __init__(self,**kwargs):
        self.embodiment=kwargs['embodiment_instance']
        self._last_activity=asyncio.get_running_loop().time()
        self.observation=SimpleNamespace(stop_input_tasks=mock.AsyncMock(),close=mock.Mock())
        self.bus=SimpleNamespace(shutdown=mock.AsyncMock())
        self.stream=None
      async def start_session(self,**kwargs):
        async with self.embodiment._action_lock:
          await asyncio.sleep(.25)
        self.embodiment.task_result={'success':True,'summary':'Slow tool completed'}
        yield {'type':'tool_call','name':'finish_task'}
    result=await run_application(self.config(model_idle_timeout=.02),'Observe',model='mock',api_key='mock',
        transport=httpx.MockTransport(MockRobot()),session_factory=Session,timeout=2.)
    self.assertTrue(result['success'])
