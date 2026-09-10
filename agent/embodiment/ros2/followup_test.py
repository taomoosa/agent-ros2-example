"""Regression coverage for completion evidence and interrupted tool operations."""

import asyncio
from pathlib import Path
import unittest
from embodiment.ros2.test_requests import named_move
from unittest import mock

import httpx

from embodiment.ros2.config import RobotConfig
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from embodiment.ros2.live_session import Ros2SessionManager
from embodiment.ros2.ros2_test import jpeg

CONFIGS = Path(__file__).resolve().parents[2] / 'configs'
POSE = dict(frame_id='world', position=[0., 0., .3], orientation=[0., 0., 0., 1.])


class FollowupTest(unittest.IsolatedAsyncioTestCase):
  def embodiment(self, filename='minimal.json', handler=None):
    result = Ros2Embodiment(RobotConfig.load(CONFIGS / filename),
        transport=httpx.MockTransport(handler or (lambda r: httpx.Response(200, json={'success': True}))))
    self.addAsyncCleanup(result.close)
    return result

  async def test_held_plan_manual_commands_require_recovery_for_each_topology(self):
    for filename in ('minimal.json', 'dual_arm.json'):
      for name in ('move', 'gripper'):
        with self.subTest(filename=filename, name=name):
          e = self.embodiment(filename)
          arm = e.config.arms[0].id
          e.manipulation.plans['p'] = dict(plan_id='p', state='picked', targets=[{'arm_id': arm}])
          args = dict(arm_ids=[arm], opening=1.) if name == 'gripper' else named_move(arm)
          e.robot._client.request = mock.AsyncMock(side_effect=AssertionError('Must not move held object'))
          self.assertFalse((await e.execute_action(name, **args))['success'])
          self.assertTrue(e.manipulation.needs_recovery)
          self.assertFalse((await e.execute_action('finish_task', success=True, summary='Done'))['success'])
          self.assertFalse((await e.execute_action('detect_targets', camera_id='overhead', instruction='Move', arm_ids=[arm]))['success'])
          e.robot._client.request.assert_not_awaited()

  async def test_malformed_command_replies_and_failed_stop_require_recovery(self):
    for payload in ({}, {'success': 'true'}, [], {'success': False, 'error': 'Stop failed'}):
      for name, args in [('move', named_move('arm')), ('stop', {})]:
        with self.subTest(payload=payload, name=name):
          e = self.embodiment(handler=lambda r: httpx.Response(200, json=payload))
          result = await e.execute_action(name, **args)
          self.assertFalse(result['success'])
          self.assertTrue(e.manipulation.needs_recovery)
          if not isinstance(payload, dict) or type(payload.get('success')) is not bool:
            self.assertEqual('unknown', result['outcome'])
          self.assertFalse((await e.execute_action('finish_task', success=True, summary='Done'))['success'])

  async def test_stop_during_detection_never_creates_a_plan(self):
    for phase in ('capture', 'er', 'create'):
      with self.subTest(phase=phase):
        e = self.embodiment()
        entered, release = asyncio.Event(), asyncio.Event()
        async def wait(value):
          entered.set()
          await release.wait()
          return value
        capture = dict(capture_id='c', width=10, height=10)
        detection = {'targets': [dict(arm_id='arm', grasp=[500,500], release=[600,600])]}
        plan = dict(success=True, plan_id='p', state='detected', targets=[{'arm_id':'arm'}])
        e.robot.capture = mock.AsyncMock(return_value=capture)
        e.reasoning.reason = mock.AsyncMock(return_value=detection)
        e.robot.workflow = mock.AsyncMock(return_value=plan)
        async def capture_wait(*args, **kwargs): return await wait(capture)
        async def er_wait(*args, **kwargs): return await wait(detection)
        async def create_wait(*args, **kwargs): return await wait(plan)
        if phase == 'capture': e.robot.capture.side_effect = capture_wait
        if phase == 'er': e.reasoning.reason.side_effect = er_wait
        if phase == 'create': e.robot.workflow.side_effect = create_wait
        task = asyncio.create_task(e.execute_action('detect_targets', camera_id='overhead', instruction='Move', arm_ids=['arm']))
        await asyncio.wait_for(entered.wait(), 1.)
        await e.execute_action('stop')
        release.set()
        self.assertFalse((await asyncio.wait_for(task, 1.))['success'])
        self.assertEqual({}, e.manipulation.plans)
        if phase != 'create': e.robot.workflow.assert_not_awaited()

  async def test_stop_during_verify_or_recovery_discards_late_success(self):
    for name in ('verify_grasp', 'recover_arms'):
      with self.subTest(name=name):
        e = self.embodiment()
        entered, release = asyncio.Event(), asyncio.Event()
        async def workflow(*args, **kwargs):
          entered.set()
          await release.wait()
          return dict(success=True, plan_id='p', state='verified', targets=[{'arm_id':'arm'}])
        e.robot.workflow = mock.AsyncMock(side_effect=workflow)
        e.manipulation.plans['p'] = dict(state='picked', targets=[{'arm_id':'arm'}])
        e.manipulation.inspections['o'] = dict(plan_id='p', arm_id='arm', delivered=True, capture={'capture_id':'c'})
        args = dict(plan_id='p', observations=[dict(arm_id='arm', observation_id='o', success=True, reason='Held')]) if name == 'verify_grasp' else {}
        task = asyncio.create_task(e.execute_action(name, **args))
        await asyncio.wait_for(entered.wait(), 1.)
        await e.execute_action('stop')
        release.set()
        self.assertFalse((await asyncio.wait_for(task, 1.))['success'])
        self.assertTrue(e.manipulation.needs_recovery)
        self.assertEqual({}, e.manipulation.plans)

  async def test_cancelled_motion_stays_unresolved(self):
    e = self.embodiment()
    entered = asyncio.Event()
    async def workflow(*args, **kwargs):
      entered.set()
      await asyncio.Future()
    e.robot.move = mock.AsyncMock(side_effect=workflow)
    task = asyncio.create_task(e.execute_action('move', **named_move('arm')))
    await asyncio.wait_for(entered.wait(), 1.)
    task.cancel()
    with self.assertRaises(asyncio.CancelledError): await task
    self.assertTrue(e.manipulation.needs_recovery)

  def session(self, e):
    s = Ros2SessionManager('mock', e, api_key='key', heartbeat_enabled=False)
    s.stream = mock.Mock()
    self.addCleanup(s.observation.close)
    self.addAsyncCleanup(s.bus.shutdown)
    return s

  async def test_final_placement_requires_delivered_scene_observation(self):
    e = self.embodiment()
    e.manipulation.final_observation_required = True
    s = self.session(e)
    e.poller.wait_for_next_frame = mock.AsyncMock(return_value=b'')
    self.assertFalse((await e.execute_action('finish_task', success=True, summary='Done'))['success'])
    self.assertFalse(await s.send_latest_video_frame())
    message = {'toolResponse': {'functionResponses': [dict(id='state1', name='get_robot_state', response={'arms':[]})]}}
    await s.send_message(message)
    self.assertFalse(message['toolResponse']['functionResponses'][0]['response']['success'])
    self.assertTrue(e.manipulation.final_observation_required)
    e.poller.wait_for_next_frame.return_value = jpeg()
    self.assertTrue(await s.send_latest_video_frame())
    message = {'toolResponse': {'functionResponses': [dict(id='state2', name='get_robot_state', response={'arms':[]})]}}
    await s.send_message(message)
    self.assertTrue(message['toolResponse']['functionResponses'][0]['response']['post_action_observation'])
    self.assertFalse(e.manipulation.final_observation_required)
    self.assertTrue((await e.execute_action('finish_task', success=True, summary='Observed at destination'))['success'])

  async def test_repeated_call_id_terminates_session_without_replay(self):
    e = self.embodiment()
    s = self.session(e)
    message = {'toolCall': {'functionCalls': [dict(id='same', name='move', args={})]}}
    with mock.patch('session_manager.SessionManager._on_message') as accept:
      s._on_message(message)
      s._on_message(message)
      await asyncio.sleep(0)
      await asyncio.sleep(0)
      self.assertEqual(1, accept.call_count)
    self.assertTrue(e.session_lost)
    self.assertFalse(await s._reconnect())
    self.assertFalse((await e.execute_action('reset_arms'))['success'])

  async def test_final_state_rechecks_late_faults_motion_and_held_sensors(self):
    for arm in ({'moving':True}, {'fault':{'code':'fault'}},
                {'gripper':{'fault':{'code':'jam'}}}, {'gripper':{'object_detected':True}}):
      with self.subTest(arm=arm):
        e = self.embodiment(handler=lambda r: httpx.Response(200,json={'arms':[arm]}))
        e.scene_revision = e.observation_revision
        self.assertFalse((await e.execute_action('finish_task',success=True,summary='Done'))['success'])
        self.assertIsNone(e.task_result)

  async def test_scene_from_before_a_motion_does_not_satisfy_final_observation(self):
    e = self.embodiment()
    s = self.session(e)
    e.poller.wait_for_next_frame = mock.AsyncMock(return_value=jpeg())
    old_revision = e.observation_revision
    await e.execute_action('move', **named_move('arm'))
    await s.send_latest_video_frame()
    message = {'toolResponse': {'functionResponses': [dict(id='old',name='get_robot_state',
        response=dict(arms=[],observation_revision=old_revision))]}}
    await s.send_message(message)
    self.assertFalse(message['toolResponse']['functionResponses'][0]['response']['success'])
    self.assertFalse((await e.execute_action('finish_task',success=True,summary='Done'))['success'])

  async def test_scene_suppressed_by_new_inspection_is_not_reported_as_delivered(self):
    e = self.embodiment()
    s = self.session(e)
    async def capture():
      e.manipulation.inspections['pending'] = {'plan_id':'p'}
      return jpeg()
    e.poller.wait_for_next_frame = mock.AsyncMock(side_effect=capture)
    self.assertFalse(await s.send_latest_video_frame())
    message = {'toolResponse': {'functionResponses': [dict(id='state',name='get_robot_state',response={'arms':[]})]}}
    await s.send_message(message)
    self.assertFalse(message['toolResponse']['functionResponses'][0]['response']['success'])
    self.assertIsNone(e.scene_revision)
    self.assertTrue(all('realtimeInput' not in call.args[0] for call in s.stream.Send.call_args_list))
