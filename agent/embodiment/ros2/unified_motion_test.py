"""Unified target selection, motion routes and interrupted motion tools."""
import asyncio
import json
import unittest
from embodiment.ros2.test_requests import pose_move
from unittest import mock
import httpx
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.robot_client import Ros2RobotClient
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from embodiment.ros2.tools import ros2_tools
from embodiment.ros2.followup_test import CONFIGS, POSE


class UnifiedMotionTest(unittest.IsolatedAsyncioTestCase):
  def config(self):
    return RobotConfig.load(CONFIGS/'primitives.json')

  async def test_shared_config_and_motion_calls_use_common_http_routes(self):
    requests=[]
    def handler(request):
      requests.append((request.url.path,json.loads(request.content)))
      return httpx.Response(200,json={'success':True})
    cfg=self.config()
    self.assertIn('home',cfg.position_names['arm'])
    robot=Ros2RobotClient(cfg,transport=httpx.MockTransport(handler))
    self.addAsyncCleanup(robot.close)
    await robot.move(**pose_move(arm_id='arm',**POSE))
    await robot.move(all_arms=True, targets=[dict(kind='pose', **POSE)])
    await robot.workflow('reset')
    await robot.gripper(**dict(arm_ids=['arm'], opening=1.))
    await robot.stop()
    self.assertEqual(['/v1/move']*3+['/v1/gripper','/v1/stop'],[r[0] for r in requests])
    self.assertEqual('named',requests[2][1]['targets'][0]['kind'])
    self.assertEqual({'all_arms':True},requests[-1][1])
    declarations={d['name']:d for d in ros2_tools(cfg)[0]['functionDeclarations']}
    for name in ('move','gripper','stop'):
      props=declarations[name]['parameters']['properties']
      self.assertIn('arm_ids',props)
      self.assertIn('all_arms',props)
    self.assertNotIn('behavior',declarations['stop'])

  async def test_invalid_selection_is_rejected_before_http(self):
    robot=Ros2RobotClient(self.config())
    self.addAsyncCleanup(robot.close)
    robot._request=mock.AsyncMock()
    for args in ({'arm_ids':[]},{'arm_ids':['arm','arm']},{'arm_ids':['unknown']},
                 {'arm_ids':['arm'],'all_arms':True},{'all_arms':False}):
      with self.assertRaises(ValueError):
        await robot.move(targets=[{'kind':'named','name':'home'}],**args)
    with self.assertRaises(ValueError):await robot.stop(arm_ids=['arm'],all_arms=True)
    robot._request.assert_not_awaited()

  async def test_new_motion_tools_keep_held_object_and_unknown_outcome_guards(self):
    for name,args in [('move',dict(all_arms=True,targets=[dict(kind='named',name='home')])),
                      ('gripper',dict(all_arms=True,opening=1.))]:
      paths=[]
      def handler(request):
        paths.append(request.url.path)
        return (httpx.Response(200,json={'success':True}) if request.url.path=='/v1/stop'
                else httpx.Response(504,json={'success':False,'outcome':'unknown'}))
      e=Ros2Embodiment(self.config(),transport=httpx.MockTransport(handler))
      self.addAsyncCleanup(e.close)
      result=await e.execute_action(name,**args)
      self.assertFalse(result['success'])
      self.assertTrue(result['stop_result']['success'])
      self.assertEqual('/v1/stop',paths[-1])
      self.assertTrue(e.manipulation.needs_recovery)
      before=len(paths)
      self.assertFalse((await e.execute_action(name,**args))['success'])
      self.assertEqual(before,len(paths))
      e.manipulation.needs_recovery=False
      e.manipulation.plans['p']={'state':'picked'}
      self.assertFalse((await e.execute_action(name,**args))['success'])
      self.assertEqual(before,len(paths))

  async def test_new_stop_selection_bypasses_motion_lock(self):
    entered,release=asyncio.Event(),asyncio.Event()
    async def handler(request):
      if request.url.path=='/v1/move':
        entered.set()
        await release.wait()
      return httpx.Response(200,json={'success':True})
    e=Ros2Embodiment(self.config(),transport=httpx.MockTransport(handler))
    self.addAsyncCleanup(e.close)
    move=asyncio.create_task(e.execute_action('move',all_arms=True,targets=[dict(kind='named',name='home')]))
    await entered.wait()
    stop=await asyncio.wait_for(e.execute_action('stop',all_arms=True),1.)
    self.assertTrue(stop['success'])
    release.set()
    self.assertEqual('unknown',(await move)['outcome'])

  async def test_all_topologies_expose_only_current_actions(self):
    for filename in ('minimal.json','single_arm.json','dual_arm.json','primitives.json'):
      cfg=RobotConfig.load(CONFIGS/filename)
      names={d['name'] for d in ros2_tools(cfg)[0]['functionDeclarations']}
      self.assertTrue({'move','gripper','stop','reset_arms'} <= names)
      self.assertFalse({'move_arm','move_arms','set_gripper'} & names)
      self.assertEqual(14 if any(c.mount=='flange' for c in cfg.cameras) else 12,len(names))
      e=Ros2Embodiment(cfg)
      self.addAsyncCleanup(e.close)
      e.robot._request=mock.AsyncMock()
      for name in ('move_arm','move_arms','set_gripper'):
        result=await e.execute_action(name)
        self.assertFalse(result['success'])
        self.assertIn('Unknown ROS2 action',result['error'])
      e.robot._request.assert_not_awaited()

  def test_removed_mode_settings_require_explicit_config_migration(self):
    import tempfile
    from pathlib import Path
    data=json.loads((CONFIGS/'minimal.json').read_text())
    with tempfile.TemporaryDirectory() as directory:
      path=Path(directory)/'config.json'
      for mode in ('legacy','primitives'):
        data['server']={'driver_mode':mode}
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'driver_mode was removed'):
          RobotConfig.load(path)

  def test_named_catalog_contains_names_and_descriptions_and_rejects_coordinates(self):
    self.assertEqual({'arm':{'home':'Initial position for task startup.'}},self.config().position_names)
    for hardware in ({'named_positions':{'arm':{'home':POSE}}},
                     {'position_names':{'arm':{'home':POSE}}},
                     {'position_names':{'arm':['home','home']}},
                     {'position_names':{'arm':['']}},
                     {'position_names':{'arm':{'home':''}}},
                     {'position_names':{'arm':{'':'Initial position.'}}}):
      with self.assertRaises(ValueError):
        RobotConfig._position_names(hardware)


  async def test_gemini_move_rejects_poses_before_http_and_exposes_metadata_only(self):
    cfg=self.config()
    declaration=next(d for d in ros2_tools(cfg)[0]['functionDeclarations'] if d['name']=='move')
    props=declaration['parameters']['properties']['targets']['items']['properties']
    self.assertEqual(['pixel','named'],props['kind']['enum'])
    self.assertFalse({'position','orientation','frame_id','reference'} & props.keys())
    self.assertIn('Initial position for task startup.',props['name']['description'])
    e=Ros2Embodiment(cfg)
    self.addAsyncCleanup(e.close)
    e.robot._request=mock.AsyncMock(return_value={'success':True})
    for targets in ([dict(kind='pose',**POSE)],
                    [dict(kind='named',name='home'),dict(kind='pose',**POSE)],
                    [dict(kind='named',name='home',frame_id='world')]):
      result=await e.execute_action('move',arm_ids=['arm'],targets=targets)
      self.assertFalse(result['success'],result)
    e.robot._request.assert_not_awaited()
    for target in (dict(kind='named',name='home'),
                   dict(kind='pixel',capture_id='observed-capture',pixel=[24,16],profile='tabletop')):
      self.assertTrue((await e.execute_action('move',arm_ids=['arm'],targets=[target]))['success'])
    self.assertEqual(2,e.robot._request.await_count)
