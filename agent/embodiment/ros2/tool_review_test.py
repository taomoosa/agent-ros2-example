import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import httpx
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.robotics_er import RoboticsER
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from embodiment.ros2.tools import ros2_tools


CONFIGS = Path(__file__).resolve().parents[2] / 'configs'


class ToolReviewTest(unittest.IsolatedAsyncioTestCase):
  async def test_custom_er_prompt_appends_guidance_and_preserves_contract(self):
    with tempfile.TemporaryDirectory() as directory:
      prompt = Path(directory)/'detect.md'
      prompt.write_text('Prefer the textured center of the blue fixture.', encoding='utf-8')
      requests = []
      def handler(request):
        requests.append(json.loads(request.content))
        return httpx.Response(200,json={'candidates':[{'finishReason':'STOP',
            'content':{'parts':[{'text':'{"targets": []}'}]}}]})
      er = RoboticsER('key',prompt_files={'detect':prompt},transport=httpx.MockTransport(handler))
      try:
        await er.reason('detect',dict(camera_id='overhead',capture_id='c',image_base64='original'), 'Move fixture', ['arm'])
        text = requests[0]['contents'][0]['parts'][1]['text']
        self.assertIn(prompt.read_text(),text)
        self.assertIn('normalized to 0..1000',text)
        self.assertIn('"targets"',text)
        self.assertIn('Move fixture',text)
      finally:
        await er.close()
      prompt.write_text('')
      with self.assertRaises(ValueError):
        RoboticsER('key',prompt_files={'detect':prompt})
      with self.assertRaises(FileNotFoundError):
        RoboticsER('key',prompt_files={'detect':Path(directory)/'missing.md'})

  def test_minimal_tools_omit_unusable_refinement_and_idle_ack(self):
    config = RobotConfig.load(CONFIGS/'minimal.json')
    self.assertEqual(1,len(config.arms))
    self.assertEqual(['world'],[c.mount for c in config.cameras])
    tools = {d['name']:d for d in ros2_tools(config)[0]['functionDeclarations']}
    self.assertNotIn('refine_grasp',tools)
    self.assertNotIn('approach_targets',tools)
    self.assertNotIn('ack',tools)
    self.assertIn('recover_arms',tools)
    self.assertIn('camera_id',tools['inspect_grasp']['parameters']['properties'])
    self.assertEqual('BLOCKING',tools['get_robot_state']['behavior'])

  async def test_driver_failure_details_survive_and_invalid_plan_cannot_repeat(self):
    requests = []
    def handler(request):
      requests.append(request)
      return httpx.Response(409,json=dict(success=False,error='Jaw jammed',code='jam',
          failed_arm_ids=['arm'],failed_phase='close',recoverable=True))
    embodiment = Ros2Embodiment(RobotConfig.load(CONFIGS/'minimal.json'),transport=httpx.MockTransport(handler))
    self.addAsyncCleanup(embodiment.close)
    embodiment.manipulation.plans['p'] = dict(plan_id='p',state='detected',targets=[{'arm_id':'arm'}])
    result = await embodiment.execute_action('pick_targets',plan_id='p')
    self.assertEqual('jam',result['code'])
    self.assertEqual('close',result['failed_phase'])
    self.assertTrue(result['recoverable'])
    self.assertNotIn('outcome',result)
    self.assertEqual('invalid',embodiment.manipulation.plan('p')['state'])
    self.assertFalse((await embodiment.execute_action('pick_targets',plan_id='p'))['success'])
    self.assertEqual(1,len(requests))
    finished = await embodiment.execute_action('finish_task',success=True,summary='Done')
    self.assertFalse(finished['success'])
    self.assertIsNone(embodiment.task_result)

  async def test_recovery_is_explicit_bounded_and_does_not_repeat_after_failed_stop(self):
    embodiment = Ros2Embodiment(RobotConfig.load(CONFIGS/'minimal.json'),max_recovery_attempts=1)
    self.addAsyncCleanup(embodiment.close)
    embodiment.robot.stop = mock.AsyncMock(return_value={'success':False,'error':'Stop not confirmed'})
    embodiment.robot.workflow = mock.AsyncMock()
    first = await embodiment.execute_action('recover_arms')
    second = await embodiment.execute_action('recover_arms')
    self.assertFalse(first['success'])
    self.assertFalse(second['success'])
    self.assertIn('limit',second['error'])
    embodiment.robot.stop.assert_awaited_once()
    embodiment.robot.workflow.assert_not_awaited()
    self.assertTrue(embodiment.manipulation.needs_recovery)

  async def test_recovery_success_clears_old_evidence_and_allows_new_detection(self):
    embodiment = Ros2Embodiment(RobotConfig.load(CONFIGS/'minimal.json'))
    self.addAsyncCleanup(embodiment.close)
    embodiment.manipulation.plans['p'] = {'state':'invalid'}
    embodiment.manipulation.inspections['stale'] = {}
    embodiment.manipulation.needs_recovery = True
    embodiment.robot.stop = mock.AsyncMock(return_value={'success':True})
    embodiment.robot.workflow = mock.AsyncMock(return_value={'success':True})
    result = await embodiment.execute_action('recover_arms')
    self.assertTrue(result['success'])
    self.assertEqual(['get_robot_state','detect_targets'],result['next_actions'])
    self.assertEqual({},embodiment.manipulation.plans)
    self.assertEqual({},embodiment.manipulation.inspections)
    self.assertFalse(embodiment.manipulation.needs_recovery)
    embodiment.robot.workflow.assert_awaited_once_with('recover')
