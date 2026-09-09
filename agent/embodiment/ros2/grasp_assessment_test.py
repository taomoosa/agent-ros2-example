import base64
import copy
import io
from pathlib import Path
import unittest
from unittest import mock

from PIL import Image
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.live_session import Ros2SessionManager
from embodiment.ros2.ros2_embodiment import Ros2Embodiment


class GraspAssessmentTest(unittest.IsolatedAsyncioTestCase):
  async def asyncSetUp(self):
    config = RobotConfig.load(Path(__file__).resolve().parents[2] / 'configs/dual_arm.json')
    self.embodiment = Ros2Embodiment(config)
    self.addAsyncCleanup(self.embodiment.close)
    self.manipulation = self.embodiment.manipulation
    self.manipulation.plans['p'] = dict(plan_id='p',state='picked',motion_stamp_ns=100,
                                      targets=[{'arm_id':'left'}, {'arm_id':'right'}])
    self.image = io.BytesIO()
    Image.new('RGB',(80,60),'green').save(self.image,'JPEG')
    self.encoded = base64.b64encode(self.image.getvalue()).decode()
    self.sequence = 100

    async def capture(camera_id):
      self.sequence += 1
      return dict(camera_id=camera_id,capture_id=f'capture{self.sequence}',stamp_ns=self.sequence,
                  image_base64=self.encoded,width=80,height=60)
    self.embodiment.robot.observation = mock.AsyncMock(side_effect=capture)
    self.embodiment.robot.get_robot_state = mock.AsyncMock(return_value={'arms':[
        dict(id=arm,moving=False,gripper={'opening':0.}) for arm in ('left','right')]})
    self.embodiment.robot.workflow = mock.AsyncMock(return_value=dict(
        self.manipulation.plans['p'],success=True,state='verified'))
    self.embodiment.reasoning.reason = mock.AsyncMock(side_effect=AssertionError('ER must not assess grasp'))
    self.session = Ros2SessionManager('mock',self.embodiment,api_key='key',heartbeat_enabled=False)
    self.addCleanup(self.session.observation.close)
    self.addAsyncCleanup(self.session.bus.shutdown)
    self.messages = []
    self.session.stream = mock.Mock()
    self.session.stream.Send.side_effect = lambda message: self.messages.append(copy.deepcopy(message))

  async def inspect(self, arm, *, deliver=True):
    result = await self.embodiment.execute_action('inspect_grasp',plan_id='p',arm_id=arm)
    self.assertTrue(result['success'],result)
    if deliver:
      await self.session.send_message({'toolResponse':{'functionResponses':[
          dict(id=arm,name='inspect_grasp',response=result)]}})
    return dict(arm_id=arm,observation_id=result['observation_id'],success=True,
                reason='The requested object is held clear of the support surface.')

  async def test_original_image_precedes_response_and_agent_assesses_both_arms(self):
    observations = [await self.inspect('left'),await self.inspect('right')]
    self.assertEqual(4,len(self.messages))
    for index in (0,2):
      self.assertEqual(self.encoded,self.messages[index]['realtimeInput']['video']['data'])
      self.assertIn('observation_id',self.messages[index+1]['toolResponse']['functionResponses'][0]['response'])
    before = len(self.messages)
    await self.session.send_message({'realtimeInput':{'video':{'data':'mosaic','mimeType':'image/jpeg'}}})
    self.assertEqual(before,len(self.messages))
    result = await self.embodiment.execute_action('verify_grasp',plan_id='p',observations=observations)
    self.assertEqual('verified',result['state'])
    self.assertEqual(observations,result['assessment'])
    submitted = self.embodiment.robot.workflow.call_args.kwargs['observations']
    self.assertEqual(['capture101','capture102'],[o['capture_id'] for o in submitted])
    self.assertEqual({},self.manipulation.inspections)
    self.embodiment.reasoning.reason.assert_not_awaited()

  async def test_missing_undelivered_wrong_arm_or_duplicate_evidence_is_rejected(self):
    left = await self.inspect('left',deliver=False)
    right = await self.inspect('right',deliver=False)
    cases = [[left], [left,left], [left,dict(right,observation_id=left['observation_id'])],
             [left,right], [dict(left,success='true'),right], [dict(left,reason=''),right]]
    for observations in cases:
      with self.subTest(observations=observations):
        result = await self.embodiment.execute_action('verify_grasp',plan_id='p',observations=observations)
        self.assertFalse(result['success'])
    self.embodiment.robot.workflow.assert_not_awaited()
    self.embodiment.reasoning.reason.assert_not_awaited()

  async def test_agent_failure_consumes_evidence_and_keeps_placement_blocked(self):
    observations = [await self.inspect('left'),await self.inspect('right')]
    observations[1].update(success=False,reason='The right gripper appears empty; uncertain grasp.')
    self.embodiment.robot.workflow.return_value = dict(success=False,plan_id='p',error='Unconfirmed')
    result = await self.embodiment.execute_action('verify_grasp',plan_id='p',observations=observations)
    self.assertFalse(result['success'])
    self.assertEqual('picked',self.manipulation.plan('p')['state'])
    observations[1]['success'] = True
    self.assertFalse((await self.embodiment.execute_action('verify_grasp',plan_id='p',observations=observations))['success'])
    self.assertFalse((await self.embodiment.execute_action('place_targets',plan_id='p'))['success'])
    self.embodiment.robot.workflow.assert_awaited_once()

  async def test_stale_image_or_moving_arm_cannot_be_inspected(self):
    self.sequence = 0
    self.assertFalse((await self.embodiment.execute_action('inspect_grasp',plan_id='p',arm_id='left'))['success'])
    self.sequence = 100
    self.embodiment.robot.get_robot_state.return_value['arms'][0]['moving'] = True
    self.assertFalse((await self.embodiment.execute_action('inspect_grasp',plan_id='p',arm_id='left'))['success'])
    self.assertEqual({},self.manipulation.inspections)

  async def test_delivery_failure_and_stop_invalidate_evidence(self):
    observation = await self.inspect('left',deliver=False)
    self.session.stream.Send.side_effect = OSError('Mock image send failed')
    with self.assertRaises(OSError):
      await self.session.send_message({'toolResponse':{'functionResponses':[
          dict(name='inspect_grasp',response=dict(success=True,observation_id=observation['observation_id']))]}})
    self.assertEqual({},self.manipulation.inspections)
    self.assertTrue(self.embodiment.session_lost)
    self.assertFalse((await self.embodiment.execute_action('inspect_grasp',plan_id='p',arm_id='left'))['success'])
    self.manipulation.invalidate()
    self.assertEqual({},self.manipulation.inspections)
    with self.assertRaises(ValueError):
      self.manipulation.inspection(observation['observation_id'])
