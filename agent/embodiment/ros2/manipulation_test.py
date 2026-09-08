import base64
import io
import json
from pathlib import Path
import unittest

import httpx
from PIL import Image
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from embodiment.ros2.robotics_er import RoboticsER, pixel


class ReasoningTest(unittest.IsolatedAsyncioTestCase):
  async def test_rest_request_uses_original_image_and_separate_model(self):
    requests = []
    def handle(request):
      requests.append(request)
      return httpx.Response(200, json={'candidates': [{'finishReason': 'STOP', 'content': {
          'parts': [{'text': '{"point": [250, 750]}'}]}}]})
    er = RoboticsER('test-key', model='test-er', transport=httpx.MockTransport(handle))
    self.addAsyncCleanup(er.close)
    result = await er.reason('refine', dict(camera_id='wrist', capture_id='c', image_base64='exact-original'), 'red part', ['left'])
    self.assertEqual({'point': [250,750]}, result)
    request = requests[0]
    self.assertTrue(str(request.url).endswith('/models/test-er:generateContent'))
    self.assertEqual('test-key', request.headers['x-goog-api-key'])
    self.assertEqual('exact-original', json.loads(request.content)['contents'][0]['parts'][0]['inlineData']['data'])

  async def test_blocked_or_malformed_model_response_fails(self):
    for response in ({}, {'candidates': [{'finishReason':'MAX_TOKENS'}]},
                     {'candidates': [{'finishReason':'STOP', 'content': {'parts':[{'text':'[]'}]}}]}):
      er = RoboticsER('key', transport=httpx.MockTransport(lambda r: httpx.Response(200, json=response)))
      try:
        with self.assertRaises(ValueError):
          await er.reason('detect', dict(camera_id='c', capture_id='c', image_base64='x'), 'task', ['left'])
      finally:
        await er.close()

  async def test_er_timeout_is_not_retried(self):
    requests = []
    def timeout(request):
      requests.append(request)
      raise httpx.ReadTimeout('Mock ER timeout', request=request)
    er = RoboticsER('key', transport=httpx.MockTransport(timeout))
    self.addAsyncCleanup(er.close)
    with self.assertRaises(httpx.ReadTimeout):
      await er.reason('detect', dict(camera_id='c',capture_id='c',image_base64='x'), 'task', ['left'])
    self.assertEqual(1,len(requests))

  def test_normalized_yx_to_original_xy(self):
    cap = dict(width=641,height=481)
    self.assertEqual([480,120], pixel([250,750], cap))
    self.assertEqual([640,480], pixel([1000,1000], cap))
    for point in ([False,0], [1001,0], [0,float('nan')], None):
      with self.assertRaises(ValueError):
        pixel(point, cap)

  async def test_invalid_detection_and_unverified_place_never_issue_motion(self):
    config = RobotConfig.load(Path(__file__).resolve().parents[2] / 'configs/dual_arm.json')
    data = io.BytesIO()
    Image.new('RGB', (48,32)).save(data, 'JPEG')
    calls = []
    def http(request):
      calls.append(request)
      return httpx.Response(200, json=dict(capture_id='c', camera_id='overhead', width=48,
          height=32, image_base64=base64.b64encode(data.getvalue()).decode()))
    def er(request):
      return httpx.Response(200, json={'candidates':[{'finishReason':'STOP', 'content':{
          'parts':[{'text':'{"targets": []}'}]}}]})
    embodiment = Ros2Embodiment(config, transport=httpx.MockTransport(http), api_key='key', robotics_transport=httpx.MockTransport(er))
    self.addAsyncCleanup(embodiment.close)
    result = await embodiment.execute_action('detect_targets', camera_id='overhead', instruction='missing part', arm_ids=['left'])
    self.assertFalse(result['success'])
    self.assertEqual(['GET'], [r.method for r in calls])
    embodiment.manipulation.plans['p'] = dict(state='picked', targets=[])
    result = await embodiment.execute_action('place_targets', plan_id='p')
    self.assertFalse(result['success'])
    self.assertEqual(1,len(calls))
