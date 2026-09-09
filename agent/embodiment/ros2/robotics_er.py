"""Separate Gemini Robotics ER image reasoning calls using the existing HTTP dependency."""

import asyncio
import json
from pathlib import Path
import re

import httpx

from embodiment.ros2.robot_client import _number


DEFAULT_ROBOTICS_MODEL = 'gemini-robotics-er-2-preview'


class RoboticsER:
  def __init__(self, api_key, *, model=DEFAULT_ROBOTICS_MODEL, transport=None, prompt_files=None, timeout=90.):
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', model):
      raise ValueError('Invalid Robotics ER model ID')
    self.extra_prompts = {}
    for mode, path in (prompt_files or {}).items():
      if mode not in {'detect', 'refine'}:
        raise ValueError('Prompt files support detect and refine only')
      content = Path(path).read_text(encoding='utf-8')
      if not content.strip():
        raise ValueError(f'Empty {mode} prompt file: {path}')
      self.extra_prompts[mode] = content
    self.timeout = _number(timeout, "er_timeout", .001, 3600.)
    self.model = model
    self.api_key = api_key
    self.client = httpx.AsyncClient(base_url='https://generativelanguage.googleapis.com',
                                   transport=transport, timeout=self.timeout, follow_redirects=False)

  async def reason(self, mode, capture, instruction, arm_ids):
    try:
      async with asyncio.timeout(self.timeout):
        return await self._reason(mode, capture, instruction, arm_ids)
    except TimeoutError as exc:
      raise httpx.ReadTimeout(f"Robotics ER elapsed deadline exceeded: {self.timeout}s") from exc

  async def _reason(self, mode, capture, instruction, arm_ids):
    if not self.api_key:
      raise ValueError('Gemini Robotics ER requires an API key')
    if mode not in ('detect', 'refine'):
      raise ValueError('Unknown reasoning mode')
    prompt = (Path(__file__).parent / 'prompts' / f'{mode}.md').read_text()
    if mode in self.extra_prompts:
      prompt += '\nApplication guidance (retain the coordinate and JSON contract above):\n' + self.extra_prompts[mode]
    if capture.get('kind') == 'plane':
      calibration = capture['plane_calibration']
      prompt += ('\nThis image uses a calibrated plane, not measured depth. Select only points '
                 'on that plane inside the original-pixel valid_region. Do not select raised '
                 'object surfaces or assume their pixels give the point directly underneath. '
                 'Return no targets if the task requires off-plane contact points. Plane context: '
                 + json.dumps(calibration))
    prompt += '\nTask context: ' + json.dumps(dict(instruction=instruction, arm_ids=arm_ids,
        camera_id=capture['camera_id'], capture_id=capture['capture_id']))
    response = await self.client.post(f'/v1beta/models/{self.model}:generateContent',
        headers={'x-goog-api-key': self.api_key}, json={
          'contents': [{'role': 'user', 'parts': [
            {'inlineData': {'mimeType': 'image/jpeg', 'data': capture['image_base64']}},
            {'text': prompt}]}],
          'generationConfig': {'responseMimeType': 'application/json',
                               'thinkingConfig': {'thinkingLevel': 'high'}}})
    response.raise_for_status()
    try:
      candidates = response.json()['candidates']
      if len(candidates) != 1 or candidates[0]['finishReason'] != 'STOP':
        raise ValueError('Robotics ER response was blocked or incomplete')
      parts = candidates[0]['content']['parts']
      result = json.loads(''.join(p.get('text', '') for p in parts if not p.get('thought')))
      if not isinstance(result, dict):
        raise ValueError('Robotics ER must return a JSON object')
      return result
    except (KeyError, IndexError, TypeError) as exc:
      raise ValueError('Malformed Robotics ER response') from exc

  async def close(self):
    await self.client.aclose()


def pixel(point, capture):
  if not isinstance(point, list) or len(point) != 2:
    raise ValueError('Robotics ER did not find a valid point')
  y, x = [_number(v, 'normalized point', 0, 1000) for v in point]
  return [round(x * (capture['width']-1) / 1000), round(y * (capture['height']-1) / 1000)]
