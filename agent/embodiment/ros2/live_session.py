"""ROS2 Live session extension for capture-bound, agent-owned grasp assessment."""

import asyncio

from session_manager import SessionManager


class Ros2SessionManager(SessionManager):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._last_inspection_frame = float('-inf')

  async def _write(self, message):
    # The caller holds the upstream stream lock across image and tool response.
    if self.stream is None:
      raise RuntimeError('Stream is not initialized')
    await asyncio.get_running_loop().run_in_executor(None, self.stream.Send, message)

  async def send_latest_video_frame(self, timeout=2.0):
    if self.embodiment.manipulation.inspections:
      # Keep the requested inspection evidence visible until the agent assesses it.
      return False
    return await super().send_latest_video_frame(timeout)

  async def send_message(self, message):
    manipulation = self.embodiment.manipulation
    responses = message.get('toolResponse', {}).get('functionResponses', [])
    inspections = [r for r in responses if r.get('name') == 'inspect_grasp'
                   and r.get('response', {}).get('success') is True]
    video = 'video' in message.get('realtimeInput', {})
    if not inspections and not video:
      return await super().send_message(message)
    async with self._stream_lock:
      if video:
        if manipulation.inspections:
          return
        await asyncio.sleep(max(0., 1. - (self.loop.time() - self._last_inspection_frame)))
      for response in inspections:
        result = response['response']
        observation_id = result['observation_id']
        try:
          # Wait after the last possible ordinary video frame. Live accepts at
          # most one video frame per second. No mosaic can interleave here.
          await asyncio.sleep(1.)
          inspection = manipulation.inspection(observation_id)
          capture = inspection['capture']
          await self._write({'realtimeInput': {'video': {
              'mimeType': 'image/jpeg', 'data': capture['image_base64']}}})
          self._last_inspection_frame = self.loop.time()
          # A concurrent stop may invalidate the observation while Send awaits.
          manipulation.inspection(observation_id)['delivered'] = True
        except (ValueError, RuntimeError, OSError) as exc:
          manipulation.inspections.pop(observation_id, None)
          response['response'] = {'success': False, 'error': f'Inspection image delivery failed: {exc}'}
      await self._write(message)
