"""ROS2 Live session extension for capture-bound, agent-owned grasp assessment."""

import asyncio

from session_manager import SessionManager
from embodiment.ros2.bounded_io import thread_call


# TOOL EXTENSION: deliver original visual evidence before its response; see server/docs/extending.md.

class Ros2SessionManager(SessionManager):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._last_activity = self.loop.time()
    self._last_inspection_frame = float('-inf')
    self._frame_results = {}
    self._sent_results = {}
    self._call_ids = set()
    self.interruption_reason = 'Gemini session disconnected; inspect and recover before restarting'

  async def _create_stream(self):
    timing = self.embodiment.config.timing
    return await thread_call(lambda: self.client.create_stream(timeout=timing.live_io_timeout),
        timing.live_io_timeout, abandoned=lambda stream: stream.Shutdown())

  async def _shutdown_stream(self):
    if self.stream is not None:
      await thread_call(self.stream.Shutdown, self.embodiment.config.timing.cleanup_timeout)

  async def _send_stream(self, message):
    try:
      await thread_call(lambda: self.stream.Send(message), self.embodiment.config.timing.live_io_timeout)
    except (TimeoutError, OSError, RuntimeError):
      self.embodiment.interrupt(session_lost=True)
      raise
    if "realtimeInput" not in message:
      self._last_activity = self.loop.time()

  def _on_done(self):
    if not self.loop.is_closed():
      self.loop.call_soon_threadsafe(lambda: self.embodiment.interrupt(session_lost=True))
      super()._on_done()

  def _on_message(self, message):
    # WebSocket callbacks may run on a worker thread; admission belongs to the loop.
    def accept():
      self._last_activity = self.loop.time()
      if self.embodiment.session_lost:
        return
      calls = (message or {}).get('toolCall', {}).get('functionCalls', [])
      ids = [str(c.get('id', '')) for c in calls]
      if calls and (any(not i for i in ids) or len(ids) != len(set(ids)) or
                    any(i in self._call_ids for i in ids)):
        self.interruption_reason = 'Missing or repeated tool call ID; session stopped without replay'
        self._on_done()
        return
      self._call_ids.update(ids)
      super(Ros2SessionManager, self)._on_message(message)
    self.loop.call_soon_threadsafe(accept)

  async def _reconnect(self, max_retries=3):
    # Reconnecting a fresh model session cannot reconstruct a physical outcome.
    await self.embodiment.execute_action('stop')
    return False

  def _to_ui_event(self, event):
    result = super()._to_ui_event(event)
    if result.get('type') == 'tool_call' and result.get('id') in self._sent_results:
      result = dict(result, result=self._sent_results.pop(result['id']))
    return result

  async def _write(self, message):
    # The caller holds the upstream stream lock across image and tool response.
    if self.stream is None:
      raise RuntimeError('Stream is not initialized')
    await self._send_stream(message)

  async def send_latest_video_frame(self, timeout=None):
    timeout = self.embodiment.observation_timeout if timeout is None else timeout
    generation = self.embodiment._stop_generation
    revision = self.embodiment.observation_revision
    sent = False
    if not self.embodiment.manipulation.inspections:
      sent = await super().send_latest_video_frame(timeout)
      sent = sent and not self.embodiment.manipulation.inspections
    self._frame_results[asyncio.current_task()] = (sent, generation, revision)
    return sent

  async def send_message(self, message):
    try:
      async with asyncio.timeout(self.embodiment.observation_timeout):
        return await self._send_message(message)
    except TimeoutError:
      self.embodiment.interrupt(session_lost=True)
      raise

  async def _send_message(self, message):
    manipulation = self.embodiment.manipulation
    responses = message.get('toolResponse', {}).get('functionResponses', [])
    observation = self._frame_results.pop(asyncio.current_task(), None) if responses else None
    for response in responses:
      result = response.get('response', {})
      if observation is not None:
        sent, generation, revision = observation
        fresh = (sent and generation == self.embodiment._stop_generation and
                 revision == self.embodiment.observation_revision and
                 result.get('observation_revision', revision) == revision)
        result['post_action_observation'] = fresh
        if generation != self.embodiment._stop_generation:
          result.update(success=False, error='Operation interrupted before result delivery', outcome='unknown')
        if response.get('name') == 'get_robot_state':
          if not fresh:
            result.update(success=False, error='Fresh scene image unavailable; observe again before continuing')
          elif result.get('success') is not False:
            result['success'] = True
            manipulation.final_observation_required = False
            self.embodiment.scene_revision = revision
      if response.get('id'):
        self._sent_results[response['id']] = result
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
          result['image_delivered'] = True
        except (ValueError, RuntimeError, OSError) as exc:
          manipulation.inspections.pop(observation_id, None)
          response['response'] = {'success': False, 'error': f'Inspection image delivery failed: {exc}'}
      for response in responses:
        if response.get('id'):
          self._sent_results[response['id']] = response['response']
      await self._write(message)
