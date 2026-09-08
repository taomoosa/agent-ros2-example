"""Capture complete fresh camera sets, reusing upstream JPEG composition."""

import asyncio
import logging

from camera_poller import stitch_camera_images

logger = logging.getLogger(__name__)


class Ros2CameraPoller:
  def __init__(self, robot, queue: asyncio.Queue, poll_hz: float = 1.0):
    if poll_hz <= 0:
      raise ValueError("poll_hz must be positive")
    self.robot = robot
    self.queue = queue
    self.interval = 1.0 / poll_hz
    self.latest_frame = b""
    self.push_enabled = True
    self._lock = asyncio.Lock()
    self._stop = asyncio.Event()

  async def wait_for_next_frame(self) -> bytes:
    # Acquire the lock before issuing requests, so post-action observations
    # cannot reuse a capture that started before the action completed.
    async with self._lock:
      if self._stop.is_set():
        return b""
      try:
        cameras = self.robot.config.cameras
        results = await asyncio.gather(*[
            self.robot.get_camera_snapshot(camera.id) for camera in cameras
        ], return_exceptions=True)
        for result in results:
          if isinstance(result, BaseException):
            raise result
        frame = await asyncio.to_thread(
            stitch_camera_images,
            {camera.id: data for camera, data in zip(cameras, results)},
        )
        self.latest_frame = frame
        return frame
      except Exception as exc:
        self.latest_frame = b""
        # Never publish a mixture containing a cached camera view.
        logger.warning("Could not capture all ROS2 cameras: %s", exc)
        return b""

  async def run(self):
    while not self._stop.is_set():
      frame = await self.wait_for_next_frame()
      if self.push_enabled and frame:
        while not self.queue.empty():
          self.queue.get_nowait()
        self.queue.put_nowait(frame)
      try:
        await asyncio.wait_for(self._stop.wait(), timeout=self.interval)
      except asyncio.TimeoutError:
        pass

  def stop(self):
    self._stop.set()
