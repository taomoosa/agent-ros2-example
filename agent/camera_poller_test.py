"""Tests for camera poller frame synchronization."""

import asyncio
import io
import unittest

import camera_poller
from PIL import Image


def _jpeg(value: int) -> bytes:
  image = Image.new("L", (80, 60), value)
  output = io.BytesIO()
  image.save(output, format="JPEG")
  return output.getvalue()


class CameraPollerTest(unittest.IsolatedAsyncioTestCase):

  def test_requires_three_continuous_seconds_of_stable_frames(self):
    now = [0.0]
    poller = camera_poller.CameraPoller(
        robot_client=None,
        video_input_queue=asyncio.Queue(),
        camera_ids=["hand_color_image"],
        clock=lambda: now[0],
    )

    poller.record_frame_stability(_jpeg(80))
    now[0] = 2.9
    poller.record_frame_stability(_jpeg(80))
    self.assertFalse(poller.is_stable_for(3.0))

    now[0] = 3.1
    poller.record_frame_stability(_jpeg(80))
    self.assertTrue(poller.is_stable_for(3.0))

  def test_visible_frame_motion_restarts_stability_window(self):
    now = [0.0]
    poller = camera_poller.CameraPoller(
        robot_client=None,
        video_input_queue=asyncio.Queue(),
        camera_ids=["hand_color_image"],
        clock=lambda: now[0],
    )

    poller.record_frame_stability(_jpeg(40))
    now[0] = 2.5
    score = poller.record_frame_stability(_jpeg(220))

    self.assertGreater(score, 0.04)
    self.assertEqual(0.0, poller.stable_for_seconds)
    now[0] = 5.4
    self.assertFalse(poller.is_stable_for(3.0))
    now[0] = 5.6
    self.assertTrue(poller.is_stable_for(3.0))

  async def test_wait_for_next_frame_does_not_return_existing_frame(self):
    poller = camera_poller.CameraPoller(
        robot_client=None,
        video_input_queue=asyncio.Queue(),
        camera_ids=["hand_color_image"],
    )
    poller._running = True  # pylint: disable=protected-access
    poller._frame_buffer.append(b"old")  # pylint: disable=protected-access
    poller._frame_sequence = 1  # pylint: disable=protected-access

    waiter = asyncio.create_task(poller.wait_for_next_frame())
    await asyncio.sleep(0)
    self.assertFalse(waiter.done())

    async with poller._new_frame_cond:  # pylint: disable=protected-access
      poller._frame_buffer.append(b"new")  # pylint: disable=protected-access
      poller._frame_sequence += 1  # pylint: disable=protected-access
      poller._new_frame_cond.notify_all()  # pylint: disable=protected-access

    self.assertEqual(b"new", await waiter)


if __name__ == "__main__":
  unittest.main()
