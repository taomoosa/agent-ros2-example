"""Background camera poller that stitches camera images (Lite version)."""

import asyncio
import collections
from collections.abc import Callable
import io
import logging
import math
import time

from PIL import Image
from PIL import ImageDraw

logger = logging.getLogger(__name__)

_STABILITY_SAMPLE_SIZE = (64, 48)


def frame_difference_score(previous: bytes, current: bytes) -> float:
  """Return normalized mean luminance difference between two JPEG frames."""
  if not previous or not current:
    return 1.0
  try:
    previous_image = Image.open(io.BytesIO(previous)).convert("L").resize(
        _STABILITY_SAMPLE_SIZE
    )
    current_image = Image.open(io.BytesIO(current)).convert("L").resize(
        _STABILITY_SAMPLE_SIZE
    )
    previous_pixels = previous_image.tobytes()
    current_pixels = current_image.tobytes()
    difference = sum(
        abs(left - right)
        for left, right in zip(previous_pixels, current_pixels)
    )
    return difference / (len(previous_pixels) * 255.0)
  except Exception:  # pylint: disable=broad-except
    return 1.0


def stitch_camera_images(
    images: dict[str, bytes],
    cell_size: int = 384,
) -> bytes:
  """Stitch camera images into a grid. Returns JPEG bytes."""
  if not images:
    return b""
  if len(images) == 1:
    _, data = next(iter(images.items()))
    img = Image.open(io.BytesIO(data))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()

  n = len(images)
  cols = math.ceil(math.sqrt(n))
  rows = math.ceil(n / cols)
  canvas = Image.new("RGB", (cols * cell_size, rows * cell_size), (0, 0, 0))
  draw = ImageDraw.Draw(canvas)

  for i, (name, jpeg_bytes) in enumerate(images.items()):
    try:
      img = Image.open(io.BytesIO(jpeg_bytes))
      img = img.resize((cell_size, cell_size), Image.LANCZOS)
      row, col = divmod(i, cols)
      x_offset = col * cell_size
      y_offset = row * cell_size

      canvas.paste(img, (x_offset, y_offset))

      # Draw camera name with a simple shadow for visibility
      text = name.replace("_", " ").upper()
      try:
        draw.text((x_offset + 11, y_offset + 11), text, fill=(0, 0, 0))
        draw.text((x_offset + 10, y_offset + 10), text, fill=(0, 255, 240))
      except Exception:
        pass  # ignore font loading issues if any
    except Exception as e:  # pylint: disable=broad-except
      logger.warning("Failed to process image %s: %s", name, e)
      return b""

  buf = io.BytesIO()
  canvas.save(buf, format="JPEG", quality=85)
  return buf.getvalue()


class CameraPoller:
  """Polls cameras at a fixed rate and pushes stitched frames to a queue.

  Supports both streaming (via stream_camera) and polling (via
  get_camera_snapshot fallback).
  """

  def __init__(
      self,
      robot_client,
      video_input_queue: asyncio.Queue,
      camera_ids: list[str],
      poll_hz: float = 5.0,
      push_hz: float = 1.0,
      cell_size: int = 384,
      stability_threshold: float = 0.04,
      clock: Callable[[], float] = time.monotonic,
  ):
    """Initialize the camera poller."""
    self._robot = robot_client
    self._queue = video_input_queue
    self._camera_ids = camera_ids
    self._poll_interval = 1.0 / poll_hz
    self._push_interval = 1.0 / push_hz
    self._cell_size = cell_size
    self._stability_threshold = stability_threshold
    self._clock = clock
    self._running = False
    self.push_enabled = True  # Push stitched frames to Gemini video queue

    # Store latest individual frames
    self._current_frames: dict[str, bytes] = {}

    # Store latest 10 stitched images
    self._frame_buffer = collections.deque(maxlen=10)
    self._frame_sequence = 0
    self._new_frame_cond = asyncio.Condition()
    self._stability_frame = b""
    self._stable_since: float | None = None
    self._last_frame_difference = 1.0

  @property
  def latest_frame(self) -> bytes:
    """The most recent stitched JPEG frame."""
    if self._frame_buffer:
      return self._frame_buffer[-1]
    return b""

  @property
  def stable_for_seconds(self) -> float:
    """Seconds the camera view has remained below the motion threshold."""
    if self._stable_since is None:
      return 0.0
    return max(0.0, self._clock() - self._stable_since)

  def is_stable_for(self, seconds: float) -> bool:
    """Return whether the camera view has been stable for the requested time."""
    return self.stable_for_seconds >= max(0.0, float(seconds))

  def record_frame_stability(
      self,
      frame: bytes,
      *,
      observed_at: float | None = None,
  ) -> float:
    """Update the continuous stable interval from a newly captured frame."""
    now = self._clock() if observed_at is None else observed_at
    if not self._stability_frame:
      self._stable_since = now
      score = 0.0
    else:
      score = frame_difference_score(self._stability_frame, frame)
      if score > self._stability_threshold:
        self._stable_since = now
    self._stability_frame = frame
    self._last_frame_difference = score
    return score

  async def get_stream(self):
    """Async generator yielding stitched MJPEG frames as they arrive."""
    try:
      while self._running:
        async with self._new_frame_cond:
          await self._new_frame_cond.wait()
          if not self._frame_buffer:
            continue
          frame = self._frame_buffer[-1]
        yield frame
    except asyncio.CancelledError:
      return

  async def wait_for_next_frame(self) -> bytes:
    """Wait for and return a frame produced after this method is called."""
    async with self._new_frame_cond:
      initial_sequence = self._frame_sequence
      await self._new_frame_cond.wait_for(
          lambda: (
              self._frame_sequence > initial_sequence or not self._running
          )
      )
      if self._frame_sequence <= initial_sequence or not self._frame_buffer:
        return b""
      return self._frame_buffer[-1]

  def stop(self) -> None:
    self._running = False

  async def _stream_camera_task(self, camera_id: str):
    """Background task reading an individual camera stream."""
    while self._running:
      try:
        # Try streaming first if supported.
        async for frame in self._robot.stream_camera(camera_id):
          if not self._running:
            break
          self._current_frames[camera_id] = frame
      except ValueError:
        # Fallback to polling snapshots if stream is not supported (like Spot).
        logger.debug(
            "Streaming not supported for camera %s, falling back to polling.",
            camera_id,
        )
        while self._running:
          frame = await self._robot.get_camera_snapshot(camera_id)
          if frame:
            self._current_frames[camera_id] = frame
          await asyncio.sleep(self._poll_interval)
      except asyncio.CancelledError:
        break
      except Exception as e:
        logger.warning("Camera stream task %s error: %s", camera_id, e)
        await asyncio.sleep(1.0)

  async def run(self) -> None:
    """Main polling loop. Call via asyncio.create_task()."""
    self._running = True
    last_push_time = 0.0

    # Start independent streamer tasks
    stream_tasks = [
        asyncio.create_task(self._stream_camera_task(cid))
        for cid in self._camera_ids
    ]

    logger.info(
        "CameraPoller started: cameras=%s poll=%.1fHz push=%.1fHz",
        self._camera_ids,
        1.0 / self._poll_interval,
        1.0 / self._push_interval,
    )

    while self._running:
      try:
        # Check if we have received frames from all requested cameras
        if all(cid in self._current_frames for cid in self._camera_ids):
          images = {cid: self._current_frames[cid] for cid in self._camera_ids}

          # Run CPU-bound stitching in a thread pool to avoid blocking the asyncio event loop
          loop = asyncio.get_event_loop()
          stitched = await loop.run_in_executor(
              None, stitch_camera_images, images, self._cell_size
          )

          if stitched:
            self.record_frame_stability(stitched)
            self._frame_buffer.append(stitched)
            self._frame_sequence += 1
            async with self._new_frame_cond:
              self._new_frame_cond.notify_all()

            # Push to Gemini at push_hz rate (only when enabled).
            if self.push_enabled:
              now = asyncio.get_event_loop().time()
              if now - last_push_time >= self._push_interval:
                # Replace any stale frame in queue with the latest.
                while not self._queue.empty():
                  try:
                    self._queue.get_nowait()
                  except asyncio.QueueEmpty:
                    break
                await self._queue.put(stitched)
                last_push_time = now
      except asyncio.CancelledError:
        break
      except Exception as e:  # pylint: disable=broad-except
        logger.warning("CameraPoller stitch loop error: %s", e)

      await asyncio.sleep(self._poll_interval)

    self._running = False
    for task in stream_tasks:
      task.cancel()

    async with self._new_frame_cond:
      self._new_frame_cond.notify_all()

    logger.info("CameraPoller stopped")
