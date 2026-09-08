"""Observation component for Proactive Agent.

Receives audio, video, and text from the external environment and streams
them to the Gemini session as JSON dicts (public BidiGenerateContent API).
"""

import asyncio
import base64
import logging
import os

from core import event_bus


class EventQueueLogHandler(logging.Handler):
  """Logging handler that puts log messages into an asyncio.Queue."""

  def __init__(self, bus: event_bus.EventBus, loop: asyncio.AbstractEventLoop):
    super().__init__()
    self.bus = bus
    self.loop = loop

  def emit(self, record: logging.LogRecord):
    try:
      event = event_bus.Event(
          type=event_bus.EventType.LOG,
          source=event_bus.EventSource.ASSISTANT,
          data={
              "level": record.levelname,
              "message": record.getMessage(),
              "module": record.module,
              "funcName": record.funcName,
              "lineno": record.lineno,
          },
      )
      self.loop.call_soon_threadsafe(self.bus.publish_nowait, event)
    except Exception:  # pylint: disable=broad-exception-caught
      self.handleError(record)


class Observation:
  """Component for receiving information from the external environment and logging."""

  def __init__(
      self,
      bus: event_bus.EventBus,
      loop: asyncio.AbstractEventLoop,
      session_manager,
      session_start_time,
      embodiment,
      input_sample_rate=16000,
      dump_video_dir=None,
      episodic_logger=None,
      media_resolution="low",
  ):
    self.bus = bus
    self.loop = loop
    self.session_manager = session_manager
    self.session_start_time = session_start_time
    self.input_sample_rate = input_sample_rate
    self.dump_video_dir = dump_video_dir
    self.episodic_logger = episodic_logger
    self.media_resolution = media_resolution
    self._frame_counter = 0
    # Event signaled after each video frame is written to the stream.
    # Used by the heartbeat loop (non-poller mode) to wait for a fresh
    # frame before sending ".".
    self._frame_sent_event = asyncio.Event()

    self.set_embodiment(embodiment)

    # Setup custom logging handler
    self.log_handler = EventQueueLogHandler(self.bus, self.loop)
    logging.getLogger().addHandler(self.log_handler)



  def set_embodiment(self, embodiment):
    """Updates the queues to read from the given embodiment.

    Restarts the input tasks so they read from the new queues. Without this,
    the old tasks would be stuck blocked on queue.get() from the previous
    embodiment's (now-unused) queues.
    """
    self.audio_queue = embodiment.get_audio_queue()
    self.video_queue = embodiment.get_video_queue()
    self.text_queue = embodiment.get_text_queue()

    # Restart input tasks if they are running, so they pick up new queues.
    if hasattr(self, 'send_audio_task'):
      self.clear_queues()
      self.send_audio_task.cancel()
      if self.send_video_task:
        self.send_video_task.cancel()
      self.send_text_task.cancel()
      self.start_input_tasks()

  def get_audio_queue(self) -> asyncio.Queue:
    return self.audio_queue

  def get_video_queue(self) -> asyncio.Queue:
    return self.video_queue

  def get_text_queue(self) -> asyncio.Queue:
    return self.text_queue

  def clear_queues(self) -> None:
    """Purges all pending audio, video, and text observation queues."""
    for attr in ("audio_queue", "video_queue", "text_queue"):
      q = getattr(self, attr, None)
      if q is not None:
        while not q.empty():
          try:
            q.get_nowait()
          except asyncio.QueueEmpty:
            break
    logging.info("Purged observation queues.")

  def start_input_tasks(self, skip_video: bool = False):
    self.send_audio_task = asyncio.create_task(self._send_audio())
    if not skip_video:
      self.send_video_task = asyncio.create_task(self._send_video())
    else:
      self.send_video_task = None
    self.send_text_task = asyncio.create_task(self._send_text())

  async def stop_input_tasks(self):
    tasks = []
    if hasattr(self, "send_audio_task"):
      self.send_audio_task.cancel()
      tasks.append(self.send_audio_task)
    if hasattr(self, "send_video_task") and self.send_video_task is not None:
      self.send_video_task.cancel()
      tasks.append(self.send_video_task)
    if hasattr(self, "send_text_task"):
      self.send_text_task.cancel()
      tasks.append(self.send_text_task)
    if tasks:
      await asyncio.gather(
          *tasks,
          return_exceptions=True,
      )

  async def _send_audio(self):
    try:
      while True:
        chunk = await self.audio_queue.get()
        msg = {
            "realtimeInput": {
                "audio": {
                    "mimeType": f"audio/pcm;rate={self.input_sample_rate}",
                    "data": base64.b64encode(chunk).decode("utf-8"),
                }
            }
        }
        
        await self.session_manager.send_message(msg)
        # Only log when the audio chunk has actual speech energy (not silence).
        # PCM 16-bit samples range ±32767; threshold filters mic noise.
        if len(chunk) >= 2:
          samples = memoryview(chunk).cast("h")  # signed 16-bit
          peak = max(abs(s) for s in samples)
          if peak > 100:
            logging.info(
                "Streamed user audio to model (%d bytes, peak=%d)",
                len(chunk),
                peak,
            )
    except asyncio.CancelledError:
      pass
    except Exception as e:
      await self.bus.publish(
          event_bus.Event(
              type=event_bus.EventType.ERROR,
              source=event_bus.EventSource.ASSISTANT,
              data=f"send_audio died: {e}",
          )
      )

  async def _send_video(self):
    try:
      while True:
        chunk = await self.video_queue.get()
        await self.bus.publish(
            event_bus.Event(
                type=event_bus.EventType.REAL_TIME_IMAGE_SENT,
                source=event_bus.EventSource.USER,
                data=chunk,
            )
        )
        
        await self.dump_frame(chunk)

        msg = {
            "realtimeInput": {
                "video": {
                    "mimeType": "image/jpeg",
                    "data": base64.b64encode(chunk).decode("utf-8"),
                }
            }
        }

        await self.session_manager.send_message(msg)
        self._frame_sent_event.set()
        logging.info("Sent video frame")
    except asyncio.CancelledError:
      pass
    except Exception as e:
      await self.bus.publish(
          event_bus.Event(
              type=event_bus.EventType.ERROR,
              source=event_bus.EventSource.ASSISTANT,
              data=f"send_video died: {e}",
          )
      )

  async def _send_text(self):
    try:
      while True:
        text = await self.text_queue.get()
        logging.info("Sending text: %s", text)

        # Ground each text turn in a frame captured immediately before it.
        # Spot's UI can keep updating while the model otherwise retains an
        # older image as its most recent visual context.
        await self.bus.publish(
            event_bus.Event(
                type=event_bus.EventType.TEXT_INPUT,
                source=event_bus.EventSource.USER,
                data=text,
            )
        )

        msg = {
            "clientContent": {
                "turns": [{
                    "role": "user",
                    "parts": [{"text": text}]
                }],
                "turnComplete": True,
            }
        }

        send_synced = getattr(
            self.session_manager, "send_text_with_fresh_video", None
        )
        if send_synced is not None:
          await send_synced(text)
        else:
          await self.session_manager.send_message(msg)
        logging.info("Streamed user text to model: %s", text)
    except asyncio.CancelledError:
      pass
    except Exception as e:
      await self.bus.publish(
          event_bus.Event(
              type=event_bus.EventType.ERROR,
              source=event_bus.EventSource.ASSISTANT,
              data=f"send_text died: {e}",
          )
      )

  async def wait_for_next_frame(self) -> None:
    """Wait until _send_video delivers a fresh frame to the stream.

    Blocks indefinitely until _send_video sets the event after a
    successful send_message call. Safe because:
    - Only one caller (heartbeat loop) at a time
    - Writes are serialized via asyncio.Lock in send_message,
      so the frame is guaranteed to be in the server buffer when
      the event fires
    """
    self._frame_sent_event.clear()
    await self._frame_sent_event.wait()

  async def dump_frame(self, chunk: bytes) -> None:
    """Dumps a video frame to the configured directory."""
    if not self.dump_video_dir:
      return
    current_counter = self._frame_counter
    self._frame_counter += 1
    path = os.path.join(self.dump_video_dir, f"frame_{current_counter:06d}.jpg")

    def _write_frame(file_path: str, data: bytes):
      with open(file_path, "wb") as f:
        f.write(data)

    try:
      await asyncio.to_thread(_write_frame, path, chunk)
    except Exception as e:
      logging.warning("Failed to write video frame asynchronously: %s", e)

  def close(self):
    """Removes the log handler to prevent leaks across sessions."""
    logging.getLogger().removeHandler(self.log_handler)
