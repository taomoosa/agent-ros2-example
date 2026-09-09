r"""Manages a Gemini Live bidirectional streaming session (Lite version).

This is a simplified version of the session manager that uses only
the public Gemini Live API (WebSocket + JSON) with zero external dependencies.
"""

import asyncio
import base64
import datetime
import json
import logging
import time
from typing import Any, Mapping

from core import audio_handler
from core import decision_making
from core import event_bus
from core import observation
from core import tool_call_handler
from model import live_api_client


logger = logging.getLogger(__name__)


def get_default_heartbeat_text() -> str:
  """Returns the default heartbeat text to send to the model."""
  return (
      "[HEARTBEAT] If no task is active, call 'ack' and wait for user"
      " input. If a task is active: observe the scene. If the current"
      " step is progressing correctly, call 'ack'. If the current step"
      " is complete, call 'run_instruction' with the next step. If the"
      " overall goal is achieved, call 'reset' and inform the user."
  )


class SessionManager:
  """Manages a Gemini Live bidirectional streaming session.

  Uses an EventBus for typed pub/sub event dispatching. Components register
  as self-subscribing handlers; the session manager bridges UI-relevant
  events to the WebSocket via a ui_queue.
  """

  _DEFAULT_SYSTEM_INSTRUCTION = (
      "You are a helpful AI assistant. Keep your responses"
      " concise. You can see the user's camera or screen"
      " which is shared as realtime input images with you."
  )

  def __init__(
      self,
      model: str,
      embodiment_instance,
      input_sample_rate: int = 16000,
      tools=None,
      system_instruction: str | None = None,
      developer_instruction: str | None = None,
      response_modality: str = "AUDIO",
      dump_video_dir: str | None = None,
      api_key: str | None = None,
      heartbeat_interval_seconds: float = 2.0,
      heartbeat_enabled: bool = True,
      heartbeat_min_delay_seconds: float = 0.0,
      agent_peers: dict[str, str] | None = None,
      peer_name: str = "unknown",
      use_event_driven_heartbeat: bool = False,
      media_resolution: str = "low",
      heartbeat_text: str = "",
      enable_send_message_to_user: bool = False,
      endpoint_type: str = "gemini_live_api",
      thinking_level: str = "none",
  ):
    self.model = model
    self.embodiment = embodiment_instance
    self.input_sample_rate = input_sample_rate
    self.tools = tools or []
    self.system_instruction = (
        system_instruction or self._DEFAULT_SYSTEM_INSTRUCTION
    )
    self.developer_instruction = developer_instruction or ""
    self.thinking_level = thinking_level
    self.response_modality = response_modality
    self.dump_video_dir = dump_video_dir
    self.heartbeat_interval_seconds = heartbeat_interval_seconds
    self.heartbeat_enabled = heartbeat_enabled
    self.heartbeat_min_delay_seconds = heartbeat_min_delay_seconds
    self._agent_peers = agent_peers or {}
    self._peer_name = peer_name
    self.use_event_driven_heartbeat = use_event_driven_heartbeat
    self.heartbeat_text = heartbeat_text
    self._enable_send_message_to_user = enable_send_message_to_user
    self._last_heartbeat_sent_time = 0.0
    self._interrupted = False

    self._heartbeat_sent_count = 0
    self._turn_completed_count = 0
    self._heartbeat_in_flight = False

    self.api_key = api_key
    self._endpoint_type = endpoint_type

    # Create the API client.
    self.client = live_api_client.GeminiLiveApiClient(api_key=api_key)
    self.stream = None
    self._stream_lock = asyncio.Lock()
    self.loop = asyncio.get_running_loop()

    # Create the event bus.
    self.bus = event_bus.EventBus()

    # Initialize Observation early to expose queues.
    self.observation = observation.Observation(
        self.bus,
        self.loop,
        self,  # SessionManager provides send_message for stream I/O
        0,  # Session start time not available yet
        self.embodiment,
        self.input_sample_rate,
        self.dump_video_dir,
        media_resolution=media_resolution,
    )
    self.decision_making = None
    self.tool_handler = None
    self.audio_handler = None

  def _get_heartbeat_text(self) -> str:
    if self.heartbeat_text:
      return self.heartbeat_text
    return get_default_heartbeat_text()

  def set_embodiment(self, embodiment):
    """Updates the embodiment dynamically."""
    self.embodiment = embodiment
    self.observation.set_embodiment(embodiment)
    if self.tool_handler:
      self.tool_handler.set_embodiment(embodiment)

  async def _create_stream(self):
    return self.client.create_stream()

  async def _shutdown_stream(self):
    self.stream.Shutdown()

  async def _send_stream(self, message):
    await asyncio.get_running_loop().run_in_executor(None, self.stream.Send, message)

  async def send_message(self, msg: Any) -> None:
    """Send a JSON message to the Gemini Live API stream.

    Serializes all writes through an asyncio.Lock to prevent concurrent
    stream.Send calls from different tasks.
    """
    assert self.stream is not None, "Stream is not initialized"
    async with self._stream_lock:
      await self._send_stream(msg)

  async def send_text_with_fresh_video(self, text: str) -> None:
    """Atomically send a current Spot frame followed by user text."""
    poller = getattr(self.embodiment, "poller", None)
    chunk = b""
    if poller is not None:
      try:
        chunk = await asyncio.wait_for(
            poller.wait_for_next_frame(), timeout=getattr(self.embodiment, "observation_timeout", 2.0)
        )
      except asyncio.TimeoutError:
        logger.warning("Timed out waiting for a pre-text camera frame")

    messages = []
    if chunk:
      await self.observation.dump_frame(chunk)
      messages.append({
          "realtimeInput": {
              "video": {
                  "mimeType": "image/jpeg",
                  "data": base64.b64encode(chunk).decode("utf-8"),
              }
          }
      })
    messages.append({
        "clientContent": {
            "turns": [{
                "role": "user",
                "parts": [{"text": text}],
            }],
            "turnComplete": True,
        }
    })

    assert self.stream is not None, "Stream is not initialized"
    async with self._stream_lock:
      for message in messages:
        await self._send_stream(message)
    if chunk:
      logger.info("Sent synchronized fresh video frame with user text")

  async def send_latest_video_frame(self, timeout: float = 2.0) -> bool:
    """Send the first poller frame captured after this method is called."""
    poller = getattr(self.embodiment, "poller", None)
    if poller is None:
      return False
    try:
      chunk = await asyncio.wait_for(
          poller.wait_for_next_frame(), timeout=timeout
      )
    except asyncio.TimeoutError:
      logger.warning("Timed out waiting for a post-tool camera frame")
      return False
    if not chunk:
      logger.warning("Camera poller stopped before producing a post-tool frame")
      return False

    await self.observation.dump_frame(chunk)
    await self.send_message({
        "realtimeInput": {
            "video": {
                "mimeType": "image/jpeg",
                "data": base64.b64encode(chunk).decode("utf-8"),
            }
        }
    })
    logger.info("Sent synchronized fresh video frame")
    return True

  def get_audio_queue(self) -> asyncio.Queue:
    return self.observation.get_audio_queue()

  def get_video_queue(self) -> asyncio.Queue:
    return self.observation.get_video_queue()

  def get_text_queue(self) -> asyncio.Queue:
    return self.observation.get_text_queue()

  def _on_message(self, msg):
    """Callback for incoming Gemini stream messages (JSON dicts)."""
    if msg is not None:
      logger.debug("Received message: %s", msg)
      self.loop.call_soon_threadsafe(
          self.bus.publish_nowait,
          event_bus.Event(
              type=event_bus.EventType.MODEL_RESPONSE,
              source=event_bus.EventSource.ASSISTANT,
              data=msg,
          ),
      )

  def _on_done(self):
    """Callback when the Gemini stream ends."""
    logger.error("[Session] on_done fired — stream disconnected")
    self.loop.call_soon_threadsafe(
        self.bus.publish_nowait,
        event_bus.Event(
            type=event_bus.EventType.SESSION_DONE,
            source=event_bus.EventSource.ASSISTANT,
        ),
    )

  async def _send_setup(self) -> None:
    """Sends the JSON setup message to the Live API stream."""
    model_name = self.model
    if not model_name.startswith("models/"):
      model_name = f"models/{model_name}"

    # Combine SI and DI into a single system instruction.
    combined_prompt = self.system_instruction
    if self.developer_instruction:
      combined_prompt += "\\n\\n" + self.developer_instruction

    modality_str = self.response_modality.upper()
    if modality_str == "AUDIO":
      modality = "AUDIO"
    elif modality_str == "TEXT":
      modality = "TEXT"
    else:
      raise ValueError(f"Unsupported response modality: {self.response_modality}")

    setup_dict = {
        "setup": {
            "model": model_name,
            "systemInstruction": {
                "role": "system",
                "parts": [{"text": combined_prompt}]
            },
            "generationConfig": {
                "responseModalities": [modality]
            },
            "tools": []
        }
    }

    # Modified for the ROS2 CLI: expose transcripts for audio-only Live models.
    if modality == "AUDIO":
      setup_dict["setup"]["outputAudioTranscription"] = {}

    # Add thinking config if a level is set.
    if self.thinking_level and self.thinking_level != "none":
      setup_dict["setup"]["generationConfig"]["thinkingConfig"] = {
          "thinkingLevel": self.thinking_level.upper(),
      }

    # Add tools.
    for tool in self.tools:
      setup_dict["setup"]["tools"].append(tool)

    logger.info(
        "[Session] Sending setup: model=%s, modality=%s, tools=%d",
        model_name,
        modality_str,
        len(self.tools),
    )

    assert self.stream is not None
    await asyncio.get_running_loop().run_in_executor(
        None, self.stream.Send, setup_dict
    )

  async def _reconnect(self, max_retries: int = 3) -> bool:
    """Attempts to reconnect the Gemini stream with exponential backoff."""
    import time
    now = time.monotonic()
    last_rc = getattr(self, '_last_reconnect_time', 0)
    rapid_count = getattr(self, '_rapid_reconnect_count', 0)
    
    if now - last_rc < 5.0:
      rapid_count += 1
    else:
      rapid_count = 0
    
    self._last_reconnect_time = now
    self._rapid_reconnect_count = rapid_count
    
    if rapid_count >= max_retries:
      logger.error("Stream disconnecting too rapidly (%d times). Aborting.", rapid_count)
      return False

    for attempt in range(max_retries):
      wait_time = 2**attempt
      logger.warning(
          "Stream disconnected. Reconnecting in %ds (attempt %d/%d)...",
          wait_time,
          attempt + 1,
          max_retries,
      )
      await asyncio.sleep(wait_time)
      try:
        self.stream = await self._create_stream()
        self.stream.Start(
            lambda msg: self._on_message(msg),
            lambda: self._on_done(),
        )
        await self._send_setup()
        logger.info("Reconnected successfully on attempt %d.", attempt + 1)
        return True
      except Exception as e:  # pylint: disable=broad-except
        logger.error("Reconnection attempt %d failed: %s", attempt + 1, e)
    logger.error("All %d reconnection attempts failed.", max_retries)
    return False

  async def start_session(
      self,
      audio_output_callback,
      audio_interrupt_callback=None,
      text_output_callback=None,
  ):
    self.session_start_time = self.loop.time()
    self.observation.session_start_time = self.session_start_time

    # --- Create the Gemini stream ---
    self.stream = await self._create_stream()
    self.stream.Start(
        lambda msg: self._on_message(msg),
        lambda: self._on_done(),
    )

    # --- Register handlers on the bus ---

    # DecisionMaking: routes MODEL_RESPONSE → typed events
    self.decision_making = decision_making.DecisionMaking(
        self.bus,
        text_output_callback=text_output_callback,
        audio_interrupt_callback=audio_interrupt_callback,
    )

    # AudioResponseHandler: accumulates AUDIO_CHUNK → AUDIO_RESPONSE
    self.audio_handler = audio_handler.AudioResponseHandler(
        self.bus,
        audio_output_callback=audio_output_callback,
    )

    # ToolCallHandler: executes TOOL_CALL → TOOL_RESULT
    self.tool_handler = tool_call_handler.ToolCallHandler(
        self.bus,
        self,  # session_manager for send_message
        self.embodiment,
        agent_peers=self._agent_peers,
        peer_name=self._peer_name,
        text_output_callback=text_output_callback,
        enable_send_message_to_user=self._enable_send_message_to_user,
        blocking_tools=tool_call_handler.blocking_tool_names(self.tools),
    )

    # --- UI queue bridge: events → async generator yield ---

    ui_queue: asyncio.Queue = asyncio.Queue()

    _UI_EVENT_TYPES = [
        event_bus.EventType.GEMINI_TEXT,
        event_bus.EventType.GEMINI_THOUGHT,
        event_bus.EventType.USER_TRANSCRIPT,
        event_bus.EventType.AUDIO_RESPONSE,
        event_bus.EventType.TOOL_RESULT,
        event_bus.EventType.TURN_COMPLETE,
        event_bus.EventType.TELEMETRY,
        event_bus.EventType.INTERRUPTED,
        event_bus.EventType.ERROR,
        event_bus.EventType.LOG,
        event_bus.EventType.SESSION_DONE,
        event_bus.EventType.TEXT_INPUT,
        event_bus.EventType.TRANSPARENT_HISTORY,
    ]

    async def _ui_handler(event: event_bus.Event):
      await ui_queue.put(event)

    self.bus.subscribe(_UI_EVENT_TYPES, _ui_handler)

    # --- Send Setup Message ---

    await self._send_setup()

    # --- Start input tasks and heartbeat ---

    has_poller = getattr(self.embodiment, "poller", None) is not None
    self.observation.start_input_tasks(
        skip_video=self.use_event_driven_heartbeat and has_poller
    )

    heartbeat_trigger = None
    if self.use_event_driven_heartbeat:
      heartbeat_signal = asyncio.Event()
      _trigger_source = [""]

      async def _on_interrupt(event: event_bus.Event):
        self._interrupted = True

      async def _on_heartbeat_trigger(event: event_bus.Event):
        if event.type == event_bus.EventType.TURN_COMPLETE:
          self._heartbeat_in_flight = False
          if self._interrupted:
            self._interrupted = False
            return

        now = asyncio.get_running_loop().time()
        if now - self._last_heartbeat_sent_time < 0.5:
          return

        if event.type == event_bus.EventType.TURN_COMPLETE:
          had_tool_call = (
              event.data.get("had_tool_call", False)
              if isinstance(event.data, dict)
              else False
          )
          if had_tool_call:
            return

        source = (
            event.data.get("source", event.type.value)
            if event.data and isinstance(event.data, dict)
            else event.type.value
        )
        _trigger_source[0] = source
        heartbeat_signal.set()

      if self.heartbeat_enabled:
        self.bus.subscribe(
            [
                event_bus.EventType.TURN_COMPLETE,
                event_bus.EventType.HEARTBEAT_TRIGGER,
            ],
            _on_heartbeat_trigger,
        )
        self.bus.subscribe(
            [event_bus.EventType.INTERRUPTED],
            _on_interrupt,
        )

      async def _heartbeat_loop():
        try:
          _trigger_source[0] = "session_start"
          heartbeat_signal.set()

          while True:
            try:
              await asyncio.wait_for(
                  heartbeat_signal.wait(), timeout=10.0
              )
              trigger = _trigger_source[0]
            except asyncio.TimeoutError:
              trigger = "safety_timeout"
              logger.warning(
                  "No model response within 10s, sending recovery heartbeat"
              )

            try:
              if (
                  self.tool_handler is not None
                  and self.tool_handler.tool_executing.is_set()
              ):
                heartbeat_signal.clear()
                logger.info(
                    "Suppressing %s heartbeat while blocking tool executes",
                    trigger,
                )
                continue

              if has_poller:
                chunk = await self.observation.get_video_queue().get()
                video_msg = {
                    "realtimeInput": {
                        "video": {
                            "mimeType": "image/jpeg",
                            "data": base64.b64encode(chunk).decode("utf-8"),
                        }
                    }
                }
                await self.observation.dump_frame(chunk)
                await self.send_message(video_msg)
                logger.info("Sent video frame with heartbeat")
              else:
                try:
                  await asyncio.wait_for(
                      self.observation.wait_for_next_frame(), timeout=2.0
                  )
                except asyncio.TimeoutError:
                  logger.warning(
                      "Timeout waiting for webcam frame, proceeding"
                  )

              heartbeat_signal.clear()

              self._heartbeat_sent_count += 1
              self._heartbeat_in_flight = True
              heartbeat_msg = {
                  "clientContent": {
                      "turns": [{
                          "role": "user",
                          "parts": [{"text": self._get_heartbeat_text()}],
                      }],
                      "turnComplete": True,
                  }
              }
              await self.send_message(heartbeat_msg)
              self._last_heartbeat_sent_time = (
                  asyncio.get_running_loop().time()
              )
              logger.info("Sent heartbeat (trigger=%s)", trigger)
              await ui_queue.put({"type": "heartbeat_sent"})
            except Exception as e:
              logger.error("Heartbeat send failed: %s", e)
              await asyncio.sleep(1.0)
        except asyncio.CancelledError:
          pass
        except Exception as e:
          logger.error("Heartbeat loop failed: %s", e)

      heartbeat_task = (
          asyncio.create_task(_heartbeat_loop())
          if self.heartbeat_enabled
          else None
      )

    else:
      # Legacy fixed-interval heartbeat with turn_complete chaining.
      heartbeat_trigger = asyncio.Event()

      async def _heartbeat_loop():
        try:
          await asyncio.sleep(2.0)
          while True:
            if (
                self.tool_handler is not None
                and self.tool_handler.tool_executing.is_set()
            ):
              await asyncio.sleep(0.5)
              continue
            while (
                self.decision_making is not None
                and self.decision_making.speaking.is_set()
            ):
              await asyncio.sleep(1.0)

            heartbeat_msg = {
                "clientContent": {
                    "turns": [{
                        "role": "user",
                        "parts": [{"text": self._get_heartbeat_text()}],
                    }],
                    "turnComplete": True,
                }
            }
            heartbeat_trigger.clear()
            await self.send_message(heartbeat_msg)
            hb_send_time = time.monotonic()
            logger.info("Sent heartbeat")
            self._heartbeat_sent_count += 1
            await ui_queue.put({"type": "heartbeat_sent"})
            try:
              await asyncio.wait_for(
                  heartbeat_trigger.wait(),
                  timeout=self.heartbeat_interval_seconds,
              )
            except asyncio.TimeoutError:
              pass
            elapsed = time.monotonic() - hb_send_time
            remaining = self.heartbeat_min_delay_seconds - elapsed
            if remaining > 0:
              await asyncio.sleep(remaining)
        except asyncio.CancelledError:
          pass
        except Exception as e:
          logger.error("Heartbeat loop failed: %s", e)

      heartbeat_task = (
          asyncio.create_task(_heartbeat_loop())
          if self.heartbeat_enabled
          else None
      )

    # --- Start the event bus ---

    self.bus.start()

    # --- Main loop: drain UI queue and yield to WebSocket ---

    try:
      while True:
        event = await ui_queue.get()

        if isinstance(event, dict):
          yield event
          continue

        if event.type == event_bus.EventType.SESSION_DONE:
          if await self._reconnect():
            continue
          break

        if event.type == event_bus.EventType.TURN_COMPLETE:
          self._turn_completed_count += 1

        # Suppress heartbeat-caused INTERRUPTED events.
        if self._enable_send_message_to_user:
          _suppress = (
              event.type == event_bus.EventType.INTERRUPTED
              and event.source == event_bus.EventSource.ASSISTANT
              and self._heartbeat_sent_count > self._turn_completed_count
              and not (
                  isinstance(event.data, dict)
                  and event.data.get("tts_preempt")
              )
          )
        else:
          _suppress = (
              event.type == event_bus.EventType.INTERRUPTED
              and self._heartbeat_in_flight
              and not (
                  isinstance(event.data, dict)
                  and event.data.get("tts_preempt")
              )
          )
        if _suppress:
          logger.info("Suppressing INTERRUPTED — heartbeat-caused")
          continue

        ui_event = self._to_ui_event(event)
        yield ui_event
        if (
            ui_event.get("type") == "turn_complete"
            and not self.use_event_driven_heartbeat
        ):
          if heartbeat_trigger is not None:
            heartbeat_trigger.set()

    finally:
      if heartbeat_task:
        heartbeat_task.cancel()
      await self.observation.stop_input_tasks()
      self.observation.close()
      await self.bus.shutdown()
      if self.stream:
        try:
          await self._shutdown_stream()
        except Exception:  # pylint: disable=broad-except
          pass
        self.stream = None

  def _to_ui_event(self, event: event_bus.Event) -> Mapping[str, Any]:
    """Convert a typed Event to the UI dict format for the WebSocket."""

    if event.type == event_bus.EventType.GEMINI_TEXT:
      return {"type": "gemini", "text": event.data.get("text", "")}
    elif event.type == event_bus.EventType.GEMINI_THOUGHT:
      return {"type": "gemini_thought", "text": event.data.get("text", "")}
    elif event.type == event_bus.EventType.TEXT_INPUT:
      return {"type": "text_input", "text": event.data}
    elif event.type == event_bus.EventType.USER_TRANSCRIPT:
      return {"type": "user_transcript", "text": event.data.get("text", "")}
    elif event.type == event_bus.EventType.AUDIO_RESPONSE:
      return {
          "type": "audio_response",
          "data": event.data.get("audio_data", ""),
      }
    elif event.type == event_bus.EventType.TOOL_RESULT:
      return event.data  # Already a dict
    elif event.type == event_bus.EventType.TURN_COMPLETE:
      d: dict[str, Any] = {"type": "turn_complete"}
      if isinstance(event.data, dict) and "had_tool_call" in event.data:
        d["had_tool_call"] = event.data["had_tool_call"]
      return d
    elif event.type == event_bus.EventType.TELEMETRY:
      d: dict[str, Any] = {"type": "telemetry"}
      if isinstance(event.data, dict):
        for key in ["token_usage", "server_ttft_ms"]:
          if key in event.data:
            d[key] = event.data[key]
      return d
    elif event.type == event_bus.EventType.INTERRUPTED:
      return {"type": "interrupted"}
    elif event.type == event_bus.EventType.ERROR:
      return {"type": "error", "error": event.data}
    elif event.type == event_bus.EventType.TRANSPARENT_HISTORY:
      # In Lite, transparent history data is already a dict (no proto).
      return {"type": "transparent_history", "data": event.data}
    elif event.type == event_bus.EventType.LOG:
      return {"type": "log", "data": event.data}
    else:
      logger.warning("Unknown UI event type: %s", event.type)
      return {"type": "unknown", "data": str(event.data)}
