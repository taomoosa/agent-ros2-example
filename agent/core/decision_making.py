"""Decides the next step based on model responses.

Subscribes to MODEL_RESPONSE events and routes them into typed events
(TOOL_CALL, GEMINI_TEXT, AUDIO_CHUNK, TURN_COMPLETE, INTERRUPTED) for
downstream handlers.

This Lite version parses JSON dicts from the public BidiGenerateContent
WebSocket API instead of protobuf messages.
"""

import asyncio
import base64
import time
from typing import Any, Awaitable, Callable

import logging

from core import event_bus


def _is_internal_heartbeat_text(text: str) -> bool:
  return text.lstrip().startswith("[HEARTBEAT]")


class DecisionMaking:
  """A component that routes model responses into typed events on the event bus.

  This component decides the next step based on model responses. Currently
  this is implemented as pure event routing — each model response is parsed
  and re-published as a more specific event type for downstream handlers.

  Future extensions may add planning, state tracking, or other decision
  logic here.
  """

  def __init__(
      self,
      bus: event_bus.EventBus,
      text_output_callback: Callable[[str], Awaitable[None]] | None = None,
      audio_interrupt_callback: Callable[[], Awaitable[None]] | None = None,
  ):
    self._bus = bus
    self._text_output_callback = text_output_callback
    self._audio_interrupt_callback = audio_interrupt_callback
    # Simple timing instead of telemetry tracker.
    self._turn_start_time: float | None = None
    self._first_response_time: float | None = None
    # True while TTS synthesis + playback is in-flight. The heartbeat
    # loop checks this to avoid sending prompts that would interrupt
    # speech playback.
    #
    # Because the EventBus dispatches each handler as a separate
    # asyncio.Task, turn_complete can be processed concurrently while a
    # previous text_output_callback (TTS) is still running. We track
    # in-flight TTS calls with a counter and only clear the flag when all
    # calls have finished.
    self.speaking = asyncio.Event()
    self._pending_tts_count = 0
    self._state_lock = asyncio.Lock()
    self._current_turn_has_tool_call = False
    # The currently active non-blocking TTS background task, if any.
    # Used to cancel in-flight TTS when new text arrives or the user
    # interrupts the model.
    self._active_tts_task: asyncio.Task | None = None
    # Cumulative token usage from usageMetadata messages.
    self._usage_metadata: dict[str, Any] = {}
    bus.subscribe([event_bus.EventType.MODEL_RESPONSE], self._handle)

  # ---- TTS lifecycle management -------------------------------------------

  async def _decrement_tts_and_maybe_clear(self) -> None:
    """Decrement TTS counter; clear speaking when it hits zero."""
    async with self._state_lock:
      self._pending_tts_count -= 1
      if self._pending_tts_count <= 0:
        self._pending_tts_count = 0
        self.speaking.clear()

  async def _cancel_active_tts(self) -> None:
    """Cancel the currently active TTS background task, if any.

    Awaits the task to ensure its cleanup (e.g. decrementing the speaking
    counter) completes before returning.
    """
    task = self._active_tts_task
    if task is not None and not task.done():
      task.cancel()
      try:
        await task
      except asyncio.CancelledError:
        pass
    self._active_tts_task = None

  async def _start_tts_task(self, text: str) -> None:
    """Start TTS synthesis in a non-blocking background task.

    If a previous TTS task is still active, it is cancelled first and the
    browser is notified to stop audio playback (via a ``tts_preempt``
    INTERRUPTED event).  The new TTS task runs concurrently with
    subsequent event processing (e.g. tool calls), eliminating the delay
    between speech announcement and action execution.

    Args:
      text: The text to synthesize.
    """
    # Cancel any in-flight TTS before updating state to avoid deadlock
    # (the cancelled task's finally block also acquires _state_lock).
    if self._active_tts_task and not self._active_tts_task.done():
      await self._cancel_active_tts()
      # Notify the browser to stop playing the old audio.
      await self._bus.publish(
          event_bus.Event(
              type=event_bus.EventType.INTERRUPTED,
              source=event_bus.EventSource.ASSISTANT,
              data={'tts_preempt': True},
          )
      )

    # Update speaking state for the new TTS call.
    async with self._state_lock:
      self.speaking.set()
      self._pending_tts_count += 1

    # Launch TTS in the background — returns immediately.
    self._active_tts_task = asyncio.create_task(self._run_tts_background(text))

  async def _run_tts_background(self, text: str) -> None:
    """Execute the TTS callback in a background task.

    Handles both normal completion and cancellation.  The speaking counter
    is always decremented in the ``finally`` block to keep the state
    consistent.
    """
    try:
      if self._text_output_callback:
        await self._text_output_callback(text)
    except asyncio.CancelledError:
      logging.info('TTS cancelled for: %s...', text[:30])
    except Exception as e:  # pylint: disable=broad-except
      logging.error('text_output_callback failed: %s', e)
    finally:
      await self._decrement_tts_and_maybe_clear()

  # ---- Simple timing helpers ----------------------------------------------

  def _mark_turn_start(self) -> None:
    if self._turn_start_time is None:
      self._turn_start_time = time.monotonic()

  def _mark_first_response(self) -> None:
    if self._first_response_time is None:
      self._first_response_time = time.monotonic()

  def _reset_timing(self) -> dict[str, Any]:
    """Reset timing and return metrics for the completed turn."""
    metrics = {}
    if self._turn_start_time and self._first_response_time:
      metrics['ttft_ms'] = round(
          (self._first_response_time - self._turn_start_time) * 1000, 1
      )
    if self._usage_metadata:
      metrics['token_usage'] = dict(self._usage_metadata)
    self._turn_start_time = None
    self._first_response_time = None
    self._usage_metadata = {}
    return metrics

  # ---- Main event handler -------------------------------------------------

  async def _handle(self, event: event_bus.Event) -> None:
    """Process a MODEL_RESPONSE event.

    The event.data is a parsed JSON dictionary.
    """
    response = event.data
    self._mark_turn_start()

    # --- Tool call ---
    if "toolCall" in response:
      self._current_turn_has_tool_call = True
      self._mark_first_response()
      await self._bus.publish(
          event_bus.Event(
              type=event_bus.EventType.TOOL_CALL,
              source=event_bus.EventSource.ASSISTANT,
              data=response["toolCall"],
          )
      )

    # --- Server content ---
    if "serverContent" in response:
      await self._handle_server_content(response["serverContent"])

  async def _handle_server_content(self, server_content: dict) -> None:
    """Route server content to appropriate events."""

    # Handle model turn (audio/text parts)
    if "modelTurn" in server_content:
      for part in server_content["modelTurn"].get("parts", []):
        # In Gemini Live API, text parts look like: {"text": "Hello"}
        # Some are thoughts if thought=True, though public API might not have this yet.
        if part.get("thought"):
          self._mark_first_response()
          await self._bus.publish(
              event_bus.Event(
                  type=event_bus.EventType.GEMINI_THOUGHT,
                  source=event_bus.EventSource.ASSISTANT,
                  data={"text": part.get("text", "")},
              )
          )
        elif "inlineData" in part:
          self._mark_first_response()
          # Publish raw audio chunk — AudioResponseHandler accumulates.
          await self._bus.publish(
              event_bus.Event(
                  type=event_bus.EventType.AUDIO_CHUNK,
                  source=event_bus.EventSource.ASSISTANT,
                  data=base64.b64decode(part["inlineData"].get("data", "")),
              )
          )
        elif "text" in part:
          if _is_internal_heartbeat_text(part["text"]):
            continue
          self._mark_first_response()
          # Publish text event immediately so downstream handlers (UI,
          # logging) receive it without waiting for TTS synthesis.
          await self._bus.publish(
              event_bus.Event(
                  type=event_bus.EventType.GEMINI_TEXT,
                  source=event_bus.EventSource.ASSISTANT,
                  data={"text": part["text"]},
              )
          )
          # Start TTS in a non-blocking background task.
          await self._start_tts_task(part["text"])

    # Modified for the ROS2 CLI: display audio-model output transcripts.
    if "outputTranscription" in server_content:
      text = server_content["outputTranscription"].get("text", "")
      if text and not _is_internal_heartbeat_text(text):
        self._mark_first_response()
        await self._bus.publish(
            event_bus.Event(
                type=event_bus.EventType.GEMINI_TEXT,
                source=event_bus.EventSource.ASSISTANT,
                data={"text": text},
            )
        )

    # Handle input transcription (user's audio translation to text)
    if "inputTranscription" in server_content:
      text = server_content["inputTranscription"].get("text", "")
      if text:
        await self._bus.publish(
            event_bus.Event(
                type=event_bus.EventType.USER_TRANSCRIPT,
                source=event_bus.EventSource.USER,
                data={"text": text},
            )
        )

    # Handle turn complete.
    if server_content.get("turnComplete"):
      had_tool_call = self._current_turn_has_tool_call
      self._current_turn_has_tool_call = False  # Reset for next turn
      logging.info('Turn complete (had_tool_call=%s)', had_tool_call)

      turn_metrics = self._reset_timing()

      await self._bus.publish(
          event_bus.Event(
              type=event_bus.EventType.TURN_COMPLETE,
              source=event_bus.EventSource.ASSISTANT,
              data={'had_tool_call': had_tool_call},
          )
      )

      if turn_metrics:
        await self._bus.publish(
            event_bus.Event(
                type=event_bus.EventType.TELEMETRY,
                source=event_bus.EventSource.ASSISTANT,
                data=turn_metrics,
            )
        )

    # Handle interruption.
    if server_content.get("interrupted"):
      self._reset_timing()
      # Cancel any active TTS playback immediately so audio stops as
      # soon as the user interrupts.
      await self._cancel_active_tts()
      if self._audio_interrupt_callback:
        try:
          await self._audio_interrupt_callback()
        except Exception as e:  # pylint: disable=broad-except
          logging.error('audio_interrupt_callback failed: %s', e)
      await self._bus.publish(
          event_bus.Event(
              type=event_bus.EventType.INTERRUPTED,
              source=event_bus.EventSource.ASSISTANT,
          )
      )
