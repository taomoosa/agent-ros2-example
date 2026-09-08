"""Handles audio accumulation, encoding, and playback.

Subscribes to AUDIO_CHUNK events to accumulate raw PCM audio data,
and to TURN_COMPLETE events to encode the accumulated audio as a WAV
file and publish it as an AUDIO_RESPONSE event.
"""

import asyncio
import base64
import io
import logging
from typing import Awaitable, Callable, Optional
import wave

from core import event_bus

logger = logging.getLogger(__name__)


def _encode_wav(pcm_data: bytes) -> str:
  """Encodes raw PCM audio bytes as a base64 WAV data URI."""
  wav_io = io.BytesIO()
  with wave.open(wav_io, 'wb') as wf:
    wf.setnchannels(1)  # Mono
    wf.setsampwidth(2)  # 16-bit
    wf.setframerate(24000)  # 24kHz
    wf.writeframes(pcm_data)

  wav_bytes = wav_io.getvalue()
  audio_b64 = base64.b64encode(wav_bytes).decode('utf-8')
  return f'data:audio/wav;base64,{audio_b64}'


class AudioResponseHandler:
  """A handler that accumulates audio chunks and encodes them as WAV on turn complete.

  This handler is responsible for:
  1. Calling the audio_output_callback for real-time playback of each chunk.
  2. Accumulating raw PCM audio data across an entire model turn.
  3. On turn complete, encoding the accumulated audio as a base64 WAV data
     URI and publishing an AUDIO_RESPONSE event for UI display.
  """

  def __init__(
      self,
      bus: event_bus.EventBus,
      audio_output_callback: Optional[
          Callable[[bytes], Awaitable[None]]
      ] = None,
  ):
    self._bus = bus
    self._audio_output_callback = audio_output_callback
    self._buffer = b''
    self._lock = asyncio.Lock()
    bus.subscribe([event_bus.EventType.AUDIO_CHUNK], self._handle_chunk)
    bus.subscribe(
        [event_bus.EventType.TURN_COMPLETE], self._handle_turn_complete
    )
    bus.subscribe([event_bus.EventType.INTERRUPTED], self._handle_interrupted)

  async def _handle_chunk(self, event: event_bus.Event) -> None:
    """Accumulate an audio chunk and forward to playback callback."""
    audio_data = event.data
    async with self._lock:
      self._buffer += audio_data
    if self._audio_output_callback:
      await self._audio_output_callback(audio_data)

  async def _handle_turn_complete(self, event: event_bus.Event) -> None:
    """Encode accumulated audio as WAV and publish AUDIO_RESPONSE."""
    async with self._lock:
      if not self._buffer:
        return
      pcm_data = self._buffer
      self._buffer = b''

    audio_data_uri = await asyncio.to_thread(_encode_wav, pcm_data)

    await self._bus.publish(
        event_bus.Event(
            type=event_bus.EventType.AUDIO_RESPONSE,
            source=event_bus.EventSource.ASSISTANT,
            data={'audio_data': audio_data_uri},
        )
    )

  async def _handle_interrupted(self, event: event_bus.Event) -> None:
    """Clears accumulated audio buffer when the model is interrupted."""
    async with self._lock:
      self._buffer = b''
