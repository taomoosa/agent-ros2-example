"""Gemini Live API client for bidirectional streaming via WebSockets.

This module provides a client that connects to the public Gemini Live API
(BidiGenerateContent) over WebSockets, enabling real-time voice and vision
interactions without requiring internal Google infrastructure (pywraprpc/LOAS).

All messages are sent and received as plain Python dicts (JSON), with no
protobuf dependency.

Usage:
  client = GeminiLiveApiClient(api_key="YOUR_API_KEY")
  stream = client.create_stream()

WebSocket endpoint:
  wss://generativelanguage.googleapis.com/ws/google.ai.generativelanguage.v1beta
      .GenerativeService.BidiGenerateContent?key=API_KEY
"""

import logging
import threading
from typing import Any, Callable

import websocket

logger = logging.getLogger(__name__)

_LIVE_API_WSS_URL = (
    "wss://generativelanguage.googleapis.com/ws/"
    "google.ai.generativelanguage.v1alpha"
    ".GenerativeService.BidiGenerateContent"
)


# pylint: disable=invalid-name


class GeminiLiveApiStream:
  """Wrapper around a WebSocket connection to the Gemini Live API.

  Provides the streaming interface (Start, Send, HalfClose, GetStatus) so
  that the SessionManager can use it transparently.
  """

  def __init__(self, ws):
    self._ws = ws
    self._on_message = None
    self._on_done = None
    self._read_thread = None

  def Start(
      self, on_message: Callable[[dict], None],
      on_done: Callable[[], None],
  ) -> None:
    """Start reading messages from the WebSocket in a background thread."""
    logger.debug("GeminiLiveApiStream.Start() called")
    self._on_message = on_message
    self._on_done = on_done
    self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
    self._read_thread.start()
    logger.debug("GeminiLiveApiStream read thread spawned and started")

  def _read_loop(self) -> None:
    """Reads messages from the WebSocket and dispatches to on_message."""
    logger.debug("GeminiLiveApiStream._read_loop() thread started")
    try:
      while True:
        try:
          opcode, data = self._ws.recv_data()
        except websocket.WebSocketTimeoutException:
          continue  # Bounded socket I/O need not disconnect an idle live session.
        if opcode == websocket.ABNF.OPCODE_CLOSE:
          import struct  # pylint: disable=g-import-not-at-top
          code = 1000
          reason = ""
          if len(data) >= 2:
            code = struct.unpack("!H", data[0:2])[0]
            reason = data[2:].decode("utf-8", errors="replace")
          logger.info(
              "WebSocket closed by server. code=%s, reason=%s", code, reason
          )
          break

        if opcode not in (
            websocket.ABNF.OPCODE_TEXT,
            websocket.ABNF.OPCODE_BINARY,
        ):
          continue

        raw = data

        # If it is bytes, it might still be a UTF-8 JSON string.
        if isinstance(raw, bytes):
          try:
            raw = raw.decode("utf-8")
          except UnicodeDecodeError:
            logger.warning("Received binary data but expected JSON")

        if isinstance(raw, str):
          import json
          try:
            parsed_msg = json.loads(raw)
            logger.info("Received message from Gemini Live API: %s", parsed_msg)
          except json.JSONDecodeError as e:
            logger.warning("Failed to decode JSON: %s", e)
            continue
        else:
          logger.warning("Unexpected message type from WebSocket: %s", type(raw))
          continue

        if self._on_message:
          self._on_message(parsed_msg)
    except Exception as e:  # pylint: disable=broad-except
      logger.warning("Error in GeminiLiveApiStream read loop: %s", e)
    finally:
      try:
        code = self._ws.close_status_code
        reason = self._ws.close_status_reason
        if code or reason:
          logger.info(
              "WebSocket closed with code=%s, reason=%s", code, reason
          )
      except Exception as e:  # pylint: disable=broad-except
        logger.warning("Failed to read close status: %s", e)
      logger.debug("GeminiLiveApiStream read loop exiting, calling on_done")
      if self._on_done:
        self._on_done()

  def Send(self, msg: Any) -> None:
    """Send a client message (must be a JSON-serializable dict)."""
    try:
      import json  # pylint: disable=g-import-not-at-top
      json_str = json.dumps(msg)
      self._ws.send(json_str)
    except Exception as e:  # pylint: disable=broad-except
      logger.warning("GeminiLiveApiStream.Send failed: %s", e)
      raise e
    return None

  def HalfClose(self) -> None:
    """Signal that no more messages will be sent."""
    try:
      self._ws.close()
    except Exception as e:  # pylint: disable=broad-except
      logger.warning("GeminiLiveApiStream.HalfClose error: %s", e)

  def GetStatus(self) -> Any:
    return None

  def Shutdown(self) -> None:
    """Force-close the WebSocket connection."""
    try:
      self._ws.close()
    except Exception as e:  # pylint: disable=broad-except
      logger.warning("GeminiLiveApiStream.Shutdown error: %s", e)


class GeminiLiveApiClient:
  """Client for the public Gemini Live API (BidiGenerateContent over WSS).

  This client uses the websocket-client library to establish a persistent
  WebSocket connection to the Gemini Live API endpoint.
  """

  def __init__(self, api_key: str | None = None):
    if not api_key:
      raise ValueError("api_key is required for the Gemini Live API endpoint.")
    self._api_key = api_key
    self._url = f"{_LIVE_API_WSS_URL}?key={api_key}"
    logger.info(
        "GeminiLiveApiClient initialized -> %s",
        _LIVE_API_WSS_URL,
    )

  def create_stream(self, timeout=None) -> GeminiLiveApiStream:
    """Create a new WebSocket connection and return a stream wrapper."""
    ws = websocket.create_connection(
        self._url,
        header={"Content-Type": "application/json"}, timeout=timeout,
    )
    logger.info("WebSocket connection established to Gemini Live API")
    return GeminiLiveApiStream(ws)
