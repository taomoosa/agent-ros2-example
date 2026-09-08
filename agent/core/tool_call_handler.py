"""Handler that executes tool calls from the model.

Subscribes to TOOL_CALL events, executes functions via the embodiment,
sends JSON dict responses back to the Gemini session, and publishes
TOOL_RESULT events.

This Lite version uses JSON dicts for the public BidiGenerateContent API
instead of protobuf messages.
"""

import asyncio
from collections.abc import Callable
import datetime
import json
import logging
import re
from typing import Any

import httpx

from core import event_bus


_PICK_CAMERA_STABLE_SECONDS = 3.0


_NEGATED_SIT_PATTERN = re.compile(
    r"(?:\b(?:do\s+not|don't|never)\b[^.!?]{0,40}\bsit\b|\bnot\s+to\s+sit\b)",
    re.IGNORECASE,
)
_SIT_PATTERN = re.compile(r"\b(?:sit(?:\s+down)?|take\s+a\s+seat)\b", re.IGNORECASE)


def user_requested_sit(text: str) -> bool:
  """Return whether a user utterance explicitly asks Spot to sit."""
  return bool(_SIT_PATTERN.search(text)) and not bool(
      _NEGATED_SIT_PATTERN.search(text)
  )


def pick_response_for_model(
    result: Any,
    post_action_frame_sent: bool,
) -> dict[str, Any]:
  """Describe pick completion without claiming visually unverified success."""
  if not isinstance(result, dict):
    result = {"result": result}

  if (
      result.get("executed") is False
      and "required_stable_seconds" in result
  ):
    return {
        "outcome": "blocked_camera_unstable",
        "executed": False,
        "visual_verification_required": False,
        "post_action_frame_sent": post_action_frame_sent,
        "camera_stable_for_seconds": result.get(
            "camera_stable_for_seconds", 0.0
        ),
        "required_stable_seconds": result["required_stable_seconds"],
        "instruction": result.get("retry_instruction", result.get("error", "")),
    }

  failure_reasons = []
  if result.get("error"):
    failure_reasons.append(str(result["error"]))
  if result.get("success") is False:
    failure_reasons.append("backend success diagnostic is false")
  if result.get("holding_item") is False:
    failure_reasons.append("gripper holding-item sensor is false")
  backend_state = str(result.get("state", ""))
  if "FAILED" in backend_state or "NO_SOLUTION" in backend_state:
    failure_reasons.append(f"backend manipulation state is {backend_state}")

  response: dict[str, Any] = {
      "outcome": "failed" if failure_reasons else "unverified",
      "visual_verification_required": not failure_reasons,
      "post_action_frame_sent": post_action_frame_sent,
      "instruction": (
          "Inspect the fresh post-pick camera image. Claim success only if the"
          " requested object is visibly secured by the gripper and has moved"
          " from its original location. Backend completion and gripper sensor"
          " signals are diagnostics, not proof of a successful pick. If the"
          " image is missing or ambiguous, report that the pick is unverified."
      ),
  }
  if backend_state:
    response["backend_state"] = backend_state
  if failure_reasons:
    response["failure_reasons"] = failure_reasons
  if isinstance(result.get("target"), dict):
    response["target"] = result["target"]
  return response


def blocking_tool_names(tools: list[dict[str, Any]] | None) -> frozenset[str]:
  """Return the names of tools whose declarations mark them BLOCKING."""
  names = set()
  for group in tools or []:
    declarations = group.get("functionDeclarations", [group])
    for declaration in declarations:
      if str(declaration.get("behavior", "")).upper() == "BLOCKING":
        name = declaration.get("name")
        if name:
          names.add(name)
  return frozenset(names)


class ToolCallHandler:
  """A handler that executes tool calls and publishes results back to the event bus."""

  def __init__(
      self,
      bus: event_bus.EventBus,
      session_manager: Any,
      embodiment: Any,
      agent_peers: dict[str, str] | None = None,
      peer_name: str = "unknown",
      text_output_callback: Any | None = None,
      clock: Callable[[], datetime.datetime] = datetime.datetime.utcnow,
      enable_send_message_to_user: bool = False,
      blocking_tools: frozenset[str] | set[str] | None = None,
  ):
    self._bus = bus
    self._clock = clock
    self._session_manager = session_manager
    self._embodiment = embodiment
    self._agent_peers = agent_peers or {}
    self._peer_name = peer_name
    # Optional TTS callback so send_message can speak the message
    # out loud before delivering it to the peer agent.
    self._text_output_callback = text_output_callback
    self._enable_send_message_to_user = enable_send_message_to_user
    self._blocking_tools = frozenset(blocking_tools or ())
    self._blocking_execution_count = 0
    self._sit_authorized = False
    self._user_transcript = ""
    # Set while any blocking tool is executing. The heartbeat
    # loop checks this and skips sending prompts to avoid re-triggering
    # the model before the tool result is in context.
    self.tool_executing = asyncio.Event()
    bus.subscribe([event_bus.EventType.TOOL_CALL], self._handle_tool_call)
    bus.subscribe(
        [event_bus.EventType.TEXT_INPUT, event_bus.EventType.USER_TRANSCRIPT],
        self._record_user_instruction,
    )

  def set_session_manager(self, session_manager: Any):
    """Updates the session manager reference."""
    self._session_manager = session_manager

  def set_embodiment(self, embodiment: Any):
    """Updates the embodiment reference."""
    self._embodiment = embodiment


  async def _handle_tool_call(self, event: event_bus.Event) -> None:
    """Handle a TOOL_CALL event by executing all function calls."""
    tool_call = event.data
    function_calls = tool_call.get("functionCalls", [])
    for fc in function_calls:
      try:
        await self._execute_single_tool(fc)
      except Exception:  # pylint: disable=broad-except
        logging.exception('Tool call %s failed', fc.get("name", "unknown"))

  async def _execute_single_tool(self, fc: dict) -> None:
    """Execute a single function call and publish the result."""
    func_name = fc.get("name", "")
    args = fc.get("args", {})
    call_id = str(fc.get("id", ""))
    is_blocking = func_name in self._blocking_tools
    if is_blocking:
      self._blocking_execution_count += 1
      self.tool_executing.set()
    try:
      if func_name in {"detect", "pick", "place"}:
        if func_name == "detect":
          await self._clear_pick_target()
        stability_error = self._camera_stability_error(func_name)
        if stability_error is not None:
          if func_name in {"pick", "place"}:
            robot = getattr(self._embodiment, "robot", None)
            clear_target = getattr(robot, "clear_detected_target", None)
            if clear_target is not None:
              clear_target()
          result = stability_error
        else:
          result = await self._invoke_tool(func_name, args)
          if func_name == "detect":
            await self._publish_detected_target(result)
      else:
        result = await self._invoke_tool(func_name, args)

      # A blocking FunctionResponse resumes model generation. Send the latest
      # visual context first instead of creating a competing heartbeat turn.
      post_action_frame_sent = False
      if is_blocking:
        send_frame = getattr(
            self._session_manager, "send_latest_video_frame", None
        )
        if send_frame is not None:
          post_action_frame_sent = bool(await send_frame())

      # Ensure result is a dict for the JSON response.
      if not isinstance(result, dict):
        result_dict = {'result': result}
      else:
        result_dict = dict(result)
      if func_name == "pick":
        result_dict = pick_response_for_model(result, post_action_frame_sent)

      # Send tool response back to the Gemini session as a JSON dict.
      msg = {
          "toolResponse": {
              "functionResponses": [{
                  "id": call_id,
                  "name": func_name,
                  "response": result_dict
              }]
          }
      }

      if self._session_manager is None:
        raise RuntimeError('Session manager is not initialized')
      await self._session_manager.send_message(msg)

      # Publish result for UI and logging.
      action_data = json.dumps(fc)

      await self._bus.publish(
          event_bus.Event(
              type=event_bus.EventType.TOOL_RESULT,
              source=event_bus.EventSource.USER,
              data={
                  "type": "tool_call",
                  "id": call_id,
                  "name": func_name,
                  "args": args,
                  "result": result,
                  "action_data": action_data,
              },
          )
      )
      logging.info(
          'Tool call %s id=%s completed with result: %s',
          func_name,
          call_id,
          result,
      )
    finally:
      if func_name in {"pick", "place"}:
        await self._clear_pick_target()
      if is_blocking:
        self._blocking_execution_count = max(
            0, self._blocking_execution_count - 1
        )
        if self._blocking_execution_count == 0:
            self.tool_executing.clear()

  async def _record_user_instruction(self, event: event_bus.Event) -> None:
    """Authorize one sit call only after an explicit user request."""
    if event.type == event_bus.EventType.TEXT_INPUT:
      self._user_transcript = str(event.data or "")
    else:
      data = event.data or {}
      text = data.get("text", "") if isinstance(data, dict) else str(data)
      self._user_transcript = f"{self._user_transcript} {text}".strip()
    self._sit_authorized = user_requested_sit(self._user_transcript)

  def _camera_stability_error(self, action_name: str) -> dict[str, Any] | None:
    """Reject pixel-grounded manipulation until the camera is stable."""
    poller = getattr(self._embodiment, "poller", None)
    is_stable_for = getattr(poller, "is_stable_for", None)
    if is_stable_for is None or is_stable_for(_PICK_CAMERA_STABLE_SECONDS):
      return None
    stable_for = float(getattr(poller, "stable_for_seconds", 0.0))
    if action_name in {"pick", "place"}:
      next_action = action_name
    else:
      next_action = "pick or place"
    return {
        "error": (
            f"Camera view is not yet stable; {action_name} was not executed."
        ),
        "executed": False,
        "camera_stable_for_seconds": round(stable_for, 2),
        "required_stable_seconds": _PICK_CAMERA_STABLE_SECONDS,
        "retry_instruction": (
            "Wait until the hand-camera view has remained stable for three"
            " seconds and inspect the latest frame again. Call detect with an"
            f" exact target description; after detection succeeds, call {next_action}"
            " with no arguments and without moving the camera."
        ),
    }

  async def _publish_detected_target(self, result: Any) -> None:
    """Show the backend-selected target without orchestrator pixel input."""
    if not isinstance(result, dict):
      return
    target = result.get("target")
    if not isinstance(target, dict):
      return
    try:
      x = float(target["normalized_x"])
      y = float(target["normalized_y"])
    except (KeyError, TypeError, ValueError):
      return
    if not 0 <= x <= 1000 or not 0 <= y <= 1000:
      return
    await self._bus.publish(
        event_bus.Event(
            type=event_bus.EventType.TOOL_RESULT,
            source=event_bus.EventSource.ASSISTANT,
            data={
                "type": "draw_points",
                "points": [{"x": x, "y": y, "label": "detected target"}],
            },
        )
    )

  async def _clear_pick_target(self) -> None:
    """Remove the pick marker after the blocking pick call finishes."""
    await self._bus.publish(
        event_bus.Event(
            type=event_bus.EventType.TOOL_RESULT,
            source=event_bus.EventSource.ASSISTANT,
            data={"type": "clear_overlay"},
        )
    )

  async def _invoke_tool(self, func_name: str, args: dict[str, Any]) -> Any:
    try:
      if func_name == "sit":
        if not self._sit_authorized:
          return {
              "executed": False,
              "error": "Sit rejected: the user did not explicitly ask Spot to sit.",
          }
        self._sit_authorized = False
      if func_name == "send_message":
        return await self._execute_send_message(args)
      return await self._embodiment.execute_action(func_name, **args)
    except Exception as exc:  # pylint: disable=broad-except
      logging.error('Tool execution error for %s: %s', func_name, exc)
      return f'Error: {exc}'

  async def _execute_send_message(self, args: dict[str, Any]) -> str:
    """Send a message to another robot agent via HTTP POST.

    Waits for any in-flight TTS/audio playback to finish before sending,
    so the receiving agent doesn't start speaking over the sender.
    """
    target = args.get("target", "")
    message = args.get("message", "")

    if not target:
      return "Error: 'target' is required"
    if not message:
      return "Error: 'message' is required"

    if target == "user":
      if not self._enable_send_message_to_user:
        return (
            "Error: sending messages to user is not enabled."
            " Use inline text instead."
        )
      if self._text_output_callback:

        async def _run_tts():
          try:
            logging.info(
                "send_message to user: speaking message via TTS (non-blocking)"
            )
            await self._text_output_callback(message)
          except Exception as e:  # pylint: disable=broad-except
            logging.exception("TTS for send_message to user failed: %s", e)

        asyncio.create_task(_run_tts())
      return f"Message delivered to user: {message}"

    target_url = self._agent_peers.get(target)
    if not target_url:
      available = list(self._agent_peers.keys())
      return f"Error: unknown target '{target}'. Available peers: {available}"

    # Speak the message out loud via TTS before sending to peer,
    # so the user hears what is being communicated and the message
    # arrives after playback finishes.
    if self._text_output_callback:
      try:
        logging.info(
            "send_message to %s: speaking message via TTS first", target
        )
        await self._text_output_callback(message)
      except Exception as e:  # pylint: disable=broad-except
        logging.error("TTS for send_message failed: %s", e)

    try:
      async with httpx.AsyncClient() as client:
        resp = await client.post(
            f"{target_url}/api/send",
            json={"text": message, "source": self._peer_name},
            timeout=10.0,
        )
        if resp.status_code == 200:
          return f"Message delivered to {target}: {message}"
        else:
          return (
              f"Error sending to {target}: HTTP {resp.status_code} {resp.text}"
          )
    except httpx.ConnectError:
      return f"Error: could not connect to {target} at {target_url}"
    except Exception as e:  # pylint: disable=broad-except
      return f"Error sending to {target}: {e}"
