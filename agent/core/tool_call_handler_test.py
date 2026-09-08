"""Tests for declaration-driven blocking tool handling."""

from __future__ import annotations

import asyncio
import unittest
from unittest import mock

from core import tool_call_handler


class FakeBus:

  def __init__(self):
    self.events = []

  def subscribe(self, _event_types, _handler):
    return None

  async def publish(self, event):
    self.events.append(event)


class FakeSessionManager:

  def __init__(self):
    self.messages = []
    self.frames_sent = 0

  async def send_message(self, message):
    self.messages.append(message)

  async def send_latest_video_frame(self):
    self.frames_sent += 1
    return True


class FakeEmbodiment:

  def __init__(self):
    self.calls = []
    self.started = asyncio.Event()
    self.release = asyncio.Event()
    self.wait_for_release = False

  async def execute_action(self, action_name, **args):
    self.calls.append((action_name, args))
    self.started.set()
    if self.wait_for_release:
      await self.release.wait()
    return {"state": "done", "call_count": len(self.calls)}


class FakeUnstablePoller:

  stable_for_seconds = 1.25

  def is_stable_for(self, seconds):
    return False


class ToolCallHandlerTest(unittest.IsolatedAsyncioTestCase):

  def setUp(self):
    self.bus = FakeBus()
    self.session = FakeSessionManager()
    self.embodiment = FakeEmbodiment()
    self.handler = tool_call_handler.ToolCallHandler(
        self.bus,
        self.session,
        self.embodiment,
        blocking_tools={"detect", "pick", "navigate"},
    )

  async def test_marks_blocking_tool_active_while_it_executes(self):
    self.embodiment.wait_for_release = True
    execution = asyncio.create_task(self.handler._execute_single_tool({
        "id": "call-1",
        "name": "pick",
        "args": {},
    }))
    await self.embodiment.started.wait()
    self.assertTrue(self.handler.tool_executing.is_set())

    self.embodiment.release.set()
    await execution

    self.assertEqual(1, len(self.embodiment.calls))
    self.assertEqual(1, self.session.frames_sent)
    self.assertFalse(self.handler.tool_executing.is_set())

    event_types = [event.type for event in self.bus.events]
    self.assertNotIn(
        tool_call_handler.event_bus.EventType.HEARTBEAT_TRIGGER,
        event_types,
    )

  async def test_executes_repeated_blocking_calls(self):
    await self.handler._execute_single_tool({
        "id": "call-1",
        "name": "navigate",
        "args": {"name": "home"},
    })
    await self.handler._execute_single_tool({
        "id": "call-2",
        "name": "navigate",
        "args": {"name": "home"},
    })

    self.assertEqual(2, len(self.embodiment.calls))
    self.assertEqual(2, self.session.frames_sent)
    for message in self.session.messages:
      response = message["toolResponse"]["functionResponses"][0]["response"]
      self.assertNotIn("deduplicated", response)

  async def test_executes_batched_blocking_calls_in_order(self):
    self.embodiment.wait_for_release = True
    event = tool_call_handler.event_bus.Event(
        type=tool_call_handler.event_bus.EventType.TOOL_CALL,
        source=tool_call_handler.event_bus.EventSource.ASSISTANT,
        data={
            "functionCalls": [
                {"id": "call-1", "name": "pick", "args": {}},
                {"id": "call-2", "name": "pick", "args": {}},
            ]
        },
    )

    execution = asyncio.create_task(self.handler._handle_tool_call(event))
    await self.embodiment.started.wait()
    await asyncio.sleep(0)
    self.assertEqual(1, len(self.embodiment.calls))

    self.embodiment.release.set()
    await execution
    self.assertEqual(
        [("pick", {}), ("pick", {})],
        self.embodiment.calls,
    )

  async def test_executes_repeated_non_blocking_calls(self):
    call = {"name": "get_battery", "args": {}}
    await self.handler._execute_single_tool({"id": "call-1", **call})
    await self.handler._execute_single_tool({"id": "call-2", **call})

    self.assertEqual(2, len(self.embodiment.calls))
    self.assertEqual(0, self.session.frames_sent)

  async def test_detect_publishes_backend_target_overlay(self):
    self.embodiment.execute_action = mock.AsyncMock(return_value={
        "detected": True,
        "target": {"normalized_x": 240, "normalized_y": 610},
    })

    await self.handler._execute_single_tool({
        "id": "call-1",
        "name": "detect",
        "args": {"instruction": "red cube"},
    })

    self.assertEqual("clear_overlay", self.bus.events[0].data["type"])
    self.assertEqual("draw_points", self.bus.events[1].data["type"])
    self.assertEqual(
        [{"x": 240.0, "y": 610.0, "label": "detected target"}],
        self.bus.events[1].data["points"],
    )
    self.assertEqual("tool_call", self.bus.events[-1].data["type"])

  async def test_detect_is_blocked_until_camera_is_stable_for_three_seconds(self):
    self.embodiment.poller = FakeUnstablePoller()

    await self.handler._execute_single_tool({
        "id": "call-1",
        "name": "detect",
        "args": {"instruction": "red cube"},
    })

    self.assertEqual([], self.embodiment.calls)
    self.assertFalse(any(
        event.data.get("type") == "draw_points"
        for event in self.bus.events
    ))
    response = self.session.messages[-1]["toolResponse"]["functionResponses"][0][
        "response"
    ]
    self.assertFalse(response["executed"])
    self.assertEqual(3.0, response["required_stable_seconds"])
    self.assertEqual(1.25, response["camera_stable_for_seconds"])

  async def test_pick_response_requires_visual_verification(self):
    self.embodiment.execute_action = mock.AsyncMock(return_value={
        "state": "MANIP_STATE_GRASP_SUCCEEDED",
        "success": True,
        "holding_item": True,
    })

    await self.handler._execute_single_tool({
        "id": "call-1",
        "name": "pick",
        "args": {},
    })

    response = self.session.messages[-1]["toolResponse"]["functionResponses"][0][
        "response"
    ]
    self.assertEqual("unverified", response["outcome"])
    self.assertTrue(response["visual_verification_required"])
    self.assertTrue(response["post_action_frame_sent"])
    self.assertNotIn("success", response)
    self.assertNotIn("holding_item", response)

  async def test_pick_response_preserves_explicit_backend_failure(self):
    self.embodiment.execute_action = mock.AsyncMock(return_value={
        "state": "MANIP_STATE_GRASP_FAILED",
        "success": False,
        "holding_item": False,
    })

    await self.handler._execute_single_tool({
        "id": "call-1",
        "name": "pick",
        "args": {},
    })

    response = self.session.messages[-1]["toolResponse"]["functionResponses"][0][
        "response"
    ]
    self.assertEqual("failed", response["outcome"])
    self.assertFalse(response["visual_verification_required"])
    self.assertTrue(response["failure_reasons"])

  async def test_sit_is_rejected_without_explicit_user_request(self):
    result = await self.handler._invoke_tool("sit", {})

    self.assertFalse(result["executed"])
    self.assertEqual([], self.embodiment.calls)

  async def test_explicit_user_request_authorizes_one_sit(self):
    await self.handler._record_user_instruction(
        tool_call_handler.event_bus.Event(
            type=tool_call_handler.event_bus.EventType.TEXT_INPUT,
            source=tool_call_handler.event_bus.EventSource.USER,
            data="Go home and then sit down.",
        )
    )

    first = await self.handler._invoke_tool("sit", {})
    second = await self.handler._invoke_tool("sit", {})

    self.assertEqual("done", first["state"])
    self.assertFalse(second["executed"])
    self.assertEqual([("sit", {})], self.embodiment.calls)

  def test_negated_sit_request_is_not_authorized(self):
    self.assertFalse(tool_call_handler.user_requested_sit("Do not sit."))
    self.assertFalse(tool_call_handler.user_requested_sit("Never ask Spot to sit"))
    self.assertTrue(tool_call_handler.user_requested_sit("Please sit down"))

  def test_extracts_all_declared_blocking_tools(self):
    tools = [{
        "functionDeclarations": [
            {"name": "pick", "behavior": "BLOCKING"},
            {"name": "get_battery"},
            {"name": "navigate", "behavior": "blocking"},
            {"name": "stop", "behavior": "NON_BLOCKING"},
        ]
    }]

    names = tool_call_handler.blocking_tool_names(tools)

    self.assertIn("pick", names)
    self.assertIn("navigate", names)
    self.assertNotIn("stop", names)
    self.assertNotIn("get_battery", names)


if __name__ == "__main__":
  unittest.main()
