"""Tests for synchronized observation input."""

import asyncio
import unittest

from core import event_bus
from core import observation


class FakeBus:

  def __init__(self):
    self.events = []

  async def publish(self, event):
    self.events.append(event)


class FakeSessionManager:

  def __init__(self):
    self.operations = []
    self.message_sent = asyncio.Event()

  async def send_text_with_fresh_video(self, text):
    self.operations.append("frame")
    self.operations.append(("text", text))
    self.message_sent.set()


class ObservationTest(unittest.IsolatedAsyncioTestCase):

  async def test_sends_fresh_frame_before_user_text(self):
    bus = FakeBus()
    session = FakeSessionManager()
    component = observation.Observation.__new__(observation.Observation)
    component.bus = bus
    component.session_manager = session
    component.text_queue = asyncio.Queue()

    task = asyncio.create_task(component._send_text())
    await component.text_queue.put("what do you see?")
    await asyncio.wait_for(session.message_sent.wait(), timeout=1.0)
    task.cancel()
    await task

    self.assertEqual("frame", session.operations[0])
    self.assertEqual("text", session.operations[1][0])
    self.assertEqual("what do you see?", session.operations[1][1])
    self.assertEqual(event_bus.EventType.TEXT_INPUT, bus.events[0].type)


if __name__ == "__main__":
  unittest.main()
