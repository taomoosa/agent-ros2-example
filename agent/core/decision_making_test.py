"""Tests for model response routing."""

import unittest

from core import decision_making
from core import event_bus


class FakeBus:

  def __init__(self):
    self.events = []

  def subscribe(self, _event_types, _handler):
    return None

  async def publish(self, event):
    self.events.append(event)


class DecisionMakingTest(unittest.IsolatedAsyncioTestCase):

  async def test_hides_heartbeat_output_transcription(self):
    bus = FakeBus()
    router = decision_making.DecisionMaking(bus)

    await router._handle_server_content({
        "outputTranscription": {
            "text": "[HEARTBEAT] inspect the scene and call ack"
        }
    })

    self.assertEqual([], bus.events)

  async def test_routes_normal_output_transcription(self):
    bus = FakeBus()
    router = decision_making.DecisionMaking(bus)

    await router._handle_server_content({
        "outputTranscription": {"text": "I can see the desk."}
    })

    self.assertEqual(1, len(bus.events))
    self.assertEqual(event_bus.EventType.GEMINI_TEXT, bus.events[0].type)
    self.assertEqual({"text": "I can see the desk."}, bus.events[0].data)


if __name__ == "__main__":
  unittest.main()
