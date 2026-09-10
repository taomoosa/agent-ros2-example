"""ROS2 contract and application tests without ROS2, Gemini or network access."""

import asyncio
import dataclasses
import io
import json
from pathlib import Path
import unittest
from embodiment.ros2.test_requests import pose_move, named_move
from unittest import mock

import httpx
from PIL import Image

from embodiment.ros2.camera_poller import Ros2CameraPoller
from embodiment.ros2.config import Arm, Camera, RobotConfig
from embodiment.ros2.robot_client import Ros2RobotClient
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from run_ros2 import run_application

CONFIGS = Path(__file__).resolve().parents[2] / "configs"


def jpeg(color="red"):
  buffer = io.BytesIO()
  Image.new("RGB", (32, 24), color).save(buffer, "JPEG")
  return buffer.getvalue()


class MockRobot:
  def __init__(self):
    self.requests = []
    self.history = []
    self.camera_failure = False
    self.move_error = None

  def __call__(self, request):
    self.requests.append(request)
    self.history.append(("http", request.method, request.url.path))
    if request.url.path.endswith("/image"):
      if self.camera_failure:
        return httpx.Response(503, json={"error": "camera disconnected"})
      color = "blue" if "wrist" in request.url.path else "red"
      return httpx.Response(200, content=jpeg(color), headers={"Content-Type": "image/jpeg"})
    if request.url.path.endswith("/move") and self.move_error:
      raise self.move_error("motion timed out", request=request)
    if request.url.path == "/v1/state":
      return httpx.Response(200, json={"arms": [{"id": "left"}, {"id": "right"}]})
    return httpx.Response(200, json={"success": True})


class MockGeminiStream:
  """Drive real SessionManager through setup, input, tools and completion."""
  def __init__(self, history, calls):
    self.history = history
    self.calls = iter(calls)
    self.messages = []
    self.closed = False

  def Start(self, on_message, on_done):
    self.on_message = on_message

  def Send(self, message):
    self.messages.append(message)
    self.history.append(("gemini", message))
    if "setup" in message:
      self.on_message({"setupComplete": {}})
    elif "clientContent" in message or "toolResponse" in message:
      call = next(self.calls, None)
      if call:
        self.on_message({"toolCall": {"functionCalls": [call]}})

  def Shutdown(self):
    self.closed = True


class ConfigTest(unittest.TestCase):
  def test_examples_and_frame_topology(self):
    for filename, count in [("single_arm.json", 1), ("dual_arm.json", 2)]:
      with self.subTest(filename=filename):
        config = RobotConfig.load(CONFIGS / filename)
        self.assertEqual(count, len(config.arms))
        self.assertIn("world", config.frame_ids)
        for camera in config.cameras:
          self.assertIn(camera.optical_frame, config.frame_ids)

  def test_rejects_invalid_topologies_and_url(self):
    config = RobotConfig.load(CONFIGS / "dual_arm.json")
    invalid = [
        {"arms": ()}, {"arms": config.arms + (Arm("third", "b", "f"),)},
        {"arms": (config.arms[0], config.arms[0])}, {"cameras": ()},
        {"cameras": (config.cameras[0], config.cameras[0])},
        {"robot_url": "file:///tmp/robot"}, {"robot_url": "http://robot/api"},
        {"cameras": (Camera("wrist", "optical", "flange", "wrong", "left"),)},
        {"cameras": (Camera("wrist", "optical", "flange", "flange", "missing"),)},
        {"cameras": (Camera("overhead", "optical", "world", "world", "left"),)},
    ]
    for overrides in invalid:
      with self.subTest(overrides=overrides), self.assertRaises(ValueError):
        dataclasses.replace(config, **overrides)
    with self.assertRaises(ValueError):
      Arm("../left", "base", "flange")


class ClientTest(unittest.IsolatedAsyncioTestCase):
  async def asyncSetUp(self):
    self.config = RobotConfig.load(CONFIGS / "dual_arm.json")
    self.backend = MockRobot()
    self.client = Ros2RobotClient(self.config, transport=httpx.MockTransport(self.backend))
    self.addAsyncCleanup(self.client.close)

  async def test_arm_routing_payloads_and_stop(self):
    for arm_id in ["left", "right"]:
      result = await self.client.move(**pose_move(arm_id=arm_id,frame_id="world",position=[0.1, 0.2, 0.3],orientation=[0, 0, 0, 1]))
      self.assertTrue(result["success"])
      request = self.backend.requests[-1]
      self.assertEqual("/v1/move", request.url.path)
      self.assertEqual(pose_move(arm_id, "world", [0.1,0.2,0.3], [0,0,0,1]), json.loads(request.content))
      await self.client.gripper(**dict(arm_ids=[arm_id], opening=0.4))
      self.assertEqual("/v1/gripper", self.backend.requests[-1].url.path)
    await self.client.stop(arm_ids=["left"])
    self.assertEqual({"arm_ids": ["left"]}, json.loads(self.backend.requests[-1].content))
    await self.client.stop()
    self.assertEqual({"all_arms": True}, json.loads(self.backend.requests[-1].content))

  async def test_invalid_motion_never_reaches_server(self):
    good = {"arm_id": "left", "frame_id": "world", "position": [0, 0, 0],
            "orientation": [0, 0, 0, 1]}
    for overrides in [{"arm_id": "missing"}, {"frame_id": "unknown"},
                      {"position": [1, 2]}, {"position": [float("nan"), 0, 0]},
                      {"orientation": [0, 0, 0, 0]}, {"duration": -1},
                      {"duration": float("inf")}, {"duration": True}]:
      with self.subTest(overrides=overrides), self.assertRaises(ValueError):
        await self.client.move(**pose_move(**good | overrides))
    for opening in [-0.1, 1.1, float("nan"), True]:
      with self.assertRaises(ValueError):
        await self.client.gripper(**dict(arm_ids=["left"], opening=opening))
    self.assertEqual([], self.backend.requests)

  async def test_timeout_is_not_retried(self):
    self.backend.move_error = httpx.ReadTimeout
    with self.assertRaises(httpx.ReadTimeout):
      await self.client.move(**pose_move(arm_id="left",frame_id="world",position=[0, 0, 0],orientation=[0, 0, 0, 1]))
    self.assertEqual(1, len(self.backend.requests))

  async def test_asynchronous_acceptance_is_not_completion(self):
    client = Ros2RobotClient(self.config, transport=httpx.MockTransport(
        lambda request: httpx.Response(202, json={"success": True})))
    try:
      result = await client.gripper(**dict(arm_ids=["left"], opening=0.5))
      self.assertFalse(result["success"])
      self.assertEqual("unknown", result["outcome"])
    finally:
      await client.close()

  async def test_camera_content_and_errors(self):
    self.assertTrue((await self.client.get_camera_snapshot("left_wrist")).startswith(b"\xff\xd8"))
    with self.assertRaises(ValueError):
      await self.client.get_camera_snapshot("unknown")
    self.backend.camera_failure = True
    with self.assertRaises(httpx.HTTPStatusError):
      await self.client.get_camera_snapshot("left_wrist")
    for content, headers in [(b"not jpeg", {"Content-Type": "image/jpeg"}),
                             (jpeg(), {"Content-Type": "text/html"})]:
      client = Ros2RobotClient(self.config, transport=httpx.MockTransport(
          lambda request: httpx.Response(200, content=content, headers=headers)))
      try:
        with self.assertRaises((ValueError, OSError)):
          await client.get_camera_snapshot("left_wrist")
      finally:
        await client.close()

  async def test_camera_grid_and_no_stale_fallback(self):
    poller = Ros2CameraPoller(self.client, asyncio.Queue(maxsize=1))
    frame = await poller.wait_for_next_frame()
    image = Image.open(io.BytesIO(frame))
    self.assertEqual((768, 768), image.size)
    self.assertGreater(image.getpixel((200, 200))[0], 200)
    self.assertGreater(image.getpixel((600, 200))[2], 200)
    self.backend.camera_failure = True
    self.assertEqual(b"", await poller.wait_for_next_frame())
    self.assertEqual(b"", poller.latest_frame)
    self.backend.camera_failure = False
    self.assertTrue(await poller.wait_for_next_frame())
    poller.stop()
    self.assertEqual(b"", await poller.wait_for_next_frame())

  async def test_fresh_capture_does_not_reuse_inflight_capture(self):
    started = asyncio.Event()
    release = asyncio.Event()
    calls = 0

    async def snapshot(camera_id):
      nonlocal calls
      calls += 1
      started.set()
      await release.wait()
      return jpeg()

    self.client.get_camera_snapshot = snapshot
    poller = Ros2CameraPoller(self.client, asyncio.Queue())
    first = asyncio.create_task(poller.wait_for_next_frame())
    await started.wait()
    second = asyncio.create_task(poller.wait_for_next_frame())
    release.set()
    await asyncio.gather(first, second)
    self.assertEqual(2 * len(self.config.cameras), calls)


class ApplicationTest(unittest.IsolatedAsyncioTestCase):
  async def test_full_gemini_http_loop_and_cleanup(self):
    for filename, arm in [("single_arm.json", "arm"), ("dual_arm.json", "right")]:
      with self.subTest(filename=filename):
        config = RobotConfig.load(CONFIGS / filename)
        backend = MockRobot()
        calls = [
            {"id": "move-1", "name": 'move', "args": named_move(arm)},
            {"id": "grip-1", "name": 'gripper', "args": dict(arm_ids=[arm],opening=0)},
            {"id": "state-final", "name": "get_robot_state", "args": {}},
            {"id": "done-1", "name": "finish_task", "args": {"success": True, "summary": "Done"}},
        ]
        stream = MockGeminiStream(backend.history, calls)
        with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
          client.return_value.create_stream.return_value = stream
          result = await run_application(config, "Move and grip", model="mock-live",
              api_key="mock-key", timeout=10, transport=httpx.MockTransport(backend))
        self.assertTrue(result["success"])
        self.assertTrue(stream.closed)
        setup = stream.messages[0]["setup"]
        self.assertEqual("models/mock-live", setup["model"])
        self.assertIn("outputAudioTranscription", setup)
        self.assertIn("flange", setup["systemInstruction"]["parts"][0]["text"])
        declarations = {item["name"]: item for item in setup["tools"][0]["functionDeclarations"]}
        self.assertEqual([a.id for a in config.arms],
            declarations["move"]["parameters"]["properties"]["arm_ids"]["items"]["enum"])
        responses = [m["toolResponse"]["functionResponses"][0] for m in stream.messages if "toolResponse" in m]
        self.assertEqual(["move-1", "grip-1", "state-final", "done-1"], [r["id"] for r in responses])
        move_index = backend.history.index(("http", "POST", "/v1/move"))
        response_index = next(i for i, entry in enumerate(backend.history)
            if entry[0] == "gemini" and "toolResponse" in entry[1])
        between = backend.history[move_index + 1:response_index]
        self.assertTrue(any(e[0] == "http" and e[2].endswith("/image") for e in between))
        self.assertTrue(any(e[0] == "gemini" and "realtimeInput" in e[1] for e in between))
        self.assertFalse(any(request.url.path == "/v1/stop" for request in backend.requests))

  async def test_http_failure_is_returned_to_gemini(self):
    config = RobotConfig.load(CONFIGS / "single_arm.json")
    backend = MockRobot()
    backend.move_error = httpx.ReadTimeout
    stream = MockGeminiStream(backend.history, [
        {"id": "1", "name": 'move', "args": named_move("arm")},
        {"id": "2", "name": "finish_task", "args": {"success": False, "summary": "Motion outcome unknown"}},
    ])
    with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
      client.return_value.create_stream.return_value = stream
      result = await run_application(config, "Move", model="mock", api_key="key",
          transport=httpx.MockTransport(backend), timeout=10)
    self.assertFalse(result["success"])
    response = next(m["toolResponse"]["functionResponses"][0]["response"]
                    for m in stream.messages if "toolResponse" in m)
    self.assertEqual("unknown", response["outcome"])
    self.assertEqual(1, sum(r.url.path.endswith("/move") for r in backend.requests))

  async def test_application_timeout_stops_arms_and_closes_stream(self):
    config = RobotConfig.load(CONFIGS / "single_arm.json")
    backend = MockRobot()
    stream = MockGeminiStream(backend.history, [])
    with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
      client.return_value.create_stream.return_value = stream
      with self.assertRaises(asyncio.TimeoutError):
        await run_application(config, "Inspect", model="mock", api_key="key",
            transport=httpx.MockTransport(backend), timeout=0.5)
    self.assertTrue(stream.closed)
    self.assertEqual("/v1/stop", backend.requests[-1].url.path)

  async def test_setup_failure_closes_stream_and_stops_arms(self):
    config = RobotConfig.load(CONFIGS / "single_arm.json")
    backend = MockRobot()
    stream = MockGeminiStream(backend.history, [])
    stream.Send = mock.Mock(side_effect=RuntimeError("setup failed"))
    with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
      client.return_value.create_stream.return_value = stream
      with self.assertRaisesRegex(RuntimeError, "setup failed"):
        await run_application(config, "Inspect", model="mock", api_key="key",
            transport=httpx.MockTransport(backend))
    self.assertTrue(stream.closed)
    self.assertEqual("/v1/stop", backend.requests[-1].url.path)

  async def test_unavailable_camera_prevents_gemini_start(self):
    config = RobotConfig.load(CONFIGS / "single_arm.json")
    backend = MockRobot()
    backend.camera_failure = True
    with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
      with self.assertRaisesRegex(RuntimeError, "All configured cameras"):
        await run_application(config, "Inspect", model="mock", api_key="key",
            transport=httpx.MockTransport(backend))
      client.assert_not_called()
    self.assertFalse(any(request.url.path.endswith("/move") for request in backend.requests))

  async def test_motion_serialization_does_not_block_stop(self):
    config = RobotConfig.load(CONFIGS / "dual_arm.json")
    first_started = asyncio.Event()
    release = asyncio.Event()
    moves = []
    stops = []

    async def handler(request):
      if request.url.path.endswith("/move"):
        moves.append(request.url.path)
        first_started.set()
        await release.wait()
      if request.url.path == "/v1/stop":
        stops.append(request.url.path)
      return httpx.Response(200, json={"success": True})

    embodiment = Ros2Embodiment(config, transport=httpx.MockTransport(handler))
    self.addAsyncCleanup(embodiment.close)
    args = {"frame_id": "world", "position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
    first = asyncio.create_task(embodiment.execute_action('move', **named_move("left")))
    await first_started.wait()
    second = asyncio.create_task(embodiment.execute_action('move', **named_move("right")))
    try:
      await asyncio.wait_for(embodiment.execute_action("stop"), timeout=0.5)
      self.assertEqual(["/v1/stop"], stops)
      self.assertEqual(["/v1/move"], moves)
    finally:
      release.set()
      await asyncio.gather(first, second)
    self.assertEqual(["/v1/move"], moves)
    self.assertFalse(first.result()['success'])
    self.assertFalse(second.result()['success'])
    self.assertFalse((await embodiment.execute_action("finish_task", success=True, summary="Done"))['success'])
    await embodiment.execute_action("finish_task", success=False, summary="Stopped; recovery required")
    result = await embodiment.execute_action('move', **named_move("left"))
    self.assertFalse(result["success"])
    self.assertEqual(1, len(moves))

  async def test_unknown_tool_and_resource_cleanup(self):
    config = RobotConfig.load(CONFIGS / "single_arm.json")
    embodiment = Ros2Embodiment(config, transport=httpx.MockTransport(MockRobot()))
    await embodiment.initialize()
    self.assertFalse((await embodiment.execute_action("navigate", name="spot-only"))['success'])
    self.assertFalse((await embodiment.execute_action("finish_task", success="yes", summary="Done"))['success'])
    await embodiment.close()
    await embodiment.close()
    self.assertTrue(embodiment.robot._client.is_closed)
    self.assertIsNone(embodiment.poller_task)


if __name__ == "__main__":
  unittest.main()
