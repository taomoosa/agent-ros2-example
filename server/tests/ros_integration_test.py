import asyncio
import unittest
from unittest import mock

import httpx

from ros2_agent_server.api import create_app
from ros2_agent_server.protocol import BridgeError
from helpers import ROOT, RosFixture, MockGeminiStream, config, eventually


class RosIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.fixture = RosFixture(config("dual_arm.json"))
        self.addCleanup(self.fixture.close)
        await self.fixture.ready()
        self.http = httpx.AsyncClient(transport=httpx.ASGITransport(
            app=create_app(self.fixture.gateway, self.fixture.config)), base_url="http://test")
        self.addAsyncCleanup(self.http.aclose)

    async def test_topics_state_and_jpeg_cross_real_ros_services(self):
        response = await self.http.get("/v1/state")
        self.assertEqual(200, response.status_code)
        self.assertEqual(["left", "right"], [arm["id"] for arm in response.json()["arms"]])
        for camera in self.fixture.config.cameras:
            before = self.fixture.robot.get_clock().now().nanoseconds
            response = await self.http.get(f"/v1/cameras/{camera.id}/image")
            self.assertEqual(200, response.status_code)
            self.assertEqual(self.fixture.driver.images[camera.id], response.content)
            self.assertGreaterEqual(int(response.headers["x-stamp-ns"]), before)
            self.assertEqual(camera.optical_frame, response.headers["x-frame-id"])

    async def test_motion_waits_for_driver_completion_and_stop_remains_available(self):
        self.fixture.driver.hold_moves = True
        move = asyncio.create_task(self.http.post("/v1/arms/left/pose", json={
            "frame_id": "world", "position": [0.1, 0, 0.3], "orientation": [0, 0, 0, 1]}))
        await eventually(lambda: bool(self.fixture.driver.held))
        self.assertFalse(move.done())
        state = await self.http.get("/v1/state")
        self.assertEqual(200, state.status_code)
        stop = await asyncio.wait_for(self.http.post("/v1/stop", json={"arm_id": "left"}), timeout=2)
        self.assertTrue(stop.json()["success"])
        self.assertFalse((await move).json()["success"])
        self.assertEqual(["move_arm", "stop"], [call[0] for call in self.fixture.driver.calls])

    async def test_multiple_camera_waiters_do_not_block_subscription_callbacks(self):
        responses = await asyncio.wait_for(asyncio.gather(*[
            self.http.get("/v1/cameras/overhead/image") for _ in range(12)
        ]), timeout=5)
        self.assertTrue(all(response.status_code == 200 for response in responses),
                        [(r.status_code, r.text if r.status_code != 200 else "JPEG") for r in responses])

    async def test_camera_timeout_returns_no_cached_image(self):
        self.fixture.driver.publish_images = False
        await asyncio.sleep(0.1)  # Drain already published DDS samples.
        reply = await self.fixture.gateway.request("camera", "overhead", {}, 0.1)
        self.assertEqual(504, reply.status)
        self.assertEqual(b"", reply.data)
        self.assertEqual([], self.fixture.robot._pending)

    async def test_driver_timeout_does_not_retry_and_stop_still_works(self):
        self.fixture.driver.hold_moves = True
        reply = await self.fixture.gateway.request("move_arm", "left", {
            "frame_id": "world", "position": [0, 0, 0], "orientation": [0, 0, 0, 1]}, 0.1)
        self.assertEqual(504, reply.status)
        self.assertEqual("unknown", reply.payload["outcome"])
        self.assertEqual(1, len(self.fixture.driver.calls))
        response = await self.http.post("/v1/stop", json={})
        self.assertTrue(response.json()["success"])

    async def test_invalid_internal_ros_request_is_rejected(self):
        reply = await self.fixture.gateway.request("set_gripper", "missing", {"opening": 0.5}, 1)
        self.assertEqual(404, reply.status)
        self.assertEqual([], self.fixture.driver.calls)

    async def test_driver_rejection_and_async_acceptance_are_not_success(self):
        self.fixture.driver.reply_status = 409
        self.fixture.driver.reply_payload = {"success": False, "error": "Unreachable"}
        response = await self.http.post("/v1/arms/right/gripper", json={"opening": 0.5})
        self.assertEqual(409, response.status_code)
        self.fixture.driver.reply_status = 200
        self.fixture.driver.reply_payload = {"success": True}
        await self.http.post('/v1/stop', json={})
        self.assertTrue((await self.http.post('/v1/arms/recover')).json()['success'])
        self.fixture.driver.reply_status = 202
        response = await self.http.post("/v1/arms/right/gripper", json={"opening": 0.5})
        self.assertEqual(502, response.status_code)
        self.assertEqual("unknown", response.json()["outcome"])

    async def test_agent_and_mock_gemini_use_real_server_and_ros_nodes(self):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application
        robot_config = AgentConfig.load(str(ROOT / "agent/configs/dual_arm.json"))
        stream = MockGeminiStream([
            {"id": "state", "name": "get_robot_state", "args": {}},
            {"id": "move", "name": "move_arm", "args": {"arm_id": "right", "frame_id": "world",
                "position": [0.2, 0, 0.4], "orientation": [0, 0, 0, 1]}},
            {"id": "grip", "name": "set_gripper", "args": {"arm_id": "right", "opening": 0.2}},
            {"id": "done", "name": "finish_task", "args": {"success": True, "summary": "Completed"}},
        ])
        with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
            client.return_value.create_stream.return_value = stream
            result = await run_application(robot_config, "Move right arm and close gripper", model="mock",
                api_key="mock-key", timeout=15, transport=httpx.ASGITransport(
                    app=create_app(self.fixture.gateway, self.fixture.config)))
        self.assertTrue(result["success"])
        self.assertTrue(stream.closed)
        self.assertEqual([("move_arm", "right"), ("set_gripper", "right")],
                         [(call[0], call[1]) for call in self.fixture.driver.calls])
        responses = [m["toolResponse"]["functionResponses"][0] for m in stream.messages if "toolResponse" in m]
        self.assertEqual(["state", "move", "grip", "done"], [r["id"] for r in responses])
        self.assertTrue(all(r["response"].get("success", True) for r in responses))
        self.assertTrue(any("realtimeInput" in message for message in stream.messages))


class TcpAgentIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def test_single_arm_agent_over_real_http_and_ros(self):
        import dataclasses
        import socket
        import uvicorn
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from run_ros2 import run_application

        fixture = RosFixture(config())
        self.addCleanup(fixture.close)
        await fixture.ready()
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        sock.listen()
        sock.setblocking(False)
        address = f"http://127.0.0.1:{sock.getsockname()[1]}"
        server = uvicorn.Server(uvicorn.Config(
            create_app(fixture.gateway, fixture.config), log_level="error", lifespan="off"))
        serving = asyncio.create_task(server.serve(sockets=[sock]))
        try:
            await eventually(lambda: server.started)
            robot_config = dataclasses.replace(AgentConfig.load(ROOT / "agent/configs/single_arm.json"), robot_url=address)
            stream = MockGeminiStream([
                {"id": "move", "name": "move_arm", "args": {"arm_id": "arm", "frame_id": "world",
                    "position": [0.2, 0, 0.4], "orientation": [0, 0, 0, 1]}},
                {"id": "grip", "name": "set_gripper", "args": {"arm_id": "arm", "opening": 0.2}},
                {"id": "done", "name": "finish_task", "args": {"success": True, "summary": "Completed"}},
            ])
            with mock.patch("model.live_api_client.GeminiLiveApiClient") as client:
                client.return_value.create_stream.return_value = stream
                result = await run_application(robot_config, "Move and grip", model="mock", api_key="mock-key", timeout=15)
            self.assertTrue(result["success"])
            self.assertTrue(stream.closed)
            self.assertEqual([("move_arm", "arm"), ("set_gripper", "arm")],
                             [(call[0], call[1]) for call in fixture.driver.calls])
            self.assertTrue(any("realtimeInput" in message for message in stream.messages))
        finally:
            server.should_exit = True
            await asyncio.wait_for(serving, timeout=5)
            sock.close()
