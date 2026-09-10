import unittest

import httpx

from ros2_agent_server.api import create_app
from ros2_agent_server.protocol import BridgeError, Reply
from helpers import config, jpeg


class Gateway:
    def __init__(self):
        self.calls = []
        self.reply = Reply(payload={"success": True})
        self.error = None

    async def request(self, *args):
        self.calls.append(args)
        if self.error:
            raise self.error
        return self.reply


class ApiTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.gateway = Gateway()
        self.client = httpx.AsyncClient(transport=httpx.ASGITransport(
            app=create_app(self.gateway, config())), base_url="http://test")
        self.addAsyncCleanup(self.client.aclose)

    async def test_routes_and_timeout_budgets(self):
        response = await self.client.post('/v1/move', json=dict(arm_ids=['arm'], targets=[dict(kind='pose', **{
            "frame_id": "world", "position": [0, 0, 0], "orientation": [0, 0, 0, 1]})], duration=4))
        self.assertEqual(200, response.status_code)
        self.assertEqual(("move", ""), self.gateway.calls[-1][:2])
        self.assertEqual(141, self.gateway.calls[-1][3])
        await self.client.post('/v1/gripper', json=dict(arm_ids=['arm'], **{"opening": 0.5}))
        self.assertEqual("gripper", self.gateway.calls[-1][0])
        await self.client.post("/v1/stop", json={"all_arms": True})
        self.assertEqual({"arm_ids": ["arm"]}, self.gateway.calls[-1][2])
        await self.client.get("/v1/state")
        self.assertEqual("state", self.gateway.calls[-1][0])

    async def test_invalid_body_and_resource_do_not_call_ros(self):
        body = {"frame_id": "world", "position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
        for path, payload, status in [
            ('/v1/move',dict(arm_ids=['missing'],targets=[dict(kind='pose',**body)]),422),
            *[('/v1/move',dict(arm_ids=['arm'],targets=[dict(kind='pose',**(body|override))]),422)
              for override in ({'position':[0,0]}, {'orientation':[0,0,0,0]}, {'frame_id':'unknown'}, {'extra':True})],
            ('/v1/gripper',dict(arm_ids=['arm'],opening=2),422),
            ('/v1/gripper',dict(arm_ids=['arm'],opening=True),422),
            ('/v1/stop',dict(arm_ids=['missing']),422),
        ]:
            with self.subTest(payload=payload):
                self.assertEqual(status, (await self.client.post(path, json=payload)).status_code)
        self.assertEqual([], self.gateway.calls)

    async def test_jpeg_response_preserves_bytes_and_metadata(self):
        image = jpeg()
        self.gateway.reply = Reply(payload={"frame_id": "overhead_optical", "stamp_ns": 123},
                                   data=image, content_type="image/jpeg")
        response = await self.client.get("/v1/cameras/overhead/image")
        self.assertEqual(image, response.content)
        self.assertEqual("image/jpeg", response.headers["content-type"])
        self.assertEqual("no-store", response.headers["cache-control"])
        self.assertEqual("overhead_optical", response.headers["x-frame-id"])

    async def test_unavailable_timeout_and_driver_failure(self):
        for status in (503, 504):
            self.gateway.error = BridgeError(status, "Unavailable", outcome="unknown")
            response = await self.client.get("/v1/state")
            self.assertEqual(status, response.status_code)
            self.assertEqual("unknown", response.json()["outcome"])
        self.gateway.error = None
        self.gateway.reply = Reply(status=409, payload={"success": False, "error": "Collision"})
        response = await self.client.post('/v1/gripper', json=dict(arm_ids=['arm'], **{"opening": 0.5}))
        self.assertEqual(409, response.status_code)
        self.assertFalse(response.json()["success"])

    async def test_nonfinite_json_is_rejected(self):
        response = await self.client.post("/v1/gripper",
            content=b'{"arm_ids":["arm"],"opening": NaN}', headers={"Content-Type": "application/json"})
        self.assertEqual(422, response.status_code)
        self.assertEqual([], self.gateway.calls)

    async def test_empty_commands_validate_body_before_dispatch(self):
        for operation in ('recover',):
            for raw in (b'{"arm_id":"missing"}', b'null', b'[]', b'false', b'42', b'""', b'{'):
                with self.subTest(operation=operation, raw=raw):
                    result = await self.client.post('/v1/arms/' + operation, content=raw,
                        headers={'Content-Type': 'application/json'})
                    self.assertEqual(422, result.status_code)
            self.assertEqual([], self.gateway.calls)
            for raw in (b'', b'{}'):
                result = await self.client.post('/v1/arms/' + operation, content=raw,
                    headers={'Content-Type': 'application/json'})
                self.assertEqual(200, result.status_code)
                self.assertEqual({}, self.gateway.calls[-1][2])
            self.gateway.calls.clear()

    async def test_removed_routes_and_stop_selector_cannot_dispatch(self):
        for route in ('/v1/arms/arm/pose', '/v1/arms/arm/gripper',
                      '/v1/arms/poses', '/v1/arms/reset'):
            with self.subTest(route=route):
                self.assertEqual(404, (await self.client.post(route,json={})).status_code)
        for body in ({'arm_id':'arm'}, {'arm_id':None}, {'all_arms':False}):
            self.assertEqual(422, (await self.client.post('/v1/stop',json=body)).status_code)
        self.assertEqual([],self.gateway.calls)
