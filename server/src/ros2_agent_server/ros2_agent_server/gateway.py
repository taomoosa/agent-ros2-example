"""ROS2 node used by HTTP handlers to request work from the robot bridge."""

import asyncio
import json

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ros2_agent_interfaces.srv import RobotRequest

from .protocol import BridgeError, Reply
from .qos import http_service_qos


class HttpGatewayNode(Node):
    def __init__(self, config, **kwargs):
        super().__init__("http_gateway", **kwargs)
        self.client = self.create_client(
            RobotRequest, config.request_service, callback_group=ReentrantCallbackGroup(),
            qos_profile=http_service_qos())

    async def request(self, operation, resource_id, payload, timeout):
        if not self.client.service_is_ready():
            raise BridgeError(503, "Robot bridge service is unavailable")
        request = RobotRequest.Request(
            operation=operation, resource_id=resource_id,
            payload_json=json.dumps(payload, allow_nan=False), timeout_sec=float(timeout))
        ros_future = self.client.call_async(request)
        loop = asyncio.get_running_loop()
        result = loop.create_future()

        def done(future):
            # rclpy callbacks run on executor threads, not the HTTP asyncio loop.
            def deliver():
                if result.done():
                    return
                try:
                    result.set_result(Reply.from_ros(future.result()))
                except Exception as exc:
                    result.set_exception(BridgeError(502, f"Robot bridge failed: {exc}", outcome="unknown"))
            try:
                loop.call_soon_threadsafe(deliver)
            except RuntimeError:
                pass  # HTTP loop already closed during shutdown.

        ros_future.add_done_callback(done)
        try:
            return await asyncio.wait_for(result, timeout=timeout + 0.5)
        except asyncio.TimeoutError as exc:
            raise BridgeError(504, "Robot bridge request timed out", outcome="unknown") from exc
        finally:
            if not ros_future.done():
                self.client.remove_pending_request(ros_future)
                ros_future.cancel()
