"""ROS2 node used by HTTP handlers to request work from the robot bridge."""

import asyncio
import json
import uuid
from .diagnostics import event, request_id

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ros2_agent_interfaces.srv import RobotRequest

from .protocol import BridgeError, Reply
from .qos import http_service_qos


class HttpGatewayNode(Node):
    def __init__(self, config, **kwargs):
        super().__init__("http_gateway", **config.server.node_options(kwargs))
        self.config = config
        self.client = self.create_client(
            RobotRequest, config.request_service, callback_group=ReentrantCallbackGroup(),
            qos_profile=http_service_qos())

    async def request(self, operation, resource_id, payload, timeout):
        trace = request_id.get() or uuid.uuid4().hex
        event(self.get_logger(), "ros_send", request_id=trace, operation=operation, resource=resource_id)
        if not self.client.service_is_ready():
            raise BridgeError(503, "Robot bridge service is unavailable")
        request = RobotRequest.Request(
            request_id=trace, operation=operation, resource_id=resource_id,
            payload_json=json.dumps(payload, allow_nan=False), timeout_sec=float(timeout),
            deadline_ns=self.get_clock().now().nanoseconds+round(timeout*1e9))
        ros_future = self.client.call_async(request)
        loop = asyncio.get_running_loop()
        result = loop.create_future()

        def done(future):
            event(self.get_logger(), "ros_future_done", request_id=trace)
            # rclpy callbacks run on executor threads, not the HTTP asyncio loop.
            def deliver():
                event(self.get_logger(), "asyncio_deliver", request_id=trace, discarded=result.done())
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
            return await asyncio.wait_for(result, timeout=timeout + self.config.server.ros_response_margin)
        except asyncio.TimeoutError as exc:
            event(self.get_logger(), "gateway_timeout", level="warning", request_id=trace)
            raise BridgeError(504, "Robot bridge request timed out", outcome="unknown") from exc
        finally:
            if not ros_future.done():
                event(self.get_logger(), "ros_wait_cancelled", request_id=trace)
                self.client.remove_pending_request(ros_future)
                ros_future.cancel()
