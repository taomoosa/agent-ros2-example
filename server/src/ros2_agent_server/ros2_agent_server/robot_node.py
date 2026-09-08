"""ROS2 topic aggregation and asynchronous forwarding to a robot driver."""

from dataclasses import dataclass
import json
import math
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from ros2_agent_interfaces.srv import RobotRequest
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .protocol import BridgeError, Reply, validate_request
from .state import StateStore


@dataclass(eq=False)
class Pending:
    future: Future
    deadline: float
    camera: tuple | None = None
    upstream: Future | None = None


class RobotBridgeNode(Node):
    def __init__(self, config, **kwargs):
        super().__init__("robot_bridge", **kwargs)
        self.config = config
        self.store = StateStore(config)
        self._pending = []
        self._lock = threading.RLock()
        self._closing = False
        self._active_requests = 0
        self._idle = threading.Event()
        self._idle.set()
        self._group = ReentrantCallbackGroup()
        self.driver = self.create_client(
            RobotRequest, config.server.driver_service, callback_group=self._group)
        self.service = self.create_service(
            RobotRequest, config.request_service, self._serve, callback_group=self._group)
        self._topic_subscriptions = []
        for arm in config.arms:
            self._topic_subscriptions.append(self.create_subscription(
                String, config.state_topic(arm.id),
                lambda msg, arm_id=arm.id: self._arm_state(arm_id, msg),
                1, callback_group=self._group))
        for camera in config.cameras:
            self._topic_subscriptions.append(self.create_subscription(
                CompressedImage, config.camera_topic(camera.id),
                lambda msg, camera_id=camera.id: self._image(camera_id, msg),
                qos_profile_sensor_data, callback_group=self._group))
        # Wall-time deadlines must also expire when simulated ROS time is paused.
        self._timer = self.create_timer(
            0.02, self._expire, callback_group=self._group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _arm_state(self, arm_id, message):
        try:
            self.store.update_arm(arm_id, json.loads(message.data))
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid state for {arm_id}: {exc}")

    def _image(self, camera_id, message):
        try:
            if "jpeg" not in message.format.lower():
                raise ValueError("Expected a JPEG CompressedImage")
            stamp = message.header.stamp
            self.store.update_camera(camera_id, bytes(message.data), message.header.frame_id,
                                     stamp.sec * 1_000_000_000 + stamp.nanosec)
        except (TypeError, ValueError, OSError) as exc:
            self.get_logger().warning(f"Ignoring invalid image for {camera_id}: {exc}")
            return
        with self._lock:
            for pending in list(self._pending):
                if pending.camera is None or pending.camera[0] != camera_id:
                    continue
                frame = self.store.fresh_camera(*pending.camera)
                if frame:
                    self._complete(pending, Reply(
                        payload={"frame_id": frame.frame_id, "stamp_ns": frame.stamp_ns},
                        data=frame.data, content_type="image/jpeg"))

    def _complete(self, pending, reply):
        with self._lock:
            if pending not in self._pending:
                return
            self._pending.remove(pending)
            if not pending.future.done():
                pending.future.set_result(reply)

    def _expire(self):
        with self._lock:
            for pending in list(self._pending):
                if time.monotonic() >= pending.deadline:
                    if pending.upstream is not None:
                        self.driver.remove_pending_request(pending.upstream)
                        pending.upstream.cancel()
                    self._complete(pending, Reply.from_error(BridgeError(
                        504, "Fresh camera frame timed out" if pending.camera else "Driver command timed out",
                        outcome=None if pending.camera else "unknown")))

    async def _camera(self, camera_id, timeout):
        with self._lock:
            if self._closing:
                raise BridgeError(503, "Robot bridge is shutting down")
            pending = Pending(
                Future(executor=self.executor), time.monotonic() + timeout,
                camera=(camera_id, self.store.sequence(camera_id), self.get_clock().now().nanoseconds))
            self._pending.append(pending)
        return await pending.future

    async def _command(self, operation, resource_id, payload, timeout):
        with self._lock:
            if self._closing:
                raise BridgeError(503, "Robot bridge is shutting down")
            if not self.driver.service_is_ready():
                raise BridgeError(503, "Robot driver service is unavailable")
            request = RobotRequest.Request(
                operation=operation, resource_id=resource_id,
                payload_json=json.dumps(payload, allow_nan=False), timeout_sec=timeout)
            upstream = self.driver.call_async(request)
            pending = Pending(Future(executor=self.executor), time.monotonic() + timeout, upstream=upstream)
            self._pending.append(pending)

        def done(future):
            if future.cancelled():
                return
            try:
                reply = Reply.from_ros(future.result())
                if (reply.status < 400 and (reply.status != 200
                        or type(reply.payload.get("success")) is not bool)):
                    raise BridgeError(502, "Driver must report completion with a boolean success", outcome="unknown")
            except BridgeError as exc:
                reply = Reply.from_error(exc)
            except Exception as exc:
                reply = Reply.from_error(BridgeError(502, f"Driver failed: {exc}", outcome="unknown"))
            self._complete(pending, reply)

        upstream.add_done_callback(done)
        return await pending.future

    async def _serve(self, request, response):
        with self._lock:
            self._active_requests += 1
            self._idle.clear()
        try:
            return await self._request(request, response)
        finally:
            with self._lock:
                self._active_requests -= 1
                if self._active_requests == 0:
                    self._idle.set()

    def wait_for_idle(self, timeout=2.0):
        return self._idle.wait(timeout)

    async def _request(self, request, response):
        try:
            if not math.isfinite(request.timeout_sec) or not 0 < request.timeout_sec <= 65:
                raise BridgeError(422, "timeout_sec must be finite and in (0, 65]")
            try:
                payload = json.loads(request.payload_json)
            except ValueError as exc:
                raise BridgeError(422, "Invalid JSON request payload") from exc
            payload = validate_request(self.config, request.operation, request.resource_id, payload)
            if request.operation == "state":
                reply = Reply(payload=self.store.state(self.get_clock().now().nanoseconds))
            elif request.operation == "camera":
                reply = await self._camera(request.resource_id, min(request.timeout_sec, self.config.server.camera_timeout))
            else:
                reply = await self._command(request.operation, request.resource_id, payload, request.timeout_sec)
        except BridgeError as exc:
            reply = Reply.from_error(exc)
        except Exception as exc:
            self.get_logger().error(f"Request failed: {exc}")
            reply = Reply.from_error(BridgeError(500, "Robot bridge request failed", outcome="unknown"))
        return reply.to_ros(response)

    def close_pending(self):
        with self._lock:
            self._closing = True
            for pending in list(self._pending):
                if pending.upstream is not None:
                    self.driver.remove_pending_request(pending.upstream)
                    pending.upstream.cancel()
                self._complete(pending, Reply.from_error(BridgeError(503, "Robot bridge is shutting down")))
