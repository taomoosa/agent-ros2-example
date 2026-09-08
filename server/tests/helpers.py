"""Test-only driver and fixtures. These never command physical hardware."""

import asyncio
import io
import json
import struct
from pathlib import Path
import threading
import time

from PIL import Image
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from ros2_agent_interfaces.srv import RobotRequest
from sensor_msgs.msg import CompressedImage, CameraInfo, Image as DepthImage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String

from ros2_agent_server.gateway import HttpGatewayNode
from ros2_agent_server.models import RobotConfig
from ros2_agent_server.protocol import Reply
from ros2_agent_server.robot_node import RobotBridgeNode
from ros2_agent_server.runtime import spin_until_stopped

ROOT = Path(__file__).resolve().parents[2]


def config(name="single_arm.json"):
    return RobotConfig.load(ROOT / "agent" / "configs" / name)


def jpeg(color="red"):
    buffer = io.BytesIO()
    Image.new("RGB", (48, 32), color).save(buffer, "JPEG")
    return buffer.getvalue()


def arm_state():
    return {"moving": False, "flange_pose": {
        "frame_id": "world", "position": [0.1, 0.0, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]},
        "gripper": {"opening": 1.0}}


async def eventually(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("Timed out waiting for test condition")
        await asyncio.sleep(0.01)


class FakeDriver(Node):
    def __init__(self, robot_config, **kwargs):
        super().__init__("test_driver", **kwargs)
        self.config = robot_config
        self.calls = []
        self.publish_images = True
        self.publish_states = True
        self.publish_geometry = True
        self.confirm_coordination = True
        self.wrist_offset = 1.0
        self.hold_moves = False
        self.reply_status = 200
        self.reply_payload = {"success": True}
        self.held = []
        self.active = 0
        self.idle = threading.Event()
        self.idle.set()
        self._lock = threading.RLock()
        self.states = {arm.id: arm_state() for arm in robot_config.arms}
        self.state_publishers = {arm.id: self.create_publisher(String, robot_config.state_topic(arm.id), 1)
                                 for arm in robot_config.arms}
        self.image_publishers = {camera.id: self.create_publisher(
            CompressedImage, robot_config.camera_topic(camera.id), qos_profile_sensor_data)
            for camera in robot_config.cameras}
        self.info_publishers = {c.id: self.create_publisher(CameraInfo,
            f'{robot_config.server.namespace}/cameras/{c.id}/camera_info', qos_profile_sensor_data)
            for c in robot_config.cameras}
        self.depth_publishers = {c.id: self.create_publisher(DepthImage,
            f'{robot_config.server.namespace}/cameras/{c.id}/depth/aligned', qos_profile_sensor_data)
            for c in robot_config.cameras}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.images = {camera.id: jpeg("blue" if camera.arm_id else "red") for camera in robot_config.cameras}
        self._group = ReentrantCallbackGroup()
        self.service = self.create_service(RobotRequest, robot_config.server.driver_service,
                                          self.serve, callback_group=self._group)
        self.timer = self.create_timer(0.04, self.publish, callback_group=self._group)

    def publish(self):
        with self._lock:
            if self.publish_states:
                for arm_id, publisher in self.state_publishers.items():
                    publisher.publish(String(data=json.dumps(self.states[arm_id])))
            if self.publish_images:
                for camera in self.config.cameras:
                    message = CompressedImage(format="jpeg", data=self.images[camera.id])
                    message.header.frame_id = camera.optical_frame
                    message.header.stamp = self.get_clock().now().to_msg()
                    self.image_publishers[camera.id].publish(message)
                    if self.publish_geometry:
                        info = CameraInfo(header=message.header, width=48, height=32,
                            k=[100.,0.,24.,0.,100.,16.,0.,0.,1.])
                        depth = DepthImage(header=message.header, width=48, height=32,
                            encoding='16UC1', step=96, data=struct.pack('<H', 1000)*48*32)
                        self.info_publishers[camera.id].publish(info)
                        self.depth_publishers[camera.id].publish(depth)
                        tf = TransformStamped()
                        tf.header.stamp = message.header.stamp
                        tf.header.frame_id = self.config.world_frame
                        tf.child_frame_id = camera.optical_frame
                        tf.transform.rotation.w = 1.
                        tf.transform.translation.x = self.wrist_offset if camera.arm_id else 0.
                        self.tf_broadcaster.sendTransform(tf)
                        if camera.arm_id:
                            tf.child_frame_id = camera.parent_frame
                            self.tf_broadcaster.sendTransform(tf)

    async def serve(self, request, response):
        with self._lock:
            self.active += 1
            self.idle.clear()
        try:
            return await self.execute(request, response)
        finally:
            with self._lock:
                self.active -= 1
                if self.active == 0:
                    self.idle.set()

    async def execute(self, request, response):
        payload = json.loads(request.payload_json)
        with self._lock:
            self.calls.append((request.operation, request.resource_id, payload))
            if request.operation == "stop":
                self.release(False)
                return Reply(payload={"success": True}).to_ros(response)
            if request.operation in {"move_arm", "execute_plan", "move_arms", "reset_arms", "recover_arms"} and self.hold_moves:
                future = Future(executor=self.executor)
                self.held.append(future)
            else:
                future = None
        if future is not None:
            completed = await future
            return Reply(payload={"success": completed, "message": "Completed" if completed else "Stopped"}).to_ros(response)
        with self._lock:
            if self.reply_status == 200 and self.reply_payload.get("success"):
                if request.operation == "move_arm":
                    self.states[request.resource_id]["flange_pose"] = {
                        key: payload[key] for key in ("frame_id", "position", "orientation")}
                if request.operation == "set_gripper":
                    self.states[request.resource_id]["gripper"] = payload
        if request.operation == 'execute_plan' and self.reply_status == 200 and self.reply_payload.get('success'):
            for target in payload['targets']:
                gripper = self.states[target['arm_id']]['gripper']
                if payload['stage'] in {'pick', 'place'}:
                    gripper['object_detected'] = payload['stage'] == 'pick'
            self.publish()
        if request.operation == 'recover_arms' and self.reply_status == 200 and self.reply_payload.get('success'):
            for state in self.states.values():
                state['fault'] = None
                state['gripper'] = {'opening': 1.0, 'fault': None, 'object_detected': False}
            self.publish()
        result = dict(self.reply_payload)
        if payload.get('coordinated') and self.confirm_coordination:
            result.update(coordinated=True, completed_arm_ids=payload.get('arm_ids',
                [t['arm_id'] for t in payload.get('targets', [])]))
        return Reply(status=self.reply_status, payload=result).to_ros(response)

    def release(self, success=True):
        with self._lock:
            for future in self.held:
                if not future.done():
                    future.set_result(success)
            self.held.clear()


class RosFixture:
    def __init__(self, robot_config):
        self.config = robot_config
        self.context = Context()
        # Each test context has a local discovery domain, independent of user robots.
        rclpy.init(context=self.context, domain_id=173,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.robot = RobotBridgeNode(robot_config, context=self.context)
        self.gateway = HttpGatewayNode(robot_config, context=self.context)
        self.driver = FakeDriver(robot_config, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.stop_event = threading.Event()
        for node in (self.robot, self.gateway, self.driver):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=spin_until_stopped, args=(self.executor, self.stop_event), name="test-ros2-executor")
        self.thread.start()

    async def ready(self):
        await eventually(lambda: self.gateway.client.service_is_ready() and self.robot.driver.service_is_ready())
        await eventually(lambda: all(self.robot.store.sequence(camera.id) for camera in self.config.cameras))
        await eventually(lambda: len(self.robot.store._arms) == len(self.config.arms))

    def close(self):
        self.driver.release(False)
        self.robot.close_pending()
        self.robot.wait_for_idle()
        self.driver.idle.wait(2.0)
        self.stop_event.set()
        self.thread.join(timeout=5.0)
        self.executor.shutdown(timeout_sec=5.0)
        for node in (self.driver, self.gateway, self.robot):
            node.destroy_node()
        self.context.try_shutdown()
        if self.thread.is_alive():
            raise AssertionError("ROS2 executor thread did not stop")


class MockGeminiStream:
    def __init__(self, calls):
        self.calls = iter(calls)
        self.messages = []
        self.closed = False

    def Start(self, on_message, on_done):
        self.on_message = on_message

    def Send(self, message):
        self.messages.append(message)
        if "setup" in message:
            self.on_message({"setupComplete": {}})
        elif "clientContent" in message or "toolResponse" in message:
            call = next(self.calls, None)
            if call:
                self.on_message({"toolCall": {"functionCalls": [call]}})

    def Shutdown(self):
        self.closed = True
