"""Test-only driver and fixtures. These never command physical hardware."""

import asyncio
import fcntl
import os
import tempfile
import copy
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
from ros2_agent_server.protocol import Reply, BridgeError
from ros2_agent_server.primitive_driver import PrimitiveAdapter, HardwareBackend
from ros2_agent_server.robot_node import RobotBridgeNode
from ros2_agent_server.runtime import spin_until_stopped

ROOT = Path(__file__).resolve().parents[2]


def config(name="single_arm.json"):
    c = RobotConfig.load(ROOT / "agent" / "configs" / name)
    c.server.hardware = {
        'profiles': {a.id: dict(orientation=[0.,0.,0.,1.],
            approach_m=.1, lift_m=.12, transfer_height_m=5.) for a in c.arms},
        'position_names': {a.id: {'home':'Initial position.', 'ready':'Ready position.'} for a in c.arms}}
    return c


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


class MeasuredBackend(HardwareBackend):
    coordinated_motion=coordinated_gripper=coupled_transfer=joint_targets=recovery=True

    def __init__(self,node):
        self.node=node
        self.fail_phase=None
        self.prepared=[]
        self.motions=[]
        self.stop_ids=[]
        self.named_positions={a.id: {'home':dict(kind='pose',reference='flange',**arm_state()['flange_pose']),
            'ready':dict(kind='pose',reference='tcp',frame_id='world',position=[.2,0.,.4],orientation=[0.,0.,0.,1.])}
            for a in node.config.arms}
        self.resolved_steps=[]

    async def prepare(self,steps,ids,coupled,context):
        context.check()
        resolved=copy.deepcopy(steps)
        for step in resolved:
            if step['operation'] != 'move':
                continue
            goals=[]
            for arm,target in zip(ids,step['targets']):
                if target['kind']=='named':
                    if target['name'] not in self.named_positions.get(arm,{}):
                        raise BridgeError(422,f'Backend has no named position {target["name"]!r} for arm {arm!r}')
                    target=copy.deepcopy(self.named_positions[arm][target['name']])
                if target['kind']=='joints' and not self.joint_targets:
                    raise BridgeError(501,'Backend joint targets are unsupported')
                goals.append(target)
            step['targets']=goals
        self.prepared=steps
        self.resolved_steps=resolved
        self.node.phase_index=0
        return dict(success=True)

    def result(self):
        if self.node.reply_status >= 400:
            error = BridgeError(self.node.reply_status, self.node.reply_payload.get('error','Injected driver failure'))
            error.payload.update(self.node.reply_payload)
            raise error
        if self.node.reply_status != 200:
            raise BridgeError(502,'Driver did not return completed operation',outcome='unknown')
        return copy.deepcopy(self.node.reply_payload)

    async def move(self,targets,ids,duration,coupled,context):
        context.check()
        result = self.result()
        if result.get('success') is not True:
            return result
        self.motions.append(copy.deepcopy(targets))
        if self.node.hold_moves:
            future=Future(executor=self.node.executor)
            self.node.held.append(future)
            if not await future:
                return dict(success=False,error='Stopped during motion',outcome='unknown')
        step=self.prepared[self.node.phase_index]
        self.node.phase_index+=1
        if self.fail_phase is not None and self.fail_phase==step.get('phase'):
            return dict(success=False,error='Measured target not reached',code='arrival_failed')
        for arm,target in zip(ids,self.resolved_steps[self.node.phase_index-1]['targets']):
            if target['kind']=='pose':
                measured={k:copy.deepcopy(target[k]) for k in ('frame_id','position','orientation')}
                if target.get('reference')=='tcp':
                    # Test-only calibration belongs to this mock mechanism.
                    from ros2_agent_server.pixels import transform_point
                    offset=transform_point([0.,0.,.05],dict(position=[0.,0.,0.],orientation=target['orientation']))
                    measured['position']=[v-o for v,o in zip(measured['position'],offset)]
                measured['position'][0]+=.0001
                self.node.states[arm]['flange_pose']=measured
        self.node.publish()
        context.check()
        return dict(success=True)

    async def gripper(self,openings,ids,context):
        context.check()
        result = self.result()
        if result.get('success') is not True:
            return result
        step=self.prepared[self.node.phase_index]
        self.node.phase_index+=1
        if self.fail_phase is not None and self.fail_phase==step.get('phase'):
            return dict(success=False,error='Gripper jammed')
        for arm,opening in zip(ids,openings):
            self.node.states[arm]['gripper']=dict(opening=opening,object_detected=(opening<.5 if step.get('phase') else None))
        self.node.publish()
        return dict(success=True)

    async def stop(self,ids,context):
        self.stop_ids.append(ids)
        self.node.release(False)
        return dict(success=True)

    async def recover(self,ids,context):
        context.check()
        result = self.result()
        if result.get('success') is not True:
            return result
        self.fail_phase=None
        for arm in ids:self.node.states[arm]=dict(arm_state(),fault=None)
        self.node.publish()
        return dict(success=True)


class FakeDriver(Node):
    def __init__(self, robot_config, **kwargs):
        super().__init__("test_driver", **robot_config.server.node_options(kwargs))
        self.config = robot_config
        self.calls = []
        self.backend = MeasuredBackend(self)
        self.adapter = PrimitiveAdapter(robot_config,self.backend,lambda:self.get_clock().now().nanoseconds)
        self.publish_images = True
        self.publish_states = True
        self.publish_geometry = True
        self.depth_data = None
        self.depth_offset_ns = 0
        self.depth_delay_ticks = 0
        self.geometry_queue = []
        self.state_stamp_ns = None
        self.skip_states = set()
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
            robot_config.camera_info_topic(c.id), qos_profile_sensor_data)
            for c in robot_config.cameras}
        self.depth_publishers = {c.id: self.create_publisher(DepthImage,
            robot_config.camera_depth_topic(c.id), qos_profile_sensor_data)
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
                    if arm_id in self.skip_states:
                        continue
                    state = self.states[arm_id]
                    stamp = self.state_stamp_ns if self.state_stamp_ns is not None else self.get_clock().now().nanoseconds
                    publisher.publish(String(data=json.dumps(dict(state, stamp_ns=stamp))))
            if self.publish_images:
                for camera in self.config.cameras:
                    message = CompressedImage(format="jpeg", data=self.images[camera.id])
                    message.header.frame_id = camera.optical_frame
                    message.header.stamp = self.get_clock().now().to_msg()
                    self.image_publishers[camera.id].publish(message)
                    if self.publish_geometry:
                        info = CameraInfo(header=message.header, width=48, height=32,
                            k=[100.,0.,24.,0.,100.,16.,0.,0.,1.],
                            r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                            p=[100.,0.,24.,0.,0.,100.,16.,0.,0.,0.,1.,0.])
                        depth = DepthImage(header=message.header, width=48, height=32,
                            encoding='16UC1', step=96, data=(self.depth_data if self.depth_data is not None
                                else struct.pack('<H', 1000)*48*32))
                        info.header = copy.deepcopy(message.header)
                        depth.header = copy.deepcopy(message.header)
                        info.header.frame_id = camera.camera_info_frame or camera.optical_frame
                        depth.header.frame_id = camera.depth_frame or camera.optical_frame
                        depth_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec+self.depth_offset_ns
                        depth.header.stamp.sec, depth.header.stamp.nanosec = divmod(depth_ns, 1_000_000_000)
                        self.info_publishers[camera.id].publish(info)
                        self.geometry_queue.append([self.depth_delay_ticks, camera.id, depth])
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

            for item in list(self.geometry_queue):
                if item[0] <= 0:
                    self.depth_publishers[item[1]].publish(item[2])
                    self.geometry_queue.remove(item)
                else:
                    item[0] -= 1

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
        self.calls.append((request.operation, request.resource_id, json.loads(request.payload_json)))
        result = await self.adapter.handle(request, response)
        # Fault injection exercises malformed/partial peer acknowledgements, not
        # an alternate driver implementation or a fake high-level plan executor.
        if not self.confirm_coordination and request.operation != 'stop':
            body = json.loads(result.payload_json)
            body.pop('coordinated', None)
            result.payload_json = json.dumps(body)
        return result

    def release(self, success=True):
        with self._lock:
            for future in self.held:
                if not future.done():
                    future.set_result(success)
            self.held.clear()


class RosFixture:
    def __init__(self, robot_config, driver_factory=FakeDriver):
        self.config = robot_config
        self.config.server.settling_dwell = 0.
        self.context = Context()
        # Lease a domain across test processes; never share delayed responses.
        self._domain_file = None
        for domain in range(180, 230):
            fd = os.open(f"{tempfile.gettempdir()}/agent-ros2-test-domain-{domain}.lock",
                         os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(fd)
                continue
            self._domain_file = fd
            break
        if self._domain_file is None:
            raise RuntimeError("No isolated ROS test domain available")
        rclpy.init(context=self.context, domain_id=domain,
                   signal_handler_options=SignalHandlerOptions.NO)
        self.robot = RobotBridgeNode(robot_config, context=self.context)
        self.gateway = HttpGatewayNode(robot_config, context=self.context)
        self.driver = driver_factory(robot_config, context=self.context)
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
        os.close(self._domain_file)
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
