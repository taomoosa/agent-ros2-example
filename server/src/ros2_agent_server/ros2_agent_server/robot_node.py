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
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException

from .pixels import PixelPlans
from std_msgs.msg import String

from .protocol import BridgeError, Reply, validate_request
from .state import StateStore
from .qos import http_service_qos


@dataclass(eq=False)
class Pending:
    future: Future
    deadline: float
    camera: tuple | None = None
    upstream: Future | None = None
    capture: bool = False


class RobotBridgeNode(Node):
    def __init__(self, config, **kwargs):
        super().__init__("robot_bridge", **kwargs)
        self.config = config
        self.store = StateStore(config)
        self.pixels = PixelPlans(config)
        self._calibration = {}
        self._depth = {}
        self._motion_busy = False
        self._motion_uncertain = False
        self._recovery_required = False
        self._all_stopped = False
        self._generation = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
            RobotRequest, config.request_service, self._serve, callback_group=self._group,
            qos_profile=http_service_qos())
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
            prefix = f"{config.server.namespace}/cameras/{camera.id}"
            self._topic_subscriptions.append(self.create_subscription(
                CameraInfo, prefix + '/camera_info',
                lambda msg, camera_id=camera.id: self._calibration.update({camera_id: msg}),
                qos_profile_sensor_data, callback_group=self._group))
            self._topic_subscriptions.append(self.create_subscription(
                Image, prefix + '/depth/aligned',
                lambda msg, camera_id=camera.id: self._depth_image(camera_id, msg),
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
        self._try_cameras()

    def _depth_image(self, camera_id, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        cache = self._depth.setdefault(camera_id, {})
        cache[stamp] = message
        while len(cache) > 16:
            del cache[next(iter(cache))]
        self._try_cameras()

    def _pose_at(self, frame_id, stamp_ns):
        tf = self.tf_buffer.lookup_transform(self.config.world_frame, frame_id, Time(nanoseconds=stamp_ns)).transform
        return dict(frame_id=self.config.world_frame,
                    position=[tf.translation.x, tf.translation.y, tf.translation.z],
                    orientation=[tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])

    def _try_cameras(self):
        with self._lock:
            for pending in list(self._pending):
                if pending.camera is None:
                    continue
                camera_id = pending.camera[0]
                frame = self.store.fresh_camera(*pending.camera)
                if frame is None:
                    continue
                if not pending.capture:
                    self._complete(pending, Reply(
                        payload={"frame_id": frame.frame_id, "stamp_ns": frame.stamp_ns},
                        data=frame.data, content_type="image/jpeg"))
                    continue
                info = self._calibration.get(camera_id)
                depth = self._depth.get(camera_id, {}).get(frame.stamp_ns)
                if info is None or depth is None:
                    continue
                camera = next(c for c in self.config.cameras if c.id == camera_id)
                try:
                    camera_pose = self._pose_at(frame.frame_id, frame.stamp_ns)
                    flange_pose = (self._pose_at(camera.parent_frame, frame.stamp_ns)
                                   if camera.mount == 'flange' else None)
                    metadata = self.pixels.capture(camera_id, frame, info, depth, camera_pose, flange_pose)
                except TransformException:
                    continue
                except ValueError as exc:
                    self._complete(pending, Reply.from_error(BridgeError(422, str(exc))))
                    continue
                self._complete(pending, Reply(payload=metadata))

    def _complete(self, pending, reply):
        with self._lock:
            if pending not in self._pending:
                return
            self._pending.remove(pending)
            if not pending.future.done():
                pending.future.set_result(reply)

    def _expire(self):
        self._try_cameras()
        with self._lock:
            for pending in list(self._pending):
                if time.monotonic() >= pending.deadline:
                    if pending.upstream is not None:
                        self.driver.remove_pending_request(pending.upstream)
                        pending.upstream.cancel()
                    self._complete(pending, Reply.from_error(BridgeError(
                        504, "Fresh synchronized image, depth, calibration or capture-time TF unavailable" if pending.camera else "Driver command timed out",
                        outcome=None if pending.camera else "unknown")))

    async def _camera(self, camera_id, timeout, *, capture=False):
        with self._lock:
            if self._closing:
                raise BridgeError(503, "Robot bridge is shutting down")
            pending = Pending(
                Future(executor=self.executor), time.monotonic() + timeout,
                camera=(camera_id, self.store.sequence(camera_id), self.get_clock().now().nanoseconds),
                capture=capture)
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
                state = self.store.state(self.get_clock().now().nanoseconds)
                state.update(recovery_required=self._recovery_required, motion_outcome_unknown=self._motion_uncertain)
                reply = Reply(payload=state)
            elif request.operation in {"camera", "capture"}:
                reply = await self._camera(request.resource_id, min(request.timeout_sec, self.config.server.camera_timeout),
                                           capture=request.operation == "capture")
            else:
                reply = await self._workflow(request.operation, request.resource_id, payload, request.timeout_sec)
        except BridgeError as exc:
            reply = Reply.from_error(exc)
        except Exception as exc:
            self.get_logger().error(f"Request failed: {exc}")
            reply = Reply.from_error(BridgeError(500, "Robot bridge request failed", outcome="unknown"))
        return reply.to_ros(response)

    async def _workflow(self, operation, resource_id, payload, timeout):
        if operation == 'stop':
            self._generation += 1
            self._recovery_required = self._recovery_required or self._motion_busy or any(
                p['state'] in {'picked', 'verified', 'executing'} for p in self.pixels.plans.values())
            self.pixels.invalidate()
            self._motion_uncertain = self._motion_uncertain or self._motion_busy
            reply = await self._command(operation, resource_id, payload, timeout)
            if reply.status == 200 and reply.payload.get('success') is True:
                if payload.get('arm_id') is None:
                    self._motion_uncertain = False
                    self._all_stopped = True
            else:
                self._motion_uncertain = True
            return reply
        if self._motion_uncertain:
            raise BridgeError(409, 'Previous motion outcome is unknown; stop and inspect before another command')
        if self._motion_busy:
            raise BridgeError(409, 'Another motion is active; stop remains available')
        if self._recovery_required and operation != 'recover_arms':
            raise BridgeError(409, 'Recovery required: stop all arms, then recover_arms before re-detecting')
        if operation == 'create_plan':
            if any(p['state'] in {'picked', 'verified'} for p in self.pixels.plans.values()):
                raise BridgeError(409, 'Finish or recover the held-object plan before detecting another grasp')
            return Reply(payload=self.pixels.create(**payload))
        if operation == 'refine_plan':
            return Reply(payload=self.pixels.refine(**payload))
        if operation == 'verify_grasp':
            self._check_health([o['arm_id'] for o in payload['observations']],
                               expect_grasp=all(o['success'] for o in payload['observations']))
            return Reply(payload=self.pixels.verify(**payload))
        plan = None
        grouped = operation in {'execute_plan', 'reset_arms', 'move_arms', 'recover_arms'}
        if operation == 'execute_plan':
            plan = self.pixels.get(payload['plan_id'])
            stage = payload['stage']
            allowed = {'approach': ('detected',), 'pick': ('detected', 'approached'), 'place': ('verified',)}
            if plan['state'] not in allowed[stage]:
                raise BridgeError(409, f"Cannot {stage} a plan in state {plan['state']}")
            self._check_health([t['arm_id'] for t in plan['targets']], expect_grasp=stage == 'place')
            phases = {'approach': ['approach'],
                      'pick': ['open', 'approach', 'descend', 'close', 'lift'],
                      'place': ['transfer', 'descend', 'open', 'retreat']}[stage]
            payload = dict(payload, targets=plan['targets'], phases=phases, coordinated=True)
            arm_ids = [t['arm_id'] for t in plan['targets']]
            for other in self.pixels.plans.values():
                if other is not plan and other['state'] != 'placed':
                    other['state'] = 'invalid'
            self.pixels.captures.clear()
            plan['state'] = 'executing'
        else:
            arm_ids = ([m['arm_id'] for m in payload['moves']] if operation == 'move_arms'
                       else [a.id for a in self.config.arms])
            if not grouped:
                arm_ids = [resource_id]
            if operation == 'recover_arms' and not self._all_stopped:
                raise BridgeError(409, 'Stop all arms successfully before recovery')
            self._check_health(arm_ids, recovering=operation == 'recover_arms')
            self.pixels.invalidate()
            if grouped:
                payload = dict(payload, arm_ids=arm_ids, coordinated=True)
                if operation == 'recover_arms':
                    payload['phases'] = ['secure_or_support_payload', 'release', 'retreat', 'home']
        generation = self._generation
        self._motion_busy = True
        self._all_stopped = False
        completed_successfully = False
        try:
            reply = await self._command(operation, resource_id, payload, timeout)
            if generation != self._generation:
                return Reply.from_error(BridgeError(409, 'Motion interrupted; outcome requires inspection', outcome='unknown'))
            self._motion_uncertain = reply.status >= 500 or reply.payload.get('outcome') == 'unknown'
            if grouped and reply.status == 200 and reply.payload.get('success') is True:
                completed = reply.payload.get('completed_arm_ids')
                if (reply.payload.get('coordinated') is not True or not isinstance(completed, list)
                        or any(not isinstance(arm, str) for arm in completed)
                        or sorted(completed) != sorted(arm_ids)):
                    self._motion_uncertain = True
                    return Reply.from_error(BridgeError(502, 'Driver did not confirm coordinated completion for every arm', outcome='unknown'))
            if reply.status == 200 and reply.payload.get('success') is True and operation != 'recover_arms':
                self._check_health(arm_ids)
            completed_successfully = reply.status == 200 and reply.payload.get('success') is True
            if operation == 'recover_arms' and completed_successfully:
                self._recovery_required = False
                reply.payload['next_actions'] = ['get_robot_state', 'detect_targets']
            if plan is not None and completed_successfully:
                plan['state'] = {'approach': 'approached', 'pick': 'picked', 'place': 'placed'}[stage]
                plan['motion_stamp_ns'] = self.get_clock().now().nanoseconds
                reply.payload.update(self.pixels.public(plan))
            return reply
        finally:
            self._motion_busy = False
            if not completed_successfully:
                self._recovery_required = True
            if plan is not None and plan['state'] == 'executing':
                plan['state'] = 'invalid'

    def _check_health(self, arm_ids, *, recovering=False, expect_grasp=False):
        state = self.store.state(self.get_clock().now().nanoseconds)
        failures = []
        for arm in state['arms']:
            if arm['id'] not in arm_ids:
                continue
            for component, fault in [('arm', arm.get('fault')), ('gripper', arm['gripper'].get('fault'))]:
                if fault and (not recovering or not fault['recoverable']):
                    failures.append(dict(arm_id=arm['id'], component=component, **fault))
            if expect_grasp and arm['gripper'].get('object_detected') is False:
                failures.append(dict(arm_id=arm['id'], component='gripper', code='no_object',
                                     message='Gripper reports no held object', recoverable=True))
        if failures:
            self._recovery_required = True
            self.pixels.invalidate()
            error = BridgeError(409, 'Arm or gripper failure; recovery or operator intervention required')
            error.payload.update(failures=failures, recoverable=all(f['recoverable'] for f in failures))
            raise error

    def close_pending(self):
        with self._lock:
            self._closing = True
            for pending in list(self._pending):
                if pending.upstream is not None:
                    self.driver.remove_pending_request(pending.upstream)
                    pending.upstream.cancel()
                self._complete(pending, Reply.from_error(BridgeError(503, "Robot bridge is shutting down")))
