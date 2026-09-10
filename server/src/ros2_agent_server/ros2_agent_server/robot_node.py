"""ROS2 topic aggregation and asynchronous forwarding to a robot driver."""

from dataclasses import dataclass, field
import json
import math
import threading
import time
import uuid
import re

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
from .diagnostics import event, require
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
    observation: bool = False
    request_id: str = ""
    started: float = field(default_factory=time.monotonic)
    reason: dict = field(default_factory=dict)
    check: object = None


class RobotBridgeNode(Node):
    def __init__(self, config, **kwargs):
        super().__init__("robot_bridge", **config.server.node_options(kwargs))
        self.config = config
        self.store = StateStore(config)
        self._last_motion_end_ns = 0
        self.pixels = PixelPlans(config)
        from .motion import MotionCompiler
        self.motion = MotionCompiler(config, self.pixels)
        self._active_arm_ids = []
        self._coupled_arm_ids = []
        self._calibration = {}
        self._calibration_keys = {}
        self._invalid_plane_cameras = set()
        self._last_ros_ns = self.get_clock().now().nanoseconds
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
            self._topic_subscriptions.append(self.create_subscription(
                CameraInfo, config.camera_info_topic(camera.id),
                lambda msg, camera_id=camera.id: self._camera_info(camera_id, msg),
                qos_profile_sensor_data, callback_group=self._group))
            self._topic_subscriptions.append(self.create_subscription(
                Image, config.camera_depth_topic(camera.id),
                lambda msg, camera_id=camera.id: self._depth_image(camera_id, msg),
                qos_profile_sensor_data, callback_group=self._group))
        # Wall-time deadlines must also expire when simulated ROS time is paused.
        self._timer = self.create_timer(
            0.02, self._expire, callback_group=self._group,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _arm_state(self, arm_id, message):
        # HARDWARE INTEGRATION: publish measured JSON through standard String.
        # See server/docs/telemetry.md for fields, timestamps and unavailable sensors.
        try:
            payload = json.loads(message.data)
            require(isinstance(payload, dict), "state", "JSON object", type(payload).__name__)
            payload = dict(payload)
            stamp = payload.pop("stamp_ns", None)
            now = self.get_clock().now().nanoseconds
            latest = now + round(self.config.server.future_skew_tolerance_sec*1e9)
            require(type(stamp) is int and 0 < stamp <= latest, "state.stamp_ns", f"(0, {latest}]", stamp)
            self.store.update_arm(arm_id, payload, stamp)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid state for {arm_id}: {exc}", throttle_duration_sec=1.)

    def _invalidate_geometry(self):
        self._recovery_required = self._recovery_required or self._motion_busy or any(
            p['state'] in {'picked', 'verified', 'executing'} for p in self.pixels.plans.values())
        if self._motion_busy:
            self._motion_uncertain = True
            self._generation += 1
        self.pixels.invalidate()

    def _camera_info(self, camera_id, message):
        from .numerics import calibration_key, same_calibration
        camera = next(c for c in self.config.cameras if c.id == camera_id)
        key = calibration_key(message)
        with self._lock:
            previous = self._calibration_keys.get(camera_id)
            changed = previous is None or not same_calibration(previous, key, camera)
            if previous is not None and changed:
                if next(c for c in self.config.cameras if c.id == camera_id).projection == 'plane':
                    self._invalid_plane_cameras.add(camera_id)
                self.store.clear_cameras(camera_id)
                self._depth.pop(camera_id, None)
                self._invalidate_geometry()
                for pending in list(self._pending):
                    if pending.camera and pending.camera[0] == camera_id:
                        self._complete(pending, Reply.from_error(BridgeError(409, f"Calibration changed for {camera_id}; reacquire",
                                                                            code="calibration_changed")))
                event(self.get_logger(), "calibration_changed", level="warning", camera=camera_id)
            if changed:
                event(self.get_logger(), "calibration_received", camera=camera_id,
                      frame=message.header.frame_id, width=message.width, height=message.height)
                # Keep the accepted baseline: repeated sub-tolerance updates must
                # not hide cumulative calibration drift.
                self._calibration_keys[camera_id] = key
                self._calibration[camera_id] = message

    def _image(self, camera_id, message):
        try:
            if "jpeg" not in message.format.lower():
                raise ValueError("Expected a JPEG CompressedImage")
            stamp = message.header.stamp
            accepted = self.store.update_camera(camera_id, bytes(message.data), message.header.frame_id,
                                                stamp.sec * 1_000_000_000 + stamp.nanosec)
            event(self.get_logger(), "rgb_received", camera=camera_id, accepted=accepted,
                  stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec, frame=message.header.frame_id)
        except (TypeError, ValueError, OSError) as exc:
            self.get_logger().warning(f"Ignoring invalid image for {camera_id}: {exc}")
            return
        self._try_cameras()

    def _depth_image(self, camera_id, message):
        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        cache = self._depth.setdefault(camera_id, {})
        cache[stamp] = message
        while len(cache) > self.config.server.camera_buffer_size:
            del cache[next(iter(cache))]
        event(self.get_logger(), "depth_received", camera=camera_id, stamp_ns=stamp,
              frame=message.header.frame_id, encoding=message.encoding)
        self._try_cameras()

    def _pose_at(self, frame_id, stamp_ns):
        try:
            tf = self.tf_buffer.lookup_transform(self.config.world_frame, frame_id, Time(nanoseconds=stamp_ns)).transform
        except TransformException as exc:
            raise TransformException(f"{self.config.world_frame} <- {frame_id} at {stamp_ns}: {exc}") from exc
        return dict(frame_id=self.config.world_frame,
                    position=[tf.translation.x, tf.translation.y, tf.translation.z],
                    orientation=[tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])

    def _waiting(self, pending, code, **details):
        reason = dict(code=code, **details)
        # Timer retries do not flood logs with the same wait condition.
        if pending.reason.get('code') != code:
            event(self.get_logger(), "request_wait", request_id=pending.request_id, **reason)
        pending.reason = reason

    def _try_cameras(self):
        with self._lock:
            now = self.get_clock().now().nanoseconds
            latest = now + round(self.config.server.future_skew_tolerance_sec*1e9)
            for pending in list(self._pending):
                if pending.camera is None or time.monotonic() >= pending.deadline:
                    continue
                camera_id, sequence, after_stamp = pending.camera
                frames = [f for f in self.store.fresh_cameras(*pending.camera)
                          if f.stamp_ns <= latest and now-f.stamp_ns <= self.config.server.camera_max_age*1e9]
                if not frames:
                    current = self.store._cameras.get(camera_id)
                    self._waiting(pending, "fresh_rgb_unavailable", camera=camera_id,
                                  after_stamp_ns=after_stamp, latest_stamp_ns=current.stamp_ns if current else None)
                    continue
                if not pending.capture:
                    frame = frames[-1]
                    if pending.observation:
                        self._complete(pending, Reply(payload=self.pixels.observe(camera_id, frame)))
                    else:
                        self._complete(pending, Reply(payload={"frame_id": frame.frame_id, "stamp_ns": frame.stamp_ns},
                                                      data=frame.data, content_type="image/jpeg"))
                    continue
                camera = next(c for c in self.config.cameras if c.id == camera_id)
                if camera.projection == 'plane':
                    if camera_id in self._invalid_plane_cameras:
                        self._complete(pending, Reply.from_error(BridgeError(409,
                            f"Plane calibration invalidated for {camera_id}; recalibrate and restart the bridge",
                            code="plane_calibration_invalidated")))
                        continue
                    try:
                        metadata = self.pixels.capture_plane(camera_id, frames[-1])
                    except ValueError as exc:
                        self._complete(pending, Reply.from_error(BridgeError(422, str(exc), code="capture_geometry_invalid")))
                    else:
                        self._complete(pending, Reply(payload=metadata))
                    continue
                info = self._calibration.get(camera_id)
                if info is None:
                    self._waiting(pending, "camera_info_unavailable", camera=camera_id)
                    continue
                camera = next(c for c in self.config.cameras if c.id == camera_id)
                depths = self._depth.get(camera_id, {})
                tolerance = round(camera.sync_tolerance_sec*1e9)
                # Select the closest AVAILABLE pair; ties use earliest RGB/depth stamps.
                # Both images must be acquired after the request; all queues are bounded.
                pairs = sorted((abs(stamp-f.stamp_ns), f.stamp_ns, stamp, f.sequence, f, depth)
                               for f in frames for stamp, depth in depths.items()
                               if after_stamp <= stamp <= latest and now-stamp <= self.config.server.camera_max_age*1e9
                               and abs(stamp-f.stamp_ns) <= tolerance)
                if not pairs:
                    self._waiting(pending, "depth_unavailable" if not depths else "depth_sync_unavailable",
                                  camera=camera_id, rgb_stamp_ns=frames[-1].stamp_ns,
                                  latest_depth_stamp_ns=max(depths, default=None), tolerance_ns=tolerance,
                                  depth_cache_count=len(depths))
                    continue
                for delta, _, _, _, frame, depth in pairs:
                    if time.monotonic() >= pending.deadline:
                        break
                    if delta and camera.mount == 'flange':
                        try:
                            state = self.store.state(now)
                            arm = next(a for a in state['arms'] if a['id'] == camera.arm_id)
                            if (self._motion_busy or arm['moving']
                                    or min(frame.stamp_ns, depth.header.stamp.sec*1_000_000_000+depth.header.stamp.nanosec) <= self._last_motion_end_ns
                                    or not self.store.stationary_interval(camera.arm_id,
                                        min(frame.stamp_ns, depth.header.stamp.sec*1_000_000_000+depth.header.stamp.nanosec),
                                        max(frame.stamp_ns, depth.header.stamp.sec*1_000_000_000+depth.header.stamp.nanosec))):
                                self._waiting(pending, "wrist_not_stationary", camera=camera_id, arm_id=camera.arm_id)
                                continue
                        except BridgeError as exc:
                            self._waiting(pending, "wrist_state_unavailable", camera=camera_id, detail=str(exc))
                            continue
                    try:
                        camera_pose = self._pose_at(frame.frame_id, frame.stamp_ns)
                        flange_pose = (self._pose_at(camera.parent_frame, frame.stamp_ns)
                                       if camera.mount == 'flange' else None)
                        metadata = self.pixels.capture(camera_id, frame, info, depth, camera_pose, flange_pose)
                    except TransformException as exc:
                        self._waiting(pending, "capture_tf_unavailable", camera=camera_id,
                                      stamp_ns=frame.stamp_ns, detail=str(exc))
                        continue
                    except ValueError as exc:
                        self._complete(pending, Reply.from_error(BridgeError(422, str(exc), code="capture_geometry_invalid")))
                        break
                    self._complete(pending, Reply(payload=metadata))
                    break

    def _complete(self, pending, reply):
        with self._lock:
            if pending not in self._pending:
                return
            if reply.status < 400 and time.monotonic() >= pending.deadline:
                reply = Reply.from_error(BridgeError(504, "Response arrived after deadline",
                    code="capture_timeout" if pending.camera else "request_timeout",
                    outcome=None if pending.camera else "unknown", details=pending.reason))
            self._pending.remove(pending)
            if pending.upstream is not None and not pending.upstream.done():
                self.driver.remove_pending_request(pending.upstream)
                pending.upstream.cancel()
            event(self.get_logger(), "request_complete", level="warning" if reply.status >= 400 else "debug",
                  request_id=pending.request_id, status=reply.status,
                  elapsed_ms=round((time.monotonic()-pending.started)*1000, 3),
                  reason=pending.reason, capture_id=reply.payload.get("capture_id"))
            if not pending.future.done():
                pending.future.set_result(reply)

    def _expire(self):
        now = self.get_clock().now().nanoseconds
        if now < self._last_ros_ns:
            with self._lock:
                self._invalidate_geometry()
                self.store.reset()
                self._depth.clear()
                self.tf_buffer.clear()
                for pending in list(self._pending):
                    self._complete(pending, Reply.from_error(BridgeError(409, "ROS clock moved backwards; reacquire state and images",
                                                                        code="clock_reset", outcome="unknown")))
            event(self.get_logger(), "clock_reset", level="warning", previous_ns=self._last_ros_ns, now_ns=now)
        self._last_ros_ns = now
        self._try_cameras()
        with self._lock:
            for pending in list(self._pending):
                if time.monotonic() < pending.deadline and pending.check is not None and pending.check():
                    self._complete(pending, Reply())
                    continue
                if time.monotonic() >= pending.deadline:
                    if pending.upstream is not None:
                        self.driver.remove_pending_request(pending.upstream)
                        pending.upstream.cancel()
                    self._complete(pending, Reply.from_error(BridgeError(
                        504, "Fresh image or configured projection inputs unavailable" if pending.camera else "Driver command timed out",
                        outcome=None if pending.camera else "unknown", code="capture_timeout" if pending.camera else "request_timeout",
                        details=pending.reason)))

    async def _camera(self, camera_id, timeout, *, capture=False, observation=False, request_id=""):
        with self._lock:
            if self._closing:
                raise BridgeError(503, "Robot bridge is shutting down")
            pending = Pending(
                Future(executor=self.executor), time.monotonic() + timeout,
                camera=(camera_id, self.store.sequence(camera_id), self.get_clock().now().nanoseconds),
                capture=capture, observation=observation, request_id=request_id)
            self._pending.append(pending)
        event(self.get_logger(), "camera_wait_started", request_id=request_id, camera=camera_id,
              mode="capture" if capture else "observation" if observation else "image",
              after_stamp_ns=pending.camera[2], sequence=pending.camera[1], timeout_sec=timeout)
        return await pending.future

    async def _wait_state(self, arm_ids, deadline, request_id, generation):
        stamp = self.get_clock().now().nanoseconds
        revisions = self.store.arm_revisions(arm_ids)
        stable = [None, None, None]
        def settled():
            if generation != self._generation:
                return True
            if not self.store.measured_after(arm_ids, stamp, revisions):
                return False
            try:
                state = self.store.state(self.get_clock().now().nanoseconds)
            except BridgeError:
                stable[:] = [None, None, None]
                return False
            arms = [a for a in state['arms'] if a['id'] in arm_ids]
            if any(a.get('fault') or a['gripper'].get('fault') for a in arms):
                return True  # The health check reports the actual fault.
            if any(a['moving'] for a in arms):
                stable[:] = [None, None, None]
                return False
            now = time.monotonic()
            if stable[0] is None:
                stable[:] = [now, self.store.arm_revisions(arm_ids), {a['id']:a['measurement_stamp_ns'] for a in arms}]
            if self.config.server.settling_dwell and any(not self.store.stationary_interval(
                    a['id'], stable[2][a['id']], a['measurement_stamp_ns']) for a in arms):
                stable[:] = [None, None, None]
                return False
            return (now-stable[0] >= self.config.server.settling_dwell and
                    (self.config.server.settling_dwell == 0 or
                     all(self.store.arm_revisions(arm_ids)[a] > stable[1][a] for a in arm_ids)))
        pending = Pending(Future(executor=self.executor), min(deadline, time.monotonic()+self.config.server.state_completion_timeout),
                          request_id=request_id, reason=dict(code="post_command_state", arm_ids=arm_ids, after_stamp_ns=stamp),
                          check=settled)
        with self._lock:
            self._pending.append(pending)
        reply = await pending.future
        if reply.status != 200:
            raise BridgeError(reply.status, "No new measured state after driver completion", outcome="unknown",
                              code="post_command_state_timeout", details=pending.reason)

    async def _command(self, operation, resource_id, payload, timeout, request_id=""):
        # HARDWARE INTEGRATION: implement the driver service, not this common guard.
        # TOOL EXTENSION: classify new motions in _workflow; see server/docs/primitive-adapter.md#adding-and-exposing-tools.
        if timeout <= 0:
            raise BridgeError(504, "Driver deadline already expired", outcome="not_started")
        event(self.get_logger(), "driver_send", request_id=request_id, operation=operation, resource=resource_id)
        with self._lock:
            if self._closing:
                raise BridgeError(503, "Robot bridge is shutting down")
            if not self.driver.service_is_ready():
                raise BridgeError(503, "Robot driver service is unavailable")
            request = RobotRequest.Request(
                request_id=request_id, operation=operation, resource_id=resource_id,
                payload_json=json.dumps(payload, allow_nan=False), timeout_sec=timeout,
                deadline_ns=self.get_clock().now().nanoseconds+round(timeout*1e9))
            upstream = self.driver.call_async(request)
            pending = Pending(Future(executor=self.executor), time.monotonic() + timeout, upstream=upstream, request_id=request_id)
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
        trace = request.request_id if re.fullmatch(r"[A-Za-z0-9_-]{1,64}", request.request_id) else uuid.uuid4().hex
        event(self.get_logger(), "bridge_received", request_id=trace, operation=request.operation, resource=request.resource_id)
        admitted = None
        try:
            if not math.isfinite(request.timeout_sec) or not 0 < request.timeout_sec <= 15000:
                raise BridgeError(422, "timeout_sec must be finite and in (0, 15000]")
            budget = request.timeout_sec
            if request.deadline_ns:
                budget = min(budget, (request.deadline_ns-self.get_clock().now().nanoseconds)/1e9)
            if budget <= 0:
                raise BridgeError(504, "Request expired before execution", code="request_expired", outcome="not_started")
            admitted = time.monotonic()+budget
            try:
                payload = json.loads(request.payload_json)
            except ValueError as exc:
                raise BridgeError(422, "Invalid JSON request payload") from exc
            payload = validate_request(self.config, request.operation, request.resource_id, payload)
            budget = admitted-time.monotonic()
            if budget <= 0:
                raise BridgeError(504, "Request expired during validation", code="request_expired", outcome="not_started")
            if request.operation == "state":
                state = self.store.state(self.get_clock().now().nanoseconds)
                state.update(recovery_required=self._recovery_required, motion_outcome_unknown=self._motion_uncertain)
                reply = Reply(payload=state)
            elif request.operation in {"camera", "capture", "observation"}:
                reply = await self._camera(request.resource_id, min(budget, self.config.server.camera_timeout),
                                           capture=request.operation == "capture", observation=request.operation == "observation", request_id=trace)
            else:
                reply = await self._workflow(request.operation, request.resource_id, payload, budget, request_id=trace)
        except BridgeError as exc:
            reply = Reply.from_error(exc)
            event(self.get_logger(), "bridge_rejected", level="warning", request_id=trace,
                  status=exc.status, detail=str(exc))
        except ValueError as exc:
            reply = Reply.from_error(BridgeError(422, str(exc), code="invalid_input"))
            event(self.get_logger(), "bridge_rejected", level="warning", request_id=trace,
                  status=422, detail=str(exc))
        except Exception as exc:
            self.get_logger().error(f"Request failed: {exc}")
            reply = Reply.from_error(BridgeError(500, "Robot bridge request failed", outcome="unknown"))
        if reply.status < 400 and admitted is not None and time.monotonic() >= admitted:
            if request.operation in {'execute_plan', 'recover_arms', 'stop', 'move', 'gripper'}:
                self._motion_uncertain = self._recovery_required = True
                self._all_stopped = False
                self.pixels.invalidate()
            reply = Reply.from_error(BridgeError(504, "Request completed after deadline", code="request_timeout", outcome="unknown"))
        event(self.get_logger(), "bridge_reply", request_id=trace, status=reply.status)
        return reply.to_ros(response)

    async def _workflow(self, operation, resource_id, payload, timeout, request_id=""):
        # TOOL EXTENSION: read-only operations need an explicit branch before motion admission.
        deadline = time.monotonic()+timeout
        event(self.get_logger(), "workflow_enter", request_id=request_id, operation=operation,
              generation=self._generation, busy=self._motion_busy, recovery=self._recovery_required)
        if operation == 'stop':
            all_ids = [a.id for a in self.config.arms]
            requested = payload['arm_ids']
            actual = set(requested)
            groups = [getattr(self, '_active_arm_ids', []), getattr(self, '_coupled_arm_ids', [])] + [
                [t['arm_id'] for t in p['targets']] for p in self.pixels.plans.values()
                if p['state'] in {'picked', 'verified', 'executing'}]
            for group in groups:
                if len(group) > 1 and actual.intersection(group):
                    actual.update(group)
            actual = [a for a in all_ids if a in actual]
            driver_payload = dict(arm_ids=actual)
            self._generation += 1
            self._recovery_required = self._recovery_required or self._motion_busy or any(
                p['state'] in {'picked', 'verified', 'executing'} for p in self.pixels.plans.values())
            self.pixels.invalidate()
            was_uncertain = self._motion_uncertain or self._motion_busy
            self._motion_uncertain = True
            self._all_stopped = False
            generation = self._generation
            reply = await self._command(operation, resource_id, driver_payload, max(0., deadline-time.monotonic()), request_id=request_id)
            if generation != self._generation:
                return Reply.from_error(BridgeError(409, 'Stop superseded by another stop', outcome='unknown'))
            if reply.status == 200 and reply.payload.get('success') is True:
                if reply.payload.get('coordinated') is not True or sorted(reply.payload.get('completed_arm_ids',[])) != sorted(actual):
                    reply = Reply.from_error(BridgeError(502,'Stop did not confirm all affected arms',outcome='unknown'))
            if reply.status == 200 and reply.payload.get('success') is True:
                if set(actual) == set(all_ids):
                    self._motion_uncertain = False
                    self._all_stopped = True
                else:
                    self._motion_uncertain = was_uncertain
            else:
                self._motion_uncertain = True
            reply.payload.update(requested_arm_ids=requested, stopped_arm_ids=actual if reply.status == 200 and reply.payload.get('success') is True else [],
                                 affected_arm_ids=actual, plans_invalidated=True)
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
        primitive_steps = None
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
            _, primitive_steps = self.motion.compile(operation,resource_id,payload)
            plan['state'] = 'executing'
        else:
            arm_ids = [a.id for a in self.config.arms]
            if operation in {'move','gripper'}:
                from .motion import Move, Grip
                arm_ids = (Move if operation == 'move' else Grip).model_validate(payload).selected(self.config)
            if operation == 'recover_arms' and not self._all_stopped:
                raise BridgeError(409, 'Stop all arms successfully before recovery')
            home_requested = (operation == 'move' and any(
                t.get('kind') == 'named' and t.get('name') == 'home' for t in payload['targets']))
            self._check_health(arm_ids, recovering=operation == 'recover_arms', expect_released=home_requested)
            if operation != 'recover_arms' and any(
                    p['state'] in {'picked', 'verified'} for p in self.pixels.plans.values()):
                self._recovery_required = True
                self.pixels.invalidate()
                raise BridgeError(409, 'Manual motion cannot discard a held-object plan; stop and recover')
            if operation != 'recover_arms':
                _, primitive_steps = self.motion.compile(operation,resource_id,payload)
            self.pixels.invalidate()
            if operation == 'recover_arms':
                payload = dict(payload, arm_ids=arm_ids)
        if primitive_steps is None:
            _, primitive_steps = self.motion.compile(operation,resource_id,payload)
        generation = self._generation
        self._active_arm_ids = list(arm_ids)
        if operation == 'execute_plan' and len(arm_ids)>1:
            self._coupled_arm_ids = list(arm_ids)
        self._motion_busy = True
        self._all_stopped = False
        completed_successfully = False
        try:
            remaining = deadline-time.monotonic()
            reserve = min(self.config.server.state_completion_timeout+self.config.server.bridge_processing_margin,
                          remaining*.25)
            if remaining <= 0:
                raise BridgeError(504, "Motion expired before driver dispatch", code="request_expired", outcome="not_started")
            from .sequencer import execute
            reply = await execute(self,primitive_steps,arm_ids,operation == 'execute_plan' and len(arm_ids)>1,
                                  deadline-reserve,request_id,generation)
            if generation != self._generation:
                return Reply.from_error(BridgeError(409, 'Motion interrupted; outcome requires inspection', outcome='unknown'))
            self._motion_uncertain = reply.status >= 500 or reply.payload.get('outcome') == 'unknown'
            if reply.status == 200 and reply.payload.get('success') is True:
                completed = reply.payload.get('completed_arm_ids')
                if (reply.payload.get('coordinated') is not True or not isinstance(completed, list)
                        or any(not isinstance(arm, str) for arm in completed)
                        or sorted(completed) != sorted(arm_ids)):
                    self._motion_uncertain = True
                    return Reply.from_error(BridgeError(502, 'Driver did not confirm coordinated completion for every arm', outcome='unknown'))
            if reply.status == 200 and reply.payload.get('success') is True:
                await self._wait_state(arm_ids, deadline, request_id, generation)
                if generation != self._generation:
                    raise BridgeError(409, "Motion superseded while waiting for measured state", outcome="unknown")
                self._check_health(arm_ids, expect_released=(operation == 'recover_arms' or
                    operation == 'execute_plan' and payload['stage'] == 'place'))
            completed_successfully = reply.status == 200 and reply.payload.get('success') is True
            if completed_successfully and (operation == 'recover_arms' or operation == 'execute_plan' and stage == 'place'):
                self._coupled_arm_ids = []
            if operation == 'recover_arms' and completed_successfully:
                self._recovery_required = False
                reply.payload['next_actions'] = ['get_robot_state', 'detect_targets']
            if plan is not None and completed_successfully:
                plan['state'] = {'approach': 'approached', 'pick': 'picked', 'place': 'placed'}[stage]
                plan['motion_stamp_ns'] = self.get_clock().now().nanoseconds
                reply.payload.update(self.pixels.public(plan))
            return reply
        except BridgeError as exc:
            if generation == self._generation and exc.payload.get('outcome') == 'unknown':
                self._motion_uncertain = True
            raise
        finally:
            event(self.get_logger(), "workflow_finished", request_id=request_id, operation=operation,
                  success=completed_successfully, generation=self._generation)
            self._motion_busy = False
            self._active_arm_ids = []
            self._last_motion_end_ns = self.get_clock().now().nanoseconds
            if not completed_successfully:
                self._recovery_required = True
            if plan is not None and plan['state'] == 'executing':
                plan['state'] = 'invalid'

    def _check_health(self, arm_ids, *, recovering=False, expect_grasp=False, expect_released=False):
        state = self.store.state(self.get_clock().now().nanoseconds)
        failures = []
        for arm in state['arms']:
            if arm['id'] not in arm_ids:
                continue
            if arm['moving']:
                failures.append(dict(arm_id=arm['id'], component='arm', code='arm_moving',
                                     message='Arm state still reports motion', recoverable=True))
            for component, fault in [('arm', arm.get('fault')), ('gripper', arm['gripper'].get('fault'))]:
                if fault and (not recovering or not fault['recoverable']):
                    failures.append(dict(arm_id=arm['id'], component=component, **fault))
            if expect_grasp and arm['gripper'].get('object_detected') is False:
                failures.append(dict(arm_id=arm['id'], component='gripper', code='no_object',
                                     message='Gripper reports no held object', recoverable=True))
            if expect_released and arm['gripper'].get('object_detected') is True:
                failures.append(dict(arm_id=arm['id'], component='gripper', code='object_not_released',
                                     message='Gripper still reports a held object', recoverable=True))
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
