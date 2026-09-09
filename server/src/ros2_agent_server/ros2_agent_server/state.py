"""Thread-safe topic snapshots. No HTTP or ROS2 transport dependency."""

from dataclasses import dataclass
import io
import threading
import time

from PIL import Image

from .diagnostics import require
from .models import ArmState
from .protocol import BridgeError


@dataclass(frozen=True)
class CameraFrame:
    data: bytes
    frame_id: str
    stamp_ns: int
    sequence: int


class StateStore:
    def __init__(self, config, *, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self._lock = threading.RLock()
        self._arms = {}
        self._cameras = {}
        self._camera_history = {}
        self._arm_sequence = 0
        self._arm_history = {}

    def update_arm(self, arm_id, payload, stamp_ns=None):
        if arm_id not in {arm.id for arm in self.config.arms}:
            raise ValueError(f"Unknown arm: {arm_id}")
        state = ArmState.model_validate(payload)
        if state.flange_pose.frame_id not in self.config.frame_ids:
            raise ValueError("Unknown state coordinate frame")
        with self._lock:
            previous = self._arms.get(arm_id)
            if stamp_ns is not None:
                require(type(stamp_ns) is int and stamp_ns > 0, "state.stamp_ns", "positive integer", stamp_ns)
                if previous and previous[2] is not None and stamp_ns <= previous[2]:
                    raise ValueError(f"state.stamp_ns: arm={arm_id}; measurement did not advance: {stamp_ns} <= {previous[2]}")
            self._arm_sequence += 1
            self._arms[arm_id] = (state.model_dump(), self.clock(), stamp_ns, self._arm_sequence)
            if stamp_ns is not None:
                history = self._arm_history.setdefault(arm_id, [])
                history.append((stamp_ns, state.moving))
                del history[:-512]

    def stationary_interval(self, arm_id, start_ns, end_ns):
        with self._lock:
            history = self._arm_history.get(arm_id, [])
            before = [i for i,(stamp,_) in enumerate(history) if stamp <= start_ns]
            after = [i for i,(stamp,_) in enumerate(history) if stamp >= end_ns]
            if not before or not after:
                return False
            window = history[before[-1]:after[0]+1]
            gap = self.config.server.telemetry_max_gap*1e9
            return (bool(window) and not any(moving for _,moving in window)
                    and all(b[0]-a[0] <= gap for a,b in zip(window,window[1:])))

    def arm_revisions(self, arm_ids):
        with self._lock:
            return {arm: self._arms[arm][3] if arm in self._arms else 0 for arm in arm_ids}

    def measured_after(self, arm_ids, stamp_ns, revisions):
        with self._lock:
            return all(arm in self._arms and self._arms[arm][3] > revisions[arm]
                       and self._arms[arm][2] is not None and self._arms[arm][2] >= stamp_ns
                       for arm in arm_ids)

    def clear_cameras(self, camera_id=None):
        with self._lock:
            if camera_id is None:
                self._cameras.clear()
                self._camera_history.clear()
            else:
                self._cameras.pop(camera_id, None)
                self._camera_history.pop(camera_id, None)

    def reset(self):
        with self._lock:
            self._arms.clear()
            self._arm_history.clear()
            self.clear_cameras()

    def state(self, stamp_ns):
        now = self.clock()
        with self._lock:
            missing = [arm.id for arm in self.config.arms if arm.id not in self._arms
                       or now - self._arms[arm.id][1] > self.config.server.state_max_age
                       or (self._arms[arm.id][2] is not None and
                           not -round(self.config.server.future_skew_tolerance_sec*1e9) <= stamp_ns-self._arms[arm.id][2] <= self.config.server.state_max_age*1e9)]
            if missing:
                raise BridgeError(503, f"Missing or stale arm state: {', '.join(missing)}")
            # Pydantic dicts contain mutable lists; do not return the cache itself.
            import copy
            arms = [dict(id=arm.id, measurement_stamp_ns=self._arms[arm.id][2],
                         **copy.deepcopy(self._arms[arm.id][0])) for arm in self.config.arms]
        return {"arms": arms, "frames": self.config.frame_ids,
                "stamp": {"sec": stamp_ns // 1_000_000_000, "nanosec": stamp_ns % 1_000_000_000}}

    def update_camera(self, camera_id, data, frame_id, stamp_ns):
        camera = next((c for c in self.config.cameras if c.id == camera_id), None)
        require(camera is not None, "camera_id", "configured camera", camera_id)
        require(camera.optical_frame == frame_id, f"{camera_id}.frame_id", camera.optical_frame, frame_id)
        require(type(stamp_ns) is int and stamp_ns > 0, f"{camera_id}.stamp_ns", "positive integer", stamp_ns)
        with Image.open(io.BytesIO(data)) as image:
            if image.format != "JPEG":
                raise ValueError("Only JPEG CompressedImage messages are supported")
            image.verify()
        with self._lock:
            previous = self._cameras.get(camera_id)
            history = self._camera_history.setdefault(camera_id, [])
            if any(frame.stamp_ns == stamp_ns for frame in history):
                return False
            frame = CameraFrame(bytes(data), frame_id, stamp_ns, (previous.sequence if previous else 0) + 1)
            self._cameras[camera_id] = frame
            history.append(frame)
            del history[:-self.config.server.camera_buffer_size]
        return True

    def sequence(self, camera_id):
        with self._lock:
            frame = self._cameras.get(camera_id)
            return frame.sequence if frame else 0

    def fresh_cameras(self, camera_id, after_sequence, after_stamp_ns):
        with self._lock:
            return sorted((f for f in self._camera_history.get(camera_id, [])
                           if f.sequence > after_sequence and f.stamp_ns >= after_stamp_ns),
                          key=lambda f: (f.stamp_ns, f.sequence))

    def fresh_camera(self, camera_id, after_sequence, after_stamp_ns):
        with self._lock:
            frame = self._cameras.get(camera_id)
            if frame and frame.sequence > after_sequence and frame.stamp_ns >= after_stamp_ns:
                return frame
        return None
