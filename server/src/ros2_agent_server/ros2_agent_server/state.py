"""Thread-safe topic snapshots. No HTTP or ROS2 transport dependency."""

from dataclasses import dataclass
import io
import threading
import time

from PIL import Image

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

    def update_arm(self, arm_id, payload):
        if arm_id not in {arm.id for arm in self.config.arms}:
            raise ValueError(f"Unknown arm: {arm_id}")
        state = ArmState.model_validate(payload)
        if state.flange_pose.frame_id not in self.config.frame_ids:
            raise ValueError("Unknown state coordinate frame")
        with self._lock:
            self._arms[arm_id] = (state.model_dump(), self.clock())

    def state(self, stamp_ns):
        now = self.clock()
        with self._lock:
            missing = [arm.id for arm in self.config.arms if arm.id not in self._arms
                       or now - self._arms[arm.id][1] > self.config.server.state_max_age]
            if missing:
                raise BridgeError(503, f"Missing or stale arm state: {', '.join(missing)}")
            # Pydantic dicts contain mutable lists; do not return the cache itself.
            import copy
            arms = [dict(id=arm.id, **copy.deepcopy(self._arms[arm.id][0])) for arm in self.config.arms]
        return {"arms": arms, "frames": self.config.frame_ids,
                "stamp": {"sec": stamp_ns // 1_000_000_000, "nanosec": stamp_ns % 1_000_000_000}}

    def update_camera(self, camera_id, data, frame_id, stamp_ns):
        camera = next((c for c in self.config.cameras if c.id == camera_id), None)
        if camera is None or camera.optical_frame != frame_id or stamp_ns <= 0:
            raise ValueError("Camera ID, optical frame or acquisition timestamp is invalid")
        with Image.open(io.BytesIO(data)) as image:
            if image.format != "JPEG":
                raise ValueError("Only JPEG CompressedImage messages are supported")
            image.verify()
        with self._lock:
            previous = self._cameras.get(camera_id)
            if previous and stamp_ns <= previous.stamp_ns:
                return False
            self._cameras[camera_id] = CameraFrame(
                bytes(data), frame_id, stamp_ns, (previous.sequence if previous else 0) + 1)
        return True

    def sequence(self, camera_id):
        with self._lock:
            frame = self._cameras.get(camera_id)
            return frame.sequence if frame else 0

    def fresh_camera(self, camera_id, after_sequence, after_stamp_ns):
        with self._lock:
            frame = self._cameras.get(camera_id)
            if frame and frame.sequence > after_sequence and frame.stamp_ns >= after_stamp_ns:
                return frame
        return None
