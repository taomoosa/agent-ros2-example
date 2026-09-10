"""Validated HTTP bodies and robot topology, independent of the agent package."""

import json
import math
from typing import Literal
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .ros_names import RosNames


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class Pose(Model):
    frame_id: str = Field(min_length=1)
    position: list[float] = Field(min_length=3, max_length=3)
    orientation: list[float] = Field(min_length=4, max_length=4)

    @field_validator("orientation")
    @classmethod
    def unit_quaternion(cls, value):
        if not math.isclose(sum(v * v for v in value), 1.0, abs_tol=1e-3):
            raise ValueError("orientation must be a unit quaternion in xyzw order")
        return value


class PlaneCalibration(Model):
    """Pixel-to-plane homography, calibrated for one fixed rectified image grid."""
    calibration_id: str = Field(min_length=1)
    image_width: int = Field(ge=2, le=65536)
    image_height: int = Field(ge=2, le=65536)
    homography: list[float] = Field(min_length=9, max_length=9)
    valid_region: list[float] = Field(min_length=4, max_length=4)
    plane_pose: Pose

    @model_validator(mode="after")
    def geometry(self):
        x0, y0, x1, y1 = self.valid_region
        if not (0 <= x0 < x1 < self.image_width and 0 <= y0 < y1 < self.image_height):
            raise ValueError("plane_calibration.valid_region must be [xmin,ymin,xmax,ymax] inside the calibrated image")
        scale = max(abs(v) for v in self.homography)
        if not scale:
            raise ValueError("plane_calibration.homography must be nonsingular")
        a,b,c,d,e,f,g,h,i = [v/scale for v in self.homography]
        determinant = a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)
        rows = math.hypot(a,b,c)*math.hypot(d,e,f)*math.hypot(g,h,i)
        if abs(determinant) <= 1e-12*rows:
            raise ValueError("plane_calibration.homography is singular or ill-conditioned")
        denominators = [g*x+h*y+i for x in (x0,x1) for y in (y0,y1)]
        if not (min(denominators) > 1e-9 or max(denominators) < -1e-9):
            raise ValueError("plane_calibration.homography horizon intersects or approaches valid_region")
        return self


class Stop(Model):
    arm_ids: list[str] | None = Field(default=None, min_length=1, max_length=2)
    all_arms: bool = False


class Fault(Model):
    code: str = Field(min_length=1)
    message: str = Field(min_length=1)
    recoverable: bool = False


class GripperState(Model):
    opening: float | None = Field(default=None, ge=0.0, le=1.0)
    fault: Fault | None = None
    object_detected: bool | None = None


class ArmState(Model):
    moving: bool
    flange_pose: Pose
    gripper: GripperState
    fault: Fault | None = None


class Arm(Model):
    id: str = Field(pattern=r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$")
    base_frame: str = Field(min_length=1)
    flange_frame: str = Field(min_length=1)


# HARDWARE INTEGRATION: declare input geometry here; see server/docs/integration.md.
class Camera(Model):
    id: str = Field(pattern=r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$")
    optical_frame: str = Field(min_length=1)
    mount: str
    parent_frame: str = Field(min_length=1)
    arm_id: str | None = None
    depth_frame: str | None = None
    camera_info_frame: str | None = None
    # An alias declares color-grid pixels AND color optical-axis Z, not raw depth.
    depth_geometry: Literal["color_optical_z"] = "color_optical_z"
    camera_info_mode: Literal["rectified_k", "ros_rectified"] = "rectified_k"
    sync_tolerance_sec: float = Field(default=0.01, ge=0.0, le=0.1)
    calibration_tolerance_px: float = Field(default=1e-6, ge=0.0, le=0.01)
    rectification_tolerance: float = Field(default=1e-9, ge=0.0, le=1e-6)
    depth_min_m: float = Field(default=0.05, gt=0.0)
    depth_max_m: float = Field(default=5.0, gt=0.0)
    depth_absolute_tolerance_m: float = Field(default=0.02, ge=0.0, le=0.1)
    depth_relative_tolerance: float = Field(default=0.01, ge=0.0, le=0.1)
    depth_min_support: float = Field(default=0.6, gt=0.5, le=1.0)
    projection: Literal["depth", "plane"] = "depth"
    plane_calibration: PlaneCalibration | None = None

    @model_validator(mode="after")
    def projection_contract(self):
        if self.depth_min_m >= self.depth_max_m:
            raise ValueError("depth_min_m must be less than depth_max_m")
        if self.projection == "plane":
            if self.mount != "world" or self.plane_calibration is None:
                raise ValueError("Plane projection requires a fixed world camera and plane_calibration")
        elif self.plane_calibration is not None:
            raise ValueError("plane_calibration requires projection='plane'; no automatic depth fallback")
        return self


class Settings(RosNames):
    hardware: dict = Field(default_factory=dict)
    motion_timeout: float = Field(default=120.0, gt=0.0, le=3600.0)
    settling_timeout: float = Field(default=15.0, gt=0.0, le=3600.0)
    bridge_processing_margin: float = Field(default=1.0, gt=0.0, le=3600.0)
    gripper_timeout: float = Field(default=15.0, gt=0.0, le=3600.0)
    stop_timeout: float = Field(default=5.0, gt=0.0, le=3600.0)
    request_timeout: float = Field(default=5.0, gt=0.0, le=3600.0)
    ros_response_margin: float = Field(default=2.0, gt=0.0, le=3600.0)
    http_response_margin: float = Field(default=3.0, gt=0.0, le=3600.0)
    er_timeout: float = Field(default=90.0, gt=0.0, le=3600.0)
    live_io_timeout: float = Field(default=10.0, gt=0.0, le=3600.0)
    cleanup_timeout: float = Field(default=3.0, gt=0.0, le=3600.0)
    model_idle_timeout: float = Field(default=120.0, gt=0.0, le=3600.0)
    capture_ttl: float = Field(default=300.0, gt=0.0, le=3600.0)
    plan_ttl: float = Field(default=600.0, gt=0.0, le=3600.0)
    settling_dwell: float = Field(default=0.2, ge=0.0, le=10.0)
    telemetry_max_gap: float = Field(default=0.25, gt=0.0, le=5.0)
    future_skew_tolerance_sec: float = Field(default=0.005, ge=0.0, le=0.1)
    state_max_age: float = Field(default=2.0, gt=0.0, le=60.0)
    camera_timeout: float = Field(default=5.0, gt=0.0, le=3600.0)
    camera_buffer_size: int = Field(default=32, ge=2, le=256)
    camera_max_age: float = Field(default=2.0, gt=0.0, le=10.0)
    state_completion_timeout: float = Field(default=5.0, gt=0.0, le=3600.0)


    def operation_timeout(self, operation, payload=None):
        payload = payload or {}
        if operation in {'camera', 'capture', 'observation'}:
            return self.camera_timeout
        if operation == 'stop':
            return self.stop_timeout
        if operation == 'move':
            execution = max(self.motion_timeout, payload.get('duration', 3.))
        elif operation in {'execute_plan', 'recover_arms'}:
            execution = self.motion_timeout
        elif operation == 'gripper':
            execution = self.gripper_timeout
        else:
            return self.request_timeout
        return execution + self.settling_timeout + self.state_completion_timeout + self.bridge_processing_margin

    @property
    def observation_timeout(self):
        return 2*(self.camera_timeout+self.ros_response_margin+self.http_response_margin)+self.live_io_timeout


class RobotConfig(Model):
    # robot_url is accepted to use exactly the same topology file as the agent.
    robot_url: str
    world_frame: str = Field(min_length=1)
    arms: list[Arm] = Field(min_length=1, max_length=2)
    cameras: list[Camera] = Field(min_length=1)
    server: Settings = Field(default_factory=Settings)

    @model_validator(mode="after")
    def topology(self):
        arms = {arm.id: arm for arm in self.arms}
        if len(arms) != len(self.arms):
            raise ValueError(f"Duplicate arm IDs: {[a.id for a in self.arms]}")
        if len({c.id for c in self.cameras}) != len(self.cameras):
            raise ValueError(f"Duplicate camera IDs: {[c.id for c in self.cameras]}")
        for camera in self.cameras:
            if camera.plane_calibration and camera.plane_calibration.plane_pose.frame_id != self.world_frame:
                raise ValueError(f"Camera {camera.id} plane_pose.frame_id must match world_frame {self.world_frame!r}")
            for field in ("depth_frame", "camera_info_frame"):
                value = getattr(camera, field)
                if value is not None and (not value.strip() or value.startswith("/")):
                    raise ValueError(f"Camera {camera.id} {field}: expected a nonempty relative frame; actual {value!r}")
            if camera.mount == "world":
                if camera.arm_id is not None:
                    raise ValueError(f"World camera {camera.id} arm_id: expected None; actual {camera.arm_id!r}")
                if camera.parent_frame != self.world_frame:
                    raise ValueError(f"Camera {camera.id} parent_frame: expected {self.world_frame!r}; actual {camera.parent_frame!r}")
            elif camera.mount == "flange":
                if camera.arm_id not in arms:
                    raise ValueError(f"Camera {camera.id} arm_id: unconfigured {camera.arm_id!r}")
                if camera.parent_frame != arms[camera.arm_id].flange_frame:
                    raise ValueError(f"Camera {camera.id} parent_frame: expected {arms[camera.arm_id].flange_frame!r}; actual {camera.parent_frame!r}")
            else:
                raise ValueError("Camera mount must be world or flange")
        if any(frame.startswith("/") or not frame.strip() for frame in self.frame_ids):
            raise ValueError("ROS2 frame IDs must be nonempty and relative")
        # Resource IDs become ROS topic segments; dots/hyphens are valid in
        # HTTP IDs but must be rejected here unless explicit remaps are added.
        import re
        if any(not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", item.id)
               for item in [*self.arms, *self.cameras]):
            raise ValueError("Server resource IDs must also be valid ROS2 topic segments")
        return self

    @property
    def frame_ids(self):
        return list(dict.fromkeys([
            self.world_frame,
            *(f for arm in self.arms for f in (arm.base_frame, arm.flange_frame)),
            *(camera.optical_frame for camera in self.cameras),
        ]))

    @property
    def request_service(self):
        return self.server.request_service

    def state_topic(self, arm_id):
        return self.server.state_topic(arm_id)

    def camera_topic(self, camera_id):
        return self.server.camera_topic(camera_id)

    def camera_info_topic(self, camera_id):
        return self.server.camera_info_topic(camera_id)

    def camera_depth_topic(self, camera_id):
        return self.server.camera_depth_topic(camera_id)

    @classmethod
    def load(cls, path):
        return cls.model_validate(json.loads(Path(path).read_text()))
