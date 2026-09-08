"""Validated HTTP bodies and robot topology, independent of the agent package."""

import json
import math
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


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


class MoveArm(Pose):
    duration: float = Field(default=3.0, ge=0.1, le=60.0)


class Gripper(Model):
    opening: float = Field(ge=0.0, le=1.0)


class Stop(Model):
    arm_id: str | None = None


class Fault(Model):
    code: str = Field(min_length=1)
    message: str = Field(min_length=1)
    recoverable: bool = False


class GripperState(Gripper):
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


class Camera(Model):
    id: str = Field(pattern=r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$")
    optical_frame: str = Field(min_length=1)
    mount: str
    parent_frame: str = Field(min_length=1)
    arm_id: str | None = None


class Settings(Model):
    namespace: str = "/robotics"
    driver_service: str = "/robot_driver/execute"
    state_max_age: float = Field(default=2.0, gt=0.0, le=60.0)
    camera_timeout: float = Field(default=1.5, gt=0.0, le=5.0)

    @field_validator("namespace", "driver_service")
    @classmethod
    def ros_name(cls, value):
        import re
        if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", value):
            raise ValueError("Expected an absolute ROS2 name without trailing slash")
        return value


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
        if len(arms) != len(self.arms) or len({c.id for c in self.cameras}) != len(self.cameras):
            raise ValueError("Duplicate arm or camera IDs")
        for camera in self.cameras:
            if camera.mount == "world":
                if camera.arm_id is not None or camera.parent_frame != self.world_frame:
                    raise ValueError("World camera must reference world_frame without an arm_id")
            elif camera.mount == "flange":
                if camera.arm_id not in arms or camera.parent_frame != arms[camera.arm_id].flange_frame:
                    raise ValueError("Flange camera must reference its arm's flange_frame")
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
        return f"{self.server.namespace}/request"

    def state_topic(self, arm_id):
        return f"{self.server.namespace}/arms/{arm_id}/state"

    def camera_topic(self, camera_id):
        return f"{self.server.namespace}/cameras/{camera_id}/image/compressed"

    @classmethod
    def load(cls, path):
        return cls.model_validate(json.loads(Path(path).read_text()))
