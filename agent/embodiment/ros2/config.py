"""Robot topology shared by prompts, tool schemas, and HTTP validation."""

import dataclasses
import json
import re
from pathlib import Path
from urllib.parse import urlsplit


def _identifier(value: str) -> str:
  if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", value):
    raise ValueError(f"Invalid resource ID: {value!r}")
  return value


def _frame(value: str) -> str:
  if not isinstance(value, str) or not value.strip() or value.startswith("/"):
    raise ValueError(f"Expected a nonempty relative ROS2 frame ID: {value!r}")
  return value


@dataclasses.dataclass(frozen=True)
class Arm:
  id: str
  base_frame: str
  flange_frame: str

  def __post_init__(self):
    _identifier(self.id)
    _frame(self.base_frame)
    _frame(self.flange_frame)


@dataclasses.dataclass(frozen=True)
class Camera:
  id: str
  optical_frame: str
  mount: str
  parent_frame: str
  arm_id: str | None = None

  def __post_init__(self):
    _identifier(self.id)
    _frame(self.optical_frame)
    _frame(self.parent_frame)
    if self.mount not in {"world", "flange"}:
      raise ValueError("Camera mount must be 'world' or 'flange'")


@dataclasses.dataclass(frozen=True)
class RobotConfig:
  robot_url: str
  world_frame: str
  arms: tuple[Arm, ...]
  cameras: tuple[Camera, ...]

  def __post_init__(self):
    url = urlsplit(self.robot_url)
    if (url.scheme not in {"http", "https"} or not url.hostname
        or url.username or url.password or url.query or url.fragment
        or url.path not in {"", "/"}):
      raise ValueError("robot_url must be an HTTP(S) origin without credentials")
    _frame(self.world_frame)
    if not 1 <= len(self.arms) <= 2:
      raise ValueError("Configure one or two arms")
    if not self.cameras:
      raise ValueError("Configure at least one camera")
    for resources in (self.arms, self.cameras):
      ids = [item.id for item in resources]
      if len(ids) != len(set(ids)):
        raise ValueError("Resource IDs must be unique within each kind")
    arms = {arm.id: arm for arm in self.arms}
    for camera in self.cameras:
      if camera.mount == "world":
        if camera.arm_id is not None or camera.parent_frame != self.world_frame:
          raise ValueError("World cameras must have world_frame as parent and no arm_id")
      elif (camera.arm_id not in arms
            or camera.parent_frame != arms[camera.arm_id].flange_frame):
        raise ValueError("Flange cameras must reference their arm and its flange_frame")

  @property
  def frame_ids(self) -> tuple[str, ...]:
    return tuple(dict.fromkeys([
        self.world_frame,
        *(frame for arm in self.arms for frame in (arm.base_frame, arm.flange_frame)),
        *(camera.optical_frame for camera in self.cameras),
    ]))

  @classmethod
  def load(cls, path: str | Path) -> "RobotConfig":
    data = json.loads(Path(path).read_text())
    return cls(
        robot_url=data["robot_url"], world_frame=data["world_frame"],
        arms=tuple(Arm(**arm) for arm in data["arms"]),
        cameras=tuple(Camera(**camera) for camera in data["cameras"]),
    )
