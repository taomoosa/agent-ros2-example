"""Robot topology shared by prompts, tool schemas, and HTTP validation."""

import dataclasses
import json
import math
import re
from pathlib import Path
from urllib.parse import urlsplit
from embodiment.ros2.timing import Timing


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
  depth_frame: str | None = None
  camera_info_frame: str | None = None
  depth_geometry: str = "color_optical_z"
  camera_info_mode: str = "rectified_k"
  sync_tolerance_sec: float = 0.01
  calibration_tolerance_px: float = 1e-6
  rectification_tolerance: float = 1e-9
  depth_min_m: float = 0.05
  depth_max_m: float = 5.0
  depth_absolute_tolerance_m: float = 0.02
  depth_relative_tolerance: float = 0.01
  depth_min_support: float = 0.6
  projection: str = "depth"
  plane_calibration: dict | None = None

  def __post_init__(self):
    _identifier(self.id)
    _frame(self.optical_frame)
    _frame(self.parent_frame)
    for frame in (self.depth_frame, self.camera_info_frame):
      if frame is not None:
        _frame(frame)
    if self.camera_info_mode not in {'rectified_k', 'ros_rectified'}:
      raise ValueError('camera_info_mode must be rectified_k or ros_rectified')
    if self.depth_geometry != 'color_optical_z':
      raise ValueError(f'Unsupported depth_geometry: {self.depth_geometry!r}')
    if (type(self.sync_tolerance_sec) not in (int, float) or not math.isfinite(self.sync_tolerance_sec)
        or not 0 <= self.sync_tolerance_sec <= .1):
      raise ValueError(f'sync_tolerance_sec must be finite in [0, 0.1]; actual {self.sync_tolerance_sec!r}')
    for name, lower, upper, inclusive in (
        ('calibration_tolerance_px', 0., .01, True),
        ('rectification_tolerance', 0., 1e-6, True),
        ('depth_min_m', 0., math.inf, False),
        ('depth_max_m', 0., math.inf, False),
        ('depth_absolute_tolerance_m', 0., .1, True),
        ('depth_relative_tolerance', 0., .1, True),
        ('depth_min_support', .5, 1., False)):
      value = getattr(self, name)
      if (type(value) not in (int, float) or not math.isfinite(value)
          or (value < lower if inclusive else value <= lower) or value > upper):
        raise ValueError(f'{name}: invalid tolerance or range value {value!r}')
    if self.depth_min_m >= self.depth_max_m:
      raise ValueError('depth_min_m must be less than depth_max_m')
    if self.projection not in {'depth', 'plane'}:
      raise ValueError('projection must be depth or plane')
    if self.projection == 'plane':
      if self.mount != 'world' or not isinstance(self.plane_calibration, dict):
        raise ValueError('Plane projection requires a fixed world camera and plane_calibration')
      required = {'calibration_id', 'image_width', 'image_height', 'homography', 'valid_region', 'plane_pose'}
      if set(self.plane_calibration) != required:
        raise ValueError(f'plane_calibration fields must be {sorted(required)}')
      calibration = self.plane_calibration
      if not isinstance(calibration['calibration_id'], str) or not calibration['calibration_id']:
        raise ValueError('plane_calibration.calibration_id must be nonempty')
      for key in ('image_width', 'image_height'):
        if type(calibration[key]) is not int or not 2 <= calibration[key] <= 65536:
          raise ValueError(f'plane_calibration.{key} must be an integer in [2, 65536]')
      for key, length in (('homography', 9), ('valid_region', 4)):
        value = calibration[key]
        if (not isinstance(value, list) or len(value) != length
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
          raise ValueError(f'plane_calibration.{key} requires {length} finite numbers')
      pose = calibration['plane_pose']
      if not isinstance(pose, dict) or set(pose) != {'frame_id', 'position', 'orientation'}:
        raise ValueError('plane_pose requires frame_id, position and orientation')
      _frame(pose['frame_id'])
      for key, length in (('position', 3), ('orientation', 4)):
        value = pose[key]
        if (not isinstance(value, list) or len(value) != length
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
          raise ValueError(f'plane_pose.{key} requires {length} finite numbers')
      if not math.isclose(sum(v*v for v in pose['orientation']), 1., abs_tol=1e-3):
        raise ValueError('plane_pose.orientation must be a unit quaternion')
      x0,y0,x1,y1 = calibration['valid_region']
      if not (0 <= x0 < x1 < calibration['image_width'] and 0 <= y0 < y1 < calibration['image_height']):
        raise ValueError('plane_calibration.valid_region must be ordered inside the image')
      scale = max(abs(v) for v in calibration['homography'])
      if not scale:
        raise ValueError('plane_calibration.homography must be nonsingular')
      a,b,c,d,e,f,g,h,i = [v/scale for v in calibration['homography']]
      determinant = a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g)
      rows = math.hypot(a,b,c)*math.hypot(d,e,f)*math.hypot(g,h,i)
      denominators = [g*x+h*y+i for x in (x0,x1) for y in (y0,y1)]
      if abs(determinant) <= 1e-12*rows:
        raise ValueError('plane_calibration.homography is singular or ill-conditioned')
      if not (min(denominators) > 1e-9 or max(denominators) < -1e-9):
        raise ValueError('plane_calibration.homography horizon intersects or approaches valid_region')
    elif self.plane_calibration is not None:
      raise ValueError('plane_calibration requires projection=plane')
    if self.mount not in {"world", "flange"}:
      raise ValueError("Camera mount must be 'world' or 'flange'")


@dataclasses.dataclass(frozen=True)
class RobotConfig:
  robot_url: str
  world_frame: str
  arms: tuple[Arm, ...]
  cameras: tuple[Camera, ...]
  timing: Timing = dataclasses.field(default_factory=Timing)

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
      if camera.plane_calibration and camera.plane_calibration['plane_pose']['frame_id'] != self.world_frame:
        raise ValueError('plane_pose.frame_id must match world_frame')
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
        timing=Timing.from_server(data.get("server", {})),
    )
