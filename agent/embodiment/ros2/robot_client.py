"""HTTP contract for the future ROS2 node; no ROS2 runtime is needed here."""

import io
import math
from typing import Any

import httpx
from PIL import Image

from embodiment.ros2.config import RobotConfig


def _number(value, name: str, minimum=None, maximum=None) -> float:
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    raise ValueError(f"{name} must be a finite number")
  if minimum is not None and value < minimum or maximum is not None and value > maximum:
    raise ValueError(f"{name} is outside the allowed range")
  return float(value)


class Ros2RobotClient:
  def __init__(self, config: RobotConfig, *, transport=None):
    self.config = config
    self._client = httpx.AsyncClient(
        base_url=config.robot_url, timeout=httpx.Timeout(10.0),
        transport=transport, follow_redirects=False,
    )

  def _arm(self, arm_id: str) -> str:
    if arm_id not in {arm.id for arm in self.config.arms}:
      raise ValueError(f"Unknown arm: {arm_id}")
    return arm_id

  async def _request(self, method: str, path: str, **kwargs) -> dict[str, Any]:
    # Motion requests are deliberately sent once: a timeout has an unknown outcome.
    response = await self._client.request(method, path, **kwargs)
    response.raise_for_status()
    if response.status_code == 202:
      return {"success": False, "outcome": "unknown",
              "error": "Expected completed operation; asynchronous acceptance is unsupported"}
    result = response.json()
    if not isinstance(result, dict):
      raise ValueError("The robot server must return a JSON object")
    return result

  async def get_robot_state(self):
    return await self._request("GET", "/v1/state")

  async def move_arm(self, arm_id: str, frame_id: str, position: list,
                     orientation: list, duration: float = 3.0):
    self._arm(arm_id)
    if frame_id not in self.config.frame_ids:
      raise ValueError(f"Unknown coordinate frame: {frame_id}")
    if not isinstance(position, list) or len(position) != 3:
      raise ValueError("position must be [x, y, z] in metres")
    if not isinstance(orientation, list) or len(orientation) != 4:
      raise ValueError("orientation must be quaternion [x, y, z, w]")
    position = [_number(value, "position") for value in position]
    orientation = [_number(value, "orientation", -1, 1) for value in orientation]
    if not math.isclose(sum(value * value for value in orientation), 1.0, abs_tol=1e-3):
      raise ValueError("orientation must be a unit quaternion")
    duration = _number(duration, "duration", 0.1, 60.0)
    return await self._request("POST", f"/v1/arms/{arm_id}/pose", json={
        "frame_id": frame_id, "position": position,
        "orientation": orientation, "duration": duration,
    }, timeout=duration + 10.0)

  async def set_gripper(self, arm_id: str, opening: float):
    self._arm(arm_id)
    return await self._request("POST", f"/v1/arms/{arm_id}/gripper", json={
        "opening": _number(opening, "opening", 0.0, 1.0),
    })

  async def stop(self, arm_id: str | None = None):
    if arm_id is not None:
      self._arm(arm_id)
    return await self._request("POST", "/v1/stop", json={"arm_id": arm_id})

  async def get_camera_snapshot(self, camera_id: str) -> bytes:
    if camera_id not in {camera.id for camera in self.config.cameras}:
      raise ValueError(f"Unknown camera: {camera_id}")
    response = await self._client.get(f"/v1/cameras/{camera_id}/image", headers={
        "Cache-Control": "no-cache",
    })
    response.raise_for_status()
    if response.headers.get("content-type", "").split(";")[0] != "image/jpeg":
      raise ValueError("Camera endpoint must return image/jpeg")
    with Image.open(io.BytesIO(response.content)) as image:
      if image.format != "JPEG":
        raise ValueError("Camera endpoint did not return JPEG data")
      image.verify()
    return response.content

  async def capture(self, camera_id):
    if camera_id not in {c.id for c in self.config.cameras}:
      raise ValueError('Unknown camera')
    result = await self._request('GET', f'/v1/cameras/{camera_id}/capture')
    if (not {'capture_id', 'camera_id', 'width', 'height', 'image_base64'} <= result.keys()
        or any(type(result[k]) is not int or result[k] <= 0 for k in ('width', 'height'))):
      raise ValueError('Invalid capture metadata')
    import base64
    data = base64.b64decode(result['image_base64'], validate=True)
    with Image.open(io.BytesIO(data)) as image:
      if image.format != 'JPEG' or image.size != (result['width'], result['height']):
        raise ValueError('Capture image dimensions do not match metadata')
      image.verify()
    if result['camera_id'] != camera_id or not isinstance(result['capture_id'], str):
      raise ValueError('Invalid capture identity')
    return result

  async def workflow(self, operation, **payload):
    paths = {'recover': '/v1/arms/recover', 'reset': '/v1/arms/reset', 'create': '/v1/plans',
             'refine': '/v1/plans/refine', 'execute': '/v1/plans/execute',
             'verify': '/v1/plans/verify', 'move_arms': '/v1/arms/poses'}
    return await self._request('POST', paths[operation], json=payload, timeout=65.)

  async def close(self):
    await self._client.aclose()
