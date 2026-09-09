"""HTTP contract for the future ROS2 node; no ROS2 runtime is needed here."""

import asyncio
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
    raise ValueError(f"{name}: expected range [{minimum}, {maximum}]; actual {value!r}")
  return float(value)


# TOOL EXTENSION: map tools to validated HTTP requests; never replay uncertain motions.
class Ros2RobotClient:
  def __init__(self, config: RobotConfig, *, transport=None):
    self.config = config
    self._client = httpx.AsyncClient(
        base_url=config.robot_url, timeout=httpx.Timeout(config.timing.request_timeout+config.timing.ros_response_margin+config.timing.http_response_margin),
        transport=transport, follow_redirects=False,
    )

  def _budget(self, operation, payload=None):
    return self.config.timing.operation_timeout(operation, payload)+self.config.timing.ros_response_margin+self.config.timing.http_response_margin

  def _arm(self, arm_id: str) -> str:
    if arm_id not in {arm.id for arm in self.config.arms}:
      raise ValueError(f"Unknown arm: {arm_id}")
    return arm_id

  async def _request(self, method: str, path: str, **kwargs) -> dict[str, Any]:
    # Motion requests are deliberately sent once: a timeout has an unknown outcome.
    budget = kwargs.pop('timeout', self.config.timing.request_timeout+self.config.timing.ros_response_margin+self.config.timing.http_response_margin)
    try:
      async with asyncio.timeout(budget):
        response = await self._client.request(method, path, timeout=budget, **kwargs)
    except TimeoutError as exc:
      raise httpx.ReadTimeout(f"Request elapsed deadline exceeded: {budget}s", request=httpx.Request(method, self.config.robot_url+path)) from exc
    response.raise_for_status()
    if response.status_code != 200:
      return {"success": False, "outcome": "unknown",
              "error": "Expected completed operation; asynchronous acceptance is unsupported"}
    try:
      result = response.json()
      if not isinstance(result, dict):
        raise ValueError("The robot server must return a JSON object")
      if method == 'POST' and type(result.get('success')) is not bool:
        raise ValueError("The robot server must report boolean completion")
    except ValueError as exc:
      raise httpx.RemoteProtocolError(str(exc), request=response.request) from exc
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
    }, timeout=self._budget("move_arm", {"duration": duration}))

  async def set_gripper(self, arm_id: str, opening: float):
    self._arm(arm_id)
    return await self._request("POST", f"/v1/arms/{arm_id}/gripper", json={
        "opening": _number(opening, "opening", 0.0, 1.0),
    }, timeout=self._budget("set_gripper"))

  async def stop(self, arm_id: str | None = None):
    if arm_id is not None:
      self._arm(arm_id)
    return await self._request("POST", "/v1/stop", json={"arm_id": arm_id}, timeout=self._budget("stop"))

  async def get_camera_snapshot(self, camera_id: str) -> bytes:
    if camera_id not in {camera.id for camera in self.config.cameras}:
      raise ValueError(f"Unknown camera: {camera_id}")
    budget = self._budget('camera')
    try:
      async with asyncio.timeout(budget):
        response = await self._client.get(f"/v1/cameras/{camera_id}/image",
            headers={"Cache-Control": "no-cache"}, timeout=budget)
    except TimeoutError as exc:
      raise httpx.ReadTimeout(f"Camera elapsed deadline exceeded: {budget}s") from exc
    response.raise_for_status()
    if response.headers.get("content-type", "").split(";")[0] != "image/jpeg":
      raise ValueError("Camera endpoint must return image/jpeg")
    with Image.open(io.BytesIO(response.content)) as image:
      if image.format != "JPEG":
        raise ValueError("Camera endpoint did not return JPEG data")
      image.verify()
    return response.content

  async def capture(self, camera_id):
    return await self._capture(camera_id, 'capture')

  async def observation(self, camera_id):
    # TOOL EXTENSION: original RGB evidence keeps identity/time without requiring depth.
    return await self._capture(camera_id, 'observation')

  async def _capture(self, camera_id, endpoint):
    if camera_id not in {c.id for c in self.config.cameras}:
      raise ValueError('Unknown camera')
    result = await self._request('GET', f'/v1/cameras/{camera_id}/{endpoint}', timeout=self._budget(endpoint))
    missing = {'capture_id', 'camera_id', 'width', 'height', 'image_base64'} - result.keys()
    if missing:
      raise ValueError(f'Capture metadata missing fields: {sorted(missing)}')
    for key in ('width', 'height'):
      if type(result[key]) is not int or result[key] <= 0:
        raise ValueError(f'Capture {key}: expected positive integer; actual {result[key]!r}')
    import base64
    data = base64.b64decode(result['image_base64'], validate=True)
    with Image.open(io.BytesIO(data)) as image:
      if image.format != 'JPEG':
        raise ValueError(f'Capture image format: expected JPEG; actual {image.format!r}')
      if image.size != (result['width'], result['height']):
        raise ValueError(f"Capture dimensions: expected {(result['width'], result['height'])}; actual {image.size}")
      image.verify()
    if result['camera_id'] != camera_id:
      raise ValueError(f"Capture camera_id: expected {camera_id!r}; actual {result['camera_id']!r}")
    if not isinstance(result['capture_id'], str) or not result['capture_id']:
      raise ValueError('Capture capture_id: expected nonempty string')
    return result

  async def workflow(self, operation, **payload):
    paths = {'recover': '/v1/arms/recover', 'reset': '/v1/arms/reset', 'create': '/v1/plans',
             'refine': '/v1/plans/refine', 'execute': '/v1/plans/execute',
             'verify': '/v1/plans/verify', 'move_arms': '/v1/arms/poses'}
    return await self._request('POST', paths[operation], json=payload, timeout=self._budget({'create':'create_plan','refine':'refine_plan','verify':'verify_grasp','execute':'execute_plan','reset':'reset_arms','recover':'recover_arms','move_arms':'move_arms'}[operation], payload))

  async def close(self):
    await self._client.aclose()
