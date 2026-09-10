"""Transport-neutral results and validation for internal ROS2 requests."""

from dataclasses import dataclass, field
import json

from pydantic import ValidationError

from .models import Stop


class BridgeError(Exception):
    def __init__(self, status: int, message: str, *, outcome=None, code=None, details=None):
        super().__init__(message)
        self.status = status
        self.payload = {"success": False, "error": message}
        if code:
            self.payload["code"] = code
        if details is not None:
            self.payload["details"] = details
        if outcome:
            self.payload["outcome"] = outcome


@dataclass
class Reply:
    status: int = 200
    payload: dict = field(default_factory=dict)
    data: bytes = b""
    content_type: str = "application/json"

    @classmethod
    def from_error(cls, error):
        return cls(status=error.status, payload=error.payload)

    def to_ros(self, response):
        response.status_code = self.status
        response.payload_json = json.dumps(self.payload, allow_nan=False)
        response.data = self.data
        response.content_type = self.content_type
        return response

    @classmethod
    def from_ros(cls, response):
        try:
            payload = json.loads(response.payload_json)
            if not isinstance(payload, dict):
                raise ValueError(f"payload_json: expected object; actual {type(payload).__name__}")
            if not 200 <= response.status_code <= 599:
                raise ValueError(f"status_code: expected 200..599; actual {response.status_code}")
            # Reject NaN/Infinity in a peer's JSON as well.
            json.dumps(payload, allow_nan=False)
            return cls(response.status_code, payload, bytes(response.data), response.content_type)
        except (TypeError, ValueError) as exc:
            raise BridgeError(502, f"Invalid response from ROS2 service: {exc}", outcome="unknown") from exc


# TOOL EXTENSION: both HTTP and direct ROS callers must pass these checks.
def validate_request(config, operation, resource_id, payload):
    if not isinstance(payload, dict):
        raise BridgeError(422, "Request payload must be a JSON object")
    arms = {arm.id for arm in config.arms}
    cameras = {camera.id for camera in config.cameras}
    if operation in {"camera", "capture", "observation"} and resource_id not in cameras:
        raise BridgeError(404, f"Unknown camera: {resource_id}")
    if operation in {'move', 'gripper'}:
        from .motion import Move, Grip
        try:
            if resource_id:
                raise ValueError('Unified commands use arm_ids/all_arms, not resource_id')
            body = (Move if operation == 'move' else Grip).model_validate(payload)
            ids = body.selected(config)
            if operation == 'move' and len(body.targets) not in (1,len(ids)):
                raise ValueError('targets must contain one shared target or one target per arm')
            if operation == 'move':
                for target in body.targets:
                    if target.kind == 'pose' and target.frame_id not in config.frame_ids:
                        raise ValueError(f'Unknown coordinate frame: {target.frame_id}')
            return body.model_dump()
        except ValueError as exc:
            raise BridgeError(422,str(exc)) from exc
    from .workflow import BODIES, validate_workflow
    if operation in BODIES or operation in {'recover_arms'}:
        try:
            return validate_workflow(config, operation, resource_id, payload)
        except ValidationError as exc:
            raise BridgeError(422, "Request validation failed", details=[
                {k: item[k] for k in ("loc", "msg", "type")} for item in exc.errors()]) from exc
        except ValueError as exc:
            raise BridgeError(422, str(exc)) from exc
    try:
        if operation == "stop":
            body = Stop.model_validate(payload)
            if resource_id:
                raise BridgeError(422, "stop uses arm_ids/all_arms, not resource_id")
            from .motion import Selection
            ids = Selection(arm_ids=body.arm_ids, all_arms=body.all_arms if "all_arms" in body.model_fields_set else body.arm_ids is None).selected(config)
            return dict(arm_ids=ids)
        if operation in {"state", "camera", "capture", "observation"}:
            if payload:
                raise BridgeError(422, f"{operation}: unexpected payload fields {sorted(payload)}")
            if operation == "state" and resource_id:
                raise BridgeError(422, f"state.resource_id: expected empty; actual {resource_id!r}")
            return payload
    except ValidationError as exc:
        raise BridgeError(422, "Request validation failed", details=[
            {k: item[k] for k in ("loc", "msg", "type")} for item in exc.errors()]) from exc
    except ValueError as exc:
        raise BridgeError(422,str(exc)) from exc
    raise BridgeError(404, f"Unknown operation: {operation}")
