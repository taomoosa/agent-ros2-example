"""Time budgets shared through the server section of the topology JSON."""

import dataclasses
import math


@dataclasses.dataclass(frozen=True)
class Timing:
    motion_timeout: float = 120.0
    settling_timeout: float = 15.0
    state_completion_timeout: float = 5.0
    bridge_processing_margin: float = 1.0
    gripper_timeout: float = 15.0
    stop_timeout: float = 5.0
    request_timeout: float = 5.0
    camera_timeout: float = 5.0
    ros_response_margin: float = 2.0
    http_response_margin: float = 3.0
    er_timeout: float = 90.0
    live_io_timeout: float = 10.0
    cleanup_timeout: float = 3.0
    model_idle_timeout: float = 120.0
    capture_ttl: float = 300.0
    plan_ttl: float = 600.0

    def __post_init__(self):
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 3600:
                raise ValueError(f"{field.name} must be finite and in (0, 3600]")

    @classmethod
    def from_server(cls, data):
        names = {field.name for field in dataclasses.fields(cls)}
        return cls(**{key: value for key, value in data.items() if key in names})

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
