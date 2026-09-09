"""ROS connection names and remapping configuration, independent of ROS imports."""

import re

from pydantic import BaseModel, ConfigDict, Field, field_validator


# HARDWARE INTEGRATION: change server.remappings in your topology JSON for hardware.
# TOOL EXTENSION: define new application topic/service names here, not in nodes.
class RosNames(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)

    namespace: str = "/robotics"
    driver_service: str = "/robot_driver/execute"
    remappings: dict[str, str] = Field(default_factory=dict)

    @staticmethod
    def absolute_name(value):
        if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", value):
            raise ValueError(f"Expected an absolute ROS2 name without trailing slash; actual {value!r}")
        return value

    @field_validator("namespace", "driver_service")
    @classmethod
    def validate_name(cls, value):
        return cls.absolute_name(value)

    @field_validator("remappings")
    @classmethod
    def validate_remappings(cls, value):
        for source, target in value.items():
            for side, name in (("source", source), ("target", target)):
                try:
                    cls.absolute_name(name)
                except ValueError as exc:
                    raise ValueError(f"remappings {side}: {exc}") from exc
        return value

    @property
    def request_service(self):
        return f"{self.namespace}/request"

    def state_topic(self, arm_id):
        return f"{self.namespace}/arms/{arm_id}/state"

    def camera_topic(self, camera_id):
        return f"{self.namespace}/cameras/{camera_id}/image/compressed"

    def camera_info_topic(self, camera_id):
        return f"{self.namespace}/cameras/{camera_id}/camera_info"

    def camera_depth_topic(self, camera_id):
        return f"{self.namespace}/cameras/{camera_id}/depth/aligned"

    def node_options(self, options):
        """Apply shared remaps to a node; explicit CLI rules take precedence."""
        if not self.remappings:
            return dict(options)
        arguments = list(options.get("cli_args") or [])
        # Finish an existing ROS argument scope before appending the defaults.
        if arguments and arguments[-1] != "--":
            arguments.append("--")
        arguments.append("--ros-args")
        for source, target in self.remappings.items():
            arguments.extend(("-r", f"{source}:={target}"))
        arguments.append("--")
        return dict(options, cli_args=arguments)
