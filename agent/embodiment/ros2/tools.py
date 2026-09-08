"""Gemini tool declarations constrained to the configured ROS2 resources."""

from embodiment.ros2.config import RobotConfig


def ros2_tools(config: RobotConfig) -> list[dict]:
  arm = {"type": "STRING", "enum": [item.id for item in config.arms]}
  declarations = []

  def add(name, description, properties, required=(), blocking=False):
    parameters = {"type": "OBJECT", "properties": properties}
    if required:
      parameters["required"] = list(required)
    declaration = {"name": name, "description": description, "parameters": parameters}
    if blocking:
      declaration["behavior"] = "BLOCKING"
    declarations.append(declaration)

  add("get_robot_state", "Read arm poses, gripper states and calibrated frame information.", {})
  add("move_arm", "Move the selected arm flange to a pose. Metres; unit quaternion in xyzw order. "
      "Use only measured or user-supplied poses. The ROS2 server resolves TF and validates motion.", {
          "arm_id": arm,
          "frame_id": {"type": "STRING", "enum": list(config.frame_ids)},
          "position": {"type": "ARRAY", "items": {"type": "NUMBER"}, "minItems": 3, "maxItems": 3},
          "orientation": {"type": "ARRAY", "items": {"type": "NUMBER"}, "minItems": 4, "maxItems": 4},
          "duration": {"type": "NUMBER", "minimum": 0.1, "maximum": 60,
                       "description": "Motion duration in seconds; default 3."},
      }, ("arm_id", "frame_id", "position", "orientation"), blocking=True)
  add("set_gripper", "Set the selected gripper opening: 0 closed, 1 open.", {
      "arm_id": arm, "opening": {"type": "NUMBER", "minimum": 0, "maximum": 1},
  }, ("arm_id", "opening"), blocking=True)
  add("stop", "Stop one arm, or all arms when arm_id is omitted.", {"arm_id": arm})
  add("ack", "No action needed; wait for the next observation or user instruction.", {})
  add("finish_task", "Finish the application after verifying its outcome in state and fresh images. "
      "Report failure honestly if the task cannot be completed.", {
          "success": {"type": "BOOLEAN"}, "summary": {"type": "STRING"},
      }, ("success", "summary"))
  return [{"functionDeclarations": declarations}]
