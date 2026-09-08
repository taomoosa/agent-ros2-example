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

  add("get_robot_state", "Observe at startup, after placement/stop, or while waiting. Read current poses, faults, "
      "gripper telemetry and local plan/recovery status; a fresh camera observation follows.", {}, blocking=True)
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
  add("finish_task", "Finish after get_robot_state delivers fresh state and imagery following the latest motion attempt. "
      "Report failure honestly if the task cannot be completed.", {
          "success": {"type": "BOOLEAN"}, "summary": {"type": "STRING"},
      }, ("success", "summary"))
  plan = {"plan_id": {"type": "STRING", "description": "Opaque ID returned by detect_targets."}}
  text = {"type": "STRING"}
  camera = {"type": "STRING", "enum": [c.id for c in config.cameras]}
  add("reset_arms", "At task startup with no held object or unresolved failure, home ALL arms. For failed manipulation use recover_arms instead.", {}, blocking=True)
  add("recover_arms", "After a failed motion/grasp, stop ALL arms and request driver recovery: "
      "support/release any payload, retreat and home together. Invalidates old plans. "
      "Only after success read state and detect a NEW plan. Attempts are limited per application; "
      "unsupported or unrecoverable faults require finish_task(success=false).", {}, blocking=True)
  add("detect_targets", "After startup/reset, successful recovery, or verified placement, ask Gemini Robotics ER for new grasp/release pixels. "
      "ROS2 converts measured depth and capture-time TF into a plan. For a shared object include both arms in one plan.",
      {"camera_id": camera, "instruction": text,
       "arm_ids": {"type": "ARRAY", "items": arm, "minItems": 1, "maxItems": len(config.arms)}},
      ("camera_id", "instruction", "arm_ids"), blocking=True)
  for name, description in (
      ("approach_targets", "Move all plan arms to driver-defined observation/approach poses before optional wrist refinement."),
      ("pick_targets", "Execute coordinated open, approach, descend, close and lift phases for every plan arm. Then inspect_grasp for each arm and verify_grasp."),
      ("place_targets", "After successful verify_grasp, execute coordinated transfer, descend, open and retreat for all plan arms.")):
    add(name, description, plan, ("plan_id",), blocking=True)
  add("refine_grasp", "Ask Robotics ER to refine one grasp in a new wrist image after approach. "
      "ROS2 binds the pixels to that image's flange pose; the release target is retained.",
      dict(plan, arm_id=arm, camera_id=camera, instruction=text),
      ("plan_id", "arm_id", "camera_id", "instruction"), blocking=True)
  add("inspect_grasp", "After pick completes, show a fresh original image and arm state. Choose camera_id explicitly, or "
      "default to this arm wrist camera, falling back to a fixed camera. "
      "Call separately for every participating arm, then judge whether the intended object is held. "
      "This tool does not call ER or decide success.", dict(plan, arm_id=arm, camera_id=camera),
      ("plan_id", "arm_id"), blocking=True)
  add("verify_grasp", "Record YOUR grasp assessment from inspect_grasp images. "
      "Include every plan arm, its observation ID, boolean success and a visual reason. "
      "Use success=false if uncertain, occluded, slipping or empty; closed grippers alone are insufficient. "
      "Placement is enabled only when every arm is confirmed.", dict(plan, observations={
          "type": "ARRAY", "minItems": 1, "maxItems": len(config.arms), "items": {
              "type": "OBJECT", "properties": {
                  "arm_id": arm, "observation_id": text, "success": {"type": "BOOLEAN"}, "reason": text},
              "required": ["arm_id", "observation_id", "success", "reason"]}}),
      ("plan_id", "observations"), blocking=True)
  pose_properties = dict(next(d for d in declarations if d['name'] == 'move_arm')['parameters']['properties'])
  add("move_arms", "Move one or both arms in ONE coordinated driver request using measured metric flange poses. "
      "Invalidates pixel plans. For a shared object prefer a dual-arm pick plan, which includes synchronized lifting.",
      {"moves": {"type": "ARRAY", "minItems": 1, "maxItems": len(config.arms),
                 "items": {"type": "OBJECT", "properties": pose_properties,
                           "required": ["arm_id", "frame_id", "position", "orientation"]}}},
      ("moves",), blocking=True)
  if not any(c.mount == 'flange' for c in config.cameras):
    declarations = [d for d in declarations if d['name'] not in {'approach_targets', 'refine_grasp'}]
  return [{"functionDeclarations": declarations}]
