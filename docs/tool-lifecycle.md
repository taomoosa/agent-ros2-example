# Tool lifecycle, failures and minimal setup

## When to call each tool

| Tool | Trigger / precondition | Next step |
|---|---|---|
| `get_robot_state` | Startup, after placement/stop/recovery, or active observation | Inspect current faults, plan status and the fresh camera frame |
| `reset_arms` | Normal startup; no held object or unresolved failure | Detect new targets |
| `detect_targets` | After startup or successful placement/recovery; no held-object plan | Pick directly, or optionally approach for wrist refinement |
| `approach_targets` | Detected plan, before optional wrist refinement | Refine a grasp or pick |
| `refine_grasp` | Approached plan, matching wrist camera | Refine another participating arm or pick |
| `pick_targets` | Detected/approached plan; no unresolved motion failure | Inspect each arm if successful; otherwise recover |
| `inspect_grasp` | Completed pick; selected arm stationary | Inspect remaining arms, then assess |
| `verify_grasp` | New delivered observation for every participating arm | Place only if every judgment and hardware check passes |
| `place_targets` | Verified grasp, no fault or negative object sensor | Observe final state/image and detect the next plan or finish |
| `recover_arms` | Failed motion/grasp or unknown outcome | Stop all, driver recovery, then observe and detect a new plan |
| `move_arm`, `move_arms`, `set_gripper` | Explicit measured/manual operation; no unresolved failure | Read state; pixel plans are invalidated |
| `stop` | Motion interruption, uncertainty or fault | Read state and recover before retrying failed manipulation |
| `finish_task` | Visible completion, or inability to proceed | End the application; unresolved held objects/failures cannot be reported as success |

Plan responses contain `state` and `next_actions`. State responses include bridge
`recovery_required` / `motion_outcome_unknown` and the agent's local plan/recovery
status. `get_robot_state` is blocking in the tool schema so a fresh image precedes
its response. The old `ack` no-op is no longer exposed to Gemini: it did not wake
up the model or schedule a subsequent turn. In a topology without wrist cameras,
`approach_targets` and `refine_grasp` are omitted from the exposed tools and hints.

## Failure detection and controlled retries

HTTP errors preserve the driver's JSON details, including optional `code`,
`failed_phase`, `failed_arm_ids`, `recoverable`, and `outcome`. They are no longer
all described as unknown communication failures. Network timeouts still have an
unknown outcome and are never automatically retried. Failed motion invalidates
the agent plan as well as the server plan, preventing a stale retry.

The arm state topic optionally accepts:

```json
{
  "fault": {"code": "drive_fault", "message": "Controller reports a drive fault", "recoverable": false},
  "gripper": {
    "opening": 0.0,
    "fault": {"code": "jaw_jam", "message": "Jaw movement blocked", "recoverable": true},
    "object_detected": false
  }
}
```

This shows added fields; the existing `moving` and `flange_pose` fields are still
required. Faults and object detection default to null when not supplied. Opening-
only legacy state messages remain valid. `recoverable` defaults to false in an
explicit fault. Missing sensor readings are not interpreted as successful grasps.

The bridge checks arm/gripper faults before ordinary motion, during grasp
verification, before placement, and again on ordinary successful command completion.
A negative object-detection reading blocks a positive visual assessment and placement.
The driver must still report phase failures promptly, stop the group after partial
failure, and implement its hardware limits; the bridge cannot infer unreported
hardware faults from a successful reply and absent telemetry.

Retry paths:

- A read-only detection/refinement failure can be retried using new imagery and
  clearer instructions. No motion target is created from malformed/absent points.
- Unclear visual confirmation can be retried with new inspection images. Submitted
  observations are consumed, so changing a failed answer requires new evidence.
- Repeating a physical grasp after failure requires `recover_arms`, then new
  detection and a new plan ID. A successful all-arm stop is necessary but does not
  by itself clear the recovery requirement. `reset_arms` cannot bypass it.
- `recover_arms` is a deliberate tool call, not automatic request replay. It first
  stops every arm. The application permits at most two recovery calls by default;
  `--max-recovery-attempts 0..10` changes that total, including failed attempts.
  Recovery does not reset this counter. Failed stop prevents the recovery command.
- Unrecoverable faults, unsupported recovery or an exhausted budget require an
  operator or task failure. No fallback blindly opens a loaded gripper.

### Recovery driver operation

`POST /v1/arms/recover` has no body fields. The bridge forwards
`operation="recover_arms"` using `RobotRequest`, with:

```json
{
  "arm_ids": ["left", "right"],
  "coordinated": true,
  "phases": ["secure_or_support_payload", "release", "retreat", "home"]
}
```

The driver must preflight recovery, support or safely place any held payload
before release, recover only eligible faults, retreat and home all arms together.
Use robot-specific recovery targets and limits. Return the normal coordinated
completion acknowledgement only when the recovery has actually completed; publish
cleared/current state. If this cannot be done, return a failure. The skeleton
supplies the request/state machine and test driver, not a physical recovery planner.
The deadline is 60 seconds; the agent's HTTP budget is 65 seconds.

## One fixed camera and one arm

Use matching `agent/configs/minimal.json` and `server/configs/minimal.json`.
There is one arm (`arm`) and one fixed camera (`overhead`), with no wrist camera.

```bash
# Server terminal, after the ROS2 build and environment setup:
python -m ros2_agent_server --config server/configs/minimal.json

# Agent terminal, from agent/:
python run_ros2.py --config configs/minimal.json --model "$GEMINI_LIVE_MODEL" \
  --task-file apps/pick_and_place.md \
  --instruction "Move the blue block onto the tray."
```

`inspect_grasp(plan_id, arm_id, camera_id?)` uses the selected arm's wrist camera
when available; otherwise it uses a configured fixed camera. An explicit camera
must be world-fixed or attached to that arm. The server accepts either type for
post-pick verification and still checks capture time and arm identity. The fixed
view must show the held object clearly; an occluded grasp is not presumed successful.

The single camera must still provide the RGB-D capture contract: rectified JPEG,
aligned measured depth, CameraInfo, and optical-to-world TF. A single RGB-D camera
is sufficient; uncalibrated monocular RGB alone does not determine metric depth.
See [pixel geometry](pixel-workflow.md#capture-geometry). A compatible driver is
needed for physical execution. The tests run this entire minimal workflow with
real ROS2 communication and mocked Gemini/driver components.

## Customizing ER prompts without editing source

```bash
python run_ros2.py --config configs/minimal.json --model "$GEMINI_LIVE_MODEL" \
  --er-detect-prompt-file prompt_examples/grasp_guidance.md \
  --task-file apps/pick_and_place.md --instruction "Move the blue block."
```

`--er-detect-prompt-file` appends UTF-8 application guidance to the default ER
position-detection prompt. `--er-refine-prompt-file` does the same for optional
wrist refinement. Paths are relative to the working directory; contents are read
once at startup. Missing/empty files fail before the application connects to the
robot. The Python API accepts `er_prompt_files={"detect": path, "refine": path}`
in `run_application`. Default templates remain in `embodiment/ros2/prompts/`.

Use these files for contact-point preferences, object-specific constraints and
visibility criteria. The built-in JSON keys and normalized `[y,x]` coordinate
contract stay in the request; model output is still strictly validated. The tool's
`instruction` adds the particular object/destination request for each call.

## Completion evidence and interruption

See [tool outcomes and scenario coverage](tool-results.md) for the complete
per-tool result/evidence table and regression mapping. Manual motion and reset
cannot discard a held-object plan; stop and recover first. Successful finish
requires a delivered `get_robot_state` scene observation after the latest motion
attempt and a new final state check. After placement, inspect the destination
before detecting again; stage completion alone does not prove task achievement.
Missing final imagery must lead to another observation or failure, not another
execution of the completed placement. Gemini disconnection or duplicate tool
call IDs terminate the application and trigger stop instead of automatic replay.
