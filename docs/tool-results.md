# Tool outcomes and scenario coverage

This guide separates transport success, command completion, visual assessment
and task achievement. It complements [tool lifecycle](tool-lifecycle.md) and the
[driver integration guide](../server/docs/integration.md). The tables cover all
15 tool names; a configuration without a wrist camera exposes 13, omitting
`approach_targets` and `refine_grasp`.

## Outcome contract

Motion completes only when the driver returns HTTP-style status 200 and boolean
`success: true`. Group operations also require `coordinated: true` and an exact
`completed_arm_ids` list. A 202 acceptance, malformed JSON or missing/non-boolean
completion is not success. Explicit failure details survive HTTP and reach Live
in the tool response. Transport failure can mean `outcome: "unknown"`; motion
is never automatically replayed.

A camera/state query succeeding is evidence acquisition, not task achievement.
ER identifies candidate contact pixels; it does not prove a grasp. The Live
agent judges grasp images, while the bridge enforces capture identity, plan
stage and telemetry constraints. Optional `object_detected: false` overrides a
positive grasp assessment. Missing/null telemetry is not evidence of success.
A moving arm or reported arm/gripper fault blocks new motion. Positive object
telemetry after place/recovery contradicts release and prevents success.

| Tool | Trigger and meaning of success | Source of the outcome / evidence | Failure or unknown outcome |
|---|---|---|---|
| `get_robot_state` | Startup, after motion/recovery, and before finishing; current state and a newly delivered scene image | `/v1/state`: arm state, faults, `recovery_required`, `motion_outcome_unknown`; Live response: `success`, `post_action_observation`, `observation_revision` | State or fresh image failure is reported as failure; observe again within the application budget or finish with failure |
| `reset_arms` | Startup without a held-object plan or unresolved failure; all arms homed | Coordinated driver completion, fresh state and fault checks | Stop/recover if needed; reset cannot replace recovery or discard a held-object plan |
| `move_arm` | Explicit measured manual flange target reached | Driver `success`, fresh arm state/fault checks | No automatic retry; stop and recover before a new attempt |
| `move_arms` | Explicit measured targets completed as one group | Driver `success`, `coordinated`, exact `completed_arm_ids`, state/fault checks | Reject manual motion that would discard a held-object plan; preserve the recovery requirement |
| `set_gripper` | Requested opening achieved, not necessarily an object grasped | Driver completion and arm/gripper faults | Report failure; closed jaws alone do not establish a grasp |
| `detect_targets` | New fixed-camera grasp/release points converted into a plan | Fresh capture, strictly validated ER JSON and normalized coordinates, measured depth/TF, bridge `plan_id`, `state`, `targets` | Invalid/missing/blocked detections produce no motion; retry with a new capture or finish with failure |
| `approach_targets` | Observation/approach phase finished for every selected arm | Group driver completion and `state: "approached"` | Interrupted/failed plan is unusable; stop/recover |
| `refine_grasp` | One grasp point replaced using its own newer wrist capture | ER output and projection; matching arm, capture-time flange pose, `plan_id` and retained release target | No movement on inference/conversion failure; obtain new evidence or recover if the plan is no longer usable |
| `pick_targets` | Open/approach/descend/close/lift completed, not a verified grasp | Group driver completion, state/fault checks, `state: "picked"` | Inspect every arm after success; stop/recover after failed or ambiguous execution |
| `inspect_grasp` | Original post-pick image and stationary arm state obtained and image delivered | `observation_id`, `capture_id`, `stamp_ns`, arm/camera identity, `image_delivered`; original image precedes the response | No visual claim is made by this tool; failed delivery/invalid evidence cannot authorize placement |
| `verify_grasp` | Live's image assessment accepted for every plan arm | Per-arm boolean, reason and delivered observation ID; bridge verifies capture time and telemetry; `state: "verified"` | Negative/uncertain assessment blocks place; new evidence is needed to reassess. Faults or uncertain plan transitions require state inspection/recovery |
| `place_targets` | Transfer/descend/open/retreat completed, not proof of the desired final scene | Group completion, state/fault/release checks, `state: "placed"` | Failed release or motion needs recovery; otherwise observe the scene before a new plan or success finish |
| `stop` | Targeted arms have stopped | Driver completion; all-arm stop differs from single-arm stop | Failed/unconfirmed stop blocks further motion. Cancellation of a service wait alone is not physical stopping |
| `recover_arms` | Confirmed all-arm stop followed by support/release/retreat/home | Stop result, coordinated recovery completion, cleared faults/release telemetry, remaining recovery budget | Do not issue recovery after failed stop; unsupported/unrecoverable/exhausted recovery ends with failure/operator action |
| `finish_task` | Application ended with an explicit result | Live's summary/visual judgment; success additionally requires no held/unresolved plan, a scene observation after the latest motion attempt, and a new state check without motion/fault/held-object telemetry | Premature success is rejected. Failure can terminate the task; failed cleanup stop is exposed as `stop_result` and `operator_required` |

`post_action_observation` records the ordinary post-tool scene image, not the
physical result of the command. Motion success is retained when that optional
image is unavailable, with this flag false; do not repeat the motion to obtain
an image. `get_robot_state` specifically promises a new scene observation, so
its response is failure when delivery fails or when another motion makes its
state/image pair obsolete. Grasp inspection uses separate original-image
`image_delivered` evidence; ordinary mosaics pause while inspection is pending.
The CLI tool-result events reflect the response sent to Live, including image
acquisition/delivery failures.

## Final placement assessment

After `place_targets`, call `get_robot_state` and examine the fixed-camera view
of the destination and the released object. Confirm that the intended object
is at the requested destination and is no longer held. If occluded or uncertain,
observe again; if misplaced, choose a new plan only after observing the scene,
or finish with failure. Driver stage completion alone cannot answer this.

The application enforces a delivered scene observation after the latest motion
attempt before success finish, and checks fresh state again at finish. It also
requires post-place observation before new detection. Image meaning and task
achievement remain the Live agent's judgment, described in the
[system instruction](../agent/embodiment/ros2/instruction.md) and application
instructions. These guards do not certify recognition accuracy or prove that
an arbitrary model's verbal success claim is truthful. No separate ER call or
new placement-verification endpoint is introduced.

## Stop, disconnection and restart policy

An agent stop invalidates plans/evidence and cancels admission of motion calls
already waiting on the action lock. Generation checks discard late detection,
refinement, verification, recovery and motion results after interruption. The
bridge also rejects superseded motion/stop replies. Unknown completion requires
all-arm stop and recovery; stopping just one arm cannot resolve group uncertainty.
Manual/reset commands cannot erase a held-object plan to bypass these checks.

The ROS2 Live session ends on disconnection instead of reconnecting and replaying
an application without its physical context. It also ends on missing/repeated
function-call IDs, preventing repeated manual commands with the same ID. Session
loss closes motion admission and triggers stop; a stop failure remains visible
in the application error/log. Recovery failures terminate explicitly and must
not silently start a new plan. The copied upstream session code remains unchanged;
this behavior lives in the ROS2-specific session subclass.

Plan, evidence, deduplication and recovery state are in memory. A bridge restart
loses them; old plan/capture IDs are rejected, but this is not persistent physical
state recovery. Before restarting an application after a server/controller
restart, the operator/driver integration must establish stopped arms, inspect
current telemetry and payloads, and complete required recovery. A driver must
reject commands when its own physical state is unknown. Do not assume a fresh
bridge's default flags mean that a previously interrupted robot is ready. This
sample does not implement durable jobs or automatic restart/resume.

## Tests mapped to the contract

Run both suites using the [server test instructions](../server/README.md#tests).
The response-driven scenarios are in
[scenario_test.py](../server/tests/scenario_test.py). `ScenarioStream` receives
actual tool responses, checks their IDs and content, and branches on completion,
errors and decoded image fixtures. ER and the physical driver are mocked;
agent dispatch, session handling, HTTP and ROS bridge processing run normally.
Each scenario has a 40-call bound and a 20-second application deadline.

| Scenario or contract | Coverage |
|---|---|
| One fixed camera, one arm | `success`; existing `test_one_fixed_camera_one_arm_completes_without_wrist` |
| Wrist refinement, coordinated dual-arm execution, repeated cycles | Existing `test_live_agent_er_http_and_ros_complete_two_dual_arm_cycles` |
| ER detection failure, invalid coordinates/JSON, timeout followed by new capture | `er_invalid`, `er_coordinates`, `er_json`, `er_timeout` |
| Occlusion, reinspection, uncertainty with bounded recovery | `reinspection`, `uncertain`; existing grasp evidence tests |
| Visual/sensor conflict | `sensor_conflict`; existing `test_negative_object_sensor_overrides_positive_visual_assessment` |
| Failed motion, lost completion, partial group acknowledgement | `phase_failure`, `motion_timeout`, `group_partial`; assert no place on the failed plan and all-arm stop before recovery |
| Stop failure, unsupported/unrecoverable recovery, exhausted attempts | `stop_failure`, `unsupported_recovery`, `unrecoverable`, `recovery_limit` |
| Missing depth/state, failed scene observation | `missing_depth`, `missing_state`, `image_failure`; existing missing/stale JPEG/TF/capture tests |
| Placement completed but image shows the wrong destination | `placement_miss`; premature finish is rejected before final observation |
| Disconnection before/during motion, duplicate call ID | `disconnect`, `disconnect_motion`, `duplicate`; assert stop, cleanup, no reconnect or replay |
| Held-object manual move and invalid reset arguments | `manual_held`, `invalid_reset`; rejection/recovery is visible to the model |
| Complete HTTP/ROS empty-command validation | `test_empty_commands_validate_body_before_dispatch`, `test_invalid_empty_commands_never_reach_driver_over_http_or_ros` |
| Single/dual-arm held plans, sensors present/absent | `test_manual_motion_cannot_forget_held_plan_with_or_without_sensor`, agent held-plan regression tests |
| Incorrect release completion, moving telemetry | `test_place_or_recover_cannot_succeed_with_a_positive_held_sensor`, `test_moving_telemetry_prevents_new_motion` |
| Stop during capture/ER/create, verification or recovery, cancelled motion | Agent `followup_test.py` interruption regressions |
| Superseded stop and stale final observation | `test_late_all_arm_stop_cannot_override_a_newer_failed_stop`, agent final-observation regressions |
| Real HTTP socket integration | Existing `test_single_arm_agent_over_real_http_and_ros` |

The driver fixture reports/simulates outcomes and records requested group stops;
it cannot verify that a real controller physically stopped a peer arm or kept a
shared object synchronized. These tests verify application control flow and
contracts, not Gemini accuracy, calibration accuracy or physical robot behavior.
