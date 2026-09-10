# Pixel-guided manipulation

## Execution and tool boundaries

The application still uses Gemini Live for task orchestration. Image reasoning
runs separately through Gemini Robotics ER's `generateContent` endpoint, using
`httpx`; there is no new Python SDK dependency. `--model` selects the Live model;
`--robotics-model` selects the image reasoning model (default
`gemini-robotics-er-2-preview`). Both use `GEMINI_API_KEY`.

```mermaid
sequenceDiagram
    participant Live as Gemini Live
    participant Agent as ROS2 embodiment / Manipulation
    participant ER as Gemini Robotics ER
    participant HTTP as FastAPI gateway
    participant ROS as Robot bridge
    participant Driver as Robot driver
    Live->>Agent: reset_arms
    Agent->>HTTP: POST /v1/move (all arms, named home)
    HTTP->>ROS: move (named home)
    ROS->>Driver: prepare then move named home targets for all arms
    Driver-->>Live: Completed result through ROS, HTTP and agent
    Live->>Agent: detect_targets(camera, instruction, arms)
    Agent->>HTTP: GET camera capture
    HTTP->>ROS: Capture new RGB with configured depth/TF or fixed-plane calibration
    ROS-->>Agent: Capture ID, original image, frozen metadata
    Agent->>ER: Original image + detection prompt
    ER-->>Agent: Normalized grasp/release points per arm
    Agent->>HTTP: POST /v1/plans with original-image pixels
    HTTP->>ROS: Project with frozen depth/TF or calibrated plane
    ROS-->>Live: Plan ID and targets through HTTP and agent
    opt Wrist refinement
        Live->>Agent: approach_targets(plan), then refine_grasp
        Agent->>ROS: Coordinated approach through HTTP
        ROS->>Driver: prepare then move all plan arms to approach poses
        Agent->>ER: New wrist capture + refinement prompt
        ER-->>Agent: Refined grasp pixel
        Agent->>ROS: Refine plan using capture-time flange/camera TF
    end
    Live->>Agent: pick_targets(plan)
    Agent->>ROS: Execute pick through HTTP
    ROS->>Driver: prepare full pick sequence
    loop open, approach, descend, close, lift
        ROS->>Driver: move or gripper for all plan arms
        Driver-->>ROS: Group completion; verify new measured state
    end
    loop Each participating arm
        Live->>Agent: inspect_grasp(plan, arm)
        Agent->>ROS: Fresh post-pick wrist or fixed RGB observation and state through HTTP
        Agent-->>Live: Original inspection image, observation ID, arm state
    end
    Live->>Agent: verify_grasp(plan, observations with success and reason)
    Agent->>ROS: Record agent assessments with bound capture IDs through HTTP
    Live->>Agent: place_targets(plan)
    Agent->>ROS: Execute verified plan through HTTP
    ROS->>Driver: prepare then execute transfer, descend, open, retreat primitives
    Live->>Agent: Inspect fresh images/state and detect the next plan
```

The implementation path is `run_ros2.py` → `Ros2SessionManager` (extends `SessionManager`) → upstream
`Observation` / `DecisionMaking` / `ToolCallHandler` →
`Ros2Embodiment.execute_action` → `Manipulation` / `Ros2RobotClient` → HTTP
`api.py` → `HttpGatewayNode` → ROS service → `RobotBridgeNode` / `PixelPlans`.
`Manipulation` invokes `RoboticsER.reason` for detection and refinement only.
For grasp assessment, `inspect_grasp` captures an original inspection image and state.
The ROS2-specific `live_session.py` sends this image immediately before its tool
response, under the shared stream lock, and records delivery. The Live agent
then calls `verify_grasp` with its own assessment. Original upstream session/tool
execution code is unchanged by this feature. Ordinary actions run serially at
the agent boundary; stop bypasses the action lock. Grouped operations contain
all participating arms (one or two) and require coordinated execution in the driver.

| Tool | Purpose |
|---|---|
| `reset_arms()` | Home all configured arms at normal startup |
| `recover_arms()` | Stop all arms and request payload support/release, retreat and home before a new attempt |
| `detect_targets(camera_id, instruction, arm_ids)` | Detect grasp/release points on one fixed-camera image and create a ROS2 plan |
| `approach_targets(plan_id)` | Move all plan arms to approach poses generated from the configured task profile |
| `refine_grasp(plan_id, arm_id, camera_id, instruction)` | Replace one grasp point using a new wrist image after approach; retain release points |
| `pick_targets(plan_id)` | Coordinated opening, approach, descent, closing and lifting |
| `inspect_grasp(plan_id, arm_id, camera_id?)` | Send one original post-pick wrist or fixed-camera image and current stationary arm state to the Live agent; return an observation ID |
| `verify_grasp(plan_id, observations)` | Register the Live agent's per-arm `{arm_id, observation_id, success, reason}` assessments; enable placement only if all arms pass |
| `place_targets(plan_id)` | Coordinated transfer, descent, opening and retreat after verification |
| `move(targets, arm_ids or all_arms)` | Capture-bound pixel or named targets for one or two arms; no model-supplied poses |
| `get_robot_state`, `gripper`, `stop`, `finish_task` | Existing state, manual motion, stopping and session tools |

For tool visibility, dispatch and extension points, use the
[adapter and tool extension guide](../server/docs/primitive-adapter.md#adding-and-exposing-tools).
For each tool's preconditions, results and failure handling, see
[tool outcomes](tool-results.md).

## Prompts and application instructions

- `agent/embodiment/ros2/instruction.md`: the Live system instruction, followed
  by the configured arm/camera topology.
- `agent/apps/pick_and_place.md` and `dual_arm.md`: application workflows loaded
  by `--task-file`. `--instruction` supplies object and destination descriptions.
- `agent/embodiment/ros2/prompts/detect.md`, `refine.md`: separate ER prompts.
  `robotics_er.py` appends the task, selected arms and capture identity.
- Grasp assessment criteria are in the Live system instruction and `tools.py`:
  identify the intended object being held, use the current arm state as context,
  reject empty/slipping/occluded or uncertain grasps, and report a visual reason.
  There is no separate ER verification prompt or request.

The ER prompts request normalized `[y, x]` values in `0..1000`, following the
[Google Robotics ER guide](https://ai.google.dev/gemini-api/docs/generate-content/robotics-overview).
The agent converts them to integer `[x, y]` original-image pixel centers using
`round(x * (width - 1) / 1000)` and the corresponding height formula. Neither the
Live model nor ER provides depth or robot poses. ER receives the exact original
JPEG, never the resized mosaic used by Live observations. Empty, malformed,
out-of-bounds, blocked or incomplete detections do not create motion targets.

Custom ER guidance can be supplied without editing source:

```bash
# From agent/, after configuring calibration, task profiles and backend positions.
python run_ros2.py --config configs/primitives.json --model "$GEMINI_LIVE_MODEL" \
  --er-detect-prompt-file prompt_examples/grasp_guidance.md \
  --task-file apps/pick_and_place.md --instruction "Move the blue block onto the tray."
```

`--er-detect-prompt-file` and `--er-refine-prompt-file` append UTF-8 guidance to
the respective built-in prompts. Files are read once at startup; missing/empty
files fail before robot connection. Relative paths use the working directory.
The Python API accepts `er_prompt_files={"detect": path, "refine": path}`.
Customize contact preferences and visibility criteria; the built-in JSON fields
and normalized coordinate contract remain mandatory. The tool's `instruction`
provides the particular object/destination for each call.

## Minimal setup

One arm and one fixed camera can execute detect, pick, inspect, verify and place.
Use matching agent/server configuration. `configs/primitives.json` includes
illustrative plane calibration, task profiles and the `home` name-description
catalog; replace calibration/clearances and implement the backend's named position
and controller hooks before motion. For RGB-D, keep the camera/topology from
`configs/minimal.json` and add the task profiles and name catalog described in
[hardware configuration](../server/docs/primitive-adapter.md#hardware-configuration).
The unmodified topology-only minimal file is not a complete pick/place setup.

Inspection uses that fixed camera when no wrist camera is configured. The
view must show the held object; occlusion is not success. Metric detection needs
registered depth/CameraInfo/image-time TF, or fixed-plane calibration for points
on that plane. RGB inspection itself requires neither depth nor TF. Follow the
[server startup](../server/README.md#setup-and-startup) and
[hardware bring-up](../server/docs/integration.md#4-bring-up-and-verify-the-integration)
steps before starting the application command above.

## Agent-owned grasp assessment

The Live agent already has the task and motion history. Keeping grasp assessment
in that session avoids an additional ER request per arm and lets the agent
explain its judgment in context. This is an architectural choice, not a measured
claim of higher recognition accuracy. A suitable camera image is still required; inspection falls back to a fixed
camera when no wrist camera exists.

Inspection sends one original inspection image per tool call, not the ordinary
resized mosaic. Its immediately following tool response identifies the arm,
camera, capture timestamp and observation ID. Ordinary mosaic updates are held
while inspection evidence is pending. The session waits at least one second
before each inspection frame and prevents images and responses from interleaving,
following the [Live API video input limit](https://ai.google.dev/gemini-api/docs/live-api/capabilities#sending-video).
After assessment consumes the evidence, ordinary observation resumes.

For example, after inspecting both arms, the Live agent may call:

```json
{
  "plan_id": "returned-plan-id",
  "observations": [
    {"arm_id": "left", "observation_id": "left-observation-id", "success": true,
     "reason": "The intended bar end is visible in the gripper, clear of the table."},
    {"arm_id": "right", "observation_id": "right-observation-id", "success": false,
     "reason": "The contact is occluded; I cannot establish a successful grasp."}
  ]
}
```

This assessment leaves the plan in `picked` and blocks placement. Gripper
telemetry accepts nullable/optional `opening`, `fault` and `object_detected`
fields in the required `gripper` object. The complete ROS state also requires
measurement `stamp_ns`, `moving` and `flange_pose`; see
[telemetry](../server/docs/telemetry.md). Closed jaws or null/missing sensor readings
are not proof of grasp; `object_detected: false` blocks a positive visual
assessment and placement. No force/contact sensor inference is fabricated.
Use a new inspection or stop when the evidence is insufficient.

## Capture geometry

For the default `projection: "depth"`, each camera used for detection or
refinement must provide the following inputs in addition to JPEG. CameraInfo and depth use
sensor-data QoS; TF uses the standard dynamic/static TF publishers:

| Topic | Message | Requirement |
|---|---|---|
| `/robotics/cameras/{id}/camera_info` | `sensor_msgs/CameraInfo` | Matching full rectified image grid; use standard rectified P by default (nonzero raw lens D is accepted), or explicit normalized rectified K via [CameraInfo mode](../server/docs/camera-info.md). Cropping, nonidentity R and stereo offsets are unsupported |
| `/robotics/cameras/{id}/depth/aligned` | `sensor_msgs/Image` | Color-grid/color-Z depth, matching dimensions, configured depth header frame and acquisition-time tolerance; `16UC1` millimetres or `32FC1` metres |
| `/tf`, `/tf_static` | Standard TF messages | Transform from camera optical frame to world at image acquisition; additionally flange-to-world at that same time for wrist cameras |

Calibration may be published independently of each exposure, but must describe
the current image geometry. Publishers must share the ROS clock. The server
waits for a new image and matching depth/TF; it never substitutes the latest
arm telemetry pose for a capture-time transform. Depth row stride and byte
order are respected. Out-of-range/non-finite depth, insufficient local depth
support, invalid calibration or mismatched geometry fails conversion. Missing synchronized data/TF returns 504.

For pixel `(u,v)` with measured optical-axis depth `z`, the bridge computes
`[(u-cx)*z/fx, (v-cy)*z/fy, z]`, then applies the frozen camera-to-world transform.
This is a surface/contact point. The common compiler applies configured
orientation and task clearances to generate a **TCP pose**; the backend handles
its own tool calibration and controller target conversion.
The same projection is used for destination support surfaces. Fixed cameras can
alternatively use [offline plane calibration](../server/docs/plane-projection.md)
with `projection: "plane"`; capture then needs only fresh RGB and the configured
homography/plane pose. Both selected points must lie on that calibrated plane.
Automatic monocular depth/plane estimation is not implemented. Wrist refinement
still requires registered measured depth and image-time TF. Grasp inspection
uses the depth-free `/observation` endpoint.

See [camera synchronization](../server/docs/synchronization.md) for bounded
approximate pairing, frame aliases, source timestamps, TF interpolation and
calibration/clock changes. The default pairing tolerance is 10 ms; tune it to
measured timing and acceptable error, or set it to zero for strict pairing.
See [numerical tolerances](../server/docs/numerical-tolerances.md) for the separate
5 ms future-clock allowance, calibration rounding and depth quality checks.

Snapshots have opaque IDs, last for `server.capture_ttl` (default 300 seconds), and are bounded to 64
entries. Up to 32 plans are retained; detected/approached plans expire after
`server.plan_ttl` (default 600 seconds). A motion invalidates other unfinished plans and pre-motion
snapshots. A plan stores its converted world points and capture provenance;
subsequent camera movement cannot change them. Source JPEGs can be evicted
without changing the converted points. Re-detect after scene changes.

## HTTP workflow and state transitions

| Method and path | JSON body / result |
|---|---|
| `GET /v1/cameras/{id}/capture` | Returns `kind: "rgbd"`, `capture_id`, `camera_id`, `width`, `height`, RGB `stamp_ns` / `frame_id`, `camera_pose`, nullable `flange_pose`, `image_base64`, `depth_stamp_ns`, `depth_frame_id`, `camera_info_frame_id`, `camera_info_stamp_ns`, signed `sync_delta_ns` and RGB `pose_stamp_ns` |
| `GET /v1/cameras/{id}/capture` (plane mode) | Returns `kind: "plane"`, common capture ID/image/dimensions/frame/time fields, and frozen `plane_calibration`; no depth or camera/flange poses |
| `GET /v1/cameras/{id}/observation` | Returns `kind: "rgb"`, `capture_id`, `camera_id`, `width`, `height`, `stamp_ns`, `frame_id`, `image_base64`; valid for visual verification, not metric projection |
| `POST /v1/plans` | `{capture_id, targets: [{arm_id, grasp: [x,y], release: [x,y]}]}` |
| `POST /v1/plans/refine` | `{plan_id, arm_id, capture_id, pixel: [x,y]}` |
| `POST /v1/plans/execute` | `{plan_id, stage: "approach" \| "pick" \| "place"}` |
| `POST /v1/plans/verify` | `{plan_id, observations: [{arm_id, capture_id, success: boolean}]}` |
| `POST /v1/move` | `{all_arms:true, targets:[{kind:"named", name:"home"}]}` for startup home |
| `POST /v1/arms/recover` | Omitted body or `{}`; all configured arms after successful all-arm stop |
| `POST /v1/move`, `POST /v1/gripper` | Shared `arm_ids`/`all_arms` selection; see [target contract](../server/docs/primitive-adapter.md) |

Plan responses include `success`, `plan_id`, `state`, and per-arm `targets` with
`grasp`/`release` fields containing `frame_id`, `position`, `capture_id`, `pixel`
and `stamp_ns`. Completed stages also include `motion_stamp_ns`.

States are `detected → approached (optional) → picked → verified → placed`.
Refinement is allowed only after approach and only from that arm's newer wrist
RGB-D capture, with both exposures after approach completion. Verification must
cover every selected arm using fixed-camera or matching wrist images acquired
**after pick completion**. A false visual result retains `picked`, so placement
remains blocked; inspect again or stop. Verification is supplied by the trusted
Live agent, not cryptographically attested. The agent submits an observation ID
that must match the plan and arm, and whose image has actually been sent to its
session. Unsent or invalidated observations and duplicate/missing arms are
rejected before HTTP submission. Evidence is consumed on submission, even on
failure; changing a failed judgment requires new inspection images. Reasons
are included in the tool result/log; the unchanged server verification endpoint
receives bound capture IDs and booleans. A placed or interrupted plan cannot be replayed.
The bridge admits one motion at a time, with stop and observation still available.
Reset, stop and manual motions invalidate unfinished plans. Reset/manual motions
cannot discard a held-object plan; stop and recover first. Unknown outcomes
require a successful stop of **all** arms and driver recovery before another attempt;
a stop of one arm does not resolve uncertainty about its peer.

## Coordinated driver contract

The common bridge expands approach/pick/place into deterministic steps, then
sends `prepare` followed by `move`/`gripper` primitives to `server.driver_service`.
The preparation includes every resolved step and participating arm. Pick uses
`open`, `approach`, `descend`, `close`, `lift`; place uses `transfer`, `descend`,
`open`, `retreat`. ER detection remains entirely on the agent side.

Configure task orientations, clearances and a name-to-description `position_names`
catalog in `server.hardware`. Register actual named coordinates and TCP
calibration in the backend. The compiler converts contact points into TCP poses;
the controller backend handles path planning, collision/force limits and
physical group synchronization. Shared objects require preservation of relative
grasp constraints. If any arm fails, stop the group and do not execute the next
phase. Missing controller capabilities are rejected at preflight.

Every primitive completion must confirm all requested arms, including singleton
commands. The bridge checks new measured stationary state after each phase.
All phases share the original execution/settling/state budget; no phase resets
the deadline. A failed sequence requests a group stop with an independent stop
budget. Cancellation of a ROS future alone does not stop hardware. Unknown
outcomes require confirmed stop and recovery, followed by a new plan.
See [time budgets](../server/docs/time-budgets.md) and
[completion telemetry](../server/docs/telemetry.md#freshness-and-command-completion).

The [adapter guide](../server/docs/primitive-adapter.md) describes exact inputs,
outputs, capability declarations and the hardware template. Partial stops expand
to coupled groups even after their plan is invalidated. The repository supplies
common orchestration; actual controllers, calibration and physical completion
checks must be implemented and validated for the mechanism.

## Validation

Unit tests cover normalized coordinate conversion, ER request/response handling,
measured depth projection, transform rotation, stride/endianness, stale captures
and invalid input. Integration tests publish actual ROS2 RGB-D and TF messages,
exercise HTTP/services, and replace only the physical driver and Gemini calls.
They cover moving wrist transforms, optional refinement, failed/stale grasp
verification, group acknowledgement, stop/timeouts, replay rejection, coordinated
metric motion, and a complete Live/ER dual-arm workflow over two task cycles.
They also assert exact original-image delivery before inspection responses,
that grasp assessment never calls ER, and that undelivered/incorrect evidence
or a failed agent assessment cannot enable placement. These tests do not establish
physical synchronization or comparative accuracy of the Live and ER models.

See [tool outcomes and scenario coverage](tool-results.md) for each tool's
completion/evidence source, final placement assessment and interruption policy.

## Plan arm selection

`detect_targets(camera_id, instruction, arm_ids)` explicitly selects the plan's
participants. In a two-arm setup, `["left"]` or `["right"]` creates a one-arm
plan; `["left", "right"]` creates a coupled two-arm plan. ER must return exactly
one grasp/release pair per selected arm. `approach_targets`, `pick_targets` and
`place_targets` take `plan_id` and act on **all and only that plan's arms**.
They never implicitly include other configured arms, and cannot select a subset
of an existing plan. To change participants, create a new plan after completing
or safely recovering the current work.

Both-arm plans require synchronized phases and shared-object constraints from
the backend. They are not two independent pick operations; separate concurrent
plans are not supported. `refine_grasp` updates one participating arm using its
own wrist camera. Call `inspect_grasp` for each participant, then `verify_grasp`
with exactly those arms. A one-arm plan needs only one assessment, including in
a two-arm setup. All configured arms must keep publishing fresh telemetry. Motion health checks
cover the selected arms; agent state observation and final task checks also
consider faults on other configured arms. `reset_arms` and `recover_arms` always affect all
configured arms; `stop` may expand to a coupled group.
