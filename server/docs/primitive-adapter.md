# Common motion execution and hardware adapter

The common server executes all plan phases through `PrimitiveAdapter`.
Use the **same topology/task configuration** in the agent, server and adapter.
Keep named coordinates and TCP calibration in the hardware backend separately.

[primitives.json](../configs/primitives.json) is a complete one-arm, fixed-plane
example. Its camera calibration and task clearances are illustrative. Replace
these with measured values; backend-owned named goals are shown separately in
[robot_hardware.json](../examples/robot_hardware.json). RGB-D cameras use the same execution path: keep their
existing camera configuration and add the `server` fields from this example.

## Responsibility and migration

The agent still captures original images and calls Robotics ER for detection and
refinement. The bridge resolves pixels using frozen capture geometry and manages
plan/evidence lifetimes. `MotionCompiler` resolves goals and expands
approach/pick/place into deterministic steps; `sequencer.execute` runs these
steps under one deadline. No extra model decision or HTTP call is needed between
steps. Hardware receives poses with an explicit `tcp`/`flange` reference, or
opaque named goals. Pixel/plan goals always carry TCP poses. The backend receives
no ER requests, raw pixels or high-level `execute_plan` operations.

Hardware operations are `prepare`, `move`, `gripper`, `stop` and optional
`recover`, handled through the supplied `PrimitiveAdapter`.

The agent advertises `move`, `gripper` and `stop` with common
`arm_ids`/`all_arms` selection. Semantic plan/inspection/recovery tools remain;
`reset_arms` sends `/v1/move` with `all_arms:true` and the named `home` target.

### Migration from the removed driver contract

There is no `server.driver_mode` switch. Remove that field from both configurations;
stale mode settings are rejected. Replace a seven-operation driver with the
backend hooks below before connecting it to this server. Old `move_arm`,
`move_arms` and `set_gripper` tool/Python aliases and `/v1/arms/{arm_id}/pose`,
`/v1/arms/{arm_id}/gripper`, `/v1/arms/poses`, `/v1/arms/reset` HTTP routes were
removed. Use `/v1/move` or `/v1/gripper`; stop uses `arm_ids`/`all_arms`, never
`arm_id`. The remaining high-level ROS `execute_plan`/`recover_arms` requests
are bridge operations, not driver operations. Camera formats and telemetry are
unchanged by this migration.

Topology-only examples can observe; trusted HTTP clients can command supplied flange poses without
`server.hardware`. Homing requires a named `home` for every selected arm;
pixel targets and plan stages require per-arm profiles. Missing configuration
fails before physical dispatch. Copy the illustrative hardware section from
`primitives.json` and replace task geometry with measured values. Register each
advertised name in the backend before execution; the common server stores no
named coordinates and performs no TCP-to-flange conversion.

## Target and selection contract

Gemini can choose only pixel or named targets. Both its tool schema and dispatch
reject direct poses. The pose HTTP contract below remains available to trusted
programmatic clients; pixel/plan execution also generates TCP poses internally.

For `POST /v1/move`, specify exactly one of `arm_ids: ["arm"]` or
`all_arms: true`. Empty/duplicate/unknown IDs and contradictory selectors are
rejected. `targets` contains one target per selected arm in **selection order**,
or a single target broadcast to the selected arms. Broadcasting a named target
asks the backend to resolve that name separately for each arm. Broadcasting a metric/pixel point
does not imply that two arms can occupy the same location: group preflight must
reject collisions and impossible shared-object geometry.

```json
{
  "arm_ids": ["arm"],
  "duration": 3.0,
  "targets": [{
    "kind": "pose", "reference": "flange", "frame_id": "world",
    "position": [0.3, 0.0, 1.2], "orientation": [0.0, 1.0, 0.0, 0.0]
  }]
}
```

`duration` is 0.1..60 seconds, default 3. The group uses one common duration.
The backend must plan synchronized arrival for all selected arms.
The operation budget remains execution + settling + state/transport margins.

| Target kind | Fields | Resolution |
|---|---|---|
| `pose` | `frame_id`, `position`, `orientation`, optional `reference` (`flange` default or `tcp`) | Metres, quaternion xyzw; preserve reference and pass to the backend. No common-server tool calibration |
| `pixel` | `capture_id`, integer original-image `pixel: [x,y]`, `profile: "tabletop"`, optional `offset_m` in 0..1 | Project the saved RGB-D/plane capture and apply tabletop orientation/contact offset/clearance, yielding a TCP pose. A wrist capture must belong to that arm |
| `named` | `name` | Check the advertised name, then pass it unchanged. The backend resolves its local pose, joint goal or controller preset |

Only fields for the selected kind are accepted. A pixel alone cannot specify a
full pose. Expired/invalidated captures and RGB-only observations cannot be used
as metric evidence. TF is taken from the capture, never replaced by the current
wrist pose. A manual move invalidates existing capture/plan context;
use approach/refine/pick/place for the managed manipulation workflow.

Home example: `{"all_arms":true,"targets":[{"kind":"named","name":"home"}]}`.
A held-object plan or unresolved failure blocks manual movement including named
home. A positive measured object sensor also blocks named home/reset even if no
plan exists; unknown sensors cannot establish whether a payload is present. Recovery is not a home alias. Joint goals can only come from trusted named
configuration, not arbitrary model-supplied joint arrays.

`POST /v1/gripper` uses the same selection and `opening` in 0..1. The single
opening applies to each selected arm; pick phases can use individually calibrated
closing openings. The backend maps normalized openings to its controller units
and contact criterion. Closed jaws alone do not establish successful grasp.

## Hardware configuration

`server.hardware` contains task `profiles` and a `position_names` catalog, each
keyed by arm ID. The catalog contains name-to-description metadata only:

```json
{"position_names":{"arm":{"home":"Initial position for task startup.","ready":"Observation position."}}}
```

Names and descriptions appear in the agent's tool description and form the server allowlist.
Actual name-to-position pairs belong to the mechanism, not this shared JSON.
Unknown arm IDs, empty names/descriptions, coordinate objects and invalid task profile fields are
rejected when the bridge constructs its compiler.

| Profile field | Meaning |
|---|---|
| `orientation` | Desired TCP rotation in world coordinates for tabletop contacts |
| `approach_m` | Positive world +Z clearance for approach and retreat |
| `lift_m` | Positive world +Z lift from grasp contact |
| `transfer_height_m` | Absolute world Z of the release-side TCP transfer waypoint; must clear the release contact |
| `contact_offset_m` | World Z contact correction; default zero |
| `close_opening` | Calibrated close request in 0..1; default zero |
| `duration` | Phase trajectory duration; default 3 seconds; group uses the maximum |

The supplied policy assumes **world +Z is vertical**. It resolves contact points
into TCP goals; the backend plans the full paths, including clearance
between waypoints and the grasp constraints. A transfer waypoint alone does not
prove the path is collision-free. For nonvertical approach or other manipulation
policies, extend `MotionCompiler` with explicit typed/configured policy handling
and corresponding tests. Do not hide those changes in an ER prompt.

The mechanism owns the actual name-to-position mapping and its format. The
shared catalog and named motion request contain no `frame_id`, pose, joint array
or TCP reference for those positions. The server does not prescribe how the
controller stores or interprets them.

[robot_hardware.json](../examples/robot_hardware.json) contains only provisional
`xyz` and `quaternion` values for `home`. These are illustrative placeholders,
not a required position schema or safe robot coordinates. Replace them with your
mechanism's format during integration. The `HARDWARE INTEGRATION` comments in
`RobotBackend.__init__`, `resolve_targets` and `prepare` mark where to add loading,
interpretation and validation. A controller-native preset, existing controller
configuration or another private representation can replace the sample values.
The common `joint_targets` capability does not prescribe the representation of
backend-private named positions.

The agent and bridge never read this backend file. Resolve each selected arm's
name and freeze its validated goals/trajectories during `prepare`; execute that
snapshot in `move` without rereading mutable coordinates. A missing backend name
must fail before any arm moves, even if the shared catalog advertises it.

The backend must honor `reference: "tcp"`, using its configured active TCP or
end-effector interface. If the controller accepts only flange targets, perform
TCP-to-flange conversion there using its calibrated tool transform. Do not feed
a TCP target directly to a flange-only controller. Keep tool calibration fixed
for a prepared sequence. Explicit flange poses still use `reference: "flange"`.
Capture-time camera/flange TF remains in the bridge for wrist image geometry;
that is separate from converting commanded TCP poses to flange poses.

For migration, move the old shared `named_positions` coordinate map into your
backend, replace it with `position_names` name-description maps in shared configuration, and
remove `tcp_offset`/`tcp_orientation` from common profiles. Those old common
fields are rejected instead of silently applying a second tool transform.

## Implementing the backend

Start from [examples/primitive_driver.py](../examples/primitive_driver.py).
The template runs a real `RobotRequest` service through `driver_node`, but its
hardware hooks explicitly fail until implemented. It does not publish fake
completion telemetry or automatically drive hardware. The common runtime still
starts only gateway and bridge; start your adapter separately.

```bash
source /opt/ros/jazzy/setup.bash
source server/.venv/bin/activate
source server/install/setup.bash
python server/examples/primitive_driver.py --config server/configs/primitives.json \
  --hardware-config server/examples/robot_hardware.json
```

Implement these asynchronous `HardwareBackend` hooks in your ROS package:

| Hook | Inputs | Successful return |
|---|---|---|
| `prepare` | All TCP/flange/named `steps`, `arm_ids`, `coupled`, context | `{"success":true}` after validating the entire group and paths, without moving |
| `move` | Original TCP/flange/named `targets`; execute the locally prepared plan, `arm_ids`, common `duration`, `coupled`, context | Same, after measured arrival and settling for every arm |
| `gripper` | Per-arm `openings`, `arm_ids`, context | Same, after actual opening/closing/contact completion |
| `stop` | Affected `arm_ids`, context | Same, after physical stop; callable while other hooks are awaiting completion |
| `recover` (optional) | `arm_ids`, context | Same, after securing/releasing payload, fault handling, retreat/home |

The wrapper validates results and adds `coordinated:true, completed_arm_ids:[...]`
only for a backend's boolean success. Backend success must mean completion of
**every** listed arm. Partial completion must return `success:false`, `error`,
and optionally `code`, `failed_arm_ids`, `recoverable`, `outcome:"unknown"`.
Do not return an action acceptance as success. Use `BridgeError` for explicit
HTTP-style errors; unexpected backend exceptions become unknown failures.

Declare `coordinated_motion`, `coordinated_gripper`, `coupled_transfer`,
`joint_targets` and `recovery` only for implemented capabilities. They default
to false. A two-arm plan requires synchronized motion, grippers and preservation
of shared-object constraints. Unsupported sequences are rejected before the
first physical step. Each adapter hook receives the whole group; implementing
it as two unrelated serial calls does not satisfy the capability declaration.

`CommandContext.deadline` is local monotonic time. Check `context.check()` during
controller waits and honor cancellation/deadlines in the actual controller.
Return controller futures asynchronously; do not block the ROS executor.
The wrapper checks admission and completion deadlines and makes concurrent stop
cancel pending contexts. A wrapper cannot forcibly terminate arbitrary backend
code; the hardware stop hook must interrupt the active controller itself.

Publish fresh **measured** telemetry throughout and after execution. The bridge
checks new state, faults and stationary dwell after each step and after the whole
operation. Closing/lifting checks a negative object sensor; opening checks that
an available sensor no longer reports a held object. Unknown sensors remain
unknown, and visual verification still belongs to the agent. Use the optional
[completion monitor](time-budgets.md#driver-completion-integration) or equivalent
controller checks; joint-target arrival requires the controller's joint feedback.

## Why state, prepare and recover are separate

A state query reports what is true **now**: stopped/moving, faults, measured
pose and gripper state. It cannot establish that an upcoming goal is reachable,
that every named goal exists, or that both arms can follow a shared-object path.

`prepare` validates the **requested future sequence** without moving. For a
single-arm home, it can resolve the name, check controller readiness and validate
the requested goal/path. For a two-arm pick, validate both arms and shared
constraints before opening either gripper. Keep the checks appropriate to the
controller; an existing planning API can supply them. Merely returning success
because `moving=false` misses missing names, limits and path constraints. The
common adapter also uses this step to reserve the sequence and reject modified,
repeated or out-of-order commands. A state read cannot replace that protocol.

`recover` is an **optional physical recovery operation** after successful stop.
A stopped arm may still hold a heavy object, be below a support surface, or have
a controller fault latched. Recovery may need support/release, a planned retreat,
controller fault reset and return home. Reading state does not perform those
actions. A no-motion recovery is valid only if the backend positively verifies
all required recovery conditions, including payload handling; stationary state
alone is insufficient. Unsupported recovery returns 501 and requires operator
intervention. Set the application's recovery attempt limit to zero to disable
automatic recovery attempts; failure handling still stops the robot.

## Sequence and stop protocol

`prepare` carries `sequence_id`, complete `steps`, `arm_ids`, `coupled`. Subsequent
`move`/`gripper`/`recover` requests carry exactly the prepared step plus
`sequence_id` and zero-based `step_index`. The wrapper admits one sequence,
rejects modified/repeated/out-of-order steps and retains a bounded history of
recent sequence IDs. This is a process-local replay guard, not durable exactly-once
execution after a restart. Never retry an uncertain physical operation.

Every step inherits the original remaining deadline. A failure prevents further
steps and requests group stop with the independent stop budget; the failure
includes `stop_result` and `operator_required`. Recovery remains explicit.
Service cancellation alone does not stop hardware.

`POST /v1/stop` accepts `arm_ids` or `all_arms:true`. An omitted selection
stops all arms. The removed `arm_id` selector is rejected. Mixed selectors are
rejected. During active group motion or a held/interrupted two-arm plan, a partial
request expands to the group. The remembered group survives plan invalidation
until successful placement or recovery. The response reports `requested_arm_ids`,
`affected_arm_ids`, `stopped_arm_ids` (empty if stop is not confirmed), and
`plans_invalidated:true`.
Stop stays outside the normal action lock and retains its own short deadline.

A single-arm stop while idle does not claim its peer stopped. Clearing unknown
motion still requires confirmed all-arm stop and recovery. Older completion or
stop replies cannot supersede a newer interruption. The driver wrapper also
expands partial direct-service stop requests for its reserved/active group.

## Tests and extension points

`primitive_test.py` uses the actual wrapper with a measured-state mock backend,
real ROS services/topics and HTTP. It covers target kinds, TCP pose preservation, backend-owned named snapshots,
capability rejection, expired/wrong-wrist captures, failure/timeout/replay,
phase dwell, group stop and Gemini mock success/recovery flows. Agent
`unified_motion_test.py` covers selection, common routes and interruption.

Search `HARDWARE INTEGRATION` in `primitive_driver.py`, `motion.py` and the example
for backend/policy configuration points. All ROS integration tests use
`FakeDriver` with the real `PrimitiveAdapter` and a measured-state mock backend.
Its synthetic feedback is not physical completion logic. No dedicated arm-state ROS message type is introduced.

## Adding and exposing tools

Replacing a mechanism usually needs configuration, sensor adapters and backend
hooks only. Named positions keep their private controller format; advertise
only name-description metadata. New high-level tools can compose existing
HTTP calls without adding a driver operation. ER prompt changes stay in the
agent; see [prompt customization](../../docs/pixel-workflow.md#prompts-and-application-instructions).

| Change | Implementation points |
|---|---|
| Gemini-visible function or arguments | [tools.py](../../agent/embodiment/ros2/tools.py): `ros2_tools` explicitly returns the declarations. No function is automatically exposed; there is no per-tool configuration switch |
| Agent execution and motion guards | [ros2_embodiment.py](../../agent/embodiment/ros2/ros2_embodiment.py): register dispatch and classify motion in `MOTION_TOOLS`; preserve plan invalidation, recovery and stop interruption |
| Composed behavior or HTTP call | [manipulation.py](../../agent/embodiment/ros2/manipulation.py) / [robot_client.py](../../agent/embodiment/ros2/robot_client.py); reuse existing endpoints and validate arguments |
| Original visual evidence | [live_session.py](../../agent/embodiment/ros2/live_session.py): deliver the identified original image before the tool response; preserve evidence lifetime and consumption |
| New HTTP/ROS request | [api.py](../src/ros2_agent_server/ros2_agent_server/api.py), [protocol.py](../src/ros2_agent_server/ros2_agent_server/protocol.py), and typed bodies in [motion.py](../src/ros2_agent_server/ros2_agent_server/motion.py) or [workflow.py](../src/ros2_agent_server/ros2_agent_server/workflow.py) |
| New bridge behavior or phase policy | [robot_node.py](../src/ros2_agent_server/ros2_agent_server/robot_node.py), `MotionCompiler` and [sequencer.py](../src/ros2_agent_server/ros2_agent_server/sequencer.py); explicitly define selected arms, health checks, deadlines and completion |
| New primitive capability | [PrimitiveAdapter](../src/ros2_agent_server/ros2_agent_server/primitive_driver.py) validation/capability checks plus the backend hook; needed only when existing primitives cannot express the operation |
| New telemetry or camera geometry | [models.py](../src/ros2_agent_server/ros2_agent_server/models.py), [state.py](../src/ros2_agent_server/ros2_agent_server/state.py), bridge callbacks and [pixels.py](../src/ros2_agent_server/ros2_agent_server/pixels.py); define freshness and provenance |
| New topic/service names | [ros_names.py](../src/ros2_agent_server/ros2_agent_server/ros_names.py); installation overrides remain in `server.remappings` |

Tool visibility and execution permission are separate. Removing a declaration
does not remove its dispatch handler. Reject prohibited calls or arguments in
the agent dispatch as well. For example, `move` declares only pixel/named targets
and rejects direct poses before HTTP; programmatic HTTP clients retain pose
support. Without a wrist camera, `ros2_tools` omits `approach_targets` and
`refine_grasp`. Keep declaration conditions, runtime checks, lifecycle hints,
[system instructions](../../agent/embodiment/ros2/instruction.md) and
[application tasks](../../agent/apps) consistent.

For a new operation, define triggers, participating arms, inputs, results and
failure/retry semantics before adding dispatch. HTTP and direct ROS requests
must share strict validation, including extra fields, finite values and known
resource IDs. Workflow `BODIES` currently validates arm IDs in `targets`,
`observations` and `arm_id`; other payload shapes need explicit checks.
Read-only operations need an explicit branch in `_request` or the guarded
plan section of `_workflow`. The remaining workflow path admits motion; adding
validation alone does not implement a safe execution path. Motion selection
comes from `arm_ids`/`all_arms` or the plan's targets, not a legacy per-arm
`resource_id` fallback. Derive the new operation's scope explicitly and keep
preflight, measured-state checks, one shared deadline and responsive group stop.

Update the [HTTP contract](../../docs/http-contract.md),
[tool results](../../docs/tool-results.md) and [workflow](../../docs/pixel-workflow.md)
alongside the implementation. Search `HARDWARE INTEGRATION` / `TOOL EXTENSION`
comments to locate the extension boundaries. Copied upstream session/dispatch
code normally needs no changes. State publishers keep standard String/JSON;
a new JSON operation usually does not require changing `RobotRequest.srv`.

Use `api_test.py` for HTTP/validation, `primitive_test.py` for backend phases and
arm scope, `scenario_test.py` for response-driven Gemini/HTTP/ROS flows, and agent
tests beside the ROS2 modules for declarations/dispatch. Cover completion,
explicit failure, unknown timeout, partial group failure, stop during motion,
recovery with new evidence, and one-arm operation in both topologies. See
[testing commands](../README.md#tests) and [deadline integration](time-budgets.md).

New package modules are discovered by `setup.py`; installed configuration/launch
files and entry points require explicit package registration. Declare actual
runtime dependencies in the owning package, keeping controller dependencies
in the backend package. Rebuild and source the install using the
[server build commands](../README.md#setup-and-startup), then restart affected
processes. Source edits alone do not update a copied install. Rebuild interface
consumers as well if the ROS service definition changes.
