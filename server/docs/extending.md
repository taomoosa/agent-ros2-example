# Adding tools and ROS capabilities

Choose the smallest extension that expresses the desired behavior. Replacing
a robot while preserving the existing contract usually needs only configuration
and a driver adapter; see [first-time integration](integration.md).

## Choose the extension boundary

| Desired change | Files/components to change |
|---|---|
| Fixed-camera pixels on a known plane instead of depth | Configure [plane projection](plane-projection.md) and offline calibration; no new tool or ROS message type |
| Different home pose, gripper hardware, controller or recovery trajectory | Your driver package/configuration; keep the existing HTTP/ROS operation contract |
| Different topic/service names with compatible content | Keep overrides in `server.remappings`; see [ROS names](ros-names.md). Existing namespace/CLI settings remain supported |
| Different sensor message/geometry | Your upstream adapter; change server subscriptions/models only if intentionally extending the contract |
| Different task wording or ER contact-point guidance | Agent application/system instruction or ER prompt files; no server changes |
| New agent tool composing existing operations | Agent declaration, dispatch and implementation; reuse existing HTTP client calls and lifecycle guards |
| New telemetry exposed through state | Publisher adapter, server state model/storage, and any agent presentation/validation using that field |
| New HTTP operation or ROS capability | Request model/validation, route, bridge handler, optional driver implementation, agent tool/client if exposed to Gemini, tests and contract docs |
| Existing controller uses another standard ROS type | Map it in an adapter; keep the standard String state boundary. No custom state message is required |

Tool names, HTTP routes and ROS operation strings are distinct interfaces. For
example, `pick_targets` calls `POST /v1/plans/execute` with `stage="pick"`; the
bridge sends `operation="execute_plan"` with projected targets and phases to
the driver. You do not add a driver handler named `pick_targets`.

## Server source map

Paths below are relative to `server/` and link to the current implementation.

| File / symbol | When and what to change |
|---|---|
| [ros_names.py](../src/ros2_agent_server/ros2_agent_server/ros_names.py): `RosNames` | Define new connection names here; use its helpers in nodes and fixture publishers. Installation-specific overrides belong in the shared JSON map |
| [models.py](../src/ros2_agent_server/ros2_agent_server/models.py): `Model`, `RobotConfig`, `ArmState` | Add strict request/state/config fields, types and ranges. New config fields need compatible agent configuration parsing too |
| [workflow.py](../src/ros2_agent_server/ros2_agent_server/workflow.py): request classes, `BODIES`, `validate_workflow` | Add workflow request shapes and validate resource IDs, uniqueness, frames and bounds |
| [protocol.py](../src/ros2_agent_server/ros2_agent_server/protocol.py): `validate_request`, `Reply`, `BridgeError` | Register non-workflow operations and their resource/body validation; maintain consistent success/failure transport |
| [api.py](../src/ros2_agent_server/ros2_agent_server/api.py): `create_app` | Add the HTTP method/path, typed body and operation mapping through the shared `request` helper; set a deadline |
| [robot_node.py](../src/ros2_agent_server/ros2_agent_server/robot_node.py): subscriptions, `_request`, `_workflow`, `_check_health` | Add ROS input callbacks or execution dispatch; define motion admission, health checks, plan invalidation, completion, stop and recovery behavior |
| [state.py](../src/ros2_agent_server/ros2_agent_server/state.py): `StateStore` | Store and expose additional telemetry with a defined freshness policy |
| [pixels.py](../src/ros2_agent_server/ros2_agent_server/pixels.py): `PixelPlans` | Change capture geometry, plan stages, provenance checks or `next_actions` only when the workflow needs them |
| [gateway.py](../src/ros2_agent_server/ros2_agent_server/gateway.py): `HttpGatewayNode.request` | Usually unchanged: it transports generic operations asynchronously; change only if transport semantics change |
| [RobotRequest.srv](../src/ros2_agent_interfaces/srv/RobotRequest.srv) | Usually unchanged: new operation names and JSON fields fit the existing envelope |
| Your driver package | Implement new hardware operations and report actual completion/failure; map to controller topics/actions/services |

## Adding an operation from contract to execution

1. Specify its trigger, prerequisites, inputs/units, result, deadline and next
   action. Decide whether it only observes, computes a plan, or moves hardware.
   Define single-arm and dual-arm behavior, including whether a group must move
   simultaneously. Add the intended contract to the
   [HTTP contract](../../docs/http-contract.md) and
   [tool lifecycle](../../docs/tool-lifecycle.md).
2. Add a strict Pydantic body derived from `Model`, in `models.py` or
   `workflow.py`. Reject extra fields, invalid/non-finite values, unknown IDs,
   duplicate arms and unsupported frames. `BODIES` is for workflow requests
   with an empty `resource_id`. Its current generic validation understands
   `targets`, `observations`, `moves` and `arm_id`; a new shape needs explicit
   resource validation. Arm-ID checks do not automatically validate camera IDs
   or new pose fields.
3. Register validation in `validate_request` (or `BODIES` plus
   `validate_workflow`), then add the typed HTTP route in `create_app`. Both
   HTTP and direct `RobotRequest` callers must pass the same validation; a
   FastAPI-only check leaves the ROS service unprotected from invalid input.
4. Add bridge execution. For new read-only telemetry, use an explicit branch
   in `_request` alongside `state`/`camera`/`capture`/`observation`. For plan
   computation, add an explicit guarded branch in `_workflow`. **The remaining `_workflow`
   path treats requests as motion**: simply accepting a new operation in
   validation can send it to the driver and invalidate plans. Decide whether
   the new operation remains available during active motion or recovery.
5. For motion, define participating arm IDs and retain health/recovery checks,
   busy admission, stop interruption, invalidation and unknown-outcome handling.
   Preserve the post-acknowledgement measured-state wait for every target arm
   within the original deadline; see [completion telemetry](telemetry.md).
   A new grouped operation must enter the `grouped` classification and derive
   its actual requested arms explicitly; the existing fallback selects all
   configured arms. Add payload enrichment and exact per-arm completion checks.
   A new single-arm operation using payload IDs needs explicit handling too:
   the current non-group fallback expects the arm in `resource_id`.
6. Implement the operation in your driver if physical work is needed. Keep
   service waits asynchronous, report completion rather than action acceptance,
   and keep stop runnable. Define recovery for partial execution; do not add an
   automatic motion retry. The current internal timeout range is `(0, 15000]`
   seconds; align HTTP route, gateway, driver and agent budgets. Configure the shared operation budgets and propagate `deadline_ns`; see
   [time budgets](time-budgets.md). Do not increase only the driver timeout.
7. Expose the capability to the agent when needed, following the map below.
   Update prompts, application examples, lifecycle hints and tests together.

For example, adding a measured gripper-force field to existing state can use
`ArmState`/`GripperState`, the publisher adapter and state tests without a new
HTTP route or driver command. Define units and unavailable/stale readings.
A new force-limited closing command instead needs a strict command body, route,
bridge motion handling, driver completion/failure semantics and an agent tool
if Gemini is to request it. Neither capability is currently implemented.

## Agent changes when exposing a tool

| File | Required change |
|---|---|
| [tools.py](../../agent/embodiment/ros2/tools.py) | Declare Gemini function name, arguments, trigger and preconditions; expose it only for supported topology |
| [ros2_embodiment.py](../../agent/embodiment/ros2/ros2_embodiment.py): `execute_action`, `_execute_action` | Add dispatch and classify failures/motion consistently; preserve stop bypass of the action lock, plan invalidation and recovery gating |
| [robot_client.py](../../agent/embodiment/ros2/robot_client.py) | Add an HTTP call if there is a new endpoint; validate inputs and preserve driver errors and timeout uncertainty |
| [manipulation.py](../../agent/embodiment/ros2/manipulation.py) | Add composed manipulation behavior, plan/evidence lifecycle and bounded recovery when applicable |
| [live_session.py](../../agent/embodiment/ros2/live_session.py) | For a new visual-evidence tool, ensure the original image reaches Live before its tool response, with matching identity; returning base64 in JSON alone does not implement the current inspection delivery mechanism |
| [instruction.md](../../agent/embodiment/ros2/instruction.md), [application examples](../../agent/apps) | Tell Gemini when to call the tool, how to interpret failure and what to do next |

Register a new motion tool in `MOTION_TOOLS` in `ros2_embodiment.py` and review
its admission, invalidation and success/error branches. Adding only its dispatch
entry is insufficient. Keep the agent's local plan/recovery state consistent
with the bridge. For visual tools, preserve capture provenance and evidence
consumption so an old or undelivered image cannot authorize a new motion.

ER prompt customization already has CLI options and does not require a server
extension. See [custom prompt files](../../docs/tool-lifecycle.md#customizing-er-prompts-without-editing-source)
and [prompt locations](../../docs/pixel-workflow.md#prompts-and-application-instructions).
The ROS-specific agent files provide these extension points; copied upstream
session/tool-dispatch code normally needs no changes.

## Build, tests and completion checklist

New Python modules within `ros2_agent_server` are discovered by `find_packages`
in [setup.py](../src/ros2_agent_server/setup.py). Installed launch/config files
need entries in `data_files`; new CLI entry points need explicit registration.
New runtime imports require dependency declarations in
[package.xml](../src/ros2_agent_server/package.xml) and the applicable
[Python requirements](../requirements.txt). Keep controller-specific dependencies
in the driver package when only the driver imports them.

A new JSON operation does not require regenerating the service definition.
If introducing a new ROS type, update
[interface CMakeLists.txt](../src/ros2_agent_interfaces/CMakeLists.txt) and
[interface package.xml](../src/ros2_agent_interfaces/package.xml), then rebuild
the interface and dependent packages.

For a Python-only server change, rebuild and source the installed package:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base server/log build \
  --base-paths server/src --build-base server/build --install-base server/install \
  --packages-select ros2_agent_server \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source server/install/setup.bash
```

This assumes the initial full build has completed and its environment has been
sourced. Omit `--packages-select ros2_agent_server` for an interface/dependency
change. The documented build uses a copied install: restart both server roles
after rebuilding and sourcing, and rebuild/restart your driver when affected.
Editing `server/src/` alone does not update an already installed/running node.

| Verification | Existing test location to extend |
|---|---|
| Body/resource/frame validation, endpoint-to-operation mapping, propagated errors | [api_test.py](../tests/api_test.py) with a mocked gateway |
| State freshness, optional telemetry, capture geometry and plan transitions | [state_test.py](../tests/state_test.py), [pixels_test.py](../tests/pixels_test.py) |
| Direct ROS validation, driver payload, completion/failure/timeout, stop while pending | [ros_integration_test.py](../tests/ros_integration_test.py) and [helpers.py](../tests/helpers.py) |
| Coordinated workflow, partial failure and exact group acknowledgement | [pixel_integration_test.py](../tests/pixel_integration_test.py) |
| Agent + HTTP + ROS flow, minimal topology and recovery/re-detection | [tool_review_integration_test.py](../tests/tool_review_integration_test.py) |
| Gemini tool dispatch/schema, prompts and failure handling | Tests beside [agent ROS2 modules](../../agent/embodiment/ros2) |

Run both suites using the [server testing commands](../README.md#tests), with
Gemini and hardware mocked. For a motion extension, cover success, explicit
driver failure, ambiguous timeout, partial group completion, stop during a
pending request, recovery followed by a new plan, and one fixed camera with one
arm. Add new tests that exercise the changed behavior; mock success alone does
not establish that failures remain detectable or recoverable. Document any
additional hardware validation required by the new capability.

## Finding extension points in source

Search from the repository root:

```bash
rg -n 'HARDWARE INTEGRATION|TOOL EXTENSION' server/src agent/embodiment/ros2
```

`HARDWARE INTEGRATION` marks the state JSON contract and input boundary,
camera geometry configuration/projection assumptions, driver service boundary,
and package data installation. Implement hardware commands in your driver
package; a marker at `_command` does not mean bypassing common motion guards.

`TOOL EXTENSION` marks Gemini declarations, motion classification, HTTP client
and routes, direct ROS validation, workflow body registration, bridge admission,
plan/evidence composition and original-image delivery. Follow the file maps
above to update prompts, applications, deadlines and tests too. An observation-
only tool needs an explicit branch before `_workflow`'s motion fallback. RGB
observation IDs cannot be used as metric captures.

Examples to trace without adding dummy tools: `move_arms` for group motion,
`detect_targets` for ER plus geometric plan creation, and `inspect_grasp` for RGB
evidence followed by `verify_grasp`. Upstream copied dispatch code normally stays
unchanged. See [synchronization](synchronization.md) for camera extension contracts
and [telemetry](telemetry.md) before adding state fields. State publishing uses
standard String JSON; do not introduce a repository-specific state message.
