# Connecting a robot for the first time

This guide identifies what the integrator must configure or implement. Start
with the build and environment commands in the [server README](../README.md#setup-and-startup).
All commands below run from the repository root in that environment.

## Supplied components and integration responsibilities

| Component | Supplied here | What you must add or change |
|---|---|---|
| HTTP gateway and ROS bridge | FastAPI routes, asynchronous ROS requests, input validation, state cache, capture and plan lifecycle | Usually no source changes when hardware conforms to the existing contract |
| Robot topology | [Minimal](../configs/minimal.json), [single-arm](../configs/single_arm.json), [dual-arm](../configs/dual_arm.json) examples | Set arm/camera IDs and frames to match your installation; use matching topology in the agent |
| Arm telemetry | Validated JSON subscription per arm | Publish current controller/gripper state through an adapter |
| Camera inputs | JPEG, CameraInfo and aligned-depth subscriptions; capture-time TF lookup | Provide correctly registered RGB-D, calibration, timestamps and TF; adapt camera output if necessary |
| Hardware commands | `RobotRequest` client and completion/failure checks | Implement a driver adapter service that calls your existing controllers, actions or MoveIt integration |
| Physical manipulation | Group requests and phase descriptions | Implement home poses, TCP geometry, grasp orientation, clearances, limits, collision checking, synchronized execution and recovery in your driver |
| Tests | Mock driver/Gemini with real ROS2 communication | Add tests for your adapter and validate its physical behavior on your system |

The runtime does not start a hardware driver. [FakeDriver](../tests/helpers.py)
is a test fixture, not a production driver or a hardware simulator launched by
the server. Keep robot-specific integration in a separate ROS package whenever
it can implement the existing topics and service. Tool extensions are covered
in [Adding tools and ROS capabilities](extending.md).

## 1. Set the topology and connection settings

For one fixed camera and one arm, start with this complete configuration:

```json
{
  "robot_url": "http://localhost:8080",
  "world_frame": "world",
  "arms": [
    {"id": "arm", "base_frame": "arm_base", "flange_frame": "arm_flange"}
  ],
  "cameras": [
    {"id": "overhead", "optical_frame": "overhead_optical", "mount": "world", "parent_frame": "world"}
  ],
  "server": {
    "namespace": "/robotics",
    "driver_service": "/robot_driver/execute",
    "state_max_age": 2.0,
    "camera_timeout": 1.5
  }
}
```

The `server` object is optional. The agent can load the same topology file.

| Setting | Integration action |
|---|---|
| `robot_url` | Set the URL reachable from the agent host; server binding is separately controlled by `--host` and `--port` |
| `world_frame`, arm frames | Match your TF/controller frames; positions use metres and orientations use unit quaternions in `[x,y,z,w]` order |
| Arm and camera `id` | Use consistent IDs in agent configuration, topics and driver responses; valid ROS name segments only, without dots or hyphens |
| Camera `mount` / `parent_frame` | Fixed: `world` and the configured world frame. Wrist: `flange`, the owning arm's flange frame, and its `arm_id` |
| Camera `optical_frame` | Match RGB, depth and CameraInfo headers and the calibrated TF optical frame |
| `server.namespace` | Prefix for bridge topics and its `request` service; keep both server roles consistent |
| `server.driver_service` | Absolute name of your adapter's `RobotRequest` service |
| `server.state_max_age` | Maximum state receipt age in seconds, greater than zero and at most 60; publish faster than this with margin |
| `server.camera_timeout` | Wait budget for a fresh capture in seconds, greater than zero and at most 5; size it for camera cadence and TF delivery |

Frame IDs must be nonempty and have no leading slash. Mount declarations do not
publish TF or calibrate cameras. Home poses, TCP offsets, grasp orientation,
trajectory limits and recovery destinations belong in **your driver package's
configuration**. There are no fields for them in this server configuration;
unknown fields are rejected by [RobotConfig](../src/ros2_agent_server/ros2_agent_server/models.py).

## 2. Provide telemetry, images and capture geometry

With the minimal configuration, publish these inputs:

| ROS connection | Type | Required content |
|---|---|---|
| `/robotics/arms/arm/state` | `std_msgs/msg/String` | Current arm/gripper JSON; reliable publisher compatible with a depth-1 subscription |
| `/robotics/cameras/overhead/image/compressed` | `sensor_msgs/msg/CompressedImage` | Original rectified JPEG with optical frame and increasing acquisition timestamps |
| `/robotics/cameras/overhead/camera_info` | `sensor_msgs/msg/CameraInfo` | Rectified pinhole `K`, matching frame and dimensions, zero distortion, no cropped ROI or downsampling |
| `/robotics/cameras/overhead/depth/aligned` | `sensor_msgs/msg/Image` | Measured depth registered to RGB, exactly matching RGB stamp, frame and dimensions; `16UC1` millimetres or `32FC1` metres |
| `/tf`, `/tf_static` | TF | World-to-camera transform chain available at the image acquisition time; wrist captures also require world-to-flange at that time |

Camera subscriptions use sensor-data QoS (best effort, volatile). Publish
CameraInfo repeatedly or after bridge startup: a one-time publication before
the volatile subscription exists is insufficient. Calibration need not have
the same stamp as each exposure, but must describe the current image geometry.

A complete arm-state message's `data` JSON can be:

```json
{
  "moving": false,
  "flange_pose": {
    "frame_id": "world",
    "position": [0.4, 0.0, 0.3],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "gripper": {"opening": 1.0, "fault": null, "object_detected": null},
  "fault": null
}
```

Map the physical gripper range to `opening` in `0..1` (closed to open). Populate
arm/gripper `fault` and `object_detected` from real controller/sensor data when
available; use null for unavailable readings. See the
[fault fields and recovery contract](../../docs/tool-lifecycle.md#failure-detection-and-controlled-retries).
Publishing an old measurement repeatedly would pass the receipt-age check, so
your adapter must detect upstream telemetry loss and avoid claiming fresh state.
JointState alone is not the arm-state JSON interface.

Existing topic names can be remapped without editing the bridge. For example,
if your camera already provides the required message content:

```bash
python -m ros2_agent_server --config server/configs/minimal.json --ros-args \
  -r /robotics/cameras/overhead/image/compressed:=/camera/rectified/image/compressed \
  -r /robotics/cameras/overhead/camera_info:=/camera/rectified/camera_info \
  -r /robotics/cameras/overhead/depth/aligned:=/camera/rectified/depth/aligned
```

Remapping changes names, not message types, pixels, header frames, calibration
or timestamps. Add an upstream adapter for raw images, distortion correction,
depth registration or incompatible state messages. Preserve actual acquisition
times; do not relabel unsynchronized depth with the RGB stamp to make it pass.
Publishers, TF and server must share the ROS clock. For simulation, pass
`--ros-args -p use_sim_time:=true` to both server roles and provide `/clock` to
all participating nodes. Request deadlines still expire while simulation time
is paused.

Plain image observation uses JPEG only. Pixel detection, refinement **and grasp
inspection** use `/capture`, which requires RGB-D and capture-time TF. One fixed
RGB-D camera is sufficient for the minimal pick/place workflow if it shows the
grasp clearly; a wrist camera is optional. Monocular depth inference is not
implemented. See [capture geometry](../../docs/pixel-workflow.md#capture-geometry)
for projection and snapshot lifetime details.

## 3. Implement the hardware driver adapter

Create a ROS node providing `ros2_agent_interfaces/srv/RobotRequest` at
`server.driver_service`. Declare `rclpy`, `ros2_agent_interfaces` and the actual
controller/action dependencies in your package's `package.xml`. Build against
the sourced server interface package. Your driver can live in its own workspace;
if placed under `server/src/`, the README's colcon build discovers it. No changes
to the generic server packages are needed for a conforming adapter.

Use the [service definition](../src/ros2_agent_interfaces/srv/RobotRequest.srv)
and implement the operations needed by your chosen application:

| Driver operation | `resource_id` / decoded `payload_json` | Required behavior |
|---|---|---|
| `move_arm` | Arm ID / `{frame_id, position, orientation, duration}` | Move the flange to a metric pose; resolve moving reference frames at request time |
| `set_gripper` | Arm ID / `{opening}` | Operate the gripper and detect failure |
| `stop` | Empty / `{arm_id: null}` for all, or a specific arm ID | Stop active motion even while another service request is pending |
| `reset_arms` | Empty / `{arm_ids, coordinated: true}` | Home all listed arms for normal startup |
| `move_arms` | Empty / `{moves, arm_ids, coordinated: true}` | Coordinate the requested flange poses; each move contains `arm_id` and the `move_arm` pose fields |
| `execute_plan` | Empty / `{plan_id, stage, targets, phases, coordinated: true}` | Execute one coordinated approach, pick or place stage; derive participating arms from `targets` |
| `recover_arms` | Empty / `{arm_ids, phases, coordinated: true}` | Secure/support any payload, release safely, retreat and home; recover only eligible faults |

For the supplied pick/place application, implement `stop`, `reset_arms`,
`execute_plan` and `recover_arms`. Manual motion tools additionally need
`move_arm`, `set_gripper` and `move_arms`. If an operation is unsupported, return
an explicit failure; successful intent/acceptance is not completion.

Agent tool names are not necessarily driver operation names. `detect_targets`
calls ER and creates a plan inside the bridge; it sends no detection command to
the driver. `approach_targets`, `pick_targets` and `place_targets` all become
`execute_plan`, with different stages. `state`, `camera`, `capture`,
`create_plan`, `refine_plan` and `verify_grasp` are handled inside the bridge.

An `execute_plan` target contains `arm_id`, `grasp` and `release`. Each point
contains `frame_id`, `position`, `capture_id`, `pixel` and `stamp_ns`. These are
projected world **contact points**, not flange poses. Your driver supplies tool
offsets, orientation, support/object offsets and approach clearance. The
[coordinated driver contract](../../docs/pixel-workflow.md#coordinated-driver-contract)
lists the exact phase arrays and dual-arm synchronization requirements. The
bridge sends a group request; physical synchronization belongs to the driver.

For completed `move_arm`, `set_gripper` or `stop`, return `status_code=200`,
`content_type="application/json"`, empty `data` and this `payload_json`:

```json
{"success": true}
```

For `reset_arms`, `move_arms`, `execute_plan` and `recover_arms`, completion must
list **every requested arm exactly once, even in a one-arm installation**:

```json
{"success": true, "coordinated": true, "completed_arm_ids": ["arm"]}
```

Report failures with `success: false` and `error`, optionally `code`,
`failed_phase`, `failed_arm_ids`, `recoverable` and `outcome`. Use
`outcome: "unknown"` if physical completion is uncertain. Publish current
telemetry throughout execution and when reporting completion; a fault, stale
state or negative object-detection reading can prevent the next operation.

Await actual action/controller completion before returning success. Keep stop
and telemetry callbacks runnable while motion is pending; do not block the
driver executor waiting synchronously on another callback. On partial dual-arm
failure, stop the group. Honor `timeout_sec` in your execution policy: expiry or
cancellation of a ROS service future does not stop the robot. The bridge never
replays a timed-out motion request. A retry after failed manipulation requires
successful all-arm stop and recovery, then a newly detected plan. Recovery must
not blindly open a loaded gripper; unsupported safe recovery must fail.

## 4. Bring up and verify the integration

Start your controllers, driver adapter, telemetry/camera publishers and TF, then
start the server using the matching configuration. Check transport and capture
before running an application that moves hardware:

```bash
ros2 service type /robot_driver/execute
ros2 topic info /robotics/arms/arm/state --verbose
ros2 topic info /robotics/cameras/overhead/image/compressed --verbose
curl --fail-with-body http://localhost:8080/v1/state
curl --fail-with-body http://localhost:8080/v1/cameras/overhead/image -o /tmp/overhead.jpg
curl --fail-with-body http://localhost:8080/v1/cameras/overhead/capture -o /tmp/overhead-capture.json
```

Expect the service type `ros2_agent_interfaces/srv/RobotRequest`, matched topic
endpoints/QoS, fresh state for every configured arm, a JPEG and capture JSON
containing `capture_id`, `image_base64`, camera pose and acquisition timestamp.
A successful state/image check does not establish that the driver or pixel
geometry is ready; verify service presence and `/capture` separately.

| Symptom | Check |
|---|---|
| State returns 503 | All configured arms publish valid current state; reliable QoS is compatible |
| Image returns 504 | New JPEGs arrive after the request, correct optical frame, increasing stamps and shared ROS clock |
| Capture returns 504 while image works | Exact RGB/depth timestamp pairing, delivered CameraInfo and TF available at that acquisition time |
| Capture returns 422 | Rectification, intrinsics, frame/dimension match and supported depth encoding |
| Command returns 503 | Driver service name/type/discovery; a subsequent motion may also require recovery |
| Group command returns 502 | Completion acknowledgement includes `coordinated: true` and every requested arm exactly once |
| Motion returns 409 | Active motion, faults, unknown previous outcome, invalid plan stage or required recovery; inspect the error and current state |

Run the [unit and integration tests](../README.md#tests), then validate your
adapter's completion, failure and stop behavior with its controller simulator
or test harness. Finally validate calibration, reachable points, grasp geometry,
physical stopping/recovery and synchronization on your hardware before running
the supplied pick/place application. Repository tests mock hardware and Gemini;
they cannot establish those physical properties.
