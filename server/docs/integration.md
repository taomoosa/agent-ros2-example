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
| Camera inputs | JPEG, depth/CameraInfo/TF capture, or configured fixed-plane projection | Supply registered RGB-D and image-time TF for depth mode; for plane mode, supply the calibrated JPEG stream and offline coefficients |
| Hardware commands | `RobotRequest` client and completion/failure checks | Implement a driver adapter service that calls your existing controllers, actions or MoveIt integration |
| Physical manipulation | Target compiler, TCP/tabletop policy and common phase executor | Configure task orientation/clearances and position_names in `server.hardware`; register named coordinates/TCP calibration and implement controller behavior in your backend |
| Tests | Mock driver/Gemini with real ROS2 communication | Add tests for your adapter and validate its physical behavior on your system |

The runtime does not start a hardware driver. [FakeDriver](../tests/helpers.py)
is a test fixture, not a production driver or a hardware simulator launched by
the server. Keep robot-specific integration in a separate ROS package whenever
it can implement the existing topics and service. Tool extensions are covered
in [Adding tools and ROS capabilities](primitive-adapter.md#adding-and-exposing-tools). Measured String/JSON state and
publisher updates are covered in [telemetry](telemetry.md); camera pairing, geometry
aliases and diagnostic logs are covered in [synchronization](synchronization.md).

For the smaller hardware integration surface, use
[common primitive execution](primitive-adapter.md): configure `server.hardware`
with task geometry and advertised position_names, then implement the basic
hardware hooks. There is one execution path for all configurations.

## 1. Set the topology and connection settings

For one fixed camera and one arm, this topology is sufficient for observation.
For homing and pixel manipulation, also configure task profiles and advertised
position names using [hardware configuration](primitive-adapter.md#hardware-configuration),
and register actual positions in the backend:

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
    "camera_timeout": 5.0
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
| Camera `optical_frame` | Match the RGB header and the color optical frame used for capture-time TF |
| Camera `depth_frame` / `camera_info_frame` | Declare each input header frame; omitted/null values default to `optical_frame`. Aliases do not register depth or transform its values |
| Camera `projection` / `plane_calibration` | Default `depth`; fixed cameras may select `plane` with an offline homography, image grid, valid region and plane pose. See [plane setup](plane-projection.md) |
| Camera `depth_geometry` | Only `color_optical_z` is supported: color-grid pixels with depth along the color optical Z axis |
| Camera `sync_tolerance_sec` | Allowed RGB/depth acquisition difference, 0..0.1 seconds, default 0.01. Tune to the measured timing and positional error; use 0 for strict pairing; see [pairing constraints](synchronization.md) |
| `server.namespace` | Prefix for bridge topics and its `request` service; keep both server roles consistent |
| `server.driver_service` | Absolute name of your adapter's `RobotRequest` service |
| `server.remappings` | One map of original absolute ROS names to hardware endpoint names, shared by both server roles; see [ROS names](ros-names.md) |
| Camera numerical settings / `server.future_skew_tolerance_sec` | See [numerical tolerances](numerical-tolerances.md) for calibration, depth quality and clock-skew limits |
| `server` timing fields | Configure motion, settling, state, transport and model deadlines together; see [time budgets and driver monitor](time-budgets.md) |
| Camera `camera_info_mode` | Select legacy rectified K or standard rectified P; see [CameraInfo integration](camera-info.md) |
| `server.state_max_age` | Maximum state measurement and receipt age in seconds, greater than zero and at most 60; publish faster than this with margin |
| `server.camera_timeout` | Wait budget for image/observation/capture in seconds, greater than zero and at most 3600; default 5 |
| `server.camera_buffer_size` | Entries retained per RGB/depth buffer, 2..256, default 32; allow for delayed and reordered delivery |
| `server.camera_max_age` | Maximum exposure age in seconds, greater than zero and at most 10; default 2. Both exposures must also be at or after the request |
| `server.state_completion_timeout` | Wait for measured state after driver acknowledgement, greater than zero and at most 3600 seconds, default 5; also bounded by the original operation deadline |

Frame IDs must be nonempty and have no leading slash. Mount declarations do not
publish TF or calibrate cameras. Use `server.hardware.profiles` for tabletop task geometry and `position_names`
for a name-to-description catalog without coordinates. Named coordinates, TCP calibration, controller limits
and recovery belong to the backend; see its separate configuration example. Other unknown fields are rejected by [RobotConfig](../src/ros2_agent_server/ros2_agent_server/models.py).

## 2. Provide telemetry, images and capture geometry

With the default depth-based minimal configuration, publish these inputs. For
`projection: "plane"`, only arm state and the calibrated fixed-camera JPEG are
required here; see [plane configuration and calibration](plane-projection.md).

| ROS connection | Type | Required content |
|---|---|---|
| `/robotics/arms/arm/state` | `std_msgs/msg/String` | Measured arm/gripper state; reliable publisher compatible with a depth-1 subscription |
| `/robotics/cameras/overhead/image/compressed` | `sensor_msgs/msg/CompressedImage` | Original rectified JPEG with optical frame and actual acquisition timestamps |
| `/robotics/cameras/overhead/camera_info` | `sensor_msgs/msg/CameraInfo` | Rectified image intrinsics selected by camera_info_mode; matching dimensions/header, no cropped ROI or downsampling. See [CameraInfo modes](camera-info.md) |
| `/robotics/cameras/overhead/depth/aligned` | `sensor_msgs/msg/Image` | Measured color-grid/color-Z depth, configured header frame, matching dimensions and stamps within the configured tolerance; `16UC1` millimetres or `32FC1` metres |
| `/tf`, `/tf_static` | TF | Camera-to-world transform (`world <- optical`) available at RGB acquisition time; wrist captures also require flange-to-world (`world <- flange`) at that time |

Camera subscriptions use sensor-data QoS (best effort, volatile). Publish
CameraInfo repeatedly or after bridge startup: a one-time publication before
the volatile subscription exists is insufficient. Calibration need not have
the same stamp as each exposure, but must describe the current image geometry.

Publish the following JSON shape in `String.data`. Replace the illustrative
`stamp_ns` with the actual measurement time in the shared ROS clock; see
[telemetry](telemetry.md) for the full publisher contract:

```json
{
  "stamp_ns": 123000000456,
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

Map the measured gripper range to `opening` in `0..1` (closed to open), or
use null/omit the field if unavailable. The `gripper` object is required. Populate
arm/gripper `fault` and `object_detected` from real controller/sensor data when
available; use null for unavailable readings. See the
[fault fields and recovery contract](../../docs/tool-results.md#recovery-and-retry-limits).
Repeated or older measurement stamps do not refresh state. Your adapter must
detect upstream telemetry loss and never restamp cached values. JointState
alone does not carry the full arm/gripper/fault contract. String JSON
requires an actual `stamp_ns` on the existing state topic; see
[the publisher update procedure](telemetry.md#updating-an-existing-publisher).

Keep persistent topic/service overrides together in `server.remappings`,
including TF names if needed; see the [complete example](ros-names.md).
Existing CLI remaps also work without editing the bridge. For example,
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

Plain `/image` and identity-bearing `/observation` use JPEG only. Grasp
inspection uses `/observation` and requires no depth or TF. Detection and
refinement use `/capture`. Its default depth mode requires RGB-D and image-time
TF; fixed cameras may instead use an offline calibrated plane. One fixed
RGB-D camera is sufficient for minimal pick/place if it shows the grasp clearly;
a wrist camera is optional and can be RGB-only for inspection. Monocular depth
inference is not implemented. See [capture geometry](../../docs/pixel-workflow.md#capture-geometry)
for projection and snapshot lifetime details.

## 3. Implement the hardware driver adapter

Create a ROS node providing `ros2_agent_interfaces/srv/RobotRequest` at
`server.driver_service`. Declare `rclpy`, `ros2_agent_interfaces` and the actual
controller/action dependencies in your package's `package.xml`. Build against
the sourced server interface package. Your driver can live in its own workspace;
if placed under `server/src/`, the README's colcon build discovers it. No changes
to the generic server packages are needed for a conforming adapter.

Use the [driver template](../examples/primitive_driver.py), backed by
`PrimitiveAdapter`. Implement these asynchronous `HardwareBackend` hooks:

| Hook | Input | Required result |
|---|---|---|
| `prepare` | All steps including TCP/flange/named goals, arm IDs, coupled flag, deadline/cancellation context | Validate the entire sequence and reserve controller resources before any motion |
| `move` | TCP/flange poses or named goals, arm IDs, duration, coupled flag, context | Plan and execute the group; confirm every target from measured feedback |
| `gripper` | Per-arm normalized openings, arm IDs, context | Operate and confirm every gripper; closed jaws alone do not prove a grasp |
| `stop` | Resolved group arm IDs, context | Interrupt pending work and confirm actual stopping |
| `recover` (optional) | Arm IDs, context | Support/release payload safely, clear eligible faults, retreat and home; fail explicitly if unsupported |

Hooks return `{"success": true}` only after their work completes. The adapter
adds `coordinated: true` and `completed_arm_ids` to the ROS response. Declare
only capabilities the controller actually supports. See the
[full hook and sequence contract](primitive-adapter.md) for payloads, preflight,
replay protection, joint targets and coupled motion constraints.

Detection and refinement call ER inside the agent. The bridge resolves frozen
pixel geometry, maintains plans/evidence, and compiles approach/pick/place into
primitive phases using configured tabletop geometry. The hardware
backend receives TCP/flange poses or names, not images, pixels or `execute_plan` requests.
`reset_arms` is an agent convenience for `/v1/move` with the named `home` target.
`recover_arms` remains a separate workflow; a simple home move cannot recover a
loaded or faulted mechanism.

Report failures with `success: false` and `error`, optionally `code`,
`failed_phase`, `failed_arm_ids`, `recoverable` and `outcome`. Use
`outcome: "unknown"` if physical completion is uncertain. Publish current
measured telemetry throughout execution and after reporting completion; a fault, stale
state or negative object-detection reading can prevent the next operation.

Await actual action/controller completion before returning success. Keep stop
and telemetry callbacks runnable while motion is pending; do not block the
driver executor waiting synchronously on another callback. On partial dual-arm
failure, stop the group. The bridge reserves post-acknowledgement verification
time before sending the remaining driver budget. Honor both `timeout_sec` and
`deadline_ns` in your execution policy, using the smaller remaining allowance;
see [time budgets](time-budgets.md) and
[completion telemetry](telemetry.md#freshness-and-command-completion).
Expiry or cancellation of a ROS service future does not stop the robot. The bridge never
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
containing `capture_id`, `image_base64` and acquisition timestamp. Depth captures
include camera pose; plane captures contain frozen `plane_calibration` instead.
A successful state/image check does not establish that the driver or pixel
geometry is ready; verify service presence and `/capture` separately.

| Symptom | Check |
|---|---|
| State returns 503 | All configured arms publish valid current state; reliable QoS is compatible |
| Image returns 504 | New JPEGs arrive after the request, correct optical frame, fresh acquisition stamps and shared ROS clock; arrival order need not match timestamp order |
| Capture returns 504 while image works | Configured RGB/depth tolerance and frame mapping, delivered CameraInfo and image-time TF; inspect timeout `details` |
| Capture returns 422 | Depth mode: rectification, intrinsics, frame/dimension match and depth encoding. Plane mode: calibrated JPEG dimensions/frame |
| Plane capture returns 409 | CameraInfo geometry changed; verify/recalibrate and restart before reusing the plane |
| Plane creation returns 422 | Selected pixels must be inside the original image and calibrated valid region |
| Command returns 503 | Driver service name/type/discovery and valid fresh arm state; a subsequent motion may also require recovery |
| Motion returns 504 with `post_command_state_timeout` | Every target arm must publish a new sample measured at or after driver acknowledgement; check measurement timestamps, publish cadence and the remaining deadline |
| Group command returns 502 | Completion acknowledgement includes `coordinated: true` and every requested arm exactly once |
| Motion returns 409 | Active motion, faults, unknown previous outcome, invalid plan stage or required recovery; inspect the error and current state |

Run the [unit and integration tests](../README.md#tests), then validate your
adapter's completion, failure and stop behavior with its controller simulator
or test harness. Finally validate calibration, reachable points, grasp geometry,
physical stopping/recovery and synchronization on your hardware before running
the supplied pick/place application. Repository tests mock hardware and Gemini;
they cannot establish those physical properties.

For result fields, final visual assessment and the policy after a Gemini
connection loss or bridge restart, see [tool outcomes](../../docs/tool-results.md).
Bridge state is in memory; establish stopped, known hardware state before
restarting an application after an interrupted process.

## Locating hardware-specific additions

Use `rg -n 'HARDWARE INTEGRATION' server/src` to find code comments linked to
this guide. The main integration points are:

| Location | What to configure or implement |
|---|---|
| Camera fields in `models.py`, validation/projection in `pixels.py` | Declare actual input frames and pairing tolerance; supply upstream registration if the existing color-grid/color-Z contract is not met |
| `models.py: ArmState`, `RobotBridgeNode._arm_state` | Publish measured controller/gripper data and map faults; see the telemetry guide |
| `RobotBridgeNode._command` | Use `PrimitiveAdapter` with basic hardware hooks, actual completion and responsive stopping |
| `motion.py` / `primitive_driver.py` / `examples/primitive_driver.py` | Common target/phase compilation and the hardware hook template; see [primitive integration](primitive-adapter.md) |
| `server.hardware` / controller configuration | Keep tabletop offsets and position_names in shared settings; register actual named poses/joints and TCP calibration in the backend |
| `ros_names.py: RosNames` | Central name definitions and remap validation; use `server.remappings` for hardware names instead of editing node constructors |
| Package `setup.py` / `package.xml` | Install any added launch/config files, register entry points and declare actual adapter dependencies |

The runtime starts the two common server nodes only. A driver package under
`server/src/` is discovered by the full colcon build but still needs its own
startup command or launch file. Proposed driver files are not supplied hardware
implementations. Do not place hardware-specific controller imports in the agent.
