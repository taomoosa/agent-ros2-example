# ROS2 HTTP server

This ROS2 package is independent of the agent. It supports one or two arms and
multiple cameras and implements the [HTTP contract](../docs/http-contract.md).

```text
Gemini agent
    | HTTP
http_gateway node (FastAPI / Uvicorn)
    | /robotics/request service
robot_bridge node (rclpy)
    +-- subscribes to arm state and camera topics
    +-- calls /robot_driver/execute on a hardware-specific driver node
```

`http_gateway` handles HTTP requests and ROS2 service calls. `robot_bridge`
validates and caches state, waits for images captured after each request, and
forwards commands to a driver. Service waits are asynchronous, allowing topic
callbacks and stop requests to run while a capture or command is pending.
Both nodes run in one process by default; `--role` can separate them into two
processes.

For your first hardware installation, follow the
[integration guide](docs/integration.md): it identifies the configuration,
telemetry/camera adapters and driver operations you must supply, with a minimal
configuration and bring-up checks. For a fixed camera with unreliable depth,
[plane projection](docs/plane-projection.md) can use an offline calibration. When adding a capability, use the
[extension guide](docs/extending.md) for the exact server/agent files to change,
validation and motion lifecycle requirements, and tests to extend.

## Setup and startup

The reference environment is Ubuntu 24.04, ROS2 Jazzy, and Python 3.12.
Install ROS2 at the OS level; `rclpy` does not need to be installed with pip.
Building requires colcon, ament_cmake, ament_python,
rosidl_default_generators, sensor_msgs, std_msgs, and tf2_ros. The integration
tests also use geometry_msgs and TF broadcasters.

Run from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages server/.venv
source server/.venv/bin/activate
python -m pip install -r server/requirements.txt

colcon --log-base server/log build \
  --base-paths server/src --build-base server/build --install-base server/install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source server/install/setup.bash

python -m ros2_agent_server --config server/configs/single_arm.json --port 8080
```

HTTP listens on `127.0.0.1:8080` by default. Interactive API documentation is
available at `/docs`. To connect an agent from another host, configure `--host`
and the agent's `robot_url`.

`--config` accepts the included minimal, planar, single-arm or dual-arm configuration, or an agent
configuration in the same format. The server accepts `robot_url` for
compatibility, but its listen address is determined by `--host` and `--port`.

To run two processes, use separate terminals with the same environment setup:

```bash
python -m ros2_agent_server --role robot --config server/configs/dual_arm.json
python -m ros2_agent_server --role http --config server/configs/dual_arm.json --port 8080
```

An optional `server` object in the configuration can override these defaults:

```json
{
  "namespace": "/robotics",
  "driver_service": "/robot_driver/execute",
  "remappings": {},
  "state_max_age": 2.0,
  "camera_timeout": 5.0,
  "camera_buffer_size": 32,
  "camera_max_age": 2.0,
  "state_completion_timeout": 5.0
}
```

Omitting the `server` object uses the values above. Put installation-specific
topic and service overrides in `server.remappings`; both nodes read this one
map. See [central ROS name configuration](docs/ros-names.md) and the complete
[remapped example](configs/remapped.json). Standard ROS2 CLI remapping remains
supported and overrides matching JSON rules. Arm and camera IDs become topic name segments, so use ASCII
letters, digits, and underscores, starting with a letter or underscore.

## ROS2 connections

The following are default names before `server.remappings` is applied.

| Connection | Type | Content |
|---|---|---|
| Subscribe to `/robotics/arms/{arm_id}/state` | `std_msgs/msg/String` | Measured JSON arm state; [migration and fields](docs/telemetry.md) |
| Subscribe to `/robotics/cameras/{camera_id}/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG, optical frame, and acquisition timestamp |
| Subscribe to `/robotics/cameras/{camera_id}/camera_info` | `sensor_msgs/msg/CameraInfo` | Rectified calibration matching the JPEG geometry |
| Subscribe to `/robotics/cameras/{camera_id}/depth/aligned` | `sensor_msgs/msg/Image` | Color-grid/color-Z depth with configured header frame and timestamp tolerance |
| Listen to `/tf` and `/tf_static` | TF messages | Camera-to-world and, for wrist cameras, flange-to-world transforms at acquisition time |
| Serve `/robotics/request` | `ros2_agent_interfaces/srv/RobotRequest` | State, image, and command requests from the HTTP gateway |
| Call `/robot_driver/execute` | `ros2_agent_interfaces/srv/RobotRequest` | Commands for a hardware-specific driver |

Publish measured JSON in standard String messages periodically; [telemetry.md](docs/telemetry.md)
explains required measurement timestamps, unavailable readings and publisher
updates. Publish this JSON shape in `String.data`, replacing the illustrative
`stamp_ns` with the actual measurement time in the shared ROS clock:

```json
{
  "stamp_ns": 123000000456,
  "moving": false,
  "flange_pose": {
    "frame_id": "world",
    "position": [0.4, 0.0, 0.3],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "gripper": {"opening": 1.0}
}
```

The HTTP gateway client and bridge service use reliable, volatile service QoS
with history depth 64 to accommodate concurrent image-request bursts.
State subscriptions use reliable QoS with depth 1. Image subscriptions use
sensor-data QoS with best-effort reliability. `/v1/state` returns 503 if any arm has no valid state or its measurement or
receipt age exceeds `state_max_age`. Replayed measurement stamps cannot refresh
state. After a successful driver motion response, every participating arm must
provide a subsequent sample measured at or after acknowledgement receipt, and
pass stationary/fault checks. Stop relies on driver completion without this
telemetry wait. See [completion timing](docs/telemetry.md#freshness-and-command-completion).

An image's `format` must be `jpeg` or a compressed transport format identifying
JPEG. Its `header.frame_id` must match the configured `optical_frame`, and
`header.stamp` must contain the acquisition time. The server returns only images
received after the HTTP request and captured at or after that request.
Publishers and the server must use the same ROS clock. Bounded image/depth
buffers handle out-of-order delivery, while duplicate RGB stamps are ignored.
Frames must satisfy request-time freshness and maximum age. If no fresh image
arrives, the server returns 504 rather than a cached image. Captures across
cameras are not strictly synchronized. See [synchronization and diagnostic
configuration](docs/synchronization.md) for tolerance, frame aliases, TF history
and the depth-free `/observation` endpoint.

## Internal service and driver interface

[RobotRequest.srv](src/ros2_agent_interfaces/srv/RobotRequest.srv) defines a shared
request and response type.

| Request field | Meaning |
|---|---|
| `request_id` | Correlates HTTP, gateway, bridge and driver logs; rebuild all service consumers after updating the interface |
| `operation` | `state`, `camera`, `capture`, `observation`, `move_arm`, `set_gripper`, `stop`, `create_plan`, `refine_plan`, `execute_plan`, `verify_grasp`, `reset_arms`, `move_arms`, `recover_arms` |
| `resource_id` | Camera ID for camera/capture/observation requests; arm ID for move/gripper requests; empty for state, stop and workflow operations |
| `payload_json` | Gateway-to-bridge: validated HTTP fields. Bridge-to-driver: command payload, enriched with targets/phases/arm IDs as applicable; see the [driver operation map](docs/integration.md#3-implement-the-hardware-driver-adapter) |
| `deadline_ns` | Shared ROS-clock deadline; zero for direct legacy callers. Rebuild every interface consumer |
| `timeout_sec` | Response deadline in seconds, greater than 0 and at most 15000 |

| Response field | Meaning |
|---|---|
| `status_code` | HTTP-style status code |
| `payload_json` | JSON object; image responses include `frame_id` and `stamp_ns` |
| `data` | JPEG bytes; empty for ordinary command responses |
| `content_type` | `application/json` or `image/jpeg` |

Drivers handle `move_arm`, `set_gripper`, `stop`, `reset_arms`, `move_arms`,
`recover_arms`, and `execute_plan`. See the [pixel workflow](../docs/pixel-workflow.md) for
additional camera_info/aligned-depth/TF subscriptions, capture and plan payloads,
and mandatory group synchronization/completion semantics. The stop target is the
`arm_id` field in `payload_json`; null means all arms.

For `move_arm`, `set_gripper` and `stop`, return `status_code=200` and
`{"success": true}` **after the command completes**. Group operations
(`reset_arms`, `move_arms`, `execute_plan`, `recover_arms`) additionally require
`"coordinated": true` and `"completed_arm_ids"` listing every requested arm
exactly once, even for a single arm.
Report failure with `{"success": false, "error": "..."}` or a 4xx/5xx status.
Responses that indicate acceptance without completion, such as 202, are
converted to 502. Motion timeouts, including missing post-command telemetry,
return 504 with `outcome: "unknown"`. Camera input timeouts instead report
`code: "capture_timeout"` and the missing-input details. Requests are not
automatically retried. Cancelling a service wait does not stop physical motion;
the driver must handle stop requests separately.

Hardware-specific MoveIt, action, and controller integrations are not
included. Capture-time TF lookup and RGB-D projection are implemented by the bridge. The driver is responsible for transforming flange targets, planning,
collision checks, speed and force limits, and stopping active motion. Physical
commands return 503 when no driver service is connected. The test driver in
`tests/helpers.py` is not used by the runtime nodes.

The [tool lifecycle guide](../docs/tool-lifecycle.md) defines recovery after
failed motion, optional arm/gripper faults and object-detection telemetry, and
the one-fixed-camera configuration `configs/minimal.json`. Recovery is a
separate coordinated driver operation; returning home alone is not recovery.

## Processing time and standard camera input

Configure operation, settling, state and delivery allowances in the shared JSON;
see [time budgets and completion integration](docs/time-budgets.md). Stop has an
independent deadline. The agent stops before cleanup on interruption and enforces
elapsed deadlines even with mock transports. See [CameraInfo modes](docs/camera-info.md)
for standard rectified P and the legacy rectified-K adapter contract.

## Numerical acceptance

See [numerical tolerances](docs/numerical-tolerances.md) for configurable camera
calibration, RGB/depth timing, clock-skew and depth-quality limits. The default
pairing tolerance is now 10 ms; set it to zero for strictly synchronized inputs.
Depth projection now rejects unsupported local depth and values outside the
configured range, so review these settings when connecting hardware.

## Tests

After building and setting up the environment above, run from the repository
root:

```bash
python -m pip install -r server/requirements-test.txt
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/tmp/agent-ros2-server-logs
export PYTHONPATH="$PWD/agent:$PYTHONPATH"
python -m unittest discover -s server/tests -p '*_test.py' -v
python -m unittest discover -s agent -p '*_test.py' -v
```

Unit tests cover input validation, state freshness, camera images, and HTTP
responses. Integration tests use real ROS2 topics and services in a test domain,
with Gemini and the hardware-specific driver mocked. The dual-arm test uses ASGI
HTTP integration; the single-arm test runs the agent against Uvicorn over a real
TCP connection. Tests also cover command completion, stop during motion,
concurrent image requests, failures, timeouts, and CLI startup and shutdown.
Pixel workflow tests additionally cover RGB-D projection, capture-time wrist TF,
ER detection/refinement, Live-agent grasp assessment, and two complete coordinated manipulation
cycles through the Live agent and server. No hardware or Gemini API key is required.

The asynchronous design follows the [ROS2 callback group documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html).
HTTP unit tests follow the [FastAPI async testing documentation](https://fastapi.tiangolo.com/advanced/async-tests/).

## License

The code in `server/` was added in this repository and is provided under Apache
License 2.0. See the [license text](../third_party/robotics-samples-LICENSE) and the
[project README](../README.md) for attribution of the upstream agent code.
