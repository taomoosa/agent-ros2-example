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

## Setup and startup

The reference environment is Ubuntu 24.04, ROS2 Jazzy, and Python 3.12.
Install ROS2 at the OS level; `rclpy` does not need to be installed with pip.
Building requires colcon, ament_cmake, ament_python,
rosidl_default_generators, sensor_msgs, and std_msgs.

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

`--config` accepts the included single-arm or dual-arm configuration, or an agent
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
  "state_max_age": 2.0,
  "camera_timeout": 1.5
}
```

Omitting the `server` object uses the values above. Standard ROS2 topic remapping
is also supported. Arm and camera IDs become topic name segments, so use ASCII
letters, digits, and underscores, starting with a letter or underscore.

## ROS2 connections

| Connection | Type | Content |
|---|---|---|
| Subscribe to `/robotics/arms/{arm_id}/state` | `std_msgs/msg/String` | Arm state JSON shown below |
| Subscribe to `/robotics/cameras/{camera_id}/image/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG, optical frame, and acquisition timestamp |
| Serve `/robotics/request` | `ros2_agent_interfaces/srv/RobotRequest` | State, image, and command requests from the HTTP gateway |
| Call `/robot_driver/execute` | `ros2_agent_interfaces/srv/RobotRequest` | Commands for a hardware-specific driver |

Publish the following JSON object in the `data` field of each arm's state topic
at regular intervals:

```json
{
  "moving": false,
  "flange_pose": {
    "frame_id": "world",
    "position": [0.4, 0.0, 0.3],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "gripper": {"opening": 1.0}
}
```

State subscriptions use reliable QoS with depth 1. Image subscriptions use
sensor-data QoS with best-effort reliability. `/v1/state` returns 503 if any arm
has no state or its last update was received more than `state_max_age` seconds
ago. State freshness is measured from receipt time; drivers must publish current
state.

An image's `format` must be `jpeg` or a compressed transport format identifying
JPEG. Its `header.frame_id` must match the configured `optical_frame`, and
`header.stamp` must contain the acquisition time. The server returns only images
received after the HTTP request and captured at or after that request.
Publishers and the server must use the same ROS clock, and timestamps must
increase monotonically for each camera. Invalid JPEGs, incorrect frames, and
old or duplicate timestamps are rejected. If no fresh image arrives, the server
returns 504 rather than a cached image. Captures across cameras are not strictly
synchronized.

## Internal service and driver interface

[RobotRequest.srv](src/ros2_agent_interfaces/srv/RobotRequest.srv) defines a shared
request and response type.

| Request field | Meaning |
|---|---|
| `operation` | `state`, `camera`, `move_arm`, `set_gripper`, or `stop` |
| `resource_id` | Camera ID for camera requests; arm ID for move/gripper requests; empty for state/stop |
| `payload_json` | JSON object matching the HTTP body; `{}` for state/camera |
| `timeout_sec` | Response deadline in seconds, greater than 0 and at most 65 |

| Response field | Meaning |
|---|---|
| `status_code` | HTTP-style status code |
| `payload_json` | JSON object; image responses include `frame_id` and `stamp_ns` |
| `data` | JPEG bytes; empty for ordinary command responses |
| `content_type` | `application/json` or `image/jpeg` |

Drivers handle `move_arm`, `set_gripper`, and `stop`. The stop target is the
`arm_id` field in `payload_json`; null means all arms.

Return `status_code=200` and `{"success": true}` **after the command completes**.
Report failure with `{"success": false, "error": "..."}` or a 4xx/5xx status.
Responses that indicate acceptance without completion, such as 202, are
converted to 502. Timeouts return 504 with `outcome: "unknown"`. Requests are not
automatically retried. Cancelling a service wait does not stop physical motion;
the driver must handle stop requests separately.

Hardware-specific MoveIt, action, controller, and TF integrations are not
included. The driver is responsible for transforming flange targets, planning,
collision checks, speed and force limits, and stopping active motion. Commands
return 503 when no driver service is connected. The test driver in
`tests/helpers.py` is not used by the runtime nodes.

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
No hardware or Gemini API key is required.

The asynchronous design follows the [ROS2 callback group documentation](https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html).
HTTP unit tests follow the [FastAPI async testing documentation](https://fastapi.tiangolo.com/advanced/async-tests/).

## License

The code in `server/` was added in this repository and is provided under Apache
License 2.0. See the [license text](../third_party/robotics-samples-LICENSE) and the
[project README](../README.md) for attribution of the upstream agent code.
