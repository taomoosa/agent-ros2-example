# ROS2 Gemini agent

See the [project README](../README.md) for usage and attribution, and the
[HTTP contract](../docs/http-contract.md) for the server interface.

The Gemini Live session, event bus, observation handling, and tool execution
components come from `robotics-samples/live-api/agent`. `run_ros2.py` selects the
ROS2 embodiment and runs application instructions. This directory does not
include the browser UI or Spot/Tinybot drivers. The ROS2 HTTP server is maintained
separately in [server/](../server/README.md).

## Setup

Use Python 3.11 or later. The runtime dependencies are `httpx>=0.27.0`,
`pillow>=10.0.0`, and `websocket-client>=1.8.0`, which are also used upstream.
They are declared in [requirements.txt](requirements.txt).

```bash
cd agent  # From the repository root.
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run_ros2.py --help
```

For the common motion backend, use `configs/primitives.json` after configuring
its calibration and hardware profiles. It selects `move`/`gripper`/`stop` tools
with `arm_ids` or `all_arms`, while retaining plan/inspection/recovery tools.
See [target formats and driver integration](../server/docs/primitive-adapter.md).

For pixel-guided applications, see [the workflow guide](../docs/pixel-workflow.md).
A fixed camera can use [calibrated plane projection](../server/docs/plane-projection.md)
instead of depth; `configs/planar.json` is an illustrative shared configuration.
All configurations use the common motion adapter. The tools `move`, `gripper`
and `stop` share `arm_ids`/`all_arms` selection. For startup homing and pixel
manipulation, supply task profiles and the `server.hardware.position_names` catalog in
the shared configuration, and register actual named coordinates in the backend; [primitives.json](configs/primitives.json) illustrates
these fields. Topology-only examples do not define robot-specific home/TCP
geometry. See the [hardware adapter and migration guide](../server/docs/primitive-adapter.md).

For example, after starting a server and compatible driver:

```bash
python run_ros2.py --config configs/dual_arm.json --model "$GEMINI_LIVE_MODEL" \
  --robotics-model gemini-robotics-er-2-preview --task-file apps/dual_arm.md \
  --instruction "Grasp the opposite ends of the red bar and move it onto the tray."
```

`--model` controls Live orchestration; `--robotics-model` controls the separate
ER requests for detection and refinement. Grasp assessment belongs to the Live
agent: `inspect_grasp` supplies original wrist or fixed-camera RGB images and
arm state, then `verify_grasp` records its decisions. Both model connections
use `GEMINI_API_KEY`. The ER client uses the existing `httpx` dependency.

For one fixed camera and one arm, follow the
[minimal setup](../docs/pixel-workflow.md#minimal-setup); the topology-only
`configs/minimal.json` needs task profiles and position names for manipulation.
Inspection falls back to that fixed camera. Customize detection with
`--er-detect-prompt-file prompt_examples/grasp_guidance.md` and set the recovery
budget with `--max-recovery-attempts` (default 2). See the
[tool outcome guide](../docs/tool-results.md) for prerequisites and retry rules.

Timing is read from the shared configuration’s `server` object. See
[operation deadlines, camera waits and shutdown](../server/docs/time-budgets.md).
The application defaults to a 900-second total limit (`--timeout`).

Camera mount metadata selects fixed-camera versus wrist workflows; TF geometry
is handled by the server. See [attachment and input requirements](../server/docs/integration.md#camera-attachment-metadata-and-tf).
CameraInfo defaults to `ros_rectified` with nonzero raw lens D and calibrated P;
normalized K-only publishers must select `rectified_k` explicitly.

## Tests

Run the tests from `agent/` with the virtual environment activated:

```bash
python -m unittest discover -p '*_test.py' -v
```

HTTP and Gemini are mocked, so the tests require no API key, ROS2 installation,
or physical hardware.

Named positions are advertised in `server.hardware.position_names` as
`{"arm":{"home":"Initial position for task startup."}}`. Only the mechanism
maps those names to actual positions, in its own format. Gemini's `move` tool
accepts pixel or named targets; direct poses are reserved for programmatic HTTP
clients. See [the plan arm-selection contract](../docs/pixel-workflow.md#plan-arm-selection) for one-arm and coupled
two-arm detection, pick and placement.
