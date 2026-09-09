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

For pixel-guided applications, see [the workflow guide](../docs/pixel-workflow.md).
A fixed camera can use [calibrated plane projection](../server/docs/plane-projection.md)
instead of depth; `configs/planar.json` is an illustrative shared configuration.
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

For a single fixed camera with one arm, use `configs/minimal.json`. Inspection
falls back to that fixed camera. Customize detection with
`--er-detect-prompt-file prompt_examples/grasp_guidance.md` and set the recovery
budget with `--max-recovery-attempts` (default 2). See the
[tool lifecycle guide](../docs/tool-lifecycle.md) for prerequisites and retry rules.

Timing is read from the shared configuration’s `server` object. See
[operation deadlines, camera waits and shutdown](../server/docs/time-budgets.md).
The application defaults to a 900-second total limit (`--timeout`).

## Tests

Run the tests from `agent/` with the virtual environment activated:

```bash
python -m unittest discover -p '*_test.py' -v
```

HTTP and Gemini are mocked, so the tests require no API key, ROS2 installation,
or physical hardware.
