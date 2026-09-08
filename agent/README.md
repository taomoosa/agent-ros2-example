# ROS2 Gemini agent

See the [project README](../README.md) for usage and attribution, and the
[HTTP contract](../docs/http-contract.md) for the server interface.

The Gemini Live session, event bus, observation handling, and tool execution
components come from `robotics-samples/live-api/agent`. `run_ros2.py` selects the
ROS2 embodiment and runs application instructions. This directory does not
include the browser UI or Spot/Tinybot drivers. The ROS2 HTTP server is maintained
separately in [server/](../server/README.md).

## Setup

Use Python 3.10 or later. The runtime dependencies are `httpx>=0.27.0`,
`pillow>=10.0.0`, and `websocket-client>=1.8.0`, which are also used upstream.
They are declared in [requirements.txt](requirements.txt).

```bash
cd agent  # From the repository root.
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python run_ros2.py --help
```

## Tests

Run the tests from `agent/` with the virtual environment activated:

```bash
python -m unittest discover -p '*_test.py' -v
```

HTTP and Gemini are mocked, so the tests require no API key, ROS2 installation,
or physical hardware.
