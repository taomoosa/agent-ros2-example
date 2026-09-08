# agent-ros2-example

A Gemini robotics agent and a ROS2 HTTP server for controlling one or two robot
arms with multiple cameras. The agent reuses the Gemini Live session and tool
execution components from `robotics-samples/live-api/agent`.

```text
agent/                              server/
Gemini Live / application tasks --HTTP--> http_gateway node (FastAPI)
                                           | ROS2 service
                                         robot_bridge node
                                           +-- subscribes to state and camera topics
                                           +-- sends commands to a robot driver
```

## Getting started

The agent requires Python 3.10 or later. See [agent/README.md](agent/README.md)
for dependency installation. ROS2 does not need to be installed on the agent host.
See [server/README.md](server/README.md) for server prerequisites, build commands,
and startup instructions. Configure matching arm IDs, camera IDs, and coordinate
frames on both sides.

```bash
cd agent
export GEMINI_API_KEY='your-api-key'
# Set GEMINI_LIVE_MODEL to a Gemini Live model available to your API key.
python run_ros2.py --config configs/single_arm.json \
  --model "$GEMINI_LIVE_MODEL" --task-file apps/inspect.md
```

`configs/single_arm.json` and `configs/dual_arm.json` define the HTTP endpoint,
arm IDs, base and flange frames, camera IDs, optical frames, and camera mounts.
World-mounted cameras use `mount: "world"`. Flange-mounted cameras use
`mount: "flange"` and `arm_id` to identify the arm they move with. Images are
combined into a grid in configuration order, with labels when there is more than
one camera. Captures across cameras are not strictly synchronized. Failed
captures are not replaced with cached images.

`apps/inspect.md` is an observation task. `apps/pick_and_place.md` is an arm
manipulation task; supply the arm ID and measured target flange poses through
`--instruction`. Positions are in meters and orientations are unit quaternions
in `[x, y, z, w]` order. Use coordinate frames listed in the configuration.
Commands to two arms run sequentially; coordinated dual-arm trajectories are not
supported. The ROS2 driver is responsible for coordinate transforms,
calibration, and motion planning.

The agent maps Gemini's `get_robot_state`, `move_arm`, `set_gripper`, and `stop`
tool calls to HTTP requests and returns results and camera images to Gemini.
`finish_task` ends the application; a reported failure exits with status 1.
An interrupted application, connection failure, or timeout triggers a stop
request and connection cleanup. Motion requests are not automatically retried
after an HTTP timeout.

Specify the model with `--model`. The default response modality is AUDIO; the CLI
displays output transcripts without playing audio. Use `--response-modality TEXT`
with a model that supports text output. Choose a model and modality supported by
your Gemini Live API access. See the [Google tool use documentation](https://ai.google.dev/gemini-api/docs/live-api/tools)
for the API's function call and response format.

## HTTP contract and tests

The [HTTP contract](docs/http-contract.md) describes the interface implemented by
`server/`. Connecting physical hardware requires a driver that provides the
specified ROS2 topics and service. The interface is not compatible with the
upstream Spot server.

```bash
cd agent
python -m unittest discover -p '*_test.py' -v
```

Agent unit tests mock HTTP and Gemini and require no ROS2 installation, API key,
or hardware. See [server testing instructions](server/README.md#tests) for server
unit and integration tests. Integration tests use real ROS2 topics, services,
and HTTP, with Gemini and the robot driver mocked. Physical hardware, TF,
collision checking, and the live Gemini API require separate validation.

## License and attribution

This repository includes code and tests copied from `live-api/agent` in
[google-gemini/robotics-samples](https://github.com/google-gemini/robotics-samples),
commit `c51cbab6e6efffff8738ecf9ce41ba85034d9654`.

The upstream code is licensed under Apache License 2.0. A copy of the license is
included in [third_party/robotics-samples-LICENSE](third_party/robotics-samples-LICENSE).
Existing copyright and license notices have been preserved. No NOTICE file was
present in the referenced upstream checkout.

The [copied file list](third_party/robotics-samples-files.txt) identifies the
upstream files. Two files were modified to support audio transcripts in the CLI:
`agent/session_manager.py` enables `outputAudioTranscription`, and
`agent/core/decision_making.py` forwards output transcription events. The other
copied files are unchanged.

`agent/embodiment/ros2/`, `agent/run_ros2.py`, the ROS2-specific tests,
configurations, application examples, `server/`, and project documentation were
added in this repository and are provided under Apache License 2.0.
