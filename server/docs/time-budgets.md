# Processing time, deadlines and measured completion

Configure timing in the shared topology JSON's `server` object. The agent reads
the same timing fields; use the same file on both sides. `duration` still means
requested trajectory time (0.1..60 seconds), not the complete operation deadline.

## Default budgets

| Setting | Seconds | Purpose |
|---|---|---|
| `motion_timeout` | 120 | Execution/planning allowance for coordinated stages, reset, recovery and group moves |
| `settling_timeout` | 15 | Additional driver allowance for settling and controller completion |
| `gripper_timeout` | 15 | Gripper execution allowance before settling and state verification |
| `state_completion_timeout` | 5 | Post-acknowledgement measured-state verification window |
| `bridge_processing_margin` | 1 | Additional operation budget |
| `stop_timeout` | 5 | Independent stop deadline; it does not grow with normal motion time |
| `request_timeout` | 5 | State and plan CRUD requests |
| `camera_timeout` | 5 | New camera input/TF acquisition |
| `ros_response_margin` | 2 | Gateway allowance for response delivery |
| `http_response_margin` | 3 | Additional agent HTTP allowance |
| `er_timeout` | 90 | Total elapsed time for one Robotics ER call, including transport work |
| `live_io_timeout` | 10 | Live connection/socket/send timeout |
| `cleanup_timeout` | 3 | Limit for each cleanup operation |
| `model_idle_timeout` | 120 | No-model-progress window outside active tool execution |
| `capture_ttl` | 300 | Capture/evidence validity from creation |
| `plan_ttl` | 600 | Detected/approached plan validity from creation |

The settings above accept finite positive values up to 3600 seconds. The
application `--timeout` defaults to 900 seconds and is configured separately.
The bridge also accepts `settling_dwell` (default 0.2, range 0..10 seconds) and
`telemetry_max_gap` (default 0.25, range (0,5] seconds). A zero dwell disables the
extra stationary dwell but still requires new post-acknowledgement telemetry.

The bridge operation budget is execution + settling + post-state + processing
margin. Every move, including a single arm, uses the larger of `duration` and
`motion_timeout` for execution. With defaults, moves, home, plan stages and
recovery have a 141-second bridge budget and a 146-second agent budget. Gripper requests have 36/41
seconds. Stop has 5/10 seconds. Recovery performs stop before the recovery
request, so its total tool duration also includes that stop.

The driver gets the **remaining** budget with time reserved for post-state
verification and bridge margin (at most a quarter of the remaining budget for
short direct ROS requests). It must fit actual execution and settling inside
that budget; the server does not grant extra time after the deadline.

These defaults are starting values, not measured hardware performance. Measure
planning, maximum travel, settling, telemetry cadence and delivery latency. A
long motion must not inherit the short stop deadline, and stop must not inherit
a long motion deadline. A timeout is not permission to replay a motion.

## Deadline propagation and stop ordering

`RobotRequest` now carries `deadline_ns` in the shared ROS clock in addition to
`timeout_sec`. Rebuild **all** interface consumers, including the driver. Gateway
queue/delivery time reduces the remaining execution allowance. Direct
callers may leave `deadline_ns=0`; that only supplies a relative timeout starting
at bridge admission. Internal relative requests are limited to (0,15000] seconds.
Do not compare monotonic timestamps from different processes or hosts.

The bridge uses monotonic deadlines during execution. An expired request is
rejected before driver dispatch. Completion callbacks recheck deadlines, so a
late success cannot win a race against the expiry timer. Slow geometry work and
other successful responses are checked again before returning. A ROS clock
rollback still invalidates existing evidence and pending requests.

Agent HTTP and ER calls have an outer elapsed deadline as well as socket
inactivity limits. This also works with MockTransport/ASGITransport. Live I/O is
bounded off the event loop; a connection completing after cancellation is closed.
The transport work runs on daemon workers so an abandoned blocking call does not
hold up the default executor's shutdown. A custom adapter must still honor its
socket timeout; abandoning a worker cannot forcibly terminate arbitrary code.

On application timeout/disconnect, the agent requests stop **before** cancelling
the consumer and waiting for generator cleanup. An uncertain motion result also
triggers stop before another model decision. Stop failure is retained in the
result with `operator_required`; do not recover or move until stop is confirmed.
The bridge does not turn service-future cancellation into physical stop: drivers
must honor execution deadlines and independently handle stop requests.

The model idle watchdog excludes active tools, including their visual delivery.
Background video alone does not reset it. Cleanup steps are bounded separately,
so the application timeout is an interruption deadline, not a guarantee that
process exit occurs at that exact instant. No session reconnect/replay is added.

## Camera and evidence timing

The agent's fresh-frame allowance covers a preceding poller capture plus its own
new capture and delivery: `2*(camera_timeout + ros_response_margin +
http_response_margin) + live_io_timeout`, 30 seconds by default. Images acquired
before the post-action request are not reused. Individual camera HTTP calls,
Live writes and the overall application have their own limits.

Exposure freshness (`camera_max_age`), RGB/depth matching (`sync_tolerance_sec`),
clock skew and processing deadlines are different quantities. Extending a wait
does not make stale geometry acceptable. Approximate wrist captures additionally
require measured stationary history bracketing both exposure times, no excessive
sample gap, and exposure after the last bridge motion ended. TF still uses the
RGB timestamp. Motion between samples cannot be disproved without faster or
hardware-synchronized telemetry; no scene-motion compensation is implemented.

Capture and plan TTLs are configurable and never silently refreshed by reading
or refining. Held/verified plans retain their existing held-object lifecycle.
If ER/model/operation time exceeds a validity limit, observe and plan again when
allowed; do not simply extend evidence forever. A stable scene remains a required
assumption throughout the configured validity window.

## Driver completion integration

The bridge waits for fresh post-acknowledgement measurements and a stationary
dwell, resetting that dwell on motion, missing/stale telemetry or a sample gap.
Faults remain immediate failures. This window can absorb short settling delays,
but the driver should acknowledge only after its own controller completion check.

[completion.py](../src/ros2_agent_server/ros2_agent_server/completion.py) provides
an optional `CompletionMonitor` for driver adapters. Instantiate one per arm and
new command, with target flange pose and an optional requested gripper opening.
This helper compares against `measured_arm_state.flange_pose`. For a TCP
command, the backend must first derive the corresponding flange target with
its own calibrated transform, or use controller-native TCP completion feedback
instead. Do not compare a TCP target directly against flange measurements.
The common bridge performs no tool transform for this helper.
Feed **measured** states and advancing acquisition times from one clock:

```python
from ros2_agent_server.completion import CompletionMonitor, CompletionPolicy

monitor = CompletionMonitor(target_pose, opening=None, policy=CompletionPolicy(
    position_m=0.002, orientation_rad=0.02, gripper_opening=0.02,
    dwell_sec=0.2, max_sample_gap_sec=0.25))
# In the driver measurement callback:
completed = monitor.update(measured_arm_state, measurement_stamp_ns / 1e9)
```

The defaults mean 2 mm position, 0.02 rad shortest rotation, 0.02 normalized
opening and 0.2 s dwell. q and -q are equivalent. Frame mismatch, motion, faults,
out-of-tolerance measurements, stale/replayed measurement times and sample gaps
prevent completion. Unknown opening cannot satisfy a requested opening. Choose
opening=None for an arm-only move or when gripper completion is checked separately.
Transform targets and measurements into the same frame first; this helper does
not perform TF lookup, plan paths or infer contact. For contact grasps, requested
closure need not equal the loaded gripper width: use your controller's contact/
force completion criterion instead of a position-only opening check.

Configure hardware-specific tolerances; these camera-independent defaults are
examples. For coordinated work, all arm monitors and phase conditions must pass.
Honor `timeout_sec`/`deadline_ns`, stop the group on failure, then report failure
or unknown outcome as appropriate. This module is an integration helper, not an
automatically loaded hardware driver or an independent safety controller.

## Verification

`deadline_geometry_test.py` covers late-success races, expired ROS admission,
shared budgets, settling/dwell, timeout/stop, configurable TTLs, and a measured
mock driver's success/failure/recovery. `independent_stream_test.py` publishes
RGB, depth, TF and state at separate jittered rates with delayed depth and dropout.
Test fixtures lease different ROS domains across processes. Agent `timing_test.py`
covers slow cleanup/connect/send, idle models, camera lock contention, mock HTTP
elapsed limits and a real TCP peer continuously sending small response chunks.


## Common primitive execution

`/v1/move` uses the group operation budget and `/v1/gripper` uses the
gripper budget. The compiler resolves all goals before execution; the driver
preflights the entire sequence. Every phase and its measured-state/dwell check
spend the original remaining budget. No phase restarts the motion timeout.
Failure initiates a group stop using the independent stop budget; this can make
failure handling extend beyond the motion deadline. See
[primitive sequence and stop semantics](primitive-adapter.md).
