# Measured arm telemetry and completion

## Standard ROS transport

Publish `std_msgs/msg/String` on `/robotics/arms/{arm_id}/state` with reliable
QoS. Its `data` field contains the JSON object below, including the actual
measurement timestamp. No custom ROS message, message-generation dependency,
or transport selection is required for a state publisher. Existing controllers
can keep their native topics; an adapter maps their measured data into this
JSON contract at the bridge boundary.

A custom state message was considered but is not used, to avoid coupling other
robot packages to a repository-specific ROS type. String provides a standard
transport, while JSON fields still have an application-specific schema checked
by the bridge. JointState or a controller's other messages cannot simply be
remapped to this topic without conversion.

```json
{
  "stamp_ns": 123000000456,
  "moving": false,
  "flange_pose": {
    "frame_id": "world",
    "position": [0.4, 0.0, 0.3],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  },
  "gripper": {"opening": 1.0, "object_detected": null, "fault": null},
  "fault": null
}
```

Replace the illustrative timestamp and values with actual measurements.

| Field | Publisher responsibility |
|---|---|
| `stamp_ns` | Required positive integer ROS measurement time, no later than bridge time. Advance for new measurements; never restamp cached values. |
| Topic `{arm_id}` | Identifies the configured arm; do not add an `arm_id` field to the JSON body. |
| `moving` | Required boolean reporting actual controller motion, including externally initiated motion. |
| `flange_pose` | Required measured flange pose in a configured frame: position in metres, unit quaternion xyzw; not TCP pose. |
| `gripper.opening` | Normalized opening (0 closed, 1 open), or null/omitted if unavailable. |
| `gripper.object_detected` | null/omitted for unknown, false for empty, true for detected. Opening alone is not a holding sensor. |
| `fault`, `gripper.fault` | null/omitted for no fault; otherwise `{code, message, recoverable}`. A present fault requires nonempty code/message; recoverable only when the driver supports recovery. |

For an adapter that already has a measured state dictionary:

```python
import json
from std_msgs.msg import String

publisher = node.create_publisher(String, "/robotics/arms/arm/state", 1)
payload = dict(measured_state, stamp_ns=measurement_stamp_ns)
publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
```

`measured_state` contains the fields above except `stamp_ns`.
`measurement_stamp_ns` must come from the measurement source in the shared ROS
clock, not from a timer that repeatedly republishes an old reading. Handle
upstream loss and controller faults explicitly. The publisher only needs the
standard String type, not an import from `ros2_agent_interfaces`.

## Freshness and command completion

Both receive age and past measurement age must be within `server.state_max_age`.
Measurements may lead the bridge clock by at most
`server.future_skew_tolerance_sec` (default 5 ms); use 0 for strict same-clock
validation. Source timestamps are preserved. This allowance does not relax
replay rejection or the post-acknowledgement ordering below. See
[numerical tolerances](numerical-tolerances.md) for clock and precision limits.
A replayed or older measurement cannot refresh the cache. `/v1/state` includes
`measurement_stamp_ns` per arm; its top-level `stamp` is the query time, not the
measurement time. A source restart must resume valid measurement timestamps in
the shared ROS clock. A ROS clock rollback clears bridge snapshots and evidence.

After a successful driver motion response, the bridge waits for a **subsequent
sample measured at or after acknowledgement receipt** for every participating
arm, then checks stationary dwell, faults and applicable holding/release
telemetry. The dwell resets on motion, stale/missing data or excessive measurement
gaps; faults are reported without waiting for the dwell. This deliberately requires periodic measurements even if the driver
published completion state just before its response. It avoids depending on the
arrival order of two independent ROS channels. It does not prove target accuracy:
the driver must check physical completion before acknowledging.

The wait is bounded by `server.state_completion_timeout` (default 5 seconds,
maximum 3600) and the remaining original operation deadline. Budget driver execution
and telemetry updates together; see the new configurable [time budgets](time-budgets.md)
and optional driver-side measured completion monitor.
On missing post-command telemetry, the result is 504,
`code: "post_command_state_timeout"`, `outcome: "unknown"`, and recovery is
required. A successful stop interrupts this wait and cannot be overwritten by
its late result. Stop itself remains available without telemetry and relies on
the driver's physical-stop acknowledgement.

## Updating an existing publisher

Keep the standard String type and `/robotics/arms/{id}/state` topic. Add an actual
integer `stamp_ns` to older unstamped JSON; absent or invalid timestamps are
rejected. Unknown fields and non-finite values are also rejected.

If you tried the interim custom-message implementation, switch the publisher
back to String on `/state` and remove `server.state_transport` from configuration.
There is no `/state_json` alternative or automatic transport fallback. Do not
run publishers of different ROS types on the same state topic. Remove custom
state-type imports and any dependency needed only for that type from your
adapter package.

The separate command service still uses `RobotRequest`, as before. Consumers
of its added `request_id` field need the updated generated service type; this
requirement does not apply to packages that only publish state. See the
[integration guide](integration.md) for command operations.
