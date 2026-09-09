# Camera synchronization and diagnostics

`/image` returns a fresh JPEG. `/observation` returns a fresh original RGB image
with identity and time for visual evidence. `/capture` additionally binds depth,
calibration and capture-time TF for metric projection in default depth mode.
Fixed cameras can alternatively use [offline plane projection](plane-projection.md),
which needs only the calibrated RGB stream. RGB/depth pairing rules below apply
to depth mode; image freshness and capture invalidation apply to both modes. An image endpoint working
does not imply that the geometry inputs are ready.

## Per-camera configuration

The following fields extend an existing camera entry. This is a geometry
contract for the publisher, not a calibration or registration implementation:

```json
{
  "id": "overhead",
  "optical_frame": "color_optical",
  "mount": "world",
  "parent_frame": "world",
  "depth_frame": "aligned_depth_header",
  "camera_info_frame": "color_optical",
  "depth_geometry": "color_optical_z",
  "sync_tolerance_sec": 0.01
}
```

`depth_frame` and `camera_info_frame` default to `optical_frame` when omitted.
A different depth header is accepted only when explicitly configured. Its pixels
must already correspond to the color grid, and its values must be **Z along the
color optical axis**, in `16UC1` millimetres or `32FC1` metres. `depth_geometry`
currently accepts only `color_optical_z`. It is not sufficient for the topic
name to contain `aligned`, or for the two images to have the same dimensions.

Check the camera driver's registration contract. Raw depth in a different
sensor geometry needs upstream registration using intrinsics and extrinsics.
Changing a header string does not perform that transformation. The bridge
projects with color CameraInfo and world-from-color TF; it does not use a depth
header alias to select another TF or another optical-axis depth convention.

The default RGB/depth tolerance is 10 ms. Measure sensor timing and acceptable
positional error before deploying; use `sync_tolerance_sec: 0` for strict pairing.
Approximate pairing never changes either source timestamp. The supported range
is 0..0.1 seconds. A value within that range is not a guarantee of adequate
accuracy for a particular arm, workpiece or camera.

## Pairing, freshness and bounded waits

Both RGB and depth must have acquisition stamps at or after the HTTP/ROS camera
request. Future timestamps are accepted only within
`server.future_skew_tolerance_sec` (default 5 ms, range 0..0.1 seconds); this small
clock-skew allowance is independent of RGB/depth pairing. Past acquisition times
must be within `server.camera_max_age` (default 2 seconds, maximum 10). CameraInfo is independent
of each exposure but must describe the current full-resolution rectified image.

RGB and depth each have a bounded buffer, controlled by
`server.camera_buffer_size` (default 32, range 2..256). Out-of-order arrivals are
supported. Among currently available eligible pairs, the bridge selects the
smallest absolute timestamp difference, breaking ties by earlier RGB and then
earlier depth time. It does not wait for a hypothetical better future pair once
an eligible pair and its TF are available. If a candidate lacks TF, another
eligible candidate can be tried. Buffers prevent a newer RGB from discarding the
only image whose delayed depth has just arrived.

Parallel requests have independent deadlines and capture IDs. They may use the
same physical exposure if it satisfies each request; this is not multi-camera
hardware synchronization. Late inputs cannot complete an expired request.
The total wait is `server.camera_timeout` (default 5 seconds, maximum 3600).
Deadlines use monotonic wall time and still expire when simulation time pauses.

TF is queried at the **RGB acquisition time**, including world-from-flange for
a wrist camera. TF samples need not have identical timestamps to the image:
interpolation within available transform history is supported. Missing history,
missing frames and extrapolation remain errors; latest arm telemetry is never
substituted for the image-time transform.

Approximate RGB-D pairing assumes that scene geometry is stable over the
selected interval. In particular, a moving camera or object can associate a
color pixel with a depth measurement of a different surface. Nonzero time-delta
wrist captures require fresh stationary arm telemetry and no bridge motion in
progress, stationary measurement history bracketing both exposures with bounded
sample gaps, and exposure after the last bridge motion. The adapter must report
actual motion, including externally initiated motion. The sampled history cannot
prove scene stability between measurements and does not compensate motion. Choose stationary acquisition or implement appropriate upstream
compensation for moving scenes; increasing the tolerance alone cannot fix them.

Successful RGB-D metadata includes the original `stamp_ns` (RGB),
`depth_stamp_ns`, signed `sync_delta_ns`, `pose_stamp_ns`, `depth_frame_id`,
`camera_info_frame_id`, `camera_info_stamp_ns`, and `kind: "rgbd"`. Calibration,
depth and camera pose are frozen with the capture. Refinement requires both
selected exposure times to be after the approach completion time.

A CameraInfo geometry change exceeding the configured numerical tolerances
clears that camera's image/depth buffers and
invalidates geometry captures/plans; pending camera requests return 409 and must
be reissued. For plane mode, an observed CameraInfo geometry change also latches
capture rejection until calibration is verified and the bridge restarted.
Held-object or active-motion state requires recovery. Repeated
CameraInfo with only a changed header timestamp is not a geometry change.

Publishers, TF and bridge must share the ROS clock. On a ROS clock rollback, the
bridge clears images, depth, state, TF history and plan evidence, rejects pending
requests, and retains necessary stop/recovery requirements. Reacquire inputs;
do not reuse an old plan. A sensor-only clock reset or clock offset is not
silently corrected: out-of-age samples and future samples beyond the configured
skew allowance fail freshness checks, with
observed timestamps in diagnostics. Restore the publisher's common clock.

See [numerical tolerances](numerical-tolerances.md) for calibration equivalence,
depth range and neighbourhood quality, tuning examples and retained exact checks.

## RGB visual evidence

`GET /v1/cameras/{id}/observation` returns `kind: "rgb"`, `capture_id`, camera/frame
identity, acquisition time, dimensions and the original JPEG in `image_base64`.
It does not require depth, CameraInfo or TF. Its ID can be used for visual grasp
verification, but projection and pixel-plan creation/refinement reject it.

`inspect_grasp` uses this endpoint and still requires a current stationary arm,
a post-pick image, the matching plan/arm, and original-image delivery to Live
before verification. A wrist RGB camera can therefore inspect a grasp even when
only the fixed camera provides RGB-D. Detection/refinement still require metric
geometry. `/image` remains a plain JPEG endpoint without evidence registration.

## Diagnostic events

Enable HTTP diagnostics and ROS DEBUG logs separately:

```bash
python -m ros2_agent_server --config server/configs/minimal.json \
  --diagnostic-log-level DEBUG --ros-args --log-level debug
```

Use the response's `X-Request-ID` to correlate `http_received`, `ros_send`,
`bridge_received`, `camera_wait_started` or `driver_send`, `request_complete`,
`bridge_reply`, `ros_future_done`, `asyncio_deliver` and `http_reply`.
`RobotRequest.request_id` propagates the ID to the hardware driver; log it there
too. Rebuild all interface consumers after updating the service definition.
Direct ROS callers can provide 1..64 ASCII letters/digits/underscores/hyphens;
otherwise the bridge generates an ID. IDs are diagnostic, not idempotency keys.

Camera wait reasons are logged when their category changes. Timeout warnings
include the last detailed state; input callbacks provide DEBUG metadata without
image/depth bodies. Representative `details.code` values are:

| Code | Investigate |
|---|---|
| `fresh_rgb_unavailable` | Publisher, acquisition clock, request time, maximum age |
| `camera_info_unavailable` | Topic/remap and delivery after the volatile subscription starts |
| `depth_unavailable` | Depth publisher and input topic |
| `depth_sync_unavailable` | RGB/depth stamps, configured tolerance, bounded buffer and delivery delay |
| `capture_tf_unavailable` | Source/target frames, requested timestamp and TF exception text |
| `wrist_not_stationary` | Arm telemetry or an active motion during approximate pairing |
| `wrist_state_unavailable` | Missing/stale wrist arm state |
| `post_command_state` | New measured telemetry after driver acknowledgement has not arrived |

Invalid geometry returns 422 with the failed field, expected value and actual
value. Missing eligible input returns 504 with `code: "capture_timeout"` and
`details`. Clock/calibration changes return 409. These are different from a
motion timeout with `outcome: "unknown"`; never replay an uncertain motion.
Logs contain selected IDs, dimensions, timestamps, result codes and elapsed
wall time. Images/base64, credentials and entire request payloads are omitted.

## Regression coverage

[hardware_contract_test.py](../tests/hardware_contract_test.py) exercises timing
boundaries, field-specific errors, geometry aliases, frozen nonzero transforms,
JSON telemetry, measurement replay and buffer ordering.
[hardware_integration_test.py](../tests/hardware_integration_test.py) exercises
real ROS input delays, TF interpolation, parallel captures, calibration/clock
changes, state/response ordering, stop interruption, request correlation, RGB
wrist verification and a response-driven Gemini scenario with delayed depth and
RGB-only inspection. These tests validate contracts, not real calibration or
physical grasp accuracy.
