# Numerical tolerances and measurement quality

These settings distinguish numerical rounding from measurement uncertainty.
Defaults are starting values for stationary tabletop work, not a claim about a
particular camera or arm's accuracy. Tune the shared topology JSON using measured
sensor timing, validated depth range and the application's positional error budget.
No new dependency or custom ROS message type is needed.

## Configuration

Add the camera fields to each relevant entry in `cameras`; add the clock field to
`server`. Both the agent and server accept the same file. Values must be finite.

| Field | Default | Unit and acceptance rule |
|---|---|---|
| Camera `sync_tolerance_sec` | `0.01` | Seconds; absolute RGB/depth stamp difference. Range 0..0.1; 0 restores strict matching |
| `server.future_skew_tolerance_sec` | `0.005` | Seconds; maximum future lead of RGB/depth/state relative to the bridge clock. Range 0..0.1; 0 disables the allowance |
| Camera `calibration_tolerance_px` | `0.000001` | Pixels; absolute change in K's fx, fy, cx, cy from the accepted baseline. Range 0..0.01 |
| Camera `rectification_tolerance` | `0.000000001` | Structural zero/one residuals in projection/rectification matrices; also near-zero D only in explicit rectified_k mode. Not a lens-distortion bound. Range 0..0.000001 |
| Camera `depth_min_m` / `depth_max_m` | `0.05` / `5.0` | Metres along color optical Z; inclusive validated working range, with 0 < min < max |
| Camera `depth_absolute_tolerance_m` | `0.02` | Metres; fixed part of local depth agreement. Range 0..0.1 |
| Camera `depth_relative_tolerance` | `0.01` | Fraction of selected depth; range 0..0.1. Added to the fixed part |
| Camera `depth_min_support` | `0.6` | Required fraction of the clipped 3x3 neighbourhood agreeing with the selected depth. Range (0.5, 1]; at least 3 pixels must support it |

For example, these fields select a calibrated 0.2..3 m depth range and a stricter
local agreement policy. The numbers are illustrative; use measured limits:

```json
{
  "sync_tolerance_sec": 0.008,
  "calibration_tolerance_px": 0.000001,
  "rectification_tolerance": 0.000000001,
  "depth_min_m": 0.2,
  "depth_max_m": 3.0,
  "depth_absolute_tolerance_m": 0.01,
  "depth_relative_tolerance": 0.005,
  "depth_min_support": 0.75
}
```

Configure `"future_skew_tolerance_sec": 0.002` inside `server` only if the shared
clock's measured positive skew is bounded by 2 ms. This is not clock-offset
estimation or correction. The new defaults change previously strict pairing and
future rejection; explicitly set both fields to zero to retain those checks.

## Calibration rounding

In `rectified_k` mode, `numerics.rectified_intrinsics` accepts only finite D and
structural K residuals within `rectification_tolerance`, then canonicalizes K's
structural entries. `ros_rectified` instead validates and uses P, permitting
finite raw distortion D. The calibrated focal lengths and principal point are
not rounded.

`RobotBridgeNode._camera_info` compares against the last accepted geometry,
using absolute tolerances without a relative term. Within-tolerance publications
retain that baseline and its CameraInfo; repeated small changes cannot walk the
baseline indefinitely. Thus the retained CameraInfo header timestamp may precede
the latest equivalent publication. A larger cumulative change invalidates
capture/plan evidence and clears camera buffers. Plane projection also latches
rejection until recalibration is verified and the bridge restarted.

Binning 0/1 and default/full-image ROI dimensions are treated as equivalent.
Frame IDs, image dimensions, ROI offsets and other discrete geometry remain
exact. A change from valid to invalid rectification residuals is never hidden by
a tolerant comparison. Plane homography conditioning and horizon checks retain
their existing numerical bounds.

The default is **ros_rectified**, using calibrated P with identity R and zero
projection translation, and permitting nonzero raw-image D and differing K.
Normalized K-only adapters must explicitly select `camera_info_mode=rectified_k`. Unsupported stereo/cropped/rotated geometry is
rejected; see [CameraInfo modes](camera-info.md). A tolerance does not implement
rectification or registration.

## Depth agreement and failure handling

`numerics.measured_depth` checks the selected pixel and a clipped 3x3 window.
A neighbour supports the selected depth z when it is finite, inside the working
range, and satisfies:

```text
abs(neighbour_depth - z) <= depth_absolute_tolerance_m + depth_relative_tolerance * z
support_count >= max(3, ceil(depth_min_support * window_pixel_count))
```

Range and agreement comparisons also allow encoding roundoff: `abs(z) * 2^-23`
for float32 readings and 1e-12 m for integer millimetres converted to float64.
Agreement includes the rounding bounds of both compared readings. This avoids
rejecting a configured boundary such as 1.234 m solely because of serialization;
it does not replace the independently configured sensor agreement band. Zero,
negative and non-finite readings remain invalid.

Missing/out-of-range pixels count against support. Image borders use only pixels
inside the image. At 1 m the default agreement band is 3 cm; at 2 m it is 4 cm.
This band is a local consistency test, not a guaranteed 3D target accuracy.
The selected measured depth is preserved; it is never replaced with an average
from a potentially different foreground/background surface.

Range failure or insufficient support returns HTTP 422 with
`code: "depth_quality_invalid"` and the failed range or support condition.
The agent can choose a better-supported original-image pixel or acquire a new
capture and repeat detection. Thin objects, boundaries and steep surfaces may
be rejected conservatively. Validate these cases before relaxing the policy.
Spatially coherent wrong depths can still pass: upstream registration/quality,
driver workspace checks and hardware validation remain necessary. Plane mode
uses no depth and therefore bypasses these depth checks; its calibration and
object-height limitations remain unchanged.

## Timing and completion

The future-skew allowance applies only to freshness checks. It does not change
source stamps, request ordering, replay rejection, post-command measurement
ordering, TF query time, TF extrapolation rules, or wall-clock deadlines. Images
must still be acquired at or after the request; refinement/verification must
still use post-motion evidence. Negative clock offsets can delay eligibility
until a new exposure or state measurement arrives. A positive allowance is not
proof of physical ordering within that clock uncertainty; use a common clock
and stationary acquisition for the required error budget.

Approximate wrist RGB-D capture checks current stationary telemetry, a sampled
stationary history bracketing both exposures, bounded sample gaps, and exposure
after the last bridge motion. This is not continuous motion compensation or
independent clock synchronization. See [synchronization](synchronization.md).

Arm pose/gripper arrival accuracy is checked by the hardware driver before its
completion response; the bridge does not currently compare measured and target
poses. Define position tolerance in metres, orientation tolerance in radians
(using the shortest rotation, with q and -q equivalent), gripper width tolerance
in the controller's calibrated units, and a stationary dwell time in that driver.
Do not use the camera's depth agreement band as an arm arrival tolerance. Unknown
opening/object sensors remain unknown, and Boolean faults/holding evidence are
not made tolerant. See [telemetry](telemetry.md) and [driver integration](integration.md).

## Numerical verification

Projection tests use an absolute 1e-6 m calculation budget for float32 depth and
noninteger intrinsics. A separate noisy-depth test uses its injected 5 mm sensor
variation plus that numerical budget. These are test-fixture bounds, not an
accuracy guarantee for arbitrary transforms or cameras. Accepted transform
quaternions retain the existing squared-norm tolerance of 1e-3 and are normalized
before rotation, preventing small norm residuals from scaling the result.

`numeric_tolerance_test.py` covers accepted/rejected residuals, cumulative
calibration drift, plane invalidation, clock allowance boundaries, exact evidence
ordering, nonbinary float32 depth, range/support boundaries, holes and positive
outliers. Existing tests keep exact identity, timestamp preservation and request
forwarding assertions. Supported standard CameraInfo modes, gradual clock drift
and measured completion monitoring are covered by `deadline_geometry_test.py`;
independent stream rates and dropout are covered by `independent_stream_test.py`.
The optional driver completion helper and its limits are described in
[time budgets](time-budgets.md). Hardware-specific deployment still requires validation.
