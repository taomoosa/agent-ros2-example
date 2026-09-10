# CameraInfo for existing pixel tools

Set `cameras[].camera_info_mode` in the shared topology JSON. Both agent and
server accept it. This applies to depth projection; plane projection continues
to use its offline homography and original image grid.

| Mode | Required input and projection |
|---|---|
| `rectified_k` (explicit adapter mode) | Adapter supplies rectified pinhole K and D within the numerical zero tolerance. A supplied nonzero P must agree with K; inconsistent P is rejected |
| `ros_rectified` (default) | Already rectified RGB plus registered color-grid/color-Z depth. Use fx/fy/cx/cy from P, even when raw-image K differs and D is nonzero |

The default accepts finite, nonzero lens distortion coefficients such as
`D=[-0.28,0.09,0.001,-0.002,-0.015]`. This is an illustrative calibrated lens,
not a universal acceptance limit. ROS CameraInfo describes the raw lens with
D/K and the rectified image with P; the driver may retain nonzero D even when
publishing rectified images. Projection uses P, so these coefficients must not
be rejected merely for exceeding a near-zero rounding tolerance. See the
[ROS CameraInfo definition](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg).

Supply an actually rectified JPEG stream and depth registered to its grid and
optical Z. Merely allowing nonzero D does not undistort raw pixels. Raw image
sources need upstream rectification, for example
[image_proc RectifyNode](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/doc/components.rst).
The server does not impose a universal maximum D: coefficient magnitude depends
on the distortion model/calibration and is not an image-error bound.

Migration: `camera_info_mode` now defaults to `ros_rectified`. A K-only adapter
must explicitly set `"camera_info_mode":"rectified_k"` and continue supplying
normalized rectified K/D. A standard CameraInfo publisher supplies calibrated P
and identity R. Missing/zero P fails explicitly rather than falling back to raw
K and silently misprojecting. `rectification_tolerance` still bounds structural
matrix rounding (and near-zero D only in explicit K-only mode); increasing it
is not a substitute for lens correction. Plane projection still requires its
calibration image grid; this mode change does not add nonlinear lens correction
to a homography.

In `ros_rectified`, the bridge currently requires identity R and zero P
translation (monocular Tx/Ty). Nonidentity R, stereo offsets, cropped ROI and
binning above 1 are explicitly rejected with field-specific errors. A zero/full
image ROI is accepted. This bounded support does not implement raw-image
undistortion, stereo reconstruction, resizing or registration. Supply those
upstream, or extend the adapter deliberately. Do not merely relabel raw images.

P/R/D/K must satisfy the selected mode's finite-value and structural checks.
Calibration change detection includes P, R, distortion model, ROI dimensions
and the existing K/D/frame/size fields, with numerical tolerances. Meaningful
changes invalidate evidence and plans. Fixed plane cameras that receive changed
CameraInfo still require calibration verification and restart.

Use `ros_rectified` when your driver publishes the standard CameraInfo for its
rectified image and meets these constraints. Use `rectified_k` when an adapter
explicitly normalizes that contract. See [numerical tolerances](numerical-tolerances.md)
and [synchronization](synchronization.md) for frame aliases and timestamps.
