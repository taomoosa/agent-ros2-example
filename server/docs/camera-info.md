# CameraInfo for existing pixel tools

Set `cameras[].camera_info_mode` in the shared topology JSON. Both agent and
server accept it. This applies to depth projection; plane projection continues
to use its offline homography and original image grid.

| Mode | Required input and projection |
|---|---|
| `rectified_k` (default) | Adapter supplies rectified pinhole K and D within the numerical zero tolerance. A supplied nonzero P must agree with K; inconsistent P is rejected |
| `ros_rectified` | Already rectified RGB plus registered color-grid/color-Z depth. Use fx/fy/cx/cy from P, even when raw-image K differs and D is nonzero |

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
