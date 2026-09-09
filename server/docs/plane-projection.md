# Fixed-camera projection onto a calibrated plane

Use `projection: "plane"` for a fixed overhead camera when the task points lie
on a known plane and measured depth is unsuitable. `projection: "depth"` remains
the default. The plane mode uses an offline pixel-to-plane homography and a
plane-to-world pose. It does not estimate depth, fit calibration online, or
fall back automatically when a depth request fails.

## Configure the calibrated camera

Start with [server/configs/planar.json](../configs/planar.json) and the matching
[agent configuration](../../agent/configs/planar.json). **The supplied coefficients
are illustrative, not a calibration for your robot.** Replace them before motion.
Only `mount: "world"` cameras can use this mode; wrist refinement still uses
RGB-D with capture-time TF.

A camera entry contains:

```json
{
  "id": "overhead",
  "optical_frame": "overhead_optical",
  "mount": "world",
  "parent_frame": "world",
  "projection": "plane",
  "plane_calibration": {
    "calibration_id": "example-table-v1",
    "image_width": 640,
    "image_height": 480,
    "homography": [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 1.0],
    "valid_region": [50.0, 50.0, 589.0, 429.0],
    "plane_pose": {
      "frame_id": "world",
      "position": [0.0, 0.0, 0.75],
      "orientation": [0.0, 0.0, 0.0, 1.0]
    }
  }
}
```

| Field | Meaning |
|---|---|
| `calibration_id` | Nonempty version identifying your offline calibration; included in projected targets |
| `image_width`, `image_height` | Exact calibrated original JPEG dimensions, integers in 2..65536; no automatic rescaling |
| `homography` | Nine finite coefficients in row-major order, mapping original `[u,v,1]` pixel centers to homogeneous plane coordinates in metres |
| `valid_region` | Inclusive `[u_min,v_min,u_max,v_max]` rectangle inside the image; choose a region fully covered and validated by calibration. Points outside it are rejected |
| `plane_pose` | Pose of the plane coordinate system in `world_frame`, metres and unit xyzw quaternion. Its local XY plane is the calibrated surface; tilted planes are supported |

For `q = H * [u,v,1]`, compute `X = q[0]/q[2]`, `Y = q[1]/q[2]`, then
`world_point = R_plane * [X,Y,0] + t_plane`. This is a point on the calibrated
surface, not a measured 3D surface point or a flange/TCP pose. Singular or
ill-conditioned matrices, a projective horizon crossing/approaching the valid
rectangle, unsupported mounts, invalid poses and inconsistent settings fail
configuration validation. Plane pose must be expressed directly in `world_frame`.

## Calibrate and validate offline

1. Fix camera mounting, lens/focus, resolution, crop and rectification settings.
   Use the same rectified original image stream that the bridge will receive.
2. Measure points distributed across the intended plane in its local XY metric
   coordinates. Record their corresponding original image pixels. Fit the
   **image-to-plane** homography offline, with more than the minimum four
   non-collinear correspondences where possible. A plane-to-image matrix must
   be inverted before it is used here. The [OpenCV homography tutorial](https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html)
   explains the projective mapping; OpenCV is not a runtime dependency here.
3. Measure `plane_pose` relative to the robot world, choose a validated image
   rectangle inside the calibration coverage, and assign a calibration version.
4. Use independent check points across that region to measure world-position
   error, including edges. Choose an acceptable error for your task. Check both
   grasp and release points before allowing the driver to move hardware.
5. Save the configuration, use matching topology on the agent/server, and
   restart the bridge when calibration changes. Observe the normal stopped-state
   and recovery procedure before restarting an interrupted application.

A single configured plane applies to **both grasp and release** from that camera.
A table calibration cannot correctly locate the top of a tall object, another
shelf, or a raised tray: perspective causes lateral error as well as a height
error. Adding an arbitrary Z offset afterwards does not generally correct it.
Use points that lie on the calibrated surface, a calibration for the actual
contact plane, or the depth workflow. Multiple selectable planes in one camera,
per-target heights and automatic plane fitting are not implemented. Driver tool
geometry and approach offsets still apply after correct contact-point projection.

Moving the camera/plane or changing image processing invalidates the calibration.
The bridge checks image dimensions and optical frame but cannot detect an
unreported physical displacement, focus change or same-size crop. If optional
CameraInfo is published and its geometry changes after an earlier message, the
bridge invalidates existing evidence/plans and latches plane capture rejection
(`409`, `plane_calibration_invalidated`) until restart with validated calibration.
Restoring the old CameraInfo message does not clear that latch. CameraInfo is
otherwise unnecessary for plane capture; coefficients are loaded at startup,
not hot-reloaded. Do not assume a new calibration ID proves physical accuracy.

## HTTP, tools and driver behavior

`GET /v1/cameras/{id}/capture` still obtains a newly acquired original JPEG after
the request. In plane mode it needs **no depth, CameraInfo or camera TF**. Common
freshness, clock, capture lifetime and motion invalidation rules still apply.
`/image` and `/observation` retain their existing meanings; an RGB observation
ID alone cannot create a metric plan even on a plane-configured camera.

A plane capture returns `kind: "plane"`, `capture_id`, `camera_id`, `width`,
`height`, `stamp_ns`, `frame_id`, `image_base64` and a frozen `plane_calibration`.
It omits depth stamps and camera/flange poses. `POST /v1/plans` keeps its existing
pixel request shape. Projected grasp/release targets carry the usual world
`frame_id`, `position`, `capture_id`, `pixel`, `stamp_ns`, plus
`projection: "plane"` and `calibration_id`. Driver adapters must accept these
provenance fields without treating them as measured depth. Motion phases and
completion acknowledgements are unchanged.

`detect_targets` continues to call ER on one original image. Its prompt includes
the server capture's plane calibration, valid pixel region and off-plane
limitations. Live's topology also describes the selected projection. Prompt
customization uses the existing `--er-detect-prompt-file`; neither Live nor ER
is asked to supply Cartesian coordinates. The server enforces image bounds and
calibrated region but cannot prove that an ER-selected surface lies on the plane.

Optional `approach_targets` / depth-based `refine_grasp` can replace a grasp
point using a wrist RGB-D capture, retaining the plane-projected release point.
Use that combination only when the initial approach point is meaningful for the
actual workpiece. `pick_targets`, RGB `inspect_grasp`, Live `verify_grasp`,
`place_targets`, stop and recovery keep their normal lifecycle.

## Tests

[plane_projection_test.py](../tests/plane_projection_test.py) checks perspective
division, translated/rotated/tilted planes, scaled matrices, invalid calibration,
image grids and bounds, frozen provenance, expiry and invalidation.
[plane_integration_test.py](../tests/plane_integration_test.py) covers RGB-only
capture over HTTP/ROS, invalid target rejection without a driver call, missing
RGB, calibration invalidation, plane detection with depth-based wrist refinement,
and an entire response-driven Gemini mock scenario through pick/verify/place.
[Agent configuration tests](../../agent/embodiment/ros2/plane_config_test.py)
validate shared settings without ROS dependencies. Run both full suites using
[the server test instructions](../README.md#tests).
