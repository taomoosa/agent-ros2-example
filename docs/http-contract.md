# ROS2 HTTP contract v1

This interface connects the agent to the FastAPI and ROS2 nodes in `server/`.
See [server/README.md](../server/README.md) for ROS2 topics, internal services,
and startup instructions. The HTTP base URL is an origin such as
`http://localhost:8080`. JSON uses UTF-8 encoding.

| Method | Path | Request / response |
|---|---|---|
| GET | `/v1/state` | Robot state as a JSON object |
| GET | `/v1/cameras/{camera_id}/image` | JPEG bytes with `Content-Type: image/jpeg` |
| POST | `/v1/arms/{arm_id}/pose` | `{frame_id, position: [x,y,z], orientation: [x,y,z,w], duration}` |
| POST | `/v1/arms/{arm_id}/gripper` | `{opening: 0..1}`; 0 is closed and 1 is open |
| GET | `/v1/cameras/{camera_id}/capture` | Capture-bound RGB-D/TF metadata and original JPEG as base64 |
| POST | `/v1/plans`, `/v1/plans/refine`, `/v1/plans/execute`, `/v1/plans/verify` | Pixel manipulation workflow, detailed below |
| POST | `/v1/arms/reset`, `/v1/arms/poses`, `/v1/arms/recover` | Coordinated home / explicit group motion / recovery |
| POST | `/v1/stop` | `{arm_id: string|null}`; null stops all arms |

Command responses are JSON objects representing completion, not merely
acceptance. A 202 response must not be treated as completion. For example:
`{"success": true, "arm_id": "left", "message": "Reached target"}`.
Report failures with an appropriate HTTP error or
`{"success": false, "error": "..."}`. The client passes the object to Gemini.
A successful response alone does not establish that an object was grasped.

Example `/v1/state` response:

```json
{
  "arms": [{
    "id": "left", "moving": false,
    "flange_pose": {
      "frame_id": "world", "position": [0.4, 0.0, 0.3],
      "orientation": [0.0, 0.0, 0.0, 1.0]
    },
    "gripper": {"opening": 1.0}
  }],
  "stamp": {"sec": 123, "nanosec": 456},
  "frames": ["world", "left_base", "left_flange", "left_optical"]
}
```

- Arm IDs, camera IDs, and frame names must match the agent configuration.
- Poses target the **flange**. Positions use meters, durations use seconds, and
  quaternions use xyzw order. Duration must be between 0.1 and 60 seconds. The
  client's motion timeout is `duration + 10` seconds.
- The robot driver must resolve TF transforms and check reachability,
  collisions, and speed and force limits. Hardware-specific drivers are not
  included. For moving reference frames, such as a flange or camera frame,
  resolve the target pose using TF at the start of request processing.
- Camera GET requests return images captured after the request. The server
  validates acquisition timestamps and optical frames. Hardware-specific nodes
  manage calibration and the consistency between image timestamps and TF.
  The agent displays only complete sets of successfully captured camera images.
  Strict synchronization across cameras is not supported.
- Stop requests must remain available while a pose request awaits completion.
- Communication failures are reported as an unknown outcome. The client does
  not automatically retry motion requests.
- Invalid input returns 422; unknown resource IDs return 404. Missing or stale
  state and unavailable services return 503. Camera or driver timeouts return
  504. Invalid driver completion responses return 502.
- Image responses include `Cache-Control: no-store`, `X-Frame-Id`, and
  `X-Stamp-Ns` headers.
- Asynchronous jobs and authentication headers are not implemented.

See [pixel workflow](pixel-workflow.md) for the complete capture/plan bodies,
state transitions, image timestamp binding, and coordinated driver contract.
Legacy metric pose requests resolve their reference frame at request time;
pixel plans instead retain world points transformed at image acquisition time.

See [tool lifecycle and recovery](tool-lifecycle.md) for optional fault/object
telemetry, failure response details and the explicit recovery driver operation.

`POST /v1/arms/reset` and `/v1/arms/recover` accept an omitted body or `{}`.
Unknown fields, JSON null, arrays/scalars and malformed JSON return 422 before
any driver call. Both HTTP and direct ROS requests use the same validation.
See [tool outcomes](tool-results.md) for final observation and stop/recovery
semantics; a successful physical stage is distinct from task achievement.
