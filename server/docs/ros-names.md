# Configure ROS topic and service names in one place

Keep installation-specific names in the **`server.remappings` object of the
configuration JSON passed with `--config`**. Both server roles read it, including
when launched as separate processes. No node source edits or duplicate agent
configuration are needed. A complete example is
[configs/remapped.json](../configs/remapped.json).

```json
{
  "server": {
    "remappings": {
      "/robotics/request": "/example/robot_request",
      "/robot_driver/execute": "/example/driver/execute",
      "/robotics/arms/arm/state": "/example/arm/measured_state",
      "/robotics/cameras/overhead/image/compressed": "/example/camera/color/image/compressed",
      "/robotics/cameras/overhead/camera_info": "/example/camera/color/camera_info",
      "/robotics/cameras/overhead/depth/aligned": "/example/camera/aligned_depth",
      "/tf": "/example/tf",
      "/tf_static": "/example/tf_static"
    }
  }
}
```

Merge this `server` object into your topology file; the full configuration also
requires `robot_url`, `world_frame`, `arms` and `cameras`. Source names on the left
are the original names after applying the configured `server.namespace` and
resource IDs. Targets on the right are the actual names on your system. Use
concrete absolute names; relative names, wildcards, node qualifiers and ROS
replacement expressions are not accepted in this JSON map. Use standard ROS
CLI arguments if you need those advanced rules.

| Original connection | Central definition / configuration |
|---|---|
| `/robotics/request` | `RosNames.request_service`; prefix from `server.namespace` |
| `/robot_driver/execute` | `server.driver_service` |
| `/robotics/arms/{id}/state` | `RosNames.state_topic(id)` |
| `/robotics/cameras/{id}/image/compressed` | `RosNames.camera_topic(id)` |
| `/robotics/cameras/{id}/camera_info` | `RosNames.camera_info_topic(id)` |
| `/robotics/cameras/{id}/depth/aligned` | `RosNames.camera_depth_topic(id)` |
| `/tf`, `/tf_static` | Standard TF library names; override in the same `server.remappings` map |
| `/clock` when using simulation time | Standard rclpy clock subscription; override in the same map and enable `use_sim_time` |

All application name definitions and validation are in
[ros_names.py](../src/ros2_agent_server/ros2_agent_server/ros_names.py).
`Settings` inherits these fields. `RobotConfig` retains its existing convenience
methods and delegates to this module. The bridge, gateway and test driver use
the same definitions; camera suffixes are no longer assembled in node callbacks
or constructors. ROS library-owned endpoints such as parameter services and
`/rosout` still follow the normal ROS naming/remapping rules.

## Behavior and precedence

- Omitting `remappings`, or using `{}`, preserves all previous names. Existing
  `server.namespace` and `server.driver_service` settings remain compatible.
  `namespace` prefixes application topics and the bridge request service; it
  does not rename TF, clock, node namespaces or the separate driver service.
- Changing `server.namespace` to `/cell/robot` means that a request-service
  override must use `/cell/robot/request` as its source. Changing
  `driver_service` likewise changes the source to match. Overrides are normal
  ROS remapping rules, not a chain of string substitutions.
- Explicit command-line `--ros-args -r ...` rules take precedence over matching
  JSON rules when using the supplied CLI. The runtime forwards these arguments
  to both nodes before the JSON defaults. With embedded Python nodes, explicit
  node `cli_args` similarly take precedence; JSON rules are node-local and take
  precedence over context/global rules according to normal ROS behavior.
- The agent uses HTTP and ignores `server`; arm/camera IDs and HTTP paths do not
  change when ROS names change. It can load the same topology JSON file.
- Remapping changes endpoint names only. It does not convert message types,
  image geometry, `header.frame_id`, TF frame IDs or timestamps.
- External camera/controller/driver nodes must publish or serve the **target**
  names, or receive corresponding remaps in their own launch setup. This JSON
  configures the two server nodes; it does not reconfigure independent hardware
  nodes automatically. The repository test driver also consumes the shared map.
- Settings are read at startup. Restart the affected server roles after changing
  names and follow the existing stopped-state/recovery procedure for hardware.

For example, run both server roles with the same file from the repository root:

```bash
python -m ros2_agent_server --role robot --config server/configs/remapped.json
python -m ros2_agent_server --role http --config server/configs/remapped.json
```

Use separate terminals with the normal ROS/server environment. For inspection,
check `ros2 node info /robot_bridge` and `ros2 node info /http_gateway`: their
resolved topics and services should match the target names. No driver is started
by these commands; see [hardware integration](integration.md).

## Extending and testing

For a new application connection, define its name in `RosNames`, expose a
`RobotConfig` helper if useful, and use it at both ends. Update the connection
list and tests; do not add another hard-coded topic suffix in a node.

[ros_names_test.py](../tests/ros_names_test.py) checks existing defaults, namespace
behavior, invalid configuration and CLI argument preservation.
[ros_names_integration_test.py](../tests/ros_names_integration_test.py) runs an
HTTP agent against actual ROS services and remapped state/RGB/depth/CameraInfo,
checks dynamic/static TF and the simulation clock subscription, and verifies
explicit CLI-rule precedence in both nodes. Run both full suites using [the server test instructions](../README.md#tests).
