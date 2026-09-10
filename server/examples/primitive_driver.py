"""Hardware adapter starting point. Unimplemented hooks always fail explicitly.

Run after sourcing ROS and the server install:
  python server/examples/primitive_driver.py --config server/configs/primitives.json \
      --hardware-config server/examples/robot_hardware.json
Provide measured arm state and camera topics independently. Replace the hooks
below with your controller integration before enabling physical motion.
"""
import argparse
import copy
import json
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from ros2_agent_server.models import RobotConfig
from ros2_agent_server.primitive_driver import HardwareBackend, driver_node
from ros2_agent_server.protocol import BridgeError


class RobotBackend(HardwareBackend):
    # HARDWARE INTEGRATION: declare capabilities only after implementing/testing them.
    # coordinated_motion = coordinated_gripper = coupled_transfer = True
    # joint_targets = True
    # recovery = True

    def __init__(self, hardware):
        # HARDWARE INTEGRATION: load name -> goal pairs here, from your controller
        # or a backend-only file. Shared metadata contains names and descriptions only.
        # The example xyz/quaternion values are placeholders, NOT a position schema.
        # Add your mechanism-specific format, units and interpretation here when
        # integrating the controller; no frame_id or common pose model is required.
        self.named_positions = copy.deepcopy(hardware.get('named_positions', {}))

    def resolve_targets(self, targets, arm_ids):
        resolved = []
        for arm, target in zip(arm_ids, targets):
            if target['kind'] == 'named':
                name = target['name']
                if name not in self.named_positions.get(arm, {}):
                    raise BridgeError(422, f'Backend has no named position {name!r} for arm {arm!r}')
                target = self.named_positions[arm][name]
            # HARDWARE INTEGRATION: resolve/validate your own controller format.
            # Named values are opaque to the agent, server and common adapter.
            resolved.append(copy.deepcopy(target))
        return resolved

    async def prepare(self, steps, arm_ids, coupled, context):
        # Validate every goal/path and the entire group without moving any arm.
        # Resolve names and pass TCP goals to your controller's TCP/end-effector
        # interface. Convert to flange only if that controller requires it.
        # Freeze resolved goals/trajectories for this sequence after validation;
        # move must not reread mutable named positions or recalibrate mid-sequence.
        for step in steps:
            context.check()
            if step['operation'] == 'move':
                resolved_targets = self.resolve_targets(step['targets'], arm_ids)
                # HARDWARE INTEGRATION: validate and plan resolved_targets here.
        raise BridgeError(501, 'Implement controller preflight in RobotBackend.prepare')

    async def move(self, targets, arm_ids, duration, coupled, context):
        # targets contains explicit tcp/flange poses or backend-owned names.
        # Execute the controller plan retained by prepare, respecting reference.
        # Await rclpy controller futures; check context while awaiting completion.
        # Coupled groups require synchronized paths and relative grasp constraints.
        # Return success only after measured arrival/settling, not action acceptance.
        raise BridgeError(501, 'Implement controller execution in RobotBackend.move')

    async def gripper(self, openings, arm_ids, context):
        # Map normalized opening to your hardware. Validate completion/contact;
        # publish measured opening and object_detected separately when available.
        raise BridgeError(501, 'Implement gripper execution in RobotBackend.gripper')

    async def stop(self, arm_ids, context):
        # Cancel controller motion and confirm physical stopping. This hook can
        # run while another hook awaits a controller; never wait for that hook.
        raise BridgeError(501, 'Implement physical stopping in RobotBackend.stop', outcome='unknown')

    async def recover(self, arm_ids, context):
        # Optional: support/release payload, clear eligible faults, retreat/home.
        raise BridgeError(501, 'Safe recovery has not been implemented')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--hardware-config', required=True, help='Backend-only named goals and controller configuration')
    args, ros_args = parser.parse_known_args()
    config = RobotConfig.load(args.config)
    rclpy.init(args=ros_args)
    node = driver_node(config, RobotBackend(json.loads(Path(args.hardware_config).read_text())))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
