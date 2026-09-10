"""Shared name overrides reach actual ROS endpoints, including library-owned TF."""

import unittest
from unittest import mock

import httpx
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from helpers import ROOT, RosFixture, eventually, config
from ros2_agent_server.models import RobotConfig
from ros2_agent_server.api import create_app
from ros2_agent_server.gateway import HttpGatewayNode
from ros2_agent_server.robot_node import RobotBridgeNode


class RosNamesIntegrationTest(unittest.IsolatedAsyncioTestCase):
    async def test_http_agent_with_remapped_services_topics_and_dynamic_static_tf(self):
        from embodiment.ros2.config import RobotConfig as AgentConfig
        from embodiment.ros2.robot_client import Ros2RobotClient
        c = RobotConfig.load(ROOT/'server/configs/remapped.json')
        c.server.hardware = config('minimal.json').server.hardware
        f = RosFixture(c)
        self.addCleanup(f.close)
        await f.ready()
        transport = httpx.ASGITransport(app=create_app(f.gateway,c))
        agent = Ros2RobotClient(AgentConfig.load(ROOT/'server/configs/remapped.json'), transport=transport)
        try:
            state = await agent.get_robot_state()
            self.assertEqual('arm', state['arms'][0]['id'])
            captured = await agent.capture('overhead')
            self.assertEqual('rgbd', captured['kind'])
            reset = await agent.workflow('reset')
            self.assertTrue(reset['success'])
            for source,target in c.server.remappings.items():
                actual = f.robot.resolve_service_name(source) if source in (c.request_service,c.server.driver_service) else f.robot.resolve_topic_name(source)
                self.assertEqual(target, actual)
            topic_names = {name for name,_ in f.robot.get_topic_names_and_types()}
            self.assertIn('/example/tf', topic_names)
            self.assertNotIn('/robotics/arms/arm/state', topic_names)
            # Clear dynamic history and provide only a static transform for geometry.
            with mock.patch.object(f.driver.tf_broadcaster, 'sendTransform'):
                f.robot.tf_buffer.clear()
                broadcaster = StaticTransformBroadcaster(f.driver)
                tf = TransformStamped()
                tf.header.frame_id = c.world_frame
                tf.child_frame_id = c.cameras[0].optical_frame
                tf.transform.translation.x = 2.
                tf.transform.rotation.w = 1.
                broadcaster.sendTransform(tf)
                await eventually(lambda: f.robot.tf_buffer.can_transform(c.world_frame, tf.child_frame_id, f.robot.get_clock().now()))
                captured = await agent.capture('overhead')
                self.assertAlmostEqual(2., captured['camera_pose']['position'][0])
        finally:
            await agent.close()

    async def test_explicit_ros_arguments_override_json_names_in_both_nodes(self):
        c = RobotConfig.load(ROOT/'server/configs/remapped.json')
        c.server.hardware = config('minimal.json').server.hardware
        f = RosFixture(c)
        self.addCleanup(f.close)
        await f.ready()
        c.server.remappings['/clock'] = '/example/clock'
        arguments = ['--ros-args', '-p', 'use_sim_time:=true', '-r', '/robotics/request:=/cli/request',
                     '-r', '/tf:=/cli/tf', '--']
        nodes = [HttpGatewayNode(c, context=f.context, cli_args=arguments),
                 RobotBridgeNode(c, context=f.context, cli_args=arguments)]
        try:
            for node in nodes:
                self.assertEqual('/cli/request', node.resolve_service_name(c.request_service))
                self.assertEqual('/cli/tf', node.resolve_topic_name('/tf'))
                self.assertEqual('/example/tf_static', node.resolve_topic_name('/tf_static'))
                self.assertEqual('/example/clock', node.resolve_topic_name('/clock'))
                self.assertTrue(node.get_subscriptions_info_by_topic('/example/clock'))
        finally:
            for node in nodes:
                node.destroy_node()
