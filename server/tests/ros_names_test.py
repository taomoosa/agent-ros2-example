"""ROS names stay configurable through one validated settings block."""

import unittest

from ros2_agent_server.models import Settings
from ros2_agent_server.ros_names import RosNames


class RosNamesTest(unittest.TestCase):
    def test_defaults_and_namespace_apply_to_every_application_connection(self):
        settings = Settings()
        self.assertEqual('/robot_driver/execute', settings.driver_service)
        for namespace in ('/robotics', '/cell/robot'):
            settings = Settings(namespace=namespace)
            self.assertEqual(namespace+'/request', settings.request_service)
            self.assertEqual(namespace+'/arms/left/state', settings.state_topic('left'))
            self.assertEqual(namespace+'/cameras/top/image/compressed', settings.camera_topic('top'))
            self.assertEqual(namespace+'/cameras/top/camera_info', settings.camera_info_topic('top'))
            self.assertEqual(namespace+'/cameras/top/depth/aligned', settings.camera_depth_topic('top'))
        self.assertEqual({}, settings.remappings)

    def test_invalid_names_are_rejected_before_creating_ros_nodes(self):
        for invalid in ('relative', '/', '/trailing/', '/bad-name', '/a//b', '/1bad', '', '/tf:=/other'):
            for field in ('namespace', 'driver_service'):
                with self.subTest(field=field, value=invalid), self.assertRaises(ValueError):
                    Settings(**{field: invalid})
            for mapping in ({invalid:'/valid'}, {'/valid':invalid}):
                with self.subTest(mapping=mapping), self.assertRaises(ValueError):
                    Settings(remappings=mapping)
        for mapping in (None, [], {'/tf':3}, {1:'/tf'}):
            with self.subTest(mapping=mapping), self.assertRaises(ValueError):
                Settings(remappings=mapping)

    def test_node_arguments_preserve_explicit_rules_and_do_not_mutate_callers(self):
        options = {'context': object(), 'cli_args':['--ros-args','-r','/tf:=/cli/tf']}
        before = list(options['cli_args'])
        settings = RosNames(remappings={'/tf':'/configured/tf'})
        result = settings.node_options(options)
        self.assertEqual(before, options['cli_args'])
        self.assertIs(options['context'], result['context'])
        self.assertLess(result['cli_args'].index('/tf:=/cli/tf'), result['cli_args'].index('/tf:=/configured/tf'))
        self.assertEqual(options, RosNames().node_options(options))
