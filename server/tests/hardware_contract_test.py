"""Measured-time, geometry and diagnostic contracts independent of ROS scheduling."""

import copy
import json
import unittest
from unittest import mock

from ros2_agent_server.diagnostics import event
from ros2_agent_server.pixels import PixelPlans
from ros2_agent_server.protocol import BridgeError
from ros2_agent_server.state import StateStore
from helpers import arm_state, config, jpeg
import pixels_test


class HardwareContractTest(unittest.TestCase):
    def setUp(self):
        self.geometry = pixels_test.PixelProjectionTest()
        self.geometry.setUp()
        self.g = self.geometry

    def test_tolerance_boundary_and_declared_frame_alias_preserve_geometry(self):
        g = self.g
        camera = g.config.cameras[0]
        camera.sync_tolerance_sec = 0.01
        camera.depth_frame = 'registered_depth_header'
        camera.camera_info_frame = 'calibrated_color_header'
        g.depth.header.frame_id = camera.depth_frame
        g.info.header.frame_id = camera.camera_info_frame
        g.depth.header.stamp.nanosec += 10_000_000
        result = g.capture()
        self.assertEqual(10_000_000, result['sync_delta_ns'])
        self.assertEqual(100, result['pose_stamp_ns'])
        self.assertEqual([2., 3., 5.], g.store.project(result['capture_id'], [2, 2])['position'])
        g.depth.header.stamp.nanosec += 1
        with self.assertRaisesRegex(ValueError, 'depth.stamp_delta_ns'):
            g.capture()
        g.depth.header.stamp.nanosec = 100
        g.depth.header.frame_id = 'unregistered_raw_depth'
        with self.assertRaisesRegex(ValueError, 'depth.frame_id'):
            g.capture()

    def test_each_geometry_condition_has_specific_diagnostics(self):
        cases = [
            ('depth.frame_id', lambda g: setattr(g.depth.header, 'frame_id', 'wrong')),
            ('camera_info.frame_id', lambda g: setattr(g.info.header, 'frame_id', 'wrong')),
            ('depth.dimensions', lambda g: setattr(g.depth, 'width', 5)),
            ('camera_info.dimensions', lambda g: setattr(g.info, 'height', 4)),
            ('camera_info.d', lambda g: setattr(g.info, 'd', [.1])),
            ('camera_info.binning_x', lambda g: setattr(g.info, 'binning_x', 2)),
            ('camera_info.roi.x_offset', lambda g: setattr(g.info.roi, 'x_offset', 1)),
            ('depth.encoding', lambda g: setattr(g.depth, 'encoding', '8UC1')),
            ('depth.step', lambda g: setattr(g.depth, 'step', 1)),
            ('depth.data_length', lambda g: setattr(g.depth, 'data', b'bad')),
            ('camera_info.k[0]', lambda g: g.info.k.__setitem__(0, 0.)),
            ('camera_info.k[4]', lambda g: g.info.k.__setitem__(4, float('nan'))),
            ('camera_info.k[8]', lambda g: g.info.k.__setitem__(8, 2.)),
        ]
        for field, change in cases:
            with self.subTest(field=field):
                self.g.setUp()
                change(self.g)
                with self.assertRaises(ValueError) as error:
                    self.g.capture()
                self.assertIn(field, str(error.exception))
                self.assertIn('expected', str(error.exception))
                self.assertIn('actual', str(error.exception))
                json.dumps(BridgeError(422, str(error.exception)).payload, allow_nan=False)

    def test_rgb_evidence_cannot_create_or_refine_metric_targets(self):
        g = self.g
        rgb = g.store.observe(g.config.cameras[0].id, g.frame)
        self.assertEqual('rgb', rgb['kind'])
        self.assertNotIn('camera_pose', rgb)
        with self.assertRaisesRegex(BridgeError, 'no depth'):
            g.store.project(rgb['capture_id'], [1, 1])
        with self.assertRaises(BridgeError):
            g.store.create(rgb['capture_id'], [dict(arm_id='left', grasp=[1,1], release=[2,2])])

    def test_measurement_replay_and_clock_age_are_not_receipt_freshness(self):
        c = config('minimal.json')
        store = StateStore(c)
        store.update_arm('arm', arm_state(), 1_000_000_000)
        self.assertEqual(1_000_000_000, store.state(1_100_000_000)['arms'][0]['measurement_stamp_ns'])
        for stamp in (1_000_000_000, 900_000_000):
            with self.assertRaisesRegex(ValueError, 'did not advance'):
                store.update_arm('arm', arm_state(), stamp)
        with self.assertRaises(BridgeError):
            store.state(4_000_000_000)
        with self.assertRaises(BridgeError):
            store.state(900_000_000)
        revisions = store.arm_revisions(['arm'])
        self.assertFalse(store.measured_after(['arm'], 1_000_000_000, revisions))
        store.update_arm('arm', arm_state(), 1_200_000_000)
        self.assertTrue(store.measured_after(['arm'], 1_100_000_000, revisions))
        store.reset()
        with self.assertRaises(BridgeError):
            store.state(1_300_000_000)

    def test_json_telemetry_preserves_unknown_detection_opening_and_fault(self):
        from ros2_agent_server.models import ArmState
        for detected in (None, False, True):
            state = arm_state()
            state['gripper'] = dict(opening=None, object_detected=detected)
            state['fault'] = dict(code='stall', message='Motor stalled', recoverable=True)
            decoded = ArmState.model_validate(json.loads(json.dumps(state)))
            self.assertIs(detected, decoded.gripper.object_detected)
            self.assertIsNone(decoded.gripper.opening)
            self.assertEqual('stall', decoded.fault.code)
            self.assertTrue(decoded.fault.recoverable)
        state['gripper']['object_detected'] = -1
        with self.assertRaises(ValueError):
            ArmState.model_validate(state)

    def test_rgb_history_accepts_reordering_and_is_bounded(self):
        c = config('minimal.json')
        c.server.camera_buffer_size = 2
        store = StateStore(c)
        for stamp in (300, 200, 400):
            store.update_camera('overhead', jpeg(), 'overhead_optical', stamp)
        self.assertEqual([200,400], [f.stamp_ns for f in store.fresh_cameras('overhead', 0, 100)])
        self.assertEqual([400], [f.stamp_ns for f in store.fresh_cameras('overhead', 2, 100)])
        self.assertFalse(store.update_camera('overhead', jpeg(), 'overhead_optical', 400))

    def test_diagnostics_serialize_only_selected_metadata(self):
        logger = mock.Mock()
        event(logger, 'capture_wait', request_id='r1', camera='overhead', reason='missing_depth')
        row = json.loads(logger.debug.call_args.args[0])
        self.assertEqual('capture_wait', row['event'])
        self.assertEqual('r1', row['request_id'])
        self.assertNotIn('image_base64', row)
