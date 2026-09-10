"""Physical units, rounding residuals and acceptance boundaries stay distinct."""

import copy
import math
import struct
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

from sensor_msgs.msg import CameraInfo

from helpers import arm_state, config, jpeg
import pixels_test
from plane_projection_test import plane_config
from ros2_agent_server.models import RobotConfig
from ros2_agent_server.numerics import calibration_key, same_calibration
from ros2_agent_server.pixels import PixelPlans, transform_point
from ros2_agent_server.protocol import BridgeError
from ros2_agent_server.robot_node import RobotBridgeNode
from ros2_agent_server.state import CameraFrame, StateStore


class NumericToleranceTest(unittest.TestCase):
    def setUp(self):
        self.g = pixels_test.PixelProjectionTest()
        self.g.setUp()
        self.camera = self.g.config.cameras[0]

    def depth(self, values):
        self.g.depth.encoding = '32FC1'
        self.g.depth.step = 16
        self.g.depth.data = struct.pack('<12f', *values)
        capture = self.g.capture()
        return self.g.store.project(capture['capture_id'], [1, 1])['position']

    def test_rectification_residuals_are_canonicalized_and_bounded(self):
        g = self.g
        t = self.camera.rectification_tolerance
        g.info.d = [t, -t]
        for i in (1, 3, 6, 7):
            g.info.k[i] = t
        capture = g.capture()
        stored = g.store.captures[capture['capture_id']][2]
        self.assertEqual([2., 0., 1., 0., 2., 1., 0., 0., 1.], stored)
        for value in (math.nextafter(t, math.inf), float('nan'), float('inf')):
            g.info.d = [value]
            with self.assertRaisesRegex(ValueError, 'camera_info.d'):
                g.capture()
        g.info.d = []
        g.info.k[1] = math.nextafter(t, math.inf)
        with self.assertRaisesRegex(ValueError, r'camera_info.k\[1\]'):
            g.capture()

    def test_calibration_baseline_retains_evidence_until_cumulative_change(self):
        c = plane_config()
        plans = PixelPlans(c)
        bridge = SimpleNamespace(config=c, _lock=threading.RLock(), _calibration_keys={},
            _calibration={}, _depth={}, _pending=[], _invalid_plane_cameras=set(),
            store=mock.Mock(), _invalidate_geometry=plans.invalidate, get_logger=lambda: mock.Mock())
        info = CameraInfo(width=48, height=32, k=[100.,0.,24.,0.,100.,16.,0.,0.,1.])
        info.header.frame_id = 'overhead_optical'
        RobotBridgeNode._camera_info(bridge, 'overhead', info)
        cap = plans.capture_plane('overhead', CameraFrame(jpeg(), 'overhead_optical', 100, 1))
        for delta in (1e-12, .4e-6, .8e-6):
            changed = copy.deepcopy(info)
            changed.k[0] += delta
            RobotBridgeNode._camera_info(bridge, 'overhead', changed)
            self.assertIn(cap['capture_id'], plans.captures)
            self.assertFalse(bridge._invalid_plane_cameras)
        changed.k[0] = info.k[0]+1.2e-6
        RobotBridgeNode._camera_info(bridge, 'overhead', changed)
        self.assertNotIn(cap['capture_id'], plans.captures)
        self.assertIn('overhead', bridge._invalid_plane_cameras)

    def test_calibration_equivalence_keeps_ids_exact_and_invalid_residual_visible(self):
        a = CameraInfo(width=4, height=3, k=[2.,0.,1.,0.,2.,1.,0.,0.,1.])
        b = copy.deepcopy(a)
        b.binning_x = b.binning_y = 1
        b.roi.width, b.roi.height = 4, 3
        b.d = [1e-12]*5
        self.assertTrue(same_calibration(calibration_key(a), calibration_key(b), self.camera))
        b.header.frame_id = 'other'
        self.assertFalse(same_calibration(calibration_key(a), calibration_key(b), self.camera))
        b.header.frame_id = a.header.frame_id
        a.k[0], b.k[0] = 1e-8, -1e-8
        self.assertFalse(same_calibration(calibration_key(a), calibration_key(b), self.camera))
        a.k[0] = b.k[0] = 2.
        a.k[1], b.k[1] = .9e-9, 1.1e-9
        self.assertFalse(same_calibration(calibration_key(a), calibration_key(b), self.camera))

    def test_nonbinary_depth_and_sensor_noise_use_separate_error_budgets(self):
        point = self.depth([1.234]*12)
        self.assertAlmostEqual(4.234, point[2], delta=1e-6)
        values = [1.234 + (i % 3-1)*.005 for i in range(12)]
        point = self.depth(values)
        self.assertAlmostEqual(4.234, point[2], delta=.005001)
        # The selected depth is preserved, not replaced by a neighbourhood mean.
        self.assertAlmostEqual(values[5]+3., point[2], delta=1e-6)

    def test_positive_outlier_holes_and_surface_boundary_are_rejected(self):
        for selected in (60., 2., 0., -.1, float('nan'), float('inf')):
            values = [1.234]*12
            values[5] = selected
            with self.subTest(selected=selected), self.assertRaises(BridgeError) as error:
                self.depth(values)
            self.assertEqual('depth_quality_invalid', error.exception.payload['code'])
        for values in ([float('nan')]*5+[1.234]+[0.]*6,
                       [1.,2.,2.,2.,1.,1.,2.,2.,1.,2.,2.,2.]):
            with self.assertRaisesRegex(BridgeError, 'depth.support'):
                self.depth(values)

    def test_depth_range_and_support_tolerances_can_be_tuned(self):
        self.camera.depth_min_m = .5
        self.camera.depth_max_m = 2.
        for value in (.5, 2.):
            self.assertAlmostEqual(value+3., self.depth([value]*12)[2], delta=1e-6)
        for value in (.49, 2.01):
            with self.assertRaisesRegex(BridgeError, 'depth.range_m'):
                self.depth([value]*12)
        self.camera.depth_absolute_tolerance_m = .015625
        self.camera.depth_relative_tolerance = 0.
        values = [1.015625]*12
        values[5] = 1.
        self.assertAlmostEqual(4., self.depth(values)[2], delta=1e-6)
        values = [1.015626]*12
        values[5] = 1.
        with self.assertRaisesRegex(BridgeError, 'depth.support'):
            self.depth(values)
        self.camera.depth_absolute_tolerance_m = 0.
        self.camera.depth_relative_tolerance = .02
        self.assertAlmostEqual(4., self.depth(values)[2], delta=1e-6)

    def test_float32_roundoff_does_not_reject_configured_depth_boundaries(self):
        self.camera.depth_min_m, self.camera.depth_max_m = .7, 1.234
        for value in (.7, 1.234):
            self.assertAlmostEqual(value+3., self.depth([value]*12)[2], delta=1e-6)
        for value in (.69999, 1.23401):
            with self.assertRaisesRegex(BridgeError, 'depth.range_m'):
                self.depth([value]*12)
        self.camera.depth_absolute_tolerance_m, self.camera.depth_relative_tolerance = .02, 0.
        values = [1.02]*12
        values[5] = 1.
        self.assertAlmostEqual(4., self.depth(values)[2], delta=1e-6)
        values = [1.02001]*12
        values[5] = 1.
        with self.assertRaisesRegex(BridgeError, 'depth.support'):
            self.depth(values)

    def test_future_skew_boundary_does_not_relax_replay_or_post_motion_order(self):
        c = config('minimal.json')
        store = StateStore(c)
        now = 1_000_000_000
        tolerance = round(c.server.future_skew_tolerance_sec*1e9)
        store.update_arm('arm', arm_state(), now+tolerance)
        self.assertEqual(now+tolerance, store.state(now)['arms'][0]['measurement_stamp_ns'])
        with self.assertRaises(BridgeError):
            store.state(now-1)
        revisions = store.arm_revisions(['arm'])
        with self.assertRaisesRegex(ValueError, 'did not advance'):
            store.update_arm('arm', arm_state(), now+tolerance)
        store.update_arm('arm', arm_state(), now+tolerance+1)
        self.assertFalse(store.measured_after(['arm'], now+tolerance+2, revisions))
        c.server.future_skew_tolerance_sec = 0.
        with self.assertRaises(BridgeError):
            store.state(now+tolerance)

    def test_ros_state_callback_applies_same_future_skew_limit(self):
        c = config('minimal.json')
        store = StateStore(c)
        bridge = SimpleNamespace(config=c, store=store, get_logger=lambda: mock.Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))
        import json
        limit = 1_005_000_000
        for stamp in (limit, limit+1):
            RobotBridgeNode._arm_state(bridge, 'arm', SimpleNamespace(
                data=json.dumps(dict(arm_state(), stamp_ns=stamp))))
        self.assertEqual(limit, store.state(1_000_000_000)['arms'][0]['measurement_stamp_ns'])

    def test_default_pairing_has_tolerance_and_strict_mode_is_available(self):
        self.g.depth.header.stamp.nanosec += 5_000_000
        self.assertEqual(5_000_000, self.g.capture()['sync_delta_ns'])
        self.camera.sync_tolerance_sec = 0.
        with self.assertRaisesRegex(ValueError, 'depth.stamp_delta_ns'):
            self.g.capture()

    def test_camera_future_skew_boundary_preserves_original_timestamps(self):
        import time
        from ros2_agent_server.robot_node import Pending
        c = config('minimal.json')
        store = StateStore(c)
        now = 1_000_000_000
        latest = now + round(c.server.future_skew_tolerance_sec*1e9)
        pending = Pending(mock.Mock(), time.monotonic()+10., camera=('overhead', 0, now))
        bridge = SimpleNamespace(config=c, store=store, _lock=threading.RLock(),
            _pending=[pending], _waiting=mock.Mock(), _complete=mock.Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now)))
        for stamp, accepted in ((latest, True), (latest+1, False), (now-1, False)):
            store.clear_cameras()
            store.update_camera('overhead', jpeg(), 'overhead_optical', stamp)
            bridge._complete.reset_mock()
            RobotBridgeNode._try_cameras(bridge)
            self.assertEqual(accepted, bridge._complete.called)
            if accepted:
                self.assertEqual(stamp, bridge._complete.call_args.args[1].payload['stamp_ns'])

    def test_noninteger_intrinsics_use_metric_calculation_tolerance(self):
        self.g.info.k = [3.1234,0.,.9234,0.,2.7891,.8765,0.,0.,1.]
        point = self.depth([1.234]*12)
        expected = [1.+(1.-.9234)*1.234/3.1234, 2.+(1.-.8765)*1.234/2.7891, 4.234]
        for actual, target in zip(point, expected):
            self.assertAlmostEqual(actual, target, delta=1e-6)

    def test_rounded_quaternion_is_normalized_before_rotating(self):
        angle = .731
        q = [0., 0., math.sin(angle/2), math.cos(angle/2)]
        pose = dict(position=[.123, -.456, .789], orientation=[v*1.0001 for v in q])
        actual = transform_point([1.234, -.321, .567], pose)
        expected = [.123+1.234*math.cos(angle)+.321*math.sin(angle),
                    -.456+1.234*math.sin(angle)-.321*math.cos(angle), .789+.567]
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=1e-12)
        pose['orientation'] = [v*1.01 for v in q]
        with self.assertRaisesRegex(ValueError, 'quaternion'):
            transform_point([1., 0., 0.], pose)

    def test_invalid_tolerance_configuration_is_rejected(self):
        data = config('minimal.json').model_dump()
        for field, value in (('calibration_tolerance_px', .1), ('rectification_tolerance', float('nan')),
                             ('depth_min_m', 6.), ('depth_max_m', .01), ('depth_min_support', .5),
                             ('depth_relative_tolerance', -.1), ('depth_absolute_tolerance_m', True)):
            invalid = copy.deepcopy(data)
            invalid['cameras'][0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                RobotConfig.model_validate(invalid)
        data['server']['future_skew_tolerance_sec'] = .2
        with self.assertRaises(ValueError):
            RobotConfig.model_validate(data)

    def test_default_accepts_lens_distortion_metadata_and_projects_rectified_pixels(self):
        from embodiment.ros2.config import Camera as AgentCamera
        from ros2_agent_server.numerics import rectified_intrinsics
        self.assertEqual('ros_rectified',config('minimal.json').cameras[0].camera_info_mode)
        self.assertEqual('ros_rectified',AgentCamera('c','optical','world','world').camera_info_mode)
        g=self.g
        g.config.cameras[0].camera_info_mode='ros_rectified'
        g.info.p=[3.2,0.,1.1,0.,0.,3.4,.9,0.,0.,0.,1.,0.]
        g.info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        for coefficients in ([-.28,.09,.001,-.002,-.015],[.12,-.04,0.,0.,.003],[.2,-.1,.01,-.005]):
            g.info.d=coefficients
            capture=g.capture()
            point=g.store.project(capture['capture_id'],[2,1])['position']
            for actual,expected in zip(point,[1.+.9*2./3.2,2.+.1*2./3.4,5.]):
                self.assertAlmostEqual(expected,actual,delta=1e-6)
        g.info.d=[float('nan')]
        with self.assertRaisesRegex(ValueError,'camera_info.d'):g.capture()
        g.info.d=[-.28,.09,.001,-.002,-.015]
        g.info.p=[0.]*12
        with self.assertRaisesRegex(ValueError,'calibrated rectified P'):g.capture()
        g.config.cameras[0].camera_info_mode='rectified_k'
        with self.assertRaisesRegex(ValueError,'ros_rectified'):g.capture()
