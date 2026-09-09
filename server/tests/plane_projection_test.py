"""Calibrated planar projection and its hardware-independent input contract."""

import copy
import math
import unittest

from helpers import config, jpeg
from ros2_agent_server.models import RobotConfig
from ros2_agent_server.pixels import PixelPlans
from ros2_agent_server.protocol import BridgeError
from ros2_agent_server.state import CameraFrame


def plane_config(name='minimal.json'):
    data = config(name).model_dump()
    data['cameras'][0].update(projection='plane', plane_calibration={
        'calibration_id': 'test-table-v1', 'image_width': 48, 'image_height': 32,
        'homography': [.01, 0., 0., 0., .02, 0., .001, 0., 1.],
        'valid_region': [2., 2., 45., 29.],
        'plane_pose': {'frame_id': 'world', 'position': [1., 2., 3.],
                       'orientation': [0., 0., math.sqrt(.5), math.sqrt(.5)]}})
    return RobotConfig.model_validate(data)


class PlaneProjectionTest(unittest.TestCase):
    def setUp(self):
        self.config = plane_config()
        self.config.server.capture_ttl = 120.
        self.clock = [0.]
        self.plans = PixelPlans(self.config, clock=lambda: self.clock[0])
        self.frame = CameraFrame(jpeg(), 'overhead_optical', 100, 1)

    def test_perspective_projection_and_plane_pose_are_frozen(self):
        captured = self.plans.capture_plane('overhead', self.frame)
        self.assertEqual('plane', captured['kind'])
        self.assertNotIn('depth_stamp_ns', captured)
        point = self.plans.project(captured['capture_id'], [10, 20])
        self.assertAlmostEqual(1.-.4/1.01, point['position'][0])
        self.assertAlmostEqual(2.+.1/1.01, point['position'][1])
        self.assertAlmostEqual(3., point['position'][2])
        self.assertEqual('test-table-v1', point['calibration_id'])
        captured['plane_calibration']['homography'][0] = 10.
        self.config.cameras[0].plane_calibration.homography[0] = 20.
        self.assertEqual(point, self.plans.project(captured['capture_id'], [10, 20]))
        self.assertEqual([10, 20], point['pixel'])
        self.assertEqual(100, point['stamp_ns'])

    def test_invalid_calibrations_fail_before_startup(self):
        base = plane_config().model_dump()
        cases = [
            ('homography', [0.]*9), ('homography', [1.,0.,0.,0.,0.,0.,0.,0.,1.]),
            ('homography', [1.,0.,0.,0.,1.,0.,1.,0.,-10.]),
            ('homography', [float('nan')]+[0.]*8), ('homography', [1.]*8),
            ('valid_region', [0.,0.,48.,31.]), ('valid_region', [20.,0.,10.,20.]),
            ('image_width', True), ('calibration_id', ''),
            ('plane_pose', {'frame_id':'other','position':[0.,0.,0.],'orientation':[0.,0.,0.,1.]}),
        ]
        for key, value in cases:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                data=copy.deepcopy(base)
                data['cameras'][0]['plane_calibration'][key]=value
                RobotConfig.model_validate(data)
        for change in ({'projection':'depth'}, {'plane_calibration':None},
                       {'mount':'flange','parent_frame':'arm_flange','arm_id':'arm'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                data=copy.deepcopy(base)
                data['cameras'][0].update(change)
                RobotConfig.model_validate(data)

    def test_grid_bounds_rgb_evidence_expiry_and_invalidation(self):
        bad_frame = CameraFrame(jpeg(), 'wrong_optical', 100, 1)
        with self.assertRaisesRegex(ValueError, 'plane.frame_id'):
            self.plans.capture_plane('overhead', bad_frame)
        self.config.cameras[0].plane_calibration.image_width = 64
        with self.assertRaisesRegex(ValueError, 'plane.image_dimensions'):
            self.plans.capture_plane('overhead', self.frame)
        self.config.cameras[0].plane_calibration.image_width = 48
        captured=self.plans.capture_plane('overhead', self.frame)
        for point in ([0,0], [46,30], [48,0], [3.5,4], [True,4]):
            with self.subTest(point=point), self.assertRaises(BridgeError):
                self.plans.project(captured['capture_id'], point)
        rgb=self.plans.observe('overhead', self.frame)
        with self.assertRaisesRegex(BridgeError, 'RGB observation'):
            self.plans.project(rgb['capture_id'], [10,20])
        self.clock[0]=121.
        with self.assertRaisesRegex(BridgeError, 'expired'):
            self.plans.project(captured['capture_id'], [10,20])
        fresh=self.plans.capture_plane('overhead', self.frame)
        self.plans.invalidate()
        with self.assertRaisesRegex(BridgeError, 'Unknown capture_id'):
            self.plans.project(fresh['capture_id'], [10,20])

    def test_scaled_matrix_and_tilted_plane(self):
        calibration=self.config.cameras[0].plane_calibration
        calibration.homography=[v*-1e-8 for v in calibration.homography]
        calibration.plane_pose.orientation=[math.sqrt(.5),0.,0.,math.sqrt(.5)]
        config_copy=RobotConfig.model_validate(self.config.model_dump())
        store=PixelPlans(config_copy)
        captured=store.capture_plane('overhead', self.frame)
        point=store.project(captured['capture_id'], [10,20])
        self.assertAlmostEqual(1.+.1/1.01, point['position'][0])
        self.assertAlmostEqual(2., point['position'][1])
        self.assertAlmostEqual(3.+.4/1.01, point['position'][2])

    def test_plan_keeps_grasp_and_release_projection_provenance(self):
        captured=self.plans.capture_plane('overhead', self.frame)
        result=self.plans.create(captured['capture_id'], [dict(arm_id='arm',grasp=[10,20],release=[20,10])])
        self.assertEqual('detected', result['state'])
        for key in ('grasp','release'):
            self.assertEqual('plane', result['targets'][0][key]['projection'])
            self.assertEqual('test-table-v1', result['targets'][0][key]['calibration_id'])
