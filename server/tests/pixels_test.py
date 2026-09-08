import copy
import io
import struct
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from PIL import Image
from ros2_agent_server.models import RobotConfig
from ros2_agent_server.pixels import PixelPlans, transform_point
from ros2_agent_server.protocol import BridgeError, validate_request
from ros2_agent_server.state import CameraFrame


class PixelProjectionTest(unittest.TestCase):
    def setUp(self):
        self.config = RobotConfig.load(Path(__file__).resolve().parents[2] / 'agent/configs/dual_arm.json')
        self.now = 0.
        self.store = PixelPlans(self.config, clock=lambda: self.now)
        buffer = io.BytesIO()
        Image.new('RGB', (4, 3)).save(buffer, 'JPEG')
        self.frame = CameraFrame(buffer.getvalue(), self.config.cameras[0].optical_frame, 100, 1)
        header = NS(frame_id=self.frame.frame_id, stamp=NS(sec=0, nanosec=100))
        self.info = NS(header=header, width=4, height=3, d=[], binning_x=0, binning_y=0,
                       roi=NS(do_rectify=False), k=[2.,0.,1.,0.,2.,1.,0.,0.,1.])
        self.depth = NS(header=copy.deepcopy(header), width=4, height=3, encoding='16UC1',
                        step=10, is_bigendian=False, data=(struct.pack('<4H', *([2000]*4))+b'xx')*3)
        self.pose = dict(frame_id='world', position=[1.,2.,3.], orientation=[0.,0.,0.,1.])

    def capture(self):
        return self.store.capture(self.config.cameras[0].id, self.frame, self.info, self.depth, self.pose)

    def test_projection_uses_frozen_pose_depth_and_row_stride(self):
        cap = self.capture()
        self.pose['position'][0] = 99.
        self.depth.data = b'\0'*30
        self.assertEqual([2.,3.,5.], self.store.project(cap['capture_id'], [2,2])['position'])
        self.assertEqual([1.,2.,3.], cap['camera_pose']['position'])

    def test_quaternion_rotation(self):
        result = transform_point([1,0,0], dict(position=[1,2,3], orientation=[0,0,2**-.5,2**-.5]))
        for actual, expected in zip(result, [1,3,3]):
            self.assertAlmostEqual(expected, actual)

    def test_float_big_endian_depth(self):
        self.depth.encoding, self.depth.is_bigendian, self.depth.step = '32FC1', True, 16
        self.depth.data = struct.pack('>12f', *([1.5]*12))
        cap = self.capture()
        self.assertEqual([1.,2.,4.5], self.store.project(cap['capture_id'], [1,1])['position'])

    def test_invalid_pixels_depth_and_expired_capture(self):
        cap = self.capture()
        for pixel in ([4,0], [-1,0], [True,0], [1.2,1], [1]):
            with self.subTest(pixel=pixel), self.assertRaises(BridgeError):
                self.store.project(cap['capture_id'], pixel)
        self.now = 121
        with self.assertRaises(BridgeError):
            self.store.project(cap['capture_id'], [1,1])
        self.depth.data = b'\0'*30
        cap = self.capture()
        with self.assertRaises(BridgeError):
            self.store.project(cap['capture_id'], [1,1])

    def test_reject_mismatched_capture_and_distortion(self):
        self.depth.header.stamp.nanosec = 101
        with self.assertRaises(ValueError):
            self.capture()
        self.depth.header.stamp.nanosec = 100
        self.info.d = [.1]
        with self.assertRaises(ValueError):
            self.capture()

    def test_ros_validation_rejects_duplicate_arms_and_unknown_fields(self):
        target = dict(arm_id='left', grasp=[1,1], release=[2,2])
        for body in (dict(capture_id='c', targets=[target, target]),
                     dict(capture_id='c', targets=[dict(target, arm_id='unknown')]),
                     dict(capture_id='c', targets=[target], extra=True)):
            with self.assertRaises(BridgeError):
                validate_request(self.config, 'create_plan', '', body)
