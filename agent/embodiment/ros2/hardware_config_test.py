"""The agent accepts shared input geometry settings and rejects invalid contracts."""

import dataclasses
import unittest

from embodiment.ros2.config import Camera


class HardwareConfigTest(unittest.TestCase):
  def test_shared_camera_geometry_fields_and_validation(self):
    camera = Camera(id='overhead', optical_frame='color_optical', mount='world', parent_frame='world',
                    depth_frame='registered_depth', camera_info_frame='color_calibration', sync_tolerance_sec=.01)
    self.assertEqual('registered_depth', camera.depth_frame)
    for field, value in (('depth_frame', '/absolute'), ('camera_info_frame', ''),
                         ('depth_geometry', 'raw_depth'), ('sync_tolerance_sec', float('nan')),
                         ('sync_tolerance_sec', True), ('sync_tolerance_sec', -.1), ('sync_tolerance_sec', .2),
                         ('calibration_tolerance_px', .1), ('rectification_tolerance', float('nan')),
                         ('depth_min_m', 6.), ('depth_max_m', .01), ('depth_min_support', .5),
                         ('depth_relative_tolerance', -.1), ('depth_absolute_tolerance_m', True)):
      with self.subTest(field=field, value=value), self.assertRaises(ValueError):
        dataclasses.replace(camera, **{field: value})

  def test_camera_tolerances_load_from_shared_json(self):
    import json
    from pathlib import Path
    import tempfile
    from embodiment.ros2.config import RobotConfig
    sample = Path(__file__).resolve().parents[2] / 'configs/minimal.json'
    data = json.loads(sample.read_text())
    data['cameras'][0].update(calibration_tolerance_px=2e-6, rectification_tolerance=2e-9,
        depth_min_m=.2, depth_max_m=3., depth_absolute_tolerance_m=.01,
        depth_relative_tolerance=.02, depth_min_support=.75)
    data['server'] = {'future_skew_tolerance_sec': .002}
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / 'robot.json'
      path.write_text(json.dumps(data))
      camera = RobotConfig.load(path).cameras[0]
    self.assertEqual(.01, camera.sync_tolerance_sec)
    self.assertEqual(2e-6, camera.calibration_tolerance_px)
    self.assertEqual(.75, camera.depth_min_support)
