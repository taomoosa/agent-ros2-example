"""Shared planar topology is validated without ROS or server dependencies."""

import copy
import dataclasses
from pathlib import Path
import unittest

from embodiment.ros2.config import RobotConfig


class PlaneConfigTest(unittest.TestCase):
  def test_shared_config_loads_and_rejects_incompatible_topology(self):
    config=RobotConfig.load(Path(__file__).resolve().parents[2]/'configs/planar.json')
    camera=config.cameras[0]
    self.assertEqual('plane',camera.projection)
    self.assertEqual('example-table-v1',camera.plane_calibration['calibration_id'])
    for change in ({'projection':'depth'}, {'plane_calibration':None}, {'projection':'unknown'},
                   {'mount':'flange','arm_id':'arm','parent_frame':'arm_flange'}):
      with self.subTest(change=change),self.assertRaises(ValueError):
        dataclasses.replace(camera,**change)
    plane=copy.deepcopy(camera.plane_calibration)
    plane['plane_pose']['frame_id']='other'
    with self.assertRaisesRegex(ValueError,'world_frame'):
      dataclasses.replace(config,cameras=(dataclasses.replace(camera,plane_calibration=plane),))

  def test_malformed_calibration_is_rejected(self):
    config=RobotConfig.load(Path(__file__).resolve().parents[2]/'configs/planar.json')
    camera=config.cameras[0]
    for key,value in (('homography',[0.]*9),('homography',[float('inf')]+[0.]*8),
                      ('homography',[1.,0.,0.,0.,1.,0.,1.,0.,-100.]),
                      ('image_width',True),('valid_region',[0.,0.,640.,480.]),
                      ('calibration_id',''),('plane_pose',{}),('unknown',1)):
      with self.subTest(key=key,value=value),self.assertRaises(ValueError):
        plane=copy.deepcopy(camera.plane_calibration)
        plane[key]=value
        dataclasses.replace(camera,plane_calibration=plane)
