"""Capture-bound RGB-D projection and manipulation plans, independent of ROS transport."""

import base64
from collections import OrderedDict
import copy
import io
import math
import struct
import time
import uuid

from PIL import Image

from .protocol import BridgeError


def transform_point(point, pose):
    x, y, z, w = pose['orientation']
    if not all(math.isfinite(v) for v in [*point, *pose['position'], x, y, z, w]):
        raise ValueError('Non-finite geometry')
    if not math.isclose(x*x+y*y+z*z+w*w, 1, abs_tol=1e-3):
        raise ValueError('Invalid transform quaternion')
    a, b, c = point
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return [a+w*tx+y*tz-z*ty+pose['position'][0],
            b+w*ty+z*tx-x*tz+pose['position'][1],
            c+w*tz+x*ty-y*tx+pose['position'][2]]


class PixelPlans:
    def __init__(self, config, *, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self.captures = OrderedDict()
        self.plans = OrderedDict()

    def capture(self, camera_id, frame, info, depth, camera_pose, flange_pose=None):
        with Image.open(io.BytesIO(frame.data)) as image:
            width, height = image.size
        # RGB and aligned depth must originate from the same exposure. CameraInfo
        # may be latched, but must describe this rectified image geometry.
        stamp = depth.header.stamp.sec * 1_000_000_000 + depth.header.stamp.nanosec
        if (stamp != frame.stamp_ns or depth.header.frame_id != frame.frame_id
                or info.header.frame_id != frame.frame_id
                or (depth.width, depth.height) != (width, height)
                or (info.width, info.height) != (width, height)):
            raise ValueError('RGB, aligned depth and calibration do not match')
        if (any(info.d) or info.binning_x > 1 or info.binning_y > 1
                or info.roi.do_rectify or getattr(info.roi, 'x_offset', 0) or getattr(info.roi, 'y_offset', 0)):
            raise ValueError('Publish full-resolution rectified RGB and aligned depth')
        if depth.encoding not in ('16UC1', '32FC1'):
            raise ValueError('Depth encoding must be 16UC1 (mm) or 32FC1 (m)')
        size = 2 if depth.encoding == '16UC1' else 4
        if depth.step < width*size or len(depth.data) != depth.step*height:
            raise ValueError('Invalid depth buffer or row stride')
        k = list(info.k)
        if not all(math.isfinite(v) for v in k) or k[0] <= 0 or k[4] <= 0 or k[1] or k[3] or k[6:9] != [0., 0., 1.]:
            raise ValueError('Invalid rectified pinhole intrinsics')
        camera = next(c for c in self.config.cameras if c.id == camera_id)
        if camera.mount == 'flange' and flange_pose is None:
            raise ValueError('Capture-time flange transform is required')
        transform_point([0., 0., 0.], camera_pose)
        if flange_pose is not None:
            transform_point([0., 0., 0.], flange_pose)
        metadata = dict(capture_id=uuid.uuid4().hex, camera_id=camera_id,
                        width=width, height=height, stamp_ns=frame.stamp_ns,
                        frame_id=frame.frame_id, camera_pose=copy.deepcopy(camera_pose),
                        flange_pose=copy.deepcopy(flange_pose),
                        image_base64=base64.b64encode(frame.data).decode())
        self.captures[metadata['capture_id']] = (self.clock(), metadata, k, copy.deepcopy(depth))
        while len(self.captures) > 64:
            self.captures.popitem(last=False)
        return copy.deepcopy(metadata)

    def get_capture(self, capture_id):
        item = self.captures.get(capture_id)
        if item is None or self.clock()-item[0] > 120:
            raise BridgeError(409, 'Capture expired or unknown; capture and detect again')
        return item

    def project(self, capture_id, pixel):
        _, meta, k, depth = self.get_capture(capture_id)
        if (not isinstance(pixel, list) or len(pixel) != 2
                or any(type(v) is not int for v in pixel)):
            raise BridgeError(422, 'Pixel must be integer [x, y]')
        x, y = pixel
        if not (0 <= x < meta['width'] and 0 <= y < meta['height']):
            raise BridgeError(422, 'Pixel outside original image')
        size, code, scale = (2, 'H', .001) if depth.encoding == '16UC1' else (4, 'f', 1.)
        z = struct.unpack_from(('>' if depth.is_bigendian else '<')+code,
                               depth.data, y*depth.step+x*size)[0]*scale
        if not math.isfinite(z) or z <= 0:
            raise BridgeError(422, 'Selected pixel has no valid measured depth')
        point = [(x-k[2])*z/k[0], (y-k[5])*z/k[4], z]
        return dict(frame_id=self.config.world_frame,
                    position=transform_point(point, meta['camera_pose']),
                    capture_id=capture_id, pixel=pixel, stamp_ns=meta['stamp_ns'])

    def create(self, capture_id, targets):
        meta = self.get_capture(capture_id)[1]
        camera = next(c for c in self.config.cameras if c.id == meta['camera_id'])
        if camera.mount != 'world':
            raise BridgeError(422, 'Initial detection requires a fixed camera')
        converted = [dict(arm_id=t['arm_id'], grasp=self.project(capture_id, t['grasp']),
                          release=self.project(capture_id, t['release'])) for t in targets]
        plan_id = uuid.uuid4().hex
        self.plans[plan_id] = dict(plan_id=plan_id, targets=converted, state='detected', created=self.clock())
        while len(self.plans) > 32:
            self.plans.popitem(last=False)
        return self.public(self.plans[plan_id])

    def get(self, plan_id):
        plan = self.plans.get(plan_id)
        if plan is None or plan['state'] in ('invalid', 'placed'):
            raise BridgeError(409, 'Plan unavailable; do not repeat a completed or interrupted motion')
        if plan['state'] in ('detected', 'approached') and self.clock()-plan['created'] > 120:
            raise BridgeError(409, 'Plan expired; detect again')
        return plan

    def refine(self, plan_id, arm_id, capture_id, pixel):
        plan = self.get(plan_id)
        if plan['state'] != 'approached':
            raise BridgeError(409, 'Approach before wrist refinement')
        meta = self.get_capture(capture_id)[1]
        camera = next(c for c in self.config.cameras if c.id == meta['camera_id'])
        if camera.mount != 'flange' or camera.arm_id != arm_id or meta['stamp_ns'] <= plan['motion_stamp_ns']:
            raise BridgeError(422, 'Use a fresh capture from this arm wrist camera after approach')
        target = next((t for t in plan['targets'] if t['arm_id'] == arm_id), None)
        if target is None:
            raise BridgeError(422, 'Arm is not in this plan')
        target['grasp'] = self.project(capture_id, pixel)
        return self.public(plan)

    def verify(self, plan_id, observations):
        plan = self.get(plan_id)
        if plan['state'] != 'picked':
            raise BridgeError(409, 'Pick before verifying grasp')
        if {o['arm_id'] for o in observations} != {t['arm_id'] for t in plan['targets']}:
            raise BridgeError(422, 'Verify every arm in the coordinated grasp')
        for observation in observations:
            meta = self.get_capture(observation['capture_id'])[1]
            camera = next(c for c in self.config.cameras if c.id == meta['camera_id'])
            if (camera.mount != 'world' and camera.arm_id != observation['arm_id']) or meta['stamp_ns'] <= plan['motion_stamp_ns']:
                raise BridgeError(422, 'Verification requires a new fixed-camera or matching wrist image after pick completion')
        if not all(o['success'] for o in observations):
            return dict(success=False, error='Grasp not visually confirmed; inspect or stop', plan_id=plan_id)
        plan['state'] = 'verified'
        return self.public(plan)

    def invalidate(self):
        self.captures.clear()
        for plan in self.plans.values():
            if plan['state'] != 'placed':
                plan['state'] = 'invalid'

    def public(self, plan):
        next_actions = {
            'detected': ['approach_targets', 'pick_targets'],
            'approached': ['refine_grasp', 'pick_targets'],
            'picked': ['inspect_grasp', 'verify_grasp', 'recover_arms'],
            'verified': ['place_targets'],
            'placed': ['get_robot_state', 'detect_targets', 'finish_task'],
        }
        available = next_actions.get(plan['state'], ['recover_arms'])
        if not any(c.mount == 'flange' and c.arm_id in {t['arm_id'] for t in plan['targets']}
                   for c in self.config.cameras):
            available = [a for a in available if a not in {'approach_targets', 'refine_grasp'}]
        return dict(success=True, next_actions=available,
                    **copy.deepcopy({k:v for k,v in plan.items() if k not in ('created',)}))
