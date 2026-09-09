"""Capture-bound depth/plane projection and manipulation plans, independent of ROS transport."""

import base64
from collections import OrderedDict
import copy
import io
import math
import time
import uuid

from PIL import Image

from .protocol import BridgeError
from .diagnostics import require
from .numerics import rectified_intrinsics, measured_depth


def transform_point(point, pose):
    x, y, z, w = pose['orientation']
    for name, values in (("point", point), ("pose.position", pose['position']), ("pose.orientation", [x,y,z,w])):
        for i, value in enumerate(values):
            require(math.isfinite(value), f"{name}[{i}]", "finite", value)
    if not math.isclose(x*x+y*y+z*z+w*w, 1, abs_tol=1e-3):
        raise ValueError('Invalid transform quaternion')
    # Accepted quaternion rounding must not introduce scaling into the rotation.
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    x, y, z, w = (v/norm for v in (x, y, z, w))
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
        # HARDWARE INTEGRATION: aliases declare color-grid/color-Z geometry.
        # Raw depth requires upstream registration; see server/docs/synchronization.md.
        camera = next(c for c in self.config.cameras if c.id == camera_id)
        stamp = depth.header.stamp.sec * 1_000_000_000 + depth.header.stamp.nanosec
        require(abs(stamp-frame.stamp_ns) <= round(camera.sync_tolerance_sec*1e9),
                "depth.stamp_delta_ns", f"<= {round(camera.sync_tolerance_sec*1e9)}", abs(stamp-frame.stamp_ns))
        require(depth.header.frame_id == (camera.depth_frame or frame.frame_id),
                "depth.frame_id", camera.depth_frame or frame.frame_id, depth.header.frame_id)
        require(info.header.frame_id == (camera.camera_info_frame or frame.frame_id),
                "camera_info.frame_id", camera.camera_info_frame or frame.frame_id, info.header.frame_id)
        for name, value in (("depth", depth), ("camera_info", info)):
            require((value.width, value.height) == (width, height), name + ".dimensions",
                    (width, height), (value.width, value.height))
        for name in ("binning_x", "binning_y"):
            require(getattr(info, name) <= 1, "camera_info." + name, "0 or 1", getattr(info, name))
        for name in ("do_rectify", "x_offset", "y_offset"):
            require(not getattr(info.roi, name, 0), "camera_info.roi." + name, 0, getattr(info.roi, name, 0))
        require(getattr(info.roi, "width", 0) in (0, width) and getattr(info.roi, "height", 0) in (0, height),
                "camera_info.roi.dimensions", "full image or zero ROI", [getattr(info.roi, "width", 0), getattr(info.roi, "height", 0)])
        require(depth.encoding in ('16UC1', '32FC1'), "depth.encoding", "16UC1 (mm) or 32FC1 (m)", depth.encoding)
        size = 2 if depth.encoding == '16UC1' else 4
        require(depth.step >= width*size, "depth.step", f">= {width*size}", depth.step)
        require(len(depth.data) == depth.step*height, "depth.data_length", depth.step*height, len(depth.data))
        k = rectified_intrinsics(info, camera)
        if camera.mount == 'flange' and flange_pose is None:
            raise ValueError('Capture-time flange transform is required')
        require(camera_pose.get('frame_id') == self.config.world_frame, "camera_pose.frame_id",
                self.config.world_frame, camera_pose.get('frame_id'))
        transform_point([0., 0., 0.], camera_pose)
        if flange_pose is not None:
            require(flange_pose.get('frame_id') == self.config.world_frame, "flange_pose.frame_id",
                    self.config.world_frame, flange_pose.get('frame_id'))
            transform_point([0., 0., 0.], flange_pose)
        metadata = dict(capture_id=uuid.uuid4().hex, camera_id=camera_id,
                        width=width, height=height, stamp_ns=frame.stamp_ns,
                        frame_id=frame.frame_id, camera_pose=copy.deepcopy(camera_pose),
                        flange_pose=copy.deepcopy(flange_pose),
                        image_base64=base64.b64encode(frame.data).decode(), kind="rgbd",
                        depth_stamp_ns=stamp, depth_frame_id=depth.header.frame_id,
                        camera_info_frame_id=info.header.frame_id,
                        camera_info_stamp_ns=info.header.stamp.sec*1_000_000_000+info.header.stamp.nanosec,
                        sync_delta_ns=stamp-frame.stamp_ns, pose_stamp_ns=frame.stamp_ns)
        self.captures[metadata['capture_id']] = (self.clock(), metadata, k, copy.deepcopy(depth))
        while len(self.captures) > 64:
            self.captures.popitem(last=False)
        return copy.deepcopy(metadata)

    def capture_plane(self, camera_id, frame):
        # HARDWARE INTEGRATION: calibrate offline; see server/docs/plane-projection.md.
        camera = next(c for c in self.config.cameras if c.id == camera_id)
        require(camera.projection == "plane" and camera.mount == "world",
                "camera.projection", "plane on a fixed world camera", camera.projection)
        calibration = camera.plane_calibration.model_dump()
        with Image.open(io.BytesIO(frame.data)) as image:
            width, height = image.size
        require((width, height) == (calibration['image_width'], calibration['image_height']),
                "plane.image_dimensions", (calibration['image_width'], calibration['image_height']), (width, height))
        require(frame.frame_id == camera.optical_frame, "plane.frame_id", camera.optical_frame, frame.frame_id)
        metadata = dict(capture_id=uuid.uuid4().hex, camera_id=camera_id, kind="plane",
                        width=width, height=height, stamp_ns=frame.stamp_ns, frame_id=frame.frame_id,
                        image_base64=base64.b64encode(frame.data).decode(),
                        plane_calibration=copy.deepcopy(calibration))
        self.captures[metadata['capture_id']] = (self.clock(), metadata, None, None)
        while len(self.captures) > 64:
            self.captures.popitem(last=False)
        return copy.deepcopy(metadata)

    def observe(self, camera_id, frame):
        # TOOL EXTENSION: RGB evidence may verify a grasp, never project a point.
        with Image.open(io.BytesIO(frame.data)) as image:
            width, height = image.size
        metadata = dict(capture_id=uuid.uuid4().hex, camera_id=camera_id, kind="rgb",
                        width=width, height=height, stamp_ns=frame.stamp_ns,
                        frame_id=frame.frame_id, image_base64=base64.b64encode(frame.data).decode())
        self.captures[metadata['capture_id']] = (self.clock(), metadata, None, None)
        while len(self.captures) > 64:
            self.captures.popitem(last=False)
        return copy.deepcopy(metadata)

    def get_capture(self, capture_id):
        item = self.captures.get(capture_id)
        if item is None:
            raise BridgeError(409, f'Unknown capture_id: {capture_id}; acquire a new image')
        if self.clock()-item[0] > self.config.server.capture_ttl:
            raise BridgeError(409, f'Capture expired: id={capture_id}; age_sec={self.clock()-item[0]:.3f}; limit={self.config.server.capture_ttl}')
        return item

    def project(self, capture_id, pixel):
        _, meta, k, depth = self.get_capture(capture_id)
        if depth is None and meta['kind'] != 'plane':
            raise BridgeError(422, "capture.kind: RGB observation has no depth or plane calibration; request a metric capture")
        if (not isinstance(pixel, list) or len(pixel) != 2
                or any(type(v) is not int for v in pixel)):
            raise BridgeError(422, 'Pixel must be integer [x, y]')
        x, y = pixel
        if not (0 <= x < meta['width'] and 0 <= y < meta['height']):
            raise BridgeError(422, 'Pixel outside original image')
        if meta['kind'] == 'plane':
            calibration = meta['plane_calibration']
            x0, y0, x1, y1 = calibration['valid_region']
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                raise BridgeError(422, f"plane.valid_region: pixel {pixel} outside {calibration['valid_region']}")
            scale = max(abs(v) for v in calibration['homography'])
            a,b,c,d,e,f,g,h,i = [v/scale for v in calibration['homography']]
            denominator = g*x+h*y+i
            if abs(denominator) <= 1e-9:
                raise BridgeError(422, f"plane.homography denominator too small at pixel {pixel}")
            position = transform_point([(a*x+b*y+c)/denominator, (d*x+e*y+f)/denominator, 0.],
                                       calibration['plane_pose'])
            if not all(math.isfinite(v) for v in position):
                raise BridgeError(422, f"plane.position is non-finite at pixel {pixel}")
            return dict(frame_id=calibration['plane_pose']['frame_id'], position=position,
                        capture_id=capture_id, pixel=pixel, stamp_ns=meta['stamp_ns'],
                        projection='plane', calibration_id=calibration['calibration_id'])
        camera = next(c for c in self.config.cameras if c.id == meta['camera_id'])
        z = measured_depth(depth, x, y, camera)
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
        if plan['state'] in ('detected', 'approached') and self.clock()-plan['created'] > self.config.server.plan_ttl:
            raise BridgeError(409, 'Plan expired; detect again')
        return plan

    def refine(self, plan_id, arm_id, capture_id, pixel):
        plan = self.get(plan_id)
        if plan['state'] != 'approached':
            raise BridgeError(409, 'Approach before wrist refinement')
        meta = self.get_capture(capture_id)[1]
        camera = next(c for c in self.config.cameras if c.id == meta['camera_id'])
        require(camera.mount == 'flange', "refine.camera.mount", "flange", camera.mount)
        require(camera.arm_id == arm_id, "refine.camera.arm_id", arm_id, camera.arm_id)
        require(min(meta['stamp_ns'], meta.get('depth_stamp_ns', meta['stamp_ns'])) > plan['motion_stamp_ns'],
                "refine.stamp_ns", f"> {plan['motion_stamp_ns']}", meta['stamp_ns'])
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
            if camera.mount != 'world' and camera.arm_id != observation['arm_id']:
                raise BridgeError(422, f"Verification camera {camera.id} belongs to {camera.arm_id}, not {observation['arm_id']}")
            if meta['stamp_ns'] <= plan['motion_stamp_ns']:
                raise BridgeError(422, f"Verification stamp {meta['stamp_ns']} must be after pick {plan['motion_stamp_ns']}")
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
