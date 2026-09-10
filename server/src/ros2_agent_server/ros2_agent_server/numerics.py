"""Numerical acceptance policies; identities and acquisition ordering stay exact."""

import math
import struct

from .diagnostics import require
from .protocol import BridgeError


def rectified_intrinsics(info, camera):
    """Use rectified P by default; raw lens D is metadata, not a zero residual."""
    tolerance = camera.rectification_tolerance
    require(len(info.k) == 9, "camera_info.k.length", 9, len(info.k))
    for i, value in enumerate(info.k):
        require(math.isfinite(value), f"camera_info.k[{i}]", "finite", value)
    if camera.camera_info_mode == 'ros_rectified':
        p, r = list(getattr(info, "p", [])), list(getattr(info, "r", []))
        require(len(p) == 12 and all(math.isfinite(v) for v in p) and p[0] > 0 and p[5] > 0,
                "camera_info.p", "calibrated rectified P with positive fx/fy; use rectified_k only for normalized K-only adapters", p)
        require(len(r) == 9 and all(math.isfinite(v) and abs(v-(1. if i in (0,4,8) else 0.)) <= tolerance
                                   for i,v in enumerate(r)), "camera_info.r", "identity rectification rotation", r)
        require(all(abs(p[i]) <= tolerance for i in (3,7,11)), "camera_info.p.translation",
                "zero monocular projection translation", [p[i] for i in (3,7,11)])
        require(all(math.isfinite(v) for v in info.d), "camera_info.d", "finite raw distortion coefficients", list(info.d))
        k = [p[i] for i in (0,1,2,4,5,6,8,9,10)]
    else:
        require(all(math.isfinite(v) and abs(v) <= tolerance for v in info.d),
                "camera_info.d", f"finite rectified residuals within {tolerance}; for original lens D with rectified images, use camera_info_mode=ros_rectified and calibrated P", list(info.d))
        k = list(info.k)
        r = list(getattr(info, 'r', []))
        if any(r):
            require(len(r) == 9 and all(math.isfinite(v) and abs(v-(1. if i in (0,4,8) else 0.)) <= tolerance
                                       for i,v in enumerate(r)), "camera_info.r", "identity or unset rectification rotation", r)
        p = list(getattr(info, 'p', []))
        if any(p):
            expected = [k[0],k[1],k[2],0.,k[3],k[4],k[5],0.,k[6],k[7],k[8],0.]
            require(len(p) == 12 and all(math.isfinite(v) and abs(v-w) <=
                    (camera.calibration_tolerance_px if i in (0,2,5,6) else tolerance)
                    for i,(v,w) in enumerate(zip(p,expected))), "camera_info.p",
                    "P consistent with rectified K, or select camera_info_mode=ros_rectified", p)
    require(len(k) == 9, "camera_info.k.length", 9, len(k))
    for i, value in enumerate(k):
        require(math.isfinite(value), f"camera_info.k[{i}]", "finite", value)
    for i in (0, 4):
        require(k[i] > 0, f"camera_info.k[{i}]", "positive focal length", k[i])
    for i, value in ((1, 0.), (3, 0.), (6, 0.), (7, 0.), (8, 1.)):
        require(abs(k[i]-value) <= tolerance, f"camera_info.k[{i}]",
                f"{value} +/- {tolerance}", k[i])
        k[i] = value
    return k


def calibration_key(info):
    """Normalize equivalent binning/full-image ROI representations."""
    roi = info.roi
    return ((info.header.frame_id, info.width, info.height,
             max(1, info.binning_x), max(1, info.binning_y),
             roi.x_offset, roi.y_offset, roi.do_rectify,
             roi.width or info.width, roi.height or info.height),
            tuple(info.k), tuple(info.d), tuple(info.p), tuple(info.r), info.distortion_model)


def same_calibration(previous, current, camera):
    if previous[0] != current[0] or len(previous[1]) != len(current[1]):
        return False
    for i, (a, b) in enumerate(zip(previous[1], current[1])):
        tolerance = camera.calibration_tolerance_px if i in (0, 2, 4, 5) else camera.rectification_tolerance
        if not math.isfinite(a) or not math.isfinite(b) or abs(a-b) > tolerance:
            return False
        if i in (0, 4) and (a > 0) != (b > 0):
            return False
        if i not in (0, 2, 4, 5):
            target = 1. if i == 8 else 0.
            # Do not hide an invalid residual through a tolerant change comparison.
            if (abs(a-target) <= tolerance) != (abs(b-target) <= tolerance):
                return False
    def distortion(values):
        if all(math.isfinite(v) and abs(v) <= camera.rectification_tolerance for v in values):
            return ()
        return values
    if previous[5] != current[5]:
        return False
    for index in (3,4):
        a, b = previous[index], current[index]
        if len(a) != len(b):
            return False
        for i,(x,y) in enumerate(zip(a,b)):
            tolerance = camera.calibration_tolerance_px if index == 3 and i in (0,2,5,6) else camera.rectification_tolerance
            if not math.isfinite(x) or not math.isfinite(y) or abs(x-y) > tolerance:
                return False
            if index == 3 and i in (0,5) and (x > 0) != (y > 0):
                return False
            if index == 4 or i not in (0,2,5,6):
                target = 1. if (index == 4 and i in (0,4,8)) or (index == 3 and i == 10) else 0.
                if (abs(x-target) <= tolerance) != (abs(y-target) <= tolerance):
                    return False
    a, b = distortion(previous[2]), distortion(current[2])
    return len(a) == len(b) and all(math.isfinite(x) and math.isfinite(y)
        and abs(x-y) <= camera.rectification_tolerance for x, y in zip(a, b))


def measured_depth(depth, x, y, camera):
    """Reject unsupported surfaces; never average across a foreground boundary."""
    size, code, scale = (2, 'H', .001) if depth.encoding == '16UC1' else (4, 'f', 1.)
    def read(u, v):
        return struct.unpack_from(('>' if depth.is_bigendian else '<')+code,
                                  depth.data, v*depth.step+u*size)[0]*scale
    def rounding(value):
        # One float32 relative precision unit, or float64 mm-to-m roundoff.
        return abs(value)*2**-23 if depth.encoding == '32FC1' else 1e-12
    def in_range(value):
        return (math.isfinite(value) and value > 0
                and camera.depth_min_m-rounding(value) <= value <= camera.depth_max_m+rounding(value))
    z = read(x, y)
    if not in_range(z):
        raise BridgeError(422, f"depth.range_m: pixel={[x,y]}; expected finite depth in "
                          f"[{camera.depth_min_m}, {camera.depth_max_m}]; actual {z}",
                          code="depth_quality_invalid")
    tolerance = camera.depth_absolute_tolerance_m + camera.depth_relative_tolerance*z
    samples = [read(u, v) for v in range(max(0, y-1), min(depth.height, y+2))
               for u in range(max(0, x-1), min(depth.width, x+2))]
    supporting = sum(in_range(value) and abs(value-z) <= tolerance+rounding(value)+rounding(z)
                     for value in samples)
    required = max(3, math.ceil(camera.depth_min_support*len(samples)))
    if supporting < required:
        raise BridgeError(422, f"depth.support: pixel={[x,y]}; depth_m={z}; tolerance_m={tolerance}; "
                          f"expected >= {required}/{len(samples)} supporting pixels; actual {supporting}",
                          code="depth_quality_invalid")
    return z
