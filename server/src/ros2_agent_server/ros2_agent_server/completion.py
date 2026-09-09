"""Optional driver-side completion monitor for measured poses in a common frame."""

import math

from .models import Model, Pose, ArmState
from pydantic import Field


class CompletionPolicy(Model):
    position_m: float = Field(default=.002, gt=0.)
    orientation_rad: float = Field(default=.02, gt=0., le=math.pi)
    gripper_opening: float = Field(default=.02, gt=0., le=1.)
    dwell_sec: float = Field(default=.2, ge=0.)
    max_sample_gap_sec: float = Field(default=.25, gt=0.)


class CompletionMonitor:
    # HARDWARE INTEGRATION: use per arm in the driver before acknowledging motion.
    # Supply measured states and acquisition times, never copied command values.
    def __init__(self, target, *, opening=None, policy=None):
        self.target = Pose.model_validate(target).model_dump()
        self.opening = opening
        if opening is not None and (type(opening) not in (int, float) or not math.isfinite(opening) or not 0 <= opening <= 1):
            raise ValueError('opening must be in [0, 1] or None')
        self.policy = policy or CompletionPolicy()
        self.since = self.previous = None

    def update(self, state, stamp_sec):
        state = ArmState.model_validate(state).model_dump()
        if not math.isfinite(stamp_sec) or (self.previous is not None and stamp_sec <= self.previous):
            self.since = None
            return False
        if self.previous is not None and stamp_sec-self.previous > self.policy.max_sample_gap_sec:
            self.since = None
        self.previous = stamp_sec
        pose = state['flange_pose']
        position_error = math.dist(pose['position'], self.target['position'])
        q, target = pose['orientation'], self.target['orientation']
        dot = abs(sum(a*b for a,b in zip(q,target)) / (math.hypot(*q)*math.hypot(*target)))
        angle = 2*math.acos(min(1.,dot))
        measured_opening = state['gripper']['opening']
        valid = (not state['moving'] and not state['fault'] and not state['gripper']['fault']
                 and pose['frame_id'] == self.target['frame_id']
                 and position_error <= self.policy.position_m and angle <= self.policy.orientation_rad
                 and (self.opening is None or measured_opening is not None
                      and abs(measured_opening-self.opening) <= self.policy.gripper_opening))
        if not valid:
            self.since = None
            return False
        if self.since is None:
            self.since = stamp_sec
        return stamp_sec-self.since >= self.policy.dwell_sec
