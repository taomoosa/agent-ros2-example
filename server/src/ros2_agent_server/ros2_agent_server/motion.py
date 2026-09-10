"""Typed targets and deterministic tabletop phases shared by hardware adapters."""

from typing import Annotated, Literal
from pydantic import Field, model_validator
from .models import Model, Pose
from .protocol import BridgeError


class Selection(Model):
    arm_ids: list[str] | None = Field(default=None, min_length=1, max_length=2)
    all_arms: bool = False

    def selected(self, config):
        if (self.arm_ids is not None) == self.all_arms:
            raise ValueError('Specify exactly one of arm_ids or all_arms=true')
        ids = [a.id for a in config.arms] if self.all_arms else self.arm_ids
        if len(set(ids)) != len(ids):
            raise ValueError('arm_ids must not contain duplicates')
        unknown = set(ids)-{a.id for a in config.arms}
        if unknown:
            raise ValueError(f'Unknown arm_ids: {sorted(unknown)}')
        return ids


class PoseTarget(Pose):
    kind: Literal['pose'] = 'pose'
    reference: Literal['flange', 'tcp'] = 'flange'


class JointTarget(Model):
    kind: Literal['joints'] = 'joints'
    names: list[str] = Field(min_length=1)
    positions: list[float] = Field(min_length=1)

    @model_validator(mode='after')
    def joints(self):
        if len(self.names) != len(self.positions) or len(set(self.names)) != len(self.names) or any(not n.strip() for n in self.names):
            raise ValueError('Joint names must be nonempty, unique and match positions')
        return self


class PixelTarget(Model):
    kind: Literal['pixel']
    capture_id: str = Field(min_length=1)
    pixel: list[int] = Field(min_length=2, max_length=2)
    profile: Literal['tabletop']
    offset_m: float = Field(default=0., ge=0., le=1.)


class NamedTarget(Model):
    kind: Literal['named']
    name: str = Field(min_length=1)


Target = Annotated[PoseTarget | PixelTarget | NamedTarget, Field(discriminator='kind')]


class Move(Selection):
    targets: list[Target] = Field(min_length=1, max_length=2)
    duration: float = Field(default=3., ge=.1, le=60.)


class Grip(Selection):
    opening: float = Field(ge=0., le=1.)


class ToolProfile(Model):
    # HARDWARE INTEGRATION: task geometry only; TCP calibration belongs to the backend.
    orientation: list[float] = Field(min_length=4, max_length=4)
    approach_m: float = Field(gt=0., le=1.)
    lift_m: float = Field(gt=0., le=1.)
    transfer_height_m: float = Field(gt=0.)
    contact_offset_m: float = Field(default=0., ge=-1., le=1.)
    close_opening: float = Field(default=0., ge=0., le=1.)
    duration: float = Field(default=3., ge=.1, le=60.)

    @model_validator(mode='after')
    def orientation_valid(self):
        Pose(frame_id='world', position=[0.,0.,0.], orientation=self.orientation)
        return self


class Hardware(Model):
    profiles: dict[str, ToolProfile] = Field(default_factory=dict)
    # Name-description metadata only; the backend owns each actual position and its format.
    position_names: dict[str, dict[str, str]] = Field(default_factory=dict)

    @model_validator(mode='after')
    def names_valid(self):
        for arm, names in self.position_names.items():
            if any(not name.strip() or not description.strip() for name, description in names.items()):
                raise ValueError(f'position_names.{arm} must map nonempty names to nonempty descriptions')
        return self


class MotionCompiler:
    def __init__(self, config, pixels):
        self.config, self.pixels = config, pixels
        self.hardware = Hardware.model_validate(config.server.hardware)
        unknown = (set(self.hardware.profiles) | set(self.hardware.position_names))-{a.id for a in config.arms}
        if unknown:
            raise ValueError(f'hardware contains unknown arms: {sorted(unknown)}')

    def profile(self, arm):
        if arm not in self.hardware.profiles:
            raise BridgeError(422, f'hardware.profiles.{arm}: tabletop geometry is not configured')
        return self.hardware.profiles[arm]

    def contact(self, arm, point, extra=0.):
        if point['frame_id'] != self.config.world_frame:
            raise BridgeError(422, 'Tabletop points must use world_frame with vertical +Z clearance')
        profile = self.profile(arm)
        pose = dict(kind='pose', reference='tcp', frame_id=point['frame_id'],
                    position=list(point['position']), orientation=profile.orientation)
        pose['position'][2] += profile.contact_offset_m+extra
        return pose

    def resolve(self, arm, target):
        if target['kind'] == 'named':
            names = self.hardware.position_names.get(arm, {})
            if target['name'] not in names:
                raise BridgeError(422, f"Unknown named position for {arm}: {target['name']}")
            return target
        if target['kind'] == 'pixel':
            meta = self.pixels.get_capture(target['capture_id'])[1]
            camera = next(c for c in self.config.cameras if c.id == meta['camera_id'])
            if camera.arm_id is not None and camera.arm_id != arm:
                raise BridgeError(422, 'Pixel target must use the selected arm wrist or a fixed camera')
            return self.contact(arm, self.pixels.project(target['capture_id'], target['pixel']), target['offset_m'])
        if target['frame_id'] not in self.config.frame_ids:
            raise BridgeError(422, f"Unknown target frame: {target['frame_id']}")
        return target

    def move(self, payload):
        body = Move.model_validate(payload)
        ids = body.selected(self.config)
        if len(body.targets) not in (1, len(ids)):
            raise BridgeError(422, 'targets must contain one shared target or one target per arm_id')
        targets = body.targets*len(ids) if len(body.targets) == 1 else body.targets
        goals = [self.resolve(arm, target.model_dump()) for arm,target in zip(ids,targets)]
        return ids, [dict(operation='move', arm_ids=ids, targets=goals, duration=body.duration)]

    def compile(self, operation, resource, payload):
        if operation == 'move':
            return self.move(payload)
        if operation == 'gripper':
            ids = Grip.model_validate(payload).selected(self.config)
            return ids, [dict(operation='gripper',arm_ids=ids,opening=payload['opening'])]
        if operation == 'recover_arms':
            ids = payload['arm_ids']
            return ids, [dict(operation='recover',arm_ids=ids)]
        ids = [t['arm_id'] for t in payload['targets']]
        points = {t['arm_id']:t for t in payload['targets']}
        stage = payload['stage']
        steps = []
        for phase in payload['phases']:
            if phase in {'open','close'}:
                # Per-arm calibrated closing targets; the adapter executes a group barrier.
                steps.append(dict(operation='gripper', arm_ids=ids,
                    openings=[1. if phase == 'open' else self.profile(a).close_opening for a in ids], phase=phase))
                continue
            goals = []
            for arm in ids:
                profile = self.profile(arm)
                point = points[arm]['release' if stage == 'place' else 'grasp']
                extra = profile.approach_m if phase in {'approach','retreat'} else profile.lift_m if phase == 'lift' else 0.
                if phase == 'transfer':
                    extra = profile.transfer_height_m-point['position'][2]-profile.contact_offset_m
                    if extra < profile.approach_m:
                        raise BridgeError(422, f'Transfer height does not clear release point for {arm}')
                goals.append(self.contact(arm,point,extra))
            steps.append(dict(operation='move',arm_ids=ids,targets=goals,
                              duration=max(self.profile(a).duration for a in ids),phase=phase))
        return ids, steps
