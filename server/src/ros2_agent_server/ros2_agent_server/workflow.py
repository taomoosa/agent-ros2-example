"""Strict pixel workflow request bodies shared by HTTP and ROS validation."""

from typing import Literal
from pydantic import Field
from .models import Model, MoveArm


class Target(Model):
    arm_id: str
    grasp: list[int] = Field(min_length=2, max_length=2)
    release: list[int] = Field(min_length=2, max_length=2)


class DetectPlan(Model):
    capture_id: str
    targets: list[Target] = Field(min_length=1, max_length=2)


class RefinePlan(Model):
    plan_id: str
    arm_id: str
    capture_id: str
    pixel: list[int] = Field(min_length=2, max_length=2)


class ExecutePlan(Model):
    plan_id: str
    stage: Literal['approach', 'pick', 'place']


class Observation(Model):
    arm_id: str
    capture_id: str
    success: bool


class VerifyPlan(Model):
    plan_id: str
    observations: list[Observation] = Field(min_length=1, max_length=2)


class ArmMove(MoveArm):
    arm_id: str


class MoveArms(Model):
    moves: list[ArmMove] = Field(min_length=1, max_length=2)


# TOOL EXTENSION: register strict workflow bodies here and classify execution in robot_node.py.
BODIES = {'create_plan': DetectPlan, 'refine_plan': RefinePlan,
          'execute_plan': ExecutePlan, 'verify_grasp': VerifyPlan, 'move_arms': MoveArms}


def validate_workflow(config, operation, resource_id, payload):
    if resource_id:
        raise ValueError('Workflow operations use payload fields, not resource_id')
    if operation in {'reset_arms', 'recover_arms'}:
        if payload:
            raise ValueError(f'{operation} takes no arguments')
        return {}
    body = BODIES[operation].model_validate(payload).model_dump()
    items = body.get('targets', body.get('observations', body.get('moves', [])))
    arm_ids = [item['arm_id'] for item in items]
    if 'arm_id' in body:
        arm_ids.append(body['arm_id'])
    if len(arm_ids) != len(set(arm_ids)):
        raise ValueError(f'Duplicate arm IDs: {arm_ids}')
    unknown = set(arm_ids) - {a.id for a in config.arms}
    if unknown:
        raise ValueError(f'Unconfigured arm IDs: {sorted(unknown)}')
    for move in body.get('moves', []):
        if move['frame_id'] not in config.frame_ids:
            raise ValueError(f"Unknown motion frame for {move['arm_id']}: {move['frame_id']!r}; expected {config.frame_ids}")
    return body
