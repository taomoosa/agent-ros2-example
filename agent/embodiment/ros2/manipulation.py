"""Pixel detection, optional wrist refinement, and visually verified manipulation tools."""

import uuid

from embodiment.ros2.robotics_er import pixel


# TOOL EXTENSION: compose workflows and preserve plan/evidence generations here.
class Manipulation:
  def __init__(self, config, robot, reasoning, *, max_recovery_attempts=2):
    self.config, self.robot, self.reasoning = config, robot, reasoning
    self.plans = {}
    self.inspections = {}
    self.max_recovery_attempts = max_recovery_attempts
    self.recovery_attempts = 0
    self.needs_recovery = False
    self.generation = 0
    self.final_observation_required = False

  def checkpoint(self, generation):
    if generation != self.generation:
      raise ValueError('Operation interrupted; discard old results and observe again')

  def camera(self, camera_id, mount, arm_id=None):
    camera = next((c for c in self.config.cameras if c.id == camera_id), None)
    if camera is None or camera.mount != mount or camera.arm_id != arm_id:
      raise ValueError('Camera mount or arm does not match the requested operation')
    return camera

  def plan(self, plan_id):
    if plan_id not in self.plans:
      raise ValueError('Unknown plan; detect targets first')
    return self.plans[plan_id]

  def remember(self, result, generation=None):
    if generation is not None:
      self.checkpoint(generation)
    if result.get('success') is True:
      self.plans[result['plan_id']] = result
    return result

  def invalidate(self):
    self.generation += 1
    self.plans.clear()
    self.inspections.clear()

  async def reset_arms(self):
    if self.needs_recovery or any(p['state'] in {'picked', 'verified'} for p in self.plans.values()):
      raise ValueError('Use recover_arms after a failure or while an object may be held; reset is for startup')
    self.invalidate()
    return await self.robot.workflow('reset')

  async def recover_arms(self):
    if self.recovery_attempts >= self.max_recovery_attempts:
      return dict(success=False, error='Recovery attempt limit reached; stop and finish with failure',
                  next_actions=['stop', 'finish_task'])
    self.recovery_attempts += 1
    self.invalidate()
    self.needs_recovery = True
    generation = self.generation
    stopped = await self.robot.stop()
    self.checkpoint(generation)
    if stopped.get('success') is not True:
      return dict(stopped, success=False, next_actions=['stop', 'finish_task'])
    # The server validates current faults and the driver decides how to support
    # and release any payload. Never open a possibly loaded gripper blindly.
    result = await self.robot.workflow('recover')
    self.checkpoint(generation)
    if result.get('success') is True:
      self.needs_recovery = False
    return dict(result, recovery_attempts=self.recovery_attempts,
                recovery_attempts_remaining=self.max_recovery_attempts-self.recovery_attempts,
                next_actions=['get_robot_state', 'detect_targets'] if result.get('success') is True
                else ['get_robot_state', 'finish_task'])

  async def detect_targets(self, camera_id, instruction, arm_ids):
    if self.needs_recovery:
      raise ValueError('Recover all arms before detecting new targets')
    self.camera(camera_id, 'world')
    if (not isinstance(arm_ids, list) or not arm_ids or len(arm_ids) != len(set(arm_ids))
        or not set(arm_ids) <= {a.id for a in self.config.arms}):
      raise ValueError('Select one or two distinct configured arms')
    if not isinstance(instruction, str) or not instruction.strip():
      raise ValueError('A detection instruction is required')
    if any(p['state'] in {'picked', 'verified'} for p in self.plans.values()):
      raise ValueError('Place or recover the held-object plan before detecting again')
    if self.final_observation_required:
      raise ValueError('Observe the placement result with get_robot_state before detecting again')
    generation = self.generation
    capture = await self.robot.capture(camera_id)
    self.checkpoint(generation)
    result = await self.reasoning.reason('detect', capture, instruction, arm_ids)
    self.checkpoint(generation)
    targets = result.get('targets')
    if (set(result) != {'targets'} or not isinstance(targets, list) or len(targets) != len(arm_ids)
        or any(not isinstance(t, dict) or set(t) != {'arm_id', 'grasp', 'release'} for t in targets)
        or sorted(t['arm_id'] for t in targets) != sorted(arm_ids)):
      raise ValueError('Robotics ER must locate grasp and release points for every selected arm')
    converted = [dict(arm_id=t['arm_id'], grasp=pixel(t['grasp'], capture),
                      release=pixel(t['release'], capture)) for t in targets]
    return self.remember(await self.robot.workflow('create', capture_id=capture['capture_id'], targets=converted), generation)

  async def refine_grasp(self, plan_id, arm_id, camera_id, instruction):
    plan = self.plan(plan_id)
    if plan['state'] != 'approached' or arm_id not in {t['arm_id'] for t in plan['targets']}:
      raise ValueError('Approach this plan before refining its grasp')
    self.camera(camera_id, 'flange', arm_id)
    generation = self.generation
    capture = await self.robot.capture(camera_id)
    self.checkpoint(generation)
    result = await self.reasoning.reason('refine', capture, instruction, [arm_id])
    self.checkpoint(generation)
    if set(result) != {'point'}:
      raise ValueError('Invalid refinement response')
    return self.remember(await self.robot.workflow('refine', plan_id=plan_id, arm_id=arm_id,
        capture_id=capture['capture_id'], pixel=pixel(result['point'], capture)), generation)

  async def execute(self, plan_id, stage):
    plan = self.plan(plan_id)
    allowed = {'approach': {'detected'}, 'pick': {'detected', 'approached'}, 'place': {'verified'}}
    if self.needs_recovery or plan['state'] not in allowed[stage]:
      raise ValueError(f'Cannot {stage} a plan in state {plan["state"]}; inspect, verify or recover first')
    self.inspections.clear()
    generation = self.generation
    try:
      result = await self.robot.workflow('execute', plan_id=plan_id, stage=stage)
    except BaseException:
      plan['state'] = 'invalid'
      self.needs_recovery = True
      raise
    if result.get('success') is not True:
      plan['state'] = 'invalid'
      self.needs_recovery = True
      return dict(result, next_actions=['stop', 'finish_task'] if result.get('recoverable') is False
                  else ['get_robot_state', 'recover_arms', 'finish_task'])
    self.checkpoint(generation)
    if stage == 'place':
      self.final_observation_required = True
    return self.remember(result, generation)

  async def approach_targets(self, plan_id):
    return await self.execute(plan_id, 'approach')

  async def pick_targets(self, plan_id):
    return await self.execute(plan_id, 'pick')

  async def place_targets(self, plan_id):
    return await self.execute(plan_id, 'place')

  def inspection(self, observation_id):
    inspection = self.inspections.get(observation_id)
    if inspection is None or self.plan(inspection['plan_id'])['state'] != 'picked':
      raise ValueError('Unknown or invalidated grasp observation; inspect again')
    return inspection

  async def inspect_grasp(self, plan_id, arm_id, camera_id=None):
    plan = self.plan(plan_id)
    if plan['state'] != 'picked' or arm_id not in {t['arm_id'] for t in plan['targets']}:
      raise ValueError('Inspect a participating arm after pick completion')
    eligible = [c for c in self.config.cameras if c.mount == 'world' or c.arm_id == arm_id]
    if camera_id is not None:
      camera = next((c for c in eligible if c.id == camera_id), None)
    else:
      camera = next((c for c in eligible if c.mount == 'flange'), next(iter(eligible), None))
    if camera is None:
      raise ValueError('Use a configured fixed camera or this arm wrist camera')
    capture = await self.robot.observation(camera.id)
    if capture['stamp_ns'] <= plan['motion_stamp_ns']:
      raise ValueError('Inspection must use an image acquired after pick completion')
    state = await self.robot.get_robot_state()
    arm_state = next((a for a in state.get('arms', []) if a.get('id') == arm_id), None)
    if arm_state is None or arm_state.get('moving') is not False:
      raise ValueError('The inspected arm must have a current stationary state')
    if self.plan(plan_id) is not plan:
      raise ValueError('Plan changed during inspection')
    # A new image replaces previous evidence for this arm. At most one image
    # per configured arm is retained; raw image bytes never enter tool JSON.
    for key, value in list(self.inspections.items()):
      if value['arm_id'] == arm_id:
        del self.inspections[key]
    observation_id = uuid.uuid4().hex
    self.inspections[observation_id] = dict(plan_id=plan_id, arm_id=arm_id,
        capture=capture, delivered=False)
    return dict(success=True, plan_id=plan_id, observation_id=observation_id,
        arm_id=arm_id, camera_id=camera.id, capture_id=capture['capture_id'],
        stamp_ns=capture['stamp_ns'], width=capture['width'], height=capture['height'],
        arm_state=arm_state, next_action='Inspect every plan arm, then call verify_grasp with your assessment.',
        image_description='The video frame immediately preceding this response is this original inspection image. '
                          'Judge whether the requested object is held; gripper closure alone is not proof.')

  async def verify_grasp(self, plan_id, observations):
    plan = self.plan(plan_id)
    if plan['state'] != 'picked':
      raise ValueError('Pick before verifying the grasp')
    arm_ids = {t['arm_id'] for t in plan['targets']}
    if (not isinstance(observations, list) or len(observations) != len(arm_ids)
        or any(not isinstance(o, dict) or set(o) != {'arm_id', 'observation_id', 'success', 'reason'}
               for o in observations)):
      raise ValueError('Provide an assessment and reason for every plan arm')
    if sorted(o['arm_id'] for o in observations) != sorted(arm_ids):
      raise ValueError('Assess each participating arm exactly once')
    submitted = []
    for observation in observations:
      if (type(observation['success']) is not bool or not isinstance(observation['reason'], str)
          or not observation['reason'].strip() or len(observation['reason']) > 2000):
        raise ValueError('Each assessment requires boolean success and a nonempty reason up to 2000 characters')
      inspection = self.inspection(observation['observation_id'])
      if (inspection['plan_id'] != plan_id or inspection['arm_id'] != observation['arm_id']
          or not inspection['delivered']):
        raise ValueError('Use the matching camera observation delivered to the Live agent')
      submitted.append(dict(arm_id=observation['arm_id'], capture_id=inspection['capture']['capture_id'],
                            success=observation['success']))
    # Consume evidence even if the server rejects it or the connection fails.
    # Another assessment requires new images rather than changing the answer.
    for observation in observations:
      del self.inspections[observation['observation_id']]
    generation = self.generation
    result = await self.robot.workflow('verify', plan_id=plan_id, observations=submitted)
    self.checkpoint(generation)
    result['assessment'] = observations
    return self.remember(result)

  async def move_arms(self, moves):
    if self.needs_recovery:
      raise ValueError('Recover arms before issuing another manual motion')
    if any(p['state'] in {'picked', 'verified'} for p in self.plans.values()):
      self.needs_recovery = True
      self.invalidate()
      raise ValueError('Manual motion cannot discard a held-object plan; stop and recover')
    self.invalidate()
    return await self.robot.workflow('move_arms', moves=moves)
