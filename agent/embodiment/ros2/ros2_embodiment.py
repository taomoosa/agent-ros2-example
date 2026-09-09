"""ROS2 counterpart of the robotics-samples Spot embodiment."""

import asyncio
import dataclasses
import json
from pathlib import Path

import httpx

from embodiment import base
from embodiment.ros2.camera_poller import Ros2CameraPoller
from embodiment.ros2.config import RobotConfig
from embodiment.ros2.robot_client import Ros2RobotClient
from embodiment.ros2.tools import ros2_tools
from embodiment.ros2.robotics_er import RoboticsER, DEFAULT_ROBOTICS_MODEL
from embodiment.ros2.manipulation import Manipulation


# TOOL EXTENSION: classify motion here as well as dispatch; preserve stop/recovery guards.
MOTION_TOOLS = frozenset({'move_arm', 'set_gripper', 'move_arms', 'reset_arms',
                          'recover_arms', 'approach_targets', 'pick_targets', 'place_targets'})


class Ros2Embodiment(base.Embodiment):
  def __init__(self, config: RobotConfig, *, transport=None, api_key="",
               robotics_model=DEFAULT_ROBOTICS_MODEL, robotics_transport=None,
               er_prompt_files=None, max_recovery_attempts=2):
    if type(max_recovery_attempts) is not int or not 0 <= max_recovery_attempts <= 10:
      raise ValueError('max_recovery_attempts must be an integer in 0..10')
    self.config = config
    self.audio_queue = asyncio.Queue()
    self.video_queue = asyncio.Queue(maxsize=1)
    self.text_queue = asyncio.Queue()
    self.reasoning = RoboticsER(api_key, model=robotics_model, transport=robotics_transport,
                               prompt_files=er_prompt_files, timeout=config.timing.er_timeout)
    self.robot = Ros2RobotClient(config, transport=transport)
    self.manipulation = Manipulation(config, self.robot, self.reasoning,
                                    max_recovery_attempts=max_recovery_attempts)
    self.poller = Ros2CameraPoller(self.robot, self.video_queue)
    self.poller_task = None
    self.task_result = None
    self._action_lock = asyncio.Lock()
    self._stop_generation = 0
    self._motion_active = False
    self.session_lost = False
    self.session_lost_event = asyncio.Event()
    self.observation_timeout = config.timing.observation_timeout
    self.observation_revision = 0
    self.scene_revision = None

  async def initialize(self):
    if self.poller_task is not None:
      return
    await self.robot.get_robot_state()
    if not await self.poller.wait_for_next_frame():
      raise RuntimeError("All configured cameras must be available before starting")
    self.poller_task = asyncio.create_task(self.poller.run())

  def get_audio_queue(self):
    return self.audio_queue

  def get_video_queue(self):
    return self.video_queue

  def get_text_queue(self):
    return self.text_queue

  def get_tools(self):
    return ros2_tools(self.config)

  def get_system_instruction(self):
    prompt = (Path(__file__).parent / "instruction.md").read_text()
    topology = dataclasses.asdict(self.config)
    topology.pop("robot_url")
    return prompt + "\nRobot topology:\n" + json.dumps(topology, ensure_ascii=False)

  def interrupt(self, *, session_lost=False):
    self._stop_generation += 1
    self.session_lost |= session_lost
    if session_lost:
      self.session_lost_event.set()
    self.manipulation.needs_recovery |= self._motion_active or session_lost or any(
        p['state'] in {'picked', 'verified'} for p in self.manipulation.plans.values())
    self.manipulation.invalidate()

  async def execute_action(self, action_name: str, **kwargs):
    # Calls queued before a stop cannot run after it releases the action lock.
    generation = self._stop_generation
    if action_name == 'stop':
      self.interrupt()
      return await self._execute_action(action_name, **kwargs)
    async with self._action_lock:
      if self.task_result is not None or self.session_lost or generation != self._stop_generation:
        return {'success': False, 'error': 'Application finished or operation interrupted'}
      self._motion_active = action_name in MOTION_TOOLS
      if self._motion_active:
        self.observation_revision += 1
      try:
        result = await self._execute_action(action_name, **kwargs)
        if self._motion_active and (result.get('outcome') == 'unknown' or result.get('http_status', 0) >= 500):
          # An uncertain motion is stopped before waiting for another model turn.
          self.manipulation.needs_recovery = True
          try:
            result['stop_result'] = await self.robot.stop()
            result['operator_required'] = result['stop_result'].get('success') is not True
          except httpx.HTTPError as exc:
            result['stop_result'] = {'success': False, 'error': str(exc)}
            result['operator_required'] = True
        if generation != self._stop_generation:
          self.manipulation.invalidate()
          if action_name == 'finish_task':
            self.task_result = None
          return {'success': False, 'error': 'Operation interrupted; old result discarded',
                  'outcome': 'unknown' if self._motion_active else 'interrupted',
                  'next_actions': ['get_robot_state', 'recover_arms', 'finish_task']}
        return result
      except (ValueError, TypeError) as exc:
        return {'success': False, 'error': str(exc)}
      except asyncio.CancelledError:
        if self._motion_active:
          self.manipulation.needs_recovery = True
          self.manipulation.invalidate()
        raise
      finally:
        self._motion_active = False

  async def _execute_action(self, action_name: str, **kwargs):
    if action_name == "ack" and not kwargs:
      return {"message": "No action taken. ack does not schedule another model turn.",
              "next_actions": ["get_robot_state", "inspect_grasp", "finish_task"]}
    if action_name == "finish_task":
      if (set(kwargs) != {"success", "summary"} or type(kwargs["success"]) is not bool
          or not isinstance(kwargs["summary"], str) or not kwargs["summary"].strip()):
        raise ValueError("finish_task requires a boolean success and nonempty summary")
      if kwargs['success'] and (self.manipulation.needs_recovery or any(
          p['state'] in {'picked', 'verified', 'invalid'} for p in self.manipulation.plans.values())):
        return {'success': False, 'error': 'Cannot finish successfully with held objects or unresolved failures',
                'next_actions': ['get_robot_state', 'recover_arms', 'finish_task']}
      if kwargs['success'] and (self.manipulation.final_observation_required or
                                self.scene_revision != self.observation_revision):
        return {'success': False, 'error': 'Observe current state and scene with get_robot_state before finishing',
                'next_actions': ['get_robot_state', 'finish_task']}
      if kwargs['success']:
        try:
          state = await self.robot.get_robot_state()
        except (httpx.HTTPError, ValueError) as exc:
          return {'success': False, 'error': f'Final state unavailable: {exc}',
                  'next_actions': ['get_robot_state', 'finish_task']}
        if (state.get('success') is False or state.get('recovery_required') or state.get('motion_outcome_unknown') or any(
            a.get('moving') is True or a.get('fault') or a.get('gripper', {}).get('fault') or
            a.get('gripper', {}).get('object_detected') is True for a in state.get('arms', []))):
          return {'success': False, 'error': 'Final state reports motion, a fault, or a held object',
                  'next_actions': ['get_robot_state', 'recover_arms', 'finish_task']}
      self.task_result = dict(kwargs)
      return self.task_result
    actions = {
        "get_robot_state": self.robot.get_robot_state,
        "move_arm": self.robot.move_arm,
        "set_gripper": self.robot.set_gripper,
        "stop": self.robot.stop,
    }
    actions.update({name: getattr(self.manipulation, name) for name in (
        'reset_arms', 'recover_arms', 'detect_targets', 'approach_targets', 'refine_grasp',
        'pick_targets', 'inspect_grasp', 'verify_grasp', 'place_targets', 'move_arms')})
    if action_name not in actions:
      raise ValueError(f"Unknown ROS2 action: {action_name}")
    try:
      if action_name in {'move_arm', 'set_gripper'} and self.manipulation.needs_recovery:
        raise ValueError('Recover arms before issuing another manual motion')
      if action_name in {'stop', 'move_arm', 'set_gripper'}:
        if any(p['state'] in {'picked', 'verified'} for p in self.manipulation.plans.values()):
          self.manipulation.needs_recovery = True
          self.manipulation.invalidate()
          if action_name != 'stop':
            raise ValueError('Manual motion cannot discard a held-object plan; stop and recover')
        self.manipulation.invalidate()
      observation_revision = self.observation_revision
      result = await actions[action_name](**kwargs)
      if action_name == 'get_robot_state':
        result['observation_revision'] = observation_revision
        self.manipulation.needs_recovery |= (result.get('recovery_required') is True or
                                            result.get('motion_outcome_unknown') is True or any(
            a.get('fault') or a.get('gripper', {}).get('fault') for a in result.get('arms', [])))
        result['manipulation'] = dict(
            plans=[dict(plan_id=k, state=v['state']) for k, v in self.manipulation.plans.items()],
            recovery_required=self.manipulation.needs_recovery,
            recovery_attempts_remaining=self.manipulation.max_recovery_attempts-self.manipulation.recovery_attempts)
      if action_name in MOTION_TOOLS | {'stop'} and result.get('success') is not True:
        self.manipulation.needs_recovery = True
      return result
    except (ValueError, TypeError) as exc:
      return {"error": str(exc), "success": False}
    except httpx.HTTPStatusError as exc:
      try:
        detail = exc.response.json()
        if not isinstance(detail, dict):
          detail = {}
      except ValueError:
        detail = {}
      result = dict(detail, success=False, http_status=exc.response.status_code)
      result.setdefault('error', str(exc))
      if action_name in MOTION_TOOLS | {'stop', 'verify_grasp'}:
        self.manipulation.needs_recovery = True
        result['next_actions'] = (['stop', 'finish_task'] if result.get('recoverable') is False
                                  else ['get_robot_state', 'recover_arms', 'finish_task'])
      return result
    except httpx.HTTPError as exc:
      if action_name in MOTION_TOOLS | {'stop', 'verify_grasp'}:
        self.manipulation.needs_recovery = True
      if action_name in {'get_robot_state', 'detect_targets', 'refine_grasp', 'inspect_grasp'}:
        return {'error': str(exc), 'success': False, 'motion_started': False,
                'retry_instruction': 'Read-only operation; retry with fresh imagery/state when available.'}
      return {"error": str(exc), "success": False,
              "outcome": "unknown", "retry_instruction":
              "Do not repeat motion automatically. Read state, stop all arms and recover before a new attempt."}

  async def close(self):
    self.poller.stop()
    if self.poller_task is not None:
      self.poller_task.cancel()
      await asyncio.gather(self.poller_task, return_exceptions=True)
      self.poller_task = None
    await self.robot.close()
    await self.reasoning.close()
