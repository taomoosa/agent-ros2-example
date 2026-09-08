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


class Ros2Embodiment(base.Embodiment):
  def __init__(self, config: RobotConfig, *, transport=None):
    self.config = config
    self.audio_queue = asyncio.Queue()
    self.video_queue = asyncio.Queue(maxsize=1)
    self.text_queue = asyncio.Queue()
    self.robot = Ros2RobotClient(config, transport=transport)
    self.poller = Ros2CameraPoller(self.robot, self.video_queue)
    self.poller_task = None
    self.task_result = None
    self._action_lock = asyncio.Lock()

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

  async def execute_action(self, action_name: str, **kwargs):
    # The upstream event bus may dispatch distinct tool-call events concurrently.
    # Stop must remain available while a motion HTTP request is pending.
    if action_name == "stop":
      return await self._execute_action(action_name, **kwargs)
    async with self._action_lock:
      if self.task_result is not None:
        return {"success": False, "error": "The application has already finished"}
      return await self._execute_action(action_name, **kwargs)

  async def _execute_action(self, action_name: str, **kwargs):
    if action_name == "ack" and not kwargs:
      return {"message": "No action needed."}
    if action_name == "finish_task":
      if (set(kwargs) != {"success", "summary"} or type(kwargs["success"]) is not bool
          or not isinstance(kwargs["summary"], str) or not kwargs["summary"].strip()):
        raise ValueError("finish_task requires a boolean success and nonempty summary")
      self.task_result = dict(kwargs)
      return self.task_result
    actions = {
        "get_robot_state": self.robot.get_robot_state,
        "move_arm": self.robot.move_arm,
        "set_gripper": self.robot.set_gripper,
        "stop": self.robot.stop,
    }
    if action_name not in actions:
      raise ValueError(f"Unknown ROS2 action: {action_name}")
    try:
      return await actions[action_name](**kwargs)
    except (ValueError, TypeError) as exc:
      return {"error": str(exc), "success": False}
    except httpx.HTTPError as exc:
      return {"error": str(exc), "success": False,
              "outcome": "unknown", "retry_instruction":
              "Do not repeat motion automatically. Read robot state and stop if needed."}

  async def close(self):
    self.poller.stop()
    if self.poller_task is not None:
      self.poller_task.cancel()
      await asyncio.gather(self.poller_task, return_exceptions=True)
      self.poller_task = None
    await self.robot.close()
