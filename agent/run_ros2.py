"""Run a Gemini Live application against the ROS2 HTTP contract."""

import argparse
import asyncio
from contextlib import aclosing
import json
import logging
import os
from pathlib import Path

from embodiment.ros2.config import RobotConfig
from embodiment.ros2.ros2_embodiment import Ros2Embodiment
from session_manager import SessionManager


async def run_application(config: RobotConfig, instruction: str, *, model: str,
                          api_key: str, response_modality: str = "AUDIO",
                          timeout: float = 180.0, transport=None,
                          session_factory=SessionManager, on_event=None):
  if not instruction.strip():
    raise ValueError("An application instruction is required")
  if timeout <= 0:
    raise ValueError("timeout must be positive")
  embodiment = Ros2Embodiment(config, transport=transport)
  session = None

  async def consume():
    nonlocal session
    await embodiment.initialize()
    session = session_factory(
        model=model, api_key=api_key, embodiment_instance=embodiment,
        tools=embodiment.get_tools(),
        system_instruction=embodiment.get_system_instruction(),
        response_modality=response_modality, heartbeat_enabled=False,
    )
    await embodiment.get_text_queue().put(instruction)

    async def audio_output(_data):
      # CLI displays transcripts; it does not require an audio playback library.
      pass

    async with aclosing(session.start_session(audio_output_callback=audio_output)) as events:
      async for event in events:
        if on_event is not None:
          on_event(event)
        if event.get("type") == "error":
          raise RuntimeError(event.get("error"))
        # TOOL_RESULT is emitted only after the function response was sent.
        if event.get("type") == "tool_call" and event.get("name") == "finish_task":
          if embodiment.task_result is not None:
            if not embodiment.task_result["success"]:
              await embodiment.robot.stop()
            return embodiment.task_result
    raise RuntimeError("Gemini session ended without finish_task")

  try:
    return await asyncio.wait_for(consume(), timeout=timeout)
  except BaseException:
    try:
      await embodiment.robot.stop()
    except Exception as exc:
      logging.error("Could not stop arms after application interruption: %s", exc)
    raise
  finally:
    # Also clean up when setup fails before upstream's generator enters its
    # main loop/finally block.
    if session is not None:
      await session.observation.stop_input_tasks()
      session.observation.close()
      await session.bus.shutdown()
      if session.stream is not None:
        session.stream.Shutdown()
        session.stream = None
    await embodiment.close()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--config", required=True, help="Robot topology JSON file")
  parser.add_argument("--model", required=True, help="Gemini Live model available to your API key")
  parser.add_argument("--instruction", default="", help="Task and calibrated target poses")
  parser.add_argument("--task-file", type=Path, help="Application instructions in a UTF-8 file")
  parser.add_argument("--response-modality", choices=("AUDIO", "TEXT"), default="AUDIO")
  parser.add_argument("--timeout", type=float, default=180.0, help="Application time limit in seconds")
  args = parser.parse_args()
  api_key = os.environ.get("GEMINI_API_KEY", "")
  if not api_key:
    parser.error("Set GEMINI_API_KEY")
  instruction = "\n\n".join(part for part in (
      args.task_file.read_text() if args.task_file else "", args.instruction,
  ) if part)
  if not instruction.strip():
    parser.error("Provide --instruction or --task-file")
  logging.basicConfig(level=logging.WARNING)

  def print_event(event):
    if event.get("type") in {"gemini", "tool_call", "error"}:
      print(json.dumps(event, ensure_ascii=False), flush=True)

  try:
    result = asyncio.run(run_application(
        RobotConfig.load(args.config), instruction, model=args.model,
        api_key=api_key, response_modality=args.response_modality,
        timeout=args.timeout, on_event=print_event,
    ))
  except KeyboardInterrupt:
    raise SystemExit(130)
  except Exception as exc:
    parser.exit(1, f"Application failed: {exc}\n")
  print(json.dumps(result, ensure_ascii=False))
  if not result["success"]:
    raise SystemExit(1)


if __name__ == "__main__":
  main()
