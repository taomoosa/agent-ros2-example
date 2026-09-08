"""Abstract base class for embodiments (Lite version).

Defines the minimal interface that every embodiment must implement.
"""

import abc
import asyncio
from typing import Any


class Embodiment(abc.ABC):
  """Abstract base class for an embodiment."""

  @abc.abstractmethod
  def get_audio_queue(self) -> asyncio.Queue:
    """Returns the queue for audio observations."""

  @abc.abstractmethod
  def get_video_queue(self) -> asyncio.Queue:
    """Returns the queue for video observations."""

  @abc.abstractmethod
  def get_text_queue(self) -> asyncio.Queue:
    """Returns the queue for text observations."""

  @abc.abstractmethod
  async def execute_action(self, action_name: str, **kwargs: Any) -> str:
    """Executes an action by name and returns a result string."""

  @abc.abstractmethod
  def get_tools(self) -> list[dict[str, Any]]:
    """Returns tool declarations as a list of dicts (Gemini API JSON format).

    Each dict has the shape:
        {"functionDeclarations": [{"name": ..., "description": ..., "parameters": {...}}]}
    """

  @abc.abstractmethod
  def get_system_instruction(self) -> str:
    """Returns the system instruction string for this embodiment."""

  async def close(self) -> None:
    """Releases resources.  Override in subclasses that hold connections."""
