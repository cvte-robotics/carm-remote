"""Quest device adaptation module."""

MODULE_ID = "quest"
MODULE_API_VERSION = 1

from .module import QuestInputModule

__all__ = ["MODULE_API_VERSION", "MODULE_ID", "QuestInputModule"]
