"""Quest 设备适配模块（输入源 provider 入口）。"""

MODULE_ID = "quest"
MODULE_API_VERSION = 1

from .module import QuestInputModule

__all__ = ["MODULE_API_VERSION", "MODULE_ID", "QuestInputModule"]
