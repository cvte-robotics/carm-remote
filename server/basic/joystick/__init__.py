"""Joystick 公开 facade。

对外暴露 JoystickController、依赖可用标记（DEPENDENCIES_AVAILABLE）与
Carm SDK 封装类；导入前先装配 Quest 输入模块的 import 路径。
"""

from .runtime.paths import setup_import_paths as _setup_import_paths

_setup_import_paths()

from .runtime.dependencies import DEPENDENCIES_AVAILABLE, Carm
from .session.controller import JoystickController

__all__ = [
    "Carm",
    "DEPENDENCIES_AVAILABLE",
    "JoystickController",
]
