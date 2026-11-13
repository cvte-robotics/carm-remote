"""Joystick 公开 facade。

对外暴露 JoystickController 与 Carm SDK 封装类；
导入前先装配 Quest 输入模块的 import 路径。
"""

from .runtime.paths import setup_import_paths as _setup_import_paths

_setup_import_paths()

from .runtime.dependencies import Carm
from .session.controller import JoystickController

__all__ = [
    "Carm",
    "JoystickController",
]
