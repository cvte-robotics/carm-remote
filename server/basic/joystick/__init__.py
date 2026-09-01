"""Joystick public facade."""

from .runtime.paths import setup_import_paths as _setup_import_paths

_setup_import_paths()

from .runtime.dependencies import DEPENDENCIES_AVAILABLE, Carm
from .session.controller import JoystickController

__all__ = [
    "Carm",
    "DEPENDENCIES_AVAILABLE",
    "JoystickController",
    "get_joystick_controller",
    "joystick_controller",
]

joystick_controller = None


def get_joystick_controller(end_effector_factory=None):
    global joystick_controller
    if joystick_controller is None:
        joystick_controller = JoystickController(
            end_effector_factory=end_effector_factory,
        )
    return joystick_controller
