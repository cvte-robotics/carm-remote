"""Joystick public facade."""

from .runtime.paths import (
    quest_only_providers_enabled as _quest_only_providers_enabled,
    setup_import_paths as _setup_import_paths,
)

_setup_import_paths()
_QUEST_ONLY_PROVIDERS = _quest_only_providers_enabled()

from .runtime.dependencies import (
    ARM_SDK_AVAILABLE,
    BLE_DEPENDENCIES_AVAILABLE,
    CORE_DEPENDENCIES_AVAILABLE,
    DEPENDENCIES_AVAILABLE,
    Carm,
)
from .runtime.config import ArmControlConfig, JoystickConfigError, load_joystick_config
from .runtime.context import JOYSTICK_CONFIG
from carm_teleop_input_contracts import (
    ButtonStatus,
    CalibrationMode,
    HAND_LEFT,
    HAND_RIGHT,
    PoseData,
    normalize_hand,
)
if not _QUEST_ONLY_PROVIDERS:
    from surreal.bluetooth import BLE_ADDRESS_RE, BackgroundBluetoothAgent
    from surreal.constants import SDK_DISCONNECT_TIMEOUT, SDK_LOOP_TIMEOUT
from quest.constants import (
    QUEST_ACTIVITY,
    QUEST_EXIT_TIMEOUT,
    QUEST_FIRST_FRAME_TIMEOUT,
    QUEST_PACKAGE,
    QUEST_PROTOCOL_VERSION,
    QUEST_RX_TIMEOUT,
    QUEST_TCP_HOST,
    QUEST_TCP_PORT,
)
from quest.receiver import QuestTcpReceiver
from quest.adb import (
    get_adb_device_info,
    get_adb_device_state,
    is_quest_adb_device,
    launch_quest_app,
    list_adb_devices,
    run_adb,
    scan_quest_devices,
)
from quest.parser import parse_quest_frame
from .core.constants import (
    CONTROL_LOOP_HZ,
    CONTROL_LOOP_INTERVAL,
    DEFAULT_GRIPPER_TAU,
    HAND_CONTROL_VELOCITY,
    POSE_SEGMENT_ANGLE_ATOL_RAD,
    POSE_SEGMENT_ANGLE_RAD,
)
from .core.models import (
    button_to_dict,
    create_button_status,
    create_pose_data,
    pose_to_dict,
)
from .core.command_controller import CommandController
from .session.constants import (
    CLEANUP_THREAD_TIMEOUT,
    CLEANUP_WORKER_NAME,
    CONTROLLER_THREAD_TIMEOUT,
    WORKER_CHECK_INTERVAL,
)
from .session.state import JoystickState
from .session.arm import (
    LOCAL_ARM_CONTROL_HOST,
    DualArmSession,
    SingleArmSession,
    build_arm_bindings,
)
from .session.controller import JoystickController

__all__ = [
    "ARM_SDK_AVAILABLE",
    "ArmControlConfig",
    "BLE_ADDRESS_RE",
    "BLE_DEPENDENCIES_AVAILABLE",
    "BackgroundBluetoothAgent",
    "ButtonStatus",
    "CLEANUP_THREAD_TIMEOUT",
    "CLEANUP_WORKER_NAME",
    "CONTROLLER_THREAD_TIMEOUT",
    "CONTROL_LOOP_HZ",
    "CONTROL_LOOP_INTERVAL",
    "CORE_DEPENDENCIES_AVAILABLE",
    "CalibrationMode",
    "Carm",
    "CommandController",
    "DEFAULT_GRIPPER_TAU",
    "DEPENDENCIES_AVAILABLE",
    "DualArmSession",
    "HAND_CONTROL_VELOCITY",
    "HAND_LEFT",
    "HAND_RIGHT",
    "JOYSTICK_CONFIG",
    "JoystickConfigError",
    "JoystickController",
    "JoystickState",
    "LOCAL_ARM_CONTROL_HOST",
    "POSE_SEGMENT_ANGLE_ATOL_RAD",
    "POSE_SEGMENT_ANGLE_RAD",
    "PoseData",
    "QUEST_ACTIVITY",
    "QUEST_EXIT_TIMEOUT",
    "QUEST_FIRST_FRAME_TIMEOUT",
    "QUEST_PACKAGE",
    "QUEST_PROTOCOL_VERSION",
    "QUEST_RX_TIMEOUT",
    "QUEST_TCP_HOST",
    "QUEST_TCP_PORT",
    "QuestTcpReceiver",
    "SDK_DISCONNECT_TIMEOUT",
    "SDK_LOOP_TIMEOUT",
    "SingleArmSession",
    "WORKER_CHECK_INTERVAL",
    "build_arm_bindings",
    "button_to_dict",
    "create_button_status",
    "create_pose_data",
    "get_adb_device_info",
    "get_adb_device_state",
    "get_joystick_controller",
    "is_quest_adb_device",
    "joystick_controller",
    "launch_quest_app",
    "list_adb_devices",
    "load_joystick_config",
    "normalize_hand",
    "parse_quest_frame",
    "pose_to_dict",
    "run_adb",
    "scan_quest_devices",
]

if _QUEST_ONLY_PROVIDERS:
    __all__ = [
        name
        for name in __all__
        if name not in {
            "BLE_ADDRESS_RE",
            "BackgroundBluetoothAgent",
            "SDK_DISCONNECT_TIMEOUT",
            "SDK_LOOP_TIMEOUT",
        }
    ]

joystick_controller = None


def get_joystick_controller():
    global joystick_controller
    if joystick_controller is None:
        joystick_controller = JoystickController()
    return joystick_controller
