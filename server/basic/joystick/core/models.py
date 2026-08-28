import logging
import math
from typing import Any, Dict, List, Optional

from carm_teleop_input_contracts import CalibrationMode, ButtonStatus, PoseData

def pose_to_dict(pose: Any) -> Dict[str, Any]:
    """将 PoseData 转换为可序列化的字典"""
    if pose is None:
        return None

    def vector_from_attr(obj, base_name):
        value = getattr(obj, base_name, None)
        if isinstance(value, (list, tuple)):
            return [float(value[0]) if len(value) > 0 else 0.0,
                    float(value[1]) if len(value) > 1 else 0.0,
                    float(value[2]) if len(value) > 2 else 0.0]
        comps = []
        for suffix in ('x', 'y', 'z'):
            attr_name = f'{base_name}_{suffix}'
            comps.append(float(getattr(obj, attr_name, 0.0)))
        return comps

    return {
        "x": getattr(pose, 'x', 0.0),
        "y": getattr(pose, 'y', 0.0),
        "z": getattr(pose, 'z', 0.0),
        "qx": getattr(pose, 'qx', 0.0),
        "qy": getattr(pose, 'qy', 0.0),
        "qz": getattr(pose, 'qz', 0.0),
        "qw": getattr(pose, 'qw', 1.0),
        "timestamp": getattr(pose, 'timestamp', None),
        "confidence": getattr(pose, 'confidence', 1.0),
        "linear_velocity": vector_from_attr(pose, 'linear_velocity'),
        "angular_velocity": vector_from_attr(pose, 'angular_velocity'),
        "acceleration": vector_from_attr(pose, 'acceleration'),
    }


def button_to_dict(button: Any) -> Dict[str, Any]:
    """将 ButtonStatus 转换为可序列化的字典"""
    if button is None:
        return None
    return {
        "trigger": getattr(button, 'trigger', 0),
        "grip": getattr(button, 'grip', 0),
        "btn_xa": getattr(button, 'btn_xa', False),
        "btn_yb": getattr(button, 'btn_yb', False),
        "btn_menu": getattr(button, 'btn_menu', False),
        "btn_system": getattr(button, 'btn_system', False),
        "joystick_x": getattr(button, 'joystick_x', getattr(button, 'thumbstick_x', 0.0)),
        "joystick_y": getattr(button, 'joystick_y', getattr(button, 'thumbstick_y', 0.0)),
        "joystick_z": getattr(button, 'joystick_z', getattr(button, 'thumbstick_z', 0.0)),
        "menu_capture": getattr(button, 'menu_capture', getattr(button, 'btn_menu', False)),
        "timestamp": getattr(button, 'timestamp', None),
    }


def create_pose_data(
    *,
    timestamp: int,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    confidence: float = 1.0,
    linear_velocity: Optional[List[float]] = None,
    linear_velocity_x: float = 0.0,
    linear_velocity_y: float = 0.0,
    linear_velocity_z: float = 0.0,
    angular_velocity: Optional[List[float]] = None,
    angular_velocity_x: float = 0.0,
    angular_velocity_y: float = 0.0,
    angular_velocity_z: float = 0.0,
    acceleration: Optional[List[float]] = None,
    acceleration_x: float = 0.0,
    acceleration_y: float = 0.0,
    acceleration_z: float = 0.0,
):
    def unpack_vector(value: Optional[List[float]], defaults: List[float]) -> List[float]:
        if not isinstance(value, (list, tuple)):
            return defaults
        normalized = list(defaults)
        for index, item in enumerate(value[:3]):
            normalized[index] = item
        return normalized

    linear_velocity_x, linear_velocity_y, linear_velocity_z = unpack_vector(
        linear_velocity,
        [linear_velocity_x, linear_velocity_y, linear_velocity_z],
    )
    angular_velocity_x, angular_velocity_y, angular_velocity_z = unpack_vector(
        angular_velocity,
        [angular_velocity_x, angular_velocity_y, angular_velocity_z],
    )
    acceleration_x, acceleration_y, acceleration_z = unpack_vector(
        acceleration,
        [acceleration_x, acceleration_y, acceleration_z],
    )

    return PoseData(
        timestamp=timestamp,
        x=x,
        y=y,
        z=z,
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        confidence=confidence,
        linear_velocity_x=linear_velocity_x,
        linear_velocity_y=linear_velocity_y,
        linear_velocity_z=linear_velocity_z,
        angular_velocity_x=angular_velocity_x,
        angular_velocity_y=angular_velocity_y,
        angular_velocity_z=angular_velocity_z,
        acceleration_x=acceleration_x,
        acceleration_y=acceleration_y,
        acceleration_z=acceleration_z,
    )


def create_button_status(
    *,
    timestamp: int,
    btn_xa: bool,
    btn_yb: bool,
    btn_menu: bool = False,
    btn_system: bool = False,
    menu_capture: bool = False,
    joystick_z: bool = False,
    reserve: int = 0,
    trigger: int = 0,
    grip: int = 0,
    joystick_x: int = 0,
    joystick_y: int = 0,
):
    button = ButtonStatus(
        timestamp=timestamp,
        btn_xa=btn_xa,
        btn_yb=btn_yb,
        menu_capture=menu_capture,
        joystick_z=joystick_z,
        reserve=reserve,
        trigger=trigger,
        grip=grip,
        joystick_x=joystick_x,
        joystick_y=joystick_y,
    )
    # Some SDK builds expose only the original constructor fields.
    setattr(button, "btn_menu", btn_menu)
    setattr(button, "btn_system", btn_system)
    return button
