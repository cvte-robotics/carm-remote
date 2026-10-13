"""将冻结输入样本投影为 SSE/HTTP 可序列化字典。"""

from typing import Any, Dict, Optional

from carm_teleop_input_contracts import ButtonSample, PoseSample


def _vector(values) -> list:
    """向量字段投影为三分量 list（不足补 0.0，与历史视图一致）。"""
    return [
        float(values[0]) if len(values) > 0 else 0.0,
        float(values[1]) if len(values) > 1 else 0.0,
        float(values[2]) if len(values) > 2 else 0.0,
    ]


def pose_to_dict(pose: Optional[PoseSample]) -> Optional[Dict[str, Any]]:
    """将最新被接受的 PoseSample 投影为 SSE/HTTP 字典。

    键顺序与值类型必须与 legacy PoseData 视图逐字节等价：标量（含 timestamp）
    原样透传，向量字段投影为 float 三分量 list。
    """
    if pose is None:
        return None
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
        "timestamp": pose.timestamp,
        "confidence": pose.confidence,
        "linear_velocity": _vector(pose.linear_velocity),
        "angular_velocity": _vector(pose.angular_velocity),
        "acceleration": _vector(pose.acceleration),
    }


def button_to_dict(button: Optional[ButtonSample]) -> Optional[Dict[str, Any]]:
    """将最新被接受的 ButtonSample 投影为 SSE/HTTP 字典。

    键顺序与值类型必须与 legacy ButtonStatus 视图逐字节等价。
    """
    if button is None:
        return None
    return {
        "trigger": button.trigger,
        "grip": button.grip,
        "btn_xa": button.btn_xa,
        "btn_yb": button.btn_yb,
        "btn_menu": button.btn_menu,
        "btn_system": button.btn_system,
        "joystick_x": button.joystick_x,
        "joystick_y": button.joystick_y,
        "joystick_z": button.joystick_z,
        "menu_capture": button.menu_capture,
        "timestamp": button.timestamp,
    }
