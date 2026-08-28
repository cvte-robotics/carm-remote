import math
from typing import Any, Dict

from .constants import LEFT_PREFIX, RIGHT_PREFIX


def _clamp_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _clamp_bool(value: Any) -> bool:
    return bool(value)


def _clamp_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _scale_axis_to_int(value: Any) -> int:
    raw = _clamp_float(value)
    return int(max(-127, min(127, round(raw * 127.0))))


def _scale_trigger_to_int(value: Any) -> int:
    raw = _clamp_float(value)
    return int(max(0, min(127, round(raw * 127.0))))


def _build_pose_dict(*, timestamp: int, x: Any, y: Any, z: Any, qx: Any, qy: Any, qz: Any, qw: Any) -> Dict[str, Any]:
    return {
        "timestamp": _clamp_int(timestamp),
        "x": _clamp_float(x),
        "y": _clamp_float(y),
        "z": _clamp_float(z),
        "qx": _clamp_float(qx),
        "qy": _clamp_float(qy),
        "qz": _clamp_float(qz),
        "qw": _clamp_float(qw, 1.0),
        "confidence": 1.0,
        "linear_velocity_x": 0.0,
        "linear_velocity_y": 0.0,
        "linear_velocity_z": 0.0,
        "angular_velocity_x": 0.0,
        "angular_velocity_y": 0.0,
        "angular_velocity_z": 0.0,
        "acceleration_x": 0.0,
        "acceleration_y": 0.0,
        "acceleration_z": 0.0,
    }


def _build_button_dict(*, timestamp: int, btn_xa: Any, btn_yb: Any, menu_capture: Any = False, joystick_z: Any = False, trigger: Any = 0, grip: Any = 0, joystick_x: Any = 0, joystick_y: Any = 0) -> Dict[str, Any]:
    return {
        "timestamp": _clamp_int(timestamp),
        "btn_xa": _clamp_bool(btn_xa),
        "btn_yb": _clamp_bool(btn_yb),
        "btn_menu": _clamp_bool(menu_capture),
        "btn_system": False,
        "menu_capture": _clamp_bool(menu_capture),
        "joystick_z": _clamp_bool(joystick_z),
        "trigger": _scale_trigger_to_int(trigger) if not isinstance(trigger, int) else int(max(0, min(127, trigger))),
        "grip": _scale_trigger_to_int(grip) if not isinstance(grip, int) else int(max(0, min(127, grip))),
        "joystick_x": _scale_axis_to_int(joystick_x) if not isinstance(joystick_x, int) else int(max(-127, min(127, joystick_x))),
        "joystick_y": _scale_axis_to_int(joystick_y) if not isinstance(joystick_y, int) else int(max(-127, min(127, joystick_y))),
    }


# Keep the low-level helper names available to callers of the former parser module.
clamp_float = _clamp_float
clamp_bool = _clamp_bool
clamp_int = _clamp_int
scale_axis_to_int = _scale_axis_to_int
scale_trigger_to_int = _scale_trigger_to_int
build_pose_dict = _build_pose_dict
build_button_dict = _build_button_dict


def _parse_hand(payload: Dict[str, Any], prefix: str, timestamp: int, frame_index: int) -> Dict[str, Any]:
    def pose_value(name: str, default: float) -> float:
        key = f"{prefix}{name}" if prefix == LEFT_PREFIX else name
        return payload.get(key, default)

    pose = _build_pose_dict(
        timestamp=timestamp,
        x=pose_value("px", 0.0), y=pose_value("py", 0.0), z=pose_value("pz", 0.0),
        qx=pose_value("qx", 0.0), qy=pose_value("qy", 0.0), qz=pose_value("qz", 0.0), qw=pose_value("qw", 1.0),
    )
    if prefix == LEFT_PREFIX:
        button = _build_button_dict(
            timestamp=timestamp, btn_xa=payload.get("x", payload.get("lta", 0)), btn_yb=payload.get("y", payload.get("ltb", 0)),
            menu_capture=payload.get("menu", payload.get("lmb", 0)), joystick_z=payload.get("lrb", 0), trigger=payload.get("lt", 0.0),
            grip=payload.get("lg", 0.0), joystick_x=payload.get("lsx", 0.0), joystick_y=payload.get("lsy", 0.0),
        )
    else:
        button = _build_button_dict(
            timestamp=timestamp, btn_xa=payload.get("a", payload.get("rta", 0)), btn_yb=payload.get("b", payload.get("rgbtn", payload.get("rtb2", 0))),
            menu_capture=payload.get("rtb", 0), joystick_z=payload.get("rgb", 0), trigger=payload.get("rt", 0.0),
            grip=payload.get("rg", 0.0), joystick_x=payload.get("rsx", 0.0), joystick_y=payload.get("rsy", 0.0),
        )

    validity_key = "lpv" if prefix == LEFT_PREFIX else "rpv"
    validity_value = payload.get(validity_key)
    if (isinstance(validity_value, bool) or not isinstance(validity_value, (int, float)) or
            not math.isfinite(validity_value) or validity_value not in (0, 1)):
        raise ValueError(f"Quest frame field {validity_key} must be numeric 0 or 1")
    return {"pose": pose, "button": button, "pose_valid": validity_value == 1, "frame_index": frame_index}


def parse_quest_frame(payload: Dict[str, Any]) -> Dict[str, Any]:
    prefixed_right_pose_fields = {f"r{name}" for name in ("px", "py", "pz", "qx", "qy", "qz", "qw")}
    if prefixed_right_pose_fields.intersection(payload):
        raise ValueError("Quest frame must use unprefixed right-hand pose fields")
    frame_index_value = payload.get("fi")
    if (isinstance(frame_index_value, bool) or not isinstance(frame_index_value, (int, float)) or
            not math.isfinite(frame_index_value) or int(frame_index_value) != frame_index_value):
        raise ValueError("Quest frame field fi must be a finite integer")
    timestamp = int(payload.get("ts") or 0)
    frame_index = int(frame_index_value)
    return {
        "timestamp": timestamp,
        "frame_index": frame_index,
        "left": _parse_hand(payload, LEFT_PREFIX, timestamp, frame_index),
        "right": _parse_hand(payload, RIGHT_PREFIX, timestamp, frame_index),
    }
