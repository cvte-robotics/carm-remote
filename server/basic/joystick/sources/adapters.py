"""provider 解析对象 → 中立冻结样本（sample）的严格适配器。"""

import math
from typing import Any, Tuple, Union

from carm_teleop_input_contracts import ButtonSample, PoseSample, RawInputBatch


_MISSING = object()


def _number(value: Any, field: str) -> Union[int, float]:
    """校验并返回有限数值（拒绝 bool）。"""
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{field} must be a finite number")
    return value


def _integer(value: Any, field: str) -> int:
    """校验并返回非 bool 整数。"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be a non-bool integer")
    return value


def _boolean(value: Any, field: str) -> bool:
    """校验并返回布尔值。"""
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value


def _attribute(obj: Any, name: str, default: Any = _MISSING) -> Any:
    """读取属性；缺失时返回 default，未给 default 则报错。"""
    value = getattr(obj, name, _MISSING)
    if value is _MISSING:
        if default is _MISSING:
            raise ValueError(f"missing required field: {name}")
        return default
    return value


def _vector(obj: Any, attribute: str) -> Tuple[float, float, float]:
    """读取三分量向量：优先 list/tuple 字段，否则回退 _x/_y/_z 分量。"""
    value = getattr(obj, attribute, _MISSING)
    if value is not _MISSING:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"{attribute} must contain exactly three numbers")
        return tuple(_number(item, f"{attribute}[{index}]") for index, item in enumerate(value))

    return tuple(
        _number(
            _attribute(obj, f"{attribute}_{axis}", 0.0),
            f"{attribute}_{axis}",
        )
        for axis in ("x", "y", "z")
    )


def pose_sample_from_object(pose: Any) -> PoseSample:
    """把 provider 位姿对象适配为冻结的 PoseSample。"""
    return PoseSample(
        x=_number(_attribute(pose, "x"), "x"),
        y=_number(_attribute(pose, "y"), "y"),
        z=_number(_attribute(pose, "z"), "z"),
        qx=_number(_attribute(pose, "qx"), "qx"),
        qy=_number(_attribute(pose, "qy"), "qy"),
        qz=_number(_attribute(pose, "qz"), "qz"),
        qw=_number(_attribute(pose, "qw"), "qw"),
        timestamp=_number(_attribute(pose, "timestamp"), "timestamp"),
        confidence=_number(_attribute(pose, "confidence", 1.0), "confidence"),
        linear_velocity=_vector(pose, "linear_velocity"),
        angular_velocity=_vector(pose, "angular_velocity"),
        acceleration=_vector(pose, "acceleration"),
    )


def button_sample_from_object(button: Any) -> ButtonSample:
    """把 provider 按钮对象适配为冻结的 ButtonSample。"""
    return ButtonSample(
        trigger=_integer(_attribute(button, "trigger"), "trigger"),
        grip=_integer(_attribute(button, "grip"), "grip"),
        btn_xa=_boolean(_attribute(button, "btn_xa"), "btn_xa"),
        btn_yb=_boolean(_attribute(button, "btn_yb"), "btn_yb"),
        btn_menu=_boolean(_attribute(button, "btn_menu", False), "btn_menu"),
        btn_system=_boolean(_attribute(button, "btn_system", False), "btn_system"),
        menu_capture=_boolean(
            _attribute(button, "menu_capture", False),
            "menu_capture",
        ),
        joystick_x=_integer(_attribute(button, "joystick_x"), "joystick_x"),
        joystick_y=_integer(_attribute(button, "joystick_y"), "joystick_y"),
        joystick_z=_boolean(_attribute(button, "joystick_z", False), "joystick_z"),
        reserve=_integer(_attribute(button, "reserve", 0), "reserve"),
        timestamp=_number(_attribute(button, "timestamp"), "timestamp"),
    )


def validate_raw_input_batch(batch: RawInputBatch) -> RawInputBatch:
    """在 Host 事件边界强制 RawInputBatch 的构造期校验。"""
    if not isinstance(batch, RawInputBatch):
        raise TypeError("input modules must publish RawInputBatch instances")
    return batch
