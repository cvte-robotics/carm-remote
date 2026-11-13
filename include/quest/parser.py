"""将 receiver 已校验的 Quest 字段投影为冻结样本，不重复校验数值。"""

import math
from typing import Any, Dict, FrozenSet, Optional

from carm_teleop_input_contracts import ButtonSample, PoseSample, WaistControlSample
from .constants import LEFT_PREFIX, RIGHT_PREFIX, QUEST_CORE_INTENTS, QUEST_INTENT_FIELDS


_HEAD_POSE_FIELDS = ("hpx", "hpy", "hpz", "hqx", "hqy", "hqz", "hqw")


def parse_waist_control(payload: Dict[str, Any]) -> Optional[WaistControlSample]:
    """解析统一腰控制扩展；结构错误仅使本帧腰输入不可用。"""
    mode_value = payload.get("wm")
    if type(mode_value) is not int or mode_value not in (0, 1):
        return None
    validity, recenter = payload.get("hpv"), payload.get("hrn")
    values = [payload.get(name) for name in _HEAD_POSE_FIELDS]
    if (type(validity) is not int or validity not in (0, 1)
            or type(recenter) is not int or recenter < 0
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values)):
        return None
    finite = all(math.isfinite(v) for v in values)
    valid = validity == 1 and finite and sum(v * v for v in values[3:]) > 1e-6
    if not finite:
        values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    head = PoseSample(*(float(v) for v in values), timestamp=float(int(payload["ts"])))
    return WaistControlSample("head" if mode_value == 0 else "stick", head, valid, recenter)


def _scale_unit_to_int(value: Any) -> int:
    """把 receiver 已校验的 [0, 1] 功能模拟量缩放到 [0, 127] 整数。"""
    return int(max(0, min(127, round(float(value) * 127.0))))


def _parse_intent(payload, channel, declared_intents):
    """按唯一字段表投影已声明的功能；receiver 已完成必填与类型校验。"""
    intent = {}
    for name, field in QUEST_INTENT_FIELDS.items():
        if name not in declared_intents or field.channel != channel:
            continue
        value = payload[name]
        if field.value_type == "toggle":
            value = bool(value)
        elif field.value_type == "analog":
            value = _scale_unit_to_int(value)
        elif field.value_type == "vector2":
            value = (float(value[0]), float(value[1]))
        else:
            value = int(value)
        intent[field.sample_field] = value
    return intent


def _parse_hand(
    payload: Dict[str, Any], prefix: str, timestamp: int,
    intent_v1: bool, declared_intents: FrozenSet[str],
) -> Dict[str, Any]:
    """投影单手数据；缺省值、兼容别名与模拟量缩放已由 receiver 完成。"""
    pose_prefix = LEFT_PREFIX if prefix == LEFT_PREFIX else ""
    pose = PoseSample(
        x=float(payload[pose_prefix + "px"]), y=float(payload[pose_prefix + "py"]),
        z=float(payload[pose_prefix + "pz"]), qx=float(payload[pose_prefix + "qx"]),
        qy=float(payload[pose_prefix + "qy"]), qz=float(payload[pose_prefix + "qz"]),
        qw=float(payload[pose_prefix + "qw"]), timestamp=timestamp,
    )
    if prefix == LEFT_PREFIX:
        button = ButtonSample(
            timestamp=timestamp, btn_xa=payload["x"], btn_yb=payload["y"],
            btn_menu=payload["menu"], menu_capture=payload["menu"],
            btn_system=False, joystick_z=payload["lrb"], trigger=payload["lt"],
            grip=payload["lg"], joystick_x=payload["lsx"], joystick_y=payload["lsy"],
            reserve=0,
        )
    else:
        button = ButtonSample(
            timestamp=timestamp, btn_xa=payload["a"], btn_yb=payload["b"],
            btn_menu=payload["rtb"], menu_capture=payload["rtb"],
            btn_system=False, joystick_z=payload["rgb"], trigger=payload["rt"],
            grip=payload["rg"], joystick_x=payload["rsx"], joystick_y=payload["rsy"],
            reserve=0,
        )
    validity_key = "lpv" if prefix == LEFT_PREFIX else "rpv"
    result = {"pose": pose, "button": button, "pose_valid": payload[validity_key] == 1}
    if intent_v1:
        channel = "left" if prefix == LEFT_PREFIX else "right"
        result["intent"] = _parse_intent(payload, channel, declared_intents)
    return result


def parse_quest_frame(
    payload: Dict[str, Any], intent_v1: bool = False,
    declared_intents: Optional[FrozenSet[str]] = None,
) -> Dict[str, Any]:
    """解析 receiver 已校验并规范化的帧，左右手直接携带样本。

    ``intent_v1`` 时另外输出 receiver 已严格校验的功能结果（保留原始按键），
    已声明的机身功能放在 ``body_intent``；``declared_intents`` 为 None 时沿用旧
    intent_v1 的核心字段默认集合。
    """
    declared = QUEST_CORE_INTENTS if declared_intents is None else declared_intents
    timestamp = int(payload["ts"])
    frame_index = int(payload["fi"])
    result = {
        "timestamp": timestamp,
        "frame_index": frame_index,
        "left": _parse_hand(payload, LEFT_PREFIX, timestamp, intent_v1, declared),
        "right": _parse_hand(payload, RIGHT_PREFIX, timestamp, intent_v1, declared),
    }
    body = _parse_intent(payload, "body", declared) if intent_v1 else None
    if body:
        result["body_intent"] = body
    return result
