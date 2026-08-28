"""Quest-specific configuration and pose-jump thresholds."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

DEFAULT_QUEST_POSITION_THRESHOLD_M = 0.05
DEFAULT_QUEST_ROTATION_THRESHOLD_DEG = 20.0


@dataclass(frozen=True)
class QuestPoseJumpConfig:
    position_threshold_m: float
    rotation_threshold_deg: float


def resolve_quest_pose_jump(data: Mapping[str, Any]) -> QuestPoseJumpConfig:
    """Resolve Quest jump thresholds without loading files or causing side effects."""
    if "quest_pose_jump" not in data:
        return QuestPoseJumpConfig(
            position_threshold_m=DEFAULT_QUEST_POSITION_THRESHOLD_M,
            rotation_threshold_deg=DEFAULT_QUEST_ROTATION_THRESHOLD_DEG,
        )
    section = data["quest_pose_jump"]
    if not isinstance(section, Mapping):
        raise ValueError("quest_pose_jump 必须是对象")

    def require_bounded_number(key: str, maximum: float) -> float:
        value = section.get(key)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value <= 0 or value > maximum):
            raise ValueError(f"quest_pose_jump.{key} 必须是 (0, {maximum}] 范围内的有限数值")
        return float(value)

    try:
        return QuestPoseJumpConfig(
            position_threshold_m=require_bounded_number("position_threshold_m", 0.2),
            rotation_threshold_deg=require_bounded_number("rotation_threshold_deg", 90.0),
        )
    except ValueError as exc:
        raise type(exc)(str(exc))
