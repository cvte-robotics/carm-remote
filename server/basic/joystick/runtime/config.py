"""手柄遥操参数配置。

配置文件只在模块导入时读取一次；运行期间不会自动重新加载。
"""

from __future__ import annotations

import copy
import json
import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

from quest.config import (
    DEFAULT_QUEST_POSITION_THRESHOLD_M,
    DEFAULT_QUEST_ROTATION_THRESHOLD_DEG,
    QuestPoseJumpConfig,
    resolve_quest_pose_jump,
)

DEFAULT_JOYSTICK_CONFIG: Dict[str, Any] = {
    "quest_pose_jump": {
        "position_threshold_m": DEFAULT_QUEST_POSITION_THRESHOLD_M,
        "rotation_threshold_deg": DEFAULT_QUEST_ROTATION_THRESHOLD_DEG,
    },
    "single_arm": {
        "gripper_tau": 25,
        "init_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
    },
    "dual_arm": {
        "left": {
            "gripper_tau": 25,
            "init_joint": {
                "six_axis": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                "seven_axis": [0, 0, 0, -1.57, 0, 0, 0],
            },
        },
        "right": {
            "gripper_tau": 25,
            "init_joint": {
                "six_axis": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                "seven_axis": [0, 0, 0, -1.57, 0, 0, 0],
            },
        },
    },
}


class JoystickConfigError(ValueError):
    """遥操配置不可用于当前初始化请求。"""


@dataclass(frozen=True)
class ArmControlConfig:
    gripper_tau: float
    init_joint: Tuple[float, ...]


class JoystickConfigSnapshot:
    """后端进程生命周期内不可重新加载的配置快照。"""

    def __init__(
        self,
        data: Optional[Mapping[str, Any]],
        source_path: Path,
        load_error: Optional[str] = None,
    ) -> None:
        self._data = copy.deepcopy(data) if data is not None else None
        self.source_path = source_path
        self.load_error = load_error

    def resolve_session(
        self,
        mode: str,
        master_side: str,
        use_seven_axis_joints: bool,
    ) -> Dict[str, ArmControlConfig]:
        self._ensure_loaded()
        normalized_mode = (mode or "single").strip().lower()
        normalized_side = (master_side or "right").strip().lower()
        if normalized_side not in ("left", "right"):
            raise JoystickConfigError(f"当前臂方向无效：{master_side}")

        if normalized_mode == "single":
            return {normalized_side: self._parse_single_arm()}
        if normalized_mode != "dual":
            raise JoystickConfigError(f"遥操模式无效：{mode}")

        axis_key = "seven_axis" if use_seven_axis_joints else "six_axis"
        return {
            side: self._parse_dual_arm(side, axis_key)
            for side in ("left", "right")
        }

    def resolve_quest_pose_jump(self) -> QuestPoseJumpConfig:
        self._ensure_loaded()
        try:
            return resolve_quest_pose_jump(self._data)
        except ValueError as exc:
            raise JoystickConfigError(str(exc)) from exc

    def _ensure_loaded(self) -> None:
        if self.load_error:
            raise JoystickConfigError(self.load_error)
        if not isinstance(self._data, Mapping):
            raise JoystickConfigError("遥操配置根节点必须是对象")

    def _parse_single_arm(self) -> ArmControlConfig:
        section = self._require_mapping(self._data, "single_arm")
        return ArmControlConfig(
            gripper_tau=self._require_tau(section, "single_arm.gripper_tau"),
            init_joint=self._require_joint_list(section, "init_joint", 6, "single_arm.init_joint"),
        )

    def _parse_dual_arm(self, side: str, axis_key: str) -> ArmControlConfig:
        dual = self._require_mapping(self._data, "dual_arm")
        section = self._require_mapping(dual, side, f"dual_arm.{side}")
        init_joint = self._require_mapping(section, "init_joint", f"dual_arm.{side}.init_joint")
        expected_count = 7 if axis_key == "seven_axis" else 6
        return ArmControlConfig(
            gripper_tau=self._require_tau(section, f"dual_arm.{side}.gripper_tau"),
            init_joint=self._require_joint_list(
                init_joint,
                axis_key,
                expected_count,
                f"dual_arm.{side}.init_joint.{axis_key}",
            ),
        )

    @staticmethod
    def _require_mapping(container: Mapping[str, Any], key: str, path: Optional[str] = None) -> Mapping[str, Any]:
        value = container.get(key)
        field_path = path or key
        if not isinstance(value, Mapping):
            raise JoystickConfigError(f"{field_path} 必须是对象")
        return value

    @staticmethod
    def _require_bounded_number(
        container: Mapping[str, Any],
        key: str,
        maximum: float,
        path: str,
    ) -> float:
        value = container.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
            or value > maximum
        ):
            raise JoystickConfigError(f"{path} 必须是 (0, {maximum}] 范围内的有限数值")
        return float(value)

    @staticmethod
    def _require_tau(container: Mapping[str, Any], path: str) -> float:
        value = container.get("gripper_tau")
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise JoystickConfigError(f"{path} 必须是大于等于 0 的有限数值")
        return float(value)

    @staticmethod
    def _require_joint_list(
        container: Mapping[str, Any],
        key: str,
        expected_count: int,
        path: str,
    ) -> Tuple[float, ...]:
        value = container.get(key)
        if not isinstance(value, list) or len(value) != expected_count:
            raise JoystickConfigError(f"{path} 必须包含 {expected_count} 个有限数值")
        if any(
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(item)
            for item in value
        ):
            raise JoystickConfigError(f"{path} 必须包含 {expected_count} 个有限数值")
        return tuple(float(item) for item in value)


def default_config_path() -> Path:
    return Path(__file__).resolve().parents[4] / "config" / "joystick_config.json"


def load_joystick_config(path: Optional[Path] = None) -> JoystickConfigSnapshot:
    config_path = Path(path) if path is not None else default_config_path()
    if not config_path.exists():
        logging.warning("[JoystickConfig] 配置文件不存在，使用内置默认值: %s", config_path)
        return JoystickConfigSnapshot(DEFAULT_JOYSTICK_CONFIG, config_path)

    try:
        with config_path.open("r", encoding="utf-8") as config_file:
            data = json.load(config_file)
    except (OSError, json.JSONDecodeError) as exc:
        error = f"遥操配置文件读取失败：{config_path}：{exc}"
        logging.error("[JoystickConfig] %s", error)
        return JoystickConfigSnapshot(None, config_path, error)

    logging.info("[JoystickConfig] 配置已加载，重启后端前不会重新读取: %s", config_path)
    return JoystickConfigSnapshot(data, config_path)
