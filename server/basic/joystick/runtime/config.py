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
    "models": {
        "A3_C": {
            "single": {
                "init_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
            },
            "dual": {
                "left": {
                    "init_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                },
                "right": {
                    "init_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                },
            },
        },
        "A3_T": {
            "single": {
                "init_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
            },
            "dual": {
                "left": {
                    "init_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                },
                "right": {
                    "init_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                },
            },
        },
        "A3_S": {
            "single": {
                "init_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
            },
            "dual": {
                "left": {
                    "init_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                },
                "right": {
                    "init_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                },
            },
        },
        "D3": {
            "dual": {
                "left": {
                    "init_joint": [0, 0, 0, -1.57, 0, 0, 0],
                },
                "right": {
                    "init_joint": [0, 0, 0, -1.57, 0, 0, 0],
                },
            },
        },
    },
}


class JoystickConfigError(ValueError):
    """遥操配置不可用于当前初始化请求。"""


def identify_arm_model(arm_name: str, arm_dof: int) -> str:
    """从名称与自由度数识别机械臂型号（A3_C/A3_T/A3_S/D3）。

    Args: arm_name 机械臂名称；arm_dof 实际自由度。
    Returns: 规范化型号字符串。
    Raises: JoystickConfigError 型号不支持/分型歧义，或轴数与型号不符。
    """
    name = str(arm_name or "").upper()
    families = [family for family in ("A3", "D3") if family in name]
    if len(families) != 1:
        raise JoystickConfigError(f"不支持或有歧义的机械臂型号：{arm_name}")
    if families[0] == "A3":
        variants = [variant for variant in "CTS" if variant in name]
        if len(variants) != 1:
            raise JoystickConfigError(f"A3 型号必须包含唯一的 C/T/S 分型：{arm_name}")
        model = "A3_" + variants[0]
    else:
        model = "D3"
    expected = 7 if model == "D3" else 6
    if arm_dof != expected:
        raise JoystickConfigError(f"型号 {arm_name} 应为 {expected} 轴，实际为 {arm_dof}")
    return model


@dataclass(frozen=True)
class ArmControlConfig:
    """单臂的不可变控制配置（当前只有初始关节角）。"""

    init_joint: Tuple[float, ...]


class JoystickConfigSnapshot:
    """后端进程生命周期内不可重新加载的配置快照。"""

    def __init__(
        self,
        data: Optional[Mapping[str, Any]],
        source_path: Path,
        load_error: Optional[str] = None,
    ) -> None:
        """保存配置快照、来源路径与加载错误。"""
        self._data = copy.deepcopy(data) if data is not None else None
        self.source_path = source_path
        self.load_error = load_error

    def resolve_arm(self, model: str, mode: str, side: str) -> ArmControlConfig:
        """解析指定型号/模式/臂侧的初始化关节配置。

        Args: model 型号；mode "single"/"dual"；side "left"/"right"。
        Returns: 对应的 ArmControlConfig。
        Raises: JoystickConfigError 模式/臂侧/型号非法，或字段缺失、类型错误。
        """
        self._ensure_loaded()
        if mode not in ("single", "dual") or side not in ("left", "right"):
            raise JoystickConfigError("无效的遥操模式或臂侧")
        if model == "D3" and mode != "dual":
            raise JoystickConfigError("D3 仅支持双臂遥操")
        if model not in ("A3_C", "A3_T", "A3_S", "D3"):
            raise JoystickConfigError(f"不支持的机械臂型号：{model}")
        models = self._require_mapping(self._data, "models")
        profile = self._require_mapping(models, model, f"models.{model}")
        section = self._require_mapping(profile, mode, f"models.{model}.{mode}")
        path = f"models.{model}.{mode}"
        if mode == "dual":
            section = self._require_mapping(section, side, f"{path}.{side}")
            path += f".{side}"
        return ArmControlConfig(self._require_joint_list(
            section, "init_joint", 7 if model == "D3" else 6, f"{path}.init_joint"))

    def resolve_quest_pose_jump(self) -> QuestPoseJumpConfig:
        """解析 Quest 位姿跳变阈值配置。

        Returns: QuestPoseJumpConfig。
        Raises: JoystickConfigError 配置不可用或字段非法。
        """
        self._ensure_loaded()
        try:
            return resolve_quest_pose_jump(self._data)
        except ValueError as exc:
            raise JoystickConfigError(str(exc)) from exc

    def _ensure_loaded(self) -> None:
        """确认配置已成功加载且根节点为对象。"""
        if self.load_error:
            raise JoystickConfigError(self.load_error)
        if not isinstance(self._data, Mapping):
            raise JoystickConfigError("遥操配置根节点必须是对象")

    @staticmethod
    def _require_mapping(container: Mapping[str, Any], key: str, path: Optional[str] = None) -> Mapping[str, Any]:
        """取出必填的子对象字段；缺失或类型不符时报错。"""
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
        """取出 (0, maximum] 区间内的有限数值字段。"""
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
    def _require_joint_list(
        container: Mapping[str, Any],
        key: str,
        expected_count: int,
        path: str,
    ) -> Tuple[float, ...]:
        """取出定长的有限数值序列并转为 tuple。"""
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
    """返回仓库默认的 joystick 配置文件路径。"""
    return Path(__file__).resolve().parents[4] / "config" / "joystick_config.json"


def load_joystick_config(path: Optional[Path] = None) -> JoystickConfigSnapshot:
    """加载 joystick 配置快照；缺失或损坏时回退内置默认值并记录日志。

    Args: path 可选配置文件路径，缺省使用仓库默认路径。
    Returns: JoystickConfigSnapshot（含来源路径与可选的加载错误）。
    """
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
