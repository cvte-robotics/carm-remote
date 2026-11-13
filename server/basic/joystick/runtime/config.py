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
                "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "teleop_reset_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
            },
            "dual": {
                "left": {
                    "init_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                },
                "right": {
                    "init_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005],
                },
            },
        },
        "A3_T": {
            "single": {
                "init_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "teleop_reset_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
            },
            "dual": {
                "left": {
                    "init_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                },
                "right": {
                    "init_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [0.002811, 1.556416, -1.010268, 0.974864, -0.013982, 0.011311],
                },
            },
        },
        "A3_S": {
            "single": {
                "init_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "teleop_reset_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
            },
            "dual": {
                "left": {
                    "init_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                },
                "right": {
                    "init_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [0.002461, 1.293866, -0.754631, 0.018857, 0.586803, -0.001139],
                },
            },
        },
        "D3": {
            "arm_angle": {
                "enabled": True,
                "deadzone": 0.15,
                "max_speed_deg_s": 45.0,
                "min_deg": -170.0,
                "max_deg": 170.0,
                "min_step_deg": 0.2,
            },
            "dual": {
                "left": {
                    "init_joint": [0, 0, 0, -1.57, 0, 0, 0],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [0, 0, 0, -1.57, 0, 0, 0],
                },
                "right": {
                    "init_joint": [0, 0, 0, -1.57, 0, 0, 0],
                    "home_joint": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "teleop_reset_joint": [0, 0, 0, -1.57, 0, 0, 0],
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
    """单臂的不可变控制配置。

    init_joint：会话初始化位置；home_joint：停止遥操时的回零位置；
    teleop_reset_joint：遥操中按回初始位键时的目标位置。
    """

    init_joint: Tuple[float, ...]
    home_joint: Tuple[float, ...]
    teleop_reset_joint: Tuple[float, ...]


@dataclass(frozen=True)
class WaistControlConfig:
    """D3 腰的统一控制配置；关节位置单位 rad。"""

    enabled: bool = False
    init_joint: Tuple[float, float] = (0.0, 0.0)
    teleop_reset_joint: Tuple[float, float] = (0.0, 0.0)
    home_joint: Tuple[float, float] = (0.0, 0.0)
    max_speed_deg_s: float = 10.0
    deadzone: float = 0.15


@dataclass(frozen=True)
class ArmAngleControlConfig:
    """D3 单臂臂型角控制配置；角度单位为度。"""

    enabled: bool = False
    deadzone: float = 0.15
    max_speed_deg_s: float = 45.0
    min_deg: float = -170.0
    max_deg: float = 170.0
    min_step_deg: float = 0.2


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
        """解析指定型号/模式/臂侧的关节位置配置。

        ``init_joint`` 必填；``home_joint`` 缺省为全零，``teleop_reset_joint`` 缺省
        与 ``init_joint`` 相同（兼容只配置了初始化位置的旧配置文件）。

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
        dof = 7 if model == "D3" else 6
        init_joint = self._require_joint_list(section, "init_joint", dof, f"{path}.init_joint")
        home_joint = (
            self._require_joint_list(section, "home_joint", dof, f"{path}.home_joint")
            if "home_joint" in section else (0.0,) * dof
        )
        teleop_reset_joint = (
            self._require_joint_list(
                section, "teleop_reset_joint", dof, f"{path}.teleop_reset_joint")
            if "teleop_reset_joint" in section else init_joint
        )
        return ArmControlConfig(init_joint, home_joint, teleop_reset_joint)

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

    def resolve_waist(self) -> WaistControlConfig:
        """解析 ``models.D3.waist``；缺省或 ``enabled=false`` 时腰不启用。

        Raises: JoystickConfigError 字段类型或数值非法。
        """
        self._ensure_loaded()
        section = self._data.get("models", {}).get("D3", {}).get("waist")
        if section is None:
            return WaistControlConfig()
        if not isinstance(section, Mapping) or type(section.get("enabled")) is not bool:
            raise JoystickConfigError("models.D3.waist.enabled 必须是布尔值")
        if not section["enabled"]:
            return WaistControlConfig()
        init_joint = self._require_joint_list(section, "init_joint", 2, "models.D3.waist.init_joint")
        teleop_reset_joint = (self._require_joint_list(section, "teleop_reset_joint", 2,
                                                       "models.D3.waist.teleop_reset_joint")
                              if "teleop_reset_joint" in section else init_joint)
        home_joint = (self._require_joint_list(section, "home_joint", 2,
                                               "models.D3.waist.home_joint")
                      if "home_joint" in section else (0.0, 0.0))
        max_speed = self._require_bounded_number(section, "max_speed_deg_s", 360.0,
                                                 "models.D3.waist.max_speed_deg_s") \
            if "max_speed_deg_s" in section else 10.0
        deadzone = section.get("deadzone", 0.15)
        if (isinstance(deadzone, bool) or not isinstance(deadzone, (int, float))
                or not math.isfinite(deadzone) or not 0 <= deadzone < 1):
            raise JoystickConfigError("models.D3.waist.deadzone 必须是 [0, 1) 范围内的有限数值")
        if "scale" in section:
            logging.warning("[JoystickConfig] models.D3.waist.scale 已废弃并被忽略：腰与手柄共用灵敏度")
        return WaistControlConfig(True, init_joint, teleop_reset_joint, home_joint,
                                  max_speed, float(deadzone))

    def resolve_arm_angle(self) -> ArmAngleControlConfig:
        """解析 ``models.D3.arm_angle``；缺省或关闭时不启用臂型角控制。"""
        self._ensure_loaded()
        section = self._data.get("models", {}).get("D3", {}).get("arm_angle")
        if section is None:
            return ArmAngleControlConfig()
        if not isinstance(section, Mapping) or type(section.get("enabled")) is not bool:
            raise JoystickConfigError("models.D3.arm_angle.enabled 必须是布尔值")
        if not section["enabled"]:
            return ArmAngleControlConfig()
        deadzone = section.get("deadzone", 0.15)
        if (isinstance(deadzone, bool) or not isinstance(deadzone, (int, float))
                or not math.isfinite(deadzone) or not 0 <= deadzone < 0.9):
            raise JoystickConfigError("models.D3.arm_angle.deadzone 必须是 [0, 0.9) 范围内的有限数值")
        max_speed = self._require_bounded_number(
            section, "max_speed_deg_s", 360.0, "models.D3.arm_angle.max_speed_deg_s")
        min_deg = section.get("min_deg", -170.0)
        max_deg = section.get("max_deg", 170.0)
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in (min_deg, max_deg)):
            raise JoystickConfigError("models.D3.arm_angle.min_deg/max_deg 必须是有限数值")
        if not -180 < min_deg < max_deg < 180:
            raise JoystickConfigError("models.D3.arm_angle 必须满足 -180 < min_deg < max_deg < 180")
        min_step = self._require_bounded_number(
            section, "min_step_deg", 180.0, "models.D3.arm_angle.min_step_deg")
        return ArmAngleControlConfig(
            True, float(deadzone), max_speed, float(min_deg), float(max_deg), min_step)

    def resolve_shared_connection(self) -> bool:
        """解析 ``sdk.shared_connection``：同一控制端点的臂与腰是否共用一条 SDK 连接，缺省启用。

        Raises: JoystickConfigError ``sdk`` 不是对象或开关不是布尔值。
        """
        self._ensure_loaded()
        section = self._data.get("sdk", {})
        value = section.get("shared_connection", True) if isinstance(section, Mapping) else None
        if type(value) is not bool:
            raise JoystickConfigError("sdk.shared_connection 必须是布尔值")
        return value

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


# 进程内配置单例：进程启动时加载一次。
JOYSTICK_CONFIG = load_joystick_config()
