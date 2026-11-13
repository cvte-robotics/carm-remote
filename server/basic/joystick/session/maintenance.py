"""不经过遥操会话的机械臂维护操作：临时读取型号与关节，管理进程内初始化位置覆盖值。"""

import logging
import time
from typing import Any, Dict, List, Optional, Tuple

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, normalize_hand

from ..core.pose_math import _is_valid_joint_position
from ..runtime.config import identify_arm_model
from ..runtime.dependencies import ARM_SDK_AVAILABLE, Carm
from .arm import active_arm_profile, apply_profile_device_indexes


def _init_position_key(model: str, mode: str, hand: Optional[str]) -> Tuple[str, str, Optional[str]]:
    """初始化位置覆盖值的缓存键：单臂只按型号区分，双臂再按手侧区分。"""
    return (model, mode, hand if mode == "dual" else None)


def read_arm_joints(control_host: str, device_index: Optional[int] = None) -> Tuple[str, List[float]]:
    """通过临时连接读取一只臂的型号与当前关节值（D3 七轴，A3 六轴）。

    Args:
        control_host: 控制端点（Host/IP）
        device_index: 同一控制服务内的臂索引；None 表示默认臂
    Returns:
        (规范化型号, 关节值)
    Raises:
        ConnectionError: 连接失败
        JoystickConfigError: 型号不支持或轴数不符
        ValueError: 关节数据非法
    """
    if not ARM_SDK_AVAILABLE or Carm is None:
        raise RuntimeError("Pure Python Carm SDK is unavailable")
    carm = None
    try:
        carm = Carm(control_host) if device_index is None else Carm(control_host, device_index)
        time.sleep(0.2)
        if not carm.is_connected() and not carm.connect(timeout=1.0):
            raise ConnectionError(f"Failed to connect to {control_host}")
        model = identify_arm_model(carm.arm_name, carm.arm_dof)
        expected_joint_count = 7 if model == "D3" else 6
        joints = carm.joint_pos
        if not _is_valid_joint_position(joints, expected_joint_count):
            raise ValueError(f"关节数据非法：期望 {expected_joint_count} 个有限数值，实际 {joints}")
        return model, list(joints)
    finally:
        if carm is not None:
            try:
                carm.disconnect()
            except Exception as e:
                logging.warning("[ArmMaintenance] Error disconnecting temporary connection: %s", e)


def _resolve_init_position_joints(
    mode: str,
    arms: List[Dict[str, Any]],
    init_joint: Optional[Any],
) -> List[List[float]]:
    """按已读取的臂侧和轴数校验覆盖值，全部成功后返回独立列表。

    Args:
        mode: 规范化的单臂或双臂模式
        arms: 本次选中并已读取型号、关节值的机械臂
        init_joint: 请求指定的关节列表或双臂手侧对象；None 使用读取值
    Returns:
        与 arms 顺序一致的初始化位置
    Raises:
        ValueError: 请求形状、手侧或关节值不符合要求
    """
    if init_joint is None:
        return [list(arm["joints"]) for arm in arms]
    if isinstance(init_joint, list):
        if len(arms) != 1:
            raise ValueError("init_joint 列表仅支持选中一只机械臂，请指定 side 或按手侧传入对象")
        positions = [init_joint]
    elif isinstance(init_joint, dict):
        if mode != "dual":
            raise ValueError("init_joint 对象仅支持双臂模式")
        selected_sides = {arm["side"] for arm in arms}
        for hand in init_joint:
            if hand not in (HAND_LEFT, HAND_RIGHT):
                raise ValueError(f"init_joint 包含无效手侧：{hand}，仅支持 left/right")
            if hand not in selected_sides:
                raise ValueError(f"init_joint 包含未选中的 {hand} 侧")
        for arm in arms:
            if arm["side"] not in init_joint:
                raise ValueError(
                    f"init_joint 缺少 {arm['side']} 侧关节值"
                    f"（期望 {len(arm['joints'])} 个有限数值）"
                )
        positions = [init_joint[arm["side"]] for arm in arms]
    else:
        raise ValueError("init_joint 必须是关节列表或双臂手侧对象")

    resolved = []
    for arm, position in zip(arms, positions):
        expected_count = len(arm["joints"])
        try:
            valid = _is_valid_joint_position(position, expected_count)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"{arm['side']} 侧 init_joint 必须是 {expected_count} 个有限数值")
        resolved.append([float(value) for value in position])
    return resolved


class ArmMaintenance:
    """维护接口的实现；Host 持有一个实例，确认值只保留到进程退出。"""

    def __init__(self, owner):
        self._owner = owner
        # 进程内确认值在后续机械臂初始化时覆盖配置值；键见 ``_init_position_key``。
        self._cached_positions: Dict[Tuple[str, str, Optional[str]], List[float]] = {}
        # 最近一次单臂确认值，兼容旧版 ``init_joint`` 返回字段。
        self._last_single_position: Optional[List[float]] = None

    def _read_arms(
        self,
        ip: str,
        mode: Optional[str],
        slave_ip: Optional[str],
        master_side: Optional[str],
        side: Optional[str],
    ) -> Tuple[str, List[Dict[str, Any]]]:
        """按会话初始化相同的拓扑规则临时读取一只或两只臂的型号与关节值。

        Args:
            ip: 主臂管理 IP（板端主臂固定走本机控制端点）
            mode: "single"（默认）或 "dual"
            slave_ip: 双臂模式的从臂管理 IP；与 ip 相同即同板双臂（D3）
            master_side: 主臂手侧；同板双臂忽略（index 0 左、index 1 右）
            side: 双臂时只读该手侧；None 表示两只臂都读，单臂忽略
        Returns:
            (规范化 mode, 每臂 {role, side, model, joints})
        Raises:
            ValueError / JoystickConfigError / ConnectionError：参数、型号或读取失败
        """
        owner = self._owner
        normalized_mode = (mode or "single").strip().lower()
        normalized_side = None
        if normalized_mode == "dual" and side:
            normalized_side = str(side).strip().lower()
            if normalized_side not in (HAND_LEFT, HAND_RIGHT):
                raise ValueError(f"无效的臂侧：{side}")
        bindings = owner._arm_binding_builder(
            normalized_mode,
            ip,
            normalize_hand(master_side, HAND_RIGHT),
            slave_ip if normalized_mode == "dual" else None,
        )
        apply_profile_device_indexes(bindings, active_arm_profile(owner))
        if normalized_side is not None:
            bindings = [binding for binding in bindings if binding["hand"] == normalized_side]

        arms = []
        for binding in bindings:
            logging.info(
                "[ArmMaintenance] Reading %s arm (side=%s) from management_ip=%s "
                "control_host=%s arm_index=%s",
                binding["role"], binding["hand"], binding["ip"],
                binding["control_host"], binding["device_index"],
            )
            model, joints = read_arm_joints(binding["control_host"], binding["device_index"])
            # 与会话初始化同一规则校验型号/模式/手侧（如 D3 仅支持双臂）。
            owner.joystick_config.resolve_arm(model, normalized_mode, binding["hand"])
            arms.append({
                "role": binding["role"],
                "side": binding["hand"],
                "model": model,
                "joints": joints,
            })
        return normalized_mode, arms

    def confirm_init_position(
        self,
        ip: str,
        mode: Optional[str] = None,
        slave_ip: Optional[str] = None,
        master_side: Optional[str] = None,
        side: Optional[str] = None,
        init_joint: Optional[Any] = None,
    ) -> dict:
        """用请求值或当前位置覆盖本进程后续会话的初始化位置。

        单臂按型号、双臂按型号+手侧覆盖；始终读取机械臂以确认可达性、型号和轴数。
        全部读取并校验成功后才一次性写入，任一臂失败均不改动缓存。
        会话存在时拒绝确认，避免临时连接与会话争用同一控制服务。

        Args:
            ip, mode, slave_ip, master_side, side: 含义见 ``_read_arms``
            init_joint: 可选关节列表（仅选中一只臂）或双臂 left/right 对象；
                轴数须匹配且元素为有限数值，None 使用当前位置
        Returns:
            成功为 ``{"mode", "arms": [{role, side, model, init_joint, source}]}``，
            source 为 request（请求值）或 current（读取值）；失败为 ``{"error": str}``
        """
        owner = self._owner
        if not owner.available:
            logging.warning("[ArmMaintenance] Controller not available")
            return {"error": "Controller not available"}

        with owner._lifecycle_lock:
            if owner.session is not None:
                return {"error": "遥操会话进行中，请先停止会话再确认初始化位置"}
            try:
                normalized_mode, arms = self._read_arms(ip, mode, slave_ip, master_side, side)
                if not arms:
                    raise ValueError("没有匹配的机械臂")
                positions = _resolve_init_position_joints(normalized_mode, arms, init_joint)
            except Exception as e:
                logging.error("[ArmMaintenance] Failed to confirm init position: %s", e)
                return {"error": f"确认初始化位置失败：{e}"}

            source = "current" if init_joint is None else "request"
            with owner._data_lock:
                for arm, position in zip(arms, positions):
                    key = _init_position_key(arm["model"], normalized_mode, arm["side"])
                    self._cached_positions[key] = list(position)
                    logging.warning(
                        "[ArmMaintenance] Init position confirmed for %s (source=%s): %s",
                        key, source, position,
                    )
                if normalized_mode == "single":
                    self._last_single_position = list(positions[0])
            return {
                "mode": normalized_mode,
                "arms": [
                    {
                        "role": arm["role"],
                        "side": arm["side"],
                        "model": arm["model"],
                        "init_joint": position,
                        "source": source,
                    }
                    for arm, position in zip(arms, positions)
                ],
            }

    def get_init_position(self) -> dict:
        """获取接口覆盖、配置文件或原生默认值中的当前生效值。

        Returns:
            ``init_joint`` 兼容旧版（会话中为主臂当前生效值，否则为最近一次单臂确认值）；
            ``arms`` 为会话中各臂生效值；``overrides`` 为本进程全部确认覆盖值
        """
        owner = self._owner
        if not owner.available:
            return {"init_joint": None, "error": "Controller not available"}

        try:
            session = owner.session
            if session and owner.controller:
                result = {
                    "init_joint": owner.controller.init_joint_,
                    "arms": [
                        {
                            "role": runtime.role,
                            "side": runtime.hand,
                            "model": runtime.controller.arm_model,
                            "init_joint": runtime.controller.init_joint_,
                        }
                        for runtime in session.arms
                    ],
                }
            else:
                result = {"init_joint": self._last_single_position}
            with owner._data_lock:
                result["overrides"] = [
                    {"model": model, "mode": mode, "side": hand, "init_joint": list(joints)}
                    for (model, mode, hand), joints in self._cached_positions.items()
                ]
            return result
        except Exception as e:
            logging.exception(e)
            return {"init_joint": None, "error": f"[ArmMaintenance] Exception getting init position: {e}"}

    def get_current_joints(
        self,
        ip: str,
        mode: Optional[str] = None,
        slave_ip: Optional[str] = None,
        master_side: Optional[str] = None,
        side: Optional[str] = None,
    ) -> dict:
        """按与 ``confirm_init_position`` 相同的拓扑读取当前关节值。

        Returns:
            ``arms`` 为每臂 {role, side, model, joints}；只读一只臂时 ``current_joints``
            为其关节值（兼容旧版），否则为 None；失败含 ``error``
        """
        if not self._owner.available:
            return {"current_joints": None, "error": "Controller not available"}

        try:
            _, arms = self._read_arms(ip, mode, slave_ip, master_side, side)
            if not arms:
                return {"current_joints": None, "error": "没有匹配的机械臂"}
            return {
                "current_joints": arms[0]["joints"] if len(arms) == 1 else None,
                "arms": arms,
            }
        except Exception as e:
            logging.exception(e)
            return {"current_joints": None, "error": f"[ArmMaintenance] Exception getting current joints: {e}"}

    def get_cached_init_position(
        self,
        model: str,
        mode: str = "single",
        hand: Optional[str] = None,
    ) -> Optional[List[float]]:
        """获取进程内的初始化位置覆盖值（双臂按手侧区分，D3 为七轴）。"""
        try:
            key = _init_position_key(model, mode, hand)
            with self._owner._data_lock:
                cached_position = self._cached_positions.get(key)
                if _is_valid_joint_position(cached_position, 7 if model == "D3" else 6):
                    logging.info("[ArmMaintenance] Retrieved cached init position for %s: %s", key, cached_position)
                    return cached_position.copy()
                if cached_position:
                    logging.warning("[ArmMaintenance] Cached position validation failed. Position: %s", cached_position)
                return None
        except Exception as e:
            logging.error("[ArmMaintenance] Error retrieving cached init position: %s", e)
            return None
