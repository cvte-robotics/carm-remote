"""手柄遥操控制算法核心（输入位姿 → 机械臂目标的映射与 SDK 派发事务）。

``ControlEpoch`` / ``TargetState`` 以对象身份作为失效依据；SDK 派发经
generation-bound 接口在 Host 锁外完成“发送前复核 + 物理发送 + 提交”。本模块不持
Host data 锁，只使用 ``pose_state_lock``（锁序 ``pose_state_lock → _policy_lock``）。
"""

import logging
import math
import numbers
import threading
import time
from concurrent.futures import CancelledError as FuturesCancelledError, TimeoutError as FuturesTimeoutError
from dataclasses import dataclass
from typing import Any, Dict, Optional, List, Tuple

from ..runtime.dependencies import (
    ARM_SDK_AVAILABLE, Carm, CarmSdk, DEPENDENCIES_AVAILABLE, np, R,
)
from carm_teleop_input_contracts import (
    ButtonSample,
    HAND_LEFT,
    HAND_RIGHT,
    PoseSample,
    normalize_hand,
)
from ..runtime.config import identify_arm_model
from .pose_math import (
    _continuous_pose_diff_with_scale,
    _is_valid_joint_position,
    apply_pose_delta,
    project_input_to_arm,
)


_POSE_SEGMENT_ANGLE_RAD = math.radians(150.0)
_POSE_SEGMENT_ANGLE_ATOL_RAD = 1e-9


@dataclass(frozen=True)
class ControlEpoch:
    """控制纪元：对象身份即纪元（复核点比较 ``is``，不比较 serial）。

    在途操作复核点必须连同 binding 身份、ControlParams、arm_connection_generation
    一起比较，不得只比较 current_epoch 身份。
    """

    serial: int


@dataclass(frozen=True)
class ControlParams:
    """Host 发布的冻结输入映射参数。

    ``epoch`` 只在 yaw 标定或灵敏度 scale 变化时推进；其余失效事件由 binding
    会话身份或 ``ControlEpoch`` 对象身份承载。冻结样本与派发 attempt 都会携带
    计算时使用的 epoch，因此过期参数集不可能被静默复用。
    """

    epoch: int
    scale: float
    calibration: Any


@dataclass(frozen=True)
class ControlSample:
    """
一个控制周期冻结的输入样本（本周期位姿/按钮/参数的完整凭证）。

``params``/``epoch`` 为计算时参数与纪元，``session_id``/``input_source``/
``source_lease``/``source_token`` 为来源身份，``arm_connection_generation``/
``arm_feedback_state_sequence``/``arm_feedback_fresh`` 为机械臂反馈身份。
"""

    pose_data: Optional[PoseSample]
    button_data: Optional[ButtonSample]
    params: ControlParams
    input_sequence: int
    session_id: int
    input_source: str
    source_lease: Any
    source_token: Any
    epoch: ControlEpoch
    arm_connection_generation: int
    arm_feedback_state_sequence: Optional[int]
    arm_feedback_fresh: bool


@dataclass(frozen=True)
class ControlComputation:
    """一次控制计算的产物；``joint_move`` 非 None 时表示有待派发的关节指令。"""

    joint_move: Any = None


@dataclass(frozen=True)
class TargetState:
    """
一个冻结的位姿目标及计算凭证：对象身份即目标身份，派发复核按对象身份比较。

``reject_pose_target()`` 只清 target（不换 epoch）；``invalidate_pose_mapping()``
换 epoch 并清空。
"""

    epoch: ControlEpoch
    pose: Tuple[float, ...]
    input_sequence: Any
    session_id: Any
    params_epoch: Any
    input_source: Any
    arm_connection_generation: Any
    arm_feedback_state_sequence: Optional[int]
    source_token: Any
    end_effector_trigger: Any


@dataclass(frozen=True)
class DispatchAttempt:
    """
一次已准入的位姿派发事务（在 Host 锁外执行的冻结凭证）。

全部字段在 ``pose_state_lock`` 临界区内捕获，物理派发与 post 复核不取 Host 锁：
``target_state`` 为目标身份，``carm``/``arm_ws_identity``/``arm_state_identity``
为连接身份，``pending_*`` 为成功后才提交的基线。
"""

    target_state: TargetState
    epoch: ControlEpoch
    carm: Any
    arm_ws_identity: Any
    arm_state_identity: Any
    arm_connection_generation: Any
    arm_feedback_state_sequence: Optional[int]
    end_effector_driver: Any
    hand_binding: str
    arm_role: str
    pending_input_pose: Any
    pending_target_pose: Any


@dataclass(frozen=True)
class JointMoveAttempt:
    """一次已准入的关节运动派发事务（A 键回零点路径）。

    ``joints`` 是待发送的关节值；``epoch`` / ``session_id`` / ``params_epoch`` /
    ``input_source`` / ``arm_connection_generation`` 与 ``carm`` /
    ``arm_ws_identity`` / ``arm_state_identity`` 是准入复核所用的完整身份。
    """

    joints: Tuple[float, ...]
    epoch: ControlEpoch
    session_id: int
    params_epoch: int
    input_source: str
    arm_connection_generation: int
    carm: Any
    arm_ws_identity: Any
    arm_state_identity: Any


class CommandController:
    """单臂控制算法核心（输入位姿 → 目标 pose / 关节指令）。

    绑定一只 hand 与一个 arm_role，持有该臂的位姿映射状态与机械臂连接身份。
    位姿映射状态只在 ``pose_state_lock`` 内读写；物理派发在锁外执行，由 attempt
    身份复核兜底。
    """

    def __init__(
        self,
        scale=0.4,
        hand_binding=HAND_RIGHT,
        arm_role="master",
        configured_init_joint=None,
    ):
        self.isStop = False
        self.end_arm_pose = [0, 0, 0, 0, 0, 0, 1]
        self.scale = scale
        self.hand_binding = normalize_hand(hand_binding, HAND_RIGHT)
        self.arm_role = arm_role
        self.default_init_joint = [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005]
        self.seven_axis_init_joint = [0, 0, 0, -1.57, 0, 0, 0]
        self.init_joint_ = (
            list(configured_init_joint)
            if configured_init_joint is not None
            else self.default_init_joint.copy()
        )

        self.zero_joint = [0,0,0,0,0,0]
        self.seven_axis_zero_joint = [0,0,0,0,0,0,0]

        self.init_end_vr_pose = None
        self.init_end_arm_pose = None
        self.current_target: Optional[TargetState] = None
        self.relative_quaternion = None
        self.pending_pose_segment = None
        self.pose_state_lock = threading.RLock()
        self._epoch_serial = 0
        self.current_epoch = ControlEpoch(serial=self._epoch_serial)
        self.arm_connection_generation = 0
        self.arm_feedback_state_sequence = 0
        self.arm_feedback_connection_generation = None
        self.arm_feedback_ws_identity = None
        self.arm_feedback_state_identity = None
        self.arm_feedback_accept_monotonic = None
        self._arm_ws_identity = None
        self._arm_state_identity = None
        self._arm_connected = False
        self.zero_quat = [0, 0, 0, 1]

        self.last_aButton = False
        self.last_bButton = False
        self.need_sync_feedback = False
        self.carm_ = None
        self._pending_joint_move = None
        self.end_effector_driver = None
        self.carm_device_index = None
        self.use_seven_axis_joints = False
        self.motion_cancel_event = threading.Event()

    def _expected_joint_count(self) -> int:
        """返回当前臂型期望的关节数（D3 为七轴，其余为六轴）。"""
        return 7 if self.use_seven_axis_joints else 6

    def _active_init_joint(self) -> list:
        """返回当前生效的初始化位置副本；配置缺失或长度不符时抛异常。"""
        expected_joint_count = self._expected_joint_count()
        if self.init_joint_ is None:
            raise RuntimeError("未加载机械臂初始化位置配置")
        if len(self.init_joint_) != expected_joint_count:
            raise RuntimeError(
                f"机械臂初始化位置配置长度错误：期望 {expected_joint_count}，实际 {len(self.init_joint_)}"
            )
        return self.init_joint_.copy()

    def _active_zero_joint(self) -> list:
        """返回当前臂型对应的零点关节值副本。"""
        if self._expected_joint_count() == 7:
            return self.seven_axis_zero_joint.copy()
        return self.zero_joint

    def _with_temp_arm_connection(
        self,
        control_host: str,
        operation,
        device_index: Optional[int] = None,
    ):
        """使用明确的控制端点建立临时机械臂连接。"""
        temp_carm = None
        try:
            if not ARM_SDK_AVAILABLE or Carm is None:
                raise RuntimeError("Pure Python Carm SDK is unavailable")
            temp_carm = (
                Carm(control_host)
                if device_index is None
                else Carm(control_host, device_index)
            )
            if temp_carm is None:
                raise RuntimeError(f"Failed to create Carm instance for {control_host}")

            time.sleep(0.2)
            if not temp_carm.is_connected():
                if not temp_carm.connect(timeout=1.0):
                    raise ConnectionError(f"Failed to connect to {control_host}")

            return operation(temp_carm)

        finally:
            if temp_carm is not None:
                try:
                    temp_carm.disconnect()
                    logging.debug(
                        "[CommandController] Temporary connection to %s (arm_index=%s) disconnected",
                        control_host,
                        device_index,
                    )
                except Exception as e:
                    logging.warning(f"[CommandController] Error disconnecting temporary connection: {e}")

    def confirmInitPosition(self, control_host: str) -> bool:
        """读取当前机械臂位置并覆盖本进程的初始化位置。"""
        try:
            def read_position(carm):
                """读取当前关节值并覆盖本进程的初始化位置。"""
                self.arm_model = identify_arm_model(carm.arm_name, carm.arm_dof)
                if self.arm_model == "D3":
                    raise ValueError("D3 不支持单臂初始化位置确认")
                temp_joint = carm.joint_pos
                if not _is_valid_joint_position(temp_joint, 6):
                    logging.error(f"[CommandController] Invalid joint position data: {temp_joint}")
                    return False

                self.init_joint_ = list(temp_joint)
                logging.warning(f"[CommandController] Init position confirmed and cached: {self.init_joint_}")
                return True

            return self._with_temp_arm_connection(control_host, read_position)
        except ConnectionError as e:
            logging.error(f"[CommandController] {e}")
            return False
        except Exception as e:
            logging.error(f"[CommandController] Error confirming init position: {e}")
            logging.exception(e)
            return False

    def getInitPosition(self) -> dict:
        """获取当前生效的初始化位置。"""
        logging.warning(f"[CommandController] Getting init position: {self.init_joint_}")
        if self.init_joint_ is None:
            return {"init_joint": None}
        return {"init_joint": self.init_joint_}

    def load_cached_init_position(self, cached_position) -> bool:
        """存在接口缓存时覆盖配置初始化值，否则保留配置或原生默认值。"""
        try:
            expected_joint_count = self._expected_joint_count()
            if _is_valid_joint_position(cached_position, expected_joint_count):
                self.init_joint_ = list(cached_position)
                logging.info(f"[CommandController] Loaded cached init position: {cached_position}")
                return True
            logging.info("[CommandController] No valid cached position found; keeping configured init position")
        except Exception as e:
            logging.error(f"[CommandController] Error loading cached init position: {e}")
        return False

    def getCurrentJoints(self, control_host: str) -> dict:
        """通过当前控制器的设备索引读取六轴或七轴关节值。"""
        try:
            expected_joint_count = self._expected_joint_count()

            def read_joints(carm):
                """通过临时连接读取当前六轴/七轴关节值。"""
                self.arm_model = identify_arm_model(carm.arm_name, carm.arm_dof)
                if self.arm_model == "D3":
                    raise ValueError("D3 不支持单臂初始化位置确认")
                temp_joint = carm.joint_pos
                if not _is_valid_joint_position(temp_joint, expected_joint_count):
                    logging.error(
                        "[CommandController] Invalid joint position data: expected=%d actual=%s",
                        expected_joint_count,
                        temp_joint,
                    )
                    return {"current_joints": None, "error": "Invalid joint position data"}

                logging.warning(f"[CommandController] Getting current joints: {temp_joint}")
                return {"current_joints": list(temp_joint)}

            return self._with_temp_arm_connection(
                control_host,
                read_joints,
                device_index=self.carm_device_index,
            )

        except ConnectionError as e:
            error_msg = f"[CommandController] {e}"
            logging.error(error_msg)
            return {"current_joints": None, "error": error_msg}
        except Exception as e:
            error_msg = f"[CommandController] Error getting current joints: {e}"
            logging.exception(e)
            return {"current_joints": None, "error": error_msg}

    @staticmethod
    def _validated_arm_pose(state, arm_index):
        """从不可变 SDK state 快照中提取一个有限的笛卡尔 pose。

        Args:
            state: SDK 发布的不可变 state 快照（dict）
            arm_index: 该快照中本臂的下标
        Returns:
            Tuple[float, ...]：四元数已归一化的 7 分量 pose；快照结构或数值非法时返回 None
        """
        if not isinstance(state, dict):
            return None
        arms = state.get("arm")
        if not isinstance(arms, list) or not isinstance(arm_index, int):
            return None
        if arm_index < 0 or arm_index >= len(arms) or not isinstance(arms[arm_index], dict):
            return None
        pose = arms[arm_index].get("pose")
        if not isinstance(pose, (list, tuple)) or len(pose) < 7:
            return None
        values = pose[:7]
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in values
        ):
            return None
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:7]))
        if quaternion_norm <= 1e-12:
            return None
        normalized = [float(value) for value in values]
        normalized[3:7] = [value / quaternion_norm for value in normalized[3:7]]
        return normalized

    def _refresh_current_pose(self) -> bool:
        """只接受当前 SDK 连接送达的、尚未消费过的合法 state 帧。

        ws 身份变化、连接断开、连接新建立都会推进 ``arm_connection_generation``
        并调用 ``invalidate_pose_mapping()``，从而作废全部在途 attempt。

        Returns:
            bool：接受了新的合法反馈帧并刷新 ``end_arm_pose`` 时为 True；
            无新帧、帧非法或连接未就绪时为 False
        """
        with self.pose_state_lock:
            if not self.carm_:
                return False

            try:
                connected = bool(self.carm_.is_connected())
                ws_identity = getattr(self.carm_, "ws", None)
                state_identity = getattr(self.carm_, "state", None)
            except Exception as e:
                logging.debug(f"[CommandController] Failed to inspect arm connection: {e}")
                return False

            if ws_identity is not self._arm_ws_identity:
                self._arm_ws_identity = ws_identity
                self._arm_state_identity = state_identity
                self._arm_connected = False
                self.arm_feedback_connection_generation = None
                self.arm_connection_generation += 1
                self.invalidate_pose_mapping()

            if not connected:
                if self._arm_connected:
                    self._arm_connected = False
                    self._arm_state_identity = state_identity
                    self.arm_feedback_connection_generation = None
                    self.arm_connection_generation += 1
                    self.invalidate_pose_mapping()
                return False

            if not self._arm_connected:
                self._arm_connected = True
                self._arm_state_identity = state_identity
                self.arm_feedback_connection_generation = None
                self.arm_connection_generation += 1
                self.invalidate_pose_mapping()
                return False

            if state_identity is self._arm_state_identity:
                return False

            try:
                if (
                    self.carm_.ws is not ws_identity
                    or not self.carm_.is_connected()
                    or self.carm_.state is not state_identity
                ):
                    return False
            except Exception as e:
                logging.debug(f"[CommandController] Arm feedback changed during snapshot: {e}")
                return False

            self._arm_state_identity = state_identity
            self.arm_feedback_state_sequence += 1
            arm_index = getattr(self.carm_, "arm_index", 0)
            current_pose = self._validated_arm_pose(state_identity, arm_index)
            if current_pose is None:
                self.arm_feedback_connection_generation = None
                self.invalidate_pose_mapping()
                logging.warning(
                    "[CommandController] Ignoring invalid arm feedback "
                    "(generation=%d, sequence=%d)",
                    self.arm_connection_generation,
                    self.arm_feedback_state_sequence,
                )
                return False

            self.end_arm_pose = current_pose
            self.arm_feedback_connection_generation = self.arm_connection_generation
            # 接受时刻快照：记录本帧被接受时的 ws/state 身份与时钟；须与关节值
            # 配对使用，改读实时值会把旧反馈绑到重连后的新连接上。
            self.arm_feedback_ws_identity = ws_identity
            self.arm_feedback_state_identity = state_identity
            self.arm_feedback_accept_monotonic = time.monotonic()
            return True

    def connectArm(self, control_host) -> bool:
        """
通过控制端点建立机械臂连接并完成型号发现（使能由 ``prepareArm`` 完成）。

Args:
    control_host: 控制端点（Host/IP）。

Returns:
    实例创建、连接与型号发现全部成功时为 True。

Raises:
    RuntimeError: 型号读取被 ``motion_cancel_event`` 取消，或连接断开/超时。
"""
        if not ARM_SDK_AVAILABLE or Carm is None:
            logging.error("[CommandController] Pure Python Carm SDK is unavailable")
            return False

        try:
            logging.info(
                "[Arm] Attempting to connect to control_host=%s arm_index=%s...",
                control_host,
                self.carm_device_index,
            )
            # 创建 API 包装器
            if self.carm_ is None:
                try:
                    if self.carm_device_index is None:
                        self.carm_ = Carm(control_host)
                    else:
                        self.carm_ = Carm(control_host, self.carm_device_index)
                except Exception as e:
                    logging.error(f"[Arm] Failed to create Carm instance: {e}")
                    self.carm_ = None
                    return False

            # 验证实例是否成功创建
            if self.carm_ is None:
                logging.error("[Arm] Carm instance is None after creation attempt")
                return False

            time.sleep(0.2)  # 增加等待时间

            # 检查连接状态
            try:
                if not self.carm_.is_connected():
                    if not self.carm_.connect(timeout=1.0):
                        error_msg = f"[Arm] Failed to connect to {control_host} - check control endpoint and service"
                        logging.error(error_msg)
                        return False
            except Exception as e:
                logging.error(f"[Arm] Error checking connection status: {e}")
                return False

            deadline = time.monotonic() + 5.0
            while not self.carm_.arm_name or not self.carm_.arm_dof:
                if self.motion_cancel_event.wait(0.05) or not self.carm_.is_connected():
                    raise RuntimeError("读取机械臂型号已取消或连接断开")
                if time.monotonic() >= deadline:
                    raise RuntimeError("读取机械臂型号超时")
            self.arm_model = identify_arm_model(self.carm_.arm_name, self.carm_.arm_dof)
            self.use_seven_axis_joints = self.arm_model == "D3"
            return True
        except Exception:
            logging.exception("[Arm] Model discovery failed")
            raise

    def prepareArm(self, control_host) -> bool:
        """
校验初始化配置后使能机械臂（``set_ready`` + 控制模式 4 并注册错误回调）。

Args:
    control_host: 控制端点（仅保留调用形态，本方法不再建连）。

Returns:
    使能准备全部成功为 True；配置非法、已取消或异常时为 False。
"""
        try:
            self._active_init_joint()
            if self.motion_cancel_event.is_set():
                return False
            logging.info("[Arm] Connected. Enabling servo...")
            self.carm_.set_ready()
            time.sleep(0.2)

            logging.info("[Arm] Setting control mode...")
            self.carm_.set_control_mode(4)
            time.sleep(0.2)

            self.carm_.on_error(lambda msg: logging.error(f"[Arm Error] {msg}"))

            logging.info("[Arm] Initialization movement preparation completed")
            return True
        except Exception as e:
            error_msg = f"[Arm] Initialization preparation failed: {e}"
            logging.error(error_msg)
            logging.exception(e)
            return False

    def moveToInitPosition(self) -> bool:
        """触发已配置的初始化位置运动并等待名义运动窗口结束。

        Returns:
            bool：未连接、发送失败或异常时为 False，否则为 True（窗口结束不等于已到位）
        """
        if not self.carm_ or not self.carm_.is_connected():
            logging.error("[CommandController] Arm is not connected before initialization movement")
            return False

        logging.info("[Arm] Moving to configured initialization position...")
        try:
            init_joint = self._active_init_joint()
            logging.info(f"[CommandController] Sending move_joint command to position: {init_joint}")
            if not self._trigger_joint_motion(init_joint):
                return False
            self._refresh_current_pose()
            logging.info("[Arm] Initialization trigger window completed")
            return True
        except Exception as e:
            logging.error(f"[CommandController] Error during initialization movement: {e}")
            logging.exception(e)
            return False

    def _trigger_joint_motion(self, joints) -> bool:
        """仅触发关节运动并等待名义时长窗口（防重叠，不是到位判定）。

        Args:
            joints: 目标关节值
        Returns:
            bool：窗口正常结束且未取消时为 True；已取消或连接断开时为 False
        """
        if self.motion_cancel_event.is_set():
            return False
        duration = 5.0
        self.carm_.move_joint(joints, duration, False)
        deadline = time.monotonic() + duration
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return not self.motion_cancel_event.is_set()
            if self.motion_cancel_event.wait(min(0.05, remaining)):
                return False
            if not self.carm_.is_connected():
                self.motion_cancel_event.set()
                return False

    def _publish_new_epoch_locked(self) -> ControlEpoch:
        """发布新的 ControlEpoch 对象（对象身份即纪元）。

        即使载荷完全相同也每次新建对象，因此全部在途复核点都能靠身份比较失效。

        Returns:
            ControlEpoch：新发布的当前纪元
        """
        self._epoch_serial += 1
        self.current_epoch = ControlEpoch(serial=self._epoch_serial)
        return self.current_epoch

    def publish_epoch(self) -> ControlEpoch:
        """
**无锁**发布新 ControlEpoch（仅限 provider gate 临界区内调用，先换纪元再清 target）。

gate 内不能取 ``pose_state_lock``（会与控制线程锁序成环）；其余状态清理由随后的
``invalidate_pose_mapping()`` 在 pose 锁内完成。

Returns:
    新发布的当前纪元。
"""
        self._epoch_serial += 1
        self.current_epoch = ControlEpoch(serial=self._epoch_serial)
        self.current_target = None
        return self.current_epoch

    def publish_target_rejection(self) -> None:
        """
**无锁**清空当前目标（仅限 provider gate 临界区内调用）。

与 :meth:`reject_pose_target` 的区别只有不取 ``pose_state_lock``；在途 attempt
复核条件立即失效，其余清理留给 data 阶段。
"""
        self.current_target = None

    def reject_pose_target(self) -> None:
        """清空当前目标与未提交的位姿分段（不换 epoch）。"""
        with self.pose_state_lock:
            self.current_target = None
            self.pending_pose_segment = None

    def invalidate_pose_mapping(self) -> None:
        """作废位姿映射：换 epoch、清目标、丢弃四元数连续分支并要求重新同步反馈。

        在机械臂连接事件、配置变更与异常路径上调用，是放弃全部在途 attempt 的
        唯一入口。
        """
        with self.pose_state_lock:
            self._publish_new_epoch_locked()
            self.reject_pose_target()
            self.relative_quaternion = None
            self.need_sync_feedback = True

    def _reset_pose_state(self, end_vr_pose):
        """重建已提交的输入/机械臂基线与未提交的分段状态。

        以本周期输入位姿作为新的输入基线，机械臂基线取当前反馈 ``end_arm_pose``
        的前 7 位。

        Args:
            end_vr_pose: 本周期的输入位姿（4x4 矩阵）
        """
        with self.pose_state_lock:
            self.init_end_vr_pose = end_vr_pose.copy()
            self.init_end_arm_pose = self.end_arm_pose[:7]
            self.reject_pose_target()
            self.relative_quaternion = None

    def _commit_dispatch_attempt_locked(self, attempt: DispatchAttempt) -> None:
        """派发成功后在 ``pose_state_lock`` 内提交 attempt 携带的待定基线。

        只有捕获时存在未提交分段（``pending_input_pose``）才推进基线，并总是把
        该分段标记为已消费（``pending_pose_segment = None``）。
        """
        if attempt.pending_input_pose is not None:
            self.init_end_vr_pose = attempt.pending_input_pose.copy()
            self.init_end_arm_pose = attempt.pending_target_pose.copy()
            self.relative_quaternion = None
        self.pending_pose_segment = None

    def _dispatch_attempt_matches_locked(self, attempt: DispatchAttempt) -> bool:
        """post 复核：attempt 捕获时的全部身份是否仍然成立。

        调用前必须持有 ``pose_state_lock``；按对象身份比较 target/epoch/carm，
        并按值比较连接代次与反馈帧序号。

        Returns:
            bool：全部身份仍然成立时为 True
        """
        target = self.current_target
        # ControlEpoch 约束（见 ControlEpoch 文档）：除 current_epoch 身份外必须同时
        # 比较 target 身份、arm_connection_generation 与连接对象身份。
        return bool(
            # 目标身份：attempt.target_state 的每个字段都在捕获时取自同一个
            # TargetState，因此对象身份比较等价于旧版逐字段等值链。
            target is attempt.target_state
            and self.current_epoch is attempt.epoch
            and not self.isStop
            and self.end_effector_driver is attempt.end_effector_driver
            # arm 连接代次与反馈帧序号是独立维度，仍按值比较。
            and self.arm_connection_generation
            == attempt.arm_connection_generation
            and self.arm_feedback_state_sequence
            == attempt.arm_feedback_state_sequence
            and self.carm_ is attempt.carm
            and self._arm_connected
            and self.arm_feedback_connection_generation
            == attempt.arm_connection_generation
            and attempt.carm.ws is attempt.arm_ws_identity
            and attempt.carm.state is attempt.arm_state_identity
        )

    def _capture_dispatch_attempt_locked(self) -> Optional[DispatchAttempt]:
        """在 ``pose_state_lock`` 内捕获一次位姿派发 attempt。

        任一前置条件（未停止、目标与纪元一致、末端驱动与机械臂连接身份一致、
        反馈代次与帧序号一致、ws/state 对象身份一致）不成立时返回 None。

        Returns:
            Optional[DispatchAttempt]：可派发的冻结事务，或 None
        """
        target = self.current_target
        if (
            self.isStop
            or target is None
            or target.epoch is not self.current_epoch
            or target.end_effector_trigger is None
            or self.end_effector_driver is None
            or target.arm_connection_generation
            != self.arm_connection_generation
            or target.arm_feedback_state_sequence
            != self.arm_feedback_state_sequence
            or not self.carm_
            or not self.carm_.is_connected()
            or not self._arm_connected
            or self.arm_feedback_connection_generation
            != self.arm_connection_generation
            or self.carm_.ws is not self._arm_ws_identity
            or self.carm_.state is not self._arm_state_identity
        ):
            return None
        pending = self.pending_pose_segment
        return DispatchAttempt(
            target_state=target,
            epoch=target.epoch,
            carm=self.carm_,
            arm_ws_identity=self._arm_ws_identity,
            arm_state_identity=self._arm_state_identity,
            arm_connection_generation=target.arm_connection_generation,
            arm_feedback_state_sequence=target.arm_feedback_state_sequence,
            end_effector_driver=self.end_effector_driver,
            hand_binding=self.hand_binding,
            arm_role=self.arm_role,
            pending_input_pose=(
                pending["input_pose"].copy() if pending is not None else None
            ),
            pending_target_pose=(
                pending["target_pose"].copy() if pending is not None else None
            ),
        )

    def _transform_input_components(self, components, calibration):
        """把 7 分量 (x,y,z,qx,qy,qz,qw) 输入位姿投影到机械臂坐标系。"""
        end_vr_pose = np.eye(4)
        end_vr_pose[:3, 3] = components[:3]
        end_vr_pose[:3, :3] = R.from_quat(components[3:7]).as_matrix()
        return project_input_to_arm(end_vr_pose, calibration)

    def _transform_input_pose(self, pose: PoseSample, calibration):
        """把 PoseSample 展开成 7 分量元组，复用 ``_transform_input_components``。"""
        return self._transform_input_components(
            (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw),
            calibration,
        )

    def _apply_input_rebase_locked(
        self,
        raw_pose,
        calibration,
        button_status,
        arm_feedback_fresh,
    ) -> bool:
        """
在 ``pose_state_lock`` 内应用一次 Quest 输入 rebase（重建基线）。

Args:
    raw_pose: provider 送达的 7 分量原始输入位姿。
    calibration: 输入投影标定。
    button_status: 本周期按钮样本（用于同步 ``last_bButton``）。
    arm_feedback_fresh: 本周期是否接受了新的反馈帧。

Returns:
    rebase 提交成功为 True；身份漂移、转换异常或前置条件不满足为 False。
"""
        expected_ws = self._arm_ws_identity
        expected_state = self._arm_state_identity
        expected_arm_generation = self.arm_connection_generation
        expected_feedback_sequence = self.arm_feedback_state_sequence
        if (
            not arm_feedback_fresh
            or self.isStop
            or not self.carm_
            or not self._arm_connected
            or self.arm_feedback_connection_generation != expected_arm_generation
            or not self.carm_.is_connected()
            or self.carm_.ws is not expected_ws
            or self.carm_.state is not expected_state
        ):
            self.reject_pose_target()
            return False

        try:
            # raw_pose 是 provider 送达的 7 分量元组，直接构建 4x4 矩阵，
            # 不再经过 legacy PoseData 中转。
            end_vr_pose = self._transform_input_components(raw_pose[:7], calibration)
        except Exception as e:
            self.reject_pose_target()
            logging.warning("[CommandController] Quest rebase pose conversion failed: %s", e)
            return False

        if (
            self.isStop
            or self.arm_connection_generation != expected_arm_generation
            or self.arm_feedback_state_sequence != expected_feedback_sequence
            or self.arm_feedback_connection_generation != expected_arm_generation
            or not self.carm_.is_connected()
            or self.carm_.ws is not expected_ws
            or self.carm_.state is not expected_state
        ):
            self.reject_pose_target()
            return False

        self.init_end_vr_pose = end_vr_pose.copy()
        self.init_end_arm_pose = self.end_arm_pose[:7]
        self.reject_pose_target()
        self.relative_quaternion = None
        self.need_sync_feedback = False
        self.last_bButton = bool(button_status.btn_yb)
        return True

    def process_input_sample(self, sample: ControlSample) -> ControlComputation:
        """把一个已归一化的 Host 输入样本应用到控制映射。

        Args:
            sample: 控制线程冻结的本周期输入样本
        Returns:
            ControlComputation，joint_move 非 None 时表示有待派发的关节指令；
            SDK 依赖不可用或计算异常时返回空计算结果
        """
        if not DEPENDENCIES_AVAILABLE:
            self.reject_pose_target()
            return ControlComputation()

        try:
            pose_sample = sample.pose_data
            button_sample = sample.button_data

            value_trigger = button_sample.trigger
            value_squeeze = button_sample.grip
            flag_squeeze = value_squeeze > 100
            flag_aButton = button_sample.btn_xa
            flag_bButton = button_sample.btn_yb

            end_vr_pose = self._transform_input_pose(pose_sample, sample.params.calibration)

            return self.compute_control(
                end_vr_pose,
                flag_squeeze,
                flag_aButton,
                flag_bButton,
                value_trigger,
                sample.input_sequence,
                sample.session_id,
                sample.params,
                sample.input_source,
                sample.epoch,
                sample.arm_connection_generation,
                sample.arm_feedback_fresh,
                sample.arm_feedback_state_sequence,
                sample.source_token,
            )
        except Exception as e:
            self.invalidate_pose_mapping()
            logging.exception(e)
            logging.error(f"[CommandController] Callback error: {e}")
            return ControlComputation()

    def compute_control(
        self,
        end_vr_pose,
        flag_squeeze,
        flag_aButton,
        flag_bButton,
        value_trigger,
        input_sequence,
        session_id,
        params,
        input_source,
        expected_epoch,
        arm_connection_generation,
        arm_feedback_fresh,
        arm_feedback_state_sequence=None,
        source_token=None,
    ):
        """在 ``pose_state_lock`` 内串行化一个控制样本对可变位姿映射状态的修改。

        Args:
            end_vr_pose: 本周期的输入位姿（4x4 矩阵）
            flag_squeeze / flag_aButton / flag_bButton: 本周期按钮状态
            value_trigger: 末端触发值
            input_sequence / session_id / params / input_source: 来源身份与参数集
            expected_epoch: 计算时持有的纪元（身份比较，不等即放弃本周期）
            arm_connection_generation: 计算时的臂连接代次
            arm_feedback_fresh: 本周期是否接受了新的反馈帧
            arm_feedback_state_sequence: 冻结的反馈帧序号
            source_token: provider 决策 token
        Returns:
            ControlComputation，joint_move 非 None 时表示有待派发的关节指令
        """
        joint_move = None
        with self.pose_state_lock:
            self._pending_joint_move = None
            if (
                self.isStop
                or expected_epoch is not self.current_epoch
                or arm_connection_generation != self.arm_connection_generation
            ):
                self.reject_pose_target()
                return ControlComputation()
            self._compute_control_locked(
                end_vr_pose,
                flag_squeeze,
                flag_aButton,
                flag_bButton,
                value_trigger,
                input_sequence,
                session_id,
                params,
                input_source,
                expected_epoch,
                arm_connection_generation,
                arm_feedback_fresh,
                arm_feedback_state_sequence,
                source_token,
            )
            joint_move = self._pending_joint_move
            self._pending_joint_move = None
            # 穿透窗口复核：``publish_epoch`` 无锁，入口校验后纪元可能已被换掉；
            # 纪元已变则本次关节指令属于旧样本，必须丢弃。
            if joint_move is not None and expected_epoch is not self.current_epoch:
                joint_move = None

        return ControlComputation(joint_move=joint_move)

    def dispatch_joint_move_attempt(self, attempt: JointMoveAttempt, admit) -> bool:
        """在 Host 锁外执行一次已准入的关节运动（SDK generation-bound 派发）。

        Args:
            attempt: 已准入的关节运动事务
            admit: SDK 发送前回调，返回 False 时拒绝物理发送
        Returns:
            bool：SDK 派发结果（True 表示已提交），由调用方判定提交语义
        Raises:
            RuntimeError: SDK 适配器缺少 generation-bound 关节派发接口
        """
        sender = getattr(attempt.carm, "dispatch_joint_for_connection", None)
        if not callable(sender):
            raise RuntimeError("Carm SDK adapter lacks generation-bound joint dispatch")

        return sender(
            list(attempt.joints),
            attempt.arm_ws_identity,
            attempt.arm_state_identity,
            admit,
            5,
            False,
        )

    def _compute_control_locked(
        self,
        end_vr_pose,
        flag_squeeze,
        flag_aButton,
        flag_bButton,
        value_trigger,
        input_sequence,
        session_id,
        params,
        input_source,
        expected_epoch,
        arm_connection_generation,
        arm_feedback_fresh,
        arm_feedback_state_sequence,
        source_token,
    ):
        """控制算法计算"""
        if flag_aButton:
            if not flag_squeeze and not self.last_aButton:
                logging.info("[Control] Reset to home")
                if self.carm_:
                    # 纪元戳取入口捕获的 ``expected_epoch``（与位姿路径对齐）：
                    # 取 current_epoch 会让旧样本指令盖上新纪元戳并穿透下发。
                    self._pending_joint_move = JointMoveAttempt(
                        joints=tuple(self._active_init_joint()),
                        epoch=expected_epoch,
                        session_id=session_id,
                        params_epoch=params.epoch,
                        input_source=input_source,
                        arm_connection_generation=self.arm_connection_generation,
                        carm=self.carm_,
                        arm_ws_identity=self._arm_ws_identity,
                        arm_state_identity=self._arm_state_identity,
                    )
                self.last_aButton = True
                self.need_sync_feedback = True
            else:
                logging.debug("请松开控制按键，等待回零点完成")
        else:
            if self.last_aButton:
                if arm_feedback_fresh:
                    self._reset_pose_state(end_vr_pose)
                else:
                    self.invalidate_pose_mapping()
            self.last_aButton = False

        if flag_bButton != self.last_bButton:
            if arm_feedback_fresh:
                self._reset_pose_state(end_vr_pose)
            else:
                self.invalidate_pose_mapping()
            logging.info(
                "[Control] Rotation %s",
                "locked" if flag_bButton else "unlocked",
            )
        self.last_bButton = flag_bButton

        if self.init_end_arm_pose is None or self.need_sync_feedback:
            if flag_squeeze and arm_feedback_fresh:
                self._reset_pose_state(end_vr_pose)
                self.need_sync_feedback = False
                logging.info("[Control] Pose state initialized on squeeze button press")
                return
            else:
                self.reject_pose_target()
                return

        if not flag_squeeze:
            self.reject_pose_target()
            self.relative_quaternion = None
            self.need_sync_feedback = True
            return

        if self.last_bButton:
            delta_pos = (
                end_vr_pose[:3, 3] - self.init_end_vr_pose[:3, 3]
            ) * params.scale
            delta_quat = np.asarray(self.zero_quat, dtype=float)
            relative_quaternion = None
            segment_angle = 0.0
            sample_is_ambiguous = False
        else:
            (
                delta_pos,
                delta_quat,
                relative_quaternion,
                segment_angle,
                sample_is_ambiguous,
            ) = _continuous_pose_diff_with_scale(
                self.init_end_vr_pose,
                end_vr_pose,
                params.scale,
                self.relative_quaternion,
            )

        if sample_is_ambiguous:
            logging.warning(
                "[Control] Dropping ambiguous 180-degree sample or undefined 360-degree branch and rebasing"
            )
            if arm_feedback_fresh:
                self._reset_pose_state(end_vr_pose)
            else:
                self.invalidate_pose_mapping()
            return

        self.relative_quaternion = relative_quaternion
        target_pose = apply_pose_delta(
            self.init_end_arm_pose,
            delta_pos,
            delta_quat,
        )
        if self.end_effector_driver is None:
            raise RuntimeError("end-effector driver is not bound")
        # 每周期一个新 TargetState 对象；其身份取代旧的 target_revision 计数。
        self.current_target = TargetState(
            epoch=expected_epoch,
            pose=tuple(target_pose[:7]),
            input_sequence=input_sequence,
            session_id=session_id,
            params_epoch=params.epoch,
            input_source=input_source,
            arm_connection_generation=arm_connection_generation,
            arm_feedback_state_sequence=arm_feedback_state_sequence,
            source_token=source_token,
            end_effector_trigger=value_trigger,
        )
        self.pending_pose_segment = None
        if (
            not self.last_bButton
            and segment_angle + _POSE_SEGMENT_ANGLE_ATOL_RAD >= _POSE_SEGMENT_ANGLE_RAD
        ):
            self.pending_pose_segment = {
                "input_pose": end_vr_pose.copy(),
                "target_pose": target_pose.copy(),
            }

    def dispatch_pose_attempt(self, attempt: DispatchAttempt):
        """在 Host 锁外执行一次已准入的位姿事务（SDK generation-bound 派发）。

        末端触发值通过 ``send_end_effector`` 随派发一并发送；末端发送失败只记日志，
        不升级为位姿事务失败。

        Args:
            attempt: 已准入的位姿派发事务
        Returns:
            bool：SDK 派发结果；适配器缺少 generation-bound 接口时返回 False
        """
        sender = getattr(attempt.carm, "dispatch_pose_for_connection", None)
        if not callable(sender):
            logging.error(
                "[CommandController] Carm SDK adapter lacks generation-bound dispatch"
            )
            return False

        def send_end_effector() -> None:
            """Best-effort末端发送，不让末端反馈失败升级为位姿事务失败。"""
            try:
                accepted = attempt.end_effector_driver.set_eff(
                    attempt.target_state.end_effector_trigger,
                )
            except Exception:
                logging.exception(
                    "[CommandController] End-effector send raised for %s",
                    attempt.arm_role,
                )
                return
            if accepted is not True:
                logging.warning(
                    "[CommandController] End-effector send was not accepted for %s: %r",
                    attempt.arm_role,
                    accepted,
                )

        return sender(
            list(attempt.target_state.pose),
            attempt.arm_ws_identity,
            send_end_effector,
        )

    def halt_control(self) -> None:
        """停止控制输出并清空目标，不发送任何机械臂运动命令。"""
        with self.pose_state_lock:
            self._publish_new_epoch_locked()
            self.isStop = True
            self.reject_pose_target()
            self.relative_quaternion = None

    def move_home(self) -> bool:
        """手动停止遥操时移动到零点位置。

        Returns:
            bool：未连接、发送失败或异常时为 False，否则为 True（窗口结束不等于已到位）
        """
        try:
            if self.carm_ and self.carm_.is_connected():
                logging.info("[CommandController] Moving to zero position...")
                try:
                    if not self._trigger_joint_motion(self._active_zero_joint()):
                        return False
                    self._refresh_current_pose()
                    logging.info("[CommandController] Home trigger window completed")
                except Exception as e:
                    logging.error(f"[CommandController] Error during stop movement: {e}")
                    raise
                return True
            else:
                logging.warning("[CommandController] Arm not connected, cannot move to zero position")
                return False
        except Exception as e:
            logging.exception(e)
            logging.error(f"[CommandController] Error moving to zero position: {e}")
            return False
