"""手柄输入到机械臂目标的映射与发送。

运行中的映射只由所属臂控制线程修改；连接身份和反馈代次仍在 SDK 发送前检查。
SDK 返回精确 True 后才提交分段基线，不再依赖输入侧失效事务。
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional, List, Tuple

from ..runtime.dependencies import (
    ARM_SDK_AVAILABLE, Carm, MOVE_SPEED_LEVELS,
)
from carm_teleop_input_contracts import (
    ButtonSample,
    HandIntentSample,
    HAND_RIGHT,
    PoseSample,
    normalize_hand,
)
from ..runtime.config import identify_arm_model
from .joint_motion import JointMotionWatch
from .pose_mapping import PoseMapping, select_output_baseline
from .pose_math import _is_valid_joint_position, input_pose_to_arm


# 手臂姿态 1:1 跟随，灵敏度 scale 只缩放位置：姿态只取决于手柄当前位姿，不随路径累积偏差。
_ROTATION_SCALE = 1.0
# 回初始位序号基线未建立（与"身份为 None"区分）。
_HOME_UNSET = object()

DEFAULT_MOVE_SPEED_LEVEL = "medium"
# move_joint 的名义窗口，只防止与 track_pose 重叠，不是到位判定。
MOVE_JOINT_WINDOW_S = 5.0


@dataclass(frozen=True)
class ControlParams:
    """Host 发布的冻结输入映射参数。

    ``epoch`` 只在 yaw 标定或灵敏度 scale 变化时推进。控制线程在周期开始按
    对象身份接收变化，整个周期使用同一份参数。
    """

    epoch: int
    scale: float
    calibration: Any


@dataclass(frozen=True)
class ControlSample:
    """
一个控制周期冻结的输入样本（本周期位姿/按钮/功能结果/参数的完整凭证）。

``params`` 为本周期冻结的映射参数，``arm_connection_generation``/
``arm_feedback_state_sequence``/``arm_feedback_fresh`` 为机械臂反馈身份；
``home_identity`` 标识回初始位序号所属的输入（会话与 lease）。
"""

    pose_data: Optional[PoseSample]
    button_data: Optional[ButtonSample]
    params: ControlParams
    arm_connection_generation: int
    arm_feedback_state_sequence: Optional[int]
    arm_feedback_fresh: bool
    intent_data: Optional[HandIntentSample] = None
    home_identity: Any = None


@dataclass(frozen=True)
class ControlComputation:
    """一次控制计算的产物；``joint_move`` 非 None 时表示有待派发的关节指令。"""

    joint_move: Any = None


@dataclass(frozen=True)
class JointMoveAttempt:
    """A 键关节运动的命令与臂连接快照。"""

    joints: Tuple[float, ...]
    arm_connection_generation: int
    carm: Any
    arm_ws_identity: Any


class CommandController:
    """单臂控制算法核心（输入位姿 → 目标 pose / 关节指令）。

    绑定一只 hand 与一个 arm_role，持有该臂的位姿映射状态与机械臂连接身份。
    运行中的位姿映射由所属控制线程独占，SDK 派发保留连接身份保护。
    生命周期线程只能在控制线程退出后执行停止收尾。
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
        self.move_speed = MOVE_SPEED_LEVELS[DEFAULT_MOVE_SPEED_LEVEL]
        self.hand_binding = normalize_hand(hand_binding, HAND_RIGHT)
        self.arm_role = arm_role
        self.default_init_joint = [-0.000192, 1.56713, -0.956551, 0.018883, 0.907339, -0.004005]
        self.init_joint_ = (
            list(configured_init_joint)
            if configured_init_joint is not None
            else self.default_init_joint.copy()
        )

        self.zero_joint = [0,0,0,0,0,0]
        self.seven_axis_zero_joint = [0,0,0,0,0,0,0]
        # 由会话按配置注入；None 时分别回退为全零与 init_joint_。
        self.home_joint_ = None
        self.teleop_reset_joint_ = None

        self.mapping = PoseMapping()
        # 最近一次成功下发的目标；连接变化或关节运动后失效。
        self._last_committed_target = None
        # 最近接受的反馈帧里控制器是否在执行任务。
        self._controller_tracking = False
        # 目标只在本臂控制线程内短暂存在；字段值为普通映射，包含派发所需的
        # 位姿、反馈身份和末端触发值。
        self.current_target = None
        self.arm_connection_generation = 0
        self.arm_feedback_state_sequence = 0
        self.arm_feedback_connection_generation = None
        self.arm_feedback_ws_identity = None
        self.arm_feedback_state_identity = None
        self.arm_feedback_accept_monotonic = None
        self._arm_ws_identity = None
        self._arm_state_identity = None
        self._arm_connected = False

        self._home_identity = _HOME_UNSET
        self._home_sequence = None
        self.last_bButton = False
        self.need_sync_feedback = False
        self.carm_ = None
        # 共享 SDK 连接时由主控制器注入通道工厂；None 时每臂独立 Carm 连接。
        self.carm_factory = None
        self._pending_joint_move = None
        # 进行中的回位运动及其所属臂连接代次；结束前不建基线、不发跟随命令。
        self._home_motion = None
        self._home_motion_generation = None
        # 最近一次触发的初始化/回零运动的窗口截止时刻。
        self._motion_deadline = None
        # 最近一次 track_pose 成功写入连接的时刻；臂型角据此判断手臂是否在跟随。
        self._last_pose_dispatch = None
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
        """返回当前生效的回零位置副本；未配置时为当前臂型的全零关节值。"""
        if self.home_joint_ is not None:
            return self._validated_joint_copy(self.home_joint_, "回零位置")
        if self._expected_joint_count() == 7:
            return self.seven_axis_zero_joint.copy()
        return self.zero_joint.copy()

    def _active_teleop_reset_joint(self) -> list:
        """返回遥操中回初始位的目标副本；未配置时与初始化位置相同。"""
        if self.teleop_reset_joint_ is not None:
            return self._validated_joint_copy(self.teleop_reset_joint_, "遥操回初始位位置")
        return self._active_init_joint()

    def _validated_joint_copy(self, joints, label: str) -> list:
        """校验关节值长度与当前臂型一致后返回副本。"""
        expected_joint_count = self._expected_joint_count()
        if len(joints) != expected_joint_count:
            raise RuntimeError(
                f"机械臂{label}配置长度错误：期望 {expected_joint_count}，实际 {len(joints)}"
            )
        return list(joints)

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
        并调用 ``invalidate_pose_mapping()``，要求下一输入从新鲜反馈重新同步。

        Returns:
            bool：接受了新的合法反馈帧并刷新 ``end_arm_pose`` 时为 True；
            无新帧、帧非法或连接未就绪时为 False
        """
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
            self._on_arm_connection_changed()

        if not connected:
            if self._arm_connected:
                self._arm_connected = False
                self._arm_state_identity = state_identity
                self._on_arm_connection_changed()
            return False

        if not self._arm_connected:
            self._arm_connected = True
            self._arm_state_identity = state_identity
            self._on_arm_connection_changed()
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
        task = state_identity["arm"][arm_index].get("task")
        self._controller_tracking = isinstance(task, dict) and task.get("exe_flag") is True
        self.arm_feedback_connection_generation = self.arm_connection_generation
        # 接受时刻快照：记录本帧被接受时的 ws/state 身份与时钟；须与关节值
        # 配对使用，改读实时值会把旧反馈绑到重连后的新连接上。
        self.arm_feedback_ws_identity = ws_identity
        self.arm_feedback_state_identity = state_identity
        self.arm_feedback_accept_monotonic = time.monotonic()
        return True

    def _on_arm_connection_changed(self) -> None:
        """连接事件：推进代次并作废映射；新连接上不能沿用旧目标。"""
        self.arm_feedback_connection_generation = None
        self.arm_connection_generation += 1
        self.invalidate_pose_mapping()
        self._last_committed_target = None
        self._controller_tracking = False

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
                    factory = self.carm_factory or Carm
                    if self.carm_device_index is None:
                        self.carm_ = factory(control_host)
                    else:
                        self.carm_ = factory(control_host, self.carm_device_index)
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

    def prepareArm(self) -> bool:
        """
校验初始化配置后使能机械臂（``set_ready`` + 控制模式 2 并注册错误回调）。

Returns:
    使能准备全部成功为 True；配置非法、已取消或异常时为 False。
"""
        try:
            self._active_init_joint()
            self._active_zero_joint()
            self._active_teleop_reset_joint()
            if self.motion_cancel_event.is_set():
                return False
            logging.info("[Arm] Connected. Enabling servo...")
            self.carm_.set_ready()
            time.sleep(0.2)

            logging.info("[Arm] Setting control mode...")
            self.carm_.set_control_mode(2)
            time.sleep(0.2)

            self.carm_.on_error(lambda msg: logging.error(f"[Arm Error] {msg}"))

            logging.info("[Arm] Initialization movement preparation completed")
            return True
        except Exception as e:
            error_msg = f"[Arm] Initialization preparation failed: {e}"
            logging.error(error_msg)
            logging.exception(e)
            return False

    def trigger_init_motion(self) -> bool:
        """发送初始化关节运动，不等待名义窗口。"""
        return self._trigger_joint_motion(self._active_init_joint())

    def _trigger_joint_motion(self, joints, disconnected_log_level=logging.ERROR) -> bool:
        """取消或断连时拒绝发送，否则触发非阻塞关节运动。"""
        if self.motion_cancel_event.is_set():
            return False
        if not self.carm_ or not self.carm_.is_connected():
            logging.log(
                disconnected_log_level,
                "[CommandController] Arm not connected before joint movement for role=%s",
                self.arm_role,
            )
            return False
        self.carm_.move_joint(joints, -1, False, 0, speed=self.move_speed)
        self._last_committed_target = None
        self._motion_deadline = time.monotonic() + MOVE_JOINT_WINDOW_S
        return True

    @staticmethod
    def wait_joint_motion_window(controllers: List["CommandController"]) -> List["CommandController"]:
        """等到最后触发的臂满一个名义窗口，返回窗口完成且已尝试刷新位姿的臂。

        窗口从触发算起，调用方在触发后做的其他等待（如腰的窗口）计入其中。所有臂共用
        Host 的取消事件；任一断连使整个窗口失败，不判断 SDK 返回或到位。位姿刷新不要求
        新反馈到达，异常则只让对应臂失败。
        """
        if not controllers:
            return []
        cancel_event = controllers[0].motion_cancel_event
        deadline = max(controller._motion_deadline for controller in controllers)

        def any_disconnected():
            return any(not controller.carm_.is_connected() for controller in controllers)

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if cancel_event.is_set():
                    return []
                # 窗口可能已被调用方的其他等待占满，循环体一次没跑，这里仍要确认连接。
                if any_disconnected():
                    cancel_event.set()
                    return []
                break
            if cancel_event.wait(min(0.05, remaining)):
                return []
            if any_disconnected():
                cancel_event.set()
                return []

        completed = []
        for controller in controllers:
            try:
                controller._refresh_current_pose()
                completed.append(controller)
            except Exception:
                logging.exception("[CommandController] Motion pose refresh failed for role=%s", controller.arm_role)
        return completed

    def reject_pose_target(self) -> None:
        """清空当前目标。"""
        self.current_target = None

    def invalidate_pose_mapping(self) -> None:
        """作废位姿映射：清目标、丢弃四元数连续分支并要求重新同步反馈。

        由控制线程在臂连接事件、配置变更、输入断续与发送失败时调用。
        """
        self.reject_pose_target()
        self.mapping.reset_branch()
        self.need_sync_feedback = True

    def _reset_pose_state(self, end_vr_pose, arm_feedback_fresh) -> bool:
        """以本周期输入位姿重建基线，输出基线按 ``select_output_baseline`` 选择。

        Args:
            end_vr_pose: 本周期的输入位姿（Pose4 元组）
            arm_feedback_fresh: 本周期是否接受了新的反馈帧
        Returns:
            已重建为 True；既没有可用目标也没有新鲜反馈为 False，调用方等下一帧
        """
        self.reject_pose_target()
        output = select_output_baseline(
            self._last_committed_target, self._controller_tracking,
            self.end_arm_pose if arm_feedback_fresh else None,
        )
        if output is None:
            return False
        self.mapping.rebase(end_vr_pose, output)
        return True

    @staticmethod
    def _transform_input_pose(pose: PoseSample, calibration):
        """把 PoseSample 投影到机械臂坐标系。"""
        return input_pose_to_arm(
            (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw), calibration,
        )

    def _apply_input_rebase(
        self,
        raw_pose,
        calibration,
        intent,
        arm_feedback_fresh,
    ) -> bool:
        """
在控制线程内应用一次 Quest 输入 rebase（重建基线）。

Args:
    raw_pose: provider 送达的 7 分量原始输入位姿。
    calibration: 输入投影标定。
    intent: 本周期功能结果（用于同步旋转锁）。
    arm_feedback_fresh: 本周期是否接受了新的反馈帧。

Returns:
    rebase 提交成功为 True；转换异常或前置条件不满足为 False。
"""
        if (
            self.isStop
            or not self.carm_
            or not self._arm_connected
            or self.arm_feedback_connection_generation != self.arm_connection_generation
            or not self.carm_.is_connected()
            or self.carm_.ws is not self._arm_ws_identity
        ):
            self.reject_pose_target()
            return False

        try:
            end_vr_pose = input_pose_to_arm(raw_pose[:7], calibration)
        except Exception as e:
            self.reject_pose_target()
            logging.warning("[CommandController] Quest rebase pose conversion failed: %s", e)
            return False

        if not self._reset_pose_state(end_vr_pose, arm_feedback_fresh):
            return False
        self.need_sync_feedback = False
        self.last_bButton = intent.rotation_locked
        return True

    def process_input_sample(self, sample: ControlSample) -> ControlComputation:
        """把一个已归一化的 Host 输入样本应用到控制映射。

        Args:
            sample: 控制线程冻结的本周期输入样本
        Returns:
            ControlComputation，joint_move 非 None 时表示有待派发的关节指令；
            计算异常时返回空计算结果
        """
        try:
            pose_sample = sample.pose_data
            intent = sample.intent_data
            if intent is None:
                self.reject_pose_target()
                return ControlComputation()
            value_trigger = intent.end_effector
            flag_squeeze = intent.motion_engaged
            home_sequence = intent.home_sequence
            flag_bButton = intent.rotation_locked

            end_vr_pose = self._transform_input_pose(pose_sample, sample.params.calibration)

            return self.compute_control(
                end_vr_pose,
                flag_squeeze,
                home_sequence,
                flag_bButton,
                value_trigger,
                sample.params,
                sample.arm_connection_generation,
                sample.arm_feedback_fresh,
                sample.arm_feedback_state_sequence,
                sample.home_identity,
            )
        except Exception as e:
            self.invalidate_pose_mapping()
            self.discard_home_intent(sample.intent_data, sample.home_identity, "input_processing_failed")
            logging.exception(e)
            logging.error(f"[CommandController] Callback error: {e}")
            return ControlComputation()

    def reset_home_baseline(self) -> None:
        """新控制事务开始时调用：下一个样本只建立回初始位序号基线，恢复后不补发。"""
        self._home_identity = _HOME_UNSET
        self._home_sequence = None

    def discard_home_intent(self, intent, identity, reason: str) -> None:
        """计算前退出时消费已有功能序号；旧身份只建基线，不能产生指令。"""
        if intent is None:
            return
        self._consume_home_sequence(
            intent.home_sequence, identity, intent.motion_engaged, discard_reason=reason,
        )

    def compute_control(
        self,
        end_vr_pose,
        flag_squeeze,
        home_sequence,
        flag_bButton,
        value_trigger,
        params,
        arm_connection_generation,
        arm_feedback_fresh,
        arm_feedback_state_sequence=None,
        home_identity=None,
    ):
        """在所属控制线程内应用一个控制样本并生成待派发目标。

        Args:
            end_vr_pose: 本周期的输入位姿（Pose4 元组）
            flag_squeeze / home_sequence / flag_bButton: 本周期功能结果（运动、回初始位序号、旋转锁）
            value_trigger: 末端触发值
            params: 本周期冻结的映射参数
            arm_connection_generation: 计算时的臂连接代次
            arm_feedback_fresh: 本周期是否接受了新的反馈帧
            arm_feedback_state_sequence: 冻结的反馈帧序号
            home_identity: 回初始位序号所属的输入身份；变化时首样本只建基线
        Returns:
            ControlComputation，joint_move 非 None 时表示有待派发的关节指令
        """
        joint_move = None
        self._pending_joint_move = None
        if self.isStop or arm_connection_generation != self.arm_connection_generation:
            self._consume_home_sequence(
                home_sequence, home_identity, flag_squeeze,
                discard_reason="control_identity_invalid_or_stopped",
            )
            self.reject_pose_target()
            return ControlComputation()
        home_requested = self._consume_home_sequence(home_sequence, home_identity, flag_squeeze)
        self._compute_control(
            end_vr_pose,
            flag_squeeze,
            home_requested,
            flag_bButton,
            value_trigger,
            params,
            arm_connection_generation,
            arm_feedback_fresh,
            arm_feedback_state_sequence,
        )
        joint_move = self._pending_joint_move
        self._pending_joint_move = None

        return ControlComputation(joint_move=joint_move)

    def dispatch_joint_move_attempt(self, attempt: JointMoveAttempt, admit) -> bool:
        """在 Host 锁外执行一次已准入的关节运动（SDK generation-bound 派发）。

        Args:
            attempt: 已准入的关节运动事务
            admit: SDK 发送前回调，返回 False 时拒绝物理发送
        Returns:
            bool：SDK 派发结果（True 表示已提交），由调用方判定提交语义
        """
        return attempt.carm.dispatch_joint_for_connection(
            list(attempt.joints),
            attempt.arm_ws_identity,
            admit,
            self.move_speed,
            False,
        )

    def begin_home_motion(self, attempt: JointMoveAttempt) -> None:
        """记录已写入连接的回位运动，之后由 ``home_motion_pending`` 判定结束。"""
        self._home_motion = JointMotionWatch(
            attempt.carm, attempt.carm.last_motion_task_key, attempt.joints,
        )
        self._home_motion_generation = attempt.arm_connection_generation
        self._last_committed_target = None

    def home_motion_pending(self, arm_feedback_fresh: bool) -> bool:
        """回位运动是否仍在进行；未到位时告警并允许从当前位置接管。"""
        watch = self._home_motion
        if watch is None:
            return False
        if self.arm_connection_generation != self._home_motion_generation:
            logging.warning("[Control] Home motion abandoned on connection change: role=%s", self.arm_role)
            self._home_motion = None
            return False
        if not arm_feedback_fresh:
            return True
        carm = watch.carm
        arrived = watch.poll(
            time.monotonic(), carm.joint_pos, carm.joint_vel,
            carm._arm_state.get("task", {}).get("exe_flag", False),
        )
        if arrived is None:
            return True
        self._home_motion = None
        if arrived:
            logging.info("[Control] Home motion finished: role=%s", self.arm_role)
        else:
            logging.warning(
                "[Control] Home motion ended short of target: role=%s joints=%s target=%s",
                self.arm_role, carm.joint_pos, watch.target,
            )
        return False

    def _consume_home_sequence(self, sequence, identity, motion_engaged, discard_reason=None):
        """先消费事件再判断运动，防止退出运动后补发；身份变化首样本只建基线。"""
        changed = self._home_identity is _HOME_UNSET or self._home_identity != identity
        requested = not changed and sequence > self._home_sequence
        self._home_identity = identity
        self._home_sequence = sequence
        if discard_reason is not None:
            if requested or (changed and sequence > 0):
                logging.info("[Control] Dropped home intent: %s (sequence=%s)", discard_reason, sequence)
            return False
        if requested and motion_engaged:
            logging.info("[Control] Dropped home intent during motion control")
            return False
        return requested

    def _compute_control(
        self,
        end_vr_pose,
        flag_squeeze,
        home_requested,
        flag_bButton,
        value_trigger,
        params,
        arm_connection_generation,
        arm_feedback_fresh,
        arm_feedback_state_sequence,
    ):
        """控制算法计算"""
        if home_requested:
            logging.info("[Control] Reset to home")
            if self.carm_:
                self._pending_joint_move = JointMoveAttempt(
                    joints=tuple(self._active_teleop_reset_joint()),
                    arm_connection_generation=self.arm_connection_generation,
                    carm=self.carm_,
                    arm_ws_identity=self._arm_ws_identity,
                )
            # A 松开时不再重置；此标记保证下次运动仍必须用新鲜反馈对齐。
            self.need_sync_feedback = True

        if flag_bButton != self.last_bButton:
            if not self._reset_pose_state(end_vr_pose, arm_feedback_fresh):
                self.invalidate_pose_mapping()
            logging.info(
                "[Control] Rotation %s",
                "locked" if flag_bButton else "unlocked",
            )
        self.last_bButton = flag_bButton

        if not self.mapping.ready or self.need_sync_feedback:
            if flag_squeeze and self._reset_pose_state(end_vr_pose, arm_feedback_fresh):
                self.need_sync_feedback = False
                logging.info("[Control] Pose state initialized on squeeze button press")
                return
            self.reject_pose_target()
            return

        if not flag_squeeze:
            self.reject_pose_target()
            self.mapping.reset_branch()
            self.need_sync_feedback = True
            return

        target_pose = self.mapping.target(end_vr_pose, params.scale, _ROTATION_SCALE, self.last_bButton)
        if target_pose is None:
            logging.warning(
                "[Control] Dropping ambiguous 180-degree sample or undefined 360-degree branch and rebasing"
            )
            if not self._reset_pose_state(end_vr_pose, arm_feedback_fresh):
                self.invalidate_pose_mapping()
            return

        if self.end_effector_driver is None:
            raise RuntimeError("end-effector driver is not bound")
        self.current_target = {
            "pose": target_pose,
            "arm_connection_generation": arm_connection_generation,
            "arm_feedback_state_sequence": arm_feedback_state_sequence,
            "end_effector_trigger": value_trigger,
        }

    def dispatch_pose(self):
        """发送控制线程计算的目标；基线在整个握持期间固定，成功后无待提交状态。

        不要求 ``carm.state`` 仍是周期开头的反馈帧：目标只由固定基线与输入增量算出，
        SDK 读线程在计算期间替换状态帧属正常情况，不能当作发送失败而作废映射。
        """
        target = self.current_target
        carm = self.carm_
        if (
            self.isStop or target is None or self.end_effector_driver is None
            or carm is None or not carm.is_connected() or not self._arm_connected
            or target["arm_connection_generation"] != self.arm_connection_generation
            or target["arm_feedback_state_sequence"] != self.arm_feedback_state_sequence
            or self.arm_feedback_connection_generation != self.arm_connection_generation
            or carm.ws is not self._arm_ws_identity
        ):
            return False

        def send_end_effector() -> None:
            """位姿成功后的末端发送；失败由 EndEffectorSender 记录。"""
            self.end_effector_driver.set_eff(target["end_effector_trigger"])

        result = carm.dispatch_pose_for_connection(
            list(target["pose"]), self._arm_ws_identity, send_end_effector,
        )
        if result is True:
            self._last_pose_dispatch = time.monotonic()
            self._last_committed_target = target["pose"]
        return result

    def pose_dispatched_within(self, seconds: float) -> bool:
        """最近 ``seconds`` 秒内是否有 track_pose 成功写入连接（无锁读，供臂型角判断跟随）。"""
        last = self._last_pose_dispatch
        return last is not None and time.monotonic() - last <= seconds

    def halt_control(self) -> None:
        """停止控制输出并清空目标，不发送任何机械臂运动命令。"""
        self.isStop = True
        self.reject_pose_target()
        self.mapping.reset_branch()

    def trigger_home_motion(self) -> bool:
        """发送回零关节运动，不等待名义窗口。"""
        return self._trigger_joint_motion(
            self._active_zero_joint(), disconnected_log_level=logging.WARNING,
        )
