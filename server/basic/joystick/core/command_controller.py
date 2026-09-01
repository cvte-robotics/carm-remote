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
from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, normalize_hand
from .constants import *
from .models import create_pose_data
from .pose_math import (
    _continuous_pose_diff_with_scale,
    _is_valid_joint_position,
    apply_pose_delta,
    project_input_to_arm,
)


@dataclass(frozen=True)
class ControlSample:
    pose_data: Any
    button_data: Any
    calibration: Any
    scale: float
    input_sequence: int
    input_generation: int
    control_generation: int
    input_source: str
    source_lease: Any
    source_token: Any
    source_controller_generation: int
    pose_epoch: int
    arm_connection_generation: int
    arm_feedback_state_sequence: Optional[int]
    arm_feedback_fresh: bool


@dataclass(frozen=True)
class ControlComputation:
    joint_move: Any = None


@dataclass(frozen=True)
class DispatchAttempt:
    target_revision: int
    target_pose: Tuple[float, ...]
    input_sequence: int
    input_generation: int
    control_generation: int
    pose_epoch: int
    input_source: str
    arm_connection_generation: int
    arm_feedback_state_sequence: Optional[int]
    source_token: Any
    source_controller_generation: int
    pending_input_pose: Any
    pending_target_pose: Any
    end_effector_trigger: int
    end_effector_driver: Any
    carm: Any
    arm_ws_identity: Any
    arm_state_identity: Any
    hand_binding: str
    arm_role: str


@dataclass(frozen=True)
class JointMoveAttempt:
    joints: Tuple[float, ...]
    pose_epoch: int
    input_generation: int
    control_generation: int
    input_source: str
    arm_connection_generation: int
    carm: Any
    arm_ws_identity: Any
    arm_state_identity: Any


class CommandController:
    """控制算法核心类"""
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
        self.target_end_arm_pose = None
        self.target_input_sequence = None
        self.target_input_generation = None
        self.target_control_generation = None
        self.target_pose_epoch = None
        self.target_input_source = None
        self.target_arm_connection_generation = None
        self.target_arm_feedback_state_sequence = None
        self.target_source_token = None
        self.target_end_effector_trigger = None
        self.target_revision = 0
        self.relative_quaternion = None
        self.pending_pose_segment = None
        self.pose_state_lock = threading.RLock()
        self.pose_epoch = 0
        self.arm_connection_generation = 0
        self.arm_feedback_state_sequence = 0
        self.arm_feedback_connection_generation = None
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
        self.source_controller_generation = 0

    def _expected_joint_count(self) -> int:
        return 7 if self.use_seven_axis_joints else 6

    def _active_init_joint(self) -> list:
        expected_joint_count = self._expected_joint_count()
        if self.init_joint_ is None:
            raise RuntimeError("未加载机械臂初始化位置配置")
        if len(self.init_joint_) != expected_joint_count:
            raise RuntimeError(
                f"机械臂初始化位置配置长度错误：期望 {expected_joint_count}，实际 {len(self.init_joint_)}"
            )
        return self.init_joint_.copy()

    def _active_zero_joint(self) -> list:
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
        """Extract one finite Cartesian pose from an immutable SDK state snapshot."""
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
        """Accept only a new valid state frame from the current SDK connection."""
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
            return True

    def prepareArm(self, control_host) -> bool:
        """通过控制端点建立机械臂连接并完成运动前准备。"""
        if not ARM_SDK_AVAILABLE or Carm is None:
            logging.error("[CommandController] Pure Python Carm SDK is unavailable")
            return False

        try:
            logging.info(
                "[Arm] Attempting to connect to control_host=%s arm_index=%s...",
                control_host,
                self.carm_device_index,
            )
            self._active_init_joint()

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
        """移动到已配置的初始化位置并等待当前机械臂完成。"""
        if not self.carm_ or not self.carm_.is_connected():
            logging.error("[CommandController] Arm is not connected before initialization movement")
            return False

        logging.info("[Arm] Moving to configured initialization position...")
        try:
            init_joint = self._active_init_joint()
            logging.info(f"[CommandController] Sending move_joint command to position: {init_joint}")
            if self.carm_.move_joint(init_joint, 5, True) is not True:
                logging.error("[CommandController] Initialization move_joint was rejected")
                return False
            self._refresh_current_pose()
            logging.info("[Arm] Initialization movement completed successfully")
            return True
        except Exception as e:
            logging.error(f"[CommandController] Error during initialization movement: {e}")
            logging.exception(e)
            return False

    def reject_pose_target(self) -> None:
        with self.pose_state_lock:
            self.target_revision += 1
            self.target_end_arm_pose = None
            self.target_input_sequence = None
            self.target_input_generation = None
            self.target_control_generation = None
            self.target_pose_epoch = None
            self.target_input_source = None
            self.target_arm_connection_generation = None
            self.target_arm_feedback_state_sequence = None
            self.target_source_token = None
            self.target_end_effector_trigger = None
            self.pending_pose_segment = None

    def invalidate_pose_mapping(self) -> None:
        with self.pose_state_lock:
            self.pose_epoch += 1
            self.reject_pose_target()
            self.relative_quaternion = None
            self.need_sync_feedback = True

    def _reset_pose_state(self, end_vr_pose):
        """Reset the committed input/arm baselines and uncommitted segment state."""
        with self.pose_state_lock:
            self.init_end_vr_pose = end_vr_pose.copy()
            self.init_end_arm_pose = self.end_arm_pose[:7]
            self.reject_pose_target()
            self.relative_quaternion = None

    def _commit_dispatch_attempt_locked(self, attempt: DispatchAttempt) -> None:
        if attempt.pending_input_pose is not None:
            self.init_end_vr_pose = attempt.pending_input_pose.copy()
            self.init_end_arm_pose = attempt.pending_target_pose.copy()
            self.relative_quaternion = None
        self.pending_pose_segment = None

    def _dispatch_attempt_matches_locked(self, attempt: DispatchAttempt) -> bool:
        return bool(
            self.target_revision == attempt.target_revision
            and not self.isStop
            and self.target_input_sequence == attempt.input_sequence
            and self.target_input_generation == attempt.input_generation
            and self.target_control_generation == attempt.control_generation
            and self.target_pose_epoch == attempt.pose_epoch
            and self.target_input_source == attempt.input_source
            and self.target_arm_connection_generation
            == attempt.arm_connection_generation
            and self.target_arm_feedback_state_sequence
            == attempt.arm_feedback_state_sequence
            and self.target_source_token == attempt.source_token
            and self.end_effector_driver is attempt.end_effector_driver
            and self.pose_epoch == attempt.pose_epoch
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
        if (
            self.isStop
            or self.target_end_arm_pose is None
            or self.target_end_effector_trigger is None
            or self.end_effector_driver is None
            or self.target_pose_epoch != self.pose_epoch
            or self.target_arm_connection_generation
            != self.arm_connection_generation
            or self.target_arm_feedback_state_sequence
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
            target_revision=self.target_revision,
            target_pose=tuple(self.target_end_arm_pose[:7]),
            input_sequence=self.target_input_sequence,
            input_generation=self.target_input_generation,
            control_generation=self.target_control_generation,
            pose_epoch=self.target_pose_epoch,
            input_source=self.target_input_source,
            arm_connection_generation=self.target_arm_connection_generation,
            arm_feedback_state_sequence=self.target_arm_feedback_state_sequence,
            source_token=self.target_source_token,
            source_controller_generation=self.source_controller_generation,
            pending_input_pose=(
                pending["input_pose"].copy() if pending is not None else None
            ),
            pending_target_pose=(
                pending["target_pose"].copy() if pending is not None else None
            ),
            end_effector_trigger=self.target_end_effector_trigger,
            end_effector_driver=self.end_effector_driver,
            carm=self.carm_,
            arm_ws_identity=self._arm_ws_identity,
            arm_state_identity=self._arm_state_identity,
            hand_binding=self.hand_binding,
            arm_role=self.arm_role,
        )

    def _transform_input_pose(self, pose_data, calibration):
        pose = [pose_data.x, pose_data.y, pose_data.z]
        quat = [pose_data.qx, pose_data.qy, pose_data.qz, pose_data.qw]
        end_vr_pose = np.eye(4)
        end_vr_pose[:3, 3] = pose
        end_vr_pose[:3, :3] = R.from_quat(quat).as_matrix()
        return project_input_to_arm(end_vr_pose, calibration)

    def _apply_input_rebase_locked(
        self,
        raw_pose,
        calibration,
        button_status,
        arm_feedback_fresh,
    ) -> bool:
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
            pose_data = create_pose_data(
                x=raw_pose[0],
                y=raw_pose[1],
                z=raw_pose[2],
                qx=raw_pose[3],
                qy=raw_pose[4],
                qz=raw_pose[5],
                qw=raw_pose[6],
                timestamp=0,
            )
            end_vr_pose = self._transform_input_pose(pose_data, calibration)
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
        """Apply one normalized host input sample to the control mapping."""
        if not DEPENDENCIES_AVAILABLE:
            self.reject_pose_target()
            return ControlComputation()

        try:
            pose_data = sample.pose_data
            button_status = sample.button_data

            value_trigger = button_status.trigger
            value_squeeze = button_status.grip
            flag_squeeze = value_squeeze > 100
            flag_aButton = button_status.btn_xa
            flag_bButton = button_status.btn_yb

            end_vr_pose = self._transform_input_pose(pose_data, sample.calibration)

            return self.compute_control(
                end_vr_pose,
                flag_squeeze,
                flag_aButton,
                flag_bButton,
                value_trigger,
                sample.input_sequence,
                sample.input_generation,
                sample.control_generation,
                sample.scale,
                sample.input_source,
                sample.pose_epoch,
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
        input_generation,
        control_generation,
        mapping_scale,
        input_source,
        expected_pose_epoch,
        arm_connection_generation,
        arm_feedback_fresh,
        arm_feedback_state_sequence=None,
        source_token=None,
    ):
        """Serialize mutable pose mapping state for one control sample."""
        joint_move = None
        with self.pose_state_lock:
            self._pending_joint_move = None
            if (
                self.isStop
                or expected_pose_epoch != self.pose_epoch
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
                input_generation,
                control_generation,
                mapping_scale,
                input_source,
                expected_pose_epoch,
                arm_connection_generation,
                arm_feedback_fresh,
                arm_feedback_state_sequence,
                source_token,
            )
            joint_move = self._pending_joint_move
            self._pending_joint_move = None

        return ControlComputation(joint_move=joint_move)

    def dispatch_joint_move_attempt(self, attempt: JointMoveAttempt, admit) -> bool:
        """Execute one already-admitted joint movement without Host locks."""
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
        input_generation,
        control_generation,
        mapping_scale,
        input_source,
        expected_pose_epoch,
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
                    self._pending_joint_move = JointMoveAttempt(
                        joints=tuple(self._active_init_joint()),
                        pose_epoch=self.pose_epoch,
                        input_generation=input_generation,
                        control_generation=control_generation,
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
            ) * mapping_scale
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
                mapping_scale,
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
        self.target_revision += 1
        self.target_end_arm_pose = target_pose
        self.target_input_sequence = input_sequence
        self.target_input_generation = input_generation
        self.target_control_generation = control_generation
        self.target_pose_epoch = expected_pose_epoch
        self.target_input_source = input_source
        self.target_arm_connection_generation = arm_connection_generation
        self.target_arm_feedback_state_sequence = arm_feedback_state_sequence
        self.target_source_token = source_token
        if self.end_effector_driver is None:
            raise RuntimeError("end-effector driver is not bound")
        self.target_end_effector_trigger = value_trigger
        self.pending_pose_segment = None
        if (
            not self.last_bButton
            and segment_angle + POSE_SEGMENT_ANGLE_ATOL_RAD >= POSE_SEGMENT_ANGLE_RAD
        ):
            self.pending_pose_segment = {
                "input_pose": end_vr_pose.copy(),
                "target_pose": target_pose.copy(),
            }

    def dispatch_pose_attempt(self, attempt: DispatchAttempt):
        """Run one already-admitted physical transaction without Host locks."""
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
                    attempt.end_effector_trigger,
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
            list(attempt.target_pose),
            attempt.arm_ws_identity,
            send_end_effector,
        )

    def halt_control(self) -> None:
        """停止控制输出并清空目标，不发送任何机械臂运动命令。"""
        with self.pose_state_lock:
            self.pose_epoch += 1
            self.isStop = True
            self.reject_pose_target()
            self.relative_quaternion = None

    def move_home(self) -> bool:
        """手动停止遥操时移动到零点位置。"""
        try:
            if self.carm_ and self.carm_.is_connected():
                logging.info("[CommandController] Moving to zero position...")
                try:
                    if self.carm_.move_joint(
                        self._active_zero_joint(), 5, True
                    ) is not True:
                        logging.error(
                            "[CommandController] Home move_joint was rejected"
                        )
                        return False
                    self._refresh_current_pose()
                    logging.info("[CommandController] Stop command completed, arm reached zero position")
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
