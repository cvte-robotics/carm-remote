"""遥操作 Host 与各输入源模块共享的中立契约（不含任何 provider 依赖）。

只定义数据类型、policy 协议、事件 sink 与模块协议；跨线程传递对象一律 frozen
dataclass 并在 ``__post_init__`` 校验。领域术语保持英文原词：lease、epoch、token、
frame、jump、rebase、policy、pose、TCP、ADB、SSE。
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import FrozenSet, Mapping, Optional, Protocol, Tuple, Union


HAND_LEFT = "left"
HAND_RIGHT = "right"
VALID_HANDS = frozenset((HAND_LEFT, HAND_RIGHT))

ScalarValue = Optional[Union[bool, int, float, str]]


def normalize_hand(hand: str, default: str = HAND_RIGHT) -> str:
    """把外部传入的 hand 文本归一化为 ``left``/``right``；无法识别时返回 ``default``。

    Args:
        hand: 原始 hand 文本（大小写/空白不限，可为 None）。
        default: 无法识别时返回的 hand。
    """
    normalized = (hand or "").strip().lower()
    return normalized if normalized in VALID_HANDS else default


def opposite_hand(hand: str) -> str:
    """返回另一只手的 hand 名；非法 hand 抛 ValueError。"""
    if hand == HAND_LEFT:
        return HAND_RIGHT
    if hand == HAND_RIGHT:
        return HAND_LEFT
    raise ValueError(f"Unsupported hand: {hand}")


@dataclass(frozen=True)
class SourceCapabilities:
    """provider 的静态能力声明；Host 据此决定调用哪些可选生命周期缝。

    能力位在模块实例生命周期内固定不变，不声明即视为不具备；Host 只用它做分支
    选择，不再做运行期方法探测（可选缝由 ``BaseInputSourceModule`` 兜底）。
    """

    # 提供 PoseSample（Cartesian 控制路径）。
    provides_pose: bool
    # 提供 ButtonSample。
    provides_buttons: bool
    # 可绑定 controller 参与机械臂控制。
    can_control_arms: bool
    # 仅建立连接，不参与控制。
    connect_only: bool
    # 断链后允许进入重连宽限期，而不是立即终结会话。
    supports_reconnect_grace: bool
    # 以关节目标（JointTargetSample）驱动，而不是 Cartesian pose。
    provides_joint_targets: bool = False
    # 支持左右手各自切换绑定/解绑，而不是双手中立。
    supports_per_arm_switch: bool = False
    # 该 provider 参与 Host 机械臂会话配置（mode/master_side/另一臂 IP）。
    # 不声明时 Web 不显示 Host 会话配置表单，连接前也不做 setSessionConfig 同步。
    configures_arm_session: bool = False
    # 提供可观察的再接入许可；状态尚未产生时仍可能返回 None。
    supports_reengagement: bool = False
    # 每个输入帧最多派发一次（provider 自带帧屏障），Host 控制环可在新帧发布时
    # 立即唤醒；不声明时控制环按固定节拍运行并重复使用最新帧。
    input_clocked_control: bool = False

    def __post_init__(self) -> None:
        values = (
            self.provides_pose,
            self.provides_buttons,
            self.can_control_arms,
            self.connect_only,
            self.supports_reconnect_grace,
            self.provides_joint_targets,
            self.supports_per_arm_switch,
            self.configures_arm_session,
            self.supports_reengagement,
            self.input_clocked_control,
        )
        if any(not isinstance(value, bool) for value in values):
            raise ValueError("source capability fields must be booleans")


@dataclass(frozen=True)
class SourceAvailability:
    """一次 ``availability()`` 查询的结果：是否可用、原因与能力声明。

    ``available`` 为 False 时 ``reason`` 应是可直接展示给用户的文案。
    """

    available: bool
    reason: Optional[str]
    capabilities: SourceCapabilities

    def __post_init__(self) -> None:
        if not isinstance(self.available, bool):
            raise ValueError("source availability must be a boolean")
        if self.reason is not None and not isinstance(self.reason, str):
            raise ValueError("source availability reason must be a string or None")
        if not isinstance(self.capabilities, SourceCapabilities):
            raise ValueError("source availability capabilities are required")


@dataclass(frozen=True)
class DeviceCandidate:
    """扫描或解析得到的一个待连接设备候选。

    ``provider_id`` 匹配模块，``public_source_type`` 是 Web 侧展示/路由用的源类型；
    ``address`` 是 provider 自己的寻址串（Quest 为 ADB serial）。
    """

    provider_id: str
    public_source_type: str
    address: str
    name: str
    capabilities: SourceCapabilities

    def __post_init__(self) -> None:
        if not self.provider_id or not isinstance(self.provider_id, str):
            raise ValueError("candidate provider_id is required")
        if not self.public_source_type or not isinstance(self.public_source_type, str):
            raise ValueError("candidate public_source_type is required")
        if not isinstance(self.address, str) or not isinstance(self.name, str):
            raise ValueError("candidate address and name must be strings")
        if not isinstance(self.capabilities, SourceCapabilities):
            raise ValueError("candidate capabilities are required")


@dataclass(frozen=True)
class SourceLease:
    """一次输入源连接的不可变身份（lease）。

    同一 lease 贯穿 connect → 输入 batch → transport event → close。
    ``module_instance_id`` 区分同一 provider 的多次实例化，``input_generation``
    每次重新绑定输入时递增，二者共同用于丢弃旧 session 的回调与事件。
    """

    provider_id: str
    module_instance_id: int
    input_generation: int

    def __post_init__(self) -> None:
        if not isinstance(self.provider_id, str) or not self.provider_id:
            raise ValueError("lease provider_id is required")
        for name, value in (
            ("module_instance_id", self.module_instance_id),
            ("input_generation", self.input_generation),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"lease {name} must be a positive integer")


class ConnectOutcome(Enum):
    """一次 ``connect()`` 的三种结果。"""

    READY = "ready"
    PENDING = "pending"
    FAILED = "failed"


class ControlAction(Enum):
    """单次输入驱动控制采样的中立决策。

    READY/REBASE/REJECT 服务于 Cartesian Host 路径；TRACK_JOINT/MOVE_JOINT 是显式
    命令种类，Host 按命令种类派发，不需要知道 provider 名称。
    """

    READY = "ready"
    REBASE = "rebase"
    REJECT = "reject"
    TRACK_JOINT = "track_joint"
    MOVE_JOINT = "move_joint"


class ControlOutcome(Enum):
    """Host 完成一次控制尝试后回报的事实。"""

    REBASE_APPLIED = "rebase_applied"
    DISPATCH_COMMITTED = "dispatch_committed"
    DISPATCH_REJECTED = "dispatch_rejected"
    STALE = "stale"


@dataclass(frozen=True)
class RebasePayload:
    """通用 Cartesian rebase 数据；token 身份在此保持不透明。"""

    raw_pose: Tuple[float, ...]
    reason: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.raw_pose, tuple) or len(self.raw_pose) < 7:
            raise ValueError("rebase payload pose must contain at least 7 values")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in self.raw_pose
        ):
            raise ValueError("rebase payload pose must be finite numbers")
        if not isinstance(self.reason, str):
            raise ValueError("rebase payload reason must be a string")


@dataclass(frozen=True)
class JointCommand:
    """由输入源 policy 产出的、可被 Host 直接执行的单条关节命令。"""

    joints: Tuple[float, ...]
    duration_s: Optional[float] = None
    end_effector_trigger: Optional[int] = None

    def __post_init__(self) -> None:
        if not isinstance(self.joints, tuple) or not self.joints:
            raise ValueError("joint command joints must be a non-empty tuple")
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in self.joints
        ):
            raise ValueError("joint command joints must be finite numbers")
        if self.duration_s is not None and (
            isinstance(self.duration_s, bool)
            or not isinstance(self.duration_s, (int, float))
            or not math.isfinite(self.duration_s)
            or self.duration_s < 0
        ):
            raise ValueError("joint command duration must be a non-negative finite number")
        if self.end_effector_trigger is not None and (
            isinstance(self.end_effector_trigger, bool)
            or not isinstance(self.end_effector_trigger, int)
            or not 0 <= self.end_effector_trigger <= 127
        ):
            raise ValueError("end_effector_trigger must be an integer in 0..127")


@dataclass(frozen=True)
class ReengageExceeded:
    """一次再接入超限的不可变观察。"""

    position_delta_m: float
    rotation_delta_deg: float

    def __post_init__(self) -> None:
        for value in (self.position_delta_m, self.rotation_delta_deg):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError("reengagement deltas must be non-negative finite numbers")

    def as_projection(self) -> dict:
        return {
            "position_delta_m": self.position_delta_m,
            "rotation_delta_deg": self.rotation_delta_deg,
        }


@dataclass(frozen=True)
class ReengageStatus:
    """单手再接入观察；None 表示尚无观察，不等同于拒绝。"""

    allowed: bool
    reason: Optional[str]
    position_threshold_m: Optional[float]
    rotation_threshold_deg: Optional[float]
    exceeded: Tuple[ReengageExceeded, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.allowed, bool):
            raise ValueError("reengagement allowed must be a boolean")
        if self.reason is not None and not isinstance(self.reason, str):
            raise ValueError("reengagement reason must be a string or None")
        for value in (self.position_threshold_m, self.rotation_threshold_deg):
            if value is not None and (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError("reengagement thresholds must be non-negative finite numbers or None")
        if not isinstance(self.exceeded, tuple) or any(
            not isinstance(item, ReengageExceeded) for item in self.exceeded
        ):
            raise ValueError("reengagement exceeded must be a tuple of ReengageExceeded")

    def as_projection(self) -> dict:
        """保持现有 HTTP/SSE joint.reengage JSON 形状。"""
        return {
            "allowed": self.allowed,
            "reason": self.reason,
            "position_threshold_m": self.position_threshold_m,
            "rotation_threshold_deg": self.rotation_threshold_deg,
            "exceeded": [item.as_projection() for item in self.exceeded],
        }


@dataclass(frozen=True)
class ArmProfile:
    """可选的拓扑/型号约束；模块上为 None 表示不附加额外限制。"""

    models: FrozenSet[str] = frozenset()
    dof: Optional[int] = None
    modes: FrozenSet[str] = frozenset()
    require_same_board_dual: bool = False
    device_index_by_hand: Mapping[str, int] = None

    def __post_init__(self) -> None:
        models = self.models if isinstance(self.models, frozenset) else frozenset(self.models or ())
        modes = self.modes if isinstance(self.modes, frozenset) else frozenset(self.modes or ())
        object.__setattr__(self, "models", models)
        object.__setattr__(self, "modes", modes)
        if self.dof is not None and (isinstance(self.dof, bool) or not isinstance(self.dof, int) or self.dof <= 0):
            raise ValueError("arm profile dof must be a positive integer")
        if not isinstance(self.require_same_board_dual, bool):
            raise ValueError("require_same_board_dual must be a boolean")
        mapping = self.device_index_by_hand or {}
        if not isinstance(mapping, Mapping):
            raise ValueError("device_index_by_hand must be a mapping")
        frozen = {}
        for hand, index in mapping.items():
            if hand not in VALID_HANDS:
                raise ValueError(f"unsupported hand in arm profile: {hand}")
            if isinstance(index, bool) or not isinstance(index, int) or index < 0:
                raise ValueError("device indexes must be non-negative integers")
            frozen[hand] = index
        object.__setattr__(self, "device_index_by_hand", frozen)

    def as_dict(self):
        """导出为可序列化 dict（集合排序，映射转普通 dict）。"""
        return {
            "models": sorted(self.models),
            "dof": self.dof,
            "modes": sorted(self.modes),
            "require_same_board_dual": self.require_same_board_dual,
            "device_index_by_hand": dict(self.device_index_by_hand),
        }


@dataclass(frozen=True)
class ControlDecision:
    """policy 对单次控制采样的决策结果。

    ``source_token`` 是该决策绑定的输入帧身份，Host 只原样冻结并在
    validate_dispatch/complete_control 时回传，不解释其内容；``rebase_payload``
    与 ``command`` 分别对应 REBASE 与关节命令两种动作。
    """

    action: ControlAction
    source_token: Optional[object]
    reason: str = ""
    rebase_payload: Optional[object] = None
    command: Optional[JointCommand] = None


@dataclass(frozen=True)
class ControlResult:
    """Host 回报控制结果后，policy 复核得到的最终结果。"""

    outcome: ControlOutcome
    source_token: Optional[object]
    reason: str = ""


@dataclass(frozen=True)
class ControlContext:
    """
Host 为单次 policy 决策冻结的样本。

同一个 control_policy 实例的 decide_control/validate_dispatch/complete_control
须串行化（Host 用自己的锁维持）；运动学 RPC 不得在持有 Host 锁时发起。
"""

    hand: str
    controller: object
    # Host 会话身份（``ActiveSourceBinding.session_id``）。
    session_id: int
    # ``ControlParams.epoch``：yaw/scale 参数的发布 epoch。其它失效事件都由冻结的
    # binding 身份或 Host 控制 epoch 承载，因此不需要单独的 controller generation。
    params_epoch: int
    bound_active: bool
    need_sync_feedback: bool
    joint_target: Optional["JointTargetSample"] = None
    feedback_joints: Optional[Tuple[float, ...]] = None
    feedback_age_s: Optional[float] = None
    now_monotonic: Optional[float] = None
    init_joint: Optional[Tuple[float, ...]] = None
    plan_pose: Optional[Tuple[float, ...]] = None
    now_pose: Optional[Tuple[float, ...]] = None
    lease: Optional["SourceLease"] = None
    arm_connection_generation: Optional[int] = None


@dataclass(frozen=True)
class PolicyDispatchContext:
    """Host 一次派发尝试的冻结身份。

    ``source_token`` 就是 ControlDecision 携带的原始对象；``require_current_frame``
    区分发送前（必须仍是当前帧）与发送后（允许更新的普通帧已到达）两种校验。
    """

    hand: str
    controller: object
    source_token: object
    session_id: int
    params_epoch: int
    require_current_frame: bool = True


@dataclass(frozen=True)
class ControlCompletion:
    """Host 派发结束后的回报：原尝试身份与其实际结果。"""

    context: PolicyDispatchContext
    result: ControlResult


class SourceControlPolicy(Protocol):
    """输入源 adapter 与 joystick Host 之间的 policy 缝。"""

    def decide_control(self, context: ControlContext) -> ControlDecision:
        """
在 Host 冻结的样本上做一次决策（可推进 policy 状态，决策与样本一一对应）。

Args:
    context: Host 冻结的样本。

Returns:
    决策，含 action、source token 以及可选的 rebase payload / 关节命令。
"""
        ...

    def needs_kinematics(self, context: ControlContext) -> bool:
        """
声明本次决策是否需要 Host 计算正运动学（须轻量、无副作用）。

返回 True 时 Host 在锁外计算 FK 并作为第二个 ControlContext 的 ``plan_pose`` /
``now_pose`` 送回；否则这两个字段为 None。
"""
        ...

    def get_reengage_status(self, hand: str) -> Optional[ReengageStatus]:
        """返回单手不可变再接入观察；无观察时返回 None。"""
        ...

    def validate_dispatch(self, context: PolicyDispatchContext) -> bool:
        """在 Host 真正下发命令前复核冻结的尝试是否仍然有效。

        Args:
            context: 冻结的派发尝试（token、session、epoch 与帧新鲜度要求）。

        Returns:
            True 表示可以下发；False 表示必须丢弃这次尝试。
        """
        ...

    def complete_control(self, completion: ControlCompletion) -> ControlResult:
        """Host 下发结束后回报结果，由 policy 判定提交还是过期。

        Args:
            completion: 原尝试身份与 Host 的实际结果。

        Returns:
            policy 复核后的最终结果；可能把成功改写为 STALE。
        """
        ...


class BaseSourceControlPolicy:
    """只实现可选观察与运动学需求；控制准入必须由具体 policy 实现。"""

    def needs_kinematics(self, context: ControlContext) -> bool:
        return False

    def get_reengage_status(self, hand: str) -> Optional[ReengageStatus]:
        return None


@dataclass(frozen=True)
class ConnectResult:
    """一次 ``connect()`` 的结果。

    ``retain_transport`` 只允许在 PENDING 时为 True：表示传输仍在建立中，Host 必须
    保留 transport 并等后续 transport event，而不是立刻回收。
    """

    outcome: ConnectOutcome
    message: str
    retain_transport: bool

    def __post_init__(self) -> None:
        if not isinstance(self.outcome, ConnectOutcome):
            raise ValueError("connect outcome is invalid")
        if not isinstance(self.message, str):
            raise ValueError("connect message must be a string")
        if not isinstance(self.retain_transport, bool):
            raise ValueError("retain_transport must be a boolean")
        if self.outcome is ConnectOutcome.PENDING:
            if not self.retain_transport:
                raise ValueError("PENDING connections must retain their transport")
        elif self.retain_transport:
            raise ValueError("only PENDING connections may retain their transport")


@dataclass(frozen=True)
class PoseSample:
    """一帧手部/TCP 位姿采样（位置 + 四元数 + 时间戳）。

    速度/加速度字段必须是三元 tuple；参与 jump 检测时四元数会先归一化。
    """

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    timestamp: float
    confidence: float = 1.0
    linear_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        vector_values = (
            self.linear_velocity,
            self.angular_velocity,
            self.acceleration,
        )
        if any(not isinstance(vector, tuple) for vector in vector_values):
            raise ValueError("pose vectors must be immutable tuples")
        if any(len(vector) != 3 for vector in vector_values):
            raise ValueError("pose vectors must contain exactly three values")
        values = (
            self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw,
            self.timestamp, self.confidence, *self.linear_velocity,
            *self.angular_velocity, *self.acceleration,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values
        ):
            raise ValueError("pose sample values must be finite numbers")


@dataclass(frozen=True)
class ButtonSample:
    """一帧按钮/摇杆采样。

    ``trigger``/``grip`` 是 0..127 的模拟量，``joystick_x``/``joystick_y`` 是
    -127..127 的摇杆轴；``btn_menu`` 与 ``menu_capture`` 同源，都表示菜单键状态。
    """

    trigger: int
    grip: int
    btn_xa: bool
    btn_yb: bool
    btn_menu: bool
    btn_system: bool
    menu_capture: bool
    joystick_x: int
    joystick_y: int
    joystick_z: bool
    reserve: int
    timestamp: float

    def __post_init__(self) -> None:
        integer_values = (self.trigger, self.grip, self.reserve)
        if any(isinstance(value, bool) or not isinstance(value, int) for value in integer_values):
            raise ValueError("button integer fields must be non-bool integers")
        bool_values = (
            self.btn_xa,
            self.btn_yb,
            self.btn_menu,
            self.btn_system,
            self.menu_capture,
        )
        if any(not isinstance(value, bool) for value in bool_values):
            raise ValueError("button flag fields must be booleans")
        numeric_values = (
            self.timestamp,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in numeric_values
        ):
            raise ValueError("button timestamp must be a finite number")
        axis_values = (self.joystick_x, self.joystick_y)
        if any(isinstance(value, bool) or not isinstance(value, int) for value in axis_values):
            raise ValueError("button joystick axes must be non-bool integers")
        if not isinstance(self.joystick_z, bool):
            raise ValueError("button joystick_z must be a boolean")


@dataclass(frozen=True)
class HandInputUpdate:
    """一次 batch 中单手位的输入更新。

    ``update_pose``/``update_button`` 表示本批是否携带对应数据，``pose_valid``
    表示位姿本身是否有效：无效位姿不应被接受，但同批按钮仍可更新。
    """

    hand: str
    update_pose: bool
    pose: Optional[PoseSample]
    pose_valid: bool
    update_button: bool
    button: Optional[ButtonSample]

    def __post_init__(self) -> None:
        if self.hand not in VALID_HANDS:
            raise ValueError(f"unsupported hand: {self.hand}")
        bool_values = (self.update_pose, self.pose_valid, self.update_button)
        if any(not isinstance(value, bool) for value in bool_values):
            raise ValueError("hand update control fields must be booleans")
        if self.pose is not None and not isinstance(self.pose, PoseSample):
            raise ValueError("hand update pose must be a PoseSample or None")
        if self.button is not None and not isinstance(self.button, ButtonSample):
            raise ValueError("hand update button must be a ButtonSample or None")
        if self.update_pose and self.pose is None:
            raise ValueError("pose is required when update_pose is true")
        if self.update_button and self.button is None:
            raise ValueError("button is required when update_button is true")


@dataclass(frozen=True)
class JointTargetSample:
    """provider 给出的单臂、已校验的关节空间目标。"""

    hand: str
    joint_names: Tuple[str, ...]
    joint_positions: Tuple[float, ...]
    switch_enabled: bool
    timestamp: float
    sequence: int
    trigger: int = 0
    axis_x: int = 0
    axis_y: int = 0
    buttons: Tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.hand not in VALID_HANDS:
            raise ValueError(f"unsupported hand: {self.hand}")
        if any(type(value) is not int for value in (self.trigger, self.axis_x, self.axis_y)):
            raise ValueError("joint input axes and trigger must be integers")
        if not isinstance(self.buttons, tuple) or any(not isinstance(value, str) for value in self.buttons):
            raise ValueError("joint input buttons must be immutable names")
        if not isinstance(self.joint_names, tuple) or not self.joint_names or not all(
            isinstance(name, str) and name for name in self.joint_names
        ):
            raise ValueError("joint_names must be a tuple of non-empty strings")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint names must be unique")
        if not isinstance(self.joint_positions, tuple) or len(self.joint_names) != len(self.joint_positions):
            raise ValueError("joint names and positions must have equal length")
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in self.joint_positions
        ):
            raise ValueError("joint positions must be finite numbers")
        if not isinstance(self.switch_enabled, bool):
            raise ValueError("switch_enabled must be a boolean")
        if isinstance(self.timestamp, bool) or not isinstance(self.timestamp, (int, float)) or not math.isfinite(self.timestamp):
            raise ValueError("joint target timestamp must be finite")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence < 0:
            raise ValueError("joint target sequence must be a non-negative integer")


@dataclass(frozen=True)
class JointInputBatch:
    """关节空间最新值帧；左手与右手目标互相独立。"""

    transport_generation: int
    batch_sequence: int
    timestamp: float
    left: Optional[JointTargetSample]
    right: Optional[JointTargetSample]
    lease: Optional[SourceLease] = None

    def __post_init__(self) -> None:
        if isinstance(self.transport_generation, bool) or not isinstance(self.transport_generation, int) or self.transport_generation < 0:
            raise ValueError("transport_generation must be a non-negative integer")
        if isinstance(self.batch_sequence, bool) or not isinstance(self.batch_sequence, int) or self.batch_sequence < 0:
            raise ValueError("batch_sequence must be a non-negative integer")
        if isinstance(self.timestamp, bool) or not isinstance(self.timestamp, (int, float)) or not math.isfinite(self.timestamp):
            raise ValueError("joint batch timestamp must be finite")
        if self.lease is not None and not isinstance(self.lease, SourceLease):
            raise ValueError("joint batch lease is invalid")
        for target, hand in ((self.left, HAND_LEFT), (self.right, HAND_RIGHT)):
            if target is not None:
                if not isinstance(target, JointTargetSample) or target.hand != hand:
                    raise ValueError(f"invalid {hand} joint target")


@dataclass(frozen=True)
class UnifiedHandSample:
    """provider 专有事件组装完成后的一次 Host 控制采样。"""
    hand: str
    pose: Optional[PoseSample]
    button: Optional[ButtonSample]
    pose_valid: bool
    input_sequence: int
    source_frame_id: int
    timestamp: float
    provider_id: str
    source_generation: int
    atomic: bool

    def __post_init__(self) -> None:
        if self.hand not in VALID_HANDS or not isinstance(self.provider_id, str):
            raise ValueError("unified sample identity is invalid")
        if self.pose is not None and not isinstance(self.pose, PoseSample):
            raise ValueError("unified sample pose is invalid")
        if self.button is not None and not isinstance(self.button, ButtonSample):
            raise ValueError("unified sample button is invalid")
        if not isinstance(self.pose_valid, bool) or not isinstance(self.atomic, bool):
            raise ValueError("unified sample flags are invalid")
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 0
               for value in (self.input_sequence, self.source_frame_id, self.source_generation)):
            raise ValueError("unified sample sequence is invalid")
        if not isinstance(self.timestamp, (int, float)) or not math.isfinite(self.timestamp):
            raise ValueError("unified sample timestamp is invalid")


@dataclass(frozen=True)
class RawInputBatch:
    """一个 lease 下一次传输批次的原始输入（可只含单手或同时含双手）。

    ``atomic_batch`` 为 True 表示本批双手数据来自同一帧、必须整体接受或整体拒绝
    （Quest 即此语义）；``batch_sequence`` 是 provider 自己的帧序号。
    """

    lease: SourceLease
    transport_generation: int
    batch_sequence: int
    atomic_batch: bool
    hands: Tuple[HandInputUpdate, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.lease, SourceLease):
            raise ValueError("input batch lease is required")
        if not isinstance(self.atomic_batch, bool):
            raise ValueError("atomic_batch must be a boolean")
        if not isinstance(self.hands, tuple) or any(
            not isinstance(update, HandInputUpdate) for update in self.hands
        ):
            raise ValueError("input batch hands must be a tuple of HandInputUpdate")
        for name, value in (
            ("transport_generation", self.transport_generation),
            ("batch_sequence", self.batch_sequence),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        hand_names = tuple(update.hand for update in self.hands)
        if len(set(hand_names)) != len(hand_names):
            raise ValueError("an input batch may update each hand at most once")


@dataclass(frozen=True)
class InputUpdateDecision:
    """模块对 ``RawInputBatch`` 的逐手决策。

    ``controller`` 是本批应绑定的 controller；``invalidate_controller`` 表示需要让
    已有绑定失效，``reject_controller`` 表示本批数据不可信、必须拒绝控制；
    ``details`` 是可直接透出的标量键值对（如 jump 事件）。
    """

    hand: str
    accept_pose: bool
    accept_button: bool
    controller: Optional[object]
    invalidate_controller: bool
    reject_controller: bool
    details: Tuple[Tuple[str, ScalarValue], ...] = ()

    def __post_init__(self) -> None:
        if self.hand not in VALID_HANDS:
            raise ValueError(f"unsupported hand: {self.hand}")
        if any(
            not isinstance(value, bool)
            for value in (
                self.accept_pose,
                self.accept_button,
                self.invalidate_controller,
                self.reject_controller,
            )
        ):
            raise ValueError("input decision flags must be booleans")
        if not isinstance(self.details, tuple):
            raise ValueError("input decision details must be immutable")


@dataclass(frozen=True)
class TransportEvent:
    """传输阶段变化事件，Host 据此推进 session 状态机。

    ``phase`` 是小写阶段名（ready / negotiating / healthy / degraded /
    disconnected / terminal 等）；``terminal`` 为 True 表示无需等待重连宽限期，
    直接终结会话。
    """

    lease: SourceLease
    transport_generation: int
    phase: str
    reason: Optional[str]
    terminal: bool

    def __post_init__(self) -> None:
        if not isinstance(self.lease, SourceLease):
            raise ValueError("transport event lease is required")
        if (
            isinstance(self.transport_generation, bool)
            or not isinstance(self.transport_generation, int)
            or self.transport_generation < 0
        ):
            raise ValueError("transport generation must be a non-negative integer")
        if not isinstance(self.phase, str) or not self.phase:
            raise ValueError("transport phase is required")
        if self.reason is not None and not isinstance(self.reason, str):
            raise ValueError("transport reason must be a string or None")
        if not isinstance(self.terminal, bool):
            raise ValueError("transport terminal must be a boolean")


@dataclass(frozen=True)
class ModuleStatus:
    """一个 lease 的模块运行状态快照，供 Host 与前端展示。

    ``transport_connected`` 表示传输已协商成功；``details`` 是可对外的标量键值对
    （协议版本、connection id、最后收包时间、传输 generation 等）。
    """

    lease: Optional[SourceLease]
    transport_connected: bool
    phase: str
    details: Tuple[Tuple[str, ScalarValue], ...] = ()

    def __post_init__(self) -> None:
        if self.lease is not None and not isinstance(self.lease, SourceLease):
            raise ValueError("module status lease must be a SourceLease or None")
        if not isinstance(self.transport_connected, bool):
            raise ValueError("transport_connected must be a boolean")
        if not isinstance(self.phase, str) or not self.phase:
            raise ValueError("module status phase is required")
        if not isinstance(self.details, tuple):
            raise ValueError("module status details must be an immutable tuple")
        for detail in self.details:
            if not isinstance(detail, tuple) or len(detail) != 2:
                raise ValueError("module status details must contain key/value tuples")
            key, value = detail
            if not isinstance(key, str):
                raise ValueError("module status detail keys must be strings")
            if value is not None and not isinstance(value, (bool, int, float, str)):
                raise ValueError("module status detail values must be scalar")
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError("module status float details must be finite")


# 状态栏文案必须覆盖的界面语言（与 Web 支持的语言一致）。
DEVICE_STATUS_LOCALES = ("zh-CN", "en-US")

LocalizedText = Tuple[Tuple[str, str], ...]


def _freeze_localized_text(text, field: str) -> LocalizedText:
    """把 ``{语言: 文本}`` 映射冻结为按 DEVICE_STATUS_LOCALES 排序的 (语言, 文本) 元组。

    Raises:
        ValueError: 不是映射、缺少必需语言，或存在非字符串/空文本。
    """
    if isinstance(text, tuple):
        text = dict(text)
    if not isinstance(text, Mapping):
        raise ValueError(f"device status {field} must map locale to text")
    for locale, value in text.items():
        if not isinstance(locale, str) or not isinstance(value, str) or not value:
            raise ValueError(f"device status {field} must map locale to non-empty text")
    missing = [locale for locale in DEVICE_STATUS_LOCALES if locale not in text]
    if missing:
        raise ValueError(f"device status {field} is missing locales: {', '.join(missing)}")
    ordered = list(DEVICE_STATUS_LOCALES)
    ordered.extend(sorted(locale for locale in text if locale not in DEVICE_STATUS_LOCALES))
    return tuple((locale, text[locale]) for locale in ordered)


@dataclass(frozen=True)
class DeviceStatusItem:
    """输入源声明的一项设备状态（电量、信号、温度、连接状态等），供界面状态栏展示。

    显示哪些项、显示什么文案与值完全由输入源决定，界面只做通用渲染：按当前语言取
    ``label``，把 ``value`` 与可选的 ``unit`` 拼接展示。``label`` 与需要翻译的文本型
    ``value`` 以 ``{语言: 文本}`` 传入，必须覆盖 DEVICE_STATUS_LOCALES，构造后冻结为
    (语言, 文本) 元组；数值、布尔与单位与语言无关。``value`` 为 None 表示该项当前
    不可读；``stale`` 为 True 表示设备当前离线，``value`` 只是最后已知值。
    """

    item_id: str
    label: LocalizedText
    value: Union[ScalarValue, LocalizedText] = None
    unit: Optional[str] = None
    stale: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.item_id, str) or not self.item_id:
            raise ValueError("device status item_id is required")
        object.__setattr__(self, "label", _freeze_localized_text(self.label, "label"))
        if isinstance(self.value, (Mapping, tuple)):
            object.__setattr__(self, "value", _freeze_localized_text(self.value, "value"))
        elif self.value is not None and not isinstance(self.value, (bool, int, float, str)):
            raise ValueError("device status value must be scalar or localized text")
        if isinstance(self.value, float) and not math.isfinite(self.value):
            raise ValueError("device status float value must be finite")
        if self.unit is not None and (not isinstance(self.unit, str) or not self.unit):
            raise ValueError("device status unit must be a non-empty string or None")
        if not isinstance(self.stale, bool):
            raise ValueError("device status stale must be a boolean")


class SourceEventSink(Protocol):
    """模块回调 Host 的唯一边界（Host 实现，模块只调用）。

    所有 publish 方法都携带 ``provider_identity``（Quest 下即 receiver 实例），
    Host 用它复核 lease 与传输实例是否仍然新鲜，过期回调必须被静默丢弃。
    """

    def publish_joint_input(self, batch: JointInputBatch, provider_identity) -> None:
        """发布一片关节空间最新值帧。"""
        ...

    def publish_input(
        self,
        batch: RawInputBatch,
        provider_identity,
    ) -> None:
        """发布一个原始输入 batch，由 Host 送入 policy 处理。"""
        ...

    def publish_transport(
        self,
        event: TransportEvent,
        provider_identity,
    ) -> None:
        """发布传输阶段变化，驱动 session 状态机与自动重连决策。"""
        ...

    def publish_telemetry(
        self,
        lease: SourceLease,
        provider_identity,
        items: Tuple[DeviceStatusItem, ...],
    ) -> None:
        """发布设备状态项；按 ``item_id`` 覆盖同名旧项，未提及的旧项保持不变。"""
        ...

    def connection_is_current(
        self,
        lease: SourceLease,
        provider_identity=None,
    ) -> bool:
        """判断给定 lease（及可选的传输实例）是否仍是当前有效连接。"""
        ...


class BaseInputSourceModule:
    """Host 每个可选生命周期缝的默认实现，须复现"方法缺失时"的 Host 行为。

    Cartesian/重连缝只在相应能力位开启时被触达，纯关节模块不会看到 controller
    绑定族。
    """

    # 重连缝：仅能力位开启时被调用；默认不安排宽限，调用方忽略返回值。
    def cancel_reconnect_timer(self, clear_deadline: bool = False) -> None:
        """默认无重连定时器可取消。"""
        return None

    def begin_reconnect_grace(self, lease, evaluate_recovery, expire) -> bool:
        """默认不进入宽限期，返回 False 让 Host 立即终结会话。"""
        return False

    # controller 绑定族：仅能力位开启时被调用。
    def bind_controller(self, *args) -> Optional[object]:
        """默认等价于“无上一控制器可失效”。"""
        return None

    def release_controller(self, *args) -> bool:
        """True 保持 Host 的“非参与模块”分支：失效映射且不回溯控制代次。"""
        return True

    def controller_for_hand(self, hand: str):
        """默认无绑定：返回 None。"""
        return None

    def owned_controllers(self):
        """默认不持有任何 controller。"""
        return ()

    def on_mapping_change(self, *args):
        """默认无绑定可失效，返回空集合。"""
        return ()

    def mark_input_disconnected(self):
        """默认无绑定可失效，返回空集合。"""
        return ()

    def reset_policy(self) -> None:
        """默认无可重置的 policy 状态。"""
        return None

    # 扫描/解析缝：与能力位无关；Host 仍按鸭子类型探测这两个方法（见 sources/registry.py），
    # 因此默认值必须与“方法缺失”时的 Host 回退文本一致。
    def scan_unavailable_message(self) -> str:
        """返回扫描不可用时的用户可见文案。"""
        try:
            availability = self.availability()
        except Exception:
            return "输入设备扫描不可用"
        return getattr(availability, "reason", None) or "输入设备扫描不可用"

    def resolution_error(self, address: str, name: str) -> Optional[str]:
        """返回地址解析失败时的用户可见文案；None 表示沿用 Host 通用文案。"""
        return None


class InputSourceModule(Protocol):
    """Host 使用的模块生命周期协议；必需方法对每个输入源都会被调用。

    可选缝由 BaseInputSourceModule 保证存在；control_policy 条目按 ControlContext
    文档所述串行化。
    """

    module_id: str
    control_policy: SourceControlPolicy
    arm_profile: Optional[ArmProfile]

    def current_lease(self) -> Optional[SourceLease]:
        """返回当前 lease；清理尚未收尾时返回待清理的 lease。"""
        ...

    def cleanup_idle_resources(self) -> bool:
        """清理空闲资源；True 表示没有遗留资源。"""
        ...

    def availability(self) -> SourceAvailability:
        """返回本源是否可用及其能力声明。"""
        ...

    def scan(self) -> Tuple[DeviceCandidate, ...]:
        """扫描当前可连接的设备候选。"""
        ...

    def resolve_device(
        self,
        address: str,
        name: str,
        scanned_candidate: Optional[DeviceCandidate],
    ) -> Optional[DeviceCandidate]:
        """把用户给出的 address/name 解析为设备候选；无法识别时返回 None。"""
        ...

    def connect(
        self,
        candidate: DeviceCandidate,
        lease: SourceLease,
        event_sink: SourceEventSink,
    ) -> ConnectResult:
        """建立连接；PENDING 时必须 retain_transport。"""
        ...

    def close_connection(self, lease: SourceLease, reason: str) -> bool:
        """关闭该 lease 的连接与传输；True 表示已彻底清理。"""
        ...

    def shutdown(self, reason: str) -> bool:
        """关闭整个模块（含所有 lease 与资源）；True 表示已关闭。"""
        ...

    def status(self, lease: SourceLease) -> ModuleStatus:
        """返回该 lease 的运行状态快照。"""
        ...

    def configure(self, config) -> None:
        """注入 Host 配置（如 pose jump 阈值）。"""
        ...

    def bind_lease(self, lease: SourceLease) -> None:
        """绑定 lease；旧 lease 未关闭或清理未完成时必须报错。"""
        ...
    def callback_matches(self, lease: SourceLease, provider_identity) -> bool:
        """判断回调携带的 lease/传输实例是否仍是当前实例。"""
        ...

    def cancel_reconnect_timer(self, clear_deadline: bool = False) -> None:
        """取消重连定时器；clear_deadline 为 True 时同时清除截止时间。"""
        ...

    def reset_policy(self) -> None:
        """把 control policy 重置回初始状态。"""
        ...

    def bind_controller(self, *args) -> Optional[object]:
        """绑定 controller，返回被替换掉的旧 controller。"""
        ...

    def release_controller(self, *args) -> bool:
        """解绑 controller；False 表示当前绑定与传入 controller 不符。"""
        ...

    def controller_for_hand(self, hand: str):
        """返回某只手当前绑定的 controller。"""
        ...

    def owned_controllers(self):
        """返回本模块当前持有的全部 controller。"""
        ...

    def on_mapping_change(self, *args):
        """映射（mode/master_side/手位）变化时使相关绑定失效。"""
        ...

    def mark_input_disconnected(self):
        """输入断链时使所有绑定失效。"""
        ...

    def validate_input_batch(self, batch: RawInputBatch) -> None:
        """校验 batch 是否属于本源且有原子性保证；不合法时抛 ValueError。"""
        ...

    def process_input_batch(self, batch: RawInputBatch, *args):
        """把 batch 转换为逐手 InputUpdateDecision。"""
        ...
