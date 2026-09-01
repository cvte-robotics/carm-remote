"""Neutral contracts shared by the teleoperation host and input modules."""

from dataclasses import dataclass
from enum import Enum
import math
import struct
from typing import Optional, Protocol, Tuple, Union


HAND_LEFT = "left"
HAND_RIGHT = "right"
VALID_HANDS = frozenset((HAND_LEFT, HAND_RIGHT))

ScalarValue = Optional[Union[bool, int, float, str]]


def normalize_hand(hand: str, default: str = HAND_RIGHT) -> str:
    normalized = (hand or "").strip().lower()
    return normalized if normalized in VALID_HANDS else default


def opposite_hand(hand: str) -> str:
    if hand == HAND_LEFT:
        return HAND_RIGHT
    if hand == HAND_RIGHT:
        return HAND_LEFT
    raise ValueError(f"Unsupported hand: {hand}")


class CalibrationMode(Enum):
    X_LEFT = 0
    X_RIGHT = 1
    X_FORWARD = 2
    X_BACKWARD = 3


@dataclass
class PoseData:
    """Legacy mutable pose view retained at the public facade boundary."""

    timestamp: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    confidence: float
    linear_velocity_x: float
    linear_velocity_y: float
    linear_velocity_z: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float

    def __init__(
        self,
        timestamp,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        confidence,
        linear_velocity_x,
        linear_velocity_y,
        linear_velocity_z,
        angular_velocity_x,
        angular_velocity_y,
        angular_velocity_z,
        acceleration_x,
        acceleration_y,
        acceleration_z,
    ):
        for name, value in locals().copy().items():
            if name != "self":
                setattr(self, name, value)

    @classmethod
    def from_bytes(cls, data: bytes):
        values = struct.unpack("<Qfffffffffffffffff", data)
        return cls(*values)

    def __str__(self) -> str:
        return (
            "PoseData(\n"
            f"  timestamp: {self.timestamp}\n"
            f"  position: ({self.x:.3f}, {self.y:.3f}, {self.z:.3f})\n"
            f"  quaternion: ({self.qx:.3f}, {self.qy:.3f}, {self.qz:.3f}, {self.qw:.3f})\n"
            f"  confidence: {self.confidence:.3f}\n"
            f"  linear_velocity: ({self.linear_velocity_x:.3f}, {self.linear_velocity_y:.3f}, {self.linear_velocity_z:.3f})\n"
            f"  angular_velocity: ({self.angular_velocity_x:.3f}, {self.angular_velocity_y:.3f}, {self.angular_velocity_z:.3f})\n"
            f"  acceleration: ({self.acceleration_x:.3f}, {self.acceleration_y:.3f}, {self.acceleration_z:.3f})\n"
            ")"
        )


@dataclass
class ButtonStatus:
    """Legacy mutable button view retained at the public facade boundary."""

    timestamp: int
    btn_xa: bool
    btn_yb: bool
    menu_capture: bool
    joystick_z: bool
    reserve: int
    trigger: int
    grip: int
    joystick_x: int
    joystick_y: int

    def __init__(
        self,
        timestamp,
        btn_xa,
        btn_yb,
        menu_capture,
        joystick_z,
        reserve,
        trigger,
        grip,
        joystick_x,
        joystick_y,
    ):
        for name, value in locals().copy().items():
            if name != "self":
                setattr(self, name, value)

    @classmethod
    def from_bytes(cls, data: bytes):
        timestamp, flags, trigger, grip, joystick_x, joystick_y = struct.unpack(
            "<QBBBBB",
            data,
        )
        return cls(
            timestamp=timestamp,
            btn_xa=bool(flags & 0x01),
            btn_yb=bool(flags & 0x02),
            menu_capture=bool(flags & 0x04),
            joystick_z=bool(flags & 0x08),
            reserve=(flags >> 4) & 0x0F,
            trigger=trigger,
            grip=grip,
            joystick_x=joystick_x,
            joystick_y=joystick_y,
        )

    def __str__(self) -> str:
        return (
            f"ButtonStatus(timestamp={self.timestamp}, "
            f"XA={self.btn_xa}, YB={self.btn_yb}, "
            f"Menu/Capture={self.menu_capture}, JoystickZ={self.joystick_z}, "
            f"Trigger={self.trigger}, Grip={self.grip}, "
            f"JoystickX={self.joystick_x}, JoystickY={self.joystick_y})"
        )


@dataclass(frozen=True)
class SourceCapabilities:
    provides_pose: bool
    provides_buttons: bool
    can_control_arms: bool
    connect_only: bool
    supports_reconnect_grace: bool

    def __post_init__(self) -> None:
        values = (
            self.provides_pose,
            self.provides_buttons,
            self.can_control_arms,
            self.connect_only,
            self.supports_reconnect_grace,
        )
        if any(not isinstance(value, bool) for value in values):
            raise ValueError("source capability fields must be booleans")


@dataclass(frozen=True)
class SourceAvailability:
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
    READY = "ready"
    PENDING = "pending"
    FAILED = "failed"


class ControlAction(Enum):
    """Neutral decision for one input-driven control sample."""

    READY = "ready"
    REBASE = "rebase"
    REJECT = "reject"


class ControlOutcome(Enum):
    """Fact reported after Host completes the control attempt."""

    REBASE_APPLIED = "rebase_applied"
    DISPATCH_COMMITTED = "dispatch_committed"
    DISPATCH_REJECTED = "dispatch_rejected"
    STALE = "stale"


@dataclass(frozen=True)
class ControlDecision:
    action: ControlAction
    source_token: Optional[object]
    reason: str = ""
    rebase_payload: Optional[object] = None


@dataclass(frozen=True)
class ControlResult:
    outcome: ControlOutcome
    source_token: Optional[object]
    reason: str = ""


@dataclass(frozen=True)
class ControlContext:
    hand: str
    controller: object
    controller_generation: int
    input_generation: int
    control_generation: int
    bound_active: bool
    need_sync_feedback: bool


@dataclass(frozen=True)
class PolicyDispatchContext:
    hand: str
    controller: object
    controller_generation: int
    source_token: object
    input_generation: int
    control_generation: int
    require_current_frame: bool = True


@dataclass(frozen=True)
class ControlCompletion:
    context: PolicyDispatchContext
    result: ControlResult


class SourceControlPolicy(Protocol):
    """Policy seam used by a source adapter and the joystick Host."""

    def decide_control(self, context: ControlContext) -> ControlDecision: ...
    def validate_dispatch(self, context: PolicyDispatchContext) -> bool: ...
    def complete_control(self, completion: ControlCompletion) -> ControlResult: ...


@dataclass(frozen=True)
class ConnectResult:
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
class RawInputBatch:
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


class SourceEventSink(Protocol):
    def publish_input(
        self,
        batch: RawInputBatch,
        provider_identity,
    ) -> None: ...

    def publish_transport(
        self,
        event: TransportEvent,
        provider_identity,
    ) -> None: ...

    def publish_telemetry(
        self,
        lease: SourceLease,
        provider_identity,
        battery_level=None,
        rssi=None,
    ) -> None: ...

    def connection_is_current(
        self,
        lease: SourceLease,
        provider_identity=None,
    ) -> bool: ...


class InputSourceModule(Protocol):
    module_id: str

    def availability(self) -> SourceAvailability: ...
    def scan(self) -> Tuple[DeviceCandidate, ...]: ...
    def resolve_device(
        self,
        address: str,
        name: str,
        scanned_candidate: Optional[DeviceCandidate],
    ) -> Optional[DeviceCandidate]: ...
    def connect(
        self,
        candidate: DeviceCandidate,
        lease: SourceLease,
        event_sink: SourceEventSink,
    ) -> ConnectResult: ...
    def close_connection(self, lease: SourceLease, reason: str) -> bool: ...
    def shutdown(self, reason: str) -> bool: ...
    def status(self, lease: SourceLease) -> ModuleStatus: ...
