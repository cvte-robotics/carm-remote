"""Host 持有的不可变观察（observation）快照及其协议投影。"""

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Mapping, Optional


def _freeze(value):
    """递归把 dict/list 冻结为只读映射与 tuple。"""
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _mutable(value):
    """递归把只读映射/tuple 还原为可 JSON 序列化的 dict/list。"""
    if isinstance(value, Mapping):
        return {key: _mutable(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_mutable(item) for item in value]
    return value


@dataclass(frozen=True)
class ObservationStatus:
    """会话状态投影，是 HTTP/SSE status 字段的唯一来源。"""

    ble_connected: bool
    arm_initialized: bool
    calibration: Optional[int]
    yaw: int
    scale: float
    status: str
    source_type: Optional[str]
    mode: str
    is_quest_device: bool
    master_side: str
    view_side: str
    capabilities: Optional[dict] = None
    arm_profile: Optional[dict] = None
    # 输入源声明的 DeviceStatusItem 元组；空元组表示该输入源没有状态栏内容。
    device_status: tuple = ()


def _device_status_payload(items) -> list:
    """把 DeviceStatusItem 元组投影为前端状态栏使用的 JSON 列表。

    ``label`` 与文本型 ``value`` 投影为 ``{语言: 文本}``，由前端按当前语言选取。
    """
    return [
        {
            "id": item.item_id,
            "label": dict(item.label),
            "value": dict(item.value) if isinstance(item.value, tuple) else item.value,
            "unit": item.unit,
            "stale": item.stale,
        }
        for item in items
    ]


@dataclass(frozen=True)
class ObservationDevice:
    """已连接输入设备的地址与名称投影。"""

    address: Optional[str]
    name: Optional[str]


@dataclass(frozen=True)
class ObservationTransport:
    """传输层状态投影（协议、连接 id、收包年龄与断开原因）。"""

    state: str
    protocol: Optional[str]
    connection_id: Optional[str]
    last_rx_age_ms: Optional[int]
    disconnect_reason: Optional[str]


@dataclass(frozen=True)
class ObservationArm:
    """单条机械臂绑定投影（IP、控制主机、索引、手侧与连接状态）。"""

    ip: Optional[str] = None
    control_host: Optional[str] = None
    arm_index: Optional[int] = None
    hand_binding: Optional[str] = None
    connected: bool = False

    @classmethod
    def from_state(cls, state):
        """从 owner.arm_states 的普通 dict 按字段名构造投影。"""
        return cls(**{field.name: state.get(field.name) for field in fields(cls)})

    def as_dict(self):
        """还原为字段顺序稳定的普通 dict。"""
        return {field.name: getattr(self, field.name) for field in fields(self)}


@dataclass(frozen=True)
class ObservationHand:
    """单手最近位姿/按钮/关节投影（值已递归冻结）。"""

    pose: object
    button: object
    joint: object = None

    @classmethod
    def from_values(cls, pose=None, button=None, joint=None):
        """从原始值构造，并递归冻结输入。"""
        return cls(_freeze(pose), _freeze(button), _freeze(joint))

    def as_dict(self):
        """还原为普通 dict；仅当存在关节数据时附带 joint 键。"""
        result = {"pose": _mutable(self.pose), "button": _mutable(self.button)}
        if self.joint is not None:
            result["joint"] = _mutable(self.joint)
        return result


@dataclass(frozen=True)
class ObservationSnapshot:
    """一次观察的不可变快照；同一快照内各字段取自同一序列号。

    ``pose`` / ``button`` 在构造后递归冻结，使 SSE/HTTP 读者不会观察到半更新
    状态；``hand`` 按视角取用对应的手部投影。
    """

    sequence: int
    available: bool
    status: ObservationStatus
    device: ObservationDevice
    transport: ObservationTransport
    master_arm: ObservationArm
    slave_arm: ObservationArm
    left_hand: ObservationHand
    right_hand: ObservationHand
    pose: object
    button: object
    stream_terminal: bool
    emit_disconnect_frame: bool

    def __post_init__(self):
        """递归冻结顶层的 pose/button。"""
        object.__setattr__(self, "pose", _freeze(self.pose))
        object.__setattr__(self, "button", _freeze(self.button))

    def hand(self, side: str) -> ObservationHand:
        """按臂侧返回单手投影（非 "left" 一律视为右手）。"""
        return self.left_hand if side == "left" else self.right_hand


def project_http_status(snapshot: ObservationSnapshot) -> dict:
    """把快照投影为 /api/joystick/status 的 data 字典。

    Args: snapshot 一致性观察快照。
    Returns: 键顺序稳定的 HTTP 状态字典；快照不可用时附带 available=False。
    """
    status = snapshot.status
    hand = snapshot.hand(status.view_side)
    payload = {
        "ble_connected": status.ble_connected,
        "arm_initialized": status.arm_initialized, "calibration": status.calibration,
        "yaw": status.yaw, "scale": status.scale, "status": status.status,
        "device_status": _device_status_payload(status.device_status),
        "source_type": status.source_type, "mode": status.mode,
        "is_quest_device": status.is_quest_device, "master_side": status.master_side,
        "view_side": status.view_side, "master_arm": snapshot.master_arm.as_dict(),
        "capabilities": _mutable(status.capabilities),
        "arm_profile": _mutable(status.arm_profile),
        "slave_arm": snapshot.slave_arm.as_dict(), "pose": _mutable(hand.pose),
        "button": _mutable(hand.button), "transport": {
            name: getattr(snapshot.transport, name)
            for name in (field.name for field in fields(snapshot.transport))
        },
    }
    if not snapshot.available:
        payload["available"] = False
    if snapshot.left_hand.joint is not None or snapshot.right_hand.joint is not None:
        payload["hands"] = {"left": snapshot.left_hand.as_dict(), "right": snapshot.right_hand.as_dict()}
    return payload


def project_sse_payload(snapshot: ObservationSnapshot, heartbeat=False, include_terminal_fields=False) -> dict:
    """把快照投影为一条 SSE 数据帧。

    Args: snapshot 观察快照；heartbeat 心跳帧（清空 pose/button）；
        include_terminal_fields 是否附带流终止字段。
    Returns: 可直接 json.dumps 的字典。
    """
    status = snapshot.status
    payload = {
        "sequence": snapshot.sequence, "pose": _mutable(snapshot.pose),
        "button": _mutable(snapshot.button),
        "status": {"ble_connected": status.ble_connected, "status": status.status,
                   "calibration": status.calibration,
                   "yaw": status.yaw,
                   "scale": status.scale,
                   "arm_initialized": status.arm_initialized,
                   "device_status": _device_status_payload(status.device_status),
                   "source_type": status.source_type, "mode": status.mode,
                   "master_side": status.master_side,
                   "capabilities": _mutable(status.capabilities),
                   "arm_profile": _mutable(status.arm_profile)},
        "device": {"address": snapshot.device.address, "name": snapshot.device.name},
        "arms": {"master": snapshot.master_arm.as_dict(), "slave": snapshot.slave_arm.as_dict()},
        "hands": {"left": snapshot.left_hand.as_dict(), "right": snapshot.right_hand.as_dict()},
        "transport": {field.name: getattr(snapshot.transport, field.name)
                      for field in fields(snapshot.transport)},
    }
    if include_terminal_fields:
        payload["stream_terminal"] = snapshot.stream_terminal
        payload["emit_disconnect_frame"] = snapshot.emit_disconnect_frame
    if heartbeat:
        payload["heartbeat"] = True
        payload["pose"] = None
        payload["button"] = None
    return payload
