"""Immutable Host observation snapshots and protocol projections."""

from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Mapping, Optional


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _mutable(value):
    if isinstance(value, Mapping):
        return {key: _mutable(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_mutable(item) for item in value]
    return value


@dataclass(frozen=True)
class ObservationStatus:
    ble_connected: bool
    arm_initialized: bool
    calibration: int
    scale: float
    status: str
    battery_level: object
    rssi: object
    source_type: Optional[str]
    mode: str
    is_quest_device: bool
    master_side: str
    view_side: str


@dataclass(frozen=True)
class ObservationDevice:
    address: Optional[str]
    name: Optional[str]


@dataclass(frozen=True)
class ObservationTransport:
    state: str
    protocol: Optional[str]
    connection_id: Optional[str]
    last_rx_age_ms: Optional[int]
    disconnect_reason: Optional[str]


@dataclass(frozen=True)
class ObservationArm:
    ip: Optional[str] = None
    control_host: Optional[str] = None
    arm_index: Optional[int] = None
    hand_binding: Optional[str] = None
    connected: bool = False

    @classmethod
    def from_state(cls, state):
        return cls(**{field.name: state.get(field.name) for field in fields(cls)})

    def as_dict(self):
        return {field.name: getattr(self, field.name) for field in fields(self)}


@dataclass(frozen=True)
class ObservationHand:
    pose: object
    button: object

    @classmethod
    def from_values(cls, pose=None, button=None):
        return cls(_freeze(pose), _freeze(button))

    def as_dict(self):
        return {"pose": _mutable(self.pose), "button": _mutable(self.button)}


@dataclass(frozen=True)
class ObservationSnapshot:
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
        object.__setattr__(self, "pose", _freeze(self.pose))
        object.__setattr__(self, "button", _freeze(self.button))

    def hand(self, side: str) -> ObservationHand:
        return self.left_hand if side == "left" else self.right_hand


def project_http_status(snapshot: ObservationSnapshot) -> dict:
    status = snapshot.status
    hand = snapshot.hand(status.view_side)
    payload = {
        "ble_connected": status.ble_connected,
        "arm_initialized": status.arm_initialized, "calibration": status.calibration,
        "scale": status.scale, "status": status.status,
        "battery_level": status.battery_level, "rssi": status.rssi,
        "source_type": status.source_type, "mode": status.mode,
        "is_quest_device": status.is_quest_device, "master_side": status.master_side,
        "view_side": status.view_side, "master_arm": snapshot.master_arm.as_dict(),
        "slave_arm": snapshot.slave_arm.as_dict(), "pose": _mutable(hand.pose),
        "button": _mutable(hand.button), "transport": {
            name: getattr(snapshot.transport, name)
            for name in (field.name for field in fields(snapshot.transport))
        },
    }
    if not snapshot.available:
        payload["available"] = False
    return payload


def project_sse_payload(snapshot: ObservationSnapshot, heartbeat=False, include_terminal_fields=False) -> dict:
    status = snapshot.status
    payload = {
        "sequence": snapshot.sequence, "pose": _mutable(snapshot.pose),
        "button": _mutable(snapshot.button),
        "status": {"ble_connected": status.ble_connected, "status": status.status,
                   "battery_level": status.battery_level, "rssi": status.rssi,
                   "source_type": status.source_type, "mode": status.mode,
                   "master_side": status.master_side},
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
