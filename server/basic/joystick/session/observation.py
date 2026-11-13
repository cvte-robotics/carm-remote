"""Host 观察快照及协议投影：每次新建、单调用方消费，嵌套值无需冻结。"""

import copy
import time
from dataclasses import dataclass, fields
from typing import Optional

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, normalize_hand

from ..core.mailbox import reengage_payload
from ..core.models import (
    body_intent_to_dict,
    button_to_dict,
    head_pose_to_dict,
    intent_to_dict,
    pose_to_dict,
)
from ..core.pose_math import yaw_to_legacy_mode
from ..sources.registry import BindingPhase
from .arm import active_arm_profile
from .control_transaction import _INPUT_FRESH_S


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
    waist: Optional[dict] = None


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
    """单手最近位姿/按钮/关节投影；嵌套值每次新建，仅本次调用方消费。"""

    pose: object = None
    button: object = None
    joint: object = None
    intent: object = None

    def as_dict(self):
        """直接引用本次快照的嵌套值；仅当存在关节数据时附带 joint 键。"""
        result = {"pose": self.pose, "button": self.button, "intent": self.intent}
        if self.joint is not None:
            result["joint"] = self.joint
        return result


@dataclass(frozen=True)
class ObservationSnapshot:
    """一次观察的冻结数据类；序列号单调，不要求唯一对应一份内容。

    嵌套字典在 data 锁内从冻结发布新建，每次只交给一个调用方，不缓存或跨线程
    共享，因此无需再冻结。顶层 ``pose`` / ``button`` 与视角手共用字典；
    ``hand`` 按视角取用对应的手部投影。
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
    intent: object = None
    direct: object = None
    head_pose: object = None
    body_intent: object = None

    def hand(self, side: str) -> ObservationHand:
        """按臂侧返回单手投影（非 "left" 一律视为右手）。"""
        return self.left_hand if side == "left" else self.right_hand


def build_observation_snapshot(owner):
    """抓取一份世代一致的 Host 观测快照（data 锁内读取并投影）。

    嵌套字典每次新建、仅本次调用方消费，不再冻结或复制。

    Returns:
        ObservationSnapshot；controller 不可用时返回 ``available=False`` 的空快照。
    """
    if not owner.available:
        empty_hand = ObservationHand()
        return ObservationSnapshot(
            sequence=0, available=False,
            status=ObservationStatus(False, False, 1, -90, getattr(owner, "scale", 0.4),
                                     "not_available", None, "single",
                                     False, HAND_RIGHT, HAND_RIGHT,
                                     capabilities=None, arm_profile=None),
            device=ObservationDevice(None, None),
            transport=ObservationTransport("disconnected", None, None, None, None),
            master_arm=ObservationArm(), slave_arm=ObservationArm(),
            left_hand=empty_hand, right_hand=empty_hand, pose=None, button=None,
            stream_terminal=True, emit_disconnect_frame=False, direct={})
    with owner._data_lock:
        binding = owner.active_source_binding
        binding_active = bool(
            binding is not None and binding.phase is not BindingPhase.CLOSED
        )
        # 能力与拓扑限制由 Host 投影出去，Web 刷新后不需要按 provider 名字猜测。
        capabilities_payload = (
            {field.name: getattr(binding.capabilities, field.name)
             for field in fields(binding.capabilities)} if binding is not None else None
        )
        arm_profile = active_arm_profile(owner) if binding is not None else None
        arm_profile_payload = None if arm_profile is None else arm_profile.as_dict()
        supports_reconnect_grace = bool(
            binding is not None
            and binding.capabilities.supports_reconnect_grace
        )
        # 传输细节按观测频率读取，避免每帧取得模块/接收器锁。
        details = {}
        transport_state = owner.state.transport_state
        module = owner.source_registry.active_module() if binding_active else None
        if module is not None and owner._optional_interface_enabled(binding.capabilities):
            transport_status = module.status(binding.lease)
            details = dict(transport_status.details)
            if binding.input_ready:
                transport_state = transport_status.phase
        last_rx = details.get("last_rx_monotonic", owner.state.quest_last_rx_monotonic)
        last_rx_age_ms = (
            max(0, int((time.monotonic() - last_rx) * 1000))
            if last_rx is not None else None
        )
        stream_connected = bool(
            owner.state.ble_connected
            or (
                binding_active
                and supports_reconnect_grace
                and owner.state.transport_state in {"negotiating", "healthy"}
            )
        )
        emit_disconnect_frame = bool(
            supports_reconnect_grace
            and not binding_active
            and not owner.state.ble_connected
        )
        hands = {}
        # 每种 publication 只读一次，双手始终来自同一冻结批次。
        # 先读计数再读内容：组装期间的新发布留给下一次 SSE 检测，不能提前消费序号。
        sequence = owner.state.data_sequence + owner._input_publication_sequence
        input_publication = owner._input_publication.load()
        joint_publication = owner._joint_publication.load()
        for hand in (HAND_LEFT, HAND_RIGHT):
            input_frame = (
                None if input_publication is None
                else input_publication.frame(hand)
            )
            joint_frame = (
                None if joint_publication is None
                else joint_publication.frame(hand)
            )
            if (not binding_active or (input_frame is not None
                    and input_frame.lease != binding.lease)):
                input_frame = None
            if (not binding_active or (joint_frame is not None
                    and joint_frame.lease != binding.lease)):
                joint_frame = None
            joint = None
            if (
                joint_frame is not None and binding_active
                and binding is not None
                and binding.capabilities.provides_joint_targets
                and joint_frame.target is not None
            ):
                target = joint_frame.target
                joint = {
                    "switch_enabled": target.switch_enabled,
                    "joint_positions": list(target.joint_positions),
                    "trigger": target.trigger,
                    "axis_x": target.axis_x,
                    "axis_y": target.axis_y,
                    "buttons": list(target.buttons),
                    "fresh": time.monotonic() - joint_frame.received_monotonic <= _INPUT_FRESH_S,
                    "sequence": joint_frame.batch_sequence,
                    "reengage": reengage_payload(owner._reengage_cells[hand].load()),
                }
            hands[hand] = ObservationHand(
                joint=joint,
                intent=intent_to_dict(input_frame.intent) if input_frame is not None else None,
                pose=(
                    pose_to_dict(input_frame.pose)
                    if input_frame is not None and input_frame.pose
                    else None
                ),
                button=(
                    button_to_dict(input_frame.button)
                    if input_frame is not None and input_frame.button
                    else None
                ),
            )
        view_side = normalize_hand(owner.state.view_side, HAND_RIGHT)
        return ObservationSnapshot(
            sequence=sequence, available=True,
            pose=hands[view_side].pose,
            button=hands[view_side].button,
            intent=hands[view_side].intent,
            head_pose=head_pose_to_dict(
                input_publication.waist_input if input_publication is not None else None),
            body_intent=body_intent_to_dict(
                input_publication.body_intent if input_publication is not None else None),
            status=ObservationStatus(stream_connected, owner.state.arm_initialized,
                yaw_to_legacy_mode(owner.state.calibration), owner.state.calibration,
                owner.state.scale, owner.state.status,
                owner.state.source_type,
                owner.state.mode, owner.state.is_quest_device, owner.state.master_side,
                view_side,
                capabilities=capabilities_payload, arm_profile=arm_profile_payload,
                device_status=tuple(owner.state.device_status.values()),
                waist=owner.state.waist),
            device=ObservationDevice(owner.state.connected_address, owner.state.connected_name),
            transport=ObservationTransport(transport_state,
                details.get("protocol_version", owner.state.quest_protocol_version),
                details.get("connection_id", owner.state.quest_connection_id),
                last_rx_age_ms, owner.state.quest_last_disconnect_reason),
            master_arm=ObservationArm.from_state(owner.state.arm_states["master"]),
            slave_arm=ObservationArm.from_state(owner.state.arm_states["slave"]),
            left_hand=hands[HAND_LEFT], right_hand=hands[HAND_RIGHT],
            stream_terminal=bool(not owner.state.ble_connected and (not supports_reconnect_grace or not binding_active)),
            emit_disconnect_frame=emit_disconnect_frame,
            direct=copy.deepcopy(owner.state.direct))


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
        "capabilities": status.capabilities,
        "arm_profile": status.arm_profile,
        "waist": status.waist,
        "slave_arm": snapshot.slave_arm.as_dict(), "pose": hand.pose,
        "button": hand.button, "intent": hand.intent, "transport": {
            name: getattr(snapshot.transport, name)
            for name in (field.name for field in fields(snapshot.transport))
        },
        "direct": snapshot.direct,
        "head_pose": snapshot.head_pose,
        "body_intent": snapshot.body_intent,
    }
    if not snapshot.available:
        payload["available"] = False
    if snapshot.left_hand.joint is not None or snapshot.right_hand.joint is not None:
        payload["hands"] = {"left": snapshot.left_hand.as_dict(), "right": snapshot.right_hand.as_dict()}
    return payload


def project_sse_payload(snapshot: ObservationSnapshot, heartbeat=False, include_terminal_fields=False) -> dict:
    """把快照投影为一条 SSE 数据帧。

    Args: snapshot 观察快照；heartbeat 心跳帧（清空位姿与功能结果）；
        include_terminal_fields 是否附带流终止字段。
    Returns: 可直接 json.dumps 的字典。
    """
    status = snapshot.status
    payload = {
        "sequence": snapshot.sequence, "pose": snapshot.pose,
        "button": snapshot.button, "intent": snapshot.intent,
        "status": {"ble_connected": status.ble_connected, "status": status.status,
                   "calibration": status.calibration,
                   "yaw": status.yaw,
                   "scale": status.scale,
                   "arm_initialized": status.arm_initialized,
                   "device_status": _device_status_payload(status.device_status),
                   "source_type": status.source_type, "mode": status.mode,
                   "master_side": status.master_side,
                   "capabilities": status.capabilities,
                   "arm_profile": status.arm_profile,
                   "waist": status.waist},
        "device": {"address": snapshot.device.address, "name": snapshot.device.name},
        "arms": {"master": snapshot.master_arm.as_dict(), "slave": snapshot.slave_arm.as_dict()},
        "hands": {"left": snapshot.left_hand.as_dict(), "right": snapshot.right_hand.as_dict()},
        "transport": {field.name: getattr(snapshot.transport, field.name)
                      for field in fields(snapshot.transport)},
        "direct": snapshot.direct,
        "head_pose": snapshot.head_pose,
        "body_intent": snapshot.body_intent,
    }
    if include_terminal_fields:
        payload["stream_terminal"] = snapshot.stream_terminal
        payload["emit_disconnect_frame"] = snapshot.emit_disconnect_frame
    if heartbeat:
        payload["heartbeat"] = True
        payload["pose"] = None
        payload["button"] = None
        payload["intent"] = None
        payload["head_pose"] = None
        payload["body_intent"] = None
    return payload
