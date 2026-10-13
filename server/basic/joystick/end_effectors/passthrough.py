"""基于单个既存 Carm 连接实现的六指 EtherCAT/CAN-FD 透传。"""

import logging
import math

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT
from . import EndEffectorBase, EndEffectorContext


CAN_FLAGS = 0x09
CAN_IDS = {
    HAND_LEFT: 0x007E010F,
    HAND_RIGHT: 0x007F010F,
}
SLAVE_IDS = {
    HAND_LEFT: 0x7E,
    HAND_RIGHT: 0x7F,
}
FINGER_COUNT = 6
MAX_POSITION = 1000
MOTION_DURATION_MS = 1000


def modbus_crc16(data: bytes) -> int:
    """计算 Modbus CRC-16（多项式 0xA001）校验值。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_hand_frame(side: str, finger_id: int, position: int) -> dict:
    """构造单指透传帧（含 CRC 与帧尾填充）。

    Args: side "left"/"right"；finger_id 0..5；position 0..1000。
    Returns: 与 Carm 透传接口一致的 dict（can_id/flags/can_fd/data）。
    Raises: ValueError 臂侧、手指编号或位置越界。
    """
    if side not in SLAVE_IDS:
        raise ValueError(f"unsupported hand side: {side}")
    if not 0 <= finger_id < FINGER_COUNT:
        raise ValueError(f"finger_id must be in range 0..{FINGER_COUNT - 1}")
    if not 0 <= position <= MAX_POSITION:
        raise ValueError(f"position must be in range 0..{MAX_POSITION}")

    payload = bytes([
        SLAVE_IDS[side],
        0x10,
        0x04,
        0x1C,
        0x00,
        0x03,
        0x06,
        (finger_id >> 8) & 0xFF,
        finger_id & 0xFF,
        (position >> 8) & 0xFF,
        position & 0xFF,
        (MOTION_DURATION_MS >> 8) & 0xFF,
        MOTION_DURATION_MS & 0xFF,
    ])
    crc = modbus_crc16(payload)
    wire_data = payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF, 0x00])
    return {
        "can_id": CAN_IDS[side],
        "flags": CAN_FLAGS,
        "can_fd": True,
        "data": wire_data.hex().upper(),
    }


class PassthroughEndEffector(EndEffectorBase):
    """通过 Carm 透传接口驱动六指手的 end-effector 实现。"""

    NAME = "passthrough"

    def __init__(self, context: EndEffectorContext, **options) -> None:
        """校验臂侧并取用 Carm SDK 的透传发送接口。

        Raises: ValueError 臂侧不支持；RuntimeError SDK 不支持透传。
        """
        super().__init__(context, **options)
        normalized_side = (context.side or "").strip().lower()
        if normalized_side not in SLAVE_IDS:
            raise ValueError(f"unsupported hand side: {context.side}")
        sender = getattr(self.carm, "set_ecat_passthrough_data", None)
        if not callable(sender):
            raise RuntimeError("current Carm SDK does not support end-effector passthrough")
        self._sender = sender
        self._side = normalized_side

    def set_eff(self, trigger: int) -> bool:
        """把 trigger 映射为六指位置，逐指下发透传帧。

        Args: trigger 为 provider 归一化后的触发值（按 0-127 归一化到 0-1000）。
        Returns: 恒为 True（单指异常只记录日志，不中断其余手指）。
        Raises: ValueError trigger 非数值或非有限值。
        """
        if isinstance(trigger, bool) or not isinstance(trigger, (int, float)):
            raise ValueError("passthrough end-effector trigger must be numeric")
        value = float(trigger)
        if not math.isfinite(value):
            raise ValueError("passthrough end-effector trigger must be finite")
        position = round(
            max(0.0, min(value, 127.0)) / 127.0 * MAX_POSITION
        )

        for finger_id in range(FINGER_COUNT):
            frame = build_hand_frame(self._side, finger_id, position)
            try:
                self._sender(0, frame)
            except Exception:
                logging.exception(
                    "[PassthroughEndEffector] Send exception side=%s finger=%d",
                    self._side,
                    finger_id,
                )
        return True
