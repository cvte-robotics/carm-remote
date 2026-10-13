"""joystick 会话的公开状态投影（HTTP/SSE 视图）。"""

from enum import Enum

from carm_teleop_input_contracts import HAND_RIGHT


class SessionPhase(str, Enum):
    """会话生命周期阶段枚举（对外状态字符串由 _PHASE_STATUS 映射）。"""

    IDLE = "idle"
    SCANNING = "scanning"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    INITIALIZING = "initializing"
    READY = "ready"
    STOPPING = "stopping"
    CLEANUP_FAILED = "cleanup_failed"


_PHASE_STATUS = {
    SessionPhase.IDLE: "idle",
    SessionPhase.SCANNING: "scanning",
    SessionPhase.CONNECTING: "connecting",
    SessionPhase.CONNECTED: "connected",
    SessionPhase.INITIALIZING: "initializing_arm",
    SessionPhase.READY: "arm_ready",
    SessionPhase.STOPPING: "stopping",
    SessionPhase.CLEANUP_FAILED: "cleanup_failed",
}


class JoystickState:
    """
Host 持有的公开快照，由 JoystickController 数据锁保护。

只保留 HTTP/SSE 的会话状态投影；每手输入数据放在 controller 的单槽最新值信箱
（``core.mailbox`` 的 input/joint/reengage cells）里。位姿与按钮按 provider 决策
独立接受，``control_sample`` 只在两者均被接受时更新。
"""
    def __init__(self):
        """初始化连接、标定、传输与会话投影字段。"""
        # 连接状态相关
        self.ble_connected = False
        self.arm_initialized = False
        self.calibration = -90  # yaw 角度值（度），保留原 X_RIGHT 默认语义。
        self.scale = 0.4
        self.phase = SessionPhase.IDLE
        self.connected_address = None  # 当前连接的手柄地址
        self.connected_name = None  # 当前连接的手柄名称
        self.last_error = None  # 存储最后一次操作的错误信息
        self.source_type = None
        self.mode = "single"
        self.master_side = HAND_RIGHT
        self.view_side = HAND_RIGHT
        self.is_quest_device = False
        self.transport_state = "disconnected"
        self.transport_terminal = False
        self.quest_protocol_version = None
        self.quest_connection_id = None
        self.quest_last_rx_monotonic = None
        self.quest_last_disconnect_reason = None
        self.quest_transport_generation = 0
        self.data_sequence = 0
        # 输入源声明的设备状态项（item_id -> DeviceStatusItem），插入序即展示序。
        self.device_status = {}
        self.arm_states = {
            "master": {
                "ip": None,
                "control_host": None,
                "arm_index": None,
                "hand_binding": None,
                "connected": False,
            },
            "slave": {
                "ip": None,
                "control_host": None,
                "arm_index": None,
                "hand_binding": None,
                "connected": False,
            },
        }

    @property
    def status(self) -> str:
        """返回当前阶段对应的对外状态字符串。"""
        return _PHASE_STATUS[self.phase]
