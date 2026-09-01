from enum import Enum

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT


class SessionPhase(str, Enum):
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
    """Host-owned public snapshot, protected by JoystickController data lock."""
    def __init__(self):
        # 连接状态相关
        self.ble_connected = False
        self.arm_initialized = False
        self.calibration = 1  # CalibrationMode.X_RIGHT
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
        self.pose_data = None  # PoseData compatibility view for SSE
        self.button_data = None  # ButtonStatus 对象
        self.data_sequence = 0
        self.control_generation = 0
        self.battery_level = None  # 手柄电量 (0-100)
        self.rssi = None  # RSSI 信号强度 (dBm)
        self.hands = {
            HAND_LEFT: {
                "pose_data": None,
                "button_data": None,
                "input_sequence": 0,
            },
            HAND_RIGHT: {
                "pose_data": None,
                "button_data": None,
                "input_sequence": 0,
            },
        }
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
        return _PHASE_STATUS[self.phase]
