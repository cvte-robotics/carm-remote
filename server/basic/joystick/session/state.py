import threading

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT

class JoystickState:
    """全局状态存储（简化版：参考 PyQt 实现，减少锁使用）"""
    def __init__(self):
        # 连接状态相关
        self.ble_connected = False
        self.arm_initialized = False
        self.scan_results = []
        self.calibration = 1  # CalibrationMode.X_RIGHT
        self.scale = 0.4
        self.status = "idle"  # idle/scanning/connecting/connected/initializing_arm/arm_ready
        self.log_messages = []
        self.connected_address = None  # 当前连接的手柄地址
        self.connected_name = None  # 当前连接的手柄名称
        self.last_error = None  # 存储最后一次操作的错误信息
        self.disconnect_reason = None  # 断开连接原因
        self.source_type = None
        self.mode = "single"
        self.master_side = HAND_RIGHT
        self.view_side = HAND_RIGHT
        self.is_quest_device = False
        self.transport_state = "disconnected"
        self.quest_protocol_version = None
        self.quest_connection_id = None
        self.quest_last_rx_monotonic = None
        self.quest_last_disconnect_reason = None
        self.quest_transport_generation = 0
        self.pose_data = None  # PoseData compatibility view for SSE
        self.button_data = None  # ButtonStatus 对象
        self.data_sequence = 0
        self.control_generation = 0
        self.last_update_ts = 0.0
        self.battery_level = None  # 手柄电量 (0-100)
        self.rssi = None  # RSSI 信号强度 (dBm)
        self.gripper_pose = None  # 计算后的夹爪位置 (0.0 - gripper_max_value)
        self.gripper_tau = None  # 计算后的夹爪力矩
        self.hands = {
            HAND_LEFT: {
                "pose_data": None,
                "button_data": None,
                "gripper_pose": None,
                "gripper_tau": None,
                "input_sequence": 0,
            },
            HAND_RIGHT: {
                "pose_data": None,
                "button_data": None,
                "gripper_pose": None,
                "gripper_tau": None,
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
                "gripper_pose": None,
                "gripper_tau": None,
            },
            "slave": {
                "ip": None,
                "control_host": None,
                "arm_index": None,
                "hand_binding": None,
                "connected": False,
                "gripper_pose": None,
                "gripper_tau": None,
            },
        }

        # 只保留一个锁用于状态管理（低频操作）
        self.status_lock = threading.Lock()
