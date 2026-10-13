"""Quest provider 共享常量：端口、超时与协议版本（超时单位为秒）。"""

# ==========================================
# Quest 常量
# ==========================================
QUEST_TCP_HOST = "127.0.0.1"
QUEST_TCP_PORT = 9999
QUEST_FIRST_FRAME_TIMEOUT = 20.0
ADB_RECOVERY_TIMEOUT = 8.0
ADB_DEVICE_LIST_TIMEOUT = 8.0
ADB_RECOVERY_POLL_INTERVAL_SECONDS = 0.5
ADB_RESTART_WAIT_TIMEOUT = 8.0
# 头盔/手柄电量轮询：系统电量变化以 1% 为单位，无需高频读取。
QUEST_DEVICE_STATUS_INTERVAL = 15.0
QUEST_DEVICE_STATUS_ADB_TIMEOUT = 5.0
QUEST_PACKAGE = "com.liuyi.armcontroller"
QUEST_ACTIVITY = "com.unity3d.player.UnityPlayerGameActivity"

LEFT_PREFIX = "l"
RIGHT_PREFIX = "r"


# Quest control v2 协议常量。
QUEST_PROTOCOL_VERSION = 2
QUEST_HEARTBEAT_INTERVAL = 0.5
QUEST_HANDSHAKE_TIMEOUT = 5.0
QUEST_RX_TIMEOUT = 5.0
QUEST_EXIT_TIMEOUT = 5.0
QUEST_LINE_LIMIT = 64 * 1024
QUEST_SEND_QUEUE_LIMIT = 32
QUEST_PROTOCOL_PREFIX = b"@carm-control/2 "


# ==========================================
