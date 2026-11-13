"""Quest provider 共享常量：端口、超时与协议版本（超时单位为秒）。"""

import re
from types import MappingProxyType
from typing import NamedTuple

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
QUEST_CAPABILITY_INTENT_V1 = "intent_v1"
QUEST_CAPABILITY_SESSION_SETUP_V1 = "session_setup_v1"
QUEST_CAPABILITY_WAIST_CONTROL_V1 = "waist_control_v1"
QUEST_SUPPORTED_CAPABILITIES = frozenset({
    QUEST_CAPABILITY_INTENT_V1,
    QUEST_CAPABILITY_SESSION_SETUP_V1,
    QUEST_CAPABILITY_WAIST_CONTROL_V1,
})
QUEST_HEARTBEAT_INTERVAL = 0.5
QUEST_HANDSHAKE_TIMEOUT = 5.0
QUEST_RX_TIMEOUT = 5.0
QUEST_EXIT_TIMEOUT = 5.0
QUEST_LINE_LIMIT = 64 * 1024
QUEST_SEND_QUEUE_LIMIT = 32
QUEST_PROTOCOL_PREFIX = b"@carm-control/2 "


# ==========================================


class QuestIntentField(NamedTuple):
    """功能线字段定义；core 标记旧 intent_v1 APK 必需的固定手臂集合。"""

    channel: str
    sample_field: str
    value_type: str
    core: bool = False


QUEST_INTENT_NAME_PATTERN = re.compile(r"[a-z0-9_]+")
# 唯一已知功能字段表：校验和解析共用。新增可选功能不扩大旧 APK 的默认声明。
# event 的线计数由 module 转成对应功能的 lease 内累计序号。
QUEST_INTENT_FIELDS = MappingProxyType({
    "im": QuestIntentField("right", "motion_engaged", "toggle", core=True),
    "il": QuestIntentField("right", "rotation_locked", "toggle", core=True),
    "ie": QuestIntentField("right", "end_effector", "analog", core=True),
    "ih": QuestIntentField("right", "home_sequence", "event", core=True),
    "lim": QuestIntentField("left", "motion_engaged", "toggle", core=True),
    "lil": QuestIntentField("left", "rotation_locked", "toggle", core=True),
    "lie": QuestIntentField("left", "end_effector", "analog", core=True),
    "lih": QuestIntentField("left", "home_sequence", "event", core=True),
    "wf": QuestIntentField("body", "waist_follow_sequence", "event"),
    "wh": QuestIntentField("body", "waist_home_sequence", "event"),
    "ws": QuestIntentField("body", "waist_stick", "vector2"),
    "ia": QuestIntentField("right", "arm_angle_stick", "vector2"),
    "lia": QuestIntentField("left", "arm_angle_stick", "vector2"),
})
QUEST_CORE_INTENTS = frozenset(name for name, field in QUEST_INTENT_FIELDS.items() if field.core)
