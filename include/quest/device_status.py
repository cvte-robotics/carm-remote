"""Quest 头盔与左右手柄电量：通过 ADB dumpsys 读取并周期发布为设备状态项。

OpenXR 运行时不向应用暴露手柄电量，因此由已持有 ADB 连接的本模块直接读取系统
服务：头盔电量来自 ``dumpsys battery``，手柄电量来自 ``dumpsys OVRRemoteService``。
后者是 Quest 系统内部格式，解析只做宽松匹配，读不到时对应项为 None 而不报错。
"""

import logging
import re
import threading
from typing import Callable, Dict, Optional, Tuple

from carm_teleop_input_contracts import DeviceStatusItem

from .adb import run_adb
from .constants import QUEST_DEVICE_STATUS_ADB_TIMEOUT, QUEST_DEVICE_STATUS_INTERVAL


HEADSET_BATTERY_ID = "headset_battery"
LEFT_CONTROLLER_BATTERY_ID = "left_controller_battery"
RIGHT_CONTROLLER_BATTERY_ID = "right_controller_battery"
_LABELS = {
    HEADSET_BATTERY_ID: {"zh-CN": "头盔", "en-US": "Headset"},
    LEFT_CONTROLLER_BATTERY_ID: {"zh-CN": "左手柄", "en-US": "Left controller"},
    RIGHT_CONTROLLER_BATTERY_ID: {"zh-CN": "右手柄", "en-US": "Right controller"},
}

_LEVEL_RE = re.compile(r"^\s*level:\s*(\d+)\s*$", re.MULTILINE)
_SCALE_RE = re.compile(r"^\s*scale:\s*(\d+)\s*$", re.MULTILINE)
_TYPE_RE = re.compile(r"\bType:\s*(Left|Right)\b", re.IGNORECASE)
_BATTERY_RE = re.compile(r"\bBattery:\s*(\d+)\s*%")
_EXTERNAL_STATUS_RE = re.compile(r"\bExternalStatus:\s*([A-Z_]+)")


def parse_headset_battery(text: str) -> Optional[int]:
    """从 ``dumpsys battery`` 输出解析头盔电量百分比；无法解析时返回 None。"""
    level = _LEVEL_RE.search(text or "")
    if level is None:
        return None
    scale = _SCALE_RE.search(text)
    scale_value = int(scale.group(1)) if scale else 100
    if scale_value <= 0:
        return None
    return max(0, min(100, round(int(level.group(1)) * 100 / scale_value)))


def parse_controller_batteries(text: str) -> Dict[str, Tuple[int, bool]]:
    """从 ``dumpsys OVRRemoteService`` 输出解析左右手柄电量。

    每个已配对手柄占一行 ``Paired device: ... Type: Left ... Battery: 90% ...
    ExternalStatus: CONNECTED_ACTIVE ...``。ExternalStatus 不是 ``CONNECTED*``
    时（如 SEARCHING）系统给出的是最后已知电量，标记为 stale。同一手有多条
    配对记录时优先取在线的一条。

    Returns:
        ``{"left"/"right": (电量百分比, stale)}``；缺失的手不出现在结果中。
    """
    result: Dict[str, Tuple[int, bool]] = {}
    for line in (text or "").splitlines():
        hand = _TYPE_RE.search(line)
        battery = _BATTERY_RE.search(line)
        if hand is None or battery is None:
            continue
        external = _EXTERNAL_STATUS_RE.search(line)
        stale = external is None or not external.group(1).startswith("CONNECTED")
        key = hand.group(1).lower()
        if key in result and (stale or not result[key][1]):
            continue
        result[key] = (max(0, min(100, int(battery.group(1)))), stale)
    return result


def _adb_shell_stdout(serial: str, service: str) -> Optional[str]:
    """执行 ``adb shell dumpsys <service>``；失败或超时返回 None。"""
    try:
        result = run_adb(
            ["-s", serial, "shell", "dumpsys", service],
            timeout=QUEST_DEVICE_STATUS_ADB_TIMEOUT,
        )
    except Exception as exc:
        logging.debug("[QuestDeviceStatus] dumpsys %s failed: %s", service, exc)
        return None
    if result.returncode != 0:
        logging.debug(
            "[QuestDeviceStatus] dumpsys %s exited %s: %s",
            service,
            result.returncode,
            (result.stderr or "").strip(),
        )
        return None
    return result.stdout


def read_quest_device_status(
    serial: str,
    shell: Callable[[str, str], Optional[str]] = _adb_shell_stdout,
) -> Tuple[DeviceStatusItem, ...]:
    """读取一次头盔与左右手柄电量，按固定顺序返回三项（读不到的项值为 None）。"""
    headset_text = shell(serial, "battery")
    controller_text = shell(serial, "OVRRemoteService")
    headset = parse_headset_battery(headset_text) if headset_text is not None else None
    controllers = (
        parse_controller_batteries(controller_text) if controller_text is not None else {}
    )
    left_value, left_stale = controllers.get("left", (None, False))
    right_value, right_stale = controllers.get("right", (None, False))
    return (
        DeviceStatusItem(HEADSET_BATTERY_ID, _LABELS[HEADSET_BATTERY_ID], headset, "%"),
        DeviceStatusItem(
            LEFT_CONTROLLER_BATTERY_ID, _LABELS[LEFT_CONTROLLER_BATTERY_ID], left_value, "%", left_stale
        ),
        DeviceStatusItem(
            RIGHT_CONTROLLER_BATTERY_ID, _LABELS[RIGHT_CONTROLLER_BATTERY_ID], right_value, "%", right_stale
        ),
    )


class QuestDeviceStatusPoller:
    """一个连接期内周期读取电量并发布的后台线程。

    ``stop`` 只置位不 join：进行中的 ADB 调用最多阻塞到超时，之后的迟到发布由
    发布回调里的连接身份复核丢弃，因此清理路径不必等待它。
    """

    def __init__(
        self,
        serial: str,
        publish: Callable[[Tuple[DeviceStatusItem, ...]], None],
        interval: float = QUEST_DEVICE_STATUS_INTERVAL,
        read: Callable[[str], Tuple[DeviceStatusItem, ...]] = read_quest_device_status,
    ) -> None:
        self._serial = serial
        self._publish = publish
        self._interval = interval
        self._read = read
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="QuestDeviceStatusPoller",
            daemon=True,
        )

    def start(self) -> None:
        """启动轮询线程（立即读取一次，之后每 ``interval`` 秒一次）。"""
        self._thread.start()

    def stop(self) -> None:
        """请求停止；不等待线程退出。"""
        self._stop_event.set()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                items = self._read(self._serial)
                if not self._stop_event.is_set():
                    self._publish(items)
            except Exception:
                logging.exception("[QuestDeviceStatus] poll failed")
            self._stop_event.wait(self._interval)
