"""Quest 的 ADB 设备发现、授权恢复与应用启动。

只封装外部 ``adb`` 命令；失败以降级返回值或日志上报，不向上抛异常。
"""

import logging
import subprocess
import time
from typing import Any, Dict, List, Optional

from .constants import (
    ADB_DEVICE_LIST_TIMEOUT, ADB_RECOVERY_POLL_INTERVAL_SECONDS,
    ADB_RECOVERY_TIMEOUT, ADB_RESTART_WAIT_TIMEOUT, QUEST_ACTIVITY, QUEST_PACKAGE,
)

def run_adb(args: List[str], timeout: float = 10.0) -> subprocess.CompletedProcess:
    """执行 adb 命令并返回结果（不检查返回码）。

    Args:
        args: 不含 ``adb`` 本身的参数列表。
        timeout: 子命令超时（秒）。

    Returns:
        已完成的进程结果。
    """
    return subprocess.run(
        ["adb", *args],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def list_adb_devices(timeout: float = ADB_DEVICE_LIST_TIMEOUT) -> List[Dict[str, Any]]:
    """列出所有 ADB 设备，返回结构化设备列表。

    Args:
        timeout: ``adb devices -l`` 的超时（秒）。

    Returns:
        每项含 serial/state/attrs/line；失败或超时返回空列表，不抛异常。
    """
    try:
        result = run_adb(["devices", "-l"], timeout=timeout)
    except Exception as e:
        logging.warning(f"[Quest] adb device listing failed: {e}")
        return []

    if result.returncode != 0:
        logging.warning(f"[Quest] adb devices failed: {result.stderr.strip()}")
        return []

    devices = []
    for raw_line in result.stdout.splitlines()[1:]:
        line = raw_line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue

        serial = parts[0]
        state = parts[1]
        attrs: Dict[str, str] = {}
        for token in parts[2:]:
            if ":" not in token:
                continue
            key, value = token.split(":", 1)
            attrs[key] = value

        devices.append({
            "serial": serial,
            "state": state,
            "attrs": attrs,
            "line": line,
        })
    return devices


def is_quest_adb_device(device: Dict[str, Any]) -> bool:
    """判定给定 ADB 设备是否为 Quest 设备。

    Args:
        device: ``list_adb_devices`` 产出的设备项。

    Returns:
        可识别为 Quest 时为 True。
    """
    attrs = device.get("attrs", {})
    candidates = [
        device.get("line", ""),
        attrs.get("model", ""),
        attrs.get("product", ""),
        attrs.get("device", ""),
    ]
    lowered = " ".join(candidates).lower()
    if any(token in lowered for token in ("quest", "oculus", "meta")):
        return True

    model = attrs.get("model", "").replace("_", " ").strip().lower()
    if model.startswith("q") and "3" in model:
        return True

    return False


def _quest_devices_in_device_state(devices: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """从设备清单中筛出处于 device 状态且可识别为 Quest 的设备。"""
    return [
        device
        for device in devices
        if device["state"] == "device" and is_quest_adb_device(device)
    ]


def _wait_for_quest_device_state(timeout: float) -> List[Dict[str, Any]]:
    """轮询 ADB，直到有 Quest 进入 device 状态或恢复超时。

    Args:
        timeout: 本次轮询的总预算（秒）。

    Returns:
        最近一次查到的设备清单（可能为空）。
    """
    deadline = time.monotonic() + timeout
    devices: List[Dict[str, Any]] = []

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return devices

        devices = list_adb_devices(timeout=min(ADB_DEVICE_LIST_TIMEOUT, remaining))
        if _quest_devices_in_device_state(devices):
            return devices

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return devices
        time.sleep(min(ADB_RECOVERY_POLL_INTERVAL_SECONDS, remaining))


def restart_adb_service() -> bool:
    """重启 ADB 服务；kill-server 失败也会尝试 start-server。

    Returns:
        start-server 成功执行时为 True。
    """
    stop_result = None
    try:
        stop_result = run_adb(["kill-server"], timeout=ADB_RECOVERY_TIMEOUT)
    except Exception as e:
        logging.warning(f"[Quest] adb kill-server failed, continuing with start-server: {e}")

    try:
        start_result = run_adb(["start-server"], timeout=ADB_RECOVERY_TIMEOUT)
    except Exception as e:
        logging.warning(f"[Quest] adb start-server failed: {e}")
        return False

    if stop_result is not None and stop_result.returncode != 0:
        logging.warning(
            "[Quest] adb kill-server failed with code %s: %s",
            stop_result.returncode,
            (stop_result.stderr or stop_result.stdout).strip(),
        )

    if start_result.returncode != 0:
        logging.warning(
            "[Quest] adb start-server failed with code %s: %s",
            start_result.returncode,
            (start_result.stderr or start_result.stdout).strip(),
        )
        return False

    return True


def get_adb_device_info(serial: str) -> Optional[Dict[str, Any]]:
    """返回指定 serial 的设备信息；查询失败与不存在都返回 None。"""
    for device in list_adb_devices():
        if device["serial"] == serial:
            return device
    return None


def scan_quest_devices() -> List[Dict[str, str]]:
    """扫描已连接的 Quest 设备，必要时重启 ADB 并等待授权恢复。

    Returns:
        每项为 ``{"name": model, "address": serial}``；无可用设备时为空列表。
    """
    devices = list_adb_devices()
    quest_devices_in_device_state = _quest_devices_in_device_state(devices)

    if quest_devices_in_device_state:
        devices = quest_devices_in_device_state
    elif any(device["state"] == "unauthorized" for device in devices):
        # 未授权设备往往不上报 model/product 元数据，先当作候选，授权后再识别。
        logging.info("[Quest] Found unauthorized ADB device, restarting adb service once")
        restart_succeeded = restart_adb_service()
        if not restart_succeeded:
            logging.warning(
                "[Quest] adb restart command did not report success; polling device state before failing"
            )
        logging.info(
            "[Quest] Waiting up to %.1fs for a Quest to enter adb device state after restart",
            ADB_RESTART_WAIT_TIMEOUT,
        )
        devices = _wait_for_quest_device_state(ADB_RESTART_WAIT_TIMEOUT)

    devices = [
        device
        for device in devices
        if device["state"] == "device" and is_quest_adb_device(device)
    ]

    quest_devices = []
    for device in devices:
        model = device["attrs"].get("model", "Quest_3S").replace("_", " ")
        quest_devices.append({"name": model, "address": device["serial"]})

    return quest_devices


def launch_quest_app(serial: str) -> bool:
    """启动 Quest 遥操 App（先尝试 explicit intent，失败则用 monkey）。

    Returns:
        任一方式成功启动时为 True。
    """
    explicit = run_adb(
        ["-s", serial, "shell", "am", "start", "-n", f"{QUEST_PACKAGE}/{QUEST_ACTIVITY}"],
        timeout=10.0,
    )
    if explicit.returncode == 0:
        return True

    fallback = run_adb(
        ["-s", serial, "shell", "monkey", "-p", QUEST_PACKAGE, "1"],
        timeout=10.0,
    )
    return fallback.returncode == 0


# ==========================================
# Quest 帧解析
# ==========================================
