"""Quest 直连遥操的板端持久配置。"""

import json
import ipaddress
import logging
import os
import tempfile
from pathlib import Path


DEFAULT_PATH = "/var/www/backend/params/joystick/quest_direct.json"
DEFAULT_CONFIG = {
    "version": 1,
    "a3": {"mode": "single", "hand": "right", "other_arm_ip": None},
    "scale": 0.4,
    "yaw": -90,
}


def config_path() -> Path:
    """返回环境变量覆盖后的配置路径。"""
    return Path(os.environ.get("CARM_QUEST_DIRECT_CONFIG", DEFAULT_PATH))


def _copy_default() -> dict:
    return json.loads(json.dumps(DEFAULT_CONFIG))


def _valid_config(value) -> bool:
    if not isinstance(value, dict) or value.get("version") != 1:
        return False
    a3 = value.get("a3")
    if not isinstance(a3, dict) or a3.get("mode") not in ("single", "dual"):
        return False
    if a3.get("hand") not in ("left", "right"):
        return False
    other = a3.get("other_arm_ip")
    if other is not None:
        if not isinstance(other, str):
            return False
        try:
            ipaddress.IPv4Address(other)
        except ValueError:
            return False
    scale, yaw = value.get("scale"), value.get("yaw")
    return (
        isinstance(scale, (int, float)) and not isinstance(scale, bool)
        and 0.1 <= float(scale) <= 2.0
        and isinstance(yaw, int) and not isinstance(yaw, bool) and -180 <= yaw <= 180
    )


def load_config(path=None) -> dict:
    """读取配置；文件缺失或非法时记录警告并返回默认值。"""
    target = Path(path) if path is not None else config_path()
    try:
        value = json.loads(target.read_text(encoding="utf-8"))
        if _valid_config(value):
            result = _copy_default()
            result["a3"].update(value["a3"])
            result["scale"] = float(value["scale"])
            result["yaw"] = value["yaw"]
            return result
        raise ValueError("字段校验失败")
    except Exception as exc:
        logging.warning("[QuestDirectConfig] 使用默认配置：%s (%s)", target, exc)
        return _copy_default()


def save_config(value: dict, path=None) -> dict:
    """校验并原子写入配置，返回规范化后的副本。

    Args:
        value: 配置字典。
        path: 可选写入路径，缺省使用 ``CARM_QUEST_DIRECT_CONFIG``。
    Raises:
        ValueError: 配置字段不合法。
    """
    if not _valid_config(value):
        raise ValueError("Quest 直连配置不合法")
    target = Path(path) if path is not None else config_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    normalized = _copy_default()
    normalized["a3"].update(value["a3"])
    normalized["scale"] = float(value["scale"])
    normalized["yaw"] = int(value["yaw"])
    fd, temp_name = tempfile.mkstemp(prefix=".quest_direct.", dir=str(target.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(normalized, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, str(target))
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    return normalized
