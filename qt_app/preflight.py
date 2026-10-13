from __future__ import annotations

import importlib.metadata
import importlib.util
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys

MIN_PYTHON = (3, 11)
MAX_PYTHON = (3, 13)
CARM_VERSION = "0.1.20260819"
CARM_REVISION = "3f45eb487170e151d164d3b110a2b8580c48daf4"
REQUIRED_LOOPBACK_PORTS = ((1999, "桌面 HTTP/SSE"), (9999, "Quest TCP"))
REQUIRED_QUEST_FILES = (
    "__init__.py",
    "adb.py",
    "config.py",
    "constants.py",
    "device_status.py",
    "module.py",
    "parser.py",
    "policy.py",
    "receiver.py",
)
REQUIRED_REPOSITORY_FILES = (
    "config/joystick_config.json",
    "include/carm_teleop_input_contracts.py",
    "qt_app/main.py",
    "qt_app/app/__init__.py",
    "qt_app/app/backend_process.py",
    "qt_app/app/http_client.py",
    "qt_app/app/logging_config.py",
    "qt_app/app/main_window.py",
    "qt_app/app/models.py",
    "qt_app/app/runtime_paths.py",
    "qt_app/app/sse_client.py",
    "qt_app/app/view_model.py",
    "server/__init__.py",
    "server/api/joystick_routes.py",
    "server/desktop/__init__.py",
    "server/desktop/app.py",
    "server/desktop/runtime.py",
    "server/desktop/watchdog.py",
    "server/basic/joystick/__init__.py",
    "server/basic/joystick/core/__init__.py",
    "server/basic/joystick/core/command_controller.py",
    "server/basic/joystick/core/models.py",
    "server/basic/joystick/core/pose_math.py",
    "server/basic/joystick/runtime/__init__.py",
    "server/basic/joystick/runtime/config.py",
    "server/basic/joystick/runtime/context.py",
    "server/basic/joystick/runtime/dependencies.py",
    "server/basic/joystick/runtime/paths.py",
    "server/basic/joystick/session/__init__.py",
    "server/basic/joystick/session/arm.py",
    "server/basic/joystick/session/controller.py",
    "server/basic/joystick/session/state.py",
    "server/basic/joystick/sources/__init__.py",
    "server/basic/joystick/sources/adapters.py",
    "server/basic/joystick/sources/registry.py",
)


def fail(message: str) -> None:
    raise RuntimeError(message)


def check_import(module: str, label: str | None = None) -> None:
    if importlib.util.find_spec(module) is None:
        fail(f"缺少 Python 依赖: {label or module}。请重新运行启动脚本安装 requirements")


def check_carm() -> None:
    check_import("carm", "Carm SDK")
    try:
        from carm import Carm
    except Exception as exc:
        fail(f"Carm SDK 导入失败: {exc}")
    if Carm is None or not callable(Carm):
        fail("Carm SDK API 不兼容: 缺少可调用 Carm")
    try:
        version = importlib.metadata.version("carm")
    except importlib.metadata.PackageNotFoundError:
        fail(f"Carm SDK 未安装固定基线 {CARM_VERSION}")
    if version < CARM_VERSION:
        fail(f"Carm SDK 版本不匹配: {version}，需要大于等于 {CARM_VERSION} (revision {CARM_REVISION})")


def check_quest() -> None:
    root = Path(__file__).resolve().parents[1] / "include" / "quest"
    missing = [name for name in REQUIRED_QUEST_FILES if not (root / name).is_file()]
    if missing:
        fail(f"仓库 Quest 模块缺失: {', '.join(missing)}")


def check_repository_layout() -> None:
    root = Path(__file__).resolve().parents[1]
    missing = [name for name in REQUIRED_REPOSITORY_FILES if not (root / name).is_file()]
    if missing:
        fail(f"独立仓库运行文件缺失: {', '.join(missing)}")


def check_quest_only_import() -> None:
    root = Path(__file__).resolve().parents[1]
    probe = """
import os
from pathlib import Path
import sys
import tempfile

root = Path(sys.argv[1]).resolve()
os.environ["CARM_JOYSTICK_PROVIDERS"] = "quest"
os.environ["CARM_DESKTOP_LOG_DIR"] = tempfile.mkdtemp(prefix="carm-desktop-preflight-")
sys.path.insert(0, str(root / "server"))
sys.path.insert(0, str(root / "qt_app"))
import app.main_window
import app.view_model
import basic.joystick
import api.joystick_routes
import desktop.app
import desktop.runtime

unexpected = [
    name for name in sys.modules
    if name == "surreal" or name.startswith("surreal.")
    or name == "bleak" or name.startswith("bleak.")
]
if unexpected:
    raise RuntimeError(f"Quest-only runtime loaded excluded modules: {unexpected}")
"""
    result = subprocess.run(
        [sys.executable, "-c", probe, str(root)],
        cwd=root,
        capture_output=True,
        text=True,
        timeout=15,
    )
    if result.returncode != 0:
        details = (result.stderr or result.stdout).strip()
        fail(f"Quest-only 后端导入失败: {details}")


def check_adb() -> None:
    adb = shutil.which("adb")
    if not adb:
        fail("未找到 adb。请安装 adb 并将其加入 PATH；没有连接 Quest 不属于启动失败")
    try:
        result = subprocess.run([adb, "version"], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        fail(f"adb version 检查失败: {exc}")
    if result.returncode != 0:
        fail(f"adb version 返回错误: {result.stderr.strip()}")


def check_loopback_ports() -> None:
    for port, label in REQUIRED_LOOPBACK_PORTS:
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe.bind(("127.0.0.1", port))
        except OSError as exc:
            fail(
                f"本机端口 127.0.0.1:{port} ({label}) 不可用: {exc}。"
                "请停止占用该端口的 Windows 或 WSL 遥操进程后重试"
            )
        finally:
            probe.close()


def validate() -> None:
    if sys.version_info < MIN_PYTHON:
        fail(f"Python 版本过低: {sys.version.split()[0]}，需要 >= {MIN_PYTHON[0]}.{MIN_PYTHON[1]}")
    if sys.version_info >= MAX_PYTHON:
        fail(f"Python 版本过高: {sys.version.split()[0]}，需要 < {MAX_PYTHON[0]}.{MAX_PYTHON[1]}")
    for module, label in (("PyQt6.QtCore", "PyQt6 QtCore"), ("PyQt6.QtNetwork", "PyQt6 QtNetwork"), ("PyQt6.QtWidgets", "PyQt6 QtWidgets"), ("flask", "Flask"), ("werkzeug", "Werkzeug"), ("numpy", "NumPy"), ("scipy.spatial.transform", "SciPy Rotation"), ("websocket", "websocket-client"), ("zeroconf", "zeroconf")):
        check_import(module, label)
    check_carm()
    check_adb()
    check_loopback_ports()
    check_repository_layout()
    check_quest()
    check_quest_only_import()


def main() -> int:
    validate()
    print("Desktop preflight passed")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"Desktop preflight failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
