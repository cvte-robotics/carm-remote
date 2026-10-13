from __future__ import annotations

import ipaddress
import logging
from pathlib import Path
from typing import Any

from PyQt6.QtCore import QObject, QTimer, pyqtSignal

from .backend_process import BackendProcess
from .http_client import HttpClient
from .models import ArmProbe, Device, ShutdownState, Snapshot, UiState
from .runtime_paths import backend_log_path, backend_root, log_dir, ui_log_path
from .sse_client import SseClient


class ViewModel(QObject):
    state_changed = pyqtSignal(object)
    snapshot_changed = pyqtSignal(object)
    toast = pyqtSignal(str)
    closed = pyqtSignal()

    def __init__(self, parent=None, *, end_effector: str = "carm"):
        super().__init__(parent)
        self.state = UiState()
        self.snapshot = Snapshot()
        self.log_directory = log_dir()
        self.ui_log_file = ui_log_path()
        self.backend_log_file = backend_log_path()
        self.http = HttpClient(parent=self)
        self.sse = SseClient(parent=self)
        self.backend = BackendProcess(
            backend_root(),
            self.log_directory,
            end_effector=end_effector,
            parent=self,
        )
        self.http.response.connect(self._response)
        self.sse.message_received.connect(self._event)
        self.sse.disconnected.connect(self._sse_disconnected)
        self.backend.failed.connect(self._backend_error)
        self.backend.exited.connect(self._backend_exited)
        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(self._health)
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.setInterval(1000)
        self.heartbeat_timer.timeout.connect(lambda: self.http.post("/api/desktop/heartbeat"))
        self.shutdown_timer = QTimer(self)
        self.shutdown_timer.setSingleShot(True)
        self.shutdown_timer.timeout.connect(self._shutdown_backend)
        self.stop_retry_timer = QTimer(self)
        self.stop_retry_timer.setSingleShot(True)
        self.stop_retry_timer.timeout.connect(self._retry_manual_stop)
        self.shutdown_state = ShutdownState.NONE
        self.pending_connect = None
        self.health_request = None
        self.scan_request = None
        self.session_request = None
        self.connect_request = None
        self.init_request = None
        self.disconnect_request = None
        self.shutdown_request = None
        self.operation_generation = 0
        self.workflow_generation = None
        self.quest_reconnecting = False
        self.stop_retry_attempt = 0

    def start(self) -> None:
        logging.getLogger("desktop.ui").info(
            "Starting desktop workflow; logs=%s", self.log_directory
        )
        self.backend.start()
        self.health_timer.start(250)

    def _health(self) -> None:
        if self.health_request is None and self.state.name == "BackendStarting":
            self.health_request = self.http.get("/api/desktop/health")

    def _set(self, name: str, message: str | None = None) -> None:
        previous = self.state.name
        self.state.name = name
        if message:
            self.state.message = message
        logging.getLogger("desktop.state").info(
            "%s -> %s: %s", previous, name, self.state.message
        )
        self.state_changed.emit(self.state)

    def _response(self, path: str, request_id: int, data: dict, error: str) -> None:
        if path == "/api/desktop/health":
            if self.shutdown_state is not ShutdownState.NONE:
                return
            if request_id != self.health_request:
                return
            self.health_request = None
            backend_pid = int((data.get("data") or {}).get("pid", 0)) if not error else 0
            backend_token = str((data.get("data") or {}).get("instance_token", "")) if not error else ""
            if (
                not error
                and data.get("code", 0) == 1
                and backend_token == self.backend.instance_token()
            ):
                backend_log_file = (data.get("data") or {}).get("log_file")
                if backend_log_file:
                    self.backend_log_file = Path(str(backend_log_file))
                self.health_timer.stop()
                self.heartbeat_timer.start()
                self._set("Idle", "准备就绪")
            elif not error and (backend_pid or backend_token):
                self.health_timer.stop()
                self._set("BackendFailed", "端口 1999 已被其他进程占用")
            return
        if path == "/api/desktop/heartbeat":
            return

        request_attributes = {
            "/api/joystick/scan": "scan_request",
            "/api/joystick/session_config": "session_request",
            "/api/joystick/connect": "connect_request",
            "/api/joystick/init_arm": "init_request",
            "/api/joystick/disconnect": "disconnect_request",
            "/api/desktop/shutdown": "shutdown_request",
        }
        request_attribute = request_attributes.get(path)
        if request_attribute is not None:
            if request_id != getattr(self, request_attribute):
                return
            setattr(self, request_attribute, None)
        if error:
            retrying_manual_stop = bool(
                path == "/api/joystick/disconnect"
                and self.shutdown_state is ShutdownState.NONE
                and self.state.name == "Stopping"
                and self.backend.is_running()
            )
            logging.getLogger("desktop.http").log(
                logging.WARNING if retrying_manual_stop else logging.ERROR,
                "%s failed (request_id=%s): %s", path, request_id, error
            )
            if path == "/api/joystick/session_config":
                self.pending_connect = None
            if path == "/api/joystick/scan":
                self._set("Idle", error)
            elif path in {"/api/joystick/session_config", "/api/joystick/connect"}:
                self.workflow_generation = None
                self._set("ScanReady", error)
            elif path == "/api/joystick/init_arm":
                self._set("Connected", error)
            elif path == "/api/joystick/disconnect":
                if self.shutdown_state is not ShutdownState.NONE:
                    self._shutdown_backend()
                elif retrying_manual_stop:
                    self._schedule_manual_stop_retry()
                    return
                else:
                    self._set("Connected", error)
            elif path == "/api/desktop/shutdown" and self.shutdown_state is not ShutdownState.NONE:
                self.shutdown_state = ShutdownState.WAITING_PROCESS_EXIT
                self._set("ShuttingDown", "后端清理已请求，等待进程退出...")
            self.toast.emit(error)
            return
        message = str(data.get("msg", "请求完成"))
        if path == "/api/joystick/session_config":
            if self.pending_connect is not None:
                device = self.pending_connect
                self.pending_connect = None
                generation = self.workflow_generation
                if generation is None:
                    return
                self.connect_request = self.http.post(
                    "/api/joystick/connect",
                    {
                        "address": device.address,
                        "name": device.name,
                        "_desktop_generation": generation,
                    },
                )
            return
        if path == "/api/joystick/scan":
            rows = data.get("data", []) or []
            self.state.devices = [Device(str(x.get("name", "Quest")), str(x.get("address", ""))) for x in rows if isinstance(x, dict)]
            probe = data.get("arm_probe") or {}
            self.state.probe = ArmProbe(str(probe.get("ip", "")), str(probe.get("arm_name", "")), int(probe.get("arm_dof", 0)), str(probe.get("family", "")))
            self.state.allowed_profiles = [str(x) for x in data.get("allowed_profiles", [])]
            self._set("ScanReady", message)
        elif path == "/api/joystick/connect":
            self.sse.open_stream()
            self._set("Connected", message)
        elif path == "/api/joystick/init_arm":
            self._set("Teleoperating", message)
        elif path == "/api/joystick/disconnect":
            self.stop_retry_timer.stop()
            self.stop_retry_attempt = 0
            self.sse.stop()
            if self.shutdown_state is not ShutdownState.NONE:
                self._shutdown_backend()
            else:
                self.clear_scan_state()
                self._set("Idle", message)
        elif path == "/api/desktop/shutdown":
            self.state.message = "后端清理完成，正在退出..."
            self.state_changed.emit(self.state)
        else:
            self.toast.emit(message)

    def _event(self, payload: dict[str, Any]) -> None:
        self.snapshot = Snapshot(payload.get("status", {}), payload.get("pose"), payload.get("button"), int(payload.get("sequence", -1)))
        self.snapshot_changed.emit(self.snapshot)
        status = self.snapshot.status.get("status")
        transport_state = str((payload.get("transport") or {}).get("state", ""))
        stream_terminal = bool(payload.get("stream_terminal"))
        if stream_terminal and self.state.active:
            self.sse.stop()
            self.clear_scan_state()
            self._set("Idle", "手柄遥操已停止")
        elif transport_state == "disconnected" and self.state.active:
            if not self.quest_reconnecting:
                self.quest_reconnecting = True
                self._set(self.state.name, "Quest 连接中断，等待恢复...")
        elif status in {"connected", "arm_ready"}:
            if self.quest_reconnecting:
                self.quest_reconnecting = False
                messages = {
                    "Connected": "手柄已连接",
                    "Initializing": "正在初始化机械臂...",
                    "Teleoperating": "遥操已恢复",
                }
                if self.state.name in messages:
                    self._set(self.state.name, messages[self.state.name])
            elif self.state.name == "Connected" and self.state.message != "手柄已连接":
                self._set("Connected", "手柄已连接")

    def scan(self, current_ip: str) -> None:
        try:
            ipaddress.IPv4Address(current_ip)
        except ipaddress.AddressValueError:
            logging.getLogger("desktop.ui").warning("Invalid arm IPv4: %s", current_ip)
            self.toast.emit("当前臂 IP 必须是 IPv4 地址")
            return
        self.operation_generation += 1
        self.clear_scan_state()
        self._set("Scanning", "正在扫描设备...")
        self.scan_request = self.http.post(
            "/api/joystick/scan",
            {
                "current_arm_ip": current_ip,
                "_desktop_generation": self.operation_generation,
            },
        )

    def connect(self, device: Device, profile: str, current_ip: str, other_ip: str, side: str, view_side: str) -> None:
        self.operation_generation += 1
        self.workflow_generation = self.operation_generation
        self.profile = profile
        self.current_ip, self.other_ip = current_ip, other_ip
        self.side, self.view_side = side, view_side
        mode, slave = self._init_payload()
        self.pending_connect = device
        self.session_request = self.http.post(
            "/api/joystick/session_config",
            {
                "mode": mode,
                "master_side": side,
                "view_side": view_side,
                "slave_ip": slave,
                "_desktop_generation": self.workflow_generation,
            },
        )
        self._set("Connecting", f"正在连接 {device.name}...")

    def _init_payload(self) -> tuple[str, str | None]:
        if self.profile == "a3_single": return "single", None
        if self.profile == "a3_dual": return "dual", self.other_ip
        return "dual", self.current_ip

    def initialize(self) -> None:
        generation = self.workflow_generation
        if generation is None:
            self.toast.emit("当前连接工作流已失效")
            return
        mode, slave = self._init_payload()
        self._set("Initializing", "正在初始化机械臂...")
        self.init_request = self.http.post(
            "/api/joystick/init_arm",
            {
                "mode": mode,
                "master_ip": self.current_ip,
                "slave_ip": slave,
                "master_side": self.side,
                "_desktop_generation": generation,
            },
        )

    def set_scale(self, value: int) -> None:
        if self.workflow_generation is None:
            return
        self.http.post(
            "/api/joystick/scale",
            {
                "value": value / 100,
                "_desktop_generation": self.workflow_generation,
            },
        )

    def set_view_side(self, value: str) -> None:
        if self.workflow_generation is None:
            return
        self.view_side = value
        self.http.post(
            "/api/joystick/view_side",
            {
                "value": value,
                "_desktop_generation": self.workflow_generation,
            },
        )

    def calibration(self, value: int) -> None:
        if self.workflow_generation is None:
            return
        self.http.post(
            "/api/joystick/yaw",
            {
                "value": value,
                "_desktop_generation": self.workflow_generation,
            },
        )

    def stop(self) -> None:
        self.operation_generation += 1
        self._invalidate_active_workflow()
        self.sse.stop()
        self.stop_retry_timer.stop()
        self.stop_retry_attempt = 0
        self._set("Stopping", "正在停止手柄遥操...")
        self._retry_manual_stop()

    def _retry_manual_stop(self) -> None:
        if (
            self.shutdown_state is not ShutdownState.NONE
            or self.state.name != "Stopping"
            or not self.backend.is_running()
        ):
            return
        self.disconnect_request = self.http.post(
            "/api/joystick/disconnect",
            {
                "stop_mode": "home",
                "reason": "manual-button",
                "_desktop_generation": self.operation_generation,
            },
        )

    def _schedule_manual_stop_retry(self) -> None:
        self.stop_retry_attempt += 1
        retry_delays_ms = (1000, 2000, 5000)
        delay_ms = retry_delays_ms[
            min(self.stop_retry_attempt - 1, len(retry_delays_ms) - 1)
        ]
        self._set(
            "Stopping",
            "资源清理未完成，%s 秒后重试..." % (delay_ms // 1000),
        )
        self.stop_retry_timer.start(delay_ms)

    def shutdown(self) -> None:
        if self.shutdown_state is not ShutdownState.NONE:
            return
        self.shutdown_state = ShutdownState.STOPPING_SESSION
        if not self.backend.is_running():
            self._finish_shutdown()
            return
        self.operation_generation += 1
        self._invalidate_active_workflow()
        self.stop_retry_timer.stop()
        self.stop_retry_attempt = 0
        self._set("ShuttingDown", "正在关闭...")
        self.sse.stop()
        self.heartbeat_timer.stop()
        self.disconnect_request = self.http.post(
            "/api/joystick/disconnect",
            {
                "stop_mode": "hold",
                "reason": "window-close",
                "_desktop_generation": self.operation_generation,
            },
        )

        self.shutdown_timer.start(1200)

    def _shutdown_backend(self) -> None:
        if self.shutdown_state is not ShutdownState.STOPPING_SESSION:
            return
        self.shutdown_state = ShutdownState.REQUESTING_BACKEND_SHUTDOWN
        self.shutdown_timer.stop()
        self.heartbeat_timer.stop()
        self.shutdown_request = self.http.post("/api/desktop/shutdown")

    def _invalidate_active_workflow(self) -> None:
        self.workflow_generation = None
        self.pending_connect = None
        self.scan_request = None
        self.session_request = None
        self.connect_request = None
        self.init_request = None

    def clear_scan_state(self) -> None:
        self.quest_reconnecting = False
        self.state.devices = []
        self.state.allowed_profiles = []
        self.state.probe = None

    def _finish_shutdown(self) -> None:
        if self.shutdown_state is ShutdownState.COMPLETE:
            return
        self.shutdown_state = ShutdownState.COMPLETE
        self._set("ShuttingDown", "后端未运行，窗口可以关闭")
        self.closed.emit()

    def _sse_disconnected(self, message: str) -> None:
        logging.getLogger("desktop.sse").warning("SSE disconnected: %s", message)
        self.toast.emit(f"手柄数据流已断开：{message}")

    def _backend_error(self, message: str) -> None:
        logging.getLogger("desktop.backend_process").error(message)
        if self.state.name != "ShuttingDown":
            self.toast.emit(f"{message}；日志：{self.backend_log_file}")

    def _backend_exited(self, exit_code: int, exit_status) -> None:
        self.stop_retry_timer.stop()
        if self.shutdown_state is not ShutdownState.NONE:
            if exit_code == 0:
                self._set("ShuttingDown", "后端已退出，等待窗口关闭")
            else:
                self._set("ShuttingDown", "后端已异常退出，hold 未被证明")
            self.shutdown_state = ShutdownState.COMPLETE
            self.closed.emit()
            return
        self.health_timer.stop()
        self.heartbeat_timer.stop()
        self.sse.stop()
        if exit_code == 0:
            message = f"后端已正常退出；日志：{self.backend_log_file}"
            logging.getLogger("desktop.backend_process").info(message)
            self._set("BackendStopped", message)
            return
        diagnostic = self.backend.last_diagnostic()
        message = f"后端进程异常退出（code={exit_code}, status={exit_status}）"
        if diagnostic:
            message += f"：{diagnostic[-240:]}"
        message += f"；日志：{self.backend_log_file}"
        logging.getLogger("desktop.backend_process").error(message)
        self._set("BackendFailed", message)
