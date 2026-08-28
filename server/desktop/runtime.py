"""Quest-only desktop runtime lifecycle and cleanup orchestration."""

from __future__ import annotations

from enum import Enum
import ipaddress
import logging
import threading
import time
from typing import Callable, Dict, Tuple, TypeVar

from basic.joystick import Carm, JoystickController
from basic.joystick.session.arm import build_arm_bindings, desktop_arm_endpoint

from .watchdog import HeartbeatWatchdog


ARM_PROBE_TIMEOUT = 5.0
OperationResult = TypeVar("OperationResult")


def normalize_ipv4(value: object) -> str:
    text = str(value or "").strip()
    try:
        return str(ipaddress.IPv4Address(text))
    except ipaddress.AddressValueError as exc:
        raise ValueError("当前臂 IP 必须是有效 IPv4 地址") from exc


def build_desktop_arm_bindings(mode, master_ip, master_side, slave_ip=None):
    normalized_master = normalize_ipv4(master_ip)
    normalized_slave = normalize_ipv4(slave_ip) if slave_ip is not None else None
    return build_arm_bindings(mode, normalized_master, master_side, normalized_slave, endpoint_resolver=desktop_arm_endpoint)


def probe_current_arm(ip: str, timeout: float = ARM_PROBE_TIMEOUT) -> Dict[str, object]:
    if Carm is None:
        raise RuntimeError("Carm SDK 不可用")
    address = normalize_ipv4(ip)
    carm = None
    try:
        carm = Carm(addr=address, arm_index=0, port=8090)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            arm_name = str(carm.arm_name or "").strip()
            arm_dof = int(carm.arm_dof or 0)
            if arm_name and arm_dof > 0:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("读取机械臂型号超时")
        upper_name = arm_name.upper()
        if "A3" in upper_name and arm_dof == 6:
            family, allowed_profiles = "A3", ["a3_single", "a3_dual"]
        elif "D3" in upper_name and arm_dof == 7:
            family, allowed_profiles = "D3", ["d3_dual"]
        else:
            raise RuntimeError(f"不支持的机械臂型号: {arm_name} ({arm_dof}DOF)")
        return {"ip": address, "arm_name": arm_name, "arm_dof": arm_dof, "family": family, "allowed_profiles": allowed_profiles}
    finally:
        if carm is not None:
            carm.disconnect()


class Lifecycle(str, Enum):
    RUNNING = "running"
    SHUTDOWN_REQUESTED = "shutdown_requested"
    CLEANING = "cleaning"
    RETRY_WAIT = "retry_wait"
    CLEANED = "cleaned"
    SERVER_STOPPING = "server_stopping"
    STOPPED = "stopped"


class DesktopRuntime:
    def __init__(self) -> None:
        self.controller = JoystickController(arm_binding_builder=build_desktop_arm_bindings)
        self._server = None
        self._state_lock = threading.RLock()
        self._operation_lock = threading.RLock()
        self._lifecycle = Lifecycle.RUNNING
        self._cleanup_complete = False
        self._cleanup_attempt = 0
        self._cleanup_thread = None
        self._retry_thread = None
        self._cleanup_reason = ""
        self._cleanup_stop_mode = "hold"
        self._latest_operation_generation = 0
        self.watchdog = HeartbeatWatchdog(self._handle_heartbeat_timeout)

    def attach_server(self, server) -> None:
        self._server = server

    def start(self) -> None:
        self.watchdog.start()

    def heartbeat(self) -> None:
        with self._state_lock:
            if self._lifecycle is Lifecycle.RUNNING:
                self.watchdog.touch()

    def lifecycle_status(self) -> dict[str, object]:
        with self._state_lock:
            return {
                "lifecycle": self._lifecycle.value,
                "cleanup_complete": self._cleanup_complete,
                "retryable": not self._cleanup_complete and self._lifecycle is not Lifecycle.STOPPED,
                "attempt": self._cleanup_attempt,
            }

    def run_operation(self, body: Dict[str, object], operation: Callable[[], OperationResult]) -> Tuple[bool, OperationResult | None]:
        value = body.get("_desktop_generation")
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return False, None
        with self._state_lock:
            if self._lifecycle is not Lifecycle.RUNNING:
                return False, None
            if value < self._latest_operation_generation:
                return False, None
            self._latest_operation_generation = value
        with self._operation_lock:
            with self._state_lock:
                if (
                    self._lifecycle is not Lifecycle.RUNNING
                    or value < self._latest_operation_generation
                ):
                    return False, None
            return True, operation()

    def scan(self, controller, body: Dict[str, object]) -> Dict[str, object]:
        with self._operation_lock:
            with self._state_lock:
                if self._lifecycle is not Lifecycle.RUNNING:
                    raise RuntimeError("桌面后端正在关闭")
            arm_probe = probe_current_arm(normalize_ipv4(body.get("current_arm_ip")))
            devices = controller.scan_devices()
        return {"data": devices, "arm_probe": {key: arm_probe[key] for key in ("ip", "arm_name", "arm_dof", "family")}, "allowed_profiles": arm_probe["allowed_profiles"]}

    def request_shutdown(self, reason: str, *, stop_mode: str = "hold") -> dict[str, object]:
        self._request_cleanup(reason, stop_mode)
        return self.lifecycle_status()

    def shutdown(self, reason: str, *, stop_mode: str = "hold") -> bool:
        self._request_cleanup(reason, stop_mode)
        with self._state_lock:
            worker = self._cleanup_thread
        if worker is not None and worker is not threading.current_thread():
            worker.join()
        return self.lifecycle_status()["cleanup_complete"] is True

    def _request_cleanup(self, reason: str, stop_mode: str) -> None:
        with self._state_lock:
            if self._lifecycle is Lifecycle.RUNNING:
                self._lifecycle = Lifecycle.SHUTDOWN_REQUESTED
                self._cleanup_reason = reason
                self._cleanup_stop_mode = "hold"
                self.watchdog.stop()
            elif self._lifecycle in {Lifecycle.CLEANED, Lifecycle.SERVER_STOPPING, Lifecycle.STOPPED}:
                return
            if self._cleanup_thread is None or not self._cleanup_thread.is_alive():
                self._cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True, name="DesktopCleanup")
                self._cleanup_thread.start()

    def _cleanup_loop(self) -> None:
        delays = (1.0, 2.0, 5.0)
        while True:
            with self._state_lock:
                if self._cleanup_complete:
                    return
                self._lifecycle = Lifecycle.CLEANING
                self._cleanup_attempt += 1
                attempt = self._cleanup_attempt
                reason, stop_mode = self._cleanup_reason, self._cleanup_stop_mode
            success = self._cleanup_attempt_once(reason, stop_mode)
            with self._state_lock:
                self._cleanup_complete = success
                if success:
                    self._lifecycle = Lifecycle.CLEANED
                else:
                    self._lifecycle = Lifecycle.RETRY_WAIT
                    delay = delays[min(attempt - 1, len(delays) - 1)]
            if success:
                self._shutdown_server()
                return
            logging.error("[DesktopRuntime] Cleanup incomplete; retrying in %.1fs", delay)
            time.sleep(delay)

    def _cleanup_attempt_once(self, reason: str, stop_mode: str) -> bool:
        with self._operation_lock:
            logging.info("[DesktopRuntime] Cleanup attempt %s: stop_session(%s)", self._cleanup_attempt, stop_mode)
            success = True
            if getattr(self.controller, "available", False):
                try:
                    success = bool(self.controller.stop_session(stop_mode="hold", reason=reason)) and success
                except Exception:
                    success = False
                    logging.exception("[DesktopRuntime] Session cleanup failed")
            registry = getattr(self.controller, "source_registry", None)
            if registry is not None:
                try:
                    success = bool(registry.shutdown(reason)) and success
                except Exception:
                    success = False
                    logging.exception("[DesktopRuntime] Provider shutdown failed")
            if hasattr(self.controller, "_cleanup_lock"):
                try:
                    success = bool(self.controller.shutdown_cleanup_thread()) and success
                except Exception:
                    success = False
                    logging.exception("[DesktopRuntime] Cleanup thread shutdown failed")
            return success

    def _shutdown_server(self) -> None:
        with self._state_lock:
            self._lifecycle = Lifecycle.SERVER_STOPPING
        if self._server is not None:
            self._server.shutdown()
        with self._state_lock:
            self._lifecycle = Lifecycle.STOPPED

    def _handle_heartbeat_timeout(self) -> None:
        self.request_shutdown("desktop-heartbeat-timeout", stop_mode="hold")
