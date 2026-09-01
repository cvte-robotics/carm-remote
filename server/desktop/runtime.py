"""Quest-only desktop runtime lifecycle and cleanup orchestration."""

from __future__ import annotations

from enum import Enum
import ipaddress
import logging
import threading
import time
from typing import Callable, Dict, Tuple, TypeVar

from basic.joystick import Carm, JoystickController
from basic.joystick.end_effectors import EndEffectorFactory
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
    def __init__(
        self,
        end_effector_factory: EndEffectorFactory | None = None,
    ) -> None:
        self.controller = JoystickController(
            arm_binding_builder=build_desktop_arm_bindings,
            end_effector_factory=end_effector_factory,
        )
        self._server = None
        self._state_lock = threading.RLock()
        self._operation_lock = threading.RLock()
        self._lifecycle = Lifecycle.RUNNING
        self._cleanup_complete = False
        self._cleanup_attempt = 0
        self._cleanup_thread = None
        self._cleanup_reason = ""
        self._latest_operation_generation = -1
        self._cancelled_operation_generation = -1
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
                "latest_generation": self._latest_operation_generation,
            }

    @staticmethod
    def _operation_generation(body: Dict[str, object]) -> int | None:
        value = body.get("_desktop_generation")
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return None
        return value

    def _admit_operation(self, generation: int, *, cancel: bool) -> bool:
        with self._state_lock:
            if self._lifecycle is not Lifecycle.RUNNING:
                return False
            if generation < self._latest_operation_generation:
                return False
            if generation > self._latest_operation_generation:
                self._latest_operation_generation = generation
            if cancel:
                self._cancelled_operation_generation = max(
                    self._cancelled_operation_generation,
                    generation,
                )
                return True
            return generation > self._cancelled_operation_generation

    def _operation_is_current(self, generation: int, *, cancel: bool = False) -> bool:
        with self._state_lock:
            return bool(
                self._lifecycle is Lifecycle.RUNNING
                and generation == self._latest_operation_generation
                and (
                    cancel
                    or generation > self._cancelled_operation_generation
                )
            )

    def require_current_operation(self, body: Dict[str, object]) -> int:
        generation = self._operation_generation(body)
        if generation is None or not self._operation_is_current(generation):
            raise RuntimeError("桌面请求已失效")
        return generation

    def run_operation(
        self,
        body: Dict[str, object],
        operation: Callable[[], OperationResult],
        *,
        cancel: bool = False,
    ) -> Tuple[bool, OperationResult | None]:
        generation = self._operation_generation(body)
        if generation is None or not self._admit_operation(generation, cancel=cancel):
            return False, None
        with self._operation_lock:
            if not self._operation_is_current(generation, cancel=cancel):
                return False, None
            return True, operation()

    def scan(self, controller, body: Dict[str, object]) -> Dict[str, object]:
        with self._operation_lock:
            self.require_current_operation(body)
            arm_probe = probe_current_arm(normalize_ipv4(body.get("current_arm_ip")))
            self.require_current_operation(body)
            devices = controller.scan_devices()
        return {"data": devices, "arm_probe": {key: arm_probe[key] for key in ("ip", "arm_name", "arm_dof", "family")}, "allowed_profiles": arm_probe["allowed_profiles"]}

    def request_shutdown(self, reason: str) -> dict[str, object]:
        self._request_cleanup(reason)
        return self.lifecycle_status()

    def shutdown(self, reason: str) -> bool:
        self._request_cleanup(reason)
        with self._state_lock:
            worker = self._cleanup_thread
        if worker is not None and worker is not threading.current_thread():
            worker.join()
        return self.lifecycle_status()["cleanup_complete"] is True

    def _request_cleanup(self, reason: str) -> None:
        with self._state_lock:
            if self._lifecycle is Lifecycle.RUNNING:
                self._lifecycle = Lifecycle.SHUTDOWN_REQUESTED
                self._cleanup_reason = reason
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
                reason = self._cleanup_reason
            success = self._cleanup_attempt_once(reason)
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

    def _cleanup_attempt_once(self, reason: str) -> bool:
        with self._operation_lock:
            logging.info(
                "[DesktopRuntime] Cleanup attempt %s: controller.shutdown()",
                self._cleanup_attempt,
            )
            try:
                return bool(self.controller.shutdown(reason))
            except Exception:
                logging.exception("[DesktopRuntime] Controller cleanup failed")
                return False

    def _shutdown_server(self) -> None:
        with self._state_lock:
            self._lifecycle = Lifecycle.SERVER_STOPPING
        if self._server is not None:
            self._server.shutdown()
        with self._state_lock:
            self._lifecycle = Lifecycle.STOPPED

    def _handle_heartbeat_timeout(self) -> None:
        self.request_shutdown("desktop-heartbeat-timeout")
