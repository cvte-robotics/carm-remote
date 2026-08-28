"""
单双臂遥操 session 编排。
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, normalize_hand, opposite_hand


LOCAL_ARM_CONTROL_HOST = "127.0.0.1"
ArmEndpointResolver = Callable[[str, str, bool], str]


def board_arm_endpoint(role: str, management_ip: str, same_board_dual: bool) -> str:
    if role == "master" or same_board_dual:
        return LOCAL_ARM_CONTROL_HOST
    return management_ip


def desktop_arm_endpoint(role: str, management_ip: str, same_board_dual: bool) -> str:
    del role, same_board_dual
    return management_ip


def build_arm_bindings(
    mode: str,
    master_ip: str,
    master_side: str,
    slave_ip: Optional[str] = None,
    endpoint_resolver: ArmEndpointResolver = board_arm_endpoint,
) -> List[Dict[str, Any]]:
    """Resolve management addresses into explicit arm-control endpoints."""
    normalized_mode = (mode or "single").strip().lower()
    normalized_master_side = normalize_hand(master_side, HAND_RIGHT)

    if normalized_mode == "single":
        return [
            {
                "role": "master",
                "ip": master_ip,
                "control_host": endpoint_resolver("master", master_ip, False),
                "device_index": None,
                "use_seven_axis_joints": False,
                "hand": normalized_master_side,
            }
        ]

    if normalized_mode != "dual":
        raise ValueError(f"Unsupported joystick mode: {mode}")
    if not slave_ip:
        raise ValueError("slave_ip is required for dual mode")

    same_board_dual = master_ip == slave_ip
    return [
        {
            "role": "master",
            "ip": master_ip,
            "control_host": endpoint_resolver("master", master_ip, same_board_dual),
            "device_index": 0 if same_board_dual else None,
            "use_seven_axis_joints": same_board_dual,
            "hand": normalized_master_side,
        },
        {
            "role": "slave",
            "ip": slave_ip,
            "control_host": endpoint_resolver("slave", slave_ip, same_board_dual),
            "device_index": 1 if same_board_dual else None,
            "use_seven_axis_joints": same_board_dual,
            "hand": opposite_hand(normalized_master_side),
        },
    ]


@dataclass
class ArmRuntime:
    role: str
    ip: str
    control_host: str
    device_index: Optional[int]
    hand: str
    controller: object
    thread: Optional[threading.Thread] = None


class BaseArmSession:
    def __init__(self, owner, bindings: List[Dict[str, Any]], controller_factory: Callable[[str, str], object]):
        self.owner = owner
        self.bindings = bindings
        self.controller_factory = controller_factory
        self.arms: List[ArmRuntime] = []

    def initialize(self) -> bool:
        runtimes: List[ArmRuntime] = []
        try:
            for binding in self.bindings:
                controller = self.controller_factory(binding["role"], binding["hand"])
                controller.carm_device_index = binding["device_index"]
                controller.use_seven_axis_joints = binding["use_seven_axis_joints"]
                runtime = ArmRuntime(
                    role=binding["role"],
                    ip=binding["ip"],
                    control_host=binding["control_host"],
                    device_index=binding["device_index"],
                    hand=binding["hand"],
                    controller=controller,
                )
                controller.load_cached_init_position(runtime.ip)
                runtimes.append(runtime)

            if len(runtimes) == 1:
                if not runtimes[0].controller.initArm(runtimes[0].control_host):
                    self._cleanup_started(runtimes)
                    return False
            else:
                for runtime in runtimes:
                    if not runtime.controller.prepareArm(runtime.control_host):
                        self._cleanup_started(runtimes)
                        return False

                if not self._move_to_init_concurrently(runtimes):
                    self._cleanup_started(runtimes)
                    return False

            if not self.owner.is_input_connected():
                logging.warning("[ArmSession] Input disconnected before control loops started")
                self._cleanup_started(runtimes)
                return False

            for runtime in runtimes:
                if not self.owner.publish_source_controller(runtime.controller):
                    logging.warning("[ArmSession] Input disconnected before controller ownership publication")
                    self._cleanup_started(runtimes)
                    return False

            for runtime in runtimes:
                controller = runtime.controller
                controller.isStop = False
                runtime.thread = threading.Thread(target=controller.loop, daemon=True, name=f"JoystickArmLoop-{runtime.role}")
                runtime.thread.start()
                self.owner.set_arm_runtime_state(
                    runtime.role,
                    runtime.ip,
                    runtime.hand,
                    True,
                    control_host=runtime.control_host,
                    device_index=runtime.device_index,
                )
            self.arms = runtimes
            return True
        except Exception:
            self._cleanup_started(runtimes)
            raise

    def _move_to_init_concurrently(self, runtimes: List[ArmRuntime]) -> bool:
        start_gate = threading.Event()
        cancel_event = threading.Event()
        results: Dict[str, bool] = {}
        start_times: Dict[str, float] = {}

        def move(runtime: ArmRuntime) -> None:
            start_gate.wait()
            if cancel_event.is_set():
                results[runtime.role] = False
                return
            start_times[runtime.role] = time.monotonic()
            try:
                results[runtime.role] = runtime.controller.moveToInitPosition()
            except Exception:
                logging.exception("[ArmSession] Initialization movement failed for role=%s", runtime.role)
                results[runtime.role] = False

        workers = [
            threading.Thread(target=move, args=(runtime,), daemon=True, name=f"JoystickArmInit-{runtime.role}")
            for runtime in runtimes
        ]
        try:
            for worker in workers:
                worker.start()
        except Exception:
            cancel_event.set()
            start_gate.set()
            for worker in workers:
                if worker.is_alive():
                    worker.join(timeout=2.0)
            raise

        start_gate.set()
        for worker in workers:
            worker.join()

        if len(start_times) > 1:
            skew_ms = (max(start_times.values()) - min(start_times.values())) * 1000.0
            logging.info("[ArmSession] Dual-arm initialization command start skew: %.3f ms", skew_ms)

        return len(results) == len(runtimes) and all(results.values())

    def stop(self, move_home: bool = False) -> bool:
        success = True
        threads_stopped = True
        for runtime in self.arms:
            runtime.controller.halt_control()

        for runtime in self.arms:
            try:
                if runtime.thread and runtime.thread.is_alive():
                    runtime.thread.join(timeout=2.0)
                    if runtime.thread.is_alive():
                        threads_stopped = False
                        success = False
            except Exception:
                threads_stopped = False
                success = False

        if move_home and threads_stopped and self.arms:
            if len(self.arms) > 1:
                success &= self._move_home_concurrently(self.arms)
            else:
                try:
                    self.arms[0].controller.move_home()
                except Exception:
                    logging.exception("[ArmSession] Single-arm home movement failed")
                    success = False

        for runtime in self.arms:
            try:
                if getattr(runtime.controller, "carm_", None):
                    runtime.controller.carm_.disconnect()
            except Exception:
                success = False
            finally:
                self.owner.set_arm_runtime_state(
                    runtime.role,
                    runtime.ip,
                    runtime.hand,
                    False,
                    control_host=runtime.control_host,
                    device_index=runtime.device_index,
                )
                self.owner.release_source_controller(runtime.controller)
        self.arms = []
        return success

    def _move_home_concurrently(self, runtimes: List[ArmRuntime]) -> bool:
        start_gate = threading.Event()
        cancel_event = threading.Event()
        results: Dict[str, bool] = {}
        start_times: Dict[str, float] = {}

        def move_home(runtime: ArmRuntime) -> None:
            start_gate.wait()
            if cancel_event.is_set():
                results[runtime.role] = False
                return
            start_times[runtime.role] = time.monotonic()
            try:
                results[runtime.role] = bool(runtime.controller.move_home())
            except Exception:
                logging.exception("[ArmSession] Home movement failed for role=%s", runtime.role)
                results[runtime.role] = False

        workers = [
            threading.Thread(target=move_home, args=(runtime,), daemon=True, name=f"JoystickArmHome-{runtime.role}")
            for runtime in runtimes
        ]
        try:
            for worker in workers:
                worker.start()
        except Exception:
            cancel_event.set()
            start_gate.set()
            for worker in workers:
                if worker.is_alive():
                    worker.join(timeout=2.0)
            logging.exception("[ArmSession] Failed to start concurrent home workers")
            return False

        start_gate.set()
        for worker in workers:
            worker.join()

        if len(start_times) > 1:
            skew_ms = (max(start_times.values()) - min(start_times.values())) * 1000.0
            logging.info("[ArmSession] Dual-arm home command start skew: %.3f ms", skew_ms)

        return len(results) == len(runtimes) and all(results.values())

    def get_primary_controller(self):
        return self.arms[0].controller if self.arms else None

    def _cleanup_started(self, started: List[ArmRuntime]) -> None:
        for runtime in started:
            runtime.controller.halt_control()

        for runtime in reversed(started):
            try:
                if runtime.thread and runtime.thread.is_alive():
                    runtime.thread.join(timeout=2.0)
                if getattr(runtime.controller, "carm_", None):
                    runtime.controller.carm_.disconnect()
            except Exception:
                pass
            finally:
                self.owner.set_arm_runtime_state(
                    runtime.role,
                    runtime.ip,
                    runtime.hand,
                    False,
                    control_host=runtime.control_host,
                    device_index=runtime.device_index,
                )
                self.owner.release_source_controller(runtime.controller)


class SingleArmSession(BaseArmSession):
    pass


class DualArmSession(BaseArmSession):
    pass
