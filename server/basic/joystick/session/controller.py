import asyncio
import logging
import math
import numbers
import threading
import time
from concurrent.futures import TimeoutError as FuturesTimeoutError
from dataclasses import dataclass
from typing import Any, Dict, Optional, List, Tuple

from ..runtime.config import ArmControlConfig, JoystickConfigError
from ..runtime.paths import quest_only_providers_enabled

from ..runtime.dependencies import (
    ARM_SDK_AVAILABLE, CORE_DEPENDENCIES_AVAILABLE,
    DEPENDENCIES_AVAILABLE, Carm,
)
from ..runtime.context import JOYSTICK_CONFIG
from carm_teleop_input_contracts import (
    CalibrationMode,
    ConnectOutcome,
    HAND_LEFT,
    HAND_RIGHT,
    RawInputBatch,
    SourceLease,
    TransportEvent,
    normalize_hand,
)
from ..sources.adapters import button_sample_to_legacy, pose_sample_to_legacy, validate_raw_input_batch
from ..sources.registry import (
    ActiveSourceBinding,
    BindingPhase,
    SourceLeaseAllocator,
    SourceRegistry,
    SourceScanError,
)
from quest.constants import *
from ..core.constants import *
from ..core.command_controller import CommandController
from ..core.pose_math import _is_valid_joint_position
from ..end_effectors import EndEffectorFactory, resolve_end_effector_factory
from quest.module import QuestInputModule
from ..core.models import (
    button_to_dict,
    pose_to_dict,
)
from .arm import BaseArmSession, build_arm_bindings, LOCAL_ARM_CONTROL_HOST
from .cleanup import CleanupContext, CleanupMode, SessionCleanup
from .constants import *
from .state import JoystickState, SessionPhase
from .observation import (
    ObservationArm,
    ObservationDevice,
    ObservationHand,
    ObservationSnapshot,
    ObservationStatus,
    ObservationTransport,
    project_http_status,
)


def _create_surreal_module():
    from surreal.module import SurrealInputModule

    return SurrealInputModule()


@dataclass(frozen=True)
class StopRequest:
    reason: str
    input_generation: Optional[int]
    lease: Optional[SourceLease]


class JoystickController:
    """手柄遥操控制器管理类"""

    def __init__(
        self,
        scale=0.4,
        joystick_config_snapshot=None,
        source_registry=None,
        arm_binding_builder=build_arm_bindings,
        end_effector_factory: Optional[EndEffectorFactory] = None,
    ):
        if not DEPENDENCIES_AVAILABLE:
            logging.warning("[JoystickController] Dependencies not available")
            self.available = False
            return

        self.available = True
        self.scale = scale
        self.joystick_config = joystick_config_snapshot or JOYSTICK_CONFIG
        self._active_arm_configs: Dict[str, ArmControlConfig] = {}

        self.controller = None
        self.init_position_controller = None
        self.session = None
        self._cleanup_home_allowed = False
        self.active_input_source = None
        self.input_generation = 0
        self._source_lease_allocator = SourceLeaseAllocator()
        self.source_registry = source_registry or SourceRegistry(
            quest_factory=QuestInputModule,
            surreal_factory=(
                None if quest_only_providers_enabled() else _create_surreal_module
            ),
            lease_allocator=self._source_lease_allocator,
        )
        self._arm_binding_builder = arm_binding_builder
        self._end_effector_factory = (
            end_effector_factory or resolve_end_effector_factory("carm")
        )
        self.active_source_binding: Optional[ActiveSourceBinding] = None
        self._source_controller_generation = 0

        self._lifecycle_lock = threading.RLock()
        self._data_lock = threading.RLock()
        self.state = JoystickState()
        self.state.scale = scale

        # 兼容旧接口：进程内确认值在后续机械臂初始化时覆盖配置值。
        self.cached_init_positions: Dict[str, List[float]] = {}

        # 单线程清理模式：使用常驻线程处理所有清理请求
        self._cleanup_event = threading.Event()  # 触发清理的事件
        self._cleanup_thread = None  # 常驻清理线程
        self._cleanup_shutdown = False  # 标记是否正在关闭
        self._pending_cleanup_request: Optional[StopRequest] = None
        self._session_cleanup = SessionCleanup(self)

        # 启动清理线程
        self._start_cleanup_thread()

    def _set_phase_locked(self, phase: SessionPhase) -> None:
        self.state.phase = phase

    def _stable_phase_locked(self) -> SessionPhase:
        if self.state.arm_initialized:
            return SessionPhase.READY
        if self.state.ble_connected:
            return SessionPhase.CONNECTED
        return SessionPhase.IDLE

    def _reset_source_policies(self) -> None:
        for provider_id in self.source_registry.provider_ids:
            module = self.source_registry.existing(provider_id)
            if module is not None:
                module.reset_policy()

    def _is_hand_bound_active_locked(self, hand: str, controller=None) -> bool:
        for arm_state in self.state.arm_states.values():
            if (
                arm_state.get("connected")
                and arm_state.get("hand_binding") == hand
                and (
                    controller is None
                    or (
                        (module := self.source_registry.active_module()) is not None
                        and module.controller_matches(hand, controller)
                    )
                )
            ):
                return True
        return False

    def _assign_source_controller(self, controller: CommandController) -> None:
        with self._data_lock:
            self._source_controller_generation += 1
            controller.source_controller_generation = self._source_controller_generation

    def publish_source_controller(self, controller: CommandController) -> bool:
        with self._data_lock:
            module = self.source_registry.active_module()
            binding = self.active_source_binding
            if module is None or binding is None or binding.phase is BindingPhase.CLOSED:
                return True
            if not self.state.ble_connected:
                return False
            self.state.control_generation += 1
            previous_controller = module.bind_controller(
                controller.hand_binding,
                controller,
                controller.source_controller_generation,
                self.input_generation,
                self.state.control_generation,
            )
            self.state.data_sequence += 1
            if previous_controller is not None and previous_controller is not controller:
                previous_controller.invalidate_pose_mapping()
            return True

    def release_source_controller(self, controller: CommandController) -> None:
        with self._data_lock:
            module = self.source_registry.active_module()
            self.state.control_generation += 1
            released = bool(
                module is not None
                and module.release_controller(
                    controller.hand_binding,
                    controller,
                    self.state.control_generation,
                )
            )
            if not released:
                self.state.control_generation -= 1
                return
            self.state.data_sequence += 1
            controller.invalidate_pose_mapping()

    def _sync_view_side_payload(self) -> None:
        view_side = normalize_hand(self.state.view_side, HAND_RIGHT)
        hand_state = self.state.hands.get(view_side, {})
        self.state.pose_data = hand_state.get("pose_data")
        self.state.button_data = hand_state.get("button_data")

    def connection_is_current(
        self,
        lease: SourceLease,
        provider_identity=None,
    ) -> bool:
        with self._data_lock:
            binding = self.active_source_binding
            if (
                binding is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != lease
            ):
                return False
            module = self.source_registry.active_module()
            if module is None:
                return False
            if provider_identity is None:
                return True
            return bool(module.callback_matches(lease, provider_identity))

    def publish_input(self, batch: RawInputBatch, provider_identity) -> None:
        self._handle_input_batch(batch, provider_identity)

    def publish_telemetry(
        self,
        lease: SourceLease,
        provider_identity,
        battery_level=None,
        rssi=None,
    ) -> None:
        with self._data_lock:
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            if (
                binding is None
                or module is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != lease
                or not module.callback_matches(lease, provider_identity)
            ):
                return
            if battery_level is not None:
                self.state.battery_level = battery_level
            if rssi is not None:
                self.state.rssi = rssi
            self.state.data_sequence += 1

    def publish_transport(
        self,
        event: TransportEvent,
        provider_identity,
    ) -> None:
        if not self.connection_is_current(event.lease, provider_identity):
            return
        schedule_terminal_cleanup = False
        schedule_reconnect = False
        suspended_controllers = set()
        with self._data_lock:
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            if (
                binding is None
                or module is None
                or binding.lease != event.lease
                or binding.phase is BindingPhase.CLOSED
                or not module.callback_matches(event.lease, provider_identity)
            ):
                return
            status = module.status(event.lease)
            details = dict(status.details)
            was_open = binding.phase in {
                BindingPhase.PROVISIONAL,
                BindingPhase.ACTIVE,
            }
            was_active = binding.phase is BindingPhase.ACTIVE
            if event.phase == "ready" and binding.phase is BindingPhase.PROVISIONAL:
                self._activate_binding_locked(
                    event.lease.provider_id,
                    event.lease.input_generation,
                )
                binding = self.active_source_binding
            elif event.phase in {"disconnected", "terminal"} and was_active:
                self._set_binding_input_ready_locked(
                    event.lease.provider_id,
                    event.lease.input_generation,
                    False,
                )
                if not event.terminal:
                    suspended_controllers.update(self._active_pose_controllers())
                    suspended_controllers.update(module.mark_input_disconnected())
            self.state.transport_state = event.phase
            self.state.transport_terminal = event.terminal
            self.state.quest_protocol_version = details.get("protocol_version")
            self.state.quest_connection_id = details.get("connection_id")
            self.state.quest_last_rx_monotonic = details.get("last_rx_monotonic")
            self.state.quest_transport_generation = details.get(
                "generation",
                event.transport_generation,
            )
            if event.reason is not None:
                self.state.quest_last_disconnect_reason = event.reason
            if event.phase == "ready":
                self.state.ble_connected = True
                if self.state.phase not in {
                    SessionPhase.INITIALIZING,
                    SessionPhase.READY,
                }:
                    self._set_phase_locked(SessionPhase.CONNECTED)
                self.state.last_error = None
            elif event.phase in {"disconnected", "terminal"}:
                self.state.ble_connected = False
                if not event.terminal:
                    self.state.pose_data = None
                    self.state.button_data = None
                    self.state.battery_level = None
                    self.state.rssi = None
                    for hand_state in self.state.hands.values():
                        hand_state["pose_data"] = None
                        hand_state["button_data"] = None
                    self.state.control_generation += 1
                else:
                    self.state.last_error = event.reason
            self.state.data_sequence += 1
            schedule_terminal_cleanup = bool(event.terminal and was_open)
            schedule_reconnect = bool(
                not event.terminal
                and event.phase == "disconnected"
                and binding.capabilities.supports_reconnect_grace
                and was_active
            )
        for controller in suspended_controllers:
            controller.invalidate_pose_mapping()
        if schedule_terminal_cleanup:
            self._mark_disconnected_and_request_cleanup(
                event.reason or "input-transport-terminal",
                expected_lease=event.lease,
            )
        elif schedule_reconnect:
            self._schedule_source_reconnect_cleanup(event.lease)

    def _binding_matches_locked(
        self,
        source: str,
        generation: int,
        require_input_ready: bool = False,
    ) -> bool:
        binding = self.active_source_binding
        identity_matches = bool(
            binding is not None
            and binding.phase is not BindingPhase.CLOSED
            and binding.lease.provider_id == source
            and binding.lease.input_generation == generation
            and self.active_input_source == source
            and self.input_generation == generation
        )
        if not identity_matches or not require_input_ready:
            return identity_matches
        return bool(
            binding.phase is BindingPhase.ACTIVE
            and binding.input_ready
        )

    def _activate_binding_locked(self, source: str, generation: int) -> None:
        binding = self.active_source_binding
        if not self._binding_matches_locked(source, generation) or binding is None:
            return
        if binding.phase is BindingPhase.PROVISIONAL:
            self.active_source_binding = binding.activate()
        elif binding.phase is BindingPhase.ACTIVE and not binding.input_ready:
            self.active_source_binding = binding.with_input_ready(True)

    def _set_binding_input_ready_locked(
        self,
        source: str,
        generation: int,
        ready: bool,
    ) -> None:
        binding = self.active_source_binding
        if (
            binding is not None
            and binding.phase is BindingPhase.ACTIVE
            and self._binding_matches_locked(source, generation)
        ):
            self.active_source_binding = binding.with_input_ready(ready)

    def _close_binding_locked(self) -> None:
        binding = self.active_source_binding
        if binding is not None and binding.phase is not BindingPhase.CLOSED:
            self.source_registry.begin_close(binding.lease)
            self.active_source_binding = binding.close()

    def _activate_input_source(
        self,
        source: str,
        public_source_type: Optional[str] = None,
        capabilities=None,
    ) -> int:
        """Publish a new input session generation and provisional binding."""
        module = self.source_registry.get_or_create(source)
        if capabilities is None:
            capabilities = module.availability().capabilities
        if public_source_type is None:
            public_source_type = source
        if module.current_lease() is not None:
            raise RuntimeError("previous input transport must be closed before activation")
        module.cancel_reconnect_timer(clear_deadline=True)
        with self._data_lock:
            generation = self.input_generation + 1
            lease = self.source_registry.allocate_lease(
                module.module_id,
                generation,
            )
            # Module acceptance precedes every host-visible identity mutation.
            module.bind_lease(lease)
            self.source_registry.activate(lease)
            self._close_binding_locked()
            self.input_generation = generation
            self.active_input_source = module.module_id
            self.active_source_binding = ActiveSourceBinding(
                lease=lease,
                public_source_type=public_source_type,
                capabilities=capabilities,
                phase=BindingPhase.PROVISIONAL,
                input_ready=False,
            )
            self._reset_source_policies()
            self.state.control_generation += 1
            self.state.data_sequence += 1
            for controller in self._active_pose_controllers():
                controller.invalidate_pose_mapping()
            return generation

    def _active_pose_controllers(self) -> List[CommandController]:
        if self.session:
            return [runtime.controller for runtime in self.session.arms]
        return [self.controller] if self.controller else []

    def _request_source_mapping_change_locked(self, reason: str):
        module = self.source_registry.active_module()
        if module is None:
            return ()
        active_hands = tuple(
            hand
            for hand in (HAND_LEFT, HAND_RIGHT)
            if (
                (controller := module.controller_for_hand(hand)) is not None
                and self._is_hand_bound_active_locked(hand, controller)
            )
        )
        return module.on_mapping_change(
            reason,
            active_hands,
            self.input_generation,
            self.state.control_generation,
        )

    def _schedule_source_reconnect_cleanup(self, lease: SourceLease) -> None:
        module = self.source_registry.active_module()
        if module is None:
            return

        def evaluate_recovery() -> Tuple[bool, bool]:
            with self._data_lock:
                binding = self.active_source_binding
                stale_session = bool(
                    binding is None
                    or binding.phase is BindingPhase.CLOSED
                    or binding.lease != lease
                )
                transport_recovered = self.state.transport_state in {
                    "healthy",
                    "ready",
                }
            return stale_session, transport_recovered

        begin_grace = getattr(module, "begin_reconnect_grace", None)
        if not callable(begin_grace):
            return
        begin_grace(
            lease,
            evaluate_recovery,
            lambda _input_generation: self._expire_source_reconnect_deadline(
                lease,
            ),
        )

    def _expire_source_reconnect_deadline(
        self,
        lease: SourceLease,
    ) -> None:
        result = self._mark_input_disconnected(
            "input-reconnect-deadline",
            expected_lease=lease,
        )
        if result is None:
            return
        with self._data_lock:
            cleanup_generation = self.input_generation
        # The existing worker blocks on lifecycle ownership until any in-flight
        # initialization returns; no polling timer is required.
        self.request_stop_session(
            reason="input-reconnect-deadline",
            expected_input_generation=cleanup_generation,
            expected_lease=lease,
        )

    def _cancel_source_reconnect_timers(
        self,
        clear_deadline: bool = False,
    ) -> None:
        for provider_id in self.source_registry.provider_ids:
            module = self.source_registry.existing(provider_id)
            if module is not None:
                module.cancel_reconnect_timer(clear_deadline=clear_deadline)

    def _mark_input_disconnected(
        self,
        reason: str,
        expected_lease: Optional[SourceLease] = None,
    ) -> Optional[bool]:
        """在异步资源清理前立即关闭输入连接状态。"""
        with self._data_lock:
            binding = self.active_source_binding
            if expected_lease is not None and (
                binding is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != expected_lease
            ):
                return None
            module = self.source_registry.active_module()
            controllers = set(self._active_pose_controllers())
            controllers.update(module.owned_controllers() if module is not None else ())
            input_was_open = bool(
                self.active_input_source is not None
                or self.state.ble_connected
                or (
                    binding is not None
                    and binding.phase is not BindingPhase.CLOSED
                )
            )
            self._close_binding_locked()
            if input_was_open:
                self.input_generation += 1
            self.active_input_source = None
            is_initializing = self.state.phase is SessionPhase.INITIALIZING
            self.state.ble_connected = False
            self.state.transport_state = "disconnected"
            self.state.transport_terminal = False
            self.state.quest_protocol_version = None
            self.state.quest_connection_id = None
            self.state.quest_last_disconnect_reason = reason
            self.state.quest_last_rx_monotonic = None
            self.state.quest_transport_generation = 0
            self.state.pose_data = None
            self.state.button_data = None
            self.state.battery_level = None
            self.state.rssi = None
            for hand_state in self.state.hands.values():
                hand_state["pose_data"] = None
                hand_state["button_data"] = None
            self.state.control_generation += 1
            if not is_initializing:
                self._set_phase_locked(self._stable_phase_locked())
            self.state.data_sequence += 1

        self._reset_source_policies()
        for controller in controllers:
            controller.invalidate_pose_mapping()
        self._cancel_source_reconnect_timers(clear_deadline=True)
        return is_initializing

    def _mark_disconnected_and_request_cleanup(
        self,
        reason: str,
        expected_lease: Optional[SourceLease] = None,
    ) -> None:
        with self._data_lock:
            binding = self.active_source_binding
            cleanup_lease = expected_lease or (
                binding.lease if binding is not None else None
            )
        result = self._mark_input_disconnected(
            reason,
            expected_lease=cleanup_lease,
        )
        if result is None:
            return
        with self._data_lock:
            cleanup_generation = self.input_generation
        self.request_stop_session(
            reason=reason,
            expected_input_generation=cleanup_generation,
            expected_lease=cleanup_lease,
        )

    def _handle_control_termination(self, termination) -> None:
        """Request cleanup only for the exact current failed arm runtime."""
        if termination.kind.value != "internal_error":
            return
        with self._data_lock:
            runtime = termination.runtime
            transaction = termination.transaction
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            current = bool(
                runtime is not None
                and runtime.transaction is transaction
                and runtime.thread is threading.current_thread()
                and module is not None
                and module.controller_matches(runtime.hand, runtime.controller)
                and runtime.controller.source_controller_generation
                == termination.source_controller_generation
                and binding is not None
                and binding.lease == termination.source_lease
                and self.input_generation == termination.input_generation
                and self.state.control_generation == termination.control_generation
            )
        if not current:
            logging.warning(
                "[JoystickController] Ignoring stale control termination: %s",
                termination.reason,
            )
            return
        self._mark_disconnected_and_request_cleanup(
            termination.reason,
            expected_lease=termination.source_lease,
        )

    def is_input_connected(self) -> bool:
        with self._data_lock:
            binding = self.active_source_binding
            return bool(
                self.state.ble_connected
                and binding is not None
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
            )

    def set_arm_runtime_state(
        self,
        role: str,
        ip: Optional[str],
        hand_binding: Optional[str],
        connected: bool,
        control_host: Optional[str] = None,
        device_index: Optional[int] = None,
    ) -> None:
        with self._data_lock:
            if role not in self.state.arm_states:
                return
            self.state.arm_states[role]["ip"] = ip
            self.state.arm_states[role]["control_host"] = control_host
            self.state.arm_states[role]["arm_index"] = device_index
            self.state.arm_states[role]["hand_binding"] = hand_binding
            self.state.arm_states[role]["connected"] = connected
            self.state.arm_initialized = any(
                arm["connected"] for arm in self.state.arm_states.values()
            )
            self.state.data_sequence += 1

    def _build_command_controller(self, role: str, hand_binding: str):
        arm_config = self._active_arm_configs.get(hand_binding)
        if arm_config is None:
            raise JoystickConfigError(f"未找到 {hand_binding} 臂的遥操配置")
        controller = CommandController(
            self.scale,
            hand_binding=hand_binding,
            arm_role=role,
            configured_init_joint=arm_config.init_joint,
        )
        self._assign_source_controller(controller)
        return controller

    def _start_cleanup_thread(self):
        """启动常驻清理线程"""
        with self._data_lock:
            if self._cleanup_thread and self._cleanup_thread.is_alive():
                return
            if self._cleanup_shutdown:
                logging.warning("[JoystickController] Cleanup system is shutting down, cannot start new thread")
                return

        self._cleanup_thread = threading.Thread(
            target=self._cleanup_worker,
            daemon=True,
            name=CLEANUP_WORKER_NAME
        )
        self._cleanup_thread.start()
        logging.info("[JoystickController] Cleanup worker thread started")

    def _requeue_stop_request(self, request: StopRequest) -> bool:
        with self._data_lock:
            if self._pending_cleanup_request is not None:
                return False
            self._pending_cleanup_request = request
            return True

    def _cleanup_worker(self):
        """Consume generation-bound stop requests on the single worker."""
        while True:
            try:
                triggered = self._cleanup_event.wait(
                    timeout=WORKER_CHECK_INTERVAL
                )
                if not triggered:
                    continue
                self._cleanup_event.clear()

                while True:
                    with self._data_lock:
                        if self._cleanup_shutdown:
                            logging.info(
                                "[CleanupWorker] Cleanup worker thread shutting down"
                            )
                            return
                        request = self._pending_cleanup_request
                        self._pending_cleanup_request = None
                        if request is None:
                            break

                    try:
                        with self._lifecycle_lock:
                            with self._data_lock:
                                cleanup_lease = self.source_registry.cleanup_lease()
                                stale = bool(
                                    self.input_generation
                                    != request.input_generation
                                    or self.active_input_source is not None
                                    or (
                                        request.lease is not None
                                        and cleanup_lease != request.lease
                                    )
                                )
                            if stale:
                                logging.info(
                                    "[CleanupWorker] Skipping stale cleanup, reason=%s",
                                    request.reason,
                                )
                                continue
                            logging.info(
                                "[CleanupWorker] Starting hold cleanup, reason=%s",
                                request.reason,
                            )
                            context = self._build_cleanup_context_locked(
                                CleanupMode.HOLD,
                                request.reason,
                                request.input_generation,
                                request.lease,
                                False,
                            )
                            result = self._run_cleanup_locked(context)
                            if not result.success:
                                if self._requeue_stop_request(request):
                                    time.sleep(WORKER_CHECK_INTERVAL)
                                    self._cleanup_event.set()
                                    break
                                continue
                            logging.info(
                                "[CleanupWorker] Cleanup operation completed"
                            )
                    except Exception:
                        logging.exception(
                            "[CleanupWorker] Cleanup operation failed"
                        )
                        if self._requeue_stop_request(request):
                            time.sleep(WORKER_CHECK_INTERVAL)
                            self._cleanup_event.set()
                            break
            except Exception:
                logging.exception("[CleanupWorker] Worker error")
                time.sleep(0.1)

    def _reset_state(self) -> None:
        """重置状态"""
        self._cancel_source_reconnect_timers(clear_deadline=True)
        with self._data_lock:
            self._close_binding_locked()
            self.input_generation += 1
            self.active_input_source = None
        with self._data_lock:
            self.state.ble_connected = False
            self.state.arm_initialized = False
            self._set_phase_locked(SessionPhase.IDLE)
            self.state.connected_address = None
            self.state.connected_name = None
            self.state.source_type = None
            self.state.mode = "single"
            self.state.master_side = HAND_RIGHT
            self.state.view_side = HAND_RIGHT
            self.state.is_quest_device = False
            self.state.transport_state = "disconnected"
            self.state.transport_terminal = False
            self.state.quest_protocol_version = None
            self.state.quest_connection_id = None
            self.state.quest_last_rx_monotonic = None
            self.state.quest_last_disconnect_reason = None
            self.state.quest_transport_generation = 0
            self.state.pose_data = None
            self.state.button_data = None
            self.state.battery_level = None
            self.state.rssi = None
            self.state.hands = {
                HAND_LEFT: {
                    "pose_data": None,
                    "button_data": None,
                    "input_sequence": 0,
                },
                HAND_RIGHT: {
                    "pose_data": None,
                    "button_data": None,
                    "input_sequence": 0,
                },
            }
            self.state.arm_states = {
                "master": {
                    "ip": None,
                    "control_host": None,
                    "arm_index": None,
                    "hand_binding": None,
                    "connected": False,
                },
                "slave": {
                    "ip": None,
                    "control_host": None,
                    "arm_index": None,
                    "hand_binding": None,
                    "connected": False,
                },
            }
            self.state.control_generation += 1
            self.state.data_sequence += 1
        self._reset_source_policies()

    def shutdown_cleanup_thread(self):
        """Stop the one cleanup worker after owner retirement succeeds."""
        with self._data_lock:
            self._cleanup_shutdown = True

        self._cleanup_event.set()
        success = True
        if self._cleanup_thread and self._cleanup_thread.is_alive():
            self._cleanup_thread.join(timeout=CLEANUP_THREAD_TIMEOUT)
            if self._cleanup_thread.is_alive():
                logging.warning("[JoystickController] Cleanup thread did not stop within timeout")
                success = False
            else:
                logging.info("[JoystickController] Cleanup thread stopped successfully")

        if success:
            self._cleanup_thread = None
        logging.info("[JoystickController] Cleanup thread shutdown completed")
        return success

    def _resolve_device_descriptor(self, address: str, name: str):
        return self.source_registry.resolve_device(address, name)

    def _handle_input_batch(self, batch: RawInputBatch, provider_identity) -> None:
        """Validate, adapt and commit one provider-neutral input transaction."""
        batch = validate_raw_input_batch(batch)
        if provider_identity is None:
            raise ValueError("provider identity is required for input batches")
        module = self.source_registry.active_module()
        legacy = {
            update.hand: {
                "pose": (
                    pose_sample_to_legacy(update.pose)
                    if update.update_pose
                    else None
                ),
                "button": (
                    button_sample_to_legacy(update.button)
                    if update.update_button
                    else None
                ),
            }
            for update in batch.hands
        }
        invalidated_controllers = set()
        rejected_controllers = set()
        with self._data_lock:
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            if (
                binding is None
                or module is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != batch.lease
                or not module.callback_matches(batch.lease, provider_identity)
            ):
                return
            module.validate_input_batch(batch)
            bound_hands = {
                update.hand
                for update in batch.hands
                if (
                    (controller := module.controller_for_hand(update.hand))
                    is not None
                    and self._is_hand_bound_active_locked(update.hand, controller)
                )
            }
            decisions = module.process_input_batch(
                batch,
                bound_hands,
                self.input_generation,
                self.state.control_generation,
            )
            if binding.phase is BindingPhase.PROVISIONAL:
                self._activate_binding_locked(
                    batch.lease.provider_id,
                    batch.lease.input_generation,
                )
                binding = self.active_source_binding
            elif binding.phase is BindingPhase.ACTIVE and not binding.input_ready:
                self._set_binding_input_ready_locked(
                    batch.lease.provider_id,
                    batch.lease.input_generation,
                    True,
                )
                binding = self.active_source_binding
            for decision in decisions:
                hand_state = self.state.hands[decision.hand]
                if decision.accept_pose:
                    hand_state["pose_data"] = legacy[decision.hand]["pose"]
                if decision.accept_button:
                    hand_state["button_data"] = legacy[decision.hand]["button"]
                hand_state["input_sequence"] += 1
                details = dict(decision.details)
                if details.get("event"):
                    logging.warning(
                        "[JoystickController] Source policy event: %s hand=%s details=%s",
                        details["event"],
                        decision.hand,
                        details,
                    )
                if decision.controller is not None:
                    if decision.invalidate_controller:
                        invalidated_controllers.add(decision.controller)
                    elif decision.reject_controller:
                        rejected_controllers.add(decision.controller)
            status = module.status(batch.lease)
            details = dict(status.details)
            self.state.ble_connected = True
            if self.state.phase not in {
                SessionPhase.INITIALIZING,
                SessionPhase.READY,
            }:
                self._set_phase_locked(SessionPhase.CONNECTED)
            self.state.last_error = None
            self.state.source_type = binding.public_source_type
            self.state.is_quest_device = bool(
                binding.capabilities.supports_reconnect_grace
            )
            self.state.transport_state = status.phase
            self.state.transport_terminal = False
            self.state.quest_protocol_version = details.get("protocol_version")
            self.state.quest_connection_id = details.get("connection_id")
            self.state.quest_last_rx_monotonic = details.get("last_rx_monotonic")
            self.state.quest_transport_generation = details.get(
                "generation",
                batch.transport_generation,
            )
            self._sync_view_side_payload()
            self.state.data_sequence += 1
            for controller in rejected_controllers - invalidated_controllers:
                controller.reject_pose_target()
            for controller in invalidated_controllers:
                controller.invalidate_pose_mapping()

    def _close_source_lease(self, lease: SourceLease, reason: str) -> bool:
        if not self.source_registry.begin_close(lease):
            return False
        module = self.source_registry.get_or_create(lease.provider_id)
        if not module.close_connection(lease, reason):
            return False
        return self.source_registry.complete_close(lease)

    def request_stop_session(
        self,
        reason: str = "unknown",
        expected_input_generation: Optional[int] = None,
        expected_lease: Optional[SourceLease] = None,
    ) -> None:
        """Queue one generation-bound cleanup request without doing cleanup."""
        with self._data_lock:
            if self._cleanup_shutdown:
                logging.warning(
                    "[JoystickController] Cleanup is shutting down; ignoring request"
                )
                return
            generation = (
                self.input_generation
                if expected_input_generation is None
                else expected_input_generation
            )
            request = StopRequest(reason, generation, expected_lease)
            pending = self._pending_cleanup_request
            if (
                pending is None
                or pending.input_generation is None
                or generation >= pending.input_generation
            ):
                self._pending_cleanup_request = request

        logging.info("[JoystickController] Requesting hold cleanup, reason=%s", reason)
        self._cleanup_event.set()

    def scan_devices(self) -> list:
        with self._lifecycle_lock:
            return self._scan_devices_locked()

    def _scan_devices_locked(self) -> list:
        with self._data_lock:
            if self.state.phase is not SessionPhase.IDLE:
                self.state.last_error = "请先停止当前遥操会话，再扫描设备"
                return []
            self._set_phase_locked(SessionPhase.SCANNING)
            self.state.last_error = None
        try:
            descriptors = self.source_registry.scan_quest_first()
            devices = [
                {
                    "name": descriptor.candidate.name,
                    "address": descriptor.candidate.address,
                }
                for descriptor in descriptors
            ]
            is_grace_source = bool(
                descriptors
                and descriptors[0].candidate.capabilities.supports_reconnect_grace
            )
            with self._data_lock:
                self._set_phase_locked(SessionPhase.IDLE)
                self.state.last_error = None
                self.state.is_quest_device = is_grace_source
            logging.info(
                "[JoystickController] Scan completed, found %d devices",
                len(devices),
            )
            return devices
        except SourceScanError as exc:
            error_msg = str(exc)
        except (asyncio.TimeoutError, FuturesTimeoutError):
            error_msg = "扫描超时，请检查蓝牙是否开启"
            logging.warning("[JoystickController] %s", error_msg)
        except Exception as exc:
            error_msg = f"扫描设备失败: {exc}"
            logging.exception(exc)

        with self._data_lock:
            self._set_phase_locked(SessionPhase.IDLE)
            self.state.last_error = error_msg
        return []

    def connect_device(self, address: str, name: str = "Unknown") -> bool:
        with self._lifecycle_lock:
            return self._connect_device_locked(address, name)

    def _connect_device_locked(self, address: str, name: str) -> bool:
        with self._data_lock:
            if self.state.phase is SessionPhase.CLEANUP_FAILED:
                self.state.last_error = "遥操资源清理未完成，请先重试停止"
                return False

        if self.source_registry.cleanup_lease() is not None:
            if not self.stop_session(
                stop_mode="hold",
                reason="replace-connected-device",
            ):
                with self._data_lock:
                    self.state.last_error = "旧遥操连接清理失败，请重试停止后再连接"
                return False

        descriptor = self._resolve_device_descriptor(address, name)
        if descriptor is None:
            with self._data_lock:
                self._set_phase_locked(SessionPhase.IDLE)
                self.state.last_error = (
                    self.source_registry.resolution_error(address, name)
                    or "不支持的遥操设备"
                )
            return False

        candidate = descriptor.candidate
        module = self.source_registry.get_or_create(candidate.provider_id)
        availability = module.availability()
        if not availability.available:
            with self._data_lock:
                self._set_phase_locked(SessionPhase.IDLE)
                self.state.last_error = availability.reason or "手柄遥操功能不可用"
            return False

        module.configure(self.joystick_config)
        generation = None
        lease = None
        try:
            generation = self._activate_input_source(
                candidate.provider_id,
                public_source_type=candidate.public_source_type,
                capabilities=candidate.capabilities,
            )
            binding = self.active_source_binding
            if binding is None:
                raise RuntimeError("input source binding was not published")
            lease = binding.lease
            with self._data_lock:
                self._set_phase_locked(SessionPhase.CONNECTING)
                self.state.last_error = None
                self.state.connected_address = candidate.address
                self.state.connected_name = candidate.name
                self.state.source_type = candidate.public_source_type
                self.state.transport_terminal = False
                self.state.is_quest_device = bool(
                    candidate.capabilities.supports_reconnect_grace
                )
                self.state.data_sequence += 1

            result = module.connect(candidate, lease, self)
            with self._data_lock:
                binding = self.active_source_binding
                terminal_observed = self.state.transport_terminal
                current = bool(
                    not terminal_observed
                    and binding is not None
                    and binding.phase is not BindingPhase.CLOSED
                    and binding.lease == lease
                    and self.source_registry.active_module() is module
                )
                if current and result.outcome is ConnectOutcome.READY:
                    if binding.phase is BindingPhase.PROVISIONAL:
                        self._activate_binding_locked(
                            lease.provider_id,
                            lease.input_generation,
                        )
                    self.state.ble_connected = True
                    self._set_phase_locked(SessionPhase.CONNECTED)
                    self.state.last_error = None
                    self.state.transport_state = "ready"
                    self.state.transport_terminal = False
                    self.state.data_sequence += 1
                elif current and result.outcome is ConnectOutcome.PENDING:
                    self.state.ble_connected = False
                    self._set_phase_locked(SessionPhase.CONNECTING)
                    self.state.last_error = result.message
                    self.state.data_sequence += 1
            if not current:
                closed = self._close_source_lease(lease, "input-connect-stale")
                with self._data_lock:
                    self._set_phase_locked(
                        SessionPhase.IDLE if closed else SessionPhase.CLEANUP_FAILED
                    )
                return False
            if result.outcome is ConnectOutcome.READY:
                return True
            if result.outcome is ConnectOutcome.PENDING and result.retain_transport:
                return False

            self._mark_input_disconnected(
                "input-connect-failed",
                expected_lease=lease,
            )
            closed = self._close_source_lease(lease, "input-connect-failed")
            with self._data_lock:
                self._set_phase_locked(
                    SessionPhase.IDLE if closed else SessionPhase.CLEANUP_FAILED
                )
                self.state.last_error = result.message
            return False
        except Exception as exc:
            logging.exception("[JoystickController] Input module connect failed")
            closed = True
            if lease is not None:
                self._mark_input_disconnected(
                    "input-connect-error",
                    expected_lease=lease,
                )
                closed = self._close_source_lease(lease, "input-connect-error")
            with self._data_lock:
                self.state.ble_connected = False
                self._set_phase_locked(
                    SessionPhase.IDLE if closed else SessionPhase.CLEANUP_FAILED
                )
                self.state.last_error = f"连接设备失败: {exc}"
            return False

    def get_last_error(self) -> str:
        """获取最后一次操作的错误信息"""
        with self._data_lock:
            return self.state.last_error

    def init_arm(self, master_ip: str, mode: str = "single", slave_ip: Optional[str] = None, master_side: str = HAND_RIGHT) -> bool:
        with self._lifecycle_lock:
            return self._init_arm_locked(
                master_ip,
                mode=mode,
                slave_ip=slave_ip,
                master_side=master_side,
            )

    def _init_arm_locked(self, master_ip: str, mode: str = "single", slave_ip: Optional[str] = None, master_side: str = HAND_RIGHT) -> bool:
        """初始化机械臂"""
        if not self.available:
            logging.warning("[JoystickController] Controller not available")
            return False

        with self._data_lock:
            binding = self.active_source_binding
            if self.state.phase is SessionPhase.CLEANUP_FAILED:
                self.state.last_error = "遥操资源清理未完成，请先重试停止"
                return False
            input_connected = bool(
                self.state.ble_connected
                and binding is not None
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
            )
            can_control_arms = bool(
                binding is not None
                and binding.capabilities.can_control_arms
            )
            if not input_connected:
                error_msg = "请先连接遥操设备，再初始化机械臂"
                self.state.last_error = error_msg
                logging.warning("[JoystickController] %s", error_msg)
                return False
            if not can_control_arms:
                error_msg = "ANS 设备仅支持连接测试，不能初始化机械臂"
                self.state.last_error = error_msg
                logging.warning("[JoystickController] %s", error_msg)
                return False

        session = None
        try:
            normalized_mode = (mode or "single").strip().lower()
            normalized_master_side = normalize_hand(master_side, HAND_RIGHT)
            with self._data_lock:
                self._set_phase_locked(SessionPhase.INITIALIZING)
                self.state.mode = normalized_mode
                self.state.master_side = normalized_master_side
                self.state.view_side = self.state.master_side
                self.state.last_error = ""

            if self.session:
                old_context = self._build_cleanup_context_locked(
                    CleanupMode.HOLD,
                    "session-reinitialize",
                    home_allowed=False,
                )
                if not self._run_cleanup_locked(old_context).success:
                    with self._data_lock:
                        self.state.arm_initialized = False
                        self._set_phase_locked(SessionPhase.CLEANUP_FAILED)
                        self.state.last_error = "旧机械臂会话清理未完成，请重试停止"
                    return False

            bindings = self._arm_binding_builder(
                self.state.mode,
                master_ip,
                self.state.master_side,
                slave_ip,
            )
            use_seven_axis_joints = any(
                binding["use_seven_axis_joints"] for binding in bindings
            )
            self._active_arm_configs = self.joystick_config.resolve_session(
                normalized_mode,
                normalized_master_side,
                use_seven_axis_joints,
            )
            session = BaseArmSession(
                self,
                bindings,
                self._build_command_controller,
                self._end_effector_factory,
            )

            logging.info(
                "[JoystickController] Initializing %s arm session "
                "(master=%s, slave=%s, side=%s, endpoints=%s)",
                self.state.mode,
                master_ip,
                slave_ip,
                self.state.master_side,
                [
                    {
                        "role": binding["role"],
                        "management_ip": binding["ip"],
                        "control_host": binding["control_host"],
                        "arm_index": binding["device_index"],
                    }
                    for binding in bindings
                ],
            )
            success = session.initialize()
            if not success:
                if session.arms:
                    self.session = session
                    self.controller = session.get_primary_controller()
                with self._data_lock:
                    self.state.arm_initialized = False
                    self._set_phase_locked(
                        SessionPhase.CLEANUP_FAILED
                        if session.arms
                        else self._stable_phase_locked()
                    )
                return False

            self.session = session
            self.controller = session.get_primary_controller()
            with self._data_lock:
                self.state.arm_initialized = True
                self._set_phase_locked(SessionPhase.READY)
            return True
        except JoystickConfigError as e:
            if session is not None and session.arms:
                self.session = session
                self.controller = session.get_primary_controller()
            error_msg = f"遥操配置错误：{e}"
            logging.error("[JoystickConfig] %s", error_msg)
            with self._data_lock:
                self.state.arm_initialized = False
                self._set_phase_locked(
                    SessionPhase.CLEANUP_FAILED
                    if session is not None and session.arms
                    else self._stable_phase_locked()
                )
                self.state.last_error = error_msg
            return False
        except Exception as e:
            if session is not None and session.arms:
                self.session = session
                self.controller = session.get_primary_controller()
            error_msg = f"机械臂初始化异常：{str(e)}"
            logging.error(error_msg)
            logging.exception(e)
            with self._data_lock:
                self.state.arm_initialized = False
                self._set_phase_locked(
                    SessionPhase.CLEANUP_FAILED
                    if session is not None and session.arms
                    else self._stable_phase_locked()
                )
                self.state.last_error = error_msg
            return False

    def set_calibration(self, value: int) -> bool:
        """设置标定模式"""
        try:
            CalibrationMode(value)
            with self._data_lock:
                affected_controllers = []
                changed = self.state.calibration != value
                self.state.calibration = value
                if changed:
                    self.state.control_generation += 1
                    affected_controllers.extend(
                        self._request_source_mapping_change_locked(
                            "calibration_change"
                        )
                    )
                    self.state.data_sequence += 1
                if changed:
                    controllers = affected_controllers or self._active_pose_controllers()
                    for controller in controllers:
                        controller.invalidate_pose_mapping()
            return True
        except Exception as e:
            logging.error(f"[JoystickController] Invalid calibration value: {e}")
            return False

    def set_scale(self, value: float) -> bool:
        """设置灵敏度"""
        try:
            if (
                not isinstance(value, numbers.Real)
                or isinstance(value, bool)
            ):
                logging.error("[JoystickController] Invalid scale value: %r", value)
                return False

            value = float(value)
            if not math.isfinite(value) or not 0.1 <= value <= 1.0:
                logging.error("[JoystickController] Invalid scale value: %r", value)
                return False

            with self._data_lock:
                self.scale = value
                affected_controllers = []
                changed = self.state.scale != value
                self.state.scale = value
                if changed:
                    self.state.control_generation += 1
                    affected_controllers.extend(
                        self._request_source_mapping_change_locked(
                            "scale_change"
                        )
                    )
                    self.state.data_sequence += 1
                if self.session:
                    for runtime in self.session.arms:
                        runtime.controller.scale = value
                elif self.controller:
                    self.controller.scale = value
                if changed:
                    controllers = affected_controllers or self._active_pose_controllers()
                    for controller in controllers:
                        controller.invalidate_pose_mapping()
            return True
        except Exception as e:
            logging.exception(e)
            return False

    def set_view_side(self, hand: str) -> bool:
        """设置当前查看侧，并同步兼容字段。"""
        try:
            with self._data_lock:
                self.state.view_side = normalize_hand(hand, self.state.master_side or HAND_RIGHT)
                self._sync_view_side_payload()
                self.state.data_sequence += 1
            return True
        except Exception as e:
            logging.exception(e)
            return False

    def set_session_config(
        self,
        mode: Optional[str] = None,
        master_side: Optional[str] = None,
        view_side: Optional[str] = None,
        slave_ip: Optional[str] = None,
    ) -> bool:
        """设置会话配置，用于连接前同步 Quest UI 选择。"""
        try:
            with self._data_lock:
                if mode is not None:
                    normalized_mode = (mode or "single").strip().lower()
                    if normalized_mode not in ("single", "dual"):
                        raise ValueError(f"Unsupported joystick mode: {mode}")
                    self.state.mode = normalized_mode
                    if normalized_mode == "single":
                        self.state.arm_states["slave"]["ip"] = None

                if master_side is not None:
                    self.state.master_side = normalize_hand(master_side, self.state.master_side or HAND_RIGHT)

                if slave_ip is not None and self.state.mode == "dual":
                    self.state.arm_states["slave"]["ip"] = slave_ip or None

                if view_side is not None:
                    self.state.view_side = normalize_hand(view_side, self.state.master_side or HAND_RIGHT)
                else:
                    self.state.view_side = normalize_hand(self.state.view_side, self.state.master_side or HAND_RIGHT)

                self._sync_view_side_payload()
                self.state.data_sequence += 1
            return True
        except Exception as e:
            logging.exception(e)
            return False

    def get_observation_snapshot(self) -> ObservationSnapshot:
        """Capture one immutable, generation-consistent Host observation."""
        if not self.available:
            empty_hand = ObservationHand.from_values()
            return ObservationSnapshot(
                sequence=0, available=False,
                status=ObservationStatus(False, False, 0, getattr(self, "scale", 0.4),
                                         "not_available", None, None, None, "single",
                                         False, HAND_RIGHT, HAND_RIGHT),
                device=ObservationDevice(None, None),
                transport=ObservationTransport("disconnected", None, None, None, None),
                master_arm=ObservationArm(), slave_arm=ObservationArm(),
                left_hand=empty_hand, right_hand=empty_hand, pose=None, button=None,
                stream_terminal=True, emit_disconnect_frame=False)
        with self._data_lock:
            binding = self.active_source_binding
            binding_active = bool(
                binding is not None and binding.phase is not BindingPhase.CLOSED
            )
            supports_reconnect_grace = bool(
                binding is not None
                and binding.capabilities.supports_reconnect_grace
            )
            last_rx_age_ms = (
                    max(
                        0,
                        int(
                            (
                                time.monotonic()
                                - self.state.quest_last_rx_monotonic
                            )
                            * 1000
                        ),
                    )
                    if self.state.quest_last_rx_monotonic is not None
                    else None
                )
            stream_connected = bool(
                self.state.ble_connected
                or (
                    binding_active
                    and supports_reconnect_grace
                    and self.state.transport_state in {"negotiating", "healthy"}
                )
            )
            emit_disconnect_frame = bool(
                supports_reconnect_grace
                and not binding_active
                and not self.state.ble_connected
            )
            hands = {}
            for hand in (HAND_LEFT, HAND_RIGHT):
                hand_state = self.state.hands[hand]
                hands[hand] = ObservationHand.from_values(
                    pose=(
                        pose_to_dict(self.state.hands[hand]["pose_data"])
                        if self.state.hands[hand]["pose_data"]
                        else None
                    ),
                    button=(
                        button_to_dict(self.state.hands[hand]["button_data"])
                        if self.state.hands[hand]["button_data"]
                        else None
                    ),
                )
            return ObservationSnapshot(
                sequence=self.state.data_sequence, available=True,
                pose=pose_to_dict(self.state.pose_data) if self.state.pose_data else None,
                button=button_to_dict(self.state.button_data) if self.state.button_data else None,
                status=ObservationStatus(stream_connected, self.state.arm_initialized,
                    self.state.calibration, self.state.scale, self.state.status,
                    self.state.battery_level, self.state.rssi, self.state.source_type,
                    self.state.mode, self.state.is_quest_device, self.state.master_side,
                    normalize_hand(self.state.view_side, HAND_RIGHT)),
                device=ObservationDevice(self.state.connected_address, self.state.connected_name),
                transport=ObservationTransport(self.state.transport_state,
                    self.state.quest_protocol_version, self.state.quest_connection_id,
                    last_rx_age_ms, self.state.quest_last_disconnect_reason),
                master_arm=ObservationArm.from_state(self.state.arm_states["master"]),
                slave_arm=ObservationArm.from_state(self.state.arm_states["slave"]),
                left_hand=hands[HAND_LEFT], right_hand=hands[HAND_RIGHT],
                stream_terminal=bool(not self.state.ble_connected and (not supports_reconnect_grace or not binding_active)),
                emit_disconnect_frame=emit_disconnect_frame)

    def get_status(self) -> dict:
        """获取当前状态"""
        return project_http_status(self.get_observation_snapshot())

    def _build_init_position_controller(self) -> CommandController:
        """为旧初始化位置接口创建采用单臂配置初值的控制器。"""
        arm_config = self.joystick_config.resolve_session("single", HAND_RIGHT, False)[HAND_RIGHT]
        controller = CommandController(
            self.scale,
            arm_role="master",
            configured_init_joint=arm_config.init_joint,
        )
        controller.load_cached_init_position(
            self.get_cached_init_position("master")
        )
        return controller

    def confirm_init_position(self, ip: str) -> bool:
        """通过本机控制端点确认初始化位置；ip 仅保留接口兼容。"""
        if not self.available:
            logging.warning("[JoystickController] Controller not available")
            return False

        try:
            if not self.init_position_controller:
                logging.info("[JoystickController] Creating CommandController for init position confirmation")
                self.init_position_controller = self._build_init_position_controller()

            logging.info(
                "[JoystickController] Resolving local temporary connection "
                "from management_ip=%s to control_host=%s",
                ip,
                LOCAL_ARM_CONTROL_HOST,
            )
            success = self.init_position_controller.confirmInitPosition(LOCAL_ARM_CONTROL_HOST)
            if success:
                position = self.init_position_controller.getInitPosition().get("init_joint")
                self.set_cached_init_position("master", position)
                logging.warning("[JoystickController] Init position confirmed successfully")
            else:
                logging.error("[JoystickController] Failed to confirm init position")
            return success
        except Exception as e:
            logging.error(f"[JoystickController] Exception during confirm init position: {str(e)}")
            logging.exception(e)
            return False

    def get_init_position(self) -> dict:
        """获取接口覆盖、配置文件或原生默认值中的当前生效值。"""
        if not self.available:
            return {"init_joint": None, "error": "Controller not available"}

        try:
            cached_position = self.get_cached_init_position("master")
            if cached_position is not None:
                return {"init_joint": cached_position}
            if self.session and self.controller:
                return self.controller.getInitPosition()
            if not self.init_position_controller:
                logging.info("[JoystickController] Creating CommandController for getting init position")
                self.init_position_controller = self._build_init_position_controller()
            return self.init_position_controller.getInitPosition()
        except Exception as e:
            error_msg = f"[JoystickController] Exception getting init position: {str(e)}"
            logging.exception(e)
            return {"init_joint": None, "error": error_msg}

    def get_current_joints(self, ip: str) -> dict:
        """通过本机控制端点获取当前主设备关节值；ip 仅保留接口兼容。"""
        if not self.available:
            return {
                "current_joints": None,
                "error": "Controller not available"
            }

        try:
            if not self.controller:
                logging.info("[JoystickController] Creating CommandController for getting current joints")
                self.controller = CommandController(
                    self.scale,
                    arm_role="master",
                )
            logging.info(
                "[JoystickController] Resolving local current-joints connection "
                "from management_ip=%s to control_host=%s",
                ip,
                LOCAL_ARM_CONTROL_HOST,
            )
            return self.controller.getCurrentJoints(LOCAL_ARM_CONTROL_HOST)
        except Exception as e:
            error_msg = f"[JoystickController] Exception getting current joints: {str(e)}"
            logging.exception(e)
            return {
                "current_joints": None,
                "error": error_msg
            }

    def stop_session(self, stop_mode: str = "hold", reason: str = "unknown") -> bool:
        """停止遥操会话；仅显式 home 模式允许机械臂回零。"""
        if not self.available:
            return False

        # Serialize teardown with provider setup so transport ownership cannot
        # be published after cleanup has inspected the session.
        with self._lifecycle_lock:
            return self._stop_session_locked(stop_mode=stop_mode, reason=reason)

    def _build_cleanup_context_locked(
        self,
        mode: CleanupMode,
        reason: str,
        input_generation: Optional[int] = None,
        source_lease: Optional[SourceLease] = None,
        home_allowed: bool = False,
    ) -> CleanupContext:
        binding = self.active_source_binding
        lease = source_lease or (binding.lease if binding is not None else None)
        runtime = self.session.arms[0] if self.session and self.session.arms else None
        return CleanupContext(
            mode=mode,
            reason=reason,
            input_generation=self.input_generation if input_generation is None else input_generation,
            control_generation=self.state.control_generation,
            source_lease=lease,
            runtime_identity=runtime,
            transaction_identity=(runtime.transaction if runtime is not None else None),
            source_controller_generation=(
                runtime.controller.source_controller_generation
                if runtime is not None else None
            ),
            home_allowed=home_allowed,
        )

    def _run_cleanup_locked(self, context: Optional[CleanupContext] = None, *, force_hold=False, reason=None):
        result = (
            self._session_cleanup.retry(force_hold=force_hold, reason=reason)
            if self._session_cleanup.retained_context is not None
            else self._session_cleanup.stop(context)
        )
        if result.success:
            self._cleanup_home_allowed = False
            self._reset_state()
        else:
            with self._data_lock:
                self._set_phase_locked(SessionPhase.CLEANUP_FAILED)
                self.state.last_error = "遥操资源清理未完成，请重试停止"
                self.state.data_sequence += 1
        logging.info(
            "[JoystickController] Session cleanup completed%s failed_steps=%s",
            " successfully" if result.success else " with retained owners",
            tuple(step.value for step in result.failed_steps),
        )
        return result

    def _stop_session_locked(self, stop_mode: str, reason: str) -> bool:
        """Invalidate input, then retry every retained cleanup owner."""

        self._cancel_source_reconnect_timers(clear_deadline=True)

        normalized_mode = (stop_mode or "hold").strip().lower()
        if normalized_mode not in ("hold", "home"):
            logging.warning("[JoystickController] Unsupported stop mode %s; falling back to hold", stop_mode)
            normalized_mode = "hold"

        retained = self._session_cleanup.retained_context
        if retained is not None:
            logging.info(
                "[JoystickController] Retrying retained session cleanup "
                "(mode=%s, reason=%s)",
                retained.mode.value,
                retained.reason,
            )
            return self._run_cleanup_locked(
                force_hold=bool(normalized_mode == "hold"),
                reason=reason,
            ).success

        with self._data_lock:
            binding = self.active_source_binding
            input_connected = bool(
                self.state.ble_connected
                and binding is not None
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
            )
        if normalized_mode == "hold":
            self._cleanup_home_allowed = False
        elif input_connected:
            self._cleanup_home_allowed = True
        home_allowed = bool(
            normalized_mode == "home"
            and getattr(self, "_cleanup_home_allowed", False)
        )
        if normalized_mode == "home" and not home_allowed:
            logging.warning("[JoystickController] Input already disconnected; downgrading home cleanup to hold")

        # Freeze home permission first, then reject every callback and pending
        # target before waiting on arm, SDK, BLE, ADB or thread cleanup.
        self._mark_input_disconnected(reason)
        with self._data_lock:
            self._set_phase_locked(SessionPhase.STOPPING)
            self.state.data_sequence += 1

        logging.info(
            "[JoystickController] Beginning session cleanup (mode=%s, reason=%s)",
            "home" if home_allowed else "hold",
            reason,
        )
        context = self._build_cleanup_context_locked(
            CleanupMode.HOME if home_allowed else CleanupMode.HOLD,
            reason,
            home_allowed=home_allowed,
        )
        return self._run_cleanup_locked(context).success

    def shutdown(self, reason: str = "controller-shutdown") -> bool:
        """Stop the session and retire every owner independently."""
        if not self.available:
            return True
        if not self.stop_session(stop_mode="hold", reason=reason):
            return False
        result = self._session_cleanup.shutdown(reason)
        if not result.success:
            return False
        return True

    def set_cached_init_position(self, role: str, position: List[float]) -> None:
        """设置旧接口使用的进程内初始化位置。"""
        if not _is_valid_joint_position(position, 6):
            logging.error("[JoystickController] Position must contain exactly 6 finite numbers: %s", position)
            return

        try:
            with self._data_lock:
                self.cached_init_positions[role] = list(position)
                logging.info("[JoystickController] Cached init position for %s: %s", role, position)
        except Exception as e:
            logging.error(f"[JoystickController] Error caching init position: {e}")

    def get_cached_init_position(self, role: str) -> Optional[List[float]]:
        """获取旧接口保存的进程内初始化位置。"""
        try:
            with self._data_lock:
                cached_position = self.cached_init_positions.get(role)
                if _is_valid_joint_position(cached_position, 6):
                    logging.info("[JoystickController] Retrieved cached init position for %s: %s", role, cached_position)
                    return cached_position.copy()
                if cached_position:
                    logging.warning("[JoystickController] Cached position validation failed. Position: %s", cached_position)
                return None
        except Exception as e:
            logging.error(f"[JoystickController] Error retrieving cached init position: {e}")
            return None
