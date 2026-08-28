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
from quest.module import QuestInputModule
from ..core.models import (
    button_to_dict,
    pose_to_dict,
)
from .arm import build_arm_bindings, DualArmSession, SingleArmSession, LOCAL_ARM_CONTROL_HOST
from .constants import *
from .constants import _UNSET
from .state import JoystickState


def _create_surreal_module():
    from surreal.module import SurrealInputModule

    return SurrealInputModule()


class JoystickController:
    """手柄遥操控制器管理类"""

    def __init__(
        self,
        scale=0.4,
        gripper_max_value=0.08,
        joystick_config_snapshot=None,
        source_registry=None,
        arm_binding_builder=build_arm_bindings,
    ):
        if not DEPENDENCIES_AVAILABLE:
            logging.warning("[JoystickController] Dependencies not available")
            self.available = False
            return

        self.available = True
        self.scale = scale
        self.gripper_max_value = gripper_max_value
        self.joystick_config = joystick_config_snapshot or JOYSTICK_CONFIG
        self._active_arm_configs: Dict[str, ArmControlConfig] = {}

        self.controller = None
        self.init_position_controller = None
        self.controller_thread = None
        self.session = None
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
        self.active_source_binding: Optional[ActiveSourceBinding] = None
        self._source_controller_generation = 0

        self.state = JoystickState()
        self.state.scale = scale

        self._initialized = False
        self._lock = threading.Lock()

        # 兼容旧接口：进程内确认值在后续机械臂初始化时覆盖配置值。
        self.cached_init_positions: Dict[str, List[float]] = {}

        # 单线程清理模式：使用常驻线程处理所有清理请求
        self._cleanup_event = threading.Event()  # 触发清理的事件
        self._cleanup_thread = None  # 常驻清理线程
        self._cleanup_lock = threading.Lock()  # 保护清理状态
        self._cleanup_in_progress = False  # 标记清理是否正在进行
        self._cleanup_shutdown = False  # 标记是否正在关闭
        self._cleanup_reason = "unknown"
        self._cleanup_input_generation = None
        self._session_stop_lock = threading.Lock()
        self._input_session_lock = threading.RLock()
        self._input_command_lock = threading.Lock()
        self._deferred_cleanup_timer = None
        self._deferred_cleanup_timer_lock = threading.Lock()

        # 启动清理线程
        self._start_cleanup_thread()

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
        with self._input_command_lock:
            self._source_controller_generation += 1
            controller.source_controller_generation = self._source_controller_generation

    def publish_source_controller(self, controller: CommandController) -> bool:
        with self._input_command_lock:
            module = self.source_registry.active_module()
            binding = self.active_source_binding
            if module is None or binding is None or binding.phase is BindingPhase.CLOSED:
                return True
            with self.state.status_lock:
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
        with self._input_command_lock:
            module = self.source_registry.active_module()
            with self.state.status_lock:
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
        self.state.gripper_pose = hand_state.get("gripper_pose")
        self.state.gripper_tau = hand_state.get("gripper_tau")

    def connection_is_current(
        self,
        lease: SourceLease,
        provider_identity=None,
    ) -> bool:
        with self._input_command_lock:
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
        with self._input_command_lock:
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
            with self.state.status_lock:
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
        with self._input_command_lock:
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
            with self.state.status_lock:
                self.state.transport_state = event.phase
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
                    if self.state.status != "initializing_arm":
                        self.state.status = "connected"
                    self.state.last_error = None
                elif event.phase in {"disconnected", "terminal"}:
                    self.state.ble_connected = False
                    if not event.terminal:
                        self.state.pose_data = None
                        self.state.button_data = None
                        self.state.gripper_pose = None
                        self.state.gripper_tau = None
                        self.state.battery_level = None
                        self.state.rssi = None
                        for hand_state in self.state.hands.values():
                            hand_state["pose_data"] = None
                            hand_state["button_data"] = None
                            hand_state["gripper_pose"] = None
                            hand_state["gripper_tau"] = None
                        self.state.control_generation += 1
                        if self.state.status != "initializing_arm":
                            self.state.status = (
                                "arm_ready"
                                if self.state.arm_initialized
                                else "connecting"
                            )
                    else:
                        self.state.last_error = event.reason
                self.state.data_sequence += 1
            schedule_terminal_cleanup = bool(event.terminal and was_active)
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
                event.reason or "input-transport-terminal"
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
        with self._input_command_lock:
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
            with self.state.status_lock:
                self.state.control_generation += 1
                self.state.data_sequence += 1
            for controller in self._active_pose_controllers():
                controller.invalidate_pose_mapping()
            return generation

    def _set_hand_state(
        self,
        source: str,
        generation: int,
        hand: str,
        pose_data=_UNSET,
        button_data=_UNSET,
    ) -> bool:
        normalized_hand = normalize_hand(hand, HAND_RIGHT)
        with self._input_command_lock:
            if not self._binding_matches_locked(
                source,
                generation,
                require_input_ready=True,
            ):
                return False
            with self.state.status_lock:
                hand_state = self.state.hands[normalized_hand]
                if pose_data is not _UNSET:
                    hand_state["pose_data"] = pose_data
                if button_data is not _UNSET:
                    hand_state["button_data"] = button_data
                hand_state["input_sequence"] += 1
                self._sync_view_side_payload()
                self.state.last_update_ts = time.time()
                self.state.data_sequence += 1
            return True

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
        return module.request_mapping_change(
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
            with self._input_command_lock:
                binding = self.active_source_binding
                stale_session = bool(
                    binding is None
                    or binding.phase is BindingPhase.CLOSED
                    or binding.lease != lease
                )
                with self.state.status_lock:
                    transport_recovered = self.state.transport_state in {
                        "healthy",
                        "ready",
                    }
            return stale_session, transport_recovered

        module.start_reconnect_grace(
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
        is_initializing = self._mark_input_disconnected(
            "input-reconnect-deadline",
            expected_lease=lease,
        )
        if is_initializing is None:
            return
        with self._input_command_lock:
            cleanup_generation = self.input_generation
        if is_initializing:
            self._schedule_cleanup_request(
                delay=0.2,
                wait_for_initialization=True,
                source="input-reconnect-deadline",
                expected_input_generation=cleanup_generation,
            )
        else:
            self.request_stop_session(
                reason="input-reconnect-deadline",
                expected_input_generation=cleanup_generation,
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
        with self._input_command_lock:
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
            self._close_binding_locked()
            self.input_generation += 1
            self.active_input_source = None
            with self.state.status_lock:
                is_initializing = self.state.status == "initializing_arm"
                self.state.ble_connected = False
                self.state.disconnect_reason = reason
                self.state.transport_state = "disconnected"
                self.state.quest_protocol_version = None
                self.state.quest_connection_id = None
                self.state.quest_last_disconnect_reason = reason
                self.state.quest_last_rx_monotonic = None
                self.state.quest_transport_generation = 0
                self.state.pose_data = None
                self.state.button_data = None
                self.state.gripper_pose = None
                self.state.gripper_tau = None
                self.state.battery_level = None
                self.state.rssi = None
                for hand_state in self.state.hands.values():
                    hand_state["pose_data"] = None
                    hand_state["button_data"] = None
                    hand_state["gripper_pose"] = None
                    hand_state["gripper_tau"] = None
                self._reset_source_policies()
                self.state.control_generation += 1
                if not is_initializing:
                    self.state.status = "arm_ready" if self.state.arm_initialized else "idle"
                self.state.data_sequence += 1
            for controller in controllers:
                controller.invalidate_pose_mapping()
            self._cancel_source_reconnect_timers(clear_deadline=True)
            return is_initializing

    def _mark_disconnected_and_request_cleanup(self, reason: str) -> None:
        result = self._mark_input_disconnected(reason)
        if result is None:
            return
        with self._input_command_lock:
            cleanup_generation = self.input_generation
        self.request_stop_session(
            reason=reason,
            expected_input_generation=cleanup_generation,
        )

    def get_hand_state(self, hand: str) -> Dict[str, Any]:
        normalized_hand = normalize_hand(hand, HAND_RIGHT)
        with self.state.status_lock:
            hand_state = self.state.hands.get(normalized_hand, {})
            return {
                "pose_data": hand_state.get("pose_data"),
                "button_data": hand_state.get("button_data"),
                "gripper_pose": hand_state.get("gripper_pose"),
                "gripper_tau": hand_state.get("gripper_tau"),
                "input_sequence": hand_state.get("input_sequence", 0),
                "control_generation": self.state.control_generation,
                "calibration": self.state.calibration,
                "scale": self.state.scale,
            }

    def get_control_hand_state(
        self,
        controller: CommandController,
        hand: str,
        arm_feedback_fresh: bool = False,
    ) -> Dict[str, Any]:
        """Capture input state or atomically apply one pending Quest rebase."""
        normalized_hand = normalize_hand(hand, HAND_RIGHT)
        with self._input_command_lock:
            with self.state.status_lock:
                hand_state = self.state.hands.get(normalized_hand, {})
                input_source = self.active_input_source
                snapshot = {
                    "pose_data": hand_state.get("pose_data"),
                    "button_data": hand_state.get("button_data"),
                    "gripper_pose": hand_state.get("gripper_pose"),
                    "gripper_tau": hand_state.get("gripper_tau"),
                    "input_sequence": hand_state.get("input_sequence", 0),
                    "input_generation": self.input_generation,
                    "control_generation": self.state.control_generation,
                    "calibration": self.state.calibration,
                    "scale": self.state.scale,
                    "input_source": input_source,
                }
                button_data = hand_state.get("button_data")
                source_bound = self._is_hand_bound_active_locked(
                    normalized_hand,
                    controller,
                )
                binding = self.active_source_binding
                input_ready = bool(
                    binding is not None
                    and binding.phase is BindingPhase.ACTIVE
                    and binding.input_ready
                    and self.state.ble_connected
                )
                module = self.source_registry.active_module()

            with controller.pose_state_lock:
                decision = (
                    module.prepare_control(
                        normalized_hand,
                        controller,
                        controller.source_controller_generation,
                        snapshot["input_generation"],
                        snapshot["control_generation"],
                        source_bound and input_ready,
                        bool(
                            controller.need_sync_feedback
                            and button_data is not None
                            and button_data.grip > 100
                        ),
                    )
                    if module is not None
                    else None
                )
                if decision is not None:
                    if decision.pin is not None:
                        pin = decision.pin
                        if not decision.pin_matches or button_data is None:
                            controller.reject_pose_target()
                            return {}
                        applied = controller._apply_input_rebase_locked(
                            pin.raw_pose,
                            snapshot["calibration"],
                            button_data,
                            arm_feedback_fresh,
                        )
                        if applied and module.commit_rebase(
                            normalized_hand,
                            pin,
                        ):
                            logging.warning(
                                "[JoystickController] Quest pose rebased: hand=%s epoch=%d fi=%d reason=%s",
                                normalized_hand,
                                pin.jump_epoch,
                                pin.frame_index,
                                pin.reason,
                            )
                        return {}

                    if not decision.ready:
                        controller.reject_pose_target()
                        return {}
                    snapshot["source_token"] = decision.source_token

                snapshot["pose_epoch"] = controller.pose_epoch
                snapshot["arm_connection_generation"] = controller.arm_connection_generation
                snapshot["arm_feedback_state_sequence"] = controller.arm_feedback_state_sequence
                return snapshot

    def is_input_connected(self) -> bool:
        binding = self.active_source_binding
        with self.state.status_lock:
            return bool(
                self.state.ble_connected
                and binding is not None
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
            )

    def claim_input_command(self) -> bool:
        """与断连发布串行化，原子判定本次输入驱动命令是否允许发送。"""
        with self._input_command_lock:
            return self.is_input_connected()

    def _commit_source_dispatch_locked(
        self,
        controller: CommandController,
        source_token,
    ) -> None:
        """Advance provider policy only after the complete physical transaction succeeds."""
        if source_token is None:
            return
        module = self.source_registry.active_module()
        if (
            module is None
            or not module.commit_dispatch(
                controller.hand_binding,
                controller,
                controller.source_controller_generation,
                source_token,
                self.input_generation,
                self.state.control_generation,
            )
        ):
            raise RuntimeError("source dispatch token changed during physical commit")

    def dispatch_pose_command(self, controller: CommandController) -> bool:
        """Validate and dispatch one teleoperation target, then commit its segment."""
        with self._input_command_lock:
            with self.state.status_lock:
                hand_state = self.state.hands.get(controller.hand_binding, {})
                arm_state = self.state.arm_states.get(controller.arm_role, {})
                button_data = hand_state.get("button_data")
                dispatch_allowed = (
                    self.active_source_binding is not None
                    and self.active_source_binding.phase is BindingPhase.ACTIVE
                    and self.active_source_binding.input_ready
                    and self.active_input_source == controller.target_input_source
                    and self.input_generation == controller.target_input_generation
                    and self.state.ble_connected
                    and arm_state.get("connected")
                    and arm_state.get("hand_binding") == controller.hand_binding
                    and hand_state.get("input_sequence") == controller.target_input_sequence
                    and self.state.control_generation == controller.target_control_generation
                    and button_data is not None
                    and button_data.grip > 100
                )
                module = self.source_registry.active_module()
                dispatch_allowed = bool(
                    dispatch_allowed
                    and module is not None
                    and module.validate_dispatch(
                        controller.hand_binding,
                        controller,
                        controller.source_controller_generation,
                        controller.target_source_token,
                        self.input_generation,
                        self.state.control_generation,
                    )
                )

            with controller.pose_state_lock:
                if (
                    not dispatch_allowed
                    or controller.isStop
                    or controller.target_pose_epoch != controller.pose_epoch
                    or controller.target_arm_connection_generation
                    != controller.arm_connection_generation
                    or controller.target_arm_feedback_state_sequence
                    != controller.arm_feedback_state_sequence
                    or controller.target_end_arm_pose is None
                    or not controller.carm_
                    or not controller.carm_.is_connected()
                    or not controller._arm_connected
                    or controller.arm_feedback_connection_generation
                    != controller.arm_connection_generation
                    or controller.carm_.ws is not controller._arm_ws_identity
                    or controller.carm_.state is not controller._arm_state_identity
                ):
                    controller.reject_pose_target()
                    return False

                dispatched = controller._dispatch_pose_current_connection(
                    controller.target_end_arm_pose[:7]
                )
                if dispatched is not True:
                    logging.error(
                        "[JoystickController] track_pose rejected for %s: %r",
                        controller.arm_role,
                        dispatched,
                    )
                    if module is not None:
                        module.reject_dispatch(
                            controller.hand_binding,
                            self.input_generation,
                            self.state.control_generation,
                            controller.target_source_token,
                        )
                    controller.invalidate_pose_mapping()
                    return False

                return True

    def set_arm_runtime_state(
        self,
        role: str,
        ip: Optional[str],
        hand_binding: Optional[str],
        connected: bool,
        control_host: Optional[str] = None,
        device_index: Optional[int] = None,
    ) -> None:
        with self._input_command_lock:
            with self.state.status_lock:
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
            self.gripper_max_value,
            self,
            hand_binding=hand_binding,
            arm_role=role,
            gripper_tau=arm_config.gripper_tau,
            configured_init_joint=arm_config.init_joint,
        )
        controller.pose_update_callback = lambda gripper_pose, gripper_tau, arm_role=role: self._update_gripper_state(
            arm_role, gripper_pose, gripper_tau
        )
        self._assign_source_controller(controller)
        return controller

    def _safe_execute(self, operation: callable, error_msg: str, log_level: str = "error") -> bool:
        """安全执行操作，自动处理异常和日志记录

        Args:
            operation: 要执行的操作函数
            error_msg: 错误信息
            log_level: 日志级别 ("debug", "info", "warning", "error")

        Returns:
            bool: 操作是否成功
        """
        try:
            operation()
            return True
        except Exception as e:
            log_func = getattr(logging, log_level, logging.error)
            log_func(f"{error_msg}: {e}")
            if log_level == "error":
                logging.exception(e)
            return False

    def _stop_controller_thread(self, move_home: bool = False, timeout: float = CONTROLLER_THREAD_TIMEOUT) -> bool:
        """停止控制器线程（辅助方法，避免代码重复）

        Args:
            move_home: 是否在停止控制输出后回零
            timeout: 等待线程停止的超时时间

        Returns:
            bool: 线程是否成功停止
        """
        # 停止控制器
        if self.controller:
            self.controller.stop(move_home=move_home)

        if not self.controller_thread:
            return True

        if not self.controller_thread.is_alive():
            self.controller_thread = None
            return True

        # 等待线程停止
        self.controller_thread.join(timeout=timeout)
        if self.controller_thread.is_alive():
            logging.error("[JoystickController] Controller thread did not stop within timeout")
            return False

        self.controller_thread = None
        return True

    def _ensure_controller_thread_stopped(self) -> bool:
        """确保控制器线程已停止（辅助方法，用于init_arm前的检查）

        Returns:
            bool: 如果线程已停止或不存在则返回True，否则返回False
        """
        if not self.controller_thread:
            return True

        if not self.controller_thread.is_alive():
            self.controller_thread = None
            return True

        return False

    def _start_cleanup_thread(self):
        """启动常驻清理线程"""
        with self._cleanup_lock:
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

    def _cleanup_worker(self):
        """常驻清理工作线程，处理所有清理请求"""
        while not self._cleanup_shutdown:
            try:
                # 等待清理请求或关闭信号
                if self._cleanup_event.wait(timeout=WORKER_CHECK_INTERVAL):  # 定期检查shutdown标志
                    self._cleanup_event.clear()

                    # 执行清理
                    with self._cleanup_lock:
                        self._cleanup_in_progress = True

                    try:
                        with self._cleanup_lock:
                            reason = self._cleanup_reason
                            expected_generation = self._cleanup_input_generation
                            self._cleanup_reason = "unknown"
                            self._cleanup_input_generation = None
                        with self._input_session_lock:
                            if expected_generation is not None:
                                with self._input_command_lock:
                                    if (
                                        self.input_generation != expected_generation
                                        or self.active_input_source is not None
                                    ):
                                        logging.info(
                                            "[CleanupWorker] Skipping stale cleanup, reason=%s",
                                            reason,
                                        )
                                        continue
                            logging.info("[CleanupWorker] Starting hold cleanup, reason=%s", reason)
                            self.stop_session(stop_mode="hold", reason=reason)
                            logging.info("[CleanupWorker] Cleanup operation completed")
                    except Exception as e:
                        logging.error(f"[CleanupWorker] Cleanup operation failed: {e}")
                        logging.exception(e)
                    finally:
                        with self._cleanup_lock:
                            self._cleanup_in_progress = False
                # 如果是timeout，继续循环检查shutdown标志

            except Exception as e:
                logging.error(f"[CleanupWorker] Worker error: {e}")
                # 短暂延迟，避免异常循环
                time.sleep(0.1)

        logging.info("[CleanupWorker] Cleanup worker thread shutting down")

    def _stop_controller(self, move_home: bool = False) -> bool:
        """停止控制器和控制线程"""
        if self.session:
            try:
                success = self.session.stop(move_home=move_home)
            finally:
                self.session = None
                self.controller = None
                self.controller_thread = None
            return success

        if not self.controller:
            return True

        try:
            logging.info("[JoystickController] Stopping control loop (move_home=%s)", move_home)

            if not self._stop_controller_thread(move_home=move_home):
                return False
            logging.info("[JoystickController] Controller thread stopped successfully")

            # 断开机械臂连接（在移动到零位之后）
            if self.controller and self.controller.carm_:
                try:
                    self.controller.carm_.disconnect()
                    logging.info("[JoystickController] Arm disconnected")
                except Exception as e:
                    logging.error(f"[JoystickController] Error disconnecting arm: {e}")
                    # 即使断开失败，也继续清理controller引用

            # 清理controller引用
            self.controller = None
            return True

        except Exception as e:
            logging.exception(e)
            return False

    def _stop_sdk(self) -> bool:
        """Close the exact active input lease through its module owner."""
        module = self.source_registry.active_module()
        if module is None:
            return True
        lease = module.current_lease()
        if lease is None:
            return True
        return self._close_source_lease(lease, "backend_stop")

    def _stop_sdk_loop(self) -> bool:
        """Connection close owns provider transport-loop shutdown."""
        return True

    def _cleanup_resources(self) -> bool:
        """Release scan-only or deferred resources through fixed modules."""
        success = True
        for provider_id in self.source_registry.provider_ids:
            module = self.source_registry.existing(provider_id)
            if module is not None and module.current_lease() is None:
                success = bool(module.cleanup_idle_resources()) and success
        return success

    def _reset_state(self) -> None:
        """重置状态"""
        self._cancel_source_reconnect_timers(clear_deadline=True)
        with self._input_command_lock:
            self._close_binding_locked()
            self.input_generation += 1
            self.active_input_source = None
            self._reset_source_policies()
        with self.state.status_lock:
            self.state.ble_connected = False
            self.state.arm_initialized = False
            self.state.status = "idle"
            self.state.connected_address = None
            self.state.connected_name = None
            self.state.source_type = None
            self.state.mode = "single"
            self.state.master_side = HAND_RIGHT
            self.state.view_side = HAND_RIGHT
            self.state.is_quest_device = False
            self.state.transport_state = "disconnected"
            self.state.quest_protocol_version = None
            self.state.quest_connection_id = None
            self.state.quest_last_rx_monotonic = None
            self.state.quest_last_disconnect_reason = None
            self.state.quest_transport_generation = 0
            self.state.pose_data = None
            self.state.button_data = None
            self.state.battery_level = None
            self.state.rssi = None
            self.state.gripper_pose = None
            self.state.gripper_tau = None
            self.state.hands = {
                HAND_LEFT: {
                    "pose_data": None,
                    "button_data": None,
                    "gripper_pose": None,
                    "gripper_tau": None,
                    "input_sequence": 0,
                },
                HAND_RIGHT: {
                    "pose_data": None,
                    "button_data": None,
                    "gripper_pose": None,
                    "gripper_tau": None,
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
                    "gripper_pose": None,
                    "gripper_tau": None,
                },
                "slave": {
                    "ip": None,
                    "control_host": None,
                    "arm_index": None,
                    "hand_binding": None,
                    "connected": False,
                    "gripper_pose": None,
                    "gripper_tau": None,
                },
            }
            self.state.control_generation += 1
            self.state.data_sequence += 1

    def shutdown_cleanup_thread(self):
        """停止清理线程"""
        # 检查当前线程是否是清理线程本身
        current_thread = threading.current_thread()
        is_cleanup_thread = (self._cleanup_thread and
                            current_thread.ident == self._cleanup_thread.ident)

        with self._cleanup_lock:
            self._cleanup_shutdown = True

        # 设置事件以唤醒等待的线程
        self._cleanup_event.set()

        if is_cleanup_thread:
            logging.info("[JoystickController] Cleanup thread shutting down itself, skipping join()")
            return True

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
        if (
            module is not None
            and module.reconnect_grace_expired(time.monotonic())
            and self.connection_is_current(batch.lease, provider_identity)
        ):
            self._expire_source_reconnect_deadline(batch.lease)
            return
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
        with self._input_command_lock:
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
            with self.state.status_lock:
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
                if self.state.status != "initializing_arm":
                    self.state.status = "connected"
                self.state.last_error = None
                self.state.source_type = binding.public_source_type
                self.state.is_quest_device = bool(
                    binding.capabilities.supports_reconnect_grace
                )
                self.state.transport_state = status.phase
                self.state.quest_protocol_version = details.get("protocol_version")
                self.state.quest_connection_id = details.get("connection_id")
                self.state.quest_last_rx_monotonic = details.get("last_rx_monotonic")
                self.state.quest_transport_generation = details.get(
                    "generation",
                    batch.transport_generation,
                )
                self.state.last_update_ts = time.time()
                self._sync_view_side_payload()
                self.state.data_sequence += 1
            for controller in rejected_controllers - invalidated_controllers:
                controller.reject_pose_target()
            for controller in invalidated_controllers:
                controller.invalidate_pose_mapping()

    def _close_source_lease(self, lease: SourceLease, reason: str) -> bool:
        module = self.source_registry.get_or_create(lease.provider_id)
        success = module.close_connection(lease, reason)
        if success:
            self.source_registry.complete_close(lease)
        return success

    def request_stop_session(
        self,
        reason: str = "unknown",
        expected_input_generation: Optional[int] = None,
    ) -> None:
        """请求停止会话（线程安全，非阻塞）"""
        with self._cleanup_lock:
            if self._cleanup_in_progress or self._cleanup_shutdown:
                logging.warning("[JoystickController] Cleanup already in progress or shutting down, ignoring new request")
                return

            self._cleanup_reason = reason
            self._cleanup_input_generation = expected_input_generation

        logging.info("[JoystickController] Requesting hold cleanup, reason=%s", reason)
        self._cleanup_event.set()

    def _schedule_cleanup_request(
        self,
        delay: float = 0.2,
        wait_for_initialization: bool = False,
        source: str = "unknown",
        expected_input_generation: Optional[int] = None,
    ) -> None:
        def callback():
            with self._deferred_cleanup_timer_lock:
                if self._deferred_cleanup_timer is timer:
                    self._deferred_cleanup_timer = None

            try:
                if expected_input_generation is not None:
                    with self._input_command_lock:
                        if (
                            self.input_generation != expected_input_generation
                            or self.active_input_source is not None
                        ):
                            return
                with self._cleanup_lock:
                    if self._cleanup_in_progress or self._cleanup_shutdown:
                        return

                if wait_for_initialization:
                    with self.state.status_lock:
                        is_initializing = (
                            self.state.status == "initializing_arm"
                            or (self.controller and self.controller.carm_ and not self.state.arm_initialized)
                        )
                    if is_initializing:
                        logging.info(f"[JoystickController] Delaying deferred cleanup until initialization completes ({source})")
                        self._schedule_cleanup_request(
                            delay=delay,
                            wait_for_initialization=True,
                            source=source,
                            expected_input_generation=expected_input_generation,
                        )
                        return

                self.request_stop_session(
                    reason=source,
                    expected_input_generation=expected_input_generation,
                )
            except Exception as e:
                logging.error(f"[JoystickController] Failed to run deferred cleanup ({source}): {e}")

        timer = threading.Timer(delay, callback)
        timer.daemon = True

        with self._deferred_cleanup_timer_lock:
            previous_timer = self._deferred_cleanup_timer
            self._deferred_cleanup_timer = timer

        if previous_timer and previous_timer.is_alive():
            previous_timer.cancel()

        timer.start()

    def initialize(self):
        """Initialize the host without constructing provider resources."""
        if not self.available:
            return False
        self._initialized = True
        return True

    def _update_gripper_state(self, role: str, gripper_pose, gripper_tau):
        """更新 gripper_pose 和 gripper_tau 到 state（由 CommandController 调用）"""
        with self.state.status_lock:
            arm_state = self.state.arm_states.get(role)
            if not arm_state:
                return
            arm_state["gripper_pose"] = float(gripper_pose) if gripper_pose is not None else None
            arm_state["gripper_tau"] = float(gripper_tau) if gripper_tau is not None else None
            hand_binding = arm_state.get("hand_binding")
            if hand_binding in self.state.hands:
                self.state.hands[hand_binding]["gripper_pose"] = arm_state["gripper_pose"]
                self.state.hands[hand_binding]["gripper_tau"] = arm_state["gripper_tau"]
            self._sync_view_side_payload()
            self.state.data_sequence += 1

    def scan_devices(self) -> list:
        with self._input_session_lock:
            return self._scan_devices_locked()

    def _scan_devices_locked(self) -> list:
        with self.state.status_lock:
            self.state.status = "scanning"
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
            with self.state.status_lock:
                self.state.scan_results = devices
                self.state.status = "idle"
                self.state.last_error = None
                self.state.is_quest_device = is_grace_source
            logging.info(
                "[JoystickController] Scan completed, found %d devices",
                len(devices),
            )
            return devices
        except SourceScanError as exc:
            with self.state.status_lock:
                self.state.scan_results = []
                self.state.status = "idle"
                self.state.last_error = str(exc)
            return []
        except (asyncio.TimeoutError, FuturesTimeoutError):
            error_msg = "扫描超时，请检查蓝牙是否开启"
            logging.warning("[JoystickController] %s", error_msg)
            with self.state.status_lock:
                self.state.status = "idle"
                self.state.last_error = error_msg
            return []
        except Exception as exc:
            error_msg = f"扫描设备失败: {exc}"
            logging.exception(exc)
            with self.state.status_lock:
                self.state.status = "idle"
                self.state.last_error = error_msg
            return []

    def connect_device(self, address: str, name: str = "Unknown") -> bool:
        with self._input_session_lock:
            return self._connect_device_locked(address, name)

    def _connect_device_locked(self, address: str, name: str) -> bool:
        active_module = self.source_registry.active_module()
        if (
            self.state.ble_connected
            or (
                active_module is not None
                and active_module.current_lease() is not None
            )
        ):
            if not self.stop_session(
                stop_mode="hold",
                reason="replace-connected-device",
            ):
                with self.state.status_lock:
                    self.state.last_error = "旧遥操连接清理失败，请重试停止后再连接"
                return False

        descriptor = self._resolve_device_descriptor(address, name)
        if descriptor is None:
            with self.state.status_lock:
                self.state.status = "idle"
                self.state.last_error = (
                    self.source_registry.resolution_error(address, name)
                    or "不支持的遥操设备"
                )
            return False

        candidate = descriptor.candidate
        module = self.source_registry.get_or_create(candidate.provider_id)
        availability = module.availability()
        if not availability.available:
            with self.state.status_lock:
                self.state.status = "idle"
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
            with self.state.status_lock:
                self.state.status = "connecting"
                self.state.last_error = None
                self.state.connected_address = candidate.address
                self.state.connected_name = candidate.name
                self.state.source_type = candidate.public_source_type
                self.state.is_quest_device = bool(
                    candidate.capabilities.supports_reconnect_grace
                )
                self.state.data_sequence += 1

            result = module.connect(candidate, lease, self)
            with self._input_command_lock:
                binding = self.active_source_binding
                current = bool(
                    binding is not None
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
                    with self.state.status_lock:
                        self.state.ble_connected = True
                        self.state.status = "connected"
                        self.state.last_error = None
                        self.state.transport_state = "ready"
                        self.state.data_sequence += 1
                elif current and result.outcome is ConnectOutcome.PENDING:
                    with self.state.status_lock:
                        self.state.ble_connected = False
                        self.state.status = "connecting"
                        self.state.last_error = result.message
                        self.state.data_sequence += 1
            if not current:
                self._close_source_lease(lease, "input-connect-stale")
                return False
            if result.outcome is ConnectOutcome.READY:
                return True
            if result.outcome is ConnectOutcome.PENDING and result.retain_transport:
                return False

            self._mark_input_disconnected(
                "input-connect-failed",
                expected_lease=lease,
            )
            self._close_source_lease(lease, "input-connect-failed")
            with self.state.status_lock:
                self.state.status = "idle"
                self.state.last_error = result.message
            return False
        except Exception as exc:
            logging.exception("[JoystickController] Input module connect failed")
            if lease is not None:
                self._mark_input_disconnected(
                    "input-connect-error",
                    expected_lease=lease,
                )
                self._close_source_lease(lease, "input-connect-error")
            with self.state.status_lock:
                self.state.ble_connected = False
                self.state.status = "idle"
                self.state.last_error = f"连接设备失败: {exc}"
            return False

    def get_last_error(self) -> str:
        """获取最后一次操作的错误信息"""
        with self.state.status_lock:
            return self.state.last_error

    def init_arm(self, master_ip: str, mode: str = "single", slave_ip: Optional[str] = None, master_side: str = HAND_RIGHT) -> bool:
        """初始化机械臂"""
        if not self.available:
            logging.warning("[JoystickController] Controller not available")
            return False

        with self._input_command_lock:
            binding = self.active_source_binding
            with self.state.status_lock:
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

        try:
            normalized_mode = (mode or "single").strip().lower()
            normalized_master_side = normalize_hand(master_side, HAND_RIGHT)
            with self.state.status_lock:
                self.state.status = "initializing_arm"
                self.state.mode = normalized_mode
                self.state.master_side = normalized_master_side
                self.state.view_side = self.state.master_side
                self.state.last_error = ""

            if self.session:
                self._stop_controller(move_home=False)

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
            session_cls = DualArmSession if self.state.mode == "dual" else SingleArmSession
            session = session_cls(self, bindings, self._build_command_controller)

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
                with self.state.status_lock:
                    self.state.arm_initialized = False
                    self.state.status = "connected" if self.state.ble_connected else "idle"
                return False

            self.session = session
            self.controller = session.get_primary_controller()
            with self.state.status_lock:
                self.state.arm_initialized = True
                self.state.status = "arm_ready"
            return True
        except JoystickConfigError as e:
            error_msg = f"遥操配置错误：{e}"
            logging.error("[JoystickConfig] %s", error_msg)
            with self.state.status_lock:
                self.state.arm_initialized = False
                self.state.status = "connected" if self.state.ble_connected else "idle"
                self.state.last_error = error_msg
            return False
        except Exception as e:
            error_msg = f"机械臂初始化异常：{str(e)}"
            logging.error(error_msg)
            logging.exception(e)
            with self.state.status_lock:
                self.state.arm_initialized = False
                self.state.status = "connected" if self.state.ble_connected else "idle"
                self.state.last_error = error_msg
            return False

    def set_calibration(self, value: int) -> bool:
        """设置标定模式"""
        try:
            CalibrationMode(value)
            with self._input_command_lock:
                affected_controllers = []
                with self.state.status_lock:
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

            with self._input_command_lock:
                self.scale = value
                affected_controllers = []
                with self.state.status_lock:
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
            with self.state.status_lock:
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
            with self.state.status_lock:
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

    def get_stream_snapshot(self) -> dict:
        """Build one generation-consistent snapshot for the SSE serializer."""
        with self._input_command_lock:
            binding = self.active_source_binding
            binding_active = bool(
                binding is not None and binding.phase is not BindingPhase.CLOSED
            )
            supports_reconnect_grace = bool(
                binding is not None
                and binding.capabilities.supports_reconnect_grace
            )
            with self.state.status_lock:
                transport = {
                    "state": self.state.transport_state,
                    "protocol": self.state.quest_protocol_version,
                    "connection_id": self.state.quest_connection_id,
                    "last_rx_age_ms": (
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
                    ),
                    "disconnect_reason": self.state.quest_last_disconnect_reason,
                }
                stream_connected = bool(
                    self.state.ble_connected
                    or (
                        binding_active
                        and supports_reconnect_grace
                        and transport["state"] in {"negotiating", "healthy"}
                    )
                )
                emit_disconnect_frame = bool(
                    supports_reconnect_grace
                    and not binding_active
                    and not self.state.ble_connected
                )
                hands = {
                    hand: {
                        "pose": (
                            pose_to_dict(self.state.hands[hand]["pose_data"])
                            if self.state.hands[hand]["pose_data"]
                            else None
                        ),
                        "button": (
                            button_to_dict(self.state.hands[hand]["button_data"])
                            if self.state.hands[hand]["button_data"]
                            else None
                        ),
                        "gripper_pose": self.state.hands[hand]["gripper_pose"],
                        "gripper_tau": self.state.hands[hand]["gripper_tau"],
                    }
                    for hand in (HAND_LEFT, HAND_RIGHT)
                }
                return {
                    "sequence": self.state.data_sequence,
                    "pose": (
                        pose_to_dict(self.state.pose_data)
                        if self.state.pose_data
                        else None
                    ),
                    "button": (
                        button_to_dict(self.state.button_data)
                        if self.state.button_data
                        else None
                    ),
                    "gripper_pose": self.state.gripper_pose,
                    "gripper_tau": self.state.gripper_tau,
                    "status": {
                        "ble_connected": stream_connected,
                        "status": self.state.status,
                        "battery_level": self.state.battery_level,
                        "rssi": self.state.rssi,
                        "source_type": self.state.source_type,
                        "mode": self.state.mode,
                        "master_side": self.state.master_side,
                    },
                    "device": {
                        "address": self.state.connected_address,
                        "name": self.state.connected_name,
                    },
                    "arms": {
                        "master": dict(self.state.arm_states["master"]),
                        "slave": dict(self.state.arm_states["slave"]),
                    },
                    "hands": hands,
                    "transport": transport,
                    "stream_terminal": bool(
                        not self.state.ble_connected
                        and (not supports_reconnect_grace or not binding_active)
                    ),
                    "emit_disconnect_frame": emit_disconnect_frame,
                }

    def get_status(self) -> dict:
        """获取当前状态"""
        if not self.available:
            return {
                "available": False,
                "status": "not_available",
                "ble_connected": False,
                "arm_initialized": False,
                "calibration": 0,
                "scale": self.scale if hasattr(self, "scale") else 0.4,
                "battery_level": None,
                "rssi": None,
                "source_type": None,
                "mode": "single",
                "is_quest_device": False,
                "master_side": HAND_RIGHT,
                "view_side": HAND_RIGHT,
                "master_arm": {},
                "slave_arm": {},
                "pose": None,
                "button": None,
                "transport": {
                    "state": "disconnected",
                    "protocol": None,
                    "connection_id": None,
                    "last_rx_age_ms": None,
                    "disconnect_reason": None,
                },
            }
        with self.state.status_lock:
            view_side = normalize_hand(self.state.view_side, HAND_RIGHT)
            active_hand = self.state.hands.get(view_side, {})
            last_rx_age_ms = (
                max(0, int((time.monotonic() - self.state.quest_last_rx_monotonic) * 1000))
                if self.state.quest_last_rx_monotonic is not None
                else None
            )
            return {
                "ble_connected": self.state.ble_connected,
                "arm_initialized": self.state.arm_initialized,
                "calibration": self.state.calibration,
                "scale": self.state.scale,
                "status": self.state.status,
                "battery_level": self.state.battery_level,
                "rssi": self.state.rssi,
                "source_type": self.state.source_type,
                "mode": self.state.mode,
                "is_quest_device": self.state.is_quest_device,
                "master_side": self.state.master_side,
                "view_side": view_side,
                "master_arm": dict(self.state.arm_states["master"]),
                "slave_arm": dict(self.state.arm_states["slave"]),
                "pose": pose_to_dict(active_hand.get("pose_data")) if active_hand.get("pose_data") else None,
                "button": button_to_dict(active_hand.get("button_data")) if active_hand.get("button_data") else None,
                "transport": {
                    "state": self.state.transport_state,
                    "protocol": self.state.quest_protocol_version,
                    "connection_id": self.state.quest_connection_id,
                    "last_rx_age_ms": last_rx_age_ms,
                    "disconnect_reason": self.state.quest_last_disconnect_reason,
                },
            }

    def get_configured_init_position(self) -> dict:
        """返回当前会话从配置快照解析出的主逻辑侧初始化位置。"""
        try:
            with self.state.status_lock:
                mode = self.state.mode
                master_side = normalize_hand(self.state.master_side, HAND_RIGHT)
                master_ip = self.state.arm_states["master"].get("ip")
                slave_ip = self.state.arm_states["slave"].get("ip")

            arm_config = self._active_arm_configs.get(master_side)
            if arm_config is None:
                use_seven_axis_joints = (
                    mode == "dual"
                    and bool(master_ip)
                    and bool(slave_ip)
                    and master_ip == slave_ip
                )
                arm_config = self.joystick_config.resolve_session(
                    mode,
                    master_side,
                    use_seven_axis_joints,
                )[master_side]

            return {"init_joint": list(arm_config.init_joint)}
        except JoystickConfigError as e:
            error_msg = f"遥操配置错误：{e}"
            logging.error("[JoystickConfig] %s", error_msg)
            return {"init_joint": None, "error": error_msg}

    def _build_init_position_controller(self) -> CommandController:
        """为旧初始化位置接口创建采用单臂配置初值的控制器。"""
        arm_config = self.joystick_config.resolve_session("single", HAND_RIGHT, False)[HAND_RIGHT]
        return CommandController(
            self.scale,
            self.gripper_max_value,
            self,
            arm_role="master",
            gripper_tau=arm_config.gripper_tau,
            configured_init_joint=arm_config.init_joint,
        )

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
                self.controller = CommandController(self.scale, self.gripper_max_value, self, arm_role="master")
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
        with self._input_session_lock:
            with self._session_stop_lock:
                return self._stop_session_locked(stop_mode=stop_mode, reason=reason)

    def _stop_session_locked(self, stop_mode: str, reason: str) -> bool:
        """串行执行会话清理，避免 home 与异常 hold 并发操作同一机械臂。"""

        self._cancel_source_reconnect_timers(clear_deadline=True)

        normalized_mode = (stop_mode or "hold").strip().lower()
        if normalized_mode not in ("hold", "home"):
            logging.warning("[JoystickController] Unsupported stop mode %s; falling back to hold", stop_mode)
            normalized_mode = "hold"
        binding = self.active_source_binding
        with self.state.status_lock:
            input_connected = bool(
                self.state.ble_connected
                and binding is not None
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
            )
        if normalized_mode == "home" and not input_connected:
            logging.warning("[JoystickController] Input already disconnected; downgrading home cleanup to hold")
            normalized_mode = "hold"
        move_home = normalized_mode == "home"

        logging.info(
            "[JoystickController] Beginning session cleanup (mode=%s, reason=%s)",
            normalized_mode,
            reason,
        )

        # 按顺序停止各个组件
        success = True
        success &= self._stop_controller(move_home=move_home)
        with self._input_command_lock:
            self._close_binding_locked()
        success &= self._stop_sdk()
        success &= self._stop_sdk_loop()
        success &= self._cleanup_resources()

        # 重置状态
        self._reset_state()

        logging.info("[JoystickController] Session cleanup completed" + (" with errors" if not success else " successfully"))
        return success

    def set_cached_init_position(self, role: str, position: List[float]) -> None:
        """设置旧接口使用的进程内初始化位置。"""
        if not _is_valid_joint_position(position, 6):
            logging.error("[JoystickController] Position must contain exactly 6 finite numbers: %s", position)
            return

        try:
            with self.state.status_lock:
                self.cached_init_positions[role] = list(position)
                logging.info("[JoystickController] Cached init position for %s: %s", role, position)
        except Exception as e:
            logging.error(f"[JoystickController] Error caching init position: {e}")

    def get_cached_init_position(self, role: str) -> Optional[List[float]]:
        """获取旧接口保存的进程内初始化位置。"""
        try:
            with self.state.status_lock:
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
