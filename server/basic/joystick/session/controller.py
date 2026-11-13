"""遥操 Host 会话状态机（joystick 全链路的核心编排层）。

用户操作排他（``user_operation``）、输入源 binding 与发布、双手同批信箱和清理编排
在本模块；观测快照、维护操作、腰与臂型角的启用判断分别在 ``observation``、
``maintenance``、``waist``、``arm_angle``。输入发布只更新冻结值与单调计数，控制器映射由
所属臂的控制线程独占。生命周期锁序为 ``_lifecycle_lock`` > ``_data_lock``。
"""

import asyncio
import copy
import logging
import math
import numbers
import threading
import time
from concurrent.futures import TimeoutError as FuturesTimeoutError
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from typing import Any, Dict, Optional, Tuple

from ..runtime.config import JOYSTICK_CONFIG, JoystickConfigError
from ..runtime.dependencies import ARM_SDK_AVAILABLE
from carm_teleop_input_contracts import (
    ConnectOutcome,
    ConnectResult,
    DeviceStatusItem,
    HAND_LEFT,
    HAND_RIGHT,
    RawInputBatch,
    SourceLease,
    TransportEvent,
    normalize_hand,
    JointInputBatch,
    UnifiedHandSample,
)
from ..sources.registry import (
    ActiveSourceBinding,
    BindingPhase,
    SourceLeaseAllocator,
    SourceRegistry,
    SourceScanError,
)
from ..core.command_controller import (
    DEFAULT_MOVE_SPEED_LEVEL, MOVE_SPEED_LEVELS, CommandController, ControlParams,
)
from ..core.mailbox import (
    HandInputFrame,
    InputPublication,
    JointInputFrame,
    JointPublication,
    LatestValueCell,
    PublicationSignal,
    hand_cells,
)
from ..core.pose_math import legacy_mode_to_yaw
from ..end_effectors import EndEffectorFactory, resolve_end_effector_factory
from quest.module import QuestInputModule
from .arm import (
    BaseArmSession, active_arm_profile, apply_profile_device_indexes, build_arm_bindings,
)
from .cleanup import CleanupContext, CleanupMode, SessionCleanup
from .state import JoystickState, SessionPhase
from .waist import create_waist_runtime
from ..runtime.carm_link import CarmLinkRegistry
from .arm_angle import arm_angle_state, create_arm_angle_runtimes
from .maintenance import ArmMaintenance
from .observation import ObservationSnapshot, build_observation_snapshot, project_http_status


_CLEANUP_THREAD_TIMEOUT = 2.0
_WORKER_CHECK_INTERVAL = 1.0
# 自动重试上界：仅限制"失败后是否再次入队"，不限制单次外部拆卸调用的退出时间。
_AUTO_RETRY_WINDOW_S = 10.0
_CLEANUP_WORKER_NAME = "JoystickCleanupWorker"


@dataclass(frozen=True)
class StopRequest:
    """一次与会话世代绑定的停止请求（清理线程的入队单元）。"""

    reason: str
    session_id: Optional[int]
    lease: Optional[SourceLease]
    # 首次清理失败的时刻；新请求（含新世代请求与手动重试后的重新入队）为 None，即重开窗口。
    first_failure_monotonic: Optional[float] = None


class JoystickController:
    """手柄遥操 Host 会话状态机（对外即"遥操控制器"）。

    持有输入源 binding、arm runtime 会话、控制参数 epoch、每手输入信箱与清理线程，
    对外提供用户操作、provider 发布入口、观测与旧版单机接口。
    """

    # Board 组合注入的 Quest 直连管理器；Desktop 不启用。
    direct_manager = None

    def __init__(
        self,
        scale=0.4,
        joystick_config_snapshot=None,
        source_registry=None,
        arm_binding_builder=build_arm_bindings,
        end_effector_factory: Optional[EndEffectorFactory] = None,
    ):
        """建立会话状态机并启动常驻清理线程。

        Args:
            scale: 初始灵敏度
            joystick_config_snapshot: 配置快照，缺省用进程内 ``JOYSTICK_CONFIG``
            source_registry: 输入源 registry，缺省只注册 quest provider
            arm_binding_builder: 会话到臂绑定的构造器（测试可注入）
            end_effector_factory: 末端执行器工厂，缺省由变体解析
        """
        self.available = True
        self.scale = scale
        self.move_speed_level = DEFAULT_MOVE_SPEED_LEVEL
        self.joystick_config = joystick_config_snapshot or JOYSTICK_CONFIG
        self._carm_links = CarmLinkRegistry()

        self.controller = None
        self.session = None
        self._cleanup_home_allowed = False
        self.active_input_source = None
        # Host 会话世代单调递增，关闭 binding 也消耗一个值，不会重用。
        self._session_sequence = 0
        self._source_lease_allocator = SourceLeaseAllocator()
        self.source_registry = source_registry or SourceRegistry(
            providers=(("quest", QuestInputModule),),
            lease_allocator=self._source_lease_allocator,
        )
        self._arm_binding_builder = arm_binding_builder
        self._end_effector_factory = (
            end_effector_factory or resolve_end_effector_factory("carm")
        )
        self.active_source_binding: Optional[ActiveSourceBinding] = None

        self._lifecycle_lock = threading.RLock()
        self._data_lock = threading.RLock()
        # 双手 publication 单槽信箱：一次批次只 store 一次，读者一次 load 即得双手同批。
        self._discontinuity_seq = {HAND_LEFT: 0, HAND_RIGHT: 0}
        self._input_transport_generations = {}
        self._input_sequences = {HAND_LEFT: 0, HAND_RIGHT: 0}
        self._input_publication = LatestValueCell()
        self._joint_publication = LatestValueCell()
        # 输入线程单独推进；与会话序号相加供 HTTP/SSE 检测变化。
        self._input_publication_sequence = 0
        # 发布唤醒信号：输入/关节帧 store 之后（data 锁外）notify，按输入节拍运行的
        # 控制线程据此在新帧到达时立即醒来；只是唤醒提示，不携带数据。
        self._publication_signal = PublicationSignal()
        # ``_reengage_cells`` 保持每手：写者是各自臂的控制线程，没有批次语义。
        self._reengage_cells = hand_cells()
        self._motion_cancel = threading.Event()
        self.state = JoystickState()
        self.state.scale = scale
        # 已发布的输入映射参数：``epoch`` 只在 yaw/scale 变化时推进。
        self._control_params = ControlParams(
            epoch=0,
            scale=self.state.scale,
            calibration=self.state.calibration,
        )

        self.maintenance = ArmMaintenance(self)

        # 单线程清理模式：使用常驻线程处理所有清理请求
        self._cleanup_event = threading.Event()  # 触发清理的事件
        self._cleanup_thread = None  # 常驻清理线程
        self._cleanup_shutdown = False  # 标记是否正在关闭
        self._pending_cleanup_request: Optional[StopRequest] = None
        self._source_reconnect_deadline: Optional[Tuple[SourceLease, float]] = None
        self._session_cleanup = SessionCleanup(self)

        # 启动清理线程
        self._start_cleanup_thread()

    def _set_phase_locked(self, phase: SessionPhase) -> None:
        """相位真的变化时写入，并推进 ``data_sequence``（调用方需持 data 锁）。"""
        if self.state.phase is not phase:
            self.state.phase = phase
            self.state.data_sequence += 1

    def _publish_control_params_locked(self) -> ControlParams:
        """发布新的控制参数 epoch（递增后返回，仅 yaw/scale 真变化时调用）。"""
        self._control_params = ControlParams(
            epoch=self._control_params.epoch + 1,
            scale=self.state.scale,
            calibration=self.state.calibration,
        )
        return self._control_params

    @contextmanager
    def user_operation(self, action, body):
        """用户操作的排他准入与相位 gate（busy 直接拒绝，不排队等待旧操作）。

        Args:
            action: 操作名（scan/connect/init_arm/session_config/disconnect 等）
            body: 操作参数；``connect`` 时用其中的 ``address`` 校验"先扫描再连接"
        Raises:
            ValueError: 另一个用户操作正在执行（busy 拒绝）
            ValueError: 当前会话相位不允许此操作（相位 gate 拒绝）
        """
        if not self._lifecycle_lock.acquire(blocking=False):
            raise ValueError("另一个遥操操作正在执行")
        try:
            allowed = {
                "scan": {SessionPhase.IDLE},
                "connect": {SessionPhase.IDLE},
                "init_arm": {SessionPhase.CONNECTED},
                "session_config": {SessionPhase.IDLE, SessionPhase.CONNECTED},
                "disconnect": {SessionPhase.IDLE, SessionPhase.CONNECTED, SessionPhase.READY, SessionPhase.CLEANUP_FAILED},
            }.get(action, {SessionPhase.IDLE, SessionPhase.CONNECTED, SessionPhase.READY})
            with self._data_lock:
                if self.state.phase not in allowed:
                    raise ValueError("当前遥操状态不允许此操作")
                if action == "connect" and not self.source_registry.was_scanned(body.get("address")):
                    raise ValueError("请先扫描并选择设备")
            yield
        finally:
            self._lifecycle_lock.release()

    def _stable_phase_locked(self) -> SessionPhase:
        """由实际连接状态推出的稳定相位（READY / CONNECTED / IDLE）。"""
        if self.state.arm_initialized:
            return SessionPhase.READY
        if self.state.ble_connected:
            return SessionPhase.CONNECTED
        return SessionPhase.IDLE

    @staticmethod
    def _optional_interface_enabled(capabilities) -> bool:
        """笛卡尔位姿/重连宽限入口只对具备该能力的模块开放。"""
        return bool(
            getattr(capabilities, "provides_pose", False)
            or getattr(capabilities, "supports_reconnect_grace", False)
        )

    def _module_capabilities(self, module):
        """读取模块自述能力；探测失败即关闭该能力组（返回 ``None``）。"""
        try:
            availability = module.availability()
        except Exception:
            logging.exception(
                "[JoystickController] Availability probe failed: %s",
                getattr(module, "module_id", module),
            )
            return None
        return getattr(availability, "capabilities", None)

    def _call_optional_interface(
        self,
        module,
        method_name: str,
        *args,
        capabilities=None,
        **kwargs
    ) -> bool:
        """能力组开启时调用一个可选方法，返回是否真的调用过。

        方法存在性由 ``BaseInputSourceModule`` 保证；这里只由能力 gate 决定该模块
        是否参与，不参与时静默跳过（返回 ``False``）。
        """
        if capabilities is None:
            capabilities = self._module_capabilities(module)
        if not self._optional_interface_enabled(capabilities):
            return False
        getattr(module, method_name)(*args, **kwargs)
        return True

    def _reset_source_policies(self) -> None:
        """向 provider 发布输入检测重置请求（不改写输入线程的比较状态）。"""
        for provider_id in self.source_registry.provider_ids:
            module = self.source_registry.existing(provider_id)
            if module is not None:
                self._call_optional_interface(module, "reset_policy")

    def _is_hand_bound_active(self, hand: str, controller=None) -> bool:
        """某只手是否被已连接的 arm runtime 绑定为当前控制手（可按 controller 限定）。

        可在锁外调用：``arm_states`` 外层键固定、重置时整体替换，内层只改已有键的值，
        遍历不会因并发写入出错；瞬时不一致只影响输入侧的绑定标志，不影响控制安全。
        """
        for arm_state in self.state.arm_states.values():
            if (
                arm_state.get("connected")
                and arm_state.get("hand_binding") == hand
                and (
                    controller is None
                    or (
                        self.session is not None
                        and any(runtime.controller is controller and runtime.hand == hand
                                for runtime in self.session.arms)
                    )
                )
            ):
                return True
        return False

    def _clear_input_cells(self) -> None:
        """在 data 锁内随会话状态清空双手 publication 与每手 reengage 信箱。"""
        self._input_publication.clear()
        self._joint_publication.clear()
        for hand in (HAND_LEFT, HAND_RIGHT):
            self._reengage_cells[hand].clear()

    def _hand_control_running(self, hand: str) -> bool:
        """绑定变更前检查该手的控制线程是否已退出。"""
        return bool(self.session and any(
            runtime.hand == hand
            and getattr(runtime, "thread", None) is not None
            and runtime.thread.is_alive()
            for runtime in self.session.arms
        ))

    def publish_source_controller(self, controller: CommandController) -> bool:
        """仅在该手控制线程停止后发布绑定；新控制事务自行同步反馈。"""
        with self._data_lock:
            module = self.source_registry.active_module()
            binding = self.active_source_binding
            if module is None or binding is None or binding.phase is BindingPhase.CLOSED:
                return True
            if not self.state.ble_connected or self._hand_control_running(controller.hand_binding):
                return False
            if self._optional_interface_enabled(binding.capabilities):
                module.bind_controller(controller.hand_binding, controller)
            self.state.data_sequence += 1
            return True

    def release_source_controller(self, controller: CommandController) -> None:
        """仅在该手控制线程停止后解除绑定，不改写映射。"""
        with self._data_lock:
            if self._hand_control_running(controller.hand_binding):
                raise RuntimeError("control thread must be joined before binding change")
            module = self.source_registry.active_module()
            if module is None:
                return
            binding = self.active_source_binding
            capabilities = binding.capabilities if binding is not None else None
            if self._optional_interface_enabled(capabilities):
                if not module.release_controller(controller.hand_binding, controller):
                    return
            self.state.data_sequence += 1

    def connection_is_current(
        self,
        lease: SourceLease,
        provider_identity=None,
    ) -> bool:
        """一次 provider 回调是否仍属于当前 binding（lease 相等 + provider 身份复核）。"""
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
        """provider 原始输入批次的发布入口（转交 ``_handle_input_batch``）。"""
        self._handle_input_batch(batch, provider_identity)

    def _input_state_needs_update(self, binding, joint=False) -> bool:
        """无锁预检；稳定帧只发布输入，不反复写相同的会话状态。"""
        return bool(
            binding.phase is BindingPhase.PROVISIONAL or not binding.input_ready
            or not self.state.ble_connected
            or (joint and self.state.transport_state != "ready")
            or (not joint and (
                self.state.phase not in {
                    SessionPhase.CONNECTED, SessionPhase.INITIALIZING, SessionPhase.READY,
                }
                or self.state.last_error is not None
                or self.state.transport_terminal
                or self.state.source_type != binding.public_source_type
                or self.state.is_quest_device != bool(binding.capabilities.supports_reconnect_grace)
            ))
        )

    def _accept_input_state(self, binding, joint=False) -> None:
        """只在就绪状态改变时持 data 锁，并复核该帧仍属于当前 lease。"""
        if not self._input_state_needs_update(binding, joint):
            return
        with self._data_lock:
            current = self.active_source_binding
            if (current is None or current.phase is BindingPhase.CLOSED
                    or current.lease != binding.lease
                    or not self._input_state_needs_update(current, joint)):
                return
            self._activate_binding_locked(current.lease.provider_id, current.lease.input_generation)
            self.state.ble_connected = True
            if joint:
                self.state.transport_state = "ready"
            else:
                if self.state.phase not in {SessionPhase.INITIALIZING, SessionPhase.READY}:
                    self._set_phase_locked(SessionPhase.CONNECTED)
                self.state.last_error = None
                self.state.transport_terminal = False
                self.state.source_type = current.public_source_type
                self.state.is_quest_device = bool(current.capabilities.supports_reconnect_grace)
            self.state.data_sequence += 1

    def publish_joint_input(self, batch, provider_identity) -> None:
        """输入线程按 lease 校验并一次发布关节双手快照。

        Raises:
            TypeError: 传入的不是 ``JointInputBatch``
        """
        if not isinstance(batch, JointInputBatch):
            raise TypeError("expected JointInputBatch")
        binding = self.active_source_binding
        if (binding is None or binding.phase is BindingPhase.CLOSED
                or binding.lease != batch.lease
                or not binding.capabilities.provides_joint_targets):
            return
        resuming = not self.state.ble_connected
        self._accept_input_state(binding, joint=True)
        received = time.monotonic()
        frames = {}
        for hand, target in ((HAND_LEFT, batch.left), (HAND_RIGHT, batch.right)):
            generation = (batch.lease, batch.transport_generation)
            previous_generation = self._input_transport_generations.get(hand)
            if resuming or (previous_generation is not None and previous_generation != generation):
                self._discontinuity_seq[hand] += 1
            self._input_transport_generations[hand] = generation
            frames[hand] = JointInputFrame(
                lease=batch.lease, batch_sequence=batch.batch_sequence,
                target=target, received_monotonic=received,
                discontinuity_seq=self._discontinuity_seq[hand],
            )
        self._joint_publication.store(JointPublication(
            left=frames[HAND_LEFT], right=frames[HAND_RIGHT],
            batch_sequence=batch.batch_sequence, lease=batch.lease,
        ))
        self._input_publication_sequence += 1
        self._publication_signal.notify()

    def publish_telemetry(
        self,
        lease: SourceLease,
        provider_identity,
        items: Tuple[DeviceStatusItem, ...],
    ) -> None:
        """按 item_id 覆盖设备状态项；只有当前 binding 的有效发布才会落盘到状态。"""
        if not isinstance(items, tuple) or not all(
            isinstance(item, DeviceStatusItem) for item in items
        ):
            raise ValueError("device status must be a tuple of DeviceStatusItem")
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
            for item in items:
                self.state.device_status[item.item_id] = item
            self.state.data_sequence += 1

    def publish_transport(
        self,
        event: TransportEvent,
        provider_identity,
    ) -> None:
        """发布传输事件（ready/disconnected/terminal）并编排后续清理或重连宽限。

        非终止断开只做暂停：binding 未关闭、会话身份不变，清空输入信箱；
        终止断开与重连宽限到期才进入
        ``_mark_disconnected_and_request_cleanup``。
        """
        if not self.connection_is_current(event.lease, provider_identity):
            return
        schedule_terminal_cleanup = False
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
                if self._optional_interface_enabled(binding.capabilities):
                    module.mark_input_disconnected()
            if (event.phase in {"disconnected", "terminal"}
                    or (event.phase == "ready"
                        and event.transport_generation != self.state.quest_transport_generation)):
                self._input_publication.clear()
                self._joint_publication.clear()
            self.state.transport_state = event.phase
            self.state.transport_terminal = event.terminal
            if event.terminal:
                self._source_reconnect_deadline = None
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
                    self.state.device_status = {}
                else:
                    self.state.last_error = event.reason
            self.state.data_sequence += 1
            schedule_terminal_cleanup = bool(event.terminal and was_open)
            if (
                not event.terminal
                and event.phase == "disconnected"
                and binding.capabilities.supports_reconnect_grace
                and was_active
            ):
                self._start_source_reconnect_grace_locked(event.lease, module)
        if schedule_terminal_cleanup:
            self._mark_disconnected_and_request_cleanup(
                event.reason or "input-transport-terminal",
                expected_lease=event.lease,
            )

    def _binding_matches_locked(
        self,
        source: str,
        session_id: int,
        require_input_ready: bool = False,
    ) -> bool:
        """复核 source/session_id 是否命中当前 binding，可按需额外要求 input_ready。"""
        binding = self.active_source_binding
        identity_matches = bool(
            binding is not None
            and binding.phase is not BindingPhase.CLOSED
            and binding.lease.provider_id == source
            and binding.session_id == session_id
            and self.active_input_source == source
        )
        if not identity_matches or not require_input_ready:
            return identity_matches
        return bool(
            binding.phase is BindingPhase.ACTIVE
            and binding.input_ready
        )

    def _activate_binding_locked(self, source: str, session_id: int) -> None:
        """PROVISIONAL → ACTIVE（或补写 input_ready）；身份不匹配则不动。"""
        binding = self.active_source_binding
        if not self._binding_matches_locked(source, session_id) or binding is None:
            return
        if binding.phase is BindingPhase.PROVISIONAL:
            self.active_source_binding = binding.activate()
        elif binding.phase is BindingPhase.ACTIVE and not binding.input_ready:
            self.active_source_binding = binding.with_input_ready(True)

    def _set_binding_input_ready_locked(
        self,
        source: str,
        session_id: int,
        ready: bool,
    ) -> None:
        """在 ACTIVE binding 上写 input_ready 标志（身份不匹配则忽略）。"""
        binding = self.active_source_binding
        if (
            binding is not None
            and binding.phase is BindingPhase.ACTIVE
            and self._binding_matches_locked(source, session_id)
        ):
            self.active_source_binding = binding.with_input_ready(ready)

    def _close_binding_locked(self) -> None:
        """关闭当前 binding（经 registry ``begin_close``），保留已关闭的 binding 对象。"""
        self._source_reconnect_deadline = None
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
        """发布新的输入会话世代与 PROVISIONAL binding，返回新的 session_id。

        先让模块接受 lease，再改 Host 可见身份（顺序契约）。

        Args:
            source: provider id
            public_source_type: 对外展示的源类型，缺省同 provider id
            capabilities: 已解析的能力；缺省向模块查询

        Returns:
            新分配且已激活的 Host 会话世代。

        Raises:
            RuntimeError: 上一个输入传输尚未关闭就开始新激活。
        """
        module = self.source_registry.get_or_create(source)
        if capabilities is None:
            capabilities = module.availability().capabilities
        if public_source_type is None:
            public_source_type = source
        if self.session and any(self._hand_control_running(hand) for hand in (HAND_LEFT, HAND_RIGHT)):
            raise RuntimeError("control threads must be joined before source activation")
        if module.current_lease() is not None:
            raise RuntimeError("previous input transport must be closed before activation")
        with self._data_lock:
            session_id = self._session_sequence + 1
            lease = self.source_registry.allocate_lease(
                module.module_id,
                session_id,
            )
            # 模块接收 lease 先于任何 Host 可见身份变更。
            module.bind_lease(lease)
            self.source_registry.activate(lease)
            self._close_binding_locked()
            self._session_sequence = session_id
            self.active_input_source = module.module_id
            self.active_source_binding = ActiveSourceBinding(
                lease=lease,
                session_id=session_id,
                public_source_type=public_source_type,
                capabilities=capabilities,
                phase=BindingPhase.PROVISIONAL,
                input_ready=False,
            )
            self._reset_source_policies()
            self.state.data_sequence += 1
            return session_id

    def _start_source_reconnect_grace_locked(self, lease: SourceLease, module) -> None:
        """首次非终止断开时记录宽限截止时间；中途恢复不重置。"""
        if self._source_reconnect_deadline is None:
            self._source_reconnect_deadline = (
                lease, time.monotonic() + module.reconnect_grace_seconds,
            )
            self._cleanup_event.set()

    def _handle_source_reconnect_deadline_locked(self) -> None:
        """清理线程在取请求前复核到期宽限；只持有 data 锁。"""
        pending = self._source_reconnect_deadline
        if pending is None or time.monotonic() < pending[1]:
            return
        lease, _ = pending
        self._source_reconnect_deadline = None
        binding = self.active_source_binding
        if (binding is None or binding.phase is BindingPhase.CLOSED
                or binding.lease != lease):
            return
        if self.state.transport_state in {"healthy", "ready"}:
            return
        self._expire_source_reconnect_deadline(lease)

    def _expire_source_reconnect_deadline(
        self,
        lease: SourceLease,
    ) -> None:
        """持 data 锁且已复核当前 lease：失效输入并按新会话序号请求清理。"""
        self._mark_input_disconnected(
            "input-reconnect-deadline",
            expected_lease=lease,
        )
        self.request_stop_session(
            reason="input-reconnect-deadline",
            expected_session_id=self._session_sequence,
            expected_lease=lease,
        )

    def _mark_input_disconnected(
        self,
        reason: str,
        expected_lease: Optional[SourceLease] = None,
    ) -> Optional[bool]:
        """在异步资源清理前立即关闭输入连接状态。

        Args:
            reason: 断开原因，写入 ``quest_last_disconnect_reason``
            expected_lease: 期望的 source lease；不匹配时不动任何状态
        Returns:
            bool：清理前相位是否处于 INITIALIZING（调用方据此决定是否落 STOPPING）；
            lease 不匹配时返回 ``None``
        """
        with self._data_lock:
            binding = self.active_source_binding
            if expected_lease is not None and (
                binding is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != expected_lease
            ):
                return None
            self._motion_cancel.set()
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
                # 闭绑定的 session_id 不再复用：关闭也消耗一个序列值。
                self._session_sequence += 1
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
            self._clear_input_cells()
            self.state.device_status = {}
            if not is_initializing:
                self._set_phase_locked(SessionPhase.STOPPING)
            self.state.data_sequence += 1

            self._reset_source_policies()
        return is_initializing

    def _mark_disconnected_and_request_cleanup(
        self,
        reason: str,
        expected_lease: Optional[SourceLease] = None,
    ) -> None:
        """先失效输入再按该世代请求清理；lease 不匹配（已过期）则直接返回。"""
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
            cleanup_session_id = self._session_sequence
        self.request_stop_session(
            reason=reason,
            expected_session_id=cleanup_session_id,
            expected_lease=cleanup_lease,
        )

    def _handle_control_termination(self, termination) -> None:
        """只对"当前这一个"失败的 arm runtime 请求清理。

        各代身份均以对象身份/lease 比对（取代旧版计数器相等）。
        """
        if termination.kind.value != "internal_error":
            return
        with self._data_lock:
            runtime = termination.runtime
            transaction = termination.transaction
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            controller = runtime.controller if runtime is not None else None
            current = bool(
                runtime is not None
                and runtime.transaction is transaction
                and runtime.thread is threading.current_thread()
                and module is not None
                and self.session is not None
                and any(arm is runtime for arm in self.session.arms)
                and controller is termination.controller_identity
                and binding is not None
                and binding.lease == termination.source_lease
                and binding.session_id == termination.session_id
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
        """输入是否真正可用：已连接且 binding 处于 ACTIVE 且 input_ready。"""
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
        """写入单臂 runtime 状态投影（IP/控制端/索引/手别/连接）并刷新 ``arm_initialized``。"""
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

    def set_direct_state(self, payload: dict) -> None:
        """发布直连管理器给出的直连状态，并推进观测序列。"""
        with self._data_lock:
            self.state.direct = copy.deepcopy(payload)
            self.state.data_sequence += 1

    def _build_command_controller(self, role: str, hand_binding: str):
        """按 role/hand_binding 构造 CommandController 并挂上运动取消事件。"""
        # 控制器对象本身就是凭据：一个 arm runtime 的控制器每会话只发布一次，
        # 重新绑定必然伴随新会话与新 binding session_id。
        controller = CommandController(
            self.scale, hand_binding=hand_binding, arm_role=role,
        )
        controller.motion_cancel_event = self._motion_cancel
        controller.move_speed = MOVE_SPEED_LEVELS[self.move_speed_level]
        links = self._shared_carm_links()
        if links is not None:
            controller.carm_factory = links.channel
        return controller

    def _shared_carm_links(self):
        """``sdk.shared_connection`` 启用且 SDK 可用时返回共享连接注册表，否则为 None。

        同一 control_host 上的臂与腰共用一条 SDK 连接；A3 异板双臂每板各一条。
        """
        if not ARM_SDK_AVAILABLE or not self.joystick_config.resolve_shared_connection():
            return None
        return self._carm_links

    def configure_arm_initialization(self, runtime):
        """按当前输入源的 arm profile 固化 runtime 的初始化、回零与遥操回初始位关节。

        Web 确认过初始化位置时，确认值同时用作遥操回初始位目标（优先于配置）。

        Raises:
            ValueError: 型号不在 profile 允许集内
            ValueError: profile 要求的关节自由度与型号不匹配
        """
        controller = runtime.controller
        model = controller.arm_model
        profile = active_arm_profile(self)
        if profile is not None and profile.models and model not in profile.models and not any(
            model.startswith(name) for name in profile.models
        ):
            raise ValueError("当前输入源不支持机械臂型号 %s" % model)
        if profile is not None and profile.dof is not None:
            expected = 7 if model == "D3" else 6
            if profile.dof != expected:
                raise ValueError("当前输入源关节自由度与机械臂型号不匹配")
        config = self.joystick_config.resolve_arm(model, self.state.mode, runtime.hand)
        controller.init_joint_ = list(config.init_joint)
        controller.home_joint_ = list(config.home_joint)
        controller.teleop_reset_joint_ = list(config.teleop_reset_joint)
        if controller.load_cached_init_position(
            self.get_cached_init_position(model, self.state.mode, runtime.hand)
        ):
            controller.teleop_reset_joint_ = list(controller.init_joint_)
        controller._active_init_joint()

    def _set_waist_state(self, status, following=False, mode=None, error=None):
        """发布腰状态投影（HTTP/SSE），并推进数据序列。"""
        value = {"status": status, "following": bool(following), "mode": mode,
                 "error": error}
        with self._data_lock:
            if self.state.waist == value:
                return
            self.state.waist = value
            self.state.data_sequence += 1
            module = self.source_registry.active_module()
        # 模块持有自己的生命周期锁；不得在 Host 数据锁内调用。
        if isinstance(module, QuestInputModule):
            module.set_waist_state(status, following, mode, error)

    def _set_arm_angle_state(self, side, value):
        """更新单臂臂型角状态并把双臂快照同步给 Quest（Web 观测不含臂型角）。"""
        with self._data_lock:
            current = dict(self.state.arm_angle or {})
            if current.get(side) == value:
                return
            current[side] = dict(value)
            current.setdefault("left", arm_angle_state("unavailable"))
            current.setdefault("right", arm_angle_state("unavailable"))
            self.state.arm_angle = current
            module = self.source_registry.active_module()
        if isinstance(module, QuestInputModule):
            module.set_arm_angle_state(current["left"], current["right"])

    def _set_arm_angle_unavailable(self, status):
        """同时设置左右臂型角的准入/禁用状态。"""
        value = arm_angle_state(status)
        self._set_arm_angle_state("left", value)
        self._set_arm_angle_state("right", value)

    def _create_waist_runtime(self, runtimes):
        """决定是否启用腰控制，见 ``create_waist_runtime``。"""
        return create_waist_runtime(self, runtimes)

    def _create_arm_angle_runtimes(self, runtimes):
        """决定是否启用臂型角，见 ``create_arm_angle_runtimes``。"""
        return create_arm_angle_runtimes(self, runtimes)

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
            name=_CLEANUP_WORKER_NAME
        )
        self._cleanup_thread.start()
        logging.info("[JoystickController] Cleanup worker thread started")

    def _requeue_stop_request(self, request: StopRequest) -> bool:
        """入队一个停止请求；已有 pending 时返回 ``False``（不覆盖）。"""
        with self._data_lock:
            if self._pending_cleanup_request is not None:
                return False
            self._pending_cleanup_request = request
            return True

    def _requeue_failed_stop_request(self, request: StopRequest) -> bool:
        """在有界重试窗口内重新入队一次失败的停止请求（超窗返回 ``False``）。"""
        now = time.monotonic()
        first_failure = request.first_failure_monotonic
        if first_failure is None:
            first_failure = now
        elif now - first_failure >= _AUTO_RETRY_WINDOW_S:
            logging.warning(
                "[CleanupWorker] Auto-retry window of %.1fs exhausted, "
                "reason=%s; cleanup stays failed until a manual retry",
                _AUTO_RETRY_WINDOW_S,
                request.reason,
            )
            return False
        return self._requeue_stop_request(
            replace(request, first_failure_monotonic=first_failure)
        )

    def _cleanup_worker(self):
        """单常驻线程消费与会话世代绑定的停止请求（generation-bound）。"""
        while True:
            try:
                with self._data_lock:
                    if self._cleanup_shutdown:
                        logging.info("[CleanupWorker] Cleanup worker thread shutting down")
                        return
                    self._handle_source_reconnect_deadline_locked()
                    request = self._pending_cleanup_request
                    self._pending_cleanup_request = None
                    timeout = _WORKER_CHECK_INTERVAL
                    if self._source_reconnect_deadline is not None:
                        timeout = min(timeout, max(
                            0.0, self._source_reconnect_deadline[1] - time.monotonic(),
                        ))
                if request is None:
                    self._cleanup_event.wait(timeout=timeout)
                    self._cleanup_event.clear()
                    continue

                try:
                    with self._lifecycle_lock:
                        with self._data_lock:
                            cleanup_lease = self.source_registry.cleanup_lease()
                            stale = bool(
                                self._session_sequence != request.session_id
                                or self.active_input_source is not None
                                or (
                                    # provider 已释放时仍需重试 Arm；仅新 lease 接管才过期。
                                    request.lease is not None
                                    and cleanup_lease is not None
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
                            CleanupMode.HOLD, request.reason,
                        )
                        result = self._run_cleanup_locked(context)
                        if not result.success:
                            if self._requeue_failed_stop_request(request):
                                time.sleep(_WORKER_CHECK_INTERVAL)
                            continue
                        logging.info("[CleanupWorker] Cleanup operation completed")
                except Exception:
                    logging.exception("[CleanupWorker] Cleanup operation failed")
                    if self._requeue_failed_stop_request(request):
                        time.sleep(_WORKER_CHECK_INTERVAL)
            except Exception:
                logging.exception("[CleanupWorker] Worker error")
                time.sleep(0.1)

    def _reset_state(self) -> None:
        """重置为 IDLE 会话：关 binding、清信箱、清遥测与臂状态，并重置 provider policy。"""
        self.source_registry.begin_scan()
        with self._data_lock:
            self._close_binding_locked()
            # 关闭也消耗一个序列值，关闭后的 binding id 不再是当前会话。
            self._session_sequence += 1
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
            self.state.device_status = {}
            self._clear_input_cells()
            self.state.waist = {"status": "unavailable", "following": False,
                                "mode": None, "error": None}
            self.state.arm_angle = None
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
            self.state.data_sequence += 1
        self._reset_source_policies()

    def shutdown_cleanup_thread(self):
        """owner 退役成功后停掉唯一的清理线程。

        Returns:
            bool：清理线程在 ``_CLEANUP_THREAD_TIMEOUT`` 内退出为 True，超时为 False
        """
        with self._data_lock:
            self._cleanup_shutdown = True

        self._cleanup_event.set()
        success = True
        if self._cleanup_thread and self._cleanup_thread.is_alive():
            self._cleanup_thread.join(timeout=_CLEANUP_THREAD_TIMEOUT)
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
        """把 address/name 解析为设备描述（由 registry 分派到 provider）。"""
        return self.source_registry.resolve_device(address, name)

    def _handle_input_batch(self, batch: RawInputBatch, provider_identity) -> None:
        """输入线程校验 lease、处理策略并一次发布冻结双手帧。

        稳态不持 data 锁；只有就绪状态变化才进入会话临界区。输入策略与断续计数
        由输入线程独占，其他线程清空后迟到的旧 lease 帧由读者过滤。

        Raises:
            ValueError: 批次的 provider identity 缺失。
        """
        publication = None
        if not isinstance(batch, RawInputBatch):
            raise TypeError("input modules must publish RawInputBatch instances")
        if provider_identity is None:
            raise ValueError("provider identity is required for input batches")
        module = self.source_registry.active_module()
        binding = self.active_source_binding
        if (
            binding is None
            or module is None
            or binding.phase is BindingPhase.CLOSED
            or binding.lease != batch.lease
        ):
            return
        bound_hands = {
            update.hand
            for update in batch.hands
            if (
                (controller := module.controller_for_hand(update.hand))
                is not None
                and self._is_hand_bound_active(update.hand, controller)
            )
        }
        self._accept_input_state(binding)
        decisions = module.process_input_batch(batch, bound_hands)
        # 双手同一批构建后一次 store：单手更新批次里另一手携带同一帧对象，
        # 读者不会看到混合批次。
        previous_publication = self._input_publication.load()
        frames = {
            hand: previous_publication.frame(hand)
            if previous_publication is not None and previous_publication.lease == batch.lease
            else None
            for hand in (HAND_LEFT, HAND_RIGHT)
        }
        for decision in decisions:
            hand = decision.hand
            update = next(update for update in batch.hands if update.hand == hand)
            generation = (batch.lease, batch.transport_generation)
            previous_generation = self._input_transport_generations.get(hand)
            if (decision.invalidate_controller or decision.reject_controller
                    or (previous_generation is not None and previous_generation != generation)):
                self._discontinuity_seq[hand] += 1
            self._input_transport_generations[hand] = generation
            previous = frames[hand]
            if previous is None:
                pose, button, intent, control_sample = None, None, None, None
            else:
                pose, button, intent = previous.pose, previous.button, previous.intent
                control_sample = previous.control_sample
            # 独立接受：位姿无效帧不阻塞按钮/功能结果刷新，反之亦然；按钮与功能结果成对刷新。
            if decision.accept_pose:
                pose = update.pose
            if decision.accept_button:
                button = update.button
                intent = update.intent
            # 显示独立刷新；控制读者比较样本序号，拒绝沿用旧控制样本。
            self._input_sequences[hand] += 1
            sequence = self._input_sequences[hand]
            if decision.accept_pose and decision.accept_button:
                control_sample = UnifiedHandSample(
                    hand=hand, pose=update.pose, button=update.button, intent=update.intent,
                    pose_valid=update.pose_valid,
                    input_sequence=sequence,
                    source_frame_id=batch.batch_sequence,
                    timestamp=max(update.pose.timestamp, update.button.timestamp),
                    provider_id=batch.lease.provider_id,
                    # 这是 Host 会话世代，不是 TCP/BLE 传输世代
                    source_generation=batch.lease.input_generation,
                    atomic=batch.atomic_batch,
                )
            frames[hand] = HandInputFrame(
                hand=hand, pose=pose, button=button, intent=intent,
                control_sample=control_sample, input_sequence=sequence,
                lease=batch.lease, timestamp_monotonic=time.monotonic(),
                discontinuity_seq=self._discontinuity_seq[hand],
            )
            details = dict(decision.details)
            if details.get("event"):
                logging.warning(
                    "[JoystickController] Source policy event: %s hand=%s details=%s",
                    details["event"],
                    decision.hand,
                    details,
                )
        if decisions:
            publication = InputPublication(
                left=frames[HAND_LEFT],
                right=frames[HAND_RIGHT],
                batch_sequence=batch.batch_sequence,
                lease=batch.lease,
                timestamp_monotonic=time.monotonic(),
                waist_input=batch.waist_input,
                body_intent=batch.body_intent,
                transport_generation=batch.transport_generation,
            )
            self._input_publication.store(publication)
        if publication is not None:
            self._input_publication_sequence += 1
            self._publication_signal.notify()

    def _close_source_lease(self, lease: SourceLease, reason: str) -> bool:
        """关闭一个 source lease（begin → close_connection → complete 三段，任一步失败即 False）。"""
        with self._data_lock:
            if (self._source_reconnect_deadline is not None
                    and self._source_reconnect_deadline[0] == lease):
                self._source_reconnect_deadline = None
        if not self.source_registry.begin_close(lease):
            return False
        module = self.source_registry.get_or_create(lease.provider_id)
        if not module.close_connection(lease, reason):
            return False
        return self.source_registry.complete_close(lease)

    def request_stop_session(
        self,
        reason: str = "unknown",
        expected_session_id: Optional[int] = None,
        expected_lease: Optional[SourceLease] = None,
    ) -> None:
        """只把一次与会话世代绑定的清理请求入队，本身不做清理。

        Args:
            reason: 清理原因（透传到日志与清理上下文）
            expected_session_id: 期望的会话世代，``None`` 表示取当前值
            expected_lease: 期望的 source lease，用于在 worker 侧跳过过期请求
        """
        with self._data_lock:
            if self._cleanup_shutdown:
                logging.warning(
                    "[JoystickController] Cleanup is shutting down; ignoring request"
                )
                return
            session_id = (
                self._session_sequence
                if expected_session_id is None
                else expected_session_id
            )
            request = StopRequest(reason, session_id, expected_lease)
            pending = self._pending_cleanup_request
            if (
                pending is None
                or pending.session_id is None
                or session_id >= pending.session_id
            ):
                self._pending_cleanup_request = request

        logging.info("[JoystickController] Requesting hold cleanup, reason=%s", reason)
        self._cleanup_event.set()

    def scan_devices(self) -> list:
        """扫描可用遥操设备（须处于 IDLE 相位，结果按 quest 优先排序）。

        Returns:
            list[dict]：设备描述列表，含 name/address/source_type/能力与 arm_profile；
            相位不允许或扫描失败时返回空列表，失败原因写入 ``last_error``
        """
        with self._lifecycle_lock:
            return self._scan_devices_locked()

    def _scan_devices_locked(self) -> list:
        """扫描实现体（须已持有 ``_lifecycle_lock``）。"""
        with self._data_lock:
            if self.state.phase is not SessionPhase.IDLE:
                self.state.last_error = "请先停止当前遥操会话，再扫描设备"
                return []
            self._set_phase_locked(SessionPhase.SCANNING)
            self.state.last_error = None
        try:
            descriptors = self.source_registry.scan_in_order()
            devices = []
            for descriptor in descriptors:
                candidate = descriptor.candidate
                capabilities = candidate.capabilities
                module = self.source_registry.existing(candidate.provider_id)
                profile = getattr(module, "arm_profile", None) if module is not None else None
                devices.append({
                    "name": candidate.name,
                    "address": candidate.address,
                    "source_type": candidate.public_source_type,
                    "capabilities": asdict(capabilities),
                    "provides_joint_targets": capabilities.provides_joint_targets,
                    "provides_pose": capabilities.provides_pose,
                    "configures_arm_session": capabilities.configures_arm_session,
                    "arm_profile": None if profile is None else profile.as_dict(),
                })
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

    def connect_device(self, address: str, name: str = "Unknown", direct: bool = False) -> ConnectResult:
        """连接指定设备并发布新的输入会话世代（须先完成扫描）。

        Args:
            address: 设备地址（来自最近一次扫描，未扫描过则拒绝）
            name: 设备显示名，仅用于状态展示
            direct: 是否接管 Quest 直连待命链路（仅 Quest 支持）
        Returns:
            ConnectResult，含 outcome/message/retain_transport；失败一律以
            ``ConnectResult(FAILED, message, False)`` 返回并写入 ``last_error``
        """
        with self._lifecycle_lock:
            return self._connect_device_locked(address, name, direct)

    def _connect_device_locked(self, address: str, name: str, direct: bool = False) -> ConnectResult:
        """连接实现体（须已持有 ``_lifecycle_lock``）。"""
        with self._data_lock:
            if self.state.phase is SessionPhase.CLEANUP_FAILED:
                self.state.last_error = "遥操资源清理未完成，请先重试停止"
                return ConnectResult(
                    ConnectOutcome.FAILED,
                    self.state.last_error,
                    False,
                )

        if self.source_registry.cleanup_lease() is not None:
            if not self.stop_session(
                stop_mode="hold",
                reason="replace-connected-device",
            ):
                with self._data_lock:
                    self.state.last_error = "旧遥操连接清理失败，请重试停止后再连接"
                return ConnectResult(
                    ConnectOutcome.FAILED,
                    self.state.last_error,
                    False,
                )

        descriptor = self._resolve_device_descriptor(address, name)
        if descriptor is None:
            with self._data_lock:
                self._set_phase_locked(SessionPhase.IDLE)
                self.state.last_error = (
                    self.source_registry.resolution_error(address, name)
                    or "不支持的遥操设备"
                )
                error_msg = self.state.last_error
            return ConnectResult(ConnectOutcome.FAILED, error_msg, False)

        candidate = descriptor.candidate
        module = self.source_registry.get_or_create(candidate.provider_id)
        availability = module.availability()
        if not availability.available:
            with self._data_lock:
                self._set_phase_locked(SessionPhase.IDLE)
                self.state.last_error = availability.reason or "手柄遥操功能不可用"
                error_msg = self.state.last_error
            return ConnectResult(ConnectOutcome.FAILED, error_msg, False)

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

            result = (
                module.connect(candidate, lease, self, direct=True)
                if direct else module.connect(candidate, lease, self)
            )
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
                    self.state.last_error = (
                        "连接已失效"
                        if closed
                        else "连接已失效且遥操资源清理未完成，请重试停止"
                    )
                    error_msg = self.state.last_error
                return ConnectResult(ConnectOutcome.FAILED, error_msg, False)
            if result.outcome is ConnectOutcome.READY:
                return result
            if result.outcome is ConnectOutcome.PENDING and result.retain_transport:
                return result

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
            return ConnectResult(ConnectOutcome.FAILED, result.message, False)
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
                error_msg = self.state.last_error
            return ConnectResult(ConnectOutcome.FAILED, error_msg, False)

    def get_last_error(self) -> str:
        """获取最后一次操作的错误信息"""
        with self._data_lock:
            return self.state.last_error

    def init_arm(self, master_ip: str, mode: str = "single", slave_ip: Optional[str] = None, master_side: str = HAND_RIGHT) -> bool:
        """初始化机械臂会话（须已有就绪输入源，且输入源允许控制机械臂）。

        Args:
            master_ip: 主臂管理 IP
            mode: 拓扑，"single" 或 "dual"
            slave_ip: 从臂管理 IP（dual 模式使用）
            master_side: 主手手别
        Returns:
            bool：会话建立且初始化成功为 True；失败原因写入 ``last_error``
        """
        with self._lifecycle_lock:
            return self._init_arm_locked(
                master_ip,
                mode=mode,
                slave_ip=slave_ip,
                master_side=master_side,
            )

    def _init_arm_locked(self, master_ip: str, mode: str = "single", slave_ip: Optional[str] = None, master_side: str = HAND_RIGHT) -> bool:
        """初始化实现体（须已持有 ``_lifecycle_lock``）。"""
        if not self.available:
            logging.warning("[JoystickController] Controller not available")
            return False

        with self._data_lock:
            if self.session is not None:
                self.state.last_error = "机械臂会话已存在，请先停止后重新连接"
                return False
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
            self._motion_cancel.clear()

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

            bindings = self._arm_binding_builder(
                self.state.mode,
                master_ip,
                self.state.master_side,
                slave_ip,
            )
            profile = active_arm_profile(self)
            if profile is not None:
                if profile.require_same_board_dual and (
                    normalized_mode != "dual" or master_ip != slave_ip
                ):
                    raise ValueError("当前输入源需要同板双臂模式")
                if profile.modes:
                    requested = (
                        "d3_dual" if normalized_mode == "dual" and master_ip == slave_ip
                        else "a3_dual" if normalized_mode == "dual"
                        else "a3_single"
                    )
                    if requested not in profile.modes:
                        raise ValueError("当前输入源不支持该遥操拓扑")
            apply_profile_device_indexes(bindings, profile)
            session = BaseArmSession(
                self,
                bindings,
                self._build_command_controller,
                self._end_effector_factory,
                waist_factory=self._create_waist_runtime,
                arm_angle_factory=self._create_arm_angle_runtimes,
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
            self.session = session
            success = session.initialize()
            if not success:
                if session.has_resources:
                    self.session = session
                    self.controller = session.get_primary_controller()
                else:
                    self.session = None
                with self._data_lock:
                    self.state.arm_initialized = False
                    self._set_phase_locked(
                        SessionPhase.CLEANUP_FAILED
                        if session.has_resources
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
            if session is not None and not session.has_resources:
                self.session = None
            if session is not None and session.has_resources:
                self.session = session
                self.controller = session.get_primary_controller()
            error_msg = f"遥操配置错误：{e}"
            logging.error("[JoystickConfig] %s", error_msg)
            with self._data_lock:
                self.state.arm_initialized = False
                self._set_phase_locked(
                    SessionPhase.CLEANUP_FAILED
                    if session is not None and session.has_resources
                    else self._stable_phase_locked()
                )
                self.state.last_error = error_msg
            return False
        except Exception as e:
            if session is not None and not session.has_resources:
                self.session = None
            if session is not None and session.has_resources:
                self.session = session
                self.controller = session.get_primary_controller()
            error_msg = f"机械臂初始化异常：{str(e)}"
            logging.error(error_msg)
            logging.exception(e)
            with self._data_lock:
                self.state.arm_initialized = False
                self._set_phase_locked(
                    SessionPhase.CLEANUP_FAILED
                    if session is not None and session.has_resources
                    else self._stable_phase_locked()
                )
                self.state.last_error = error_msg
            return False

    def set_calibration(self, value: int) -> bool:
        """旧四档标定：value 为 0-3，映射为 yaw 后写入。

        Args:
            value: 旧四档标定值 0-3
        Returns:
            bool：映射后的 yaw 写入成功为 True；映射越界或写入异常返回 False
        """
        try:
            return self.set_yaw(legacy_mode_to_yaw(value))
        except Exception as e:
            logging.error(f"[JoystickController] Invalid calibration value: {e}")
            return False

    def set_yaw(self, value: int) -> bool:
        """设置水平 yaw 标定角度，并在真的变化时发布新的控制参数 epoch。

        Args:
            value: yaw 角度，-180..180 的整数（非整数/非有限值/超范围都拒绝）
        Returns:
            bool：写入成功为 True；非法值记日志后返回 False
        """
        try:
            if isinstance(value, bool) or not isinstance(value, numbers.Real):
                raise ValueError("calibration angle must be numeric")
            if not math.isfinite(float(value)) or float(value) != int(value):
                raise ValueError("calibration angle must be an integer")
            value = int(value)
            if not -180 <= value <= 180:
                raise ValueError("calibration angle must be between -180 and 180")
            with self._data_lock:
                changed = self.state.calibration != value
                self.state.calibration = value
                if changed:
                    self._publish_control_params_locked()
                    self.state.data_sequence += 1
            if self.direct_manager is not None:
                self.direct_manager.update_parameter("yaw", value)
            return True
        except Exception as e:
            logging.error(f"[JoystickController] Invalid yaw value: {e}")
            return False

    def set_scale(self, value: float) -> bool:
        """设置灵敏度 scale，并在真的变化时发布新的控制参数 epoch。

        Args:
            value: 0.1..2.0 的有限实数
        Returns:
            bool：写入成功为 True；非法值记日志后返回 False
        """
        try:
            if (
                not isinstance(value, numbers.Real)
                or isinstance(value, bool)
            ):
                logging.error("[JoystickController] Invalid scale value: %r", value)
                return False

            value = float(value)
            if not math.isfinite(value) or not 0.1 <= value <= 2.0:
                logging.error("[JoystickController] Invalid scale value: %r", value)
                return False

            with self._data_lock:
                self.scale = value
                changed = self.state.scale != value
                self.state.scale = value
                if changed:
                    self._publish_control_params_locked()
                    self.state.data_sequence += 1
            if self.direct_manager is not None:
                self.direct_manager.update_parameter("scale", value)
            return True
        except Exception as e:
            logging.exception(e)
            return False

    def get_move_speed_level(self) -> Dict[str, Any]:
        """读取当前遥操 move_joint 速度档位。

        Returns:
            dict：``{"level": 档位名, "speed": SDK speed 百分比}``
        """
        with self._data_lock:
            level = self.move_speed_level
        return {"level": level, "speed": MOVE_SPEED_LEVELS[level]}

    def set_move_speed_level(self, level: str) -> bool:
        """设置遥操 move_joint 速度档位，立即作用于当前会话的各臂。

        Args:
            level: ``MOVE_SPEED_LEVELS`` 中的档位名（low/medium/high）
        Returns:
            bool：写入成功为 True；非法档位记日志后返回 False
        """
        if level not in MOVE_SPEED_LEVELS:
            logging.error("[JoystickController] Invalid move speed level: %r", level)
            return False
        speed = MOVE_SPEED_LEVELS[level]
        with self._data_lock:
            self.move_speed_level = level
            if self.session:
                for runtime in self.session.arms:
                    runtime.controller.move_speed = speed
            elif self.controller:
                self.controller.move_speed = speed
        return True

    def set_view_side(self, hand: str) -> bool:
        """设置当前查看侧（视图侧位姿/按钮直接由对应手信箱投影）。"""
        try:
            with self._data_lock:
                self.state.view_side = normalize_hand(hand, self.state.master_side or HAND_RIGHT)
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
        """设置会话配置，用于连接前同步 Quest UI 选择。

        Args:
            mode: 拓扑，"single" 或 "dual"；single 会清掉从臂 IP
            master_side: 主手手别
            view_side: 视图侧手别；缺省时按新 master_side 归一化
            slave_ip: 从臂 IP，仅 dual 模式生效
        Returns:
            bool：写入成功为 True；非法 mode 等异常记日志后返回 False
        """
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

                self.state.data_sequence += 1
            return True
        except Exception as e:
            logging.exception(e)
            return False

    def get_observation_sequence(self) -> int:
        """返回会话变化与输入发布计数之和（单调，供 SSE 变更检测）。"""
        with self._data_lock:
            return self.state.data_sequence + self._input_publication_sequence

    def get_observation_snapshot(self) -> ObservationSnapshot:
        """抓取一份世代一致的 Host 观测快照，见 ``build_observation_snapshot``。"""
        return build_observation_snapshot(self)

    def get_status(self) -> dict:
        """获取当前状态（观测快照的 HTTP 投影）。"""
        return project_http_status(self.get_observation_snapshot())

    def confirm_init_position(self, *args, **kwargs) -> dict:
        """确认初始化位置，见 ``ArmMaintenance.confirm_init_position``。"""
        return self.maintenance.confirm_init_position(*args, **kwargs)

    def get_init_position(self) -> dict:
        """读取当前生效的初始化位置，见 ``ArmMaintenance.get_init_position``。"""
        return self.maintenance.get_init_position()

    def get_current_joints(self, *args, **kwargs) -> dict:
        """临时读取当前关节值，见 ``ArmMaintenance.get_current_joints``。"""
        return self.maintenance.get_current_joints(*args, **kwargs)

    def stop_session(self, stop_mode: str = "hold", reason: str = "unknown") -> bool:
        """停止遥操会话；仅显式 home 模式允许机械臂回零。

        Args:
            stop_mode: "hold" 或 "home"，非法值降级为 hold
            reason: 停止原因（透传到清理上下文与日志）
        Returns:
            bool：清理全部成功为 True；失败进入 ``CLEANUP_FAILED`` 并返回 False
        """
        if not self.available:
            return False

        # 拆卸与 provider 建立过程共用 lifecycle 锁串行化，避免清理检查完会话之后
        # 才发布传输所有权。
        with self._lifecycle_lock:
            return self._stop_session_locked(stop_mode=stop_mode, reason=reason)

    def _build_cleanup_context_locked(
        self,
        mode: CleanupMode,
        reason: str,
        home_allowed: bool = False,
    ) -> CleanupContext:
        """构造一次清理上下文；实际 owner 由清理步骤在生命周期锁内读取。"""
        return CleanupContext(
            mode=mode,
            reason=reason,
            home_allowed=home_allowed,
        )

    def _run_cleanup_locked(self, context: Optional[CleanupContext] = None, *, force_hold=False, reason=None):
        """执行（或重试）清理；成功则重置状态，失败则进入 CLEANUP_FAILED 并保留 owner。"""
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
        """先失效输入，再重试每一个仍被持有的清理 owner。"""

        with self._data_lock:
            self._source_reconnect_deadline = None

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
            succeeded = self._run_cleanup_locked(
                force_hold=bool(normalized_mode == "hold" or self._motion_cancel.is_set()),
                reason=reason,
            ).success
            if not succeeded:
                # 手动重试不受自动重试窗口限制；失败后以当前会话序列
                # 重新入队，自动重试以全新窗口继续。
                self.request_stop_session(reason=reason)
            return succeeded

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

        # 先冻结 home 权限，再拒绝所有回调与在途目标，最后才等待 arm/SDK/BLE/ADB/
        # 线程清理。
        self._mark_input_disconnected(reason)
        if home_allowed:
            self._motion_cancel.clear()
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
        """停止会话并独立退役每一个 owner；任一步失败即返回 ``False``。

        Args:
            reason: 停机原因（透传到清理上下文与日志）
        Returns:
            bool：controller 不可用时视为已停机，返回 True
        """
        if not self.available:
            return True
        if not self.stop_session(stop_mode="hold", reason=reason):
            return False
        result = self._session_cleanup.shutdown(reason)
        if not result.success:
            return False
        return True

    def get_cached_init_position(self, model: str, mode: str = "single", hand: Optional[str] = None):
        """读取进程内的初始化位置覆盖值，见 ``ArmMaintenance.get_cached_init_position``。"""
        return self.maintenance.get_cached_init_position(model, mode, hand)
