"""遥操 Host 会话状态机（joystick 全链路的核心编排层）。

用户操作排他（``user_operation``）、输入源 binding 与发布、双手同批信箱、观测镜像、
清理编排与观测快照均在本模块；锁序 ``_lifecycle_lock`` > ``_data_lock`` >
``pose_state_lock``，``_policy_lock`` 是叶子 gate。
"""

import asyncio
import logging
import math
import numbers
import threading
import time
from concurrent.futures import TimeoutError as FuturesTimeoutError
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from typing import Any, Dict, Optional, List, Tuple

from ..runtime.config import JoystickConfigError
from ..runtime.dependencies import (
    ARM_SDK_AVAILABLE, CORE_DEPENDENCIES_AVAILABLE,
    DEPENDENCIES_AVAILABLE, Carm,
)
from ..runtime.context import JOYSTICK_CONFIG
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
from ..sources.adapters import validate_raw_input_batch
from ..sources.registry import (
    ActiveSourceBinding,
    BindingPhase,
    SourceLeaseAllocator,
    SourceRegistry,
    SourceScanError,
)
from ..core.command_controller import CommandController, ControlParams
from ..core.mailbox import (
    HandInputFrame,
    InputPublication,
    JointInputFrame,
    JointPublication,
    LatestValueCell,
    PublicationSignal,
    hand_cells,
    reengage_payload,
)
from ..core.pose_math import _is_valid_joint_position, legacy_mode_to_yaw, yaw_to_legacy_mode
from ..end_effectors import EndEffectorFactory, resolve_end_effector_factory
from quest.module import QuestInputModule
from ..core.models import (
    button_to_dict,
    pose_to_dict,
)
from .arm import BaseArmSession, build_arm_bindings, LOCAL_ARM_CONTROL_HOST
from .cleanup import CleanupContext, CleanupMode, SessionCleanup
from .state import JoystickState, SessionPhase
from .control_transaction import _INPUT_FRESH_S
from .observation import (
    ObservationArm,
    ObservationDevice,
    ObservationHand,
    ObservationSnapshot,
    ObservationStatus,
    ObservationTransport,
    project_http_status,
)


_CLEANUP_THREAD_TIMEOUT = 2.0
_WORKER_CHECK_INTERVAL = 1.0
# 自动重试上界：仅限制"失败后是否再次入队"，不限制单次外部拆卸调用的退出时间。
_AUTO_RETRY_WINDOW_S = 10.0
_CLEANUP_WORKER_NAME = "JoystickCleanupWorker"


def _active_arm_profile(owner):
    """当前激活 provider 的 arm_profile（无激活模块时为 ``None``）。"""
    registry = getattr(owner, "source_registry", None)
    module = None if registry is None else registry.active_module()
    return None if module is None else getattr(module, "arm_profile", None)


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
        if not DEPENDENCIES_AVAILABLE:
            logging.warning("[JoystickController] Dependencies not available")
            self.available = False
            return

        self.available = True
        self.scale = scale
        self.joystick_config = joystick_config_snapshot or JOYSTICK_CONFIG

        self.controller = None
        self.init_position_controller = None
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
        # policy gate 叶子锁：持有期间禁止获取任何其他锁（含 _data_lock）。唯一
        # 豁免：gate 块内可经 ``module.callback_matches`` 取模块 lifecycle_lock。
        # 前提：任何模块不得在持有自身 lifecycle_lock 时回调 Host。
        self._policy_lock = threading.RLock()
        # 双手 publication 单槽信箱：一次批次只 store 一次，读者一次 load 即得双手同批。
        self._input_publication = LatestValueCell()
        self._joint_publication = LatestValueCell()
        # 观测镜像：只在 ``_data_lock`` 临界区内写、在 ``get_observation_snapshot``
        # 的 data 锁内读；任一 ``data_sequence`` 取值只映射一份观测内容。
        self._observed_input_publication = LatestValueCell()
        self._observed_joint_publication = LatestValueCell()
        # 发布唤醒信号：输入/关节帧 store 之后（gate 外）notify，按输入节拍运行的
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
        """把所有 provider 的 policy 状态归零（在 gate 内）。"""
        for provider_id in self.source_registry.provider_ids:
            module = self.source_registry.existing(provider_id)
            if module is not None:
                # policy 状态与 provider 发布路径共用同一个叶子 gate。
                with self._policy_lock:
                    self._call_optional_interface(module, "reset_policy")

    def _is_hand_bound_active_locked(self, hand: str, controller=None) -> bool:
        """某只手是否被已连接的 arm runtime 绑定为当前控制手（可按 controller 限定）。"""
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
        """清空双手 publication、观测镜像与每手 reengage 信箱（须在 data 锁内与序号同临界区）。"""
        self._input_publication.clear()
        self._joint_publication.clear()
        self._observed_input_publication.clear()
        self._observed_joint_publication.clear()
        for hand in (HAND_LEFT, HAND_RIGHT):
            self._reengage_cells[hand].clear()

    def _withdraw_published_frames(self, publication, joint: bool = False) -> None:
        """撤销一次已提交但随即失效的双手发布（仅当槽位仍是本次 publication）。

        观测镜像不参与撤销：被撤销的 publication 从未进入镜像。
        """
        cell = self._joint_publication if joint else self._input_publication
        if cell.load() is publication:
            cell.clear()

    def _clear_input_display_cells(self) -> None:
        """
        非终止断开：只清位姿/按钮显示槽位，保留"可用于控制的数据"。

        读—改—写须在 ``_policy_lock`` 内完成，避免覆盖发布线程新存的 publication；
        观测镜像的显示清空在 data 锁内对已提交内容完成。
        """
        with self._policy_lock:
            publication = self._input_publication.load()
            if publication is not None:
                cleared = publication.without_display()
                if cleared is not publication:
                    self._input_publication.store(cleared)
        observed = self._observed_input_publication.load()
        if observed is not None:
            self._observed_input_publication.store(observed.without_display())

    def publish_source_controller(self, controller: CommandController) -> bool:
        """把 controller 绑定到当前输入源 binding（controller 绑定族）。

        Returns:
            bool：无有效 binding/模块（无需绑定）返回 True；输入未连接返回 False；
            其余情况完成绑定后返回 True
        """
        with self._data_lock:
            module = self.source_registry.active_module()
            binding = self.active_source_binding
            if module is None or binding is None or binding.phase is BindingPhase.CLOSED:
                return True
            if not self.state.ble_connected:
                return False
            # controller 绑定族是可选接口：仅位姿/重连宽限模块参与，
            # Host 自身的会话身份与状态推进不受模块是否实现影响。
            previous_controller = None
            if self._optional_interface_enabled(binding.capabilities):
                # policy 状态（controller 绑定、pin、token）只经 gate 变更。
                with self._policy_lock:
                    previous_controller = module.bind_controller(
                        controller.hand_binding,
                        controller,
                        binding.session_id,
                        self._control_params.epoch,
                    )
            self.state.data_sequence += 1
            if previous_controller is not None and previous_controller is not controller:
                previous_controller.invalidate_pose_mapping()
            return True

    def release_source_controller(self, controller: CommandController) -> None:
        """解除 controller 绑定；释放成功（或最小契约模块路径）都会失效位姿映射。"""
        with self._data_lock:
            module = self.source_registry.active_module()
            if module is None:
                return
            binding = self.active_source_binding
            capabilities = binding.capabilities if binding is not None else None
            if not self._optional_interface_enabled(capabilities):
                # 最小契约模块不参与 controller 绑定族；Host 仍失效映射。
                self.state.data_sequence += 1
                controller.invalidate_pose_mapping()
                return
            with self._policy_lock:
                released = bool(
                    module.release_controller(
                        controller.hand_binding,
                        controller,
                        self._control_params.epoch,
                    )
                )
            if not released:
                return
            self.state.data_sequence += 1
            controller.invalidate_pose_mapping()

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

    def publish_joint_input(self, batch, provider_identity) -> None:
        """发布一只手的关节目标帧，提交规则同 ``publish_input``（gate 内冻结、data 锁内激活与推序号）。

        Raises:
            TypeError: 传入的不是 ``JointInputBatch``
        """
        if not isinstance(batch, JointInputBatch):
            raise TypeError("expected JointInputBatch")
        # 提交规则（1.1）：policy 视角（此处为 provider stub）与冻结帧的发布在同一
        # 个 gate 临界区内推进；gate 是叶子锁，持有期间不取任何其他锁。
        with self._policy_lock:
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            if (binding is None or module is None or binding.phase is BindingPhase.CLOSED
                    or binding.lease != batch.lease
                    or not binding.capabilities.provides_joint_targets
                    or not module.callback_matches(batch.lease, provider_identity)):
                return
            received = time.monotonic()
            # 双手在**同一个** publication 里一次落地：读者一次 load 即得双手同批。
            publication = JointPublication(
                left=JointInputFrame(
                    lease=batch.lease,
                    batch_sequence=batch.batch_sequence,
                    target=batch.left,
                    received_monotonic=received,
                ),
                right=JointInputFrame(
                    lease=batch.lease,
                    batch_sequence=batch.batch_sequence,
                    target=batch.right,
                    received_monotonic=received,
                ),
                batch_sequence=batch.batch_sequence,
                lease=batch.lease,
            )
            self._joint_publication.store(publication)
        # gate 外唤醒控制线程（信号锁是叶子锁，不得在 gate 内获取）。
        self._publication_signal.notify()
        # 先 store 后 bump：SSE 以 data_sequence 做变更检测，必须在帧可见后推进。
        with self._data_lock:
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            if (binding is None or module is None or binding.phase is BindingPhase.CLOSED
                    or binding.lease != batch.lease):
                # 提交点之后会话才失效：撤销本次发布（写入即失效）。
                self._withdraw_published_frames(publication, joint=True)
                return
            if binding.phase is BindingPhase.PROVISIONAL:
                self._activate_binding_locked(batch.lease.provider_id, batch.lease.input_generation)
            self.state.ble_connected = True
            self.state.transport_state = "ready"
            # 成对提交：本次冻结帧在此与序号一起对观测可见（gate 预提交只对控制路径
            # 即时可见，见 get_observation_snapshot 的观测镜像说明）。
            self._observed_joint_publication.store(publication)
            self.state.data_sequence += 1

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

        非终止断开只做"暂停"：binding 未关闭、会话身份不变，控制失效由暂停
        controller 的纪元发布承担；终止断开与重连宽限到期才进入
        ``_mark_disconnected_and_request_cleanup``。
        """
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
                    # 非终止断开：仍属同一会话（binding 未关闭），因此没有任何
                    # 会话身份变化；控制失效由暂停控制器的纪元发布承担。
                    suspended_controllers.update(self._active_pose_controllers())
                    if self._optional_interface_enabled(binding.capabilities):
                        with self._policy_lock:
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
                    self._clear_input_display_cells()
                    self.state.device_status = {}
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
        if module.current_lease() is not None:
            raise RuntimeError("previous input transport must be closed before activation")
        self._call_optional_interface(
            module, "cancel_reconnect_timer", clear_deadline=True,
            capabilities=capabilities,
        )
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
            for controller in self._active_pose_controllers():
                controller.invalidate_pose_mapping()
            return session_id

    def _active_pose_controllers(self) -> List[CommandController]:
        """当前参与位姿映射的 controller 集合（arm 会话优先，否则单体 controller）。"""
        if self.session:
            return [runtime.controller for runtime in self.session.arms]
        return [self.controller] if self.controller else []

    def _request_source_mapping_change_locked(self, reason: str):
        """把映射变化通知给 provider（最小契约模块返回空元组）；在 gate 内计算活跃手。"""
        module = self.source_registry.active_module()
        if module is None:
            return ()
        binding = self.active_source_binding
        capabilities = binding.capabilities if binding is not None else None
        if not self._optional_interface_enabled(capabilities):
            # 最小契约模块不参与 controller 绑定族与映射通知。
            return ()
        with self._policy_lock:
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
                self.active_source_binding.session_id,
                self._control_params.epoch,
            )

    def _schedule_source_reconnect_cleanup(self, lease: SourceLease) -> None:
        """启动 provider 重连宽限：注入恢复判定回调与到期回调。"""
        module = self.source_registry.active_module()
        if module is None:
            return

        def evaluate_recovery() -> Tuple[bool, bool]:
            """宽限到期时判定：会话是否已过期、传输是否已自愈。"""
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

        module.begin_reconnect_grace(
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
        """重连宽限到期：先失效输入，再按当前会话世代请求清理。"""
        result = self._mark_input_disconnected(
            "input-reconnect-deadline",
            expected_lease=lease,
        )
        if result is None:
            return
        with self._data_lock:
            cleanup_session_id = self._session_sequence
        # 已有 worker 会阻塞等待 lifecycle 所有权，直到在飞的初始化返回；
        # 因此这里不需要轮询定时器。
        self.request_stop_session(
            reason="input-reconnect-deadline",
            expected_session_id=cleanup_session_id,
            expected_lease=lease,
        )

    def _cancel_source_reconnect_timers(
        self,
        clear_deadline: bool = False,
    ) -> None:
        """取消所有 provider 的重连定时器（可选一并清掉 deadline）。"""
        for provider_id in self.source_registry.provider_ids:
            module = self.source_registry.existing(provider_id)
            if module is not None:
                self._call_optional_interface(
                    module, "cancel_reconnect_timer", clear_deadline=clear_deadline,
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
            module = self.source_registry.active_module()
            controllers = set(self._active_pose_controllers())
            owned = ()
            if module is not None and self._optional_interface_enabled(
                binding.capabilities if binding is not None else None
            ):
                with self._policy_lock:
                    owned = module.owned_controllers() or ()
            controllers.update(owned)
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
        for controller in controllers:
            controller.invalidate_pose_mapping()
        self._cancel_source_reconnect_timers(clear_deadline=True)
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

    def _build_command_controller(self, role: str, hand_binding: str):
        """按 role/hand_binding 构造 CommandController 并挂上运动取消事件。"""
        # 控制器对象本身就是凭据：一个 arm runtime 的控制器每会话只发布一次，
        # 重新绑定必然伴随新会话与新 binding session_id。
        controller = CommandController(
            self.scale, hand_binding=hand_binding, arm_role=role,
        )
        controller.motion_cancel_event = self._motion_cancel
        return controller

    def configure_arm_initialization(self, runtime):
        """按当前输入源的 arm profile 固化 runtime 的初始化关节。

        Raises:
            ValueError: 型号不在 profile 允许集内
            ValueError: profile 要求的关节自由度与型号不匹配
        """
        controller = runtime.controller
        model = controller.arm_model
        profile = _active_arm_profile(self)
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
        if self.state.mode == "single":
            controller.load_cached_init_position(self.get_cached_init_position(model))
        controller._active_init_joint()

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
                triggered = self._cleanup_event.wait(
                    timeout=_WORKER_CHECK_INTERVAL
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
                                    self._session_sequence
                                    != request.session_id
                                    or self.active_input_source is not None
                                    or (
                                        # 只有"已被另一个 lease 接管"才算 stale；
                                        # provider 步已释放时仍需重试未完成的 Arm 步。
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
                                CleanupMode.HOLD,
                                request.reason,
                            )
                            result = self._run_cleanup_locked(context)
                            if not result.success:
                                if self._requeue_failed_stop_request(request):
                                    time.sleep(_WORKER_CHECK_INTERVAL)
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
                        if self._requeue_failed_stop_request(request):
                            time.sleep(_WORKER_CHECK_INTERVAL)
                            self._cleanup_event.set()
                            break
            except Exception:
                logging.exception("[CleanupWorker] Worker error")
                time.sleep(0.1)

    def _reset_state(self) -> None:
        """重置为 IDLE 会话：关 binding、清信箱、清遥测与臂状态，并重置 provider policy。"""
        self._cancel_source_reconnect_timers(clear_deadline=True)
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
        """校验、适配并提交一次 provider 中立的输入事务。

        提交规则：gate 前快照 ``bound_hands``；policy 推进与冻结帧发布在同一个
        ``_policy_lock`` 临界区；之后在 data 锁内更新标志与 ``data_sequence``。

        Args:
            batch: provider 构造的原始输入批次
            provider_identity: provider 侧身份，用于 ``callback_matches`` 复核

        Raises:
            ValueError: 批次的 provider identity 缺失。
        """
        batch = validate_raw_input_batch(batch)
        if provider_identity is None:
            raise ValueError("provider identity is required for input batches")
        with self._data_lock:
            module = self.source_registry.active_module()
            binding = self.active_source_binding
            if (
                binding is None
                or module is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != batch.lease
                or not module.callback_matches(batch.lease, provider_identity)
            ):
                return
            bound_hands = {
                update.hand
                for update in batch.hands
                if (
                    (controller := module.controller_for_hand(update.hand))
                    is not None
                    and self._is_hand_bound_active_locked(update.hand, controller)
                )
            }
            # 会话标志（binding 相位/就绪）在 gate 之前推进：帧发布时控制线程
            # 看到的会话状态已经就绪，不需要等待一次 data_sequence bump。
            if binding.phase is BindingPhase.PROVISIONAL:
                self._activate_binding_locked(
                    batch.lease.provider_id,
                    batch.lease.input_generation,
                )
            elif not binding.input_ready:
                self._set_binding_input_ready_locked(
                    batch.lease.provider_id,
                    batch.lease.input_generation,
                    True,
                )
        invalidated_controllers = set()
        rejected_controllers = set()
        publication = None
        with self._policy_lock:
            binding = self.active_source_binding
            module = self.source_registry.active_module()
            if (
                binding is None
                or module is None
                or binding.phase is BindingPhase.CLOSED
                or binding.lease != batch.lease
                # 豁免：gate 内允许获取模块 lifecycle_lock（见 _policy_lock 注释）。
                or not module.callback_matches(batch.lease, provider_identity)
            ):
                return
            module.validate_input_batch(batch)
            decisions = module.process_input_batch(
                batch,
                bound_hands,
                binding.session_id,
                self._control_params.epoch,
            )
            # 双手同一批构建后一次 store：单手更新批次里另一手携带同一帧对象，
            # 读者不会看到混合批次。
            previous_publication = self._input_publication.load()
            frames = {
                hand: previous_publication.frame(hand)
                if previous_publication is not None
                else None
                for hand in (HAND_LEFT, HAND_RIGHT)
            }
            for decision in decisions:
                hand = decision.hand
                update = next(update for update in batch.hands if update.hand == hand)
                # 失效事件须在同一 gate 临界区内、store 之前发布，否则旧 attempt
                # 会在"已 store、新纪元未发布"的窗口通过 admit。
                if decision.controller is not None:
                    if decision.invalidate_controller:
                        decision.controller.publish_epoch()
                        invalidated_controllers.add(decision.controller)
                    elif decision.reject_controller:
                        decision.controller.publish_target_rejection()
                        rejected_controllers.add(decision.controller)
                previous = frames[hand]
                if previous is None:
                    pose, button, control_sample, sequence = None, None, None, 0
                else:
                    pose, button = previous.pose, previous.button
                    control_sample, sequence = previous.control_sample, previous.input_sequence
                # 独立接受：位姿无效帧不阻塞按钮槽位刷新，反之亦然。
                if decision.accept_pose:
                    pose = update.pose
                if decision.accept_button:
                    button = update.button
                sequence += 1
                if decision.accept_pose and decision.accept_button:
                    control_sample = UnifiedHandSample(
                        hand=hand, pose=update.pose, button=update.button,
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
                    hand=hand, pose=pose, button=button,
                    control_sample=control_sample, input_sequence=sequence,
                    lease=batch.lease, timestamp_monotonic=time.monotonic(),
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
                )
                self._input_publication.store(publication)
        # gate 外唤醒控制线程（信号锁是叶子锁，不得在 gate 内获取）。
        if publication is not None:
            self._publication_signal.notify()
        # ``module.status`` 在 gate 外取：只读快照，只需模块自己的 lifecycle_lock，
        # 不参与冻结帧提交。它供下面的 data 块使用，所以放在 data 块之前、gate 之后。
        status = module.status(batch.lease)
        with self._data_lock:
            binding = self.active_source_binding
            if (
                binding is None
                or binding.lease != batch.lease
                or binding.phase is BindingPhase.CLOSED
            ):
                # 提交点之后会话才失效：撤销本次发布（“写入即失效”），
                # 不让迟到发布者把已死会话的帧留在信箱里。
                if publication is not None:
                    self._withdraw_published_frames(publication)
                return
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
            # 成对提交：本次 gate 预提交的 publication 在此与序号一起对观测可见
            # （``decisions`` 为空时不发布，镜像保持上一份已提交内容）。
            if publication is not None:
                self._observed_input_publication.store(publication)
            self.state.data_sequence += 1
            for controller in rejected_controllers - invalidated_controllers:
                controller.reject_pose_target()
            for controller in invalidated_controllers:
                # 双重发布：gate 内已发过纪元，这里再完整失效一次（偏保守，
                # 只多拒绝不多放行），避免把状态清理拆分复制到 gate 内。
                controller.invalidate_pose_mapping()

    def _close_source_lease(self, lease: SourceLease, reason: str) -> bool:
        """关闭一个 source lease（begin → close_connection → complete 三段，任一步失败即 False）。"""
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

    def connect_device(self, address: str, name: str = "Unknown") -> ConnectResult:
        """连接指定设备并发布新的输入会话世代（须先完成扫描）。

        Args:
            address: 设备地址（来自最近一次扫描，未扫描过则拒绝）
            name: 设备显示名，仅用于状态展示
        Returns:
            ConnectResult，含 outcome/message/retain_transport；失败一律以
            ``ConnectResult(FAILED, message, False)`` 返回并写入 ``last_error``
        """
        with self._lifecycle_lock:
            return self._connect_device_locked(address, name)

    def _connect_device_locked(self, address: str, name: str) -> ConnectResult:
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
            profile = _active_arm_profile(self)
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
                for arm_binding in bindings:
                    index = profile.device_index_by_hand.get(arm_binding["hand"])
                    if index is not None:
                        arm_binding["device_index"] = index
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
            self.session = session
            success = session.initialize()
            if not success:
                if session.arms:
                    self.session = session
                    self.controller = session.get_primary_controller()
                else:
                    self.session = None
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
            if session is not None and not session.arms:
                self.session = None
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
            if session is not None and not session.arms:
                self.session = None
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
                affected_controllers = []
                changed = self.state.calibration != value
                self.state.calibration = value
                if changed:
                    self._publish_control_params_locked()
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
                affected_controllers = []
                changed = self.state.scale != value
                self.state.scale = value
                if changed:
                    self._publish_control_params_locked()
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
        """返回用于 SSE 变更检测的 ``data_sequence``。"""
        with self._data_lock:
            return self.state.data_sequence

    def get_observation_snapshot(self) -> ObservationSnapshot:
        """抓取一份不可变、世代一致的 Host 观测快照（data 锁内读取并投影）。

        Returns:
            ObservationSnapshot；controller 不可用时返回 ``available=False`` 的空快照。
        """
        if not self.available:
            empty_hand = ObservationHand.from_values()
            return ObservationSnapshot(
                sequence=0, available=False,
                status=ObservationStatus(False, False, 1, -90, getattr(self, "scale", 0.4),
                                         "not_available", None, "single",
                                         False, HAND_RIGHT, HAND_RIGHT,
                                         capabilities=None, arm_profile=None),
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
            # 能力与拓扑限制由 Host 投影出去，Web 刷新后不需要按 provider 名字猜测。
            capabilities_payload = (
                asdict(binding.capabilities) if binding is not None else None
            )
            arm_profile = _active_arm_profile(self) if binding is not None else None
            arm_profile_payload = None if arm_profile is None else arm_profile.as_dict()
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
            # 输入/关节各读一次观测镜像：镜像与 ``data_sequence`` 在同一 data 锁
            # 临界区内成对提交，任一序号只映射一份观测内容；gate 预提交只对控制
            # 路径即时可见，观测推迟到 data 阶段提交。
            input_publication = self._observed_input_publication.load()
            joint_publication = self._observed_joint_publication.load()
            for hand in (HAND_LEFT, HAND_RIGHT):
                input_frame = (
                    None if input_publication is None
                    else input_publication.frame(hand)
                )
                joint_frame = (
                    None if joint_publication is None
                    else joint_publication.frame(hand)
                )
                joint = None
                if (
                    joint_frame is not None and binding_active
                    and binding is not None
                    and binding.capabilities.provides_joint_targets
                    and joint_frame.target is not None
                ):
                    target = joint_frame.target
                    joint = {
                        "switch_enabled": target.switch_enabled,
                        "joint_positions": list(target.joint_positions),
                        "trigger": target.trigger,
                        "axis_x": target.axis_x,
                        "axis_y": target.axis_y,
                        "buttons": list(target.buttons),
                        "fresh": time.monotonic() - joint_frame.received_monotonic <= _INPUT_FRESH_S,
                        "sequence": joint_frame.batch_sequence,
                        "reengage": reengage_payload(self._reengage_cells[hand].load()),
                    }
                hands[hand] = ObservationHand.from_values(
                    joint=joint,
                    pose=(
                        pose_to_dict(input_frame.pose)
                        if input_frame is not None and input_frame.pose
                        else None
                    ),
                    button=(
                        button_to_dict(input_frame.button)
                        if input_frame is not None and input_frame.button
                        else None
                    ),
                )
            view_side = normalize_hand(self.state.view_side, HAND_RIGHT)
            return ObservationSnapshot(
                sequence=self.state.data_sequence, available=True,
                pose=hands[view_side].pose,
                button=hands[view_side].button,
                status=ObservationStatus(stream_connected, self.state.arm_initialized,
                    yaw_to_legacy_mode(self.state.calibration), self.state.calibration,
                    self.state.scale, self.state.status,
                    self.state.source_type,
                    self.state.mode, self.state.is_quest_device, self.state.master_side,
                    view_side,
                    capabilities=capabilities_payload, arm_profile=arm_profile_payload,
                    device_status=tuple(self.state.device_status.values())),
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
        """获取当前状态（观测快照的 HTTP 投影）。"""
        return project_http_status(self.get_observation_snapshot())

    def _build_init_position_controller(self) -> CommandController:
        """旧接口确认型号前不展示假定型号的初始化位置。"""
        controller = CommandController(self.scale, arm_role="master")
        controller.init_joint_ = None
        return controller

    def confirm_init_position(self, ip: str) -> bool:
        """通过本机控制端点确认初始化位置；ip 仅保留接口兼容。

        Args:
            ip: 管理 IP，仅用于日志（实际连本机 ``LOCAL_ARM_CONTROL_HOST``）
        Returns:
            bool：确认成功并缓存初始化位置为 True；异常/失败均返回 False
        """
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
                self.set_cached_init_position(self.init_position_controller.arm_model, position)
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

    def set_cached_init_position(self, model: str, position: List[float]) -> None:
        """设置旧接口使用的进程内初始化位置。"""
        if not _is_valid_joint_position(position, 6):
            logging.error("[JoystickController] Position must contain exactly 6 finite numbers: %s", position)
            return

        try:
            with self._data_lock:
                self.cached_init_positions[model] = list(position)
                logging.info("[JoystickController] Cached init position for %s: %s", model, position)
        except Exception as e:
            logging.error(f"[JoystickController] Error caching init position: {e}")

    def get_cached_init_position(self, model: str) -> Optional[List[float]]:
        """获取旧接口保存的进程内初始化位置。"""
        try:
            with self._data_lock:
                cached_position = self.cached_init_positions.get(model)
                if _is_valid_joint_position(cached_position, 6):
                    logging.info("[JoystickController] Retrieved cached init position for %s: %s", model, cached_position)
                    return cached_position.copy()
                if cached_position:
                    logging.warning("[JoystickController] Cached position validation failed. Position: %s", cached_position)
                return None
        except Exception as e:
            logging.error(f"[JoystickController] Error retrieving cached init position: {e}")
            return None
