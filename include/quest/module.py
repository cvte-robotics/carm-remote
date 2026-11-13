"""Quest 输入适配模块：拥有全部 ADB/TCP 资源与连接生命周期。

ADB 扫描/握手、reverse 转发与 TCP 接收器均由本模块创建与销毁，重连宽限由 Host 管理
（policy 只做帧语义、receiver 负责协议状态机与数值校验）。清理失败进入 ``_pending_cleanup``
重试，pending 非空时端口视为未释放。
"""

import logging
from dataclasses import dataclass, field
import threading
import time
from typing import Any, Callable, Dict, NamedTuple, Optional, Tuple

from carm_teleop_input_contracts import (
    ArmProfile,
    BaseInputSourceModule,
    ConnectOutcome,
    ConnectResult,
    DeviceCandidate,
    HAND_LEFT,
    HAND_RIGHT,
    ModuleStatus,
    SourceAvailability,
    SourceCapabilities,
    RawInputBatch,
    HandInputUpdate,
    HandIntentSample,
    BodyIntentSample,
    SourceLease,
    SourceEventSink,
    TransportEvent,
)

from carm_teleop_intent import LegacyHandIntentAdapter

from .adb import (
    get_adb_device_info,
    is_quest_adb_device,
    launch_quest_app,
    run_adb,
    scan_quest_devices,
)
from .constants import (
    QUEST_EXIT_TIMEOUT,
    QUEST_FIRST_FRAME_TIMEOUT,
    QUEST_PACKAGE,
    QUEST_TCP_HOST,
    QUEST_TCP_PORT,
)
from .device_status import QuestDeviceStatusPoller
from .parser import parse_waist_control, parse_quest_frame
from .policy import QuestInputPolicy
from .receiver import QuestTcpReceiver


FrameCallback = Callable[[RawInputBatch, object], None]
DisconnectCallback = Callable[[str, int, int, QuestTcpReceiver], None]
StatusCallback = Callable[[str, int, int, QuestTcpReceiver], None]


WAIST_CONTROL_STATS_WINDOW_S = 5.0
_ARM_ANGLE_UNAVAILABLE = {"status": "unavailable", "angle_deg": None, "target_deg": None, "error": None}


class WaistControlStats:
    """按固定窗口记录腰控制方式变化次数，供现场诊断。"""

    def __init__(self, window_s: float = WAIST_CONTROL_STATS_WINDOW_S, clock=time.monotonic):
        """绑定窗口长度与单调时钟（测试可注入）。"""
        self.window_s = window_s
        self.clock = clock
        self._generation = None
        self._reset(None)

    def _reset(self, generation) -> None:
        """开始一个新窗口。"""
        self._generation = generation
        self._start = self.clock()
        self._frames = 0
        self._mode_changes = 0
        self._last_mode = None

    def record(self, generation: int, sample) -> Optional[str]:
        """记录一帧腰输入；窗口结束时返回汇总文本，否则返回 None。"""
        if generation != self._generation:
            self._reset(generation)
        self._frames += 1
        if sample is not None:
            if self._last_mode is not None and sample.mode != self._last_mode:
                self._mode_changes += 1
            self._last_mode = sample.mode
        if self.clock() - self._start < self.window_s:
            return None
        summary = self._summary()
        self._reset(generation)
        return summary

    def _summary(self) -> str:
        """生成窗口汇总。"""
        return (f"[WaistControlStats] 窗口={self.clock() - self._start:.1f}s "
                f"帧数={self._frames} 模式变化={self._mode_changes}")


class _DirectRoute(NamedTuple):
    """直连会话接管待命 receiver 后的回调归属（即 connect 里的三个 Host 回调）。"""

    lease: SourceLease
    on_frame: FrameCallback
    on_disconnect: DisconnectCallback
    on_status: StatusCallback


@dataclass
class _QuestEventCounter:
    """lease 内事件序号；换代首帧、重建基线凭据变化、计数回退只建基线，每帧增加最多记一次。"""

    receiver_generation: Optional[int] = None
    rebaseline_token: Optional[object] = None
    count: Optional[int] = None
    sequence: int = 0

    def rebaseline_needed(self, generation, rebaseline_token) -> bool:
        """本帧是否属于新的传输基线（换代或 receiver 被替换）。"""
        return self.receiver_generation != generation or self.rebaseline_token is not rebaseline_token

    def advance(self, count, generation, rebaseline_token):
        """吸收线计数并返回 lease 内累计序号。"""
        if self.rebaseline_needed(generation, rebaseline_token):
            self.receiver_generation, self.rebaseline_token, self.count = generation, rebaseline_token, None
        if self.count is not None and count > self.count:
            self.sequence += 1
        self.count = count
        return self.sequence


@dataclass
class _QuestHandIntentState:
    """手臂事件与旧按键适配共用同一 lease 内序号；只由输入线程读写。"""

    home: _QuestEventCounter = field(default_factory=_QuestEventCounter)
    legacy: LegacyHandIntentAdapter = field(default_factory=LegacyHandIntentAdapter)


@dataclass(frozen=True)
class _QuestConnectionSnapshot:
    """一次连接拆解时的资源快照（用于可重试的清理）。

    ``_detach_connection`` 先把 lease/receiver/serial 从实例上摘下来并冻结成
    本对象，之后在锁外做 ADB 与 socket 清理；清理失败就把快照放进 ``_pending_cleanup``。
    """

    lease: SourceLease
    receiver: Optional[QuestTcpReceiver]
    serial: Optional[str]
    reason: str


class QuestInputModule(BaseInputSourceModule):
    """Quest 外部骨骼/手柄输入源：ADB 连接 + TCP/NDJSON 位姿流。

    能力位：provides_pose/provides_buttons/can_control_arms/supports_reconnect_grace/
    configures_arm_session 均为 True，即参与 Host 机械臂会话配置并支持重连宽限；
    input_clocked_control 为 True：输入帧屏障与控制线程序号共同保证每帧最多派发一次，控制环按新帧唤醒。
    输入必须是一次原子的双手 batch，policy 单实例串行处理。
    """

    module_id = "quest"
    reconnect_grace_seconds = QUEST_EXIT_TIMEOUT
    arm_profile = ArmProfile(
        models=frozenset({"A3", "D3"}),
        modes=frozenset({"a3_single", "a3_dual", "d3_dual"}),
    )

    def __init__(self) -> None:
        self.policy = QuestInputPolicy()
        self.receiver: Optional[QuestTcpReceiver] = None
        self.serial: Optional[str] = None
        self.lease: Optional[SourceLease] = None
        self._device_status_poller: Optional[QuestDeviceStatusPoller] = None
        self.lifecycle_lock = threading.RLock()
        self.cleanup_lock = threading.Lock()
        self._pending_cleanup = []
        self.pose_jump_config = None
        self.pose_jump_error: Optional[str] = None
        self._shutdown = False
        # (lease, 每手状态, 机身事件计数) 由输入线程读写；生命周期线程只整体替换该元组，
        # 迟到的旧 lease 帧用临时状态计算（Host 会按 lease 丢弃）；替换 receiver 时只
        # 发布重建基线凭据。
        self._intent_slot: Tuple[
            Optional[SourceLease], Dict[str, _QuestHandIntentState], Dict[str, _QuestEventCounter],
        ] = (None, {}, {})
        self._intent_rebaseline_token = object()
        # 诊断统计只由输入线程更新。
        self.waist_control_stats = WaistControlStats()
        self._waist_state = {"type": "waist_state", "status": "unavailable",
                             "following": False, "mode": None, "error": None}
        self._arm_angle_state = {
            "type": "arm_angle_state",
            "left": dict(_ARM_ANGLE_UNAVAILABLE),
            "right": dict(_ARM_ANGLE_UNAVAILABLE),
        }
        # 直连待命：Quest 在线且没有会话时保持 reverse 与监听，APK 一打开就能握手。
        # 直连连接把同一个 receiver 的回调路由给 Host（_direct_route），会话结束再路由回待命。
        self._direct_listener = None
        self._direct_route: Optional[_DirectRoute] = None

    @staticmethod
    def availability() -> SourceAvailability:
        """返回 Quest 源的可用性与能力声明（始终 available，参数错误在 configure 时暴露）。"""
        return SourceAvailability(
            available=True,
            reason=None,
            capabilities=SourceCapabilities(
                provides_pose=True,
                provides_buttons=True,
                can_control_arms=True,
                connect_only=False,
                supports_reconnect_grace=True,
                configures_arm_session=True,
                input_clocked_control=True,
            ),
        )

    @staticmethod
    def scan() -> Tuple[DeviceCandidate, ...]:
        """扫描当前可用的 Quest ADB 设备并包成设备候选。

        Returns:
            设备候选 tuple；无可用设备时为空 tuple。
        """
        capabilities = QuestInputModule.availability().capabilities
        return tuple(
            DeviceCandidate(
                provider_id="quest",
                public_source_type="quest",
                address=device["address"],
                name=device["name"],
                capabilities=capabilities,
            )
            for device in scan_quest_devices()
        )

    @staticmethod
    def resolve_device(
        address: str,
        name: str,
        scanned_candidate: Optional[DeviceCandidate],
    ) -> Optional[DeviceCandidate]:
        """
把用户输入的 address/name 解析为 Quest 设备候选（真正的 Quest 身份在 connect 阶段复核）。

Args:
    address: ADB serial。
    name: 用户看到的设备名。
    scanned_candidate: 扫描阶段已得到的候选；非 None 时直接采用。

Returns:
    Quest 设备候选；无法识别时返回 None。
"""
        if scanned_candidate is not None:
            return scanned_candidate if scanned_candidate.provider_id == "quest" else None
        device_info = get_adb_device_info(address)
        lowered_name = (name or "").lower()
        if not (
            (device_info and is_quest_adb_device(device_info))
            or any(marker in lowered_name for marker in ("quest", "oculus", "meta"))
        ):
            return None
        return DeviceCandidate(
            provider_id="quest",
            public_source_type="quest",
            address=address,
            name=name,
            capabilities=QuestInputModule.availability().capabilities,
        )

    def validate_input_batch(self, batch: RawInputBatch) -> None:
        """校验 batch 是否为本模块的原子双手批次。

        Raises:
            ValueError: provider 不符、非 atomic batch，或不是一个左手 + 一个右手。
        """
        if (
            batch.lease.provider_id != self.module_id
            or not batch.atomic_batch
            or {update.hand for update in batch.hands} != {HAND_LEFT, HAND_RIGHT}
        ):
            raise ValueError("Quest input must be one atomic dual-hand batch")

    def process_input_batch(
        self,
        batch: RawInputBatch,
        bound_hands,
    ):
        """
校验 batch 并交给 policy，直接返回逐手 InputUpdateDecision。

Args:
    batch: 一个 Quest 帧对应的原始输入 batch。
    bound_hands: Host 侧当前已绑定的 hand 集合。

Returns:
    与 batch.hands 同序的逐手决策 tuple；重复帧返回空 tuple。

Raises:
    ValueError: batch 非法，或配置缺失/错误。
"""
        self.validate_input_batch(batch)
        if self.pose_jump_error is not None:
            raise ValueError(self.pose_jump_error)
        if self.pose_jump_config is None:
            raise ValueError("Quest pose jump configuration is missing")
        return self.policy.process_frame(
            batch,
            bound_hands,
            self.pose_jump_config.position_threshold_m,
            self.pose_jump_config.rotation_threshold_deg,
        )

    def connect(
        self,
        candidate: DeviceCandidate,
        lease: SourceLease,
        event_sink: SourceEventSink,
        direct: bool = False,
    ) -> ConnectResult:
        """
拥有完整的 ADB、listener、reverse 与 first-frame 握手流程，任一步失败返回 FAILED。

依次：校验身份 → 确认 Quest → 清理旧应用/reverse → 启动 receiver → 注册
serial → adb reverse → 启动 App → 等待首帧（每 4s 重试启动）。失败路径不回收
receiver，由 close_connection/cleanup_idle_resources 负责。直连时不走这条路径，
而是接管已与 APK 握手的待命 receiver，不重启 App、不重建 TCP 连接。

Args:
    candidate: 已解析的设备候选。
    lease: 本次连接身份。
    event_sink: Host 的事件 sink，用于发布 transport/input 事件。
    direct: 是否接管直连待命链路。

Returns:
    READY（首帧已协商成功）、PENDING 或 FAILED；本实现不返回 PENDING。
"""
        if candidate.provider_id != self.module_id or lease.provider_id != self.module_id:
            return ConnectResult(ConnectOutcome.FAILED, "Quest 连接身份无效", False)
        info = self.device_info(candidate.address)
        state = info.get("state") if info else None
        if state != "device":
            message = {
                "unauthorized": "Quest 未授权，请先在头显中允许 USB 调试",
                "offline": "Quest 设备离线，请重连 USB",
            }.get(state, "Quest 未授权或不在线")
            return ConnectResult(ConnectOutcome.FAILED, message, False)
        if not self.is_quest_device(info):
            return ConnectResult(ConnectOutcome.FAILED, "当前 ADB 设备不是 Quest", False)

        def current(receiver=None) -> bool:
            """sink 侧复核：该 lease 与该 transport 实例是否仍是当前连接。"""
            try:
                return bool(event_sink.connection_is_current(lease, receiver))
            except Exception:
                return False

        def publish_transport(phase, generation, reason=None, terminal=False, receiver=None):
            """连接仍有效时发布一个 transport 阶段事件。"""
            if not current(receiver):
                return
            event_sink.publish_transport(
                TransportEvent(lease, generation, phase, reason, terminal),
                receiver,
            )

        def on_frame(batch, receiver):
            """receiver 帧回调：Host 按冻结 batch 的 lease 判定归属。"""
            event_sink.publish_input(batch, receiver)

        def on_disconnect(reason, input_generation, receiver_generation, receiver):
            """receiver 断链回调：按 reason 判定是否属于不可恢复的 terminal 断链。"""
            if not current(receiver):
                return
            terminal = (
                reason == "quest-invalid-frame"
                or reason == "quest_receiver_error"
                or reason.startswith("quest_protocol_error")
                or reason in {"listener_failed", "quest-listener-failed"}
            )
            publish_transport(
                "terminal" if terminal else "disconnected",
                receiver_generation,
                reason,
                terminal,
                receiver,
            )

        def on_status(status, input_generation, receiver_generation, receiver):
            """receiver 协议状态回调：转成小写 phase 的 transport 事件。"""
            publish_transport(
                status.lower(),
                receiver_generation,
                getattr(receiver, "last_error", None),
                status.lower() == "listenerfailed",
                receiver,
            )

        try:
            if direct:
                receiver = self._route_standby_to_host(
                    lease, candidate.address, on_frame, on_disconnect, on_status,
                )
                if receiver is None:
                    return ConnectResult(ConnectOutcome.FAILED, "Quest 待命链路未就绪，请确认 APK 已打开", False)
                return self._finish_connect(lease, candidate.address, receiver, event_sink, current, publish_transport)
            self.stop_standby()
            if not self.reset_device(candidate.address):
                return ConnectResult(ConnectOutcome.FAILED, "Quest 旧应用或转发清理失败", False)
            receiver = self.ensure_receiver(lease, on_frame, on_disconnect, on_status)
            self.start_receiver(lease)
            if not self.register_serial(lease, candidate.address):
                return ConnectResult(ConnectOutcome.FAILED, "Quest 连接已过期", False)
            reverse = self.setup_reverse(candidate.address)
            if getattr(reverse, "returncode", 1) != 0:
                return ConnectResult(
                    ConnectOutcome.FAILED,
                    f"adb reverse 失败: {getattr(reverse, 'stderr', '') or getattr(reverse, 'stdout', '')}",
                    False,
                )
            if not current(receiver):
                return ConnectResult(ConnectOutcome.FAILED, "Quest 连接已过期", False)
            if not self.launch(candidate.address):
                return ConnectResult(ConnectOutcome.FAILED, "Quest 应用启动失败", False)

            started = time.monotonic()
            next_launch = started + 4.0
            while not receiver.first_frame_event.is_set():
                if not current(receiver):
                    return ConnectResult(ConnectOutcome.FAILED, "Quest 连接已过期", False)
                now = time.monotonic()
                if now >= next_launch:
                    self.launch(candidate.address)
                    next_launch = now + 4.0
                if now - started >= QUEST_FIRST_FRAME_TIMEOUT:
                    break
                receiver.first_frame_event.wait(timeout=0.5)

            if not receiver.first_frame_event.is_set():
                return ConnectResult(
                    ConnectOutcome.FAILED,
                    "等待 Quest 首帧数据超时",
                    False,
                )
            if not current(receiver) or not receiver.connection_negotiated:
                return ConnectResult(ConnectOutcome.FAILED, "Quest 首帧连接已失效，请重新连接", False)
            return self._finish_connect(lease, candidate.address, receiver, event_sink, current, publish_transport)
        except OSError as exc:
            return ConnectResult(ConnectOutcome.FAILED, f"Quest 接收端口启动失败: {exc}", False)
        except Exception as exc:
            logging.exception("[QuestInputModule] connect failed")
            return ConnectResult(ConnectOutcome.FAILED, f"Quest 接收器初始化失败: {exc}", False)

    def _finish_connect(self, lease, serial, receiver, event_sink, current, publish_transport) -> ConnectResult:
        """连接成功的公共收尾：发布 ready 并启动电量轮询。"""
        publish_transport("ready", receiver.connection_generation, None, False, receiver)

        def publish_device_status(items):
            """轮询线程回调：连接仍有效时发布头盔/手柄电量。"""
            if current(receiver):
                event_sink.publish_telemetry(lease, receiver, items)

        self.start_device_status(lease, serial, publish_device_status)
        return ConnectResult(ConnectOutcome.READY, "连接成功", False)

    def start_standby(self, serial: str, listener) -> bool:
        """
没有会话时为在线的 Quest 建立待命链路（reverse + 监听）；已在待命则直接返回。

待命期间位姿帧丢弃，控制记录与握手交给 listener。listener 需提供
``enqueue_control_record(payload, receiver, generation)``、``on_negotiated(receiver, generation)``
与 ``on_standby_disconnect(reason)``，它们在接收器线程上调用，只能入队。

Returns:
    待命链路就绪为 True；有会话占用或 reverse 失败为 False。

Raises:
    OSError: 监听端口绑定失败。
"""
        with self.lifecycle_lock:
            if self.lease is not None or self._shutdown:
                return False
            if (
                self._direct_listener is listener and self.serial == serial
                and self.receiver is not None and self.receiver.listening
            ):
                return True
        self.stop_standby()
        if self.setup_reverse(serial).returncode != 0:
            return False
        receiver = QuestTcpReceiver(
            QUEST_TCP_HOST,
            QUEST_TCP_PORT,
            self._direct_frame,
            self._direct_disconnect,
            self._direct_status,
            control_callback=listener.enqueue_control_record,
            negotiated_callback=self._on_negotiated,
        )
        with self.lifecycle_lock:
            if self.lease is not None or self.receiver is not None or self._shutdown:
                return False
            receiver.start()
            self.receiver = receiver
            self.serial = serial
            self._direct_listener = listener
            self._direct_route = None
        return True

    def stop_standby(self) -> None:
        """停止待命监听并移除 reverse（Quest 离线、Web 接管或模块关闭时）。"""
        with self.lifecycle_lock:
            if self._direct_listener is None or self.lease is not None:
                return
            receiver, serial = self.receiver, self.serial
            self.receiver = None
            self.serial = None
            self._direct_listener = None
        if receiver is not None:
            receiver.stop()
        if serial:
            try:
                run_adb(["-s", serial, "reverse", "--remove", f"tcp:{QUEST_TCP_PORT}"], timeout=5.0)
            except Exception as exc:
                logging.warning("[QuestInputModule] Failed to remove standby adb reverse: %s", exc)

    def direct_link_ready(self) -> bool:
        """待命链路上是否有已握手的 APK。"""
        with self.lifecycle_lock:
            receiver = self.receiver
            return bool(self._direct_listener is not None and receiver is not None and receiver.link_ready)

    def _on_negotiated(self, receiver, generation):
        """两种链路握手后发送缓存腰状态；直连通知仍只交给 listener 入队。"""
        with self.lifecycle_lock:
            if self.receiver is not receiver or self._shutdown:
                return
            if receiver.waist_control_v1_enabled:
                receiver._queue_control(self._waist_state)
            if receiver.intent_v1_enabled and {"ia", "lia"} <= receiver.declared_intents:
                receiver._queue_control(self._arm_angle_state)
            listener = self._direct_listener
        if listener is not None:
            listener.on_negotiated(receiver, generation)

    def set_waist_state(self, status, following=False, mode=None, error=None):
        """缓存最新腰状态；仅向已协商统一腰能力的连接发送变化。"""
        record = {"type": "waist_state", "status": status, "following": bool(following),
                  "mode": mode, "error": error}
        with self.lifecycle_lock:
            if record == self._waist_state:
                return
            self._waist_state = record
            receiver = self.receiver
            if receiver is not None and receiver.link_ready and receiver.waist_control_v1_enabled:
                receiver._queue_control(record)

    def set_arm_angle_state(self, left, right):
        """缓存最新臂型角状态，变化时发给已声明两个臂型角 intent 的 Quest 连接。"""
        record = {"type": "arm_angle_state", "left": dict(left), "right": dict(right)}
        with self.lifecycle_lock:
            if record == self._arm_angle_state:
                return
            self._arm_angle_state = record
            receiver = self.receiver
            if (receiver is not None and receiver.link_ready and receiver.intent_v1_enabled
                    and {"ia", "lia"} <= receiver.declared_intents):
                receiver._queue_control(record)

    def _publish_unavailable_states(self) -> None:
        """会话结束：腰与臂型角回到不可用，并清除下次握手可能重发的会话状态。"""
        self.set_waist_state("unavailable")
        self.set_arm_angle_state(_ARM_ANGLE_UNAVAILABLE, _ARM_ANGLE_UNAVAILABLE)

    def send_direct_record(self, record: Dict[str, Any]) -> bool:
        """向已协商 session_setup_v1 的直连链路发送一条控制记录。"""
        with self.lifecycle_lock:
            receiver = self.receiver
            if self._direct_listener is None or receiver is None or not receiver.link_ready \
                    or not receiver.session_setup_v1_enabled:
                return False
            return receiver._queue_control(record)

    def _route_standby_to_host(self, lease, serial, on_frame, on_disconnect, on_status):
        """把已握手的待命 receiver 路由给 Host lease；链路未就绪返回 None。"""
        with self.lifecycle_lock:
            receiver = self.receiver
            if self._direct_listener is None or self.serial != serial \
                    or receiver is None or not receiver.link_ready:
                return None
            self.bind_lease(lease)
            self._direct_route = _DirectRoute(lease, on_frame, on_disconnect, on_status)
            return receiver

    def _release_direct_lease(self, lease: SourceLease) -> bool:
        """直连会话结束：只解绑 lease，receiver 与 reverse 留作待命，不通知 APK 退出。"""
        with self.lifecycle_lock:
            route = self._direct_route
            if route is None or route.lease != lease or self.lease != lease:
                return False
            self._direct_route = None
            self.lease = None
            self._intent_slot = (None, {}, {})
            self._publish_unavailable_states()
            poller, self._device_status_poller = self._device_status_poller, None
        if poller is not None:
            poller.stop()
        return True

    def _direct_frame(self, payload: dict, receiver_generation: int) -> None:
        """待命 receiver 的帧回调：有会话时交给 Host，待命时丢弃。

        稳态输入路径不取 lifecycle_lock：路由与 receiver 都是整体替换的引用，迟到帧
        由 Host 按 lease 过滤。
        """
        route, receiver = self._direct_route, self.receiver
        if route is not None:
            self._publish_receiver_payload(route.lease, payload, receiver_generation, receiver, route.on_frame)

    def _direct_disconnect(self, reason: str, receiver_generation: int) -> None:
        """待命 receiver 的断链回调：有会话时走 Host 的重连宽限，待命时通知 listener。"""
        with self.lifecycle_lock:
            route, receiver, listener = self._direct_route, self.receiver, self._direct_listener
        if route is not None:
            route.on_disconnect(reason, route.lease.input_generation, receiver_generation, receiver)
        elif listener is not None:
            listener.on_standby_disconnect(reason)

    def _direct_status(self, status: str, receiver_generation: int) -> None:
        """待命 receiver 的协议状态回调：只在有会话时转给 Host。"""
        with self.lifecycle_lock:
            route, receiver = self._direct_route, self.receiver
        if route is not None:
            route.on_status(status, route.lease.input_generation, receiver_generation, receiver)

    def bind_lease(self, lease: SourceLease) -> None:
        """绑定 lease（创建 receiver 前的前置条件）。

        Raises:
            RuntimeError: 模块已 shutdown、旧 lease 未关闭，或上一次清理尚未完成。
            ValueError: lease 的 provider 不是 quest。
        """
        with self.lifecycle_lock:
            if self._shutdown:
                raise RuntimeError("Quest module is shut down")
            if lease.provider_id != self.module_id:
                raise ValueError("Quest module requires a Quest lease")
            if self.lease is not None and self.lease != lease:
                raise RuntimeError("previous Quest lease must be closed before rebinding")
            if self._pending_cleanup:
                raise RuntimeError("previous Quest cleanup must finish before rebinding")
            if self.lease != lease:
                self._intent_slot = (lease, {}, {})
            self.lease = lease

    def start_device_status(self, lease: SourceLease, serial: str, publish) -> None:
        """为当前 lease 启动电量轮询；lease 已变或已 shutdown 时不启动。"""
        with self.lifecycle_lock:
            if self.lease != lease or self._shutdown:
                return
            previous = self._device_status_poller
            poller = QuestDeviceStatusPoller(serial, publish)
            self._device_status_poller = poller
            poller.start()
        if previous is not None:
            previous.stop()

    def current_lease(self) -> Optional[SourceLease]:
        """返回当前 lease；若仅剩待清理快照，则返回该快照的 lease。"""
        with self.lifecycle_lock:
            if self.lease is not None:
                return self.lease
            if self._pending_cleanup:
                return self._pending_cleanup[0].lease
            return None

    def callback_matches(self, lease: SourceLease, receiver) -> bool:
        """判断回调携带的 lease/receiver 是否仍是当前连接（已 shutdown 则一律 False）。"""
        with self.lifecycle_lock:
            return bool(
                self.lease == lease
                and self.receiver is receiver
                and not self._shutdown
            )

    def configure(self, config) -> None:
        """从 Host 配置解析 pose jump 阈值；解析失败时记录错误以拒绝后续帧。"""
        try:
            self.configure_pose_jump(config.resolve_quest_pose_jump())
        except Exception as exc:
            self.configure_pose_jump(None, str(exc))
            logging.error(
                "[QuestInputModule] Frames will be rejected by the config gate: %s",
                exc,
            )

    def ensure_receiver(
        self,
        lease: SourceLease,
        frame_callback: FrameCallback,
        disconnect_callback: DisconnectCallback,
        status_callback: StatusCallback,
    ) -> QuestTcpReceiver:
        """
创建（并绑定到 lease 的）TCP 接收器，必要时停掉旧接收器。

Args:
    lease: 本次连接身份。
    frame_callback: 帧回调（frame 批次 + receiver）。
    disconnect_callback: 断链回调（reason + input/receiver generation + receiver）。
    status_callback: 协议状态回调（status + generation + receiver）。

Returns:
    尚未启动的新 receiver（调用方需再调用 ``start_receiver``）。

Raises:
    RuntimeError: lease 在创建期间发生变化或模块已 shutdown。
"""
        self.bind_lease(lease)
        generation = lease.input_generation
        receiver_holder = {}
        receiver = QuestTcpReceiver(
            QUEST_TCP_HOST,
            QUEST_TCP_PORT,
            lambda payload, receiver_generation: self._publish_receiver_payload(
                lease,
                payload,
                receiver_generation,
                receiver_holder["receiver"],
                frame_callback,
            ),

            lambda reason, receiver_generation: disconnect_callback(
                reason,
                generation,
                receiver_generation,
                receiver_holder["receiver"],
            ),
            lambda status, receiver_generation: status_callback(
                status,
                generation,
                receiver_generation,
                receiver_holder["receiver"],
            ),
            negotiated_callback=self._on_negotiated,
        )
        receiver_holder["receiver"] = receiver
        with self.lifecycle_lock:
            stale_lease = self.lease != lease or self._shutdown
            previous = None if stale_lease else self.receiver
            if not stale_lease:
                self.receiver = receiver
                # 同一 lease 可替换 receiver，其 generation 可能重新从 1 开始。
                # 保留累计序号，只使下一帧重新建立传输基线。
                self._intent_rebaseline_token = object()
        if stale_lease:
            receiver.stop()
            raise RuntimeError("Quest lease changed while creating receiver")
        if previous is not None:
            previous.stop()
        return receiver

    def start_receiver(self, lease: SourceLease) -> QuestTcpReceiver:
        """启动该 lease 的 receiver。

        Raises:
            RuntimeError: 该 lease 不是当前 lease，或 receiver 尚未创建。
        """
        with self.lifecycle_lock:
            if self.lease != lease or self.receiver is None:
                raise RuntimeError("Quest receiver is not configured for this lease")
            self.receiver.start()
            return self.receiver

    def parse_frame(self, payload: dict) -> dict:
        """把 receiver 已校验的 payload 投影为左右手冻结样本（由 parser 实现）。"""
        return parse_quest_frame(
            payload, intent_v1=self.receiver is not None and self.receiver.intent_v1_enabled,
            declared_intents=self.receiver.declared_intents if self.receiver is not None else None,
        )

    def _intent_states(self, lease):
        """返回本 lease 的 (每手状态, 机身事件计数)；迟到的旧 lease 帧得到临时状态。"""
        slot_lease, hands, body = self._intent_slot
        if slot_lease != lease:
            return {}, {}
        return hands, body

    def _hand_intent(self, lease, hand, parsed_intent, button, receiver_generation):
        """累计事件不生成脉冲；运动中是否丢弃由消费功能结果的控制器决定。

        只由输入线程调用；生命周期线程不直接改写状态，见 ``_intent_slot``。
        """
        hands, _ = self._intent_states(lease)
        state = hands.setdefault(hand, _QuestHandIntentState())
        counter = state.home
        rebaseline = self._intent_rebaseline_token
        if counter.rebaseline_needed(receiver_generation, rebaseline):
            state.legacy.reset(counter.sequence)
        if parsed_intent is None:
            counter.receiver_generation = receiver_generation
            counter.rebaseline_token = rebaseline
            counter.count = None
            intent = state.legacy.adapt(button)
            counter.sequence = intent.home_sequence
            return intent
        sequence = counter.advance(parsed_intent["home_sequence"], receiver_generation, rebaseline)
        return HandIntentSample(
            motion_engaged=parsed_intent["motion_engaged"],
            rotation_locked=parsed_intent["rotation_locked"],
            end_effector=parsed_intent["end_effector"],
            home_sequence=sequence, timestamp=button.timestamp,
            arm_angle_stick=parsed_intent.get("arm_angle_stick", (0.0, 0.0)),
        )

    def _body_intent(self, lease, parsed, generation, timestamp):
        """机身事件复用手臂事件计数器；三个腰字段齐全才发布腰控制输入。"""
        if parsed is None:
            return None
        _, body = self._intent_states(lease)
        rebaseline = self._intent_rebaseline_token
        sequences = {}
        for name in ("waist_follow_sequence", "waist_home_sequence"):
            if name in parsed:
                sequences[name] = body.setdefault(name, _QuestEventCounter()).advance(
                    parsed[name], generation, rebaseline)
        if "waist_stick" not in parsed or not {"waist_follow_sequence", "waist_home_sequence"} <= sequences.keys():
            return None
        return BodyIntentSample(sequences["waist_follow_sequence"],
                                sequences["waist_home_sequence"],
                                parsed["waist_stick"], float(timestamp))

    def _publish_receiver_payload(
        self,
        lease: SourceLease,
        payload: dict,
        receiver_generation: int,
        receiver: QuestTcpReceiver,
        frame_callback: FrameCallback,
    ) -> None:
        """输入线程构造冻结 batch；Host 和读者按 lease 过滤迟到帧。"""
        batch = self._build_input_batch(lease, payload, receiver_generation)
        summary = (self.waist_control_stats.record(receiver_generation, batch.waist_input)
                   if receiver.waist_control_v1_enabled else None)
        if summary is not None:
            logging.info(summary)
        frame_callback(batch, receiver)

    def _build_input_batch(
        self,
        lease: SourceLease,
        payload: dict,
        receiver_generation: int,
    ) -> RawInputBatch:
        """把 receiver 已校验的 Quest 数据投影成冻结 batch（并推进功能序号），再交给 Host。"""
        parsed = self.parse_frame(payload)
        updates = []
        for hand in ("left", "right"):
            parsed_hand = parsed[hand]
            updates.append(HandInputUpdate(
                hand=hand,
                update_pose=True,
                pose=parsed_hand["pose"],
                pose_valid=parsed_hand["pose_valid"],
                update_button=True,
                button=parsed_hand["button"],
                intent=self._hand_intent(
                    lease, hand, parsed_hand.get("intent"), parsed_hand["button"], receiver_generation,
                ),
            ))
        return RawInputBatch(
            lease=lease,
            transport_generation=receiver_generation,
            batch_sequence=parsed["frame_index"],
            atomic_batch=True,
            hands=tuple(updates),
            waist_input=(parse_waist_control(payload) if self.receiver is not None
                         and self.receiver.waist_control_v1_enabled else None),
            body_intent=self._body_intent(lease, parsed.get("body_intent"), receiver_generation, parsed["timestamp"]),
        )


    @staticmethod
    def device_info(serial: str):
        """返回指定 serial 的 ADB 设备信息（不存在时 None）。"""
        return get_adb_device_info(serial)

    @staticmethod
    def is_quest_device(device_info) -> bool:
        """判断一份 ADB 设备信息是否可识别为 Quest。"""
        return bool(device_info and is_quest_adb_device(device_info))

    def register_serial(self, lease: SourceLease, serial: str) -> bool:
        """登记本次连接使用的 ADB serial；lease 已变或已 shutdown 时返回 False。"""
        with self.lifecycle_lock:
            if self.lease != lease or self._shutdown:
                return False
            self.serial = serial
            return True

    @staticmethod
    def reset_device(serial: str) -> bool:
        """
强制停掉旧 Quest 应用并移除旧 reverse 监听（listener 不存在也算成功）。

Returns:
    force-stop 与 reverse 清理都已生效时为 True。
"""
        stopped = run_adb(["-s", serial, "shell", "am", "force-stop", QUEST_PACKAGE], timeout=8.0)
        removed = run_adb(["-s", serial, "reverse", "--remove", f"tcp:{QUEST_TCP_PORT}"], timeout=5.0)
        missing = f"listener 'tcp:{QUEST_TCP_PORT}' not found" in (removed.stderr + removed.stdout).lower()
        return stopped.returncode == 0 and (removed.returncode == 0 or missing)

    @staticmethod
    def device_is_absent(serial: str) -> bool:
        """只有成功的 ADB 清单查询才能真正证明设备不在。"""
        result = run_adb(["devices"], timeout=5.0)
        if result.returncode != 0:
            return False
        lines = result.stdout.splitlines()
        if not any(line.startswith("List of devices attached") for line in lines):
            return False
        return not any(line.split() and line.split()[0] == serial for line in lines)

    @staticmethod
    def setup_reverse(serial: str):
        """建立 adb reverse（把设备的 TCP 端口映射回本地）。"""
        return run_adb(
            ["-s", serial, "reverse", f"tcp:{QUEST_TCP_PORT}", f"tcp:{QUEST_TCP_PORT}"],
            timeout=8.0,
        )

    @staticmethod
    def launch(serial: str) -> bool:
        """启动设备上的 Quest 遥操 App。"""
        return launch_quest_app(serial)

    def owned_controllers(self):
        """返回 policy 当前持有的全部 controller。"""
        return self.policy.owned_controllers()

    @property
    def control_policy(self):
        """输入检测策略实例；控制线程只读取 Host 冻结帧。"""
        return self.policy

    def controller_for_hand(self, hand: str):
        """代理到 policy：返回某手绑定的 controller。"""
        return self.policy.controller_for_hand(hand)

    def controller_matches(self, hand: str, controller: object) -> bool:
        """代理到 policy：按对象身份判断某手的绑定是否与给定 controller 一致。"""
        return self.policy.controller_matches(hand, controller)

    def bind_controller(self, hand, controller):
        """代理到 policy：绑定 controller 并返回被替换的旧 controller。"""
        return self.policy.bind_controller(hand, controller)

    def release_controller(self, hand, controller):
        """代理到 policy：解绑 controller；False 表示当前绑定与传入 controller 不符。"""
        return self.policy.release_controller(hand, controller)

    def mark_input_disconnected(self):
        """代理到输入策略：断链时清空比较基准和边沿。"""
        return self.policy.mark_disconnected()

    def reset_policy(self) -> None:
        """代理到输入策略：重置比较状态与绑定。"""
        self.policy.reset()

    def configure_pose_jump(self, config, error: Optional[str] = None) -> None:
        """记录 pose jump 阈值或配置错误（error 非 None 时后续帧一律被拒绝）。"""
        self.pose_jump_config = config
        self.pose_jump_error = error

    def _detach_connection(
        self,
        lease: SourceLease,
        reason: str,
    ) -> Optional[_QuestConnectionSnapshot]:
        """
从实例上摘下 lease/receiver/serial 并冻结为快照（资源清理在锁外进行）。

Args:
    lease: 待摘除的连接身份。
    reason: 关闭原因，会随快照传到 session_end 与日志。

Returns:
    资源快照；lease 已不匹配时返回 None。
"""
        with self.lifecycle_lock:
            if self.lease != lease:
                return None
            receiver = self.receiver
            serial = self.serial
            self.receiver = None
            self.serial = None
            self.lease = None
            self._intent_slot = (None, {}, {})
            self._publish_unavailable_states()
            if self._device_status_poller is not None:
                # 只置位不等待；迟到的发布会因 lease 已摘除被身份复核丢弃。
                self._device_status_poller.stop()
                self._device_status_poller = None
            return _QuestConnectionSnapshot(
                lease=lease,
                receiver=receiver,
                serial=serial,
                reason=reason,
            )

    @staticmethod
    def _cleanup_snapshot(snapshot: _QuestConnectionSnapshot) -> bool:
        """
执行一次快照的资源清理（session_end → 停表 → ADB 清理）。

Returns:
    全部资源已确认释放为 True；仍有残留为 False（调用方放进 ``_pending_cleanup`` 重试）。
"""
        success = True
        receiver = snapshot.receiver
        serial = snapshot.serial
        if receiver is not None and receiver._connected:
            if not receiver.send_session_end(snapshot.reason, True):
                # session_end 只是通知；传输/进程的拆除才是权威动作。
                logging.warning("[QuestInputModule] Failed to send best-effort Quest session_end")

        if receiver is not None and receiver.stop() is False:
            success = False

        absent = False
        if serial:
            try:
                absent = QuestInputModule.device_is_absent(serial)
            except Exception:
                logging.warning("[QuestInputModule] Cannot confirm device absence")
        if serial and not absent:
            try:
                reverse_remove = run_adb(
                    ["-s", serial, "reverse", "--remove", f"tcp:{QUEST_TCP_PORT}"],
                    timeout=5.0,
                )
                if reverse_remove.returncode != 0:
                    reverse_remove_error = "\n".join(
                        output.strip()
                        for output in (reverse_remove.stderr, reverse_remove.stdout)
                        if output and output.strip()
                    )
                    listener_missing = (
                        f"listener 'tcp:{QUEST_TCP_PORT}' not found"
                        in reverse_remove_error.lower()
                    )
                    # ADB 已无监听时，清理的后置条件已经满足。
                    if listener_missing:
                        logging.info(
                            "[QuestInputModule] adb reverse listener already absent: tcp:%s",
                            QUEST_TCP_PORT,
                        )
                    else:
                        logging.warning(
                            "[QuestInputModule] Failed to remove adb reverse: %s",
                            reverse_remove_error,
                        )
                        success = False
            except Exception as exc:
                logging.warning("[QuestInputModule] Failed to remove adb reverse: %s", exc)
                success = False
            try:
                force_stop = run_adb(
                    ["-s", serial, "shell", "am", "force-stop", QUEST_PACKAGE],
                    timeout=8.0,
                )
                if force_stop.returncode != 0:
                    logging.warning(
                        "[QuestInputModule] Failed to force-stop Quest app: %s",
                        force_stop.stderr.strip() or force_stop.stdout.strip(),
                    )
                    success = False
            except Exception as exc:
                logging.warning("[QuestInputModule] Failed to force-stop Quest app: %s", exc)
                success = False

        return success

    def drain_pending_cleanup(self) -> bool:
        """按顺序重试全部 pending 清理（cleanup_lock 串行化）。"""
        with self.cleanup_lock:
            return self._drain_pending_cleanup()

    def _drain_pending_cleanup(self) -> bool:
        """FIFO 重试待清理快照：一个失败就停下，下次再从那一个继续。"""
        while True:
            with self.lifecycle_lock:
                if not self._pending_cleanup:
                    return True
                snapshot = self._pending_cleanup[0]
            if not self._cleanup_snapshot(snapshot):
                return False
            with self.lifecycle_lock:
                if self._pending_cleanup and self._pending_cleanup[0] is snapshot:
                    self._pending_cleanup.pop(0)

    def close_connection(
        self,
        lease: SourceLease,
        reason: str = "backend_stop",
    ) -> bool:
        """关闭指定 lease 的连接（cleanup_lock 串行化，可从任意线程调用）。

        Args:
            lease: 待关闭的连接身份。
            reason: 关闭原因。

        Returns:
            True 表示连接与资源已彻底释放。
        """
        with self.cleanup_lock:
            return self._close_connection(lease, reason)

    def _close_connection(
        self,
        lease: SourceLease,
        reason: str = "backend_stop",
    ) -> bool:
        """
关闭或重试关闭一个 lease（调用方须已持有 cleanup_lock）。

失败则把快照追加到 ``_pending_cleanup``。

Args:
    lease: 待关闭的连接身份。
    reason: 关闭原因。

Returns:
    完成为 True；lease 不匹配或仍有残留已转为 pending 为 False。
"""
        if self._release_direct_lease(lease):
            return True
        with self.lifecycle_lock:
            pending = next(
                (
                    snapshot
                    for snapshot in self._pending_cleanup
                    if snapshot.lease == lease
                ),
                None,
            )
        if pending is not None:
            if not self._cleanup_snapshot(pending):
                return False
            with self.lifecycle_lock:
                if pending in self._pending_cleanup:
                    self._pending_cleanup.remove(pending)
            return True

        snapshot = self._detach_connection(lease, reason)
        if snapshot is None:
            return False
        if self._cleanup_snapshot(snapshot):
            return True
        with self.lifecycle_lock:
            self._pending_cleanup.append(snapshot)
        return False

    def cleanup_idle_resources(self) -> bool:
        """Host 周期性调用：重试 pending 清理。

        Returns:
            True 表示当前无残留（不代表从未有过连接）。
        """
        return self.drain_pending_cleanup()

    def shutdown(self, reason: str) -> bool:
        """
关闭模块：关闭当前 lease、排空 pending 清理，成功后置 shutdown 并重置 policy。

任一步残留返回 False 且不置 shutdown，供 Host 重试（已 shutdown 幂等返回 True）。

Args:
    reason: 关闭原因。

Returns:
    已关闭且无残留资源为 True。
"""
        with self.lifecycle_lock:
            if self._shutdown:
                return True
        lease = self.current_lease()
        if lease is not None:
            self.close_connection(lease, reason)
        self.stop_standby()
        success = self.drain_pending_cleanup() and self.current_lease() is None
        if success:
            self.policy.reset()
            with self.lifecycle_lock:
                self._shutdown = True
                self._intent_slot = (None, {}, {})
        return success

    def status(self, lease: SourceLease) -> ModuleStatus:
        """返回该 lease 的模块状态（协议阶段、是否已协商、connection id 与收包时间等）。

        lease 不匹配时返回中性 disconnected 状态（lease 为 None）。
        """
        with self.lifecycle_lock:
            current_lease = self.lease
            receiver = self.receiver
        if current_lease != lease:
            return ModuleStatus(None, False, "disconnected")
        return ModuleStatus(
            lease=lease,
            transport_connected=bool(receiver and receiver.connection_negotiated),
            phase=(receiver.protocol_state.lower() if receiver else "disconnected"),
            details=(
                ("protocol_version", getattr(receiver, "protocol_version", None) if receiver else None),
                ("connection_id", getattr(receiver, "connection_id", None) if receiver else None),
                ("last_rx_monotonic", getattr(receiver, "last_valid_rx_monotonic", None) if receiver else None),
                ("generation", getattr(receiver, "connection_generation", 0) if receiver else 0),
                ("waist_control_ready", bool(receiver and getattr(receiver, "waist_control_v1_enabled", False)
                    and receiver.intent_v1_enabled and {"wf", "wh", "ws"} <= receiver.declared_intents)),
                ("arm_angle_ready", bool(receiver and getattr(receiver, "intent_v1_enabled", False)
                    and {"ia", "lia"} <= receiver.declared_intents)),
            ),
        )
