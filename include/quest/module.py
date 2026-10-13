"""Quest 输入适配模块：拥有全部 ADB/TCP 资源与连接生命周期。

ADB 扫描/握手、reverse 转发、TCP 接收器与重连宽限定时器均由本模块创建与销毁
（policy 只做帧语义、receiver 只做协议状态机）。清理失败进入 ``_pending_cleanup``
重试，pending 非空时端口视为未释放。
"""

import logging
from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional, Tuple

from carm_teleop_input_contracts import (
    ArmProfile,
    BaseInputSourceModule,
    ConnectOutcome,
    ConnectResult,
    DeviceCandidate,
    HAND_LEFT,
    HAND_RIGHT,
    InputUpdateDecision,
    ModuleStatus,
    SourceAvailability,
    SourceCapabilities,
    RawInputBatch,
    HandInputUpdate,
    ButtonSample,
    PoseSample,
    SourceLease,
    SourceEventSink,
    TransportEvent,
)

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
from .parser import parse_quest_frame
from .policy import QuestFrameContext, QuestInputPolicy
from .receiver import QuestTcpReceiver


FrameCallback = Callable[[RawInputBatch, object], None]
DisconnectCallback = Callable[[str, int, int, QuestTcpReceiver], None]
StatusCallback = Callable[[str, int, int, QuestTcpReceiver], None]


@dataclass(frozen=True)
class _QuestConnectionSnapshot:
    """一次连接拆解时的资源快照（用于可重试的清理）。

    ``_detach_connection`` 先把 lease/receiver/serial/定时器从实例上摘下来并冻结成
    本对象，之后在锁外做 ADB 与 socket 清理；清理失败就把快照放进 ``_pending_cleanup``。
    """

    lease: SourceLease
    receiver: Optional[QuestTcpReceiver]
    serial: Optional[str]
    reconnect_timer: Optional[threading.Timer]
    reason: str


class QuestInputModule(BaseInputSourceModule):
    """Quest 外部骨骼/手柄输入源：ADB 连接 + TCP/NDJSON 位姿流。

    能力位：provides_pose/provides_buttons/can_control_arms/supports_reconnect_grace/
    configures_arm_session 均为 True，即参与 Host 机械臂会话配置并支持重连宽限；
    input_clocked_control 为 True：policy 帧屏障保证每帧最多派发一次，控制环按新帧唤醒。
    输入必须是一次原子的双手 batch，policy 单实例串行处理。
    """

    module_id = "quest"
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
        self.reconnect_deadline_monotonic: Optional[float] = None
        self.reconnect_timer: Optional[threading.Timer] = None
        self.reconnect_timer_lock = threading.Lock()
        self.lifecycle_lock = threading.RLock()
        self.cleanup_lock = threading.Lock()
        self._pending_cleanup = []
        self.pose_jump_config = None
        self.pose_jump_error: Optional[str] = None
        self._shutdown = False

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
        session_id: int,
        params_epoch: int,
    ):
        """
校验 batch 并把一帧双手数据交给 policy 处理，再转成逐手 InputUpdateDecision。

Args:
    batch: 一个 Quest 帧对应的原始输入 batch。
    bound_hands: Host 侧当前已绑定的 hand 集合。
    session_id: 当前 Host 会话身份。
    params_epoch: 当前参数发布 epoch。

Returns:
    与 batch.hands 同序的逐手决策 tuple；重复帧返回空 tuple。

Raises:
    ValueError: batch 非法、配置缺失/错误，或标记为有效的位姿不可用。
"""
        self.validate_input_batch(batch)
        if self.pose_jump_error is not None:
            raise ValueError(self.pose_jump_error)
        if self.pose_jump_config is None:
            raise ValueError("Quest pose jump configuration is missing")
        contexts = []
        for update in batch.hands:
            raw_pose = None
            if update.pose_valid:
                pose = update.pose
                raw_pose = self.policy.normalize_pose({
                    "x": pose.x,
                    "y": pose.y,
                    "z": pose.z,
                    "qx": pose.qx,
                    "qy": pose.qy,
                    "qz": pose.qz,
                    "qw": pose.qw,
                })
                if raw_pose is None:
                    raise ValueError(
                        f"Quest {update.hand} pose is marked valid but unusable"
                    )
            contexts.append(
                QuestFrameContext(
                    hand=update.hand,
                    frame_index=batch.batch_sequence,
                    pose_valid=update.pose_valid,
                    raw_pose=raw_pose,
                    grip=update.button.grip,
                    rotation_locked=bool(update.button.btn_yb),
                    bound_active=update.hand in bound_hands,
                )
            )
        decisions = self.policy.process_frame(
            contexts,
            session_id,
            params_epoch,
            batch.transport_generation,
            self.pose_jump_config.position_threshold_m,
            self.pose_jump_config.rotation_threshold_deg,
        )
        result = []
        for decision in decisions:
            details = ()
            if decision.jump_triggered:
                details = (
                    ("event", "pose_jump"),
                    ("frame_index", batch.batch_sequence),
                    ("position_m", decision.jump_position_m),
                    ("rotation_deg", decision.jump_rotation_deg),
                )
            result.append(
                InputUpdateDecision(
                    hand=decision.hand,
                    accept_pose=decision.accept_pose,
                    accept_button=True,
                    controller=decision.controller,
                    invalidate_controller=decision.invalidate_controller,
                    reject_controller=decision.reject_controller,
                    details=details,
                )
            )
        return tuple(result)

    def connect(
        self,
        candidate: DeviceCandidate,
        lease: SourceLease,
        event_sink: SourceEventSink,
    ) -> ConnectResult:
        """
拥有完整的 ADB、listener、reverse 与 first-frame 握手流程，任一步失败返回 FAILED。

依次：校验身份 → 确认 Quest → 清理旧应用/reverse → 启动 receiver → 注册
serial → adb reverse → 启动 App → 等待首帧（每 4s 重试启动）。失败路径不回收
receiver，由 close_connection/cleanup_idle_resources 负责。

Args:
    candidate: 已解析的设备候选。
    lease: 本次连接身份。
    event_sink: Host 的事件 sink，用于发布 transport/input 事件。

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
            """receiver 帧回调：连接仍有效时把 batch 交给 Host。"""
            if current(receiver):
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
            publish_transport(
                "ready",
                receiver.connection_generation,
                None,
                False,
                receiver,
            )

            def publish_device_status(items):
                """轮询线程回调：连接仍有效时发布头盔/手柄电量。"""
                if current(receiver):
                    event_sink.publish_telemetry(lease, receiver, items)

            self.start_device_status(lease, candidate.address, publish_device_status)
            return ConnectResult(ConnectOutcome.READY, "连接成功", False)
        except OSError as exc:
            return ConnectResult(ConnectOutcome.FAILED, f"Quest 接收端口启动失败: {exc}", False)
        except Exception as exc:
            logging.exception("[QuestInputModule] connect failed")
            return ConnectResult(ConnectOutcome.FAILED, f"Quest 接收器初始化失败: {exc}", False)

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
        )
        receiver_holder["receiver"] = receiver
        with self.lifecycle_lock:
            stale_lease = self.lease != lease or self._shutdown
            previous = None if stale_lease else self.receiver
            if not stale_lease:
                self.receiver = receiver
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
        """把一帧原始 payload 解析为规范化的左右手字典（由 parser 实现）。"""
        return parse_quest_frame(payload)

    def _publish_receiver_payload(
        self,
        lease: SourceLease,
        payload: dict,
        receiver_generation: int,
        receiver: QuestTcpReceiver,
        frame_callback: FrameCallback,
    ) -> None:
        """receiver 帧回调入口：复核连接仍有效后在锁外送给 Host。

        锁内只校验 lease/receiver/shutdown 并构造冻结 batch，避免把 Host 回调放在
        lifecycle_lock 里执行。
        """
        with self.lifecycle_lock:
            if self.lease != lease or self.receiver is not receiver or self._shutdown:
                return
            batch = self._build_input_batch(lease, payload, receiver_generation)
        frame_callback(batch, receiver)

    def _build_input_batch(
        self,
        lease: SourceLease,
        payload: dict,
        receiver_generation: int,
    ) -> RawInputBatch:
        """解析并把一帧严格的 Quest 数据冻结成 batch，再交给 host。"""
        parsed = self.parse_frame(payload)
        updates = []
        for hand in ("left", "right"):
            parsed_hand = parsed[hand]
            pose_data = parsed_hand["pose"]
            button_data = parsed_hand["button"]
            pose = PoseSample(
                x=pose_data["x"], y=pose_data["y"], z=pose_data["z"],
                qx=pose_data["qx"], qy=pose_data["qy"], qz=pose_data["qz"],
                qw=pose_data["qw"], timestamp=pose_data["timestamp"],
                confidence=pose_data["confidence"],
                linear_velocity=(pose_data["linear_velocity_x"], pose_data["linear_velocity_y"], pose_data["linear_velocity_z"]),
                angular_velocity=(pose_data["angular_velocity_x"], pose_data["angular_velocity_y"], pose_data["angular_velocity_z"]),
                acceleration=(pose_data["acceleration_x"], pose_data["acceleration_y"], pose_data["acceleration_z"]),
            )
            button = ButtonSample(
                trigger=button_data["trigger"], grip=button_data["grip"],
                btn_xa=button_data["btn_xa"], btn_yb=button_data["btn_yb"],
                btn_menu=button_data["btn_menu"], btn_system=button_data["btn_system"],
                menu_capture=button_data["menu_capture"],
                joystick_x=button_data["joystick_x"], joystick_y=button_data["joystick_y"],
                joystick_z=button_data["joystick_z"], reserve=button_data.get("reserve", 0),
                timestamp=button_data["timestamp"],
            )
            updates.append(HandInputUpdate(
                hand=hand,
                update_pose=True,
                pose=pose,
                pose_valid=parsed_hand["pose_valid"],
                update_button=True,
                button=button,
            ))
        return RawInputBatch(
            lease=lease,
            transport_generation=receiver_generation,
            batch_sequence=parsed["frame_index"],
            atomic_batch=True,
            hands=tuple(updates),
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

    def begin_reconnect_grace(
        self,
        lease: SourceLease,
        evaluate_recovery: Callable[[], Tuple[bool, bool]],
        expire: Callable[[int], None],
    ) -> bool:
        """
进入重连宽限期：设置截止时间（只首次设置）并安排到期复核回调。

Args:
    lease: 本次连接身份。
    evaluate_recovery: 返回 (会话已陈旧, 传输已恢复) 的复核函数。
    expire: 宽限期真正过期时的回调，入参为 lease.input_generation。

Returns:
    已进入宽限期为 True；lease 不匹配或已 shutdown 时为 False。
"""
        with self.lifecycle_lock:
            if self.lease != lease or self._shutdown:
                return False
        self.ensure_reconnect_deadline(time.monotonic(), QUEST_EXIT_TIMEOUT)
        self.schedule_reconnect_cleanup(
            lease.input_generation,
            evaluate_recovery,
            expire,
        )
        return True

    def owned_controllers(self):
        """返回 policy 当前持有的全部 controller。"""
        return self.policy.owned_controllers()

    @property
    def control_policy(self):
        """Host 使用的 control policy 实例（单例、串行处理输入）。"""
        return self.policy

    def controller_for_hand(self, hand: str):
        """代理到 policy：返回某手绑定的 controller。"""
        return self.policy.controller_for_hand(hand)

    def controller_matches(self, hand: str, controller: object) -> bool:
        """代理到 policy：按对象身份判断某手的绑定是否与给定 controller 一致。"""
        return self.policy.controller_matches(hand, controller)

    def bind_controller(self, hand, controller, session_id, params_epoch):
        """代理到 policy：绑定 controller 并返回被替换的旧 controller。"""
        return self.policy.bind_controller(hand, controller, session_id, params_epoch)

    def release_controller(self, hand, controller, params_epoch):
        """代理到 policy：解绑 controller；False 表示当前绑定与传入 controller 不符。"""
        return self.policy.release_controller(hand, controller, params_epoch)

    def on_mapping_change(self, reason, active_hands, session_id, params_epoch):
        """代理到 policy：映射变化时为已绑定的手请求 rebase。"""
        return self.policy.request_mapping_change(reason, active_hands, session_id, params_epoch)

    def mark_input_disconnected(self):
        """代理到 policy：断链时清空 pin/token 并返回受影响 controller。"""
        return self.policy.mark_disconnected()

    def reset_policy(self) -> None:
        """代理到 policy：重置全部 jump/pin/token 状态。"""
        self.policy.reset()

    def configure_pose_jump(self, config, error: Optional[str] = None) -> None:
        """记录 pose jump 阈值或配置错误（error 非 None 时后续帧一律被拒绝）。"""
        self.pose_jump_config = config
        self.pose_jump_error = error

    def reconnect_deadline(self) -> Optional[float]:
        """返回重连宽限期截止时间（monotonic）；未进入宽限期时为 None。"""
        with self.reconnect_timer_lock:
            return self.reconnect_deadline_monotonic

    def ensure_reconnect_deadline(self, now: float, timeout: float) -> float:
        """首次调用时设置截止时间为 now + timeout；已存在则原值不变。"""
        with self.reconnect_timer_lock:
            if self.reconnect_deadline_monotonic is None:
                self.reconnect_deadline_monotonic = now + timeout
            return self.reconnect_deadline_monotonic

    def cancel_reconnect_timer(self, clear_deadline: bool = False) -> None:
        """取消重连定时器；clear_deadline 为 True 时同时清除截止时间。"""
        with self.reconnect_timer_lock:
            timer = self.reconnect_timer
            self.reconnect_timer = None
            if clear_deadline:
                self.reconnect_deadline_monotonic = None
        if timer is not None:
            timer.cancel()

    def schedule_reconnect_cleanup(
        self,
        expected_input_generation: int,
        evaluate_recovery: Callable[[], Tuple[bool, bool]],
        expire: Callable[[int], None],
    ) -> None:
        """
按当前截止时间安排一次重连清理定时器（覆盖旧定时器，单次生效）。

回调先复核会话/传输状态，只有真正过期才调用 ``expire``。

Args:
    expected_input_generation: 期望触发过期时的 input_generation。
    evaluate_recovery: 返回 (会话已陈旧, 传输已恢复) 的复核函数。
    expire: 宽限期真正过期时的回调。
"""
        with self.reconnect_timer_lock:
            deadline = self.reconnect_deadline_monotonic
            if deadline is None:
                return
            delay = max(0.0, deadline - time.monotonic())
            previous_timer = self.reconnect_timer

            def callback() -> None:
                """到期回调：双重校验定时器身份/截止时间，再复核会话与传输状态。"""
                with self.reconnect_timer_lock:
                    if (
                        self.reconnect_timer is not timer
                        or self.reconnect_deadline_monotonic != deadline
                    ):
                        return
                stale_session, transport_recovered = evaluate_recovery()
                with self.reconnect_timer_lock:
                    if (
                        self.reconnect_timer is not timer
                        or self.reconnect_deadline_monotonic != deadline
                    ):
                        return
                    self.reconnect_timer = None
                    if stale_session:
                        return
                    if transport_recovered:
                        self.reconnect_deadline_monotonic = None
                        return
                expire(expected_input_generation)

            timer = threading.Timer(delay, callback)
            timer.daemon = True
            self.reconnect_timer = timer
        if previous_timer is not None:
            previous_timer.cancel()
        timer.start()

    def _detach_connection(
        self,
        lease: SourceLease,
        reason: str,
    ) -> Optional[_QuestConnectionSnapshot]:
        """
从实例上摘下 lease/receiver/serial/定时器并冻结为快照（资源清理在锁外进行）。

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
            if self._device_status_poller is not None:
                # 只置位不等待；迟到的发布会因 lease 已摘除被身份复核丢弃。
                self._device_status_poller.stop()
                self._device_status_poller = None
            with self.reconnect_timer_lock:
                reconnect_timer = self.reconnect_timer
                self.reconnect_timer = None
                self.reconnect_deadline_monotonic = None
            return _QuestConnectionSnapshot(
                lease=lease,
                receiver=receiver,
                serial=serial,
                reconnect_timer=reconnect_timer,
                reason=reason,
            )

    @staticmethod
    def _cleanup_snapshot(snapshot: _QuestConnectionSnapshot) -> bool:
        """
执行一次快照的资源清理（取消定时器 → session_end → 停表 → ADB 清理）。

Returns:
    全部资源已确认释放为 True；仍有残留为 False（调用方放进 ``_pending_cleanup`` 重试）。
"""
        if snapshot.reconnect_timer is not None:
            snapshot.reconnect_timer.cancel()
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
        success = self.drain_pending_cleanup() and self.current_lease() is None
        if success:
            self.policy.reset()
            with self.lifecycle_lock:
                self._shutdown = True
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
            ),
        )
