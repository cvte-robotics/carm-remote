"""Quest input adaptation resource owner."""

import logging
from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional, Tuple

from carm_teleop_input_contracts import (
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
from .parser import parse_quest_frame
from .policy import QuestFrameContext, QuestInputPolicy
from .receiver import QuestTcpReceiver


FrameCallback = Callable[[RawInputBatch, object], None]
DisconnectCallback = Callable[[str, int, int, QuestTcpReceiver], None]
StatusCallback = Callable[[str, int, int, QuestTcpReceiver], None]


@dataclass(frozen=True)
class _QuestConnectionSnapshot:
    lease: SourceLease
    receiver: Optional[QuestTcpReceiver]
    serial: Optional[str]
    reconnect_timer: Optional[threading.Timer]
    reason: str


class QuestInputModule:
    module_id = "quest"

    def __init__(self) -> None:
        self.policy = QuestInputPolicy()
        self.receiver: Optional[QuestTcpReceiver] = None
        self.serial: Optional[str] = None
        self.lease: Optional[SourceLease] = None
        self.reconnect_deadline_monotonic: Optional[float] = None
        self.reconnect_timer: Optional[threading.Timer] = None
        self.reconnect_timer_lock = threading.Lock()
        self.lifecycle_lock = threading.RLock()
        self.cleanup_lock = threading.Lock()
        self._pending_cleanup = []
        self.pose_jump_config = None
        self.pose_jump_error: Optional[str] = None
        self.frame_count = 0
        self.frame_window_start: Optional[float] = None
        self._shutdown = False

    @staticmethod
    def availability() -> SourceAvailability:
        return SourceAvailability(
            available=True,
            reason=None,
            capabilities=SourceCapabilities(
                provides_pose=True,
                provides_buttons=True,
                can_control_arms=True,
                connect_only=False,
                supports_reconnect_grace=True,
            ),
        )

    @staticmethod
    def scan() -> Tuple[DeviceCandidate, ...]:
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
        input_generation: int,
        control_generation: int,
    ):
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
            input_generation,
            control_generation,
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
        """Own the complete ADB, listener, reverse and first-frame handshake."""
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
            try:
                return bool(event_sink.connection_is_current(lease, receiver))
            except Exception:
                return False

        def publish_transport(phase, generation, reason=None, terminal=False, receiver=None):
            if not current(receiver):
                return
            event_sink.publish_transport(
                TransportEvent(lease, generation, phase, reason, terminal),
                receiver,
            )

        holder = {}

        def on_frame(batch, receiver):
            if current(receiver):
                event_sink.publish_input(batch, receiver)

        def on_disconnect(reason, input_generation, receiver_generation, receiver):
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
            publish_transport(
                status.lower(),
                receiver_generation,
                getattr(receiver, "last_error", None),
                status.lower() == "listenerfailed",
                receiver,
            )

        try:
            receiver = self.ensure_receiver(lease, on_frame, on_disconnect, on_status)
            holder["receiver"] = receiver
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
            next_launch = started
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
                if receiver.protocol_state != "ListenerFailed" and receiver._running:
                    return ConnectResult(
                        ConnectOutcome.PENDING,
                        "等待 Quest 首帧数据超时，继续等待连接",
                        True,
                    )
                return ConnectResult(
                    ConnectOutcome.FAILED,
                    "等待 Quest 首帧数据超时",
                    False,
                )
            if not current(receiver) or not receiver.connection_negotiated:
                return ConnectResult(
                    ConnectOutcome.PENDING,
                    "Quest 首帧已过期，继续等待当前连接",
                    True,
                )
            publish_transport(
                "ready",
                receiver.connection_generation,
                None,
                False,
                receiver,
            )
            return ConnectResult(ConnectOutcome.READY, "连接成功", False)
        except OSError as exc:
            return ConnectResult(ConnectOutcome.FAILED, f"Quest 接收端口启动失败: {exc}", False)
        except Exception as exc:
            logging.exception("[QuestInputModule] connect failed")
            return ConnectResult(ConnectOutcome.FAILED, f"Quest 接收器初始化失败: {exc}", False)

    def bind_lease(self, lease: SourceLease) -> None:
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

    def current_lease(self) -> Optional[SourceLease]:
        with self.lifecycle_lock:
            if self.lease is not None:
                return self.lease
            if self._pending_cleanup:
                return self._pending_cleanup[0].lease
            return None

    def callback_matches(self, lease: SourceLease, receiver) -> bool:
        with self.lifecycle_lock:
            return bool(
                self.lease == lease
                and self.receiver is receiver
                and not self._shutdown
            )

    def configure(self, config) -> None:
        try:
            self.configure_pose_jump(config.resolve_quest_pose_jump())
        except Exception as exc:
            self.configure_pose_jump(None, str(exc))
            logging.error(
                "[QuestInputModule] Frames will be rejected by the config gate: %s",
                exc,
            )

    @staticmethod
    def resolution_error(address: str, name: str) -> Optional[str]:
        return None

    def ensure_receiver(
        self,
        lease: SourceLease,
        frame_callback: FrameCallback,
        disconnect_callback: DisconnectCallback,
        status_callback: StatusCallback,
    ) -> QuestTcpReceiver:
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
        with self.lifecycle_lock:
            if self.lease != lease or self.receiver is None:
                raise RuntimeError("Quest receiver is not configured for this lease")
            self.receiver.start()
            return self.receiver

    def parse_frame(self, payload: dict) -> dict:
        return parse_quest_frame(payload)

    def _publish_receiver_payload(
        self,
        lease: SourceLease,
        payload: dict,
        receiver_generation: int,
        receiver: QuestTcpReceiver,
        frame_callback: FrameCallback,
    ) -> None:
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
        """Parse and freeze one strict Quest frame before it reaches the host."""
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
        return get_adb_device_info(serial)

    @staticmethod
    def is_quest_device(device_info) -> bool:
        return bool(device_info and is_quest_adb_device(device_info))

    def register_serial(self, lease: SourceLease, serial: str) -> bool:
        with self.lifecycle_lock:
            if self.lease != lease or self._shutdown:
                return False
            self.serial = serial
            return True

    @staticmethod
    def setup_reverse(serial: str):
        return run_adb(
            ["-s", serial, "reverse", f"tcp:{QUEST_TCP_PORT}", f"tcp:{QUEST_TCP_PORT}"],
            timeout=8.0,
        )

    @staticmethod
    def launch(serial: str) -> bool:
        return launch_quest_app(serial)

    def begin_reconnect_grace(
        self,
        lease: SourceLease,
        evaluate_recovery: Callable[[], Tuple[bool, bool]],
        expire: Callable[[int], None],
    ) -> bool:
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
        return self.policy.owned_controllers()

    @property
    def control_policy(self):
        return self.policy

    def controller_for_hand(self, hand: str):
        return self.policy.controller_for_hand(hand)

    def controller_matches(self, hand: str, controller: object) -> bool:
        return self.policy.controller_matches(hand, controller)

    def bind_controller(self, hand, controller, controller_generation, input_generation, control_generation):
        return self.policy.bind_controller(hand, controller, controller_generation, input_generation, control_generation)

    def release_controller(self, hand, controller, control_generation):
        return self.policy.release_controller(hand, controller, control_generation)

    def on_mapping_change(self, reason, active_hands, input_generation, control_generation):
        return self.policy.request_mapping_change(reason, active_hands, input_generation, control_generation)

    def mark_input_disconnected(self):
        return self.policy.mark_disconnected()

    def reset_policy(self) -> None:
        self.policy.reset()

    def configure_pose_jump(self, config, error: Optional[str] = None) -> None:
        self.pose_jump_config = config
        self.pose_jump_error = error

    def record_frame(self, now: float):
        if self.frame_window_start is None:
            self.frame_window_start = now
            self.frame_count = 0
        self.frame_count += 1
        elapsed = now - self.frame_window_start
        if elapsed < 1.0:
            return None
        report = (self.frame_count / elapsed, self.frame_count, elapsed)
        self.frame_window_start = now
        self.frame_count = 0
        return report

    def reconnect_deadline(self) -> Optional[float]:
        with self.reconnect_timer_lock:
            return self.reconnect_deadline_monotonic

    def ensure_reconnect_deadline(self, now: float, timeout: float) -> float:
        with self.reconnect_timer_lock:
            if self.reconnect_deadline_monotonic is None:
                self.reconnect_deadline_monotonic = now + timeout
            return self.reconnect_deadline_monotonic

    def cancel_reconnect_timer(self, clear_deadline: bool = False) -> None:
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
        with self.reconnect_timer_lock:
            deadline = self.reconnect_deadline_monotonic
            if deadline is None:
                return
            delay = max(0.0, deadline - time.monotonic())
            previous_timer = self.reconnect_timer

            def callback() -> None:
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
        with self.lifecycle_lock:
            if self.lease != lease:
                return None
            receiver = self.receiver
            serial = self.serial
            self.receiver = None
            self.serial = None
            self.lease = None
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
        if snapshot.reconnect_timer is not None:
            snapshot.reconnect_timer.cancel()
        success = True
        receiver = snapshot.receiver
        serial = snapshot.serial
        if receiver is not None and receiver._connected:
            if not receiver.send_session_end(snapshot.reason, True):
                # Session-end is advisory; transport/process teardown remains authoritative.
                logging.warning("[QuestInputModule] Failed to send best-effort Quest session_end")

        if serial:
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
                    # The cleanup postcondition is already satisfied when ADB has no listener.
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

        if receiver is not None and receiver.stop() is False:
            success = False
        return success

    def defer_connection_cleanup(self, lease: SourceLease, reason: str) -> bool:
        with self.lifecycle_lock:
            snapshot = self._detach_connection(lease, reason)
            if snapshot is None:
                return False
            self._pending_cleanup.append(snapshot)
            return True

    def has_pending_cleanup(self) -> bool:
        with self.lifecycle_lock:
            return bool(self._pending_cleanup)

    def drain_pending_cleanup(self) -> bool:
        with self.cleanup_lock:
            return self._drain_pending_cleanup()

    def _drain_pending_cleanup(self) -> bool:
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
        with self.cleanup_lock:
            return self._close_connection(lease, reason)

    def _close_connection(
        self,
        lease: SourceLease,
        reason: str = "backend_stop",
    ) -> bool:
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
        return self.drain_pending_cleanup()

    def shutdown(self, reason: str) -> bool:
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
