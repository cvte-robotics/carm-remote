"""D3 单臂臂型角控制运行时。

该模块只拥有臂型角的输入状态机，不开线程：由所属臂的控制线程每个周期调用
``tick()``，按 ``STEP_INTERVAL_S`` 节拍步进。所有 SDK 调用只发不等；需要实际臂型角
时（开始调节、松键对齐）登记请求后由后续周期取回复，不阻塞控制线程。普通手臂控制、
腰控制、会话清理仍由各自的 owner 负责；调用方通过 ``read_input`` 和 ``publish`` 两个
窄接口接入。实际臂型角只用于控制，不再对外显示（``angle_deg``/``target_deg`` 恒为 null）。
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Callable, Optional

from quest.module import QuestInputModule

from ..runtime.config import ArmAngleControlConfig
from ..sources.registry import BindingPhase


# 左臂与右臂的臂型角正方向相反：左手柄右推时左臂臂型角减小（向外）。
ARM_ANGLE_DIRECTION = {"left": -1.0, "right": 1.0}

INPUT_TIMEOUT_S = 0.2
HOME_PAUSE_S = 5.0
FEEDBACK_RETRY_S = 0.5
MAX_STEP_S = 0.1
# 步进节拍：控制线程按输入帧运行，臂型角仍按 20 Hz 积分与下发，避免放大控制器负载。
STEP_INTERVAL_S = 0.05
READ_TIMEOUT_S = 1.0


def arm_angle_state(status: str, error: Optional[str] = None) -> dict:
    """单侧 ``arm_angle_state`` 记录；实际臂型角不对外显示，角度字段恒为 null。"""
    return {"status": status, "angle_deg": None, "target_deg": None, "error": error}


@dataclass(frozen=True)
class ArmAngleInput:
    """当前 lease 下单手臂型角输入的最小快照。"""

    x: float
    received_monotonic: float
    lease: object
    transport_generation: object
    home_sequence: int
    motion_engaged: bool
    # 手臂最近是否真的发出了 track_pose。控制器只在跟随时执行臂型角目标，
    # 运动键按住但未跟随（追踪丢失、回位中、等待对齐）时不能下发。
    tracking: bool = True


class ArmAngleRuntime:
    """单臂臂型角状态机；只由所属臂的控制线程步进。"""

    def __init__(
        self,
        role: str,
        hand: str,
        carm,
        config: ArmAngleControlConfig,
        read_input: Callable[[], Optional[ArmAngleInput]],
        publish: Callable[[str, dict], None],
        clock: Callable[[], float] = time.monotonic,
    ):
        self.role = role
        self.hand = hand
        self._direction = ARM_ANGLE_DIRECTION.get(hand, 1.0)
        self.carm = carm
        self.config = config
        self.read_input = read_input
        self.publish = publish
        self.clock = clock

        self.status = "disabled" if not config.enabled else "waiting_center"
        self.error: Optional[str] = None
        self.target_deg: Optional[float] = None
        self._last_published = None
        self._expected_ws = None
        self._lease = None
        self._transport_generation = None
        self._home_sequence = None
        self._last_tick = None
        self._last_step = -math.inf
        self._last_feedback_retry = -math.inf
        self._pause_until = 0.0
        self._last_command_target = None
        # 控制器里可能留有未走完的目标，退出跟随后要对齐到实际值。
        self._needs_align = False
        # 在途的实际臂型角读取：("start" | "align", PendingReply)。
        self._pending = None

    def _snapshot(self) -> dict:
        return arm_angle_state(self.status, self.error if self.status == "fault" else None)

    def _publish(self, force: bool = False) -> None:
        snapshot = self._snapshot()
        if force or snapshot != self._last_published:
            self._last_published = snapshot
            self.publish(self.hand, snapshot)

    def _set_status(self, status: str, error: Optional[str] = None, force: bool = False) -> None:
        self.status = status
        self.error = error
        self._publish(force=force)

    def _connection_is_current(self) -> bool:
        try:
            if not self.carm.is_connected():
                return False
            ws = getattr(self.carm, "ws", None)
        except Exception:
            return False
        if ws is None:
            return False
        if self._expected_ws is None:
            self._expected_ws = ws
        return ws is self._expected_ws

    def _request_angle(self, purpose: str) -> bool:
        """登记一次实际臂型角读取；回复在后续周期由 ``_poll_angle`` 取回。"""
        if not self._connection_is_current():
            return False
        try:
            pending = self.carm.request_nowait(
                {"command": "getArmAngle", "arm_index": self.carm.arm_index},
                timeout=READ_TIMEOUT_S,
            )
        except Exception as exc:
            logging.warning("[ArmAngleRuntime] get failed role=%s: %s", self.role, exc)
            return False
        if pending is None:
            return False
        self._pending = (purpose, pending)
        return True

    def _poll_angle(self, now: float):
        """返回 (已完成, 实际臂型角度数或 None)；回复格式与 SDK ``get_arm_angle`` 一致。"""
        purpose, pending = self._pending
        result = pending.poll(now)
        if result is None:
            return False, None
        self._pending = None
        value = result.get("arm_angle")
        if (result.get("recv") != "Task_Recieve" or isinstance(value, bool)
                or not isinstance(value, (int, float)) or not math.isfinite(value)):
            return True, None
        return True, math.degrees(float(value))

    def _cancel_start_read(self) -> None:
        if self._pending is not None and self._pending[0] == "start":
            self._pending[1].cancel()
            self._pending = None

    def _send_angle(self, target_deg: float) -> bool:
        """下发臂型角目标，只发不等回复；返回值只表示已写入连接。"""
        if not self._connection_is_current():
            return False
        try:
            return bool(self.carm.send_only({
                "command": "setArmAngle",
                "arm_index": self.carm.arm_index,
                "arm_angle": math.radians(target_deg),
            }))
        except Exception as exc:
            logging.warning("[ArmAngleRuntime] send failed role=%s: %s", self.role, exc)
            return False

    def start(self) -> None:
        """激活状态机；须在所属臂控制线程启动前调用。"""
        if not self.config.enabled:
            return
        self._expected_ws = getattr(self.carm, "ws", None)
        self._set_status("waiting_center", force=True)

    def stop(self) -> None:
        """所属臂控制线程退出后调用：放弃在途读取并发布停止。"""
        if self._pending is not None:
            self._pending[1].cancel()
            self._pending = None
        if self.config.enabled and self.status != "stopped":
            self._set_status("stopped")

    def tick(self) -> None:
        """控制线程每周期调用；按节拍步进，异常只让臂型角进入 fault，不影响手臂控制。"""
        now = self.clock()
        if now - self._last_step < STEP_INTERVAL_S:
            return
        self._last_step = now
        try:
            self.step(now)
        except Exception as exc:
            logging.exception("[ArmAngleRuntime] unexpected failure role=%s", self.role)
            self._set_status("fault", str(exc))

    def _valid_input(self, value: Optional[ArmAngleInput], now: float) -> bool:
        if value is None or now - value.received_monotonic > INPUT_TIMEOUT_S:
            return False
        if self._lease is None:
            self._lease = value.lease
            self._transport_generation = value.transport_generation
            self._home_sequence = value.home_sequence
            return True
        if value.lease != self._lease or value.transport_generation != self._transport_generation:
            self._lease = value.lease
            self._transport_generation = value.transport_generation
            self._home_sequence = value.home_sequence
            self._last_tick = None
            return False
        return True

    def _interrupt_for_input(self, status: str = "waiting_input") -> None:
        self.target_deg = None
        self._last_command_target = None
        self._cancel_start_read()
        self._set_status(status)
        # 失败多半是连接失配，保留标记，恢复后由后续周期重试。
        self._request_align()

    def _request_align(self) -> bool:
        """有未对齐的目标时登记一次读取，回复到达后（``_finish_align``）只发不等地写回。

        控制器只在跟随 ``track_pose`` 时执行臂型角目标；不对齐的话，未走完的目标
        会在下次跟随时一次性补走。

        Returns: 已登记或无需对齐为 True；发送失败为 False（标记保留）。
        """
        if not self._needs_align or self._pending is not None:
            return True
        if not self._request_angle("align"):
            return False
        self._needs_align = False
        return True

    def _align_or_fault(self) -> bool:
        """退出跟随时对齐；失败进入 fault，回中后恢复，不反复重试。"""
        if self._request_align():
            return True
        self._needs_align = False
        self._set_status("fault", "arm angle sync failed")
        return False

    def _stop_adjusting(self) -> None:
        """运动键松开或手臂不再跟随：结束调节并对齐控制器暂存的目标。"""
        self.target_deg = None
        self._last_command_target = None
        self._cancel_start_read()
        if self._align_or_fault():
            self._set_status("idle")

    def _finish_align(self, angle: Optional[float]) -> None:
        if angle is None or not self._send_angle(angle):
            self._set_status("fault", "arm angle sync failed")

    def _is_centered(self, x: float) -> bool:
        return abs(x) <= self.config.deadzone

    def _start_adjusting(self, now: float) -> None:
        if self._pending is not None:
            return
        if (self.status == "waiting_feedback"
                and now - self._last_feedback_retry < FEEDBACK_RETRY_S):
            return
        self._last_feedback_retry = now
        self._request_angle("start")
        self._set_status("waiting_feedback")

    def _finish_start(self, now: float, angle: Optional[float]) -> None:
        if angle is None:
            return
        self.target_deg = angle
        self._last_command_target = angle
        self._last_tick = now
        self._needs_align = True
        self._set_status("adjusting")

    def _integrate(self, x: float, dt: float) -> None:
        magnitude = (abs(x) - self.config.deadzone) / (1.0 - self.config.deadzone)
        delta = (math.copysign(magnitude * self.config.max_speed_deg_s * dt, x)
                 * self._direction)
        current = self.target_deg
        if current < self.config.min_deg:
            if delta < 0:
                delta = 0.0
            target = min(self.config.max_deg, current + delta)
        elif current > self.config.max_deg:
            if delta > 0:
                delta = 0.0
            target = max(self.config.min_deg, current + delta)
        else:
            target = max(self.config.min_deg, min(self.config.max_deg, current + delta))
        self.target_deg = target
        if (self._last_command_target is None
                or abs(target - self._last_command_target) >= self.config.min_step_deg):
            if not self._send_angle(target):
                self._set_status("fault", "set_arm_angle send failed")
                self._last_command_target = None
                return
            self._last_command_target = target

    def step(self, now: float) -> None:
        """执行一次状态机步进；公开给无设备的确定性测试使用。"""
        if not self.config.enabled or self.status == "stopped":
            return
        # 在途读取先于输入判定处理：松键对齐不依赖输入是否仍有效。
        if self._pending is not None:
            purpose = self._pending[0]
            done, angle = self._poll_angle(now)
            if done and purpose == "align":
                self._finish_align(angle)
            elif done and self.status == "waiting_feedback":
                self._finish_start(now, angle)
        value = self.read_input()
        if not self._valid_input(value, now):
            if self.status not in {"waiting_center", "paused_home"}:
                self._interrupt_for_input()
            return

        if not self._connection_is_current():
            try:
                if self.carm.is_connected():
                    self._expected_ws = getattr(self.carm, "ws", None)
            except Exception:
                pass
            self._interrupt_for_input()
            return

        if value.home_sequence != self._home_sequence:
            self._home_sequence = value.home_sequence
            if not value.motion_engaged:
                self.target_deg = None
                self._cancel_start_read()
                self._request_align()
                self._pause_until = now + HOME_PAUSE_S
                self._set_status("paused_home")
                return

        if self.status == "paused_home":
            if now < self._pause_until:
                return
            self.target_deg = None
            self._set_status("waiting_center")
            return

        # 臂型角是遥操副任务：只在运动键开启且手臂确实在跟随时调节。
        adjustable = value.motion_engaged and value.tracking
        if not adjustable:
            if self.status in {"adjusting", "waiting_feedback"}:
                self._stop_adjusting()
                self._last_tick = now
                return
            # 回中后才松键时状态已是 idle，同样要对齐。
            if not self._align_or_fault():
                return

        if self._is_centered(value.x):
            if self.status in {"waiting_center", "waiting_input", "waiting_feedback", "fault"}:
                self._cancel_start_read()
                self.error = None
                self._set_status("idle")
            elif self.status == "adjusting":
                self.target_deg = None
                self._set_status("idle")
            self._last_tick = now
            return

        if self.status in {"waiting_center", "waiting_input", "fault"}:
            return
        if not adjustable:
            return
        if self.status in {"idle", "waiting_feedback"}:
            self._start_adjusting(now)
            return
        if self.status != "adjusting":
            return

        previous = self._last_tick if self._last_tick is not None else now
        dt = max(0.0, min(MAX_STEP_S, now - previous))
        self._last_tick = now
        self._integrate(value.x, dt)


def create_arm_angle_runtimes(owner, runtimes):
    """双臂连接后按顺序判定是否启用臂型角；任一条件失败时双臂都不调节。

    条件依次为：同板 D3 双臂 7 轴 Quest 输入、板端 SDK 同时提供读写接口、
    本连接 ``arm_angle_ready``、配置启用。不满足时只发布原因，遥操照常。
    """
    def unavailable(status):
        """左右臂同时记入同一未启用原因。"""
        owner._set_arm_angle_unavailable(status)
        return None

    if len(runtimes) != 2 or any(runtime.controller.arm_model != "D3" for runtime in runtimes):
        return unavailable("unavailable")
    if len({runtime.control_host for runtime in runtimes}) != 1:
        return unavailable("unavailable")
    with owner._data_lock:
        binding = owner.active_source_binding
        module = owner.source_registry.active_module()
    if binding is None or not isinstance(module, QuestInputModule):
        return unavailable("unavailable")
    lease = binding.lease
    if dict(module.status(lease).details).get("arm_angle_ready") is not True:
        return unavailable("legacy_input")
    for runtime in runtimes:
        carm = getattr(runtime.controller, "carm_", None)
        if (carm is None or not callable(getattr(carm, "get_arm_angle", None))
                or not callable(getattr(carm, "set_arm_angle", None))):
            return unavailable("unavailable")
    config = owner.joystick_config.resolve_arm_angle()
    if not config.enabled:
        return unavailable("disabled")

    def make_read_input(hand, controller):
        """只返回当前 lease 下该手最新发布的臂型角摇杆。"""
        def read_input():
            current = owner.active_source_binding
            if (current is None or current.lease != lease
                    or current.phase is not BindingPhase.ACTIVE
                    or not current.input_ready or not owner.state.ble_connected):
                return None
            publication = owner._input_publication.load()
            if publication is None or publication.lease != lease:
                return None
            frame = publication.frame(hand)
            if frame is None or frame.intent is None:
                return None
            return ArmAngleInput(
                x=float(frame.intent.arm_angle_stick[0]),
                received_monotonic=publication.timestamp_monotonic,
                lease=publication.lease,
                transport_generation=publication.transport_generation,
                home_sequence=frame.intent.home_sequence,
                motion_engaged=frame.intent.motion_engaged,
                tracking=controller.pose_dispatched_within(INPUT_TIMEOUT_S),
            )
        return read_input

    for runtime in runtimes:
        runtime.arm_angle = ArmAngleRuntime(
            role=runtime.role,
            hand=runtime.hand,
            carm=runtime.controller.carm_,
            config=config,
            read_input=make_read_input(runtime.hand, runtime.controller),
            publish=owner._set_arm_angle_state,
        )
    return runtimes
