"""Host 主导的单臂控制事务（每臂一个控制环，不取 Host data 锁）。

每周期把输入帧变成一次控制计算与 SDK 派发（关节路径优先，否则位姿路径），并维护
按接受时刻身份钉扎的反馈缓存。调度：binding 声明 ``input_clocked_control`` 时新帧
发布即唤醒（最长 1/72s 超时兜底）；否则按 72Hz 绝对时间节拍运行。映射只由本臂
控制线程写入，data 锁只用于再接入观测和终止凭证。
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Tuple

from carm_teleop_input_contracts import (
    ControlAction,
    ControlContext,
    ReengageStatus,
    HAND_RIGHT,
    normalize_hand,
)

from ..core.command_controller import ControlSample
from ..sources.registry import BindingPhase
from .control_stats import ControlLoopStats


_CONTROL_LOOP_HZ = 72.0
_CONTROL_LOOP_INTERVAL = 1.0 / _CONTROL_LOOP_HZ
_LIMIT_EPS = 1e-8
_INPUT_FRESH_S = 0.2
_DISPATCH_REJECT_LOG_INTERVAL_S = 1.0


class ControlStepOutcome(Enum):
    """单个控制周期结束时的状态投影（仅用于日志/统计，不参与任何判断）。"""

    NO_INPUT = "no_input"
    REBASE_APPLIED = "rebase_applied"
    COMPUTED = "computed"
    DISPATCH_COMMITTED = "dispatch_committed"
    DISPATCH_REJECTED = "dispatch_rejected"
    IDENTITY_INVALIDATED = "identity_invalidated"


class LoopTerminationKind(Enum):
    """控制环终止原因分类（主动停止 / 内部错误）。"""

    STOP_REQUESTED = "stop_requested"
    INTERNAL_ERROR = "internal_error"


@dataclass(frozen=True)
class LoopTermination:
    """控制环终止的冻结凭证（交给 Host 做收尾）。

    Host 通过 runtime、transaction、线程、session_id、source_lease 与
    controller_identity 判断终止是否仍属于当前会话。
    """

    kind: LoopTerminationKind
    reason: str
    runtime: Any
    transaction: Any
    session_id: Optional[int]
    source_lease: Any
    controller_identity: Any


@dataclass(frozen=True)
class JointControlInput:
    """本周期关节输入、反馈和臂连接快照，不参与派发事务复核。"""

    hand: str
    frame: Any
    target: Any
    received: float
    binding: Any
    lease: Any
    session_id: int
    params_epoch: int
    carm: Any
    ws: Any
    arm_connection_generation: Any
    init_joint: Tuple
    feedback_joints: Any
    feedback_age_s: Any
    now_monotonic: float
    module: Any


class ControlTransaction:
    """独占一个 ArmRuntime 的完整实时控制环（输入节拍或 72Hz 固定节拍，每臂一个实例）。

    反馈缓存额外按“接受该帧的连接对象身份”（``carm`` / ``ws``）钉扎：接受时刻的
    ws 身份不再成立时立即清空缓存，避免 SDK 重连后旧反馈被挂到新连接上。
    """

    def __init__(self, owner, runtime):
        self._owner = owner
        self._runtime = runtime
        self._controller = runtime.controller
        self._stop_event = threading.Event()
        self._feedback_joints = None
        self._feedback_stamp = None
        self._feedback_connection_generation = None
        self._feedback_sequence = None
        # 缓存反馈被接受时的臂连接对象。SDK 重连可以在不推进上面这些代次的情况下
        # 替换 carm_/ws，因此缓存同时按对象身份钉扎。
        self._feedback_carm = None
        self._feedback_ws = None
        self._joint_eff_missing_logged = False
        self._stats = ControlLoopStats(getattr(self._controller, "arm_role", "unknown"))
        # 本步 ``_capture_sample`` 读到的本手帧（统计派发延迟与跳帧用）。
        self._captured_frame = None
        self._next_deadline = None
        self._last_control_params = None
        self._cycle_control_params = None
        binding = owner.active_source_binding
        self._last_source_lease = None if binding is None else binding.lease
        self._handled_discontinuity_seq = 0
        self._mapping_started = False
        self._home_baseline_reset = False
        self._rebase_frame_sequence = None
        self._last_dispatched_sequence = None
        self._dispatch_rejects = 0
        self._dispatch_reject_logged = -math.inf

    def request_stop(self) -> None:
        """请求控制环退出（幂等）：置停止标志并唤醒可能在等待新帧的控制线程。"""
        self._stop_event.set()
        signal = getattr(self._owner, "_publication_signal", None)
        if signal is not None:
            signal.notify()

    def run(self) -> LoopTermination:
        """运行控制环直到收到停止请求或发生内部错误。

        Returns:
            LoopTermination：终止原因与终止时刻的身份快照；内部错误时先
            ``halt_control()`` 再回报给 Host。
        """
        controller = self._controller
        signal = getattr(self._owner, "_publication_signal", None)
        logging.info("[ControlTransaction] Loop started for role=%s", controller.arm_role)
        self._next_deadline = time.monotonic()
        while not self._stop_event.is_set():
            try:
                # 步前取发布序号：步内到达的新帧会让下一次等待立即返回。
                seen = signal.sequence if signal is not None else None
                started = time.monotonic()
                outcome = self._run_step()
                arm_angle = getattr(self._runtime, "arm_angle", None)
                if arm_angle is not None:
                    arm_angle.tick()
                finished = time.monotonic()
                self._stats.record_step(started, finished, outcome)
                self._stats.maybe_report(finished)
                self._wait_next_step(signal, seen, finished)
            except Exception as exc:
                logging.exception(
                    "[ControlTransaction] Loop failed for role=%s",
                    controller.arm_role,
                )
                controller.halt_control()
                invalidate = getattr(controller.end_effector_driver, "invalidate", None)
                if callable(invalidate):
                    invalidate()
                termination = self._termination(
                    LoopTerminationKind.INTERNAL_ERROR,
                    f"control-loop-error:{controller.arm_role}:{exc}",
                )
                self._owner._handle_control_termination(termination)
                return termination

        return self._termination(
            LoopTerminationKind.STOP_REQUESTED,
            f"control-loop-stop:{controller.arm_role}",
        )

    def _input_clocked(self) -> bool:
        """当前 binding 是否声明按输入节拍唤醒（原子引用读，无锁）。"""
        binding = getattr(self._owner, "active_source_binding", None)
        capabilities = getattr(binding, "capabilities", None)
        return bool(getattr(capabilities, "input_clocked_control", False))

    def _wait_next_step(self, signal, seen, now) -> None:
        """等待下一步的开始时刻。

        输入节拍：步内已有新发布则立即返回，否则等新发布，最长一个控制周期（保留
        反馈刷新、grip 释放与停止检查）。固定节拍：绝对时间 deadline 防止"执行后再
        sleep"的周期漂移；落后时不追赶。
        """
        if signal is not None and self._input_clocked():
            self._next_deadline = now + _CONTROL_LOOP_INTERVAL
            signal.wait_newer(seen, _CONTROL_LOOP_INTERVAL)
            return
        self._next_deadline += _CONTROL_LOOP_INTERVAL
        if self._next_deadline < now:
            self._next_deadline = now
        self._stop_event.wait(self._next_deadline - now)

    def _run_step(self) -> ControlStepOutcome:
        """执行一个控制周期：刷新反馈 → 关节路径优先 → 位姿路径。

        Returns:
            ControlStepOutcome：本周期结束状态（无输入/已计算/已提交/已拒绝/身份失效）
        """
        controller = self._controller
        if self._stop_event.is_set() or controller.isStop:
            return ControlStepOutcome.NO_INPUT
        self._begin_control_cycle()
        try:
            arm_feedback_fresh = controller._refresh_current_pose()
            self._observe_feedback(arm_feedback_fresh)
            attempt = self._capture_joint_input()
            if attempt is not None:
                self._stats.record_input(attempt.frame.batch_sequence)
                outcome = self._run_joint_command(attempt)
                if outcome is ControlStepOutcome.DISPATCH_COMMITTED:
                    self._stats.record_dispatch(
                        attempt.frame.batch_sequence, attempt.received, time.monotonic(),
                    )
                return outcome
            self._captured_frame = None
            outcome, sample = self._capture_sample(arm_feedback_fresh)
            if sample is None:
                return outcome
            if sample.pose_data is None or sample.intent_data is None:
                controller.discard_home_intent(
                    sample.intent_data, sample.home_identity, "incomplete_control_sample",
                )
                controller.reject_pose_target()
                return ControlStepOutcome.NO_INPUT
            computation = controller.process_input_sample(sample)
            if computation.joint_move is not None:
                self._dispatch_joint_move(computation.joint_move)
            if (
                controller.current_target is not None
                and controller.carm_
                and controller.carm_.is_connected()
            ):
                outcome = self._dispatch_pose()
                frame = self._captured_frame
                if outcome is ControlStepOutcome.DISPATCH_COMMITTED and frame is not None:
                    self._stats.record_dispatch(
                        frame.input_sequence, frame.timestamp_monotonic, time.monotonic(),
                    )
                return outcome
            return ControlStepOutcome.COMPUTED
        finally:
            self._cycle_control_params = None

    def _begin_control_cycle(self):
        """在控制线程内接收最新参数，并为参数变化失效一次映射。"""
        params = self._owner._control_params
        if self._last_control_params is None:
            self._last_control_params = params
        elif params is not self._last_control_params:
            self._controller.invalidate_pose_mapping()
            self._last_control_params = params
        binding = self._owner.active_source_binding
        lease = None if binding is None else binding.lease
        if lease != self._last_source_lease:
            self._controller.invalidate_pose_mapping()
            self._last_source_lease = lease
            self._handled_discontinuity_seq = None
            self._rebase_frame_sequence = None
            self._last_dispatched_sequence = None
        self._cycle_control_params = params

    def _control_params_for_cycle(self):
        """返回本周期冻结的参数；直接调用内部采样辅助时回退到当前发布值。"""
        if self._cycle_control_params is not None:
            return self._cycle_control_params
        return self._owner._control_params

    def _clear_feedback_cache(self):
        """清空反馈缓存（含连接对象身份钉扎字段）。"""
        self._feedback_joints = None
        self._feedback_stamp = None
        self._feedback_connection_generation = None
        self._feedback_sequence = None
        self._feedback_carm = None
        self._feedback_ws = None

    def _feedback_cache_matches(self, carm):
        """缓存反馈是否来自当前活连接对象（``carm`` 与 ``ws`` 双重身份）。"""
        return (
            self._feedback_carm is carm
            and self._feedback_ws is getattr(carm, "ws", None)
        )

    def _discard_mismatched_feedback_cache(self, carm):
        """丢弃被重连改指到另一个 ws 的缓存反馈（本来就无缓存时不做任何事）。"""
        if self._feedback_stamp is None and self._feedback_joints is None:
            return
        if not self._feedback_cache_matches(carm):
            self._clear_feedback_cache()

    def _current_feedback_age(self, controller=None):
        """返回缓存反馈的年龄；无缓存或缓存身份不匹配时返回 None。

        身份不匹配时顺带清空缓存（重连后旧缓存不可再用）。
        """
        if self._feedback_stamp is None:
            return None
        if (
            controller is not None
            and not self._feedback_cache_matches(controller.carm_)
        ):
            self._clear_feedback_cache()
            return None
        return time.monotonic() - self._feedback_stamp

    def _observe_feedback(self, refreshed):
        """
仅在"接受该帧时的连接身份"仍成立时缓存臂反馈（接受时刻身份快照由 controller 记录）。

Args:
    refreshed: ``_refresh_current_pose`` 本周期是否接受了新帧。
"""
        controller = self._controller
        generation = getattr(controller, "arm_connection_generation", None)
        feedback_generation = getattr(
            controller, "arm_feedback_connection_generation", None
        )
        if not refreshed:
            # 未接受新帧是两次接帧之间的常态：只有当连接代次已推进、或 controller
            # 不再把缓存的代次报告为已接受时，才作废缓存。
            if (
                self._feedback_connection_generation is not None
                and (
                    generation != self._feedback_connection_generation
                    or feedback_generation != generation
                )
            ):
                self._clear_feedback_cache()
            return
        carm = controller.carm_
        # 用 controller 记录的接受时刻快照判定身份与年龄，改读实时值会把旧反馈
        # 绑到重连后的新连接上。
        accepted_ws = getattr(controller, "arm_feedback_ws_identity", None)
        accepted_state = getattr(controller, "arm_feedback_state_identity", None)
        accepted_monotonic = getattr(controller, "arm_feedback_accept_monotonic", None)
        joints = None
        if carm is not None and accepted_state is not None:
            try:
                joints = tuple(accepted_state["arm"][carm.arm_index]["reality"]["pose"])
                if (
                    not joints
                    or len(joints) != getattr(carm, "arm_dof", None)
                    or any(isinstance(q, bool) or not math.isfinite(q) for q in joints)
                ):
                    # 关节值个数必须等于当前臂 DOF（兼容 6/7 轴）：长度不符即整帧
                    # 无效，否则锁存拒绝不生效，臂会在无效反馈后继续跟随。
                    logging.warning(
                        "[ControlTransaction] Rejected arm feedback: joints=%s dof=%s",
                        None if joints is None else len(joints),
                        getattr(carm, "arm_dof", None),
                    )
                    joints = None
            except (TypeError, KeyError, IndexError, AttributeError, ValueError):
                joints = None
        if (
            joints is None
            or feedback_generation != generation
            or carm is None
            or accepted_ws is None
            or accepted_monotonic is None
            or getattr(carm, "ws", None) is not accepted_ws
        ):
            # 要么该已接受帧无法缓存，要么连接在接受之后发生了漂移：两种情况下旧
            # 条目都已过期。
            self._clear_feedback_cache()
            return
        self._feedback_joints = joints
        self._feedback_stamp = accepted_monotonic
        self._feedback_connection_generation = generation
        self._feedback_sequence = getattr(controller, "arm_feedback_state_sequence", None)
        self._feedback_carm = carm
        self._feedback_ws = accepted_ws

    def _capture_joint_input(self):
        """
读取本周期关节输入和反馈快照；控制环不取 Host data 锁。

Returns:
    binding/module 不提供关节目标、或本手尚无关节帧时为 None。
"""
        owner, controller = self._owner, self._controller
        hand = normalize_hand(controller.hand_binding, HAND_RIGHT)
        binding = owner.active_source_binding
        module = owner.source_registry.active_module()
        if (
            binding is None or module is None
            or not getattr(binding.capabilities, "provides_joint_targets", False)
        ):
            return None
        # 单手读自**一次** publication 快照（双手同批；见 mailbox 的"双手同批"节）。
        publication = owner._joint_publication.load()
        frame = None if publication is None else publication.frame(hand)
        if frame is None:
            return None
        params = self._control_params_for_cycle()
        carm = controller.carm_
        # 复用缓存前必须复核身份：被暂停的控制线程可能在 SDK 重连换掉 carm.ws 之后
        # 恢复运行，此时“旧反馈 + 新 ws”会通过 OFF→ON 闸门。
        self._discard_mismatched_feedback_cache(carm)
        try:
            init_joint = tuple(controller._active_init_joint())
        except Exception:
            init_joint = ()
        age = self._current_feedback_age()
        return JointControlInput(
            hand=hand, frame=frame, target=frame.target,
            received=frame.received_monotonic,
            binding=binding, lease=frame.lease,
            session_id=binding.session_id,
            params_epoch=params.epoch,
            carm=carm, ws=getattr(carm, "ws", None),
            arm_connection_generation=getattr(controller, "arm_connection_generation", None),
            init_joint=init_joint, feedback_joints=self._feedback_joints,
            feedback_age_s=age, now_monotonic=time.monotonic(),
            module=module,
        )

    def _runtime_matches(self, hand: str) -> bool:
        """
本事务 runtime/控制器对象身份（控制环不读 ``state.arm_states`` 投影；映射失效由
臂连接事件与 binding 相位承担）。
"""
        runtime = self._runtime
        return bool(
            runtime is not None
            and runtime.controller is self._controller
            and runtime.hand == hand
        )

    def _source_bound(self, hand: str) -> bool:
        """``_is_hand_bound_active`` 的控制环无锁等价物。

        除 runtime/控制器身份外保留原有的"会话成员"语义：该 runtime 必须仍在
        ``session.arms`` 中（列表整体替换，原子引用读）。
        """
        owner = self._owner
        session = getattr(owner, "session", None)
        if session is None or not self._runtime_matches(hand):
            return False
        arms = getattr(session, "arms", None)
        if arms is None:
            return True
        return any(
            getattr(arm, "controller", None) is self._controller for arm in arms
        )

    def _arm_connection_matches(self, carm, ws, generation):
        """只复核机械臂连接和停止请求；输入/参数变化由下一周期接收。"""
        controller = self._controller
        return bool(
            not self._stop_event.is_set() and not controller.isStop
            and controller.carm_ is carm and carm is not None
            and carm.is_connected()
            and getattr(carm, "ws", None) is ws
            and controller.arm_connection_generation == generation
        )

    def _joint_context(self, attempt, **overrides):
        """构造本周期关节计算上下文。"""
        context = dict(
            hand=attempt.hand, controller=self._controller,
            session_id=attempt.session_id,
            params_epoch=attempt.params_epoch,
            bound_active=True, need_sync_feedback=self._controller.need_sync_feedback,
            discontinuity_seq=attempt.frame.discontinuity_seq,
            joint_target=attempt.target, feedback_joints=attempt.feedback_joints,
            feedback_age_s=attempt.feedback_age_s, now_monotonic=attempt.now_monotonic,
            init_joint=attempt.init_joint, lease=attempt.lease,
            arm_connection_generation=attempt.arm_connection_generation,
        )
        context.update(overrides)
        return ControlContext(**context)

    def _clip_joints(self, requested, carm):
        """按 SDK 关节限位裁剪关节值；信息不足或越界（超出 eps 容差）时返回 None。"""
        limits = getattr(carm, "limit", None) or {}
        lower, upper = limits.get("limit_lower", []), limits.get("limit_upper", [])
        dof = getattr(carm, "arm_dof", None)
        if (
            not isinstance(dof, int) or dof <= 0
            or len(lower) != dof or len(upper) != dof or len(requested) != dof
        ):
            return None
        clipped = []
        for q, lo, hi in zip(requested, lower, upper):
            if (
                isinstance(q, bool) or isinstance(lo, bool) or isinstance(hi, bool)
                or not math.isfinite(q) or not math.isfinite(lo) or not math.isfinite(hi)
                or lo > hi or q < lo - _LIMIT_EPS or q > hi + _LIMIT_EPS
            ):
                return None
            clipped.append(min(hi, max(lo, q)))
        return tuple(clipped)

    def _kinematics_for(self, attempt):
        """构造一个绑定当前连接身份的 FK 回调（调用方在锁外使用）。

        回调内先复核 ``carm`` / ``ws`` / 代次，再调用 ``forward_kine``；返回非法
        位姿时统一返回 None。
        """
        carm, ws, generation = attempt.carm, attempt.ws, attempt.arm_connection_generation
        controller = self._controller

        def kinematics(joints):
            """绑定身份的 FK 调用：身份漂移或返回非法 pose 时返回 None。"""
            if (
                controller.carm_ is not carm
                or getattr(carm, "ws", None) is not ws
                or getattr(controller, "arm_connection_generation", None) != generation
            ):
                return None
            try:
                pose = carm.forward_kine(list(joints), tool=0)
            except Exception:
                return None
            if not isinstance(pose, (list, tuple)) or len(pose) != 7:
                return None
            pose = tuple(pose)
            if any(isinstance(v, bool) or not math.isfinite(v) for v in pose):
                return None
            return pose

        return kinematics

    def _run_joint_command(self, attempt):
        """关节路径在控制线程内计算一次，再经连接钉扎发送，不做 policy 事务复核。"""
        owner, controller = self._owner, self._controller
        binding = attempt.binding
        if (
            binding.phase is not BindingPhase.ACTIVE or not binding.input_ready
            or not owner.state.ble_connected or attempt.lease != binding.lease
            or not self._source_bound(attempt.hand)
            or attempt.target is None
            or time.monotonic() - attempt.received > _INPUT_FRESH_S
        ):
            controller.invalidate_pose_mapping()
            return ControlStepOutcome.NO_INPUT
        if attempt.frame.discontinuity_seq != self._handled_discontinuity_seq:
            controller.invalidate_pose_mapping()
            self._handled_discontinuity_seq = attempt.frame.discontinuity_seq
        policy = attempt.module.control_policy
        if policy is None:
            return ControlStepOutcome.NO_INPUT
        needs_fk = bool(policy.needs_kinematics(self._joint_context(attempt)))
        plan_pose = now_pose = None
        if needs_fk:
            kinematics = self._kinematics_for(attempt)
            plan_pose = kinematics(attempt.target.joint_positions)
            if attempt.feedback_joints is not None:
                now_pose = kinematics(attempt.feedback_joints)
        computed = policy.compute_joint_command(self._joint_context(
            attempt, feedback_age_s=self._current_feedback_age(controller),
            now_monotonic=time.monotonic(), plan_pose=plan_pose, now_pose=now_pose,
        ))
        self._publish_reengage(attempt, policy)
        if computed is None:
            return ControlStepOutcome.NO_INPUT
        action, command = computed
        if action not in (ControlAction.TRACK_JOINT, ControlAction.MOVE_JOINT) or command is None:
            return ControlStepOutcome.NO_INPUT
        joints = self._clip_joints(command.joints, attempt.carm)
        if joints is None:
            controller.invalidate_pose_mapping()
            return ControlStepOutcome.DISPATCH_REJECTED

        def admit():
            """SDK 准入只保护停止请求与臂连接，不再复核输入帧。"""
            return self._arm_connection_matches(
                attempt.carm, attempt.ws, attempt.arm_connection_generation,
            )

        if not admit():
            return ControlStepOutcome.NO_INPUT
        if action is ControlAction.MOVE_JOINT:
            result = attempt.carm.dispatch_joint_for_connection(
                list(joints), attempt.ws, admit, controller.move_speed, False,
            )
        else:
            result = attempt.carm.dispatch_joint_target_for_connection(
                joints, attempt.ws, admit,
                self._build_joint_end_effector_send(controller, attempt.hand, command.end_effector_trigger),
            )
        committed = result is True or type(result) is int and result == 1
        if committed:
            controller.need_sync_feedback = False
            return ControlStepOutcome.DISPATCH_COMMITTED
        controller.invalidate_pose_mapping()
        return ControlStepOutcome.DISPATCH_REJECTED

    def _build_joint_end_effector_send(self, controller, hand, trigger):
        """构造随关节派发一并发送的末端回调（驱动缺失只告警一次）。"""
        driver = controller.end_effector_driver

        def send_end_effector():
            """发送关节路径的末端触发值（驱动缺失只告警一次，发送失败由 EndEffectorSender 记录）。"""
            if trigger is None:
                return
            if driver is None:
                if not self._joint_eff_missing_logged:
                    self._joint_eff_missing_logged = True
                    logging.warning(
                        "[ControlTransaction] %s arm has no end-effector driver; gripper is not driven",
                        hand,
                    )
                return
            driver.set_eff(trigger)

        return send_end_effector

    def _publish_reengage(self, attempt, policy):
        """
发布再接入状态到每手单槽（控制线程写、SSE 读）；只有状态变化时才在 data 锁下
推进 ``data_sequence``。计算时的 binding 已被替换则丢弃本次观察。
None 是有效观察，须清除上一状态。
"""
        owner = self._owner
        status = policy.get_reengage_status(attempt.hand)
        if status is not None and not isinstance(status, ReengageStatus):
            raise TypeError("policy reengagement status must be ReengageStatus or None")
        if not attempt.binding.capabilities.supports_reengagement:
            status = None
        with owner._data_lock:
            if owner.active_source_binding is not attempt.binding:
                return
            cell = owner._reengage_cells[attempt.hand]
            if cell.load() == status:
                return
            cell.store(status)
            owner.state.data_sequence += 1
        if status is not None and status.reason in ("pose_delta_exceeded", "feedback_unavailable"):
            logging.warning(
                "[ControlTransaction] %s arm reengagement blocked: %s; position_threshold=%s m; rotation_threshold=%s deg; exceeded=%s; switch OFF, align and ON to retry",
                attempt.hand, status.reason, status.position_threshold_m,
                status.rotation_threshold_deg,
                [item.as_projection() for item in status.exceeded],
            )

    def _capture_sample(self, arm_feedback_fresh: bool):
        """一次读取本手冻结帧；断续事件由本控制线程重定基并确认。"""
        owner, controller = self._owner, self._controller
        hand = normalize_hand(controller.hand_binding, HAND_RIGHT)
        binding = owner.active_source_binding
        params = self._control_params_for_cycle()
        publication = owner._input_publication.load()
        frame = None if publication is None else publication.frame(hand)
        if frame is not None:
            self._captured_frame = frame
            self._stats.record_input(frame.input_sequence)
        if not self._home_baseline_reset:
            # 新事务不补发上一事务期间累计的回初始位请求。
            controller.reset_home_baseline()
            self._home_baseline_reset = True
        home_identity = (
            None if frame is None
            else (None if binding is None else binding.session_id, frame.lease)
        )

        def discard_home(reason):
            # 不可控期间的回初始位请求只消费不补发；位姿无效时控制样本保留旧值，
            # 因此读帧上最新的功能结果。
            if frame is not None:
                controller.discard_home_intent(frame.intent, home_identity, reason)

        if (
            self._stop_event.is_set() or controller.isStop
            or binding is None or binding.phase is not BindingPhase.ACTIVE
            or not binding.input_ready or not owner.state.ble_connected
            or not self._source_bound(hand)
        ):
            discard_home("input_not_ready_or_unbound")
            controller.invalidate_pose_mapping()
            return ControlStepOutcome.NO_INPUT, None
        sample = None if frame is None else frame.control_sample
        if (
            sample is None or frame.lease != binding.lease
            or sample.input_sequence != frame.input_sequence
            or sample.source_generation != binding.session_id
            or not sample.pose_valid or sample.pose is None or sample.intent is None
        ):
            discard_home("missing_sample_or_source_identity_changed")
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        if not self._mapping_started:
            # 新事务首个有效样本不能沿用上一线程的映射，必须同步新鲜反馈。
            controller.invalidate_pose_mapping()
            self._mapping_started = True
        if controller.home_motion_pending(arm_feedback_fresh):
            # 运动中的实测位姿不能当基线，否则回位结束后跟随会把臂拉回这里。
            discard_home("home_motion_in_progress")
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        intent = sample.intent
        if (frame.discontinuity_seq != self._handled_discontinuity_seq
                or (binding.capabilities.input_clocked_control
                    and controller.need_sync_feedback and intent.motion_engaged)):
            discard_home("input_rebase")
            if not intent.motion_engaged:
                controller.invalidate_pose_mapping()
                applied = True
            else:
                pose = sample.pose
                raw_pose = (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)
                applied = controller._apply_input_rebase(
                    raw_pose, params.calibration, intent, arm_feedback_fresh,
                )
            if applied:
                self._handled_discontinuity_seq = frame.discontinuity_seq
                self._rebase_frame_sequence = frame.input_sequence
                logging.warning(
                    "[ControlTransaction] Input rebased: hand=%s discontinuity=%s",
                    hand, frame.discontinuity_seq,
                )
                return ControlStepOutcome.REBASE_APPLIED, None
            # 未取得新鲜反馈或有效位姿时保持 pending，禁止旧目标泄漏。
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        if frame.input_sequence == self._rebase_frame_sequence:
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        if (binding.capabilities.input_clocked_control
                and frame.input_sequence == self._last_dispatched_sequence):
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        return ControlStepOutcome.COMPUTED, ControlSample(
            pose_data=sample.pose, button_data=sample.button, params=params,
            arm_connection_generation=controller.arm_connection_generation,
            arm_feedback_state_sequence=controller.arm_feedback_state_sequence,
            arm_feedback_fresh=arm_feedback_fresh,
            intent_data=intent, home_identity=home_identity,
        )

    def _dispatch_pose(self) -> ControlStepOutcome:
        """发送当前目标，成功则提交分段基线，失败则失效映射。"""
        controller = self._controller
        if self._stop_event.is_set() or controller.isStop:
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT
        dispatched = controller.dispatch_pose()
        if dispatched is True:
            frame = self._captured_frame
            if frame is not None:
                self._last_dispatched_sequence = frame.input_sequence
            return ControlStepOutcome.DISPATCH_COMMITTED
        controller.invalidate_pose_mapping()
        # 每次拒绝都会让下一个握持样本重定基；限频记录以区分发送失败与松开运动键。
        self._dispatch_rejects += 1
        now = time.monotonic()
        if now - self._dispatch_reject_logged >= _DISPATCH_REJECT_LOG_INTERVAL_S:
            logging.warning(
                "[ControlTransaction] Pose dispatch rejected: role=%s result=%r count=%d",
                getattr(controller, "arm_role", "unknown"), dispatched, self._dispatch_rejects,
            )
            self._dispatch_reject_logged = now
            self._dispatch_rejects = 0
        return ControlStepOutcome.DISPATCH_REJECTED

    def _dispatch_joint_move(self, attempt) -> bool:
        """A 键运动只保留停止与臂连接准入，参数/输入变化留待下一周期处理。"""
        def admit():
            """SDK 发送前确认停止请求及当前机械臂连接。"""
            return self._arm_connection_matches(
                attempt.carm, attempt.arm_ws_identity, attempt.arm_connection_generation,
            )

        result = admit() and self._controller.dispatch_joint_move_attempt(attempt, admit)
        if result is True:
            self._controller.begin_home_motion(attempt)
        else:
            # 按键序号已消费，不会补发；记录下来，否则操作者只看到"按了没反应"。
            logging.warning(
                "[ControlTransaction] Home motion not sent: role=%s result=%r",
                getattr(self._controller, "arm_role", "unknown"), result,
            )
            self._controller.invalidate_pose_mapping()
        return result is True

    def _termination(
        self,
        kind: LoopTerminationKind,
        reason: str,
    ) -> LoopTermination:
        """构造终止凭证：在 data 锁内读取当前 binding 身份快照。

        Returns:
            LoopTermination：终止原因与终止时刻的身份快照
        """
        owner = self._owner
        controller = self._controller
        with owner._data_lock:
            binding = owner.active_source_binding
            return LoopTermination(
                kind=kind,
                reason=reason,
                runtime=self._runtime,
                transaction=self,
                session_id=binding.session_id if binding is not None else None,
                source_lease=binding.lease if binding is not None else None,
                controller_identity=controller,
            )
