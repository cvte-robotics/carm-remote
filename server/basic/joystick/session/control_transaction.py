"""Host 主导的单臂控制事务（每臂一个控制环，不取 Host data 锁）。

每周期把输入帧变成一次控制计算与 SDK 派发（关节路径优先，否则位姿路径），并维护
按接受时刻身份钉扎的反馈缓存。调度：binding 声明 ``input_clocked_control`` 时新帧
发布即唤醒（最长 1/72s 超时兜底）；否则按 72Hz 绝对时间节拍运行。``_policy_lock`` 是叶子 gate；锁序
``pose_state_lock → _policy_lock``，data 锁只出现在低频旁路且总在最外层。
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
    ControlCompletion,
    ControlContext,
    ControlOutcome,
    ControlResult,
    PolicyDispatchContext,
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
    controller_identity 判断终止是否仍属于当前会话。``epoch`` 仅保留为终止时刻的
    诊断快照；参数或映射纪元变化本身不会把当前控制线程的异常判为陈旧。
    """

    kind: LoopTerminationKind
    reason: str
    runtime: Any
    transaction: Any
    session_id: Optional[int]
    epoch: Any
    source_lease: Any
    controller_identity: Any


@dataclass(frozen=True)
class JointCommandAttempt:
    """一次冻结的关节命令尝试（关节 provider 路径，控制环无 Host data 锁）。

    ``publication`` 是本手帧所属的双手快照对象（身份即"本手帧仍是当前帧"）；
    其余身份字段用于准入与 post 复核；``epoch`` 为冻结时 ``controller.current_epoch``
    的对象身份，用于拒绝无效反馈后的在途 attempt。
    """

    hand: str
    frame: Any
    publication: Any
    target: Any
    received: float
    binding: Any
    lease: Any
    session_id: int
    params_epoch: int
    epoch: Any
    carm: Any
    ws: Any
    state: Any
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
        arm_feedback_fresh = controller._refresh_current_pose()
        self._observe_feedback(arm_feedback_fresh)
        attempt = self._freeze_joint_attempt()
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
        if sample.pose_data is None or sample.button_data is None:
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

    def _freeze_joint_attempt(self):
        """
冻结一个关节命令尝试；控制环不取 Host data 锁（一致性由发布侧提交规则保证）。

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
        params = owner._control_params
        carm = controller.carm_
        # 复用缓存前必须复核身份：被暂停的控制线程可能在 SDK 重连换掉 carm.ws 之后
        # 恢复运行，此时“旧反馈 + 新 ws”会通过 OFF→ON 闸门。
        self._discard_mismatched_feedback_cache(carm)
        try:
            init_joint = tuple(controller._active_init_joint())
        except Exception:
            init_joint = ()
        age = self._current_feedback_age()
        return JointCommandAttempt(
            hand=hand, frame=frame, publication=publication, target=frame.target,
            received=frame.received_monotonic,
            binding=binding, lease=frame.lease,
            session_id=binding.session_id,
            params_epoch=params.epoch,
            epoch=controller.current_epoch,
            carm=carm, ws=getattr(carm, "ws", None),
            state=getattr(carm, "state", None),
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
        """``_is_hand_bound_active_locked`` 的控制环无锁等价物。

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

    def _joint_session_matches(self, attempt, require_current_frame=True):
        """关节命令的准入/提交复核（调用前必须持有 ``_policy_lock``）。

        Args:
            attempt: 冻结的关节命令尝试
            require_current_frame: True 时额外要求信箱中仍是同一个冻结帧对象、目标
                存在且接收时间新鲜（发送前用）；False 时只做身份复核（post 用，
                故意更弱）
        Returns:
            bool：全部身份与帧条件成立时为 True
        """
        owner, controller = self._owner, self._controller
        binding = owner.active_source_binding
        params = owner._control_params
        if not (
            not self._stop_event.is_set() and not controller.isStop
            and binding is not None
            and binding.phase is BindingPhase.ACTIVE and binding.input_ready
            and binding.lease == attempt.lease
            and attempt.lease is not None
            and binding.session_id == attempt.session_id
            and params.epoch == attempt.params_epoch
            # ControlEpoch 约束：无效反馈帧只发布新纪元（binding/参数/连接全不变），
            # 因此必须比较纪元对象身份，否则新关节帧仍会通过 admit。
            and controller.current_epoch is attempt.epoch
            and owner.state.ble_connected
            and self._source_bound(attempt.hand)
            and controller.carm_ is attempt.carm
            and getattr(attempt.carm, "ws", None) is attempt.ws
            and getattr(controller, "arm_connection_generation", None) == attempt.arm_connection_generation
        ):
            return False
        if require_current_frame:
            # 发送要求信箱中仍是同一个冻结 publication（它携带本手的同一个帧对象）、
            # 目标可用且接收时间新鲜；发送后的复核故意放宽这一条。
            if (
                owner._joint_publication.load() is not attempt.publication
                or attempt.target is None
                or time.monotonic() - attempt.received > _INPUT_FRESH_S
            ):
                return False
        return True

    def _joint_context(self, attempt, **overrides):
        """从冻结 attempt 构造 PolicyDispatch 上下文（字段与旧版逐字对应）。"""
        context = dict(
            hand=attempt.hand, controller=self._controller,
            session_id=attempt.session_id,
            params_epoch=attempt.params_epoch,
            bound_active=True, need_sync_feedback=False,
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
        """
关节命令：冻结 → needs_kinematics → 锁外 FK → 复核 → decide → admit → post。

顺序不可换：关节输入策略可能在 OFF→ON 边沿一次性消费开关状态并依赖
FK 结果，decide 先于 FK 会让该周期永久拒绝。锁：只有叶子 ``_policy_lock``。

Args:
    attempt: 冻结的关节命令尝试。

Returns:
    ControlStepOutcome：本周期结束状态。
"""
        owner, controller = self._owner, self._controller
        policy = attempt.module.control_policy
        if policy is None:
            return ControlStepOutcome.NO_INPUT
        with owner._policy_lock:
            if not self._joint_session_matches(attempt, require_current_frame=True):
                return ControlStepOutcome.NO_INPUT
            # 契约：needs_kinematics 在 gate 内运行（廉价），FK RPC 留在 gate 外。
            needs_fk = bool(policy.needs_kinematics(self._joint_context(attempt)))
        plan_pose = now_pose = None
        if needs_fk:
            kinematics = self._kinematics_for(attempt)
            if attempt.target is not None:
                plan_pose = kinematics(attempt.target.joint_positions)
            if attempt.feedback_joints is not None:
                now_pose = kinematics(attempt.feedback_joints)
        with owner._policy_lock:
            if not self._joint_session_matches(attempt, require_current_frame=False):
                return ControlStepOutcome.IDENTITY_INVALIDATED
            decision = policy.decide_control(self._joint_context(
                attempt,
                feedback_age_s=self._current_feedback_age(controller),
                now_monotonic=time.monotonic(),
                plan_pose=plan_pose, now_pose=now_pose,
            ))
        self._publish_reengage(attempt, policy)
        if decision.action is ControlAction.REJECT:
            return ControlStepOutcome.NO_INPUT
        if decision.action not in (ControlAction.TRACK_JOINT, ControlAction.MOVE_JOINT):
            return ControlStepOutcome.NO_INPUT
        command = decision.command
        if command is None:
            return ControlStepOutcome.NO_INPUT
        dispatch_ctx = PolicyDispatchContext(
            attempt.hand, controller,
            decision.source_token, attempt.session_id, attempt.params_epoch, True,
        )
        with owner._policy_lock:
            if not self._joint_session_matches(attempt, require_current_frame=True):
                return ControlStepOutcome.NO_INPUT
            if not policy.validate_dispatch(dispatch_ctx):
                return ControlStepOutcome.NO_INPUT
            joints = self._clip_joints(command.joints, attempt.carm)
        if joints is None:
            return ControlStepOutcome.DISPATCH_REJECTED

        def admit():
            """SDK 发送前准入：gate 内复核 attempt 身份并交给 policy validate_dispatch。"""
            with owner._policy_lock:
                if not self._joint_session_matches(attempt, require_current_frame=True):
                    return False
                return policy.validate_dispatch(PolicyDispatchContext(
                    attempt.hand, controller,
                    decision.source_token, attempt.session_id, attempt.params_epoch, True,
                ))

        if decision.action is ControlAction.MOVE_JOINT:
            result = attempt.carm.dispatch_joint_for_connection(
                list(joints), attempt.ws, attempt.state, admit, command.duration_s or 5, False,
            )
        else:
            result = attempt.carm.dispatch_joint_target_for_connection(
                joints, attempt.ws, admit,
                self._build_joint_end_effector_send(controller, attempt.hand, command.end_effector_trigger),
            )
        committed = result is True or type(result) is int and result == 1
        with owner._policy_lock:
            session_ok = self._joint_session_matches(attempt, require_current_frame=False)
            outcome = (
                ControlOutcome.DISPATCH_COMMITTED if committed else ControlOutcome.DISPATCH_REJECTED
            )
            if not session_ok:
                outcome = ControlOutcome.STALE
            policy.complete_control(ControlCompletion(
                PolicyDispatchContext(
                    attempt.hand, controller,
                    decision.source_token, attempt.session_id, attempt.params_epoch, False,
                ),
                ControlResult(outcome, decision.source_token, outcome.value),
            ))
        self._publish_reengage(attempt, policy)
        if not session_ok:
            return ControlStepOutcome.IDENTITY_INVALIDATED
        if committed:
            return ControlStepOutcome.DISPATCH_COMMITTED
        return ControlStepOutcome.DISPATCH_REJECTED

    def _build_joint_end_effector_send(self, controller, hand, trigger):
        """构造随关节派发一并发送的末端回调（驱动缺失只告警一次）。"""
        driver = controller.end_effector_driver

        def send_end_effector():
            """发送关节路径的末端触发值（驱动缺失只告警一次）。"""
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
            try:
                accepted = driver.set_eff(trigger)
            except Exception:
                logging.exception("[ControlTransaction] %s arm end-effector send raised", hand)
                return
            if accepted is not True:
                logging.warning(
                    "[ControlTransaction] %s arm end-effector send was not accepted: %r",
                    hand, accepted,
                )

        return send_end_effector

    def _publish_reengage(self, attempt, policy):
        """
发布再接入状态到每手单槽（控制线程写、SSE 读）；只有状态变化时才在 data 锁下
bump ``data_sequence``。闸内读与 data 锁内 store 分两段，之间 binding 被替换则丢弃。
None 是有效观察，须清除上一状态。
"""
        owner = self._owner
        with owner._policy_lock:
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
        """采样路径：无 data 锁读帧/binding/params，gate 内决策与帧身份复核。

        Args:
            arm_feedback_fresh: 本周期是否接受了新的臂反馈帧
        Returns:
            Tuple[ControlStepOutcome, Optional[ControlSample]]：需在本周期计算的
            冻结样本，或 (结束状态, None)
        """
        owner = self._owner
        controller = self._controller
        normalized_hand = normalize_hand(controller.hand_binding, HAND_RIGHT)
        binding = owner.active_source_binding
        module = owner.source_registry.active_module()
        policy = module.control_policy if module is not None else None
        params = owner._control_params
        # 单手读自**一次** publication 快照：双手来自同一次发布，不存在混合批次
        # （见 mailbox 的"双手同批"节）。
        input_publication = owner._input_publication.load()
        frame = (
            None if input_publication is None
            else input_publication.frame(normalized_hand)
        )
        unified_sample = None if frame is None else frame.control_sample
        if frame is not None:
            self._captured_frame = frame
            self._stats.record_input(frame.input_sequence)
        session_id = binding.session_id if binding is not None else None
        # 帧自身所属会话与其中控制样本的来源会话都必须对得上：迟到发布的旧帧
        # 不得在当前会话被消费。
        if (
            unified_sample is None
            or unified_sample.source_generation != session_id
            or frame.lease != (binding.lease if binding is not None else None)
        ):
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        if not unified_sample.pose_valid or unified_sample.pose is None or unified_sample.button is None:
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None
        # 控制路径只消费双接受的冻结样本，直接携带，不再投影回 legacy 视图。
        button_data = unified_sample.button
        pose_data = unified_sample.pose
        input_ready = bool(
            not self._stop_event.is_set()
            and
            binding is not None
            and binding.phase is BindingPhase.ACTIVE
            and binding.input_ready
            and owner.state.ble_connected
        )
        source_bound = self._source_bound(normalized_hand)
        if not input_ready or not source_bound:
            controller.invalidate_pose_mapping()
            return ControlStepOutcome.NO_INPUT, None

        epoch = controller.current_epoch
        with owner._policy_lock:
            decision = (
                policy.decide_control(ControlContext(
                    hand=normalized_hand,
                    controller=controller,
                    session_id=session_id,
                    params_epoch=params.epoch,
                    bound_active=source_bound and input_ready,
                    need_sync_feedback=bool(
                        controller.need_sync_feedback
                        and button_data is not None
                        and button_data.grip > 100
                    ),
                ))
                if module is not None
                else None
            )
            # 帧身份复核：帧已被更新批次替换（decision 属于新帧）则放弃本周期，
            # 不把旧帧载荷配上新帧 token。比较本手帧对象而非 publication 整体：
            # 另一手的更新不必让本手白丢一拍。
            current_publication = owner._input_publication.load()
            frame_is_current = (
                current_publication is not None
                and current_publication.frame(normalized_hand) is frame
            )
        if not frame_is_current:
            # 本周期放弃；新帧的发布会立即唤醒下一步（输入节拍）或在下一节拍处理。
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None

        if decision is not None and decision.action is ControlAction.REBASE:
            token = decision.source_token
            payload = decision.rebase_payload
            raw_pose = getattr(payload, "raw_pose", None)
            if token is None or button_data is None or raw_pose is None:
                controller.reject_pose_target()
                return ControlStepOutcome.NO_INPUT, None
            with controller.pose_state_lock:
                if controller.current_epoch is not epoch:
                    controller.reject_pose_target()
                    return ControlStepOutcome.IDENTITY_INVALIDATED, None
                applied = controller._apply_input_rebase_locked(
                    raw_pose,
                    params.calibration,
                    button_data,
                    arm_feedback_fresh,
                )
                committed_rebase = bool(
                    applied
                    and self._complete_rebase(
                        policy, normalized_hand, controller, token, session_id, params.epoch,
                    )
                )
            if committed_rebase:
                logging.warning(
                    "[ControlTransaction] pose rebased: hand=%s reason=%s",
                    normalized_hand,
                    getattr(payload, "reason", ""),
                )
                return ControlStepOutcome.REBASE_APPLIED, None
            return ControlStepOutcome.IDENTITY_INVALIDATED, None

        if decision is not None and decision.action is not ControlAction.READY:
            controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT, None

        return ControlStepOutcome.COMPUTED, ControlSample(
            pose_data=pose_data,
            button_data=button_data,
            params=params,
            input_sequence=(frame.input_sequence if frame is not None else 0),
            session_id=session_id,
            input_source=owner.active_input_source,
            source_lease=binding.lease if binding is not None else None,
            source_token=(decision.source_token if decision is not None else None),
            epoch=epoch,
            arm_connection_generation=controller.arm_connection_generation,
            arm_feedback_state_sequence=controller.arm_feedback_state_sequence,
            arm_feedback_fresh=arm_feedback_fresh,
        )

    def _complete_rebase(self, policy, hand, controller, token, session_id, params_epoch):
        """在 gate 内提交 rebase（与 decide 串行化 policy 状态）。"""
        with self._owner._policy_lock:
            return (
                policy.complete_control(ControlCompletion(
                    PolicyDispatchContext(hand, controller, token, session_id, params_epoch),
                    ControlResult(ControlOutcome.REBASE_APPLIED, token, "rebase"),
                )).outcome
                is ControlOutcome.REBASE_APPLIED
            )

    def _host_matches(self, attempt, module, lease, require_current_frame=True):
        """Host 侧复核：本次 attempt 在控制环视角是否仍然有效（gate 内执行）。

        Args:
            attempt: 位姿派发事务
            module: 捕获时的 provider module（按对象身份比较）
            lease: 捕获时的 lease
            require_current_frame: True 时要求信箱中当前帧的 input_sequence 与目标
                一致（发送前用）；False 时跳过该子句（post 用）
        Returns:
            bool：runtime/binding/params/lease/policy 全部仍然成立时为 True
        """
        owner = self._owner
        target = attempt.target_state
        binding = owner.active_source_binding
        # decide/准入/提交与 provider 的"policy 推进 + 帧发布"共用同一个 gate，
        # 帧与按钮也在 gate 内读取（gate 外读到的陈旧 grip-on 会误放行/误报提交）。
        # 锁序 pose_state_lock → _policy_lock；复核比较全部身份，不得只比较纪元。
        with owner._policy_lock:
            # gate 内读一次 publication 快照（而不是读两次每手 cell）：双手同批。
            publication = owner._input_publication.load()
            frame = (
                None if publication is None
                else publication.frame(attempt.hand_binding)
            )
            button_data = None if frame is None else frame.button
            return bool(
                self._runtime_matches(attempt.hand_binding)
                and not self._stop_event.is_set()
                and binding is not None
                and binding.lease == lease
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
                and binding.session_id == target.session_id
                and owner.active_input_source == target.input_source
                and owner.state.ble_connected
                and (
                    not require_current_frame
                    or (frame is not None and frame.input_sequence == target.input_sequence)
                )
                and owner._control_params.epoch == target.params_epoch
                and button_data is not None
                and button_data.grip > 100
                and owner.source_registry.active_module() is module
                and module.control_policy.validate_dispatch(PolicyDispatchContext(
                    hand=attempt.hand_binding,
                    controller=self._controller,
                    source_token=target.source_token,
                    session_id=target.session_id,
                    params_epoch=target.params_epoch,
                    require_current_frame=require_current_frame,
                ))
            )

    def _dispatch_pose(self) -> ControlStepOutcome:
        """
完成一次位姿派发事务：捕获 attempt → gate 内复核 → 锁外发送 → post 复核。

SDK 物理发送在 gate 外（否则控制环与 provider 回调互相阻塞）；pre-send 的
``_host_matches`` 是最后一个准入点，post 复核捕获在途窗口的失效。

Returns:
    ControlStepOutcome。
"""
        owner = self._owner
        controller = self._controller
        module = owner.source_registry.active_module()
        binding = owner.active_source_binding
        if module is None or binding is None:
            with controller.pose_state_lock:
                controller.reject_pose_target()
            return ControlStepOutcome.NO_INPUT
        lease = binding.lease
        with controller.pose_state_lock:
            attempt = controller._capture_dispatch_attempt_locked()
            if attempt is None or not self._host_matches(attempt, module, lease):
                controller.reject_pose_target()
                return ControlStepOutcome.IDENTITY_INVALIDATED

        dispatched = controller.dispatch_pose_attempt(attempt)
        if dispatched is not True:
            logging.error(
                "[ControlTransaction] track_pose rejected for %s: %r",
                attempt.arm_role,
                dispatched,
            )

        # post 复核须一致读 pose 状态（锁序 data > pose > gate），并与
        # ``complete_control`` 在同一 gate 临界区内完成，否则其间 store 的新帧
        # 会让失效派发被误报为已提交；invalidate 留在 gate 外。
        with controller.pose_state_lock:
            attempt_current = controller._dispatch_attempt_matches_locked(attempt)
            outcome = None
            with owner._policy_lock:
                host_current = self._host_matches(
                    attempt,
                    module,
                    lease,
                    require_current_frame=False,
                )
                if attempt_current and host_current:
                    if dispatched is not True:
                        module.control_policy.complete_control(ControlCompletion(
                            PolicyDispatchContext(attempt.hand_binding, controller, attempt.target_state.source_token, attempt.target_state.session_id, attempt.target_state.params_epoch, False),
                            ControlResult(
                                ControlOutcome.DISPATCH_REJECTED,
                                attempt.target_state.source_token,
                                "physical_dispatch_failed",
                            ),
                        ))
                        outcome = ControlStepOutcome.DISPATCH_REJECTED
                    else:
                        result = module.control_policy.complete_control(ControlCompletion(
                            PolicyDispatchContext(attempt.hand_binding, controller, attempt.target_state.source_token, attempt.target_state.session_id, attempt.target_state.params_epoch, False),
                            ControlResult(
                                ControlOutcome.DISPATCH_COMMITTED,
                                attempt.target_state.source_token,
                                "physical_dispatch_succeeded",
                            ),
                        ))
                        if result.outcome is ControlOutcome.DISPATCH_COMMITTED:
                            controller._commit_dispatch_attempt_locked(attempt)
                            outcome = ControlStepOutcome.DISPATCH_COMMITTED
                        else:
                            outcome = ControlStepOutcome.DISPATCH_REJECTED
            if outcome is None:
                if attempt_current:
                    controller.invalidate_pose_mapping()
                return ControlStepOutcome.IDENTITY_INVALIDATED
            if outcome is ControlStepOutcome.DISPATCH_REJECTED:
                controller.invalidate_pose_mapping()
            return outcome

    def _joint_attempt_matches(self, attempt) -> bool:
        """A 键关节运动 attempt 的身份复核（ControlEpoch 硬约束见函数内注释）。

        Returns:
            bool：停止标志、binding/lease/session/params/输入源、epoch 身份、
            连接对象身份与连接代次全部成立时为 True
        """
        owner = self._owner
        controller = self._controller
        binding = owner.active_source_binding
        # ControlEpoch 约束：除 current_epoch 身份外必须同时比较 binding 身份
        # （lease/session）、ControlParams（params_epoch）与 arm_connection_generation。
        return bool(
            not self._stop_event.is_set()
            and binding is not None
            and binding.phase is BindingPhase.ACTIVE
            and binding.input_ready
            and owner.state.ble_connected
            and binding.session_id == attempt.session_id
            and owner._control_params.epoch == attempt.params_epoch
            and owner.active_input_source == attempt.input_source
            and not controller.isStop
            and controller.current_epoch is attempt.epoch
            and controller.arm_connection_generation == attempt.arm_connection_generation
            and controller.carm_ is attempt.carm
            and attempt.carm.ws is attempt.arm_ws_identity
            and attempt.carm.state is attempt.arm_state_identity
        )

    def _dispatch_joint_move(self, attempt) -> bool:
        """派发 A 键关节运动；发送前后各做一次 attempt 身份复核。"""
        owner = self._owner
        controller = self._controller
        if not self._joint_attempt_matches(attempt):
            logging.warning(
                "[ControlTransaction] Input disconnected before A-button move"
            )
            return False

        def admit() -> bool:
            """SDK 发送前准入：gate 内复核 A 键关节运动 attempt 身份。"""
            with owner._policy_lock:
                return self._joint_attempt_matches(attempt)

        return controller.dispatch_joint_move_attempt(attempt, admit)

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
                epoch=controller.current_epoch,
                source_lease=binding.lease if binding is not None else None,
                controller_identity=controller,
            )
