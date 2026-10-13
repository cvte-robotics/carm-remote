"""Quest 专用输入 policy：只推进帧语义（jump 检测、rebase pin、frame barrier）。

资源与生命周期全部由 module/receiver 层持有。
"""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from carm_teleop_input_contracts import (
    BaseSourceControlPolicy,
    ControlAction,
    ControlCompletion,
    ControlContext,
    ControlDecision,
    ControlOutcome,
    ControlResult,
    HAND_LEFT,
    HAND_RIGHT,
    PolicyDispatchContext,
    RebasePayload,
)


@dataclass(frozen=True)
class _QuestRebasePin:
    """一次待执行的 rebase 请求（不可变快照）。

    pin 在创建时冻结当时的 session、epoch、receiver generation 与 raw pose；Host
    确认 pin 仍然匹配后才会把它作为 RebasePayload 下发。任何失效路径必须换掉或
    清空 ``_QuestHandJumpState.pin``。
    """

    jump_epoch: int
    frame_index: int
    raw_pose: Tuple[float, ...]
    session_id: int
    receiver_generation: int
    params_epoch: int
    reason: str


@dataclass
class _QuestHandJumpState:
    """
单手位在 Quest 帧序列上的可变状态。

``jump_epoch``/``required_epoch``/``handled_jump_epoch`` 构成 jump 事件生命周期
（required == handled 表示无未处理跳变）；``comparison_pose`` 是下次 jump 检测
基准；``rebase_sequence``/``last_handled_fi`` 构成"该帧不得再次下发"屏障。
"""

    comparison_pose: Optional[Tuple[float, ...]] = None
    latest_valid_pose: Optional[Tuple[float, ...]] = None
    latest_valid_fi: Optional[int] = None
    latest_frame_pose_valid: bool = False
    latest_frame_new_valid: bool = False
    latest_receiver_generation: Optional[int] = None
    jump_epoch: int = 0
    required_epoch: int = 0
    pin: Optional[_QuestRebasePin] = None
    deferred_unlock_rebase: bool = False
    rotation_locked: bool = False
    controller_identity: Optional[object] = None
    handled_jump_epoch: int = 0
    rebase_sequence: Optional[int] = None
    last_handled_fi: Optional[int] = None
    # 当前帧唯一活跃 token：校验比较对象身份，每条失效路径必须丢掉该引用，
    # 帧绑定未变时复用同一对象。
    issued_token: Optional["QuestSourceToken"] = None


@dataclass(frozen=True)
class QuestFrameContext:
    """同一帧内单手位的输入快照（由 module 从 RawInputBatch 组装）。

    ``raw_pose`` 是 policy 归一化后的 pose（位姿无效时为 None），``bound_active``
    表示该手当前已绑定 controller。
    """

    hand: str
    frame_index: int
    pose_valid: bool
    raw_pose: Optional[Tuple[float, ...]]
    grip: int
    rotation_locked: bool
    bound_active: bool


@dataclass(frozen=True)
class QuestFrameDecision:
    """policy 对单帧单手位的决策，由 module 转成 InputUpdateDecision。

    ``controller`` 是决策时该手绑定的 controller；``invalidate_controller`` 表示
    本帧产生了新 pin（需先重取位姿），``reject_controller`` 表示本帧没有带来新的
    有效位姿、必须拒绝该 controller 的控制。
    """

    hand: str
    accept_pose: bool
    controller: Optional[object]
    invalidate_controller: bool
    reject_controller: bool
    jump_triggered: bool = False
    jump_position_m: Optional[float] = None
    jump_rotation_deg: Optional[float] = None


@dataclass(frozen=True)
class QuestSourceToken:
    """
policy 私有的帧身份（模块外不透明）：派发校验按对象身份比较，字段只记录铸造
时的帧绑定，供 ``prepare_control`` 判断未变帧可否沿用同一对象。session 语义由
对象身份覆盖（session 变化会清空本 policy 状态）。
"""

    jump_epoch: int
    frame_index: int
    receiver_generation: int


@dataclass(frozen=True)
class QuestControlDecision:
    """一次 ``prepare_control`` 的结果。

    ``pin`` 非 None 表示需要 Host 先执行 rebase，``pin_matches`` 表示该 pin 与当前
    session/epoch/receiver generation/jump epoch 一致；``ready`` 表示可直接下发，
    此时 ``source_token`` 为 policy 私有的 frame token。
    """

    pin: Optional[_QuestRebasePin]
    pin_matches: bool
    ready: bool
    source_token: Optional[QuestSourceToken] = None


def _quest_pose_values(pose):
    """把 pose 映射归一化为 (x, y, z, 单位四元数)。

    Args:
        pose: 含 x/y/z/qx/qy/qz/qw 的映射。

    Returns:
        7 元 tuple，后半段四元数已归一化；任一字段不是有限数值，或四元数模长
        近似为 0 时返回 None。
    """
    values = tuple(pose.get(key) for key in ("x", "y", "z", "qx", "qy", "qz", "qw"))
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in values
    ):
        return None
    quaternion_norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
    if quaternion_norm <= 1e-12:
        return None
    normalized = tuple(float(value) for value in values)
    return normalized[:3] + tuple(value / quaternion_norm for value in normalized[3:])


def _quest_pose_jump(
    previous_pose,
    current_pose,
    position_threshold_m,
    rotation_threshold_deg,
    rotation_locked,
):
    """比较两个已归一化 pose，判定是否发生 jump。

    Args:
        previous_pose: 上一次接受的归一化 pose。
        current_pose: 本帧归一化 pose。
        position_threshold_m: 位置跳变阈值（米）。
        rotation_threshold_deg: 旋转跳变阈值（度）。
        rotation_locked: 为 True 时忽略旋转跳变。

    Returns:
        (是否 jump, 位置位移, 旋转角度) 三元组；旋转角取自四元数点积的绝对值，
        因此不区分四元数整体符号翻转。
    """
    position_delta = math.sqrt(
        sum((current_pose[index] - previous_pose[index]) ** 2 for index in range(3))
    )
    quaternion_dot = abs(
        sum(previous_pose[index] * current_pose[index] for index in range(3, 7))
    )
    rotation_delta_deg = math.degrees(
        2.0 * math.acos(max(0.0, min(1.0, quaternion_dot)))
    )
    position_jump = position_delta > position_threshold_m
    rotation_jump = not rotation_locked and rotation_delta_deg > rotation_threshold_deg
    return position_jump or rotation_jump, position_delta, rotation_delta_deg


class QuestInputPolicy(BaseSourceControlPolicy):
    """持有全部 Quest jump、pin 与 frame barrier 状态迁移。"""

    def __init__(self):
        self.__hand_states: Dict[str, _QuestHandJumpState] = self._new_hand_states()
        self.__frame_receiver_generation: Optional[int] = None
        self.__last_frame_index: Optional[int] = None

    @staticmethod
    def _new_hand_states() -> Dict[str, _QuestHandJumpState]:
        """新建双手的空白状态表。"""
        return {
            HAND_LEFT: _QuestHandJumpState(),
            HAND_RIGHT: _QuestHandJumpState(),
        }

    def reset(self) -> None:
        """丢弃全部手的 jump/pin/token 状态与帧屏障，回到初始状态。"""
        self.__hand_states.clear()
        self.__hand_states.update(self._new_hand_states())
        self.__frame_receiver_generation = None
        self.__last_frame_index = None

    def owned_controllers(self) -> List[object]:
        """返回当前仍被任一手的状态引用的 controller（去重、保持发现顺序）。"""
        controllers = []
        for hand_state in self.__hand_states.values():
            controller = hand_state.controller_identity
            if controller is not None and controller not in controllers:
                controllers.append(controller)
        return controllers

    def controller_for_hand(self, hand: str) -> Optional[object]:
        """返回某只手当前绑定的 controller；未绑定返回 None。"""
        return self.__hand_states[hand].controller_identity

    def controller_matches(self, hand: str, controller: object) -> bool:
        """按对象身份判断某只手是否正绑定给定 controller。"""
        return self.__hand_states[hand].controller_identity is controller

    def bind_controller(
        self,
        hand: str,
        controller: object,
        session_id: int,
        params_epoch: int,
    ) -> Optional[object]:
        """
绑定一手位的 controller（替换旧绑定），同时视为一次失效事件：立即为该手
请求 rebase，并把未完成的 pin 重绑到新 params_epoch。

Args:
    hand: 目标手。
    controller: 新 controller。
    session_id: 当前 Host 会话身份。
    params_epoch: 当前参数发布 epoch。

Returns:
    被替换掉的旧 controller；此前未绑定时为 None。
"""
        state = self.__hand_states[hand]
        previous = state.controller_identity
        state.controller_identity = controller
        self.request_rebase(
            hand,
            "controller_replacement",
            session_id,
            params_epoch,
        )
        # 参数发布是 Host 全局的、同时作用于双手。为保持旧契约，这里把所有未完成的
        # pin 重新绑定到新的 params_epoch。
        self.refresh_pins(params_epoch)
        return previous

    def release_controller(
        self,
        hand: str,
        controller: object,
        params_epoch: int,
    ) -> bool:
        """
解除一手位的 controller 绑定（仅当传入 controller 仍是当前绑定才生效）。

生效时丢弃该手的 pin 与已发出 token，并把其余 pin 重绑到新 params_epoch。

Args:
    hand: 目标手。
    controller: 期望被解绑的 controller。
    params_epoch: 当前参数发布 epoch。

Returns:
    已解绑为 True；绑定不符时为 False（保持原状）。
"""
        state = self.__hand_states[hand]
        if state.controller_identity is not controller:
            return False
        state.controller_identity = None
        state.pin = None
        state.latest_frame_new_valid = False
        state.issued_token = None
        self.refresh_pins(params_epoch)
        return True

    def build_pin(
        self,
        hand_state: _QuestHandJumpState,
        reason: str,
        session_id: int,
        params_epoch: int,
        receiver_generation: Optional[int] = None,
    ) -> Optional[_QuestRebasePin]:
        """为一次 rebase 构造 pin 快照。

        Args:
            hand_state: 目标手的当前状态。
            reason: pin 原因（会随 RebasePayload 一起交给 Host）。
            session_id: 当前 Host 会话身份。
            params_epoch: 当前参数发布 epoch。
            receiver_generation: 传输 generation；None 表示取该手最新观测值（无则为 0）。

        Returns:
            不可变 pin；若尚无可用位姿或该手未绑定 controller 则返回 None。
        """
        if (
            hand_state.latest_valid_pose is None
            or hand_state.latest_valid_fi is None
            or hand_state.controller_identity is None
        ):
            return None
        return _QuestRebasePin(
            jump_epoch=hand_state.required_epoch,
            frame_index=hand_state.latest_valid_fi,
            raw_pose=hand_state.latest_valid_pose,
            session_id=session_id,
            receiver_generation=(
                receiver_generation
                if receiver_generation is not None
                else hand_state.latest_receiver_generation or 0
            ),
            params_epoch=params_epoch,
            reason=reason,
        )

    def refresh_pins(self, params_epoch: int) -> None:
        """把所有未完成 pin 重新绑定到新的 params_epoch（其余字段不变）。"""
        for hand_state in self.__hand_states.values():
            pin = hand_state.pin
            if pin is None:
                continue
            hand_state.pin = _QuestRebasePin(
                jump_epoch=pin.jump_epoch,
                frame_index=pin.frame_index,
                raw_pose=pin.raw_pose,
                session_id=pin.session_id,
                receiver_generation=pin.receiver_generation,
                params_epoch=params_epoch,
                reason=pin.reason,
            )

    def request_rebase(
        self,
        hand: str,
        reason: str,
        session_id: int,
        params_epoch: int,
        receiver_generation: Optional[int] = None,
    ) -> None:
        """
发起一次 rebase 请求（使当前帧 token 全部失效）：递增 jump_epoch、清空已发出
token 并尝试构造 pin；无 pin 时由后续帧重试补齐。

Args:
    hand: 目标手。
    reason: 本次 rebase 的原因。
    session_id: 当前 Host 会话身份。
    params_epoch: 当前参数发布 epoch。
    receiver_generation: 本次请求所属的传输 generation。
"""
        state = self.__hand_states[hand]
        state.jump_epoch += 1
        state.required_epoch = state.jump_epoch
        # rebase 请求即是失效事件：任何为上一 epoch 发出的 token 都必须停止校验，
        # 即使该 jump epoch 可能因为同一个 frame index 被重新铸造。
        state.issued_token = None
        state.pin = self.build_pin(
            state,
            reason,
            session_id,
            params_epoch,
            receiver_generation,
        )

    @staticmethod
    def normalize_pose(pose):
        """把 pose 映射归一化为 ``_quest_pose_values`` 的 7 元 tuple（非法时 None）。"""
        return _quest_pose_values(pose)

    def process_frame(
        self,
        contexts: Iterable[QuestFrameContext],
        session_id: int,
        params_epoch: int,
        receiver_generation: int,
        position_threshold_m: float,
        rotation_threshold_deg: float,
    ) -> Tuple[QuestFrameDecision, ...]:
        """
推进一帧的双手状态并产出逐手决策。

帧屏障：同一 receiver generation 内不递增的重复帧返回空 tuple，generation 变化
则无条件重新开始。

Args:
    contexts: 同一帧内每个手位的输入快照，必须来自同一个 frame index。
    session_id: 当前 Host 会话身份。
    params_epoch: 当前参数发布 epoch。
    receiver_generation: 本帧所属传输 generation。
    position_threshold_m: 位置 jump 阈值。
    rotation_threshold_deg: 旋转 jump 阈值。

Returns:
    与 ``contexts`` 同序的逐手决策；重复帧返回空 tuple。

Raises:
    ValueError: ``contexts`` 含多个 frame index，或标记为有效的位姿不可用。
"""
        contexts = tuple(contexts)
        frame_indices = {context.frame_index for context in contexts}
        if len(frame_indices) != 1:
            raise ValueError("Quest input transaction must use one frame index")
        frame_index = next(iter(frame_indices))
        receiver_changed = self.__frame_receiver_generation != receiver_generation
        if (
            not receiver_changed
            and self.__last_frame_index is not None
            and frame_index <= self.__last_frame_index
        ):
            return ()
        self.__frame_receiver_generation = receiver_generation
        self.__last_frame_index = frame_index

        decisions = []
        for context in contexts:
            state = self.__hand_states[context.hand]
            controller = state.controller_identity
            was_rotation_locked = state.rotation_locked
            state.rotation_locked = context.rotation_locked
            if was_rotation_locked and not state.rotation_locked and context.bound_active:
                state.deferred_unlock_rebase = True

            state.latest_frame_pose_valid = False
            state.latest_frame_new_valid = False
            jump_position = None
            jump_rotation = None
            jump_triggered = False
            accept_pose = False

            if context.pose_valid:
                if context.raw_pose is None:
                    raise ValueError(f"Quest {context.hand} pose is marked valid but unusable")
                receiver_changed = state.latest_receiver_generation != receiver_generation
                different_fi = receiver_changed or context.frame_index != state.latest_valid_fi
                if different_fi:
                    previous_pose = None if receiver_changed else state.comparison_pose
                    pose_jump = False
                    if previous_pose is not None:
                        pose_jump, jump_position, jump_rotation = _quest_pose_jump(
                            previous_pose,
                            context.raw_pose,
                            position_threshold_m,
                            rotation_threshold_deg,
                            state.rotation_locked,
                        )
                    state.comparison_pose = context.raw_pose
                    state.latest_valid_pose = context.raw_pose
                    state.latest_valid_fi = context.frame_index
                    state.latest_receiver_generation = receiver_generation
                    state.latest_frame_pose_valid = True
                    state.latest_frame_new_valid = True
                    accept_pose = True

                    if context.grip > 100 and context.bound_active and pose_jump:
                        jump_triggered = True
                        self.request_rebase(
                            context.hand,
                            "pose_jump",
                            session_id,
                            params_epoch,
                            receiver_generation,
                        )
                    elif state.deferred_unlock_rebase and context.bound_active:
                        state.deferred_unlock_rebase = False
                        self.request_rebase(
                            context.hand,
                            "rotation_unlock",
                            session_id,
                            params_epoch,
                            receiver_generation,
                        )
                    elif (
                        controller is not None
                        and state.required_epoch > state.handled_jump_epoch
                        and state.pin is None
                    ):
                        state.pin = self.build_pin(
                            state,
                            "receiver_reconnect" if receiver_changed else "deferred_rebase",
                            session_id,
                            params_epoch,
                            receiver_generation,
                        )
                    elif receiver_changed and controller is not None:
                        self.request_rebase(
                            context.hand,
                            "receiver_reconnect",
                            session_id,
                            params_epoch,
                            receiver_generation,
                        )

            decisions.append(
                QuestFrameDecision(
                    hand=context.hand,
                    accept_pose=accept_pose,
                    controller=controller,
                    invalidate_controller=bool(controller is not None and state.pin is not None),
                    reject_controller=bool(
                        controller is not None
                        and state.pin is None
                        and (
                            not state.latest_frame_pose_valid
                            or not state.latest_frame_new_valid
                        )
                    ),
                    jump_triggered=jump_triggered,
                    jump_position_m=jump_position,
                    jump_rotation_deg=jump_rotation,
                )
            )
        return tuple(decisions)

    def mark_disconnected(self) -> Tuple[object, ...]:
        """输入断链：清空 pin、token 与帧有效性，并推进双手 jump epoch。

        Returns:
            断链前本 policy 持有的全部 controller，供 Host 使映射失效。
        """
        controllers = tuple(self.owned_controllers())
        for state in self.__hand_states.values():
            state.latest_frame_pose_valid = False
            state.latest_frame_new_valid = False
            state.pin = None
            state.issued_token = None
            state.deferred_unlock_rebase = False
            state.rotation_locked = False
            state.jump_epoch += 1
            state.required_epoch = state.jump_epoch
        return controllers

    def request_mapping_change(
        self,
        reason: str,
        active_hands: Iterable[str],
        session_id: int,
        params_epoch: int,
    ) -> Tuple[object, ...]:
        """映射（mode/master_side/手位）变化时，为每只已绑定的手请求 rebase。

        Args:
            reason: 本次映射变化的原因，直接作为 pin reason。
            active_hands: 需要处理的 hand 集合；未绑定的手跳过。
            session_id: 当前 Host 会话身份。
            params_epoch: 当前参数发布 epoch。

        Returns:
            被本次变化影响到的 controller（去重、保持发现顺序）。
        """
        controllers = []
        for hand in active_hands:
            controller = self.__hand_states[hand].controller_identity
            if controller is None:
                continue
            self.request_rebase(
                hand,
                reason,
                session_id,
                params_epoch,
            )
            if controller not in controllers:
                controllers.append(controller)
        return tuple(controllers)

    def prepare_control(
        self,
        hand: str,
        controller: object,
        session_id: int,
        params_epoch: int,
        bound_active: bool,
        need_sync_feedback: bool,
    ) -> QuestControlDecision:
        """
汇总一次控制决策所需的全部状态判断（先处理需 rebase 的 sync feedback）。

``ready`` 要求绑定生效、无 pin、无未处理 jump epoch、有新有效位姿且该帧未被
rebase/派发过；只有 ready 时才铸造/复用 token。

Args:
    hand: 目标手。
    controller: Host 当前认为该手绑定的 controller。
    session_id: 当前 Host 会话身份。
    params_epoch: 当前参数发布 epoch。
    bound_active: 该手在 Host 侧是否处于已绑定状态。
    need_sync_feedback: Host 是否要求重新同步 feedback。

Returns:
    本轮的 pin/pin_matches/ready 与可选 token。
"""
        state = self.__hand_states[hand]
        gate = bool(
            bound_active and state.controller_identity is controller
        )
        # ``gate`` 也是 token 身份中 controller 的那一半：token 只在该绑定成立时才会
        # 在下面被铸造/复用，而 bind_controller/release_controller 会失效该引用。
        if (
            gate
            and state.pin is None
            and state.latest_frame_pose_valid
            and state.latest_frame_new_valid
            and need_sync_feedback
        ):
            self.request_rebase(
                hand,
                "pose_mapping_invalidated",
                session_id,
                params_epoch,
                state.latest_receiver_generation,
            )
        pin = state.pin if gate else None
        pin_matches = bool(
            pin
            and pin.session_id == session_id
            and pin.receiver_generation == state.latest_receiver_generation
            and pin.params_epoch == params_epoch
            and pin.jump_epoch == state.required_epoch
        )
        ready = bool(
            gate
            and pin is None
            and state.required_epoch == state.handled_jump_epoch
            and state.latest_frame_pose_valid
            and state.latest_frame_new_valid
            and state.latest_valid_fi is not None
            and state.latest_valid_fi != state.rebase_sequence
            and state.latest_valid_fi != state.last_handled_fi
        )
        source_token = None
        if ready:
            source_token = self.issue_source_token(state)
        return QuestControlDecision(
            pin=pin,
            pin_matches=pin_matches,
            ready=ready,
            source_token=source_token,
        )

    def issue_source_token(
        self,
        state: _QuestHandJumpState,
    ) -> QuestSourceToken:
        """
返回当前帧唯一的活跃 token 并原地复用：72Hz 决策多次命中同一帧时必须交出同一
对象；只有帧绑定移动（frame index / jump epoch / receiver generation）时才铸造
新对象。
"""
        token = state.issued_token
        if (
            token is None
            or token.jump_epoch != state.required_epoch
            or token.frame_index != state.latest_valid_fi
            or token.receiver_generation != state.latest_receiver_generation
        ):
            token = QuestSourceToken(
                jump_epoch=state.required_epoch,
                frame_index=state.latest_valid_fi,
                receiver_generation=state.latest_receiver_generation,
            )
            state.issued_token = token
        return token

    def decide_control(self, context: ControlContext) -> ControlDecision:
        """
把 ``prepare_control`` 的结果翻译为中立的 ControlDecision：pin 不匹配 REJECT、
匹配 REBASE；无 pin 未 ready REJECT；否则 READY 并携带本帧 source token。

Args:
    context: Host 冻结的样本。

Returns:
    REJECT / REBASE / READY 三选一的决策。
"""
        decision = self.prepare_control(
            context.hand,
            context.controller,
            context.session_id,
            context.params_epoch,
            context.bound_active,
            context.need_sync_feedback,
        )
        if decision.pin is not None:
            if not decision.pin_matches:
                return ControlDecision(ControlAction.REJECT, None, "stale_rebase_pin")
            return ControlDecision(
                ControlAction.REBASE,
                decision.pin,
                "pose_mapping_invalidated",
                rebase_payload=RebasePayload(decision.pin.raw_pose, decision.pin.reason),
            )
        if not decision.ready:
            return ControlDecision(ControlAction.REJECT, None, "input_not_ready")
        return ControlDecision(ControlAction.READY, decision.source_token, "ready")

    def complete_control(self, completion: ControlCompletion) -> ControlResult:
        """
Host 回报结果后判定"提交"还是"过期"：REBASE_APPLIED 走 ``commit_rebase``、
DISPATCH_COMMITTED 走 ``commit_dispatch``，未提交改写为 STALE；DISPATCH_REJECTED
触发一次 ``reject_dispatch`` 并原样返回。

Args:
    completion: 原尝试身份（含 controller/session/epoch）与 Host 的实际结果。

Returns:
    policy 复核后的最终结果。
"""
        context = completion.context
        result = completion.result
        token = result.source_token
        if result.outcome is ControlOutcome.REBASE_APPLIED:
            committed = token is not None and self.commit_rebase(context.hand, token)
            return ControlResult(
                ControlOutcome.REBASE_APPLIED if committed else ControlOutcome.STALE,
                token,
                "committed" if committed else "stale",
            )
        if result.outcome is ControlOutcome.DISPATCH_COMMITTED:
            committed = token is not None and self.commit_dispatch(
                context.hand,
                context.controller,
                token,
                context.session_id,
                context.params_epoch,
            )
            return ControlResult(
                result.outcome if committed else ControlOutcome.STALE,
                token,
                "committed" if committed else "stale",
            )
        if result.outcome is ControlOutcome.DISPATCH_REJECTED:
            self.reject_dispatch(
                context.hand,
                context.session_id,
                context.params_epoch,
                token,
            )
        return result

    def commit_rebase(self, hand: str, pin: _QuestRebasePin) -> bool:
        """
提交一次已生效的 rebase：仅当 pin 仍是当前 pin 才生效；清空 pin 与已发出
token，并把 pin 帧记为已处理（不得再派发）。

Args:
    hand: 目标手。
    pin: Host 回报已应用的 pin。

Returns:
    提交成功为 True；pin 已被更新失效事件取代为 False。
"""
        state = self.__hand_states[hand]
        if state.pin is not pin:
            return False
        state.pin = None
        state.latest_frame_new_valid = False
        state.issued_token = None
        state.handled_jump_epoch = pin.jump_epoch
        state.rebase_sequence = pin.frame_index
        state.last_handled_fi = pin.frame_index
        return True

    def _validate_dispatch(
        self,
        hand: str,
        controller: object,
        source_token: Optional[QuestSourceToken],
        session_id: int,
        params_epoch: int,
        require_current_frame: bool = True,
    ) -> bool:
        """
校验一个已冻结的 Quest token；token 对 policy 不透明。

session/epoch 由 Host 在调用前比对，签名保留仅满足契约。发送前/admit
（require_current_frame=True）要求 token 是精确当前帧；发送后提交只拒绝
jump/rebase/disconnect 类失效（不对称是故意保留的）。
"""
        if source_token is None:
            return False
        state = self.__hand_states[hand]
        return bool(
            # 对象身份取代了六字段相等链：该对象仍是当前帧发出的那个 token，因为
            # 每条失效路径都会清空 state.issued_token。
            state.issued_token is source_token
            and state.controller_identity is controller
            and state.pin is None
            and state.required_epoch == state.handled_jump_epoch == source_token.jump_epoch
            and state.latest_frame_pose_valid
            and (
                not require_current_frame
                or (
                    state.latest_frame_new_valid
                    and state.latest_valid_fi == source_token.frame_index
                    and state.latest_receiver_generation == source_token.receiver_generation
                )
            )
            and source_token.frame_index != state.rebase_sequence
            and source_token.frame_index != state.last_handled_fi
        )

    def validate_dispatch(self, context: PolicyDispatchContext) -> bool:
        """按 Host 冻结的派发尝试调用 ``_validate_dispatch``（含帧新鲜度要求）。"""
        return self._validate_dispatch(
            context.hand,
            context.controller,
            context.source_token,
            context.session_id,
            context.params_epoch,
            require_current_frame=context.require_current_frame,
        )

    def commit_dispatch(
        self,
        hand: str,
        controller: object,
        source_token: QuestSourceToken,
        session_id: int,
        params_epoch: int,
    ) -> bool:
        """
提交一次已下发的派发：以发送后语义（``require_current_frame=False``）复核
token；通过后该 frame index 不得再被派发。

Args:
    hand: 目标手。
    controller: 派发时的 controller。
    source_token: 冻结的 token。
    session_id: 派发时的 Host 会话身份。
    params_epoch: 派发时的参数 epoch。

Returns:
    已提交为 True；被失效事件拒绝为 False。
"""
        if not self._validate_dispatch(
            hand,
            controller,
            source_token,
            session_id,
            params_epoch,
            require_current_frame=False,
        ):
            return False
        state = self.__hand_states[hand]
        state.last_handled_fi = source_token.frame_index
        state.latest_frame_new_valid = False
        return True

    def reject_dispatch(
        self,
        hand: str,
        session_id: int,
        params_epoch: int,
        source_token: Optional[QuestSourceToken],
    ) -> None:
        """Host 拒绝一次派发时，为该手请求一次 rebase（dispatch_rejected）。"""
        self.request_rebase(
            hand,
            "dispatch_rejected",
            session_id,
            params_epoch,
            (
                source_token.receiver_generation
                if source_token is not None
                else None
            ),
        )
