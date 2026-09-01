"""Quest-specific input policy with no host, Carm, or transport ownership."""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from carm_teleop_input_contracts import (
    ControlAction,
    ControlCompletion,
    ControlContext,
    ControlDecision,
    ControlOutcome,
    ControlResult,
    HAND_LEFT,
    HAND_RIGHT,
    PolicyDispatchContext,
)


@dataclass(frozen=True)
class _QuestRebasePin:
    jump_epoch: int
    frame_index: int
    raw_pose: Tuple[float, ...]
    input_generation: int
    receiver_generation: int
    control_generation: int
    controller_generation: int
    reason: str


@dataclass
class _QuestHandJumpState:
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
    controller_generation: int = 0
    handled_jump_epoch: int = 0
    rebase_sequence: Optional[int] = None
    last_handled_fi: Optional[int] = None


@dataclass(frozen=True)
class QuestFrameContext:
    hand: str
    frame_index: int
    pose_valid: bool
    raw_pose: Optional[Tuple[float, ...]]
    grip: int
    rotation_locked: bool
    bound_active: bool


@dataclass(frozen=True)
class QuestFrameDecision:
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
    jump_epoch: int
    frame_index: int
    input_generation: int
    receiver_generation: int
    control_generation: int
    controller_generation: int


@dataclass(frozen=True)
class QuestControlDecision:
    pin: Optional[_QuestRebasePin]
    pin_matches: bool
    ready: bool
    source_token: Optional[QuestSourceToken] = None


def _quest_pose_values(pose):
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


class QuestInputPolicy:
    """Own every mutable Quest jump, pin, and frame-barrier transition."""

    def __init__(self):
        self.__hand_states: Dict[str, _QuestHandJumpState] = self._new_hand_states()
        self.__frame_receiver_generation: Optional[int] = None
        self.__last_frame_index: Optional[int] = None

    @staticmethod
    def _new_hand_states() -> Dict[str, _QuestHandJumpState]:
        return {
            HAND_LEFT: _QuestHandJumpState(),
            HAND_RIGHT: _QuestHandJumpState(),
        }

    def reset(self) -> None:
        self.__hand_states.clear()
        self.__hand_states.update(self._new_hand_states())
        self.__frame_receiver_generation = None
        self.__last_frame_index = None

    def owned_controllers(self) -> List[object]:
        controllers = []
        for hand_state in self.__hand_states.values():
            controller = hand_state.controller_identity
            if controller is not None and controller not in controllers:
                controllers.append(controller)
        return controllers

    def controller_for_hand(self, hand: str) -> Optional[object]:
        return self.__hand_states[hand].controller_identity

    def controller_matches(self, hand: str, controller: object) -> bool:
        return self.__hand_states[hand].controller_identity is controller

    def bind_controller(
        self,
        hand: str,
        controller: object,
        controller_generation: int,
        input_generation: int,
        control_generation: int,
    ) -> Optional[object]:
        state = self.__hand_states[hand]
        previous = state.controller_identity
        state.controller_identity = controller
        state.controller_generation = controller_generation
        self.request_rebase(
            hand,
            "controller_replacement",
            input_generation,
            control_generation,
        )
        # control_generation is global across both hands. Preserve the old
        # contract by rebinding every outstanding pin to the new generation.
        self.refresh_pins(control_generation)
        return previous

    def release_controller(
        self,
        hand: str,
        controller: object,
        control_generation: int,
    ) -> bool:
        state = self.__hand_states[hand]
        if state.controller_identity is not controller:
            return False
        state.controller_identity = None
        state.controller_generation = 0
        state.pin = None
        state.latest_frame_new_valid = False
        self.refresh_pins(control_generation)
        return True

    def build_pin(
        self,
        hand_state: _QuestHandJumpState,
        reason: str,
        input_generation: int,
        control_generation: int,
        receiver_generation: Optional[int] = None,
    ) -> Optional[_QuestRebasePin]:
        if (
            hand_state.latest_valid_pose is None
            or hand_state.latest_valid_fi is None
            or hand_state.controller_identity is None
            or hand_state.controller_generation <= 0
        ):
            return None
        return _QuestRebasePin(
            jump_epoch=hand_state.required_epoch,
            frame_index=hand_state.latest_valid_fi,
            raw_pose=hand_state.latest_valid_pose,
            input_generation=input_generation,
            receiver_generation=(
                receiver_generation
                if receiver_generation is not None
                else hand_state.latest_receiver_generation or 0
            ),
            control_generation=control_generation,
            controller_generation=hand_state.controller_generation,
            reason=reason,
        )

    def refresh_pins(self, control_generation: int) -> None:
        for hand_state in self.__hand_states.values():
            pin = hand_state.pin
            if pin is None:
                continue
            hand_state.pin = _QuestRebasePin(
                jump_epoch=pin.jump_epoch,
                frame_index=pin.frame_index,
                raw_pose=pin.raw_pose,
                input_generation=pin.input_generation,
                receiver_generation=pin.receiver_generation,
                control_generation=control_generation,
                controller_generation=pin.controller_generation,
                reason=pin.reason,
            )

    def request_rebase(
        self,
        hand: str,
        reason: str,
        input_generation: int,
        control_generation: int,
        receiver_generation: Optional[int] = None,
    ) -> None:
        state = self.__hand_states[hand]
        state.jump_epoch += 1
        state.required_epoch = state.jump_epoch
        state.pin = self.build_pin(
            state,
            reason,
            input_generation,
            control_generation,
            receiver_generation,
        )

    @staticmethod
    def normalize_pose(pose):
        return _quest_pose_values(pose)

    def process_frame(
        self,
        contexts: Iterable[QuestFrameContext],
        input_generation: int,
        control_generation: int,
        receiver_generation: int,
        position_threshold_m: float,
        rotation_threshold_deg: float,
    ) -> Tuple[QuestFrameDecision, ...]:
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
                            input_generation,
                            control_generation,
                            receiver_generation,
                        )
                    elif state.deferred_unlock_rebase and context.bound_active:
                        state.deferred_unlock_rebase = False
                        self.request_rebase(
                            context.hand,
                            "rotation_unlock",
                            input_generation,
                            control_generation,
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
                            input_generation,
                            control_generation,
                            receiver_generation,
                        )
                    elif receiver_changed and controller is not None:
                        self.request_rebase(
                            context.hand,
                            "receiver_reconnect",
                            input_generation,
                            control_generation,
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
        controllers = tuple(self.owned_controllers())
        for state in self.__hand_states.values():
            state.latest_frame_pose_valid = False
            state.latest_frame_new_valid = False
            state.pin = None
            state.deferred_unlock_rebase = False
            state.rotation_locked = False
            state.jump_epoch += 1
            state.required_epoch = state.jump_epoch
        return controllers

    def request_mapping_change(
        self,
        reason: str,
        active_hands: Iterable[str],
        input_generation: int,
        control_generation: int,
    ) -> Tuple[object, ...]:
        controllers = []
        for hand in active_hands:
            controller = self.__hand_states[hand].controller_identity
            if controller is None:
                continue
            self.request_rebase(
                hand,
                reason,
                input_generation,
                control_generation,
            )
            if controller not in controllers:
                controllers.append(controller)
        return tuple(controllers)

    def prepare_control(
        self,
        hand: str,
        controller: object,
        controller_generation: int,
        input_generation: int,
        control_generation: int,
        bound_active: bool,
        need_sync_feedback: bool,
    ) -> QuestControlDecision:
        state = self.__hand_states[hand]
        gate = bool(
            bound_active
            and state.controller_identity is controller
            and state.controller_generation == controller_generation
        )
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
                input_generation,
                control_generation,
                state.latest_receiver_generation,
            )
        pin = state.pin if gate else None
        pin_matches = bool(
            pin
            and pin.input_generation == input_generation
            and pin.receiver_generation == state.latest_receiver_generation
            and pin.control_generation == control_generation
            and pin.controller_generation == controller_generation
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
            source_token = QuestSourceToken(
                jump_epoch=state.required_epoch,
                frame_index=state.latest_valid_fi,
                input_generation=input_generation,
                receiver_generation=state.latest_receiver_generation,
                control_generation=control_generation,
                controller_generation=state.controller_generation,
            )
        return QuestControlDecision(
            pin=pin,
            pin_matches=pin_matches,
            ready=ready,
            source_token=source_token,
        )

    def decide_control(self, context: ControlContext) -> ControlDecision:
        decision = self.prepare_control(
            context.hand,
            context.controller,
            context.controller_generation,
            context.input_generation,
            context.control_generation,
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
                decision.pin.raw_pose,
            )
        if not decision.ready:
            return ControlDecision(ControlAction.REJECT, None, "input_not_ready")
        return ControlDecision(ControlAction.READY, decision.source_token, "ready")

    def complete_control(self, completion: ControlCompletion) -> ControlResult:
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
                context.controller_generation,
                token,
                context.input_generation,
                context.control_generation,
            )
            return ControlResult(
                result.outcome if committed else ControlOutcome.STALE,
                token,
                "committed" if committed else "stale",
            )
        if result.outcome is ControlOutcome.DISPATCH_REJECTED:
            self.reject_dispatch(
                context.hand,
                context.input_generation,
                context.control_generation,
                token,
            )
        return result

    def commit_rebase(self, hand: str, pin: _QuestRebasePin) -> bool:
        state = self.__hand_states[hand]
        if state.pin is not pin:
            return False
        state.pin = None
        state.latest_frame_new_valid = False
        state.handled_jump_epoch = pin.jump_epoch
        state.rebase_sequence = pin.frame_index
        state.last_handled_fi = pin.frame_index
        return True

    def _validate_dispatch(
        self,
        hand: str,
        controller: object,
        controller_generation: int,
        source_token: Optional[QuestSourceToken],
        input_generation: int,
        control_generation: int,
        require_current_frame: bool = True,
    ) -> bool:
        if source_token is None:
            return False
        state = self.__hand_states[hand]
        return bool(
            state.controller_identity is controller
            and state.controller_generation == controller_generation
            and source_token.controller_generation == controller_generation
            and source_token.input_generation == input_generation
            and source_token.control_generation == control_generation
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
        return self._validate_dispatch(
            context.hand,
            context.controller,
            context.controller_generation,
            context.source_token,
            context.input_generation,
            context.control_generation,
            require_current_frame=context.require_current_frame,
        )

    def commit_dispatch(
        self,
        hand: str,
        controller: object,
        controller_generation: int,
        source_token: QuestSourceToken,
        input_generation: int,
        control_generation: int,
    ) -> bool:
        if not self._validate_dispatch(
            hand,
            controller,
            controller_generation,
            source_token,
            input_generation,
            control_generation,
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
        input_generation: int,
        control_generation: int,
        source_token: Optional[QuestSourceToken],
    ) -> None:
        self.request_rebase(
            hand,
            "dispatch_rejected",
            input_generation,
            control_generation,
            (
                source_token.receiver_generation
                if source_token is not None
                else None
            ),
        )
