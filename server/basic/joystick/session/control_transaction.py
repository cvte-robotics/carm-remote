"""Host-owned control transaction for one arm runtime."""

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

from carm_teleop_input_contracts import (
    ControlAction,
    ControlCompletion,
    ControlContext,
    ControlOutcome,
    ControlResult,
    PolicyDispatchContext,
    HAND_RIGHT,
    normalize_hand,
)

from ..core.constants import CONTROL_LOOP_INTERVAL
from ..core.command_controller import ControlComputation, ControlSample
from ..sources.registry import BindingPhase


class ControlStepOutcome(Enum):
    NO_INPUT = "no_input"
    REBASE_APPLIED = "rebase_applied"
    COMPUTED = "computed"
    DISPATCH_COMMITTED = "dispatch_committed"
    DISPATCH_REJECTED = "dispatch_rejected"
    IDENTITY_INVALIDATED = "identity_invalidated"


class LoopTerminationKind(Enum):
    STOP_REQUESTED = "stop_requested"
    INTERNAL_ERROR = "internal_error"


@dataclass(frozen=True)
class LoopTermination:
    kind: LoopTerminationKind
    reason: str
    runtime: Any
    transaction: Any
    input_generation: Optional[int]
    control_generation: Optional[int]
    source_lease: Any
    source_controller_generation: int


class ControlTransaction:
    """Own the complete realtime control loop for one ArmRuntime."""

    def __init__(self, owner, runtime):
        self._owner = owner
        self._runtime = runtime
        self._controller = runtime.controller
        self._stop_event = threading.Event()

    def request_stop(self) -> None:
        self._stop_event.set()

    def run(self) -> LoopTermination:
        controller = self._controller
        logging.info("[ControlTransaction] Loop started for role=%s", controller.arm_role)
        while not self._stop_event.is_set():
            try:
                self._run_step()
                time.sleep(CONTROL_LOOP_INTERVAL)
            except Exception as exc:
                logging.exception(
                    "[ControlTransaction] Loop failed for role=%s",
                    controller.arm_role,
                )
                controller.halt_control()
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

    def _run_step(self) -> ControlStepOutcome:
        controller = self._controller
        arm_feedback_fresh = controller._refresh_current_pose()
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
            controller.target_end_arm_pose is not None
            and controller.carm_
            and controller.carm_.is_connected()
        ):
            return self._dispatch_pose()
        return ControlStepOutcome.COMPUTED

    def _capture_sample(self, arm_feedback_fresh: bool):
        owner = self._owner
        controller = self._controller
        normalized_hand = normalize_hand(controller.hand_binding, HAND_RIGHT)
        with owner._data_lock:
            hand_state = owner.state.hands.get(normalized_hand, {})
            binding = owner.active_source_binding
            module = owner.source_registry.active_module()
            policy = module.control_policy if module is not None else None
            input_generation = owner.input_generation
            control_generation = owner.state.control_generation
            button_data = hand_state.get("button_data")
            source_bound = owner._is_hand_bound_active_locked(
                normalized_hand,
                controller,
            )
            input_ready = bool(
                not self._stop_event.is_set()
                and
                binding is not None
                and binding.phase is BindingPhase.ACTIVE
                and binding.input_ready
                and owner.state.ble_connected
            )

            with controller.pose_state_lock:
                decision = (
                    policy.decide_control(ControlContext(
                        hand=normalized_hand,
                        controller=controller,
                        controller_generation=controller.source_controller_generation,
                        input_generation=input_generation,
                        control_generation=control_generation,
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
                if decision is not None and decision.action is ControlAction.REBASE:
                    pin = decision.source_token
                    if pin is None or button_data is None:
                        controller.reject_pose_target()
                        return ControlStepOutcome.NO_INPUT, None
                    applied = controller._apply_input_rebase_locked(
                        pin.raw_pose,
                        owner.state.calibration,
                        button_data,
                        arm_feedback_fresh,
                    )
                    if applied and policy.complete_control(ControlCompletion(
                        PolicyDispatchContext(normalized_hand, controller, controller.source_controller_generation, pin, input_generation, control_generation),
                        ControlResult(ControlOutcome.REBASE_APPLIED, pin, "rebase"),
                    )).outcome is ControlOutcome.REBASE_APPLIED:
                        logging.warning(
                            "[ControlTransaction] Quest pose rebased: hand=%s epoch=%d fi=%d reason=%s",
                            normalized_hand,
                            pin.jump_epoch,
                            pin.frame_index,
                            pin.reason,
                        )
                        return ControlStepOutcome.REBASE_APPLIED, None
                    return ControlStepOutcome.IDENTITY_INVALIDATED, None

                if decision is not None and decision.action is not ControlAction.READY:
                    controller.reject_pose_target()
                    return ControlStepOutcome.NO_INPUT, None

                return ControlStepOutcome.COMPUTED, ControlSample(
                    pose_data=hand_state.get("pose_data"),
                    button_data=button_data,
                    calibration=owner.state.calibration,
                    scale=owner.state.scale,
                    input_sequence=hand_state.get("input_sequence", 0),
                    input_generation=input_generation,
                    control_generation=control_generation,
                    input_source=owner.active_input_source,
                    source_lease=binding.lease if binding is not None else None,
                    source_token=(decision.source_token if decision is not None else None),
                    source_controller_generation=controller.source_controller_generation,
                    pose_epoch=controller.pose_epoch,
                    arm_connection_generation=controller.arm_connection_generation,
                    arm_feedback_state_sequence=controller.arm_feedback_state_sequence,
                    arm_feedback_fresh=arm_feedback_fresh,
                )

    def _host_matches_locked(self, attempt, module, lease, require_current_frame=True):
        owner = self._owner
        binding = owner.active_source_binding
        hand_state = owner.state.hands.get(attempt.hand_binding, {})
        arm_state = owner.state.arm_states.get(attempt.arm_role, {})
        button_data = hand_state.get("button_data")
        return bool(
            not self._stop_event.is_set()
            and binding is not None
            and binding.lease == lease
            and binding.phase is BindingPhase.ACTIVE
            and binding.input_ready
            and owner.active_input_source == attempt.input_source
            and owner.input_generation == attempt.input_generation
            and owner.state.ble_connected
            and arm_state.get("connected")
            and arm_state.get("hand_binding") == attempt.hand_binding
            and (
                not require_current_frame
                or hand_state.get("input_sequence") == attempt.input_sequence
            )
            and owner.state.control_generation == attempt.control_generation
            and button_data is not None
            and button_data.grip > 100
            and owner.source_registry.active_module() is module
            and module.control_policy.validate_dispatch(PolicyDispatchContext(
                hand=attempt.hand_binding,
                controller=self._controller,
                controller_generation=attempt.source_controller_generation,
                source_token=attempt.source_token,
                input_generation=attempt.input_generation,
                control_generation=attempt.control_generation,
                require_current_frame=require_current_frame,
            ))
        )

    def _dispatch_pose(self) -> ControlStepOutcome:
        owner = self._owner
        controller = self._controller
        with owner._data_lock:
            module = owner.source_registry.active_module()
            binding = owner.active_source_binding
            if module is None or binding is None:
                with controller.pose_state_lock:
                    controller.reject_pose_target()
                return ControlStepOutcome.NO_INPUT
            lease = binding.lease
            with controller.pose_state_lock:
                attempt = controller._capture_dispatch_attempt_locked()
                if attempt is None or not self._host_matches_locked(
                    attempt,
                    module,
                    lease,
                ):
                    controller.reject_pose_target()
                    return ControlStepOutcome.IDENTITY_INVALIDATED

        dispatched = controller.dispatch_pose_attempt(attempt)
        if dispatched is not True:
            logging.error(
                "[ControlTransaction] track_pose rejected for %s: %r",
                attempt.arm_role,
                dispatched,
            )

        with owner._data_lock:
            with controller.pose_state_lock:
                attempt_current = controller._dispatch_attempt_matches_locked(attempt)
                host_current = self._host_matches_locked(
                    attempt,
                    module,
                    lease,
                    require_current_frame=False,
                )
                if not attempt_current or not host_current:
                    if attempt_current:
                        controller.invalidate_pose_mapping()
                    return ControlStepOutcome.IDENTITY_INVALIDATED

                if dispatched is not True:
                    module.control_policy.complete_control(ControlCompletion(
                        PolicyDispatchContext(attempt.hand_binding, controller, attempt.source_controller_generation, attempt.source_token, attempt.input_generation, attempt.control_generation, False),
                        ControlResult(
                            ControlOutcome.DISPATCH_REJECTED,
                            attempt.source_token,
                            "physical_dispatch_failed",
                        ),
                    ))
                    controller.invalidate_pose_mapping()
                    return ControlStepOutcome.DISPATCH_REJECTED

                result = module.control_policy.complete_control(ControlCompletion(
                    PolicyDispatchContext(attempt.hand_binding, controller, attempt.source_controller_generation, attempt.source_token, attempt.input_generation, attempt.control_generation, False),
                    ControlResult(
                        ControlOutcome.DISPATCH_COMMITTED,
                        attempt.source_token,
                        "physical_dispatch_succeeded",
                    ),
                ))
                if result.outcome is not ControlOutcome.DISPATCH_COMMITTED:
                    controller.invalidate_pose_mapping()
                    return ControlStepOutcome.DISPATCH_REJECTED
                controller._commit_dispatch_attempt_locked(attempt)
                return ControlStepOutcome.DISPATCH_COMMITTED

    def _joint_attempt_matches_locked(self, attempt) -> bool:
        owner = self._owner
        controller = self._controller
        binding = owner.active_source_binding
        return bool(
            not self._stop_event.is_set()
            and binding is not None
            and binding.phase is BindingPhase.ACTIVE
            and binding.input_ready
            and owner.state.ble_connected
            and owner.input_generation == attempt.input_generation
            and owner.state.control_generation == attempt.control_generation
            and owner.active_input_source == attempt.input_source
            and not controller.isStop
            and controller.pose_epoch == attempt.pose_epoch
            and controller.arm_connection_generation == attempt.arm_connection_generation
            and controller.carm_ is attempt.carm
            and attempt.carm.ws is attempt.arm_ws_identity
            and attempt.carm.state is attempt.arm_state_identity
        )

    def _dispatch_joint_move(self, attempt) -> bool:
        owner = self._owner
        controller = self._controller
        with owner._data_lock:
            with controller.pose_state_lock:
                if not self._joint_attempt_matches_locked(attempt):
                    logging.warning(
                        "[ControlTransaction] Input disconnected before A-button move"
                    )
                    return False

        def admit() -> bool:
            with owner._data_lock:
                with controller.pose_state_lock:
                    return self._joint_attempt_matches_locked(attempt)

        return controller.dispatch_joint_move_attempt(attempt, admit)

    def _termination(
        self,
        kind: LoopTerminationKind,
        reason: str,
    ) -> LoopTermination:
        owner = self._owner
        controller = self._controller
        with owner._data_lock:
            binding = owner.active_source_binding
            return LoopTermination(
                kind=kind,
                reason=reason,
                runtime=self._runtime,
                transaction=self,
                input_generation=owner.input_generation,
                control_generation=owner.state.control_generation,
                source_lease=binding.lease if binding is not None else None,
                source_controller_generation=controller.source_controller_generation,
            )
