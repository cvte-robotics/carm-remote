"""Host-owned session cleanup orchestration."""

import logging
from dataclasses import dataclass, replace
from enum import Enum
from typing import Any, Optional, Tuple

from carm_teleop_input_contracts import SourceLease


class CleanupMode(Enum):
    HOLD = "hold"
    HOME = "home"


class CleanupStep(Enum):
    ARM_SESSION = "arm_session"
    INPUT_PROVIDER = "input_provider"
    IDLE_PROVIDER_RESOURCES = "idle_provider_resources"
    PROVIDER_RETIREMENT = "provider_retirement"
    WORKER_SHUTDOWN = "worker_shutdown"


@dataclass(frozen=True)
class CleanupContext:
    mode: CleanupMode
    reason: str
    input_generation: Optional[int]
    control_generation: Optional[int]
    source_lease: Optional[SourceLease]
    runtime_identity: Any = None
    transaction_identity: Any = None
    source_controller_generation: Optional[int] = None
    home_allowed: bool = False


@dataclass(frozen=True)
class CleanupResult:
    completed_steps: Tuple[CleanupStep, ...] = ()
    failed_steps: Tuple[CleanupStep, ...] = ()
    retryable: bool = True
    owners_retained: bool = False

    @property
    def success(self) -> bool:
        return not self.failed_steps


class SessionCleanup:
    """Coordinate one session cleanup without owning transports or workers."""

    def __init__(self, owner):
        self._owner = owner
        self._retained_context: Optional[CleanupContext] = None

    @property
    def retained_context(self) -> Optional[CleanupContext]:
        return self._retained_context

    def stop(self, context: CleanupContext) -> CleanupResult:
        if self._retained_context is not None:
            raise RuntimeError("cleanup owners are retained; use retry()")
        return self._run(context)

    def _run(self, context: CleanupContext) -> CleanupResult:
        self._retained_context = context
        completed = []
        failed = []

        operations = (
            (CleanupStep.ARM_SESSION, lambda: self._stop_arm_session(
                context.mode is CleanupMode.HOME and context.home_allowed
            )),
            (CleanupStep.INPUT_PROVIDER, lambda: self._stop_input_provider(context.reason)),
            (CleanupStep.IDLE_PROVIDER_RESOURCES,
             self._owner.source_registry.cleanup_idle_resources),
        )
        for step, operation in operations:
            try:
                success = bool(operation())
            except Exception:
                logging.exception("[SessionCleanup] Cleanup step failed: %s", step.value)
                success = False
            if success:
                completed.append(step)
            else:
                failed.append(step)

        result = CleanupResult(
            tuple(completed),
            tuple(failed),
            retryable=bool(failed),
            owners_retained=bool(failed),
        )
        if result.success:
            self._retained_context = None
        return result

    def retry(self, *, force_hold: bool = False, reason: Optional[str] = None) -> CleanupResult:
        context = self._retained_context
        if context is None:
            raise RuntimeError("no retained cleanup context")
        if force_hold and context.mode is CleanupMode.HOME:
            context = replace(
                context,
                mode=CleanupMode.HOLD,
                reason=reason or context.reason,
                home_allowed=False,
            )
        return self._run(context)

    def _stop_arm_session(self, move_home: bool) -> bool:
        owner = self._owner
        session = owner.session
        if session is None:
            owner.controller = None
            return True
        try:
            success = bool(session.stop(move_home=move_home))
        except Exception:
            logging.exception("[SessionCleanup] ArmSession cleanup failed")
            return False
        if success and not session.arms:
            owner.session = None
            owner.controller = None
        else:
            owner.controller = session.get_primary_controller()
        return success

    def _stop_input_provider(self, reason: str) -> bool:
        owner = self._owner
        lease = owner.source_registry.cleanup_lease()
        if lease is None:
            return True
        return owner._close_source_lease(lease, reason)

    def shutdown(self, reason: str) -> CleanupResult:
        completed = []
        failed = []
        try:
            provider_ok = bool(self._owner.source_registry.shutdown(reason))
        except Exception:
            logging.exception("[SessionCleanup] Provider retirement failed")
            provider_ok = False
        (completed if provider_ok else failed).append(CleanupStep.PROVIDER_RETIREMENT)
        try:
            worker_ok = bool(self._owner.shutdown_cleanup_thread())
        except Exception:
            logging.exception("[SessionCleanup] Cleanup worker shutdown failed")
            worker_ok = False
        (completed if worker_ok else failed).append(CleanupStep.WORKER_SHUTDOWN)
        return CleanupResult(tuple(completed), tuple(failed), retryable=bool(failed), owners_retained=bool(failed))
