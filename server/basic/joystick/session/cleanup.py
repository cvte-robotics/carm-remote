"""Host 持有的会话清理（cleanup）编排。"""

import logging
from dataclasses import dataclass, replace
from enum import Enum
from typing import Optional, Tuple


class CleanupMode(Enum):
    """清理动作模式：HOLD 就地保持，HOME 先回零点。"""

    HOLD = "hold"
    HOME = "home"


class CleanupStep(Enum):
    """清理步骤标识，用于在 CleanupResult 中报告完成/失败。"""

    ARM_SESSION = "arm_session"
    INPUT_PROVIDER = "input_provider"
    IDLE_PROVIDER_RESOURCES = "idle_provider_resources"
    PROVIDER_RETIREMENT = "provider_retirement"
    WORKER_SHUTDOWN = "worker_shutdown"


@dataclass(frozen=True)
class CleanupContext:
    """一次清理调用的不可变上下文（模式、原因与 home 许可）。"""

    mode: CleanupMode
    reason: str
    home_allowed: bool = False


@dataclass(frozen=True)
class CleanupResult:
    """一次清理调用的结果：已完成/失败步骤与 owner 保留标记。"""

    completed_steps: Tuple[CleanupStep, ...] = ()
    failed_steps: Tuple[CleanupStep, ...] = ()
    retryable: bool = True
    owners_retained: bool = False

    @property
    def success(self) -> bool:
        """全部步骤均未失败时为 True。"""
        return not self.failed_steps


class SessionCleanup:
    """编排单次会话清理，但不持有 transport 与 worker。"""

    def __init__(self, owner):
        """绑定 Host owner 并清空保留的清理上下文。"""
        self._owner = owner
        self._retained_context: Optional[CleanupContext] = None

    @property
    def retained_context(self) -> Optional[CleanupContext]:
        """返回尚未成功收尾、仍需 retry 的清理上下文。"""
        return self._retained_context

    def stop(self, context: CleanupContext) -> CleanupResult:
        """执行一次清理；若上一次仍保留 owner 则拒绝并提示改用 retry。

        Args: context 本次清理上下文。
        Returns: CleanupResult。
        Raises: RuntimeError 上一次清理仍保留 owner（owners_retained）。
        """
        if self._retained_context is not None:
            raise RuntimeError("cleanup owners are retained; use retry()")
        return self._run(context)

    def _run(self, context: CleanupContext) -> CleanupResult:
        """按固定顺序执行清理步骤，并记录完成/失败步骤。

        失败时保留 ``_retained_context`` 与 owner（``owners_retained=True``），
        使调用方可通过 :meth:`retry` 继续收尾；只有全部步骤成功才释放保留上下文。
        """
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
        """用保留的上下文重试清理；force_hold 可把 HOME 降级为 HOLD。

        Args: force_hold 是否强制改为就地保持；reason 覆盖后的原因文案。
        Returns: CleanupResult。
        Raises: RuntimeError 当前没有可重试的保留上下文。
        """
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
        """停止 ArmSession，并同步 owner 的 session/controller 归属。"""
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
        """关闭当前 provider lease（无 lease 时视为已完成）。"""
        owner = self._owner
        lease = owner.source_registry.cleanup_lease()
        if lease is None:
            return True
        return owner._close_source_lease(lease, reason)

    def shutdown(self, reason: str) -> CleanupResult:
        """进程退出路径：停用 provider 并关闭清理 worker。

        Args: reason 停用原因。
        Returns: CleanupResult（失败步骤同样保留 owner）。
        """
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
