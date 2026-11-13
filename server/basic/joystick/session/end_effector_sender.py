"""每臂一个的末端发送器：在控制线程内按"值变化或保活到期"就地下发。

末端驱动只发不等（Carm 夹爪走 ``sdk_nowait`` 下的 ``set_eeff``，透传帧走 ``send_only``），
不再阻塞控制周期，因此不需要独立线程。控制环在位姿成功后调用 ``set_eff(trigger)``：
值未变且未到保活间隔时跳过；驱动返回失败时清空基线，下一次提交必定重发。
"""

import logging
import time

from .control_stats import percentile

_KEEPALIVE_S = 0.5
_STATS_WINDOW_S = 5.0


class EndEffectorSender:
    """包装一个已校验的 ``EndEffectorBase`` 驱动，对控制环暴露去重与保活后的 ``set_eff``。

    线程模型：``set_eff``/``invalidate`` 只由所属臂的控制线程调用，``stop`` 在控制线程
    join 之后调用，因此不加锁。
    """

    def __init__(self, driver, role: str, keepalive_s: float = _KEEPALIVE_S,
                 clock=time.monotonic):
        """绑定真实驱动与臂角色。"""
        self.driver = driver
        self.role = role
        self._keepalive_s = float(keepalive_s)
        self._clock = clock
        self._last_sent = None
        self._last_sent_at = None
        self._stopping = False
        self._window_start = clock()
        self._submitted = 0
        self._sent = 0
        self._skipped = 0
        self._failed = 0
        self._durations_ms = []

    def set_eff(self, trigger) -> bool:
        """按需下发 trigger；停止后拒绝。

        Returns: 已下发或因去重跳过时为 True；驱动未接受、抛异常或已停止时为 False。
        """
        if self._stopping:
            return False
        now = self._clock()
        self._submitted += 1
        if (self._last_sent is not None and trigger == self._last_sent
                and now - self._last_sent_at < self._keepalive_s):
            self._skipped += 1
            self._report(now)
            return True
        accepted = False
        try:
            accepted = self.driver.set_eff(trigger) is True
            if not accepted:
                logging.warning("[EndEffectorSender] %s end-effector send was not accepted", self.role)
        except Exception:
            logging.exception("[EndEffectorSender] %s end-effector send raised", self.role)
        finished = self._clock()
        self._durations_ms.append((finished - now) * 1000.0)
        if accepted:
            self._sent += 1
            self._last_sent, self._last_sent_at = trigger, finished
        else:
            self._failed += 1
            self._last_sent = self._last_sent_at = None
        self._report(finished)
        return accepted

    def invalidate(self) -> None:
        """让下一次提交必定发送（halt/停止路径）。"""
        self._last_sent = self._last_sent_at = None

    def stop(self) -> bool:
        """停止接受提交；没有在途调用，恒为 True。"""
        self._stopping = True
        self.invalidate()
        return True

    def _report(self, now: float) -> None:
        """统计窗口到期时输出一行（窗口内无提交则静默重置）。"""
        if now - self._window_start < _STATS_WINDOW_S:
            return
        elapsed = now - self._window_start
        if self._submitted:
            durations = sorted(self._durations_ms)
            logging.info(
                "[EndEffectorStats] role=%s window=%.1fs submit=%.1fHz sent=%d skipped=%d "
                "failed=%d send_ms p50/p95/max=%.2f/%.2f/%.2f",
                self.role, elapsed, self._submitted / elapsed, self._sent,
                self._skipped, self._failed, percentile(durations, 0.50),
                percentile(durations, 0.95), durations[-1] if durations else 0.0,
            )
        self._window_start = now
        self._submitted = self._sent = self._skipped = self._failed = 0
        self._durations_ms = []
