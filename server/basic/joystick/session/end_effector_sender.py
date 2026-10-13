"""每臂一个的末端异步发送器：控制环只提交最新 trigger，SDK 阻塞调用在独立线程执行。

SDK 的 ``set_eeff`` / 透传接口是 request-response（单次最多阻塞 1s），放在控制环
里会拉长控制周期并在控制器应答慢时让臂收不到跟踪目标。本发送器是最新值单槽：
控制环在 strict pose 成功后调用 ``set_eff(trigger)`` 只做提交；worker 在值变化或
保活间隔到期时调用真实驱动，失败则在下一次提交时重发。
"""

import logging
import threading
import time
from typing import Optional

from .control_stats import percentile

_KEEPALIVE_S = 0.5
_STATS_WINDOW_S = 5.0


class AsyncEndEffectorSender:
    """包装一个已校验的 ``EndEffectorBase`` 驱动，对控制环暴露非阻塞 ``set_eff``。

    线程模型：``set_eff``/``invalidate`` 由控制线程或停止路径调用，只在内部
    条件锁内改单槽；真实驱动只在 worker 线程调用。内部锁是叶子锁。
    """

    def __init__(self, driver, role: str, keepalive_s: float = _KEEPALIVE_S,
                 clock=time.monotonic):
        """绑定真实驱动与臂角色（不启动线程）。"""
        self.driver = driver
        self.role = role
        self._keepalive_s = float(keepalive_s)
        self._clock = clock
        self._condition = threading.Condition(threading.Lock())
        self._pending: Optional[float] = None
        self._last_sent: Optional[float] = None
        self._last_sent_at: Optional[float] = None
        # invalidate 计数：在途发送完成时若计数已变，不把结果记为"已发送基线"。
        self._invalidations = 0
        self._stopping = False
        self._thread: Optional[threading.Thread] = None
        self._window_start = clock()
        self._submitted = 0
        self._sent = 0
        self._skipped = 0
        self._failed = 0
        self._durations_ms = []

    def start(self) -> None:
        """启动 worker 线程（幂等；已停止的发送器不可重启）。"""
        with self._condition:
            if self._thread is not None or self._stopping:
                return
            self._thread = threading.Thread(
                target=self._run,
                daemon=True,
                name=f"JoystickEndEffector-{self.role}",
            )
            self._thread.start()

    def set_eff(self, trigger) -> bool:
        """提交最新 trigger（不调用 SDK）；停止后拒绝提交。

        Returns: 已进入单槽时为 True；发送器已停止时为 False。
        """
        with self._condition:
            if self._stopping:
                return False
            self._pending = trigger
            self._submitted += 1
            self._condition.notify()
        return True

    def invalidate(self) -> None:
        """丢弃尚未发送的 trigger，并让下一次提交必定发送（halt/停止路径）。"""
        with self._condition:
            self._pending = None
            self._last_sent = None
            self._last_sent_at = None
            self._invalidations += 1

    def stop(self, timeout: float = 2.0) -> bool:
        """停止 worker 并丢弃待发值；正在进行的 SDK 调用会被等待至 timeout。

        Returns: worker 已退出（或从未启动）时为 True；仍在 SDK 调用中时为 False，
        调用方可重试。
        """
        with self._condition:
            self._stopping = True
            self._pending = None
            self._condition.notify()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=timeout)
            if thread.is_alive():
                return False
        return True

    def _take_pending(self):
        """等待下一个需要真实发送的值。

        Returns: (True, trigger, invalidations) 或停止时 (False, None, None)。
        """
        with self._condition:
            while True:
                if self._stopping:
                    return False, None, None
                if self._pending is not None:
                    trigger = self._pending
                    self._pending = None
                    now = self._clock()
                    if (
                        self._last_sent is not None
                        and trigger == self._last_sent
                        and self._last_sent_at is not None
                        and now - self._last_sent_at < self._keepalive_s
                    ):
                        self._skipped += 1
                        continue
                    return True, trigger, self._invalidations
                self._condition.wait(timeout=_STATS_WINDOW_S)
                self._report_locked(self._clock())

    def _run(self) -> None:
        """worker 主循环：取值 → 调用真实驱动 → 记录结果。"""
        while True:
            ok, trigger, invalidations = self._take_pending()
            if not ok:
                return
            started = self._clock()
            accepted = False
            try:
                accepted = self.driver.set_eff(trigger) is True
                if not accepted:
                    logging.warning(
                        "[EndEffectorSender] %s end-effector send was not accepted",
                        self.role,
                    )
            except Exception:
                logging.exception(
                    "[EndEffectorSender] %s end-effector send raised", self.role,
                )
            finished = self._clock()
            with self._condition:
                self._durations_ms.append((finished - started) * 1000.0)
                if accepted:
                    self._sent += 1
                    if invalidations == self._invalidations:
                        self._last_sent = trigger
                        self._last_sent_at = finished
                else:
                    self._failed += 1
                    self._last_sent = None
                    self._last_sent_at = None
                self._report_locked(finished)

    def _report_locked(self, now: float) -> None:
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
