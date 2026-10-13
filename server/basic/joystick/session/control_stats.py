"""控制环运行统计：按固定窗口汇总周期、步时、派发与输入节拍，输出一行日志。

只由所属控制线程调用（无锁）；不参与任何控制判断。
"""

import logging
import time
from typing import List, Optional

_STATS_WINDOW_S = 5.0
# 相邻两次派发间隔超过该值视为控制暂停，不把期间的输入序号差计为跳帧。
_CONTINUOUS_GAP_S = 0.1


def percentile(sorted_values: List[float], fraction: float) -> float:
    """返回已排序序列的近似分位数（空序列为 0.0）。"""
    if not sorted_values:
        return 0.0
    index = int(round(fraction * (len(sorted_values) - 1)))
    return sorted_values[max(0, min(len(sorted_values) - 1, index))]


class ControlLoopStats:
    """单臂控制环的窗口统计。

    指标：控制周期（相邻两步开始时刻之差）、步时、派发频率、各结束状态次数、
    观察到的输入帧频率（按 host 每手 ``input_sequence`` 推进量）、派发时跳过的
    输入帧数，以及"帧发布 → 物理派发"延迟。
    """

    def __init__(self, role: str, window_s: float = _STATS_WINDOW_S, clock=time.monotonic):
        """绑定臂角色与统计窗口长度。"""
        self.role = role
        self._window_s = float(window_s)
        self._clock = clock
        self._last_dispatched_sequence: Optional[int] = None
        self._last_dispatch_monotonic: Optional[float] = None
        self._reset(clock())

    def _reset(self, now: float) -> None:
        """开始一个新窗口（保留跨窗口的派发序号基线）。"""
        self._window_start = now
        self._last_step_start: Optional[float] = None
        self._periods_ms: List[float] = []
        self._steps_ms: List[float] = []
        self._latencies_ms: List[float] = []
        self._outcomes = {}
        self._dispatches = 0
        self._skipped_frames = 0
        self._first_input_sequence: Optional[int] = None
        self._last_input_sequence: Optional[int] = None

    def record_step(self, started: float, finished: float, outcome) -> None:
        """记录一步：开始/结束时刻与结束状态。"""
        if self._last_step_start is not None:
            self._periods_ms.append((started - self._last_step_start) * 1000.0)
        self._last_step_start = started
        self._steps_ms.append((finished - started) * 1000.0)
        name = getattr(outcome, "value", str(outcome))
        self._outcomes[name] = self._outcomes.get(name, 0) + 1

    def record_input(self, input_sequence: Optional[int]) -> None:
        """记录本步看到的输入帧序号（每手单调递增）。"""
        if input_sequence is None:
            return
        if self._first_input_sequence is None:
            self._first_input_sequence = input_sequence
        self._last_input_sequence = input_sequence

    def record_dispatch(self, input_sequence: Optional[int], published_monotonic: Optional[float],
                        dispatched_monotonic: float) -> None:
        """记录一次已提交的物理派发及其输入帧。"""
        self._dispatches += 1
        if published_monotonic is not None:
            self._latencies_ms.append((dispatched_monotonic - published_monotonic) * 1000.0)
        if input_sequence is not None:
            previous = self._last_dispatched_sequence
            # 控制暂停（如松开 grip）之后的序号差不是跳帧，只统计连续派发段。
            continuous = (
                self._last_dispatch_monotonic is not None
                and dispatched_monotonic - self._last_dispatch_monotonic <= _CONTINUOUS_GAP_S
            )
            if continuous and previous is not None and input_sequence > previous + 1:
                self._skipped_frames += input_sequence - previous - 1
            self._last_dispatched_sequence = input_sequence
        self._last_dispatch_monotonic = dispatched_monotonic

    def maybe_report(self, now: Optional[float] = None) -> bool:
        """窗口到期时输出一行统计并开新窗口；无输入且无派发的窗口静默丢弃。

        Returns: 本次是否输出了日志。
        """
        now = self._clock() if now is None else now
        elapsed = now - self._window_start
        if elapsed < self._window_s:
            return False
        input_advance = 0
        if self._first_input_sequence is not None and self._last_input_sequence is not None:
            input_advance = max(0, self._last_input_sequence - self._first_input_sequence)
        reported = False
        if self._dispatches or input_advance:
            periods = sorted(self._periods_ms)
            steps = sorted(self._steps_ms)
            latencies = sorted(self._latencies_ms)
            # logging.info(
            #     "[ControlStats] role=%s window=%.1fs loop=%.1fHz period_ms p50/p95/max=%.1f/%.1f/%.1f "
            #     "step_ms p50/p95/max=%.2f/%.2f/%.2f dispatch=%.1fHz input=%.1fHz skipped_frames=%d "
            #     "publish_to_dispatch_ms p50/p95/max=%.1f/%.1f/%.1f outcomes=%s",
            #     self.role, elapsed, len(steps) / elapsed,
            #     percentile(periods, 0.50), percentile(periods, 0.95), periods[-1] if periods else 0.0,
            #     percentile(steps, 0.50), percentile(steps, 0.95), steps[-1] if steps else 0.0,
            #     self._dispatches / elapsed, input_advance / elapsed, self._skipped_frames,
            #     percentile(latencies, 0.50), percentile(latencies, 0.95),
            #     latencies[-1] if latencies else 0.0,
            #     ",".join(f"{key}={value}" for key, value in sorted(self._outcomes.items())),
            # )
            reported = True
        self._reset(now)
        return reported
