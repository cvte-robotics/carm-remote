"""遥操中回位运动的完成判定，不依赖预估的到达时间（PVT 与按速度规划的 MOVJ 通用）。

控制器在任务结束后的下一帧发出该任务的 ``last_task_key``，SDK 以 ``on_task_finish`` 回调；
任务被顶替或中止时同样会发，所以结束后还要核对实测位置。控制器也可能不产生任务（如 PVT
的目标就是当前位置），因此任务不在执行且持续静止时也视为运动已结束。
"""

# 实测结束帧的位置误差：MOVJ 不超过 0.0077 rad，5 s PVT 不超过 0.0155 rad。
ARRIVAL_TOLERANCE_RAD = 0.03
STALL_VELOCITY_RAD_S = 0.02
# 远大于任务启动延迟（实测 12~42 ms）。
STALL_TIMEOUT_S = 1.0


class JointMotionWatch:
    """一次关节运动的完成判定，只由发起运动的控制线程调用。"""

    def __init__(self, carm, task_key, target):
        self.carm, self.task_key = carm, task_key
        self.target = tuple(target)
        self._still_since = None

    def poll(self, now, position, velocity, executing):
        """用一帧新鲜反馈判定运动是否结束。

        Args: now 单调时钟；position/velocity 实测关节角与速度；executing 控制器是否在执行任务。
        Returns: 未结束为 None；已结束时到位为 True，未到位为 False。
        """
        arrived = all(abs(p - t) <= ARRIVAL_TOLERANCE_RAD for p, t in zip(position, self.target))
        if self.carm.last_finished_task_key == self.task_key:
            return arrived
        if executing or any(abs(v) > STALL_VELOCITY_RAD_S for v in velocity):
            self._still_since = None
            return None
        if self._still_since is None:
            self._still_since = now
        return arrived if now - self._still_since >= STALL_TIMEOUT_S else None
