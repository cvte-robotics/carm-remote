"""手臂与腰共用的增量位姿映射：目标 = 输出基线 + (当前输入 - 输入基线)。

两者的差别由调用方决定：手臂姿态 1:1、灵敏度只缩放位置，腰的位置和姿态都按灵敏度缩放；
触发方式、中断处理和发送也各归各的。这里只负责基线与目标计算，由所属控制线程独占读写。
"""

import math

from .pose_math import _continuous_pose_diff_with_scale, _copy_pose4, apply_pose_delta


# 上次目标与新鲜反馈的偏差上限；超过说明目标到不了或有外部运动，要从实际位姿接管。
MAX_TARGET_POSITION_DEVIATION_M = 0.05
MAX_TARGET_ROTATION_DEVIATION_RAD = math.radians(10.0)


def _near(first, second) -> bool:
    """两个已归一化的七分量位姿是否在偏差上限内（不区分四元数整体符号）。"""
    if math.dist(first[:3], second[:3]) > MAX_TARGET_POSITION_DEVIATION_M:
        return False
    dot = abs(sum(a * b for a, b in zip(first[3:7], second[3:7])))
    return 2.0 * math.acos(min(1.0, dot)) <= MAX_TARGET_ROTATION_DEVIATION_RAD


def select_output_baseline(last_target, tracking, feedback):
    """选择重建基线时的输出位姿。

    控制器还在跟踪时反馈落后于目标，取反馈会把目标往回拉；已经不跟踪时手臂或腰就停在
    实际位置，取旧目标可能跳一下。

    Args: last_target 最近一次下发的目标（没有或已失效为 None）；tracking 控制器是否还在
        执行任务；feedback 新鲜的实际位姿（没有为 None）。
    Returns: 七分量位姿；两者都不可用时为 None，调用方等下一帧。
    """
    if last_target is not None and tracking and (feedback is None or _near(last_target, feedback)):
        return tuple(last_target)
    return None if feedback is None else tuple(feedback[:7])


class PoseMapping:
    """一段连续跟随的基线与四元数连续分支。"""

    def __init__(self):
        self.input_base = None
        self.output_base = None
        self.relative_quaternion = None

    @property
    def ready(self) -> bool:
        """是否已建立基线。"""
        return self.output_base is not None

    def rebase(self, input_pose, output_pose) -> None:
        """以本周期输入位姿（Pose4）和给定输出位姿重建基线。"""
        self.input_base = _copy_pose4(input_pose)
        self.output_base = tuple(output_pose[:7])
        self.relative_quaternion = None

    def reset_branch(self) -> None:
        """丢弃四元数连续分支；基线保留。"""
        self.relative_quaternion = None

    def target(self, input_pose, position_scale, rotation_scale, rotation_locked=False):
        """由当前输入位姿（Pose4）算出七分量目标。

        Returns: 目标位姿；相对旋转落入未定义分支时为 None，调用方应重建基线。
        """
        position_delta = tuple(
            (input_pose[index][3] - self.input_base[index][3]) * position_scale
            for index in range(3)
        )
        if rotation_locked:
            self.relative_quaternion = None
            rotation_delta = (0.0, 0.0, 0.0, 1.0)
        else:
            _, rotation_delta, relative, _, ambiguous = _continuous_pose_diff_with_scale(
                self.input_base, input_pose, rotation_scale, self.relative_quaternion,
            )
            if ambiguous:
                return None
            self.relative_quaternion = relative
        return tuple(apply_pose_delta(self.output_base, position_delta, rotation_delta)[:7])
