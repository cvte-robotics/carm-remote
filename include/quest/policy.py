"""Quest 输入线程的跳变检测、旋转锁边沿和帧序号屏障。

只输出断续事件，不读取或改写控制器的映射状态。比较状态由输入线程独占；Host
只替换绑定引用或发布重置凭据，输入线程在下一帧接收凭据。
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, InputUpdateDecision, RawInputBatch


@dataclass
class _QuestHandState:
    """仅用于比较相邻输入帧的单手策略状态。"""

    comparison_pose: Optional[Tuple[float, ...]] = None
    receiver_generation: Optional[int] = None
    deferred_unlock_rebase: bool = False
    rotation_locked: bool = False


def _quest_pose_values(pose):
    """直接归一化边界已校验的 PoseSample，不再复查字段或构造映射。"""
    quaternion = (pose.qx, pose.qy, pose.qz, pose.qw)
    quaternion_norm = math.sqrt(sum(value ** 2 for value in quaternion))
    return (pose.x, pose.y, pose.z) + tuple(value / quaternion_norm for value in quaternion)


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


class QuestInputPolicy:
    """只持有输入检测状态，不参与计算、发送或提交。"""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        """Host 清空绑定并发布重置凭据，不改写输入线程的比较状态。"""
        self._controllers = {HAND_LEFT: None, HAND_RIGHT: None}
        self._reset_token = object()

    def owned_controllers(self):
        """返回当前绑定的控制器身份，保持首次出现顺序。"""
        controllers = []
        for controller in self._controllers.values():
            if controller is not None and not any(item is controller for item in controllers):
                controllers.append(controller)
        return controllers

    def controller_for_hand(self, hand):
        """只读取绑定身份，不读取控制器状态。"""
        return self._controllers[hand]

    def controller_matches(self, hand, controller):
        """判断绑定身份。"""
        return self.controller_for_hand(hand) is controller

    def bind_controller(self, hand, controller):
        """Host 在控制线程停止后一次替换该手的绑定引用。"""
        previous = self._controllers[hand]
        self._controllers[hand] = controller
        return previous

    def release_controller(self, hand, controller):
        """Host 在控制线程停止后按身份解绑。"""
        if self._controllers[hand] is not controller:
            return False
        self._controllers[hand] = None
        return True

    def process_frame(
        self, batch: RawInputBatch, bound_hands,
        position_threshold_m: float, rotation_threshold_deg: float,
    ) -> Tuple[InputUpdateDecision, ...]:
        """直接读取已校验的逐手输入并返回决策；重复帧不发布，断续事件仅当帧置位。"""
        token = self._reset_token
        if token is not getattr(self, "_handled_reset_token", None):
            self._hands = {hand: _QuestHandState() for hand in (HAND_LEFT, HAND_RIGHT)}
            self._receiver_generation = None
            self._last_frame_index = None
            self._handled_reset_token = token
        frame_index = batch.batch_sequence
        receiver_generation = batch.transport_generation
        if (self._receiver_generation == receiver_generation
                and self._last_frame_index is not None
                and frame_index <= self._last_frame_index):
            return ()
        self._receiver_generation = receiver_generation
        self._last_frame_index = frame_index
        decisions = []
        for update in batch.hands:
            bound_active = update.hand in bound_hands
            rotation_locked = update.intent is not None and update.intent.rotation_locked
            state = self._hands[update.hand]
            controller = self._controllers[update.hand]
            if state.rotation_locked and not rotation_locked and bound_active:
                state.deferred_unlock_rebase = True
            state.rotation_locked = rotation_locked
            event = False
            jump = False
            position = rotation = None
            if update.pose_valid:
                raw_pose = _quest_pose_values(update.pose)
                receiver_changed = state.receiver_generation != receiver_generation
                if not receiver_changed and state.comparison_pose is not None:
                    jump, position, rotation = _quest_pose_jump(
                        state.comparison_pose, raw_pose, position_threshold_m,
                        rotation_threshold_deg, state.rotation_locked,
                    )
                state.comparison_pose = raw_pose
                state.receiver_generation = receiver_generation
                jump = bool(jump and update.intent.motion_engaged and bound_active)
                if jump:
                    event = True
                elif state.deferred_unlock_rebase and bound_active:
                    state.deferred_unlock_rebase = False
                    event = True
                elif receiver_changed and controller is not None:
                    event = True
            details = ()
            if jump:
                details = (
                    ("event", "pose_jump"),
                    ("frame_index", frame_index),
                    ("position_m", position),
                    ("rotation_deg", rotation),
                )
            decisions.append(InputUpdateDecision(
                hand=update.hand, accept_pose=update.pose_valid, accept_button=True,
                controller=controller, invalidate_controller=event,
                reject_controller=bool(controller is not None and not update.pose_valid),
                details=details,
            ))
        return tuple(decisions)

    def mark_disconnected(self):
        """Host 发布重置凭据，下一输入帧再丢弃比较基准与边沿。"""
        self._reset_token = object()
        return tuple(self.owned_controllers())
