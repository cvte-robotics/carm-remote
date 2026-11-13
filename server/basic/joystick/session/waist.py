"""D3 统一腰控制运行时：按每帧输入方式在头盔位姿与腰摇杆之间切换。

腰与左右臂并列，由 ArmSession 持有，使用同一控制板的 ``arm_index=2`` 通道，不进入机械臂的
握把或位姿映射状态。一个 ``WaistRuntime`` 只有一个按输入帧唤醒的控制线程；Session
必须先停掉该线程，才能在同一个 Carm 上回零与释放，保证同一通道任意时刻只有一个运动所有者。

头盔方式使用 ``track_pose`` 与手柄相同的映射链路；摇杆方式用死区、线性角速度和单调时间积分
生成关节目标，再通过 ``track_joint`` 下发。腰回位键回到 ``teleop_reset_joint``，停止遥操
回到 ``home_joint``。输入、反馈、重定位、换代或模式变化造成的中断都必须再次触发腰跟随键。
"""

import logging
import math
import threading
import time
from dataclasses import dataclass

from quest.config import (
    DEFAULT_QUEST_POSITION_THRESHOLD_M,
    DEFAULT_QUEST_ROTATION_THRESHOLD_DEG,
)

from ..core.joint_motion import JointMotionWatch
from ..core.pose_mapping import PoseMapping, select_output_baseline
from ..core.pose_math import input_pose_to_arm
from ..runtime.dependencies import Carm, sdk_nowait
from ..runtime.carm_link import CarmChannel
from ..sources.registry import BindingPhase


# 无新帧时的最长等待，与双臂控制环周期一致；有发布信号时按新帧唤醒，频率跟随 APK 发送频率。
LOOP_INTERVAL_S = 1.0 / 72.0
# 头盔帧新鲜度门限；TCP 心跳不能代替帧更新。
INPUT_TIMEOUT_S = 0.2
# SDK 反馈新鲜度门限；以新 state 对象的接收时刻计。
FEEDBACK_TIMEOUT_S = 0.2
# 停止确认：两轴实际速度都不超过该值并持续 STOP_SETTLE_S 才视为已停。
STOP_VELOCITY = math.radians(0.5)
STOP_SETTLE_S = 0.1
# 停止命令只发不等；超过该时间仍未停稳则重发停止。
STOP_TIMEOUT_S = 2.0
# move_joint 的期望时长；初始化与回零沿用同长的名义窗口，遥操中回位按实际完成判定。
MOVE_JOINT_WINDOW_S = 5.0
# 相邻两帧头盔跳变门限，沿用 Quest 手柄跳变检测的默认值。
HEAD_JUMP_POSITION_M = DEFAULT_QUEST_POSITION_THRESHOLD_M
HEAD_JUMP_ROTATION_RAD = math.radians(DEFAULT_QUEST_ROTATION_THRESHOLD_DEG)


class WaistFault(RuntimeError):
    """腰控制器报错或反馈结构异常；沿用会话异常清理，不在腰内恢复。"""


def _pair(values):
    """校验并转换两轴数值（反馈角度/速度、限位）。

    Raises: ValueError 不是长度 2 的有限非布尔数值序列。
    """
    if (not isinstance(values, (list, tuple)) or len(values) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values)):
        raise ValueError("腰反馈/限位必须包含两个有限数值")
    return tuple(float(v) for v in values)


def _cart_pose(values):
    """校验腰末端笛卡尔位姿反馈 ``[x,y,z,qx,qy,qz,qw]`` 并归一化四元数。

    Raises: WaistFault 不是 7 个有限非布尔数值，或四元数不可归一化。
    """
    if (not isinstance(values, (list, tuple)) or len(values) < 7
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values[:7])):
        raise WaistFault("腰笛卡尔位姿反馈无效")
    pose = [float(v) for v in values[:7]]
    norm = math.sqrt(sum(v * v for v in pose[3:]))
    if norm <= 1e-12:
        raise WaistFault("腰笛卡尔位姿四元数无效")
    return tuple(pose[:3] + [v / norm for v in pose[3:]])


class WaistCarmMixin:
    """腰专用 Carm 适配层：兼容无末端执行器的两轴反馈，并按连接身份准入命令。

    只使用 SDK 的 ``move_joint`` / ``track_pose`` / ``stop_task``；SDK 布尔返回值只代表
    命令被接收，不代表到位。不影响双臂所用的 Carm 实例。独立连接与共享通道共用。
    """

    @property
    def _arm_state(self):
        """返回本通道 state 的副本，缺失的 eeff 字段补为 flange。"""
        arms = (self.state or {}).get("arm", [])
        if not isinstance(arms, list) or len(arms) <= self.arm_index:
            return {}
        arm = arms[self.arm_index]
        if not isinstance(arm, dict):
            return {}
        # SDK 假定所有通道都有 eeff；不改写收到的共享 state 对象。
        result = dict(arm)
        result["eeff"] = dict(arm.get("eeff") or {})
        result["eeff"].setdefault("eeff_type", "flange")
        result["eeff"].setdefault("eeff_name", "flange")
        return result

    def move_joint_for_connection(self, joints, expected_ws, desire_time, wait):
        """在连接世代锁内对腰通道只发不等地下发关节运动（腰回位键使用，wait 须为 False）。

        Returns: 连接失配时为 False，否则为是否已写入连接。
        """
        with self._connection_generation_lock:
            if not self._running or self.ws is not expected_ws or not self.is_connected():
                return False
            with sdk_nowait():
                return self.move_joint(list(joints), desire_time, wait)

    def stop_for_connection(self, expected_ws):
        """在连接世代锁内对腰通道只发不等地请求立即停止；是否停稳由反馈确认。

        Returns: 连接失配时为 False，否则为是否已写入连接。
        """
        with self._connection_generation_lock:
            if not self._running or self.ws is not expected_ws or not self.is_connected():
                return False
            with sdk_nowait():
                return self.stop_task(at_once=True)


class WaistCarm(WaistCarmMixin, Carm if Carm is not None else object):
    """独立 SDK 连接上的腰 Carm（``sdk.shared_connection`` 关闭时使用）。"""


class WaistChannel(WaistCarmMixin, CarmChannel):
    """共享 SDK 连接上的腰通道。"""


class FreshRobotState:
    """按 ws/state 对象身份读取新鲜反馈，隔离重连前的缓存。

    只有新的 state 对象才推进接收时刻；连接对象变化后的首次读取只建立基线并返回 None。
    """

    def __init__(self, carm, clock=time.monotonic):
        """绑定 Carm 与单调时钟（测试可注入）。"""
        self.carm = carm
        self.clock = clock
        self.ws = None
        self.state = None
        self.accepted = None
        self.received = 0.0

    def read(self):
        """读取当前连接上不超过 FEEDBACK_TIMEOUT_S 的 state；否则返回 None。"""
        carm = self.carm
        if not carm.is_connected():
            self.ws = None
            self.accepted = None
            return None
        ws, state = carm.ws, carm.state
        if ws is None or ws is not self.ws:
            self.ws, self.state, self.accepted = ws, state, None
            return None
        if state is not self.state:
            self.state = state
            self.accepted = state if isinstance(state, dict) else None
            self.received = self.clock()
        if (self.accepted is None or self.clock() - self.received > FEEDBACK_TIMEOUT_S
                or carm.ws is not ws or not carm.is_connected()):
            return None
        return self.accepted


def detect_waist(carm, cancelled, timeout=3.0):
    """用已连接臂的新鲜完整反馈判断是否存在两轴腰，不发送使能或运动命令。

    完整的两个 7 轴 D3 通道且无第三通道表示无腰；第三通道必须是两轴。

    Returns: 存在两轴腰为 True，确认无腰为 False。
    Raises: RuntimeError 识别被取消、反馈结构异常或超时未取得新鲜反馈
        （缺反馈不能当作无腰）。
    """
    reader = FreshRobotState(carm)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cancelled():
            raise RuntimeError("腰硬件识别已取消")
        state = reader.read()
        arms = state.get("arm") if state else None
        if isinstance(arms, list) and len(arms) >= 2:
            if not all(isinstance(arm, dict) and arm.get("arm_dof") == 7 for arm in arms[:2]):
                raise RuntimeError("D3 双臂反馈结构不完整")
            if len(arms) == 2:
                return False
            if not isinstance(arms[2], dict) or arms[2].get("arm_dof") != 2:
                raise RuntimeError("腰通道存在但两轴反馈不可用")
            return True
        time.sleep(0.01)
    raise RuntimeError("无法读取新鲜的完整 D3 反馈，不能判断是否有腰")


@dataclass(frozen=True)
class WaistFeedback:
    """一次腰反馈的冻结快照；ws/state 保留对象身份，用于命令准入与停止确认。

    ``position``/``velocity`` 为两轴关节角/速度，``cart_pose`` 为末端笛卡尔位姿。
    """

    ws: object
    state: object
    position: tuple
    velocity: tuple
    task: dict
    cart_pose: tuple


def _head_quaternion(head):
    """PoseSample → (x, y, z, w) 四元数。"""
    return head.qx, head.qy, head.qz, head.qw


def _head_components(head):
    """PoseSample → 7 分量 (x, y, z, qx, qy, qz, qw)。"""
    return (head.x, head.y, head.z) + _head_quaternion(head)


def _head_jumped(previous, current):
    """相邻两帧头盔位置或姿态变化是否超过跳变门限。"""
    distance = math.dist((previous.x, previous.y, previous.z), (current.x, current.y, current.z))
    a, b = _head_quaternion(previous), _head_quaternion(current)
    norm = math.sqrt(sum(v * v for v in a)) * math.sqrt(sum(v * v for v in b))
    dot = abs(sum(x * y for x, y in zip(a, b))) / norm
    return distance > HEAD_JUMP_POSITION_M or 2.0 * math.acos(min(1.0, dot)) > HEAD_JUMP_ROTATION_RAD


class WaistRuntime:
    """统一腰控制状态机，按每帧 mode 在头盔与摇杆控制律之间分发。"""

    def __init__(self, host, config, read_input, read_params, publish, on_failure,
                 carm_factory=WaistCarm, clock=time.monotonic, sleep=time.sleep, motion_cancel_event=None,
                 publication_signal=None):
        self.host, self.config = host, config
        self.read_input, self.read_params = read_input, read_params
        self.publication_signal = publication_signal
        self.publish, self.on_failure = publish, on_failure
        self.carm_factory, self.clock, self.sleep = carm_factory, clock, sleep
        self.motion_cancel_event = motion_cancel_event or threading.Event()
        self.carm = self.feedback_reader = self.thread = None
        self.stop_event = threading.Event()
        self.home_completed = False
        self.motion_active = self.stop_pending = False
        # 停止确认在后续周期进行：发出停止时的连接/帧、停稳起点与重发截止。
        self.stop_ws = self.stop_state = self.stop_settled = None
        self.stop_deadline = 0.0
        # 停稳前到达的跟随/回位点击，停稳后补执行。
        self.pending_follow = self.pending_home = False
        self.lower = self.upper = self.max_velocity = None
        self.target = None
        self.following = False
        self.mode = None
        self.mapping = PoseMapping()
        self.base_epoch = None
        self.generation = self.follow_sequence = self.home_sequence = None
        self.recenter_count = None
        self.last_head = None
        self.last_tick = None
        self.return_watch = self.return_ws = None
        self.status = "connecting"
        self._published_following = None
        self._published_mode = None

    def _status(self, value, error=None):
        if (value != self.status or error or self.following != self._published_following
                or self.mode != self._published_mode):
            self.status = value
            self._published_following = self.following
            self._published_mode = self.mode
            self.publish(value, self.following, self.mode, error)

    def _feedback(self, allow_error=False):
        state = self.feedback_reader.read()
        if state is None:
            return None
        arms = state.get("arm", [])
        if not isinstance(arms, list) or len(arms) <= 2 or not isinstance(arms[2], dict):
            raise WaistFault("腰反馈缺少 index 2")
        arm = arms[2]
        if arm.get("arm_dof") != 2:
            raise WaistFault("腰反馈自由度不是 2")
        if not allow_error and (arm.get("state") == -1 or
                (state.get("error", 0) or state.get("errMsg")) and
                (state.get("error_all_arm") or state.get("error_arm_index") == 2)):
            raise WaistFault("腰控制器报告错误")
        reality = arm.get("reality", {})
        return WaistFeedback(self.feedback_reader.ws, state, _pair(reality.get("pose")),
                             _pair(reality.get("vel")), dict(arm.get("task") or {}),
                             _cart_pose(arm.get("pose")))

    def _wait_feedback(self, timeout=3.0, allow_error=False):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            feedback = self._feedback(allow_error=allow_error)
            if feedback is not None:
                return feedback
            self.sleep(0.01)
        raise RuntimeError("腰反馈超时或连接已变化")

    def _validate_target(self, target):
        if any(not low <= value <= high for value, low, high in zip(target, self.lower, self.upper)):
            raise ValueError("腰目标超出设备关节限位")

    def prepare(self):
        if Carm is None and self.carm_factory is WaistCarm:
            raise RuntimeError("Carm SDK 不可用")
        self.carm = self.carm_factory(self.host, 2)
        self.feedback_reader = FreshRobotState(self.carm, self.clock)
        if not self.carm.is_connected() and self.carm.connect(timeout=1.0) is not True:
            raise RuntimeError("腰 Carm 连接失败")
        feedback = self._wait_feedback()
        limits = self.carm.get_limits()
        self.lower, self.upper = _pair(limits.get("limit_lower")), _pair(limits.get("limit_upper"))
        device_velocity = _pair(limits.get("joint_vel"))
        if any(low >= high for low, high in zip(self.lower, self.upper)):
            raise ValueError("腰关节上下限无效")
        if any(value <= 0 for value in device_velocity):
            raise ValueError("腰设备速度限制无效")
        self.max_velocity = tuple(min(math.radians(self.config.max_speed_deg_s), value)
                                 for value in device_velocity)
        for target in (self.config.init_joint, self.config.teleop_reset_joint,
                       self.config.home_joint, feedback.position):
            self._validate_target(target)
        if self.carm.set_ready() is not True or self.carm.set_control_mode(1) is not True:
            raise RuntimeError("腰使能或控制模式设置失败")
        self.target = list(self._wait_feedback().position)

    def _trigger_joint_motion(self, target, status):
        if self.motion_cancel_event.is_set():
            return False
        self.following = False
        self._status(status)
        self.carm.move_joint(list(target), MOVE_JOINT_WINDOW_S, False)
        deadline = self.clock() + MOVE_JOINT_WINDOW_S
        while True:
            remaining = deadline - self.clock()
            if remaining <= 0:
                break
            self.sleep(min(0.05, remaining))
            if self.motion_cancel_event.is_set():
                return False
            if not self.carm.is_connected():
                self.motion_cancel_event.set()
                return False
        if self.motion_cancel_event.is_set():
            return False
        self.motion_active = self.stop_pending = False
        self.target = None
        self._status("idle")
        return True

    def move_to_init(self):
        if self.carm is None or not self.carm.is_connected():
            return False
        return self._trigger_joint_motion(self.config.init_joint, "returning_init")

    def move_home(self):
        if self.home_completed:
            return True
        if self.carm is None or not self.carm.is_connected():
            return False
        self.home_completed = self._trigger_joint_motion(self.config.home_joint, "returning_zero")
        return self.home_completed

    def hold(self):
        """停腰并检查是否已停稳，不阻塞控制线程。

        首次调用只发不等地发出停止，之后每次调用检查一次反馈：新帧、同一连接、任务
        不在执行且两轴速度不超过 STOP_VELOCITY 持续 STOP_SETTLE_S 视为已停稳。
        STOP_TIMEOUT_S 内未停稳则重发停止。

        Returns: 无运动或已停稳为 True；停止尚未确认为 False。
        """
        self.following = False
        if not self.motion_active and not self.stop_pending:
            return True
        now = self.clock()
        try:
            if not self.stop_pending or now >= self.stop_deadline:
                ws = self.carm.ws
                self.stop_pending = True
                self.stop_ws, self.stop_state, self.stop_settled = ws, self.carm.state, None
                sent = self.carm.stop_for_connection(ws) is True
                # 发送失败时下次调用立即重发。
                self.stop_deadline = now + STOP_TIMEOUT_S if sent else now
                return False
            feedback = self._feedback(allow_error=True)
            stopped = (feedback is not None and feedback.state is not self.stop_state
                       and feedback.ws is self.stop_ws
                       and not feedback.task.get("exe_flag", True)
                       and all(abs(v) <= STOP_VELOCITY for v in feedback.velocity))
            if not stopped:
                self.stop_settled = None
                return False
            if self.stop_settled is None:
                self.stop_settled = now
            if now - self.stop_settled < STOP_SETTLE_S:
                return False
            self.motion_active = self.stop_pending = False
            self.target = list(feedback.position if self.mode == "stick" else feedback.cart_pose)
            return True
        except Exception:
            logging.exception("[WaistControl] 停止确认失败")
            return False

    def _pause(self, status):
        # 中断时丢弃停稳前暂存的点击，恢复后须重新触发。
        self.pending_follow = self.pending_home = False
        self.hold()
        self._status(status)

    def _input(self):
        publication = self.read_input()
        if (publication is None or publication.waist_input is None
                or publication.body_intent is None
                or self.clock() - publication.timestamp_monotonic > INPUT_TIMEOUT_S):
            return None
        return publication

    def _admitted(self, generation):
        publication = self._input()
        return (not self.stop_event.is_set() and self.following and publication is not None
                and publication.transport_generation == generation)

    def _rebase(self, head, pose, params):
        self.mapping.rebase(input_pose_to_arm(_head_components(head), params.calibration), pose)
        self.base_epoch = params.epoch

    def _engage_head(self, head, feedback, params):
        last_target = self.target if self.motion_active else None
        pose = select_output_baseline(last_target, feedback.task.get("exe_flag", False), feedback.cart_pose)
        self.target = list(pose)
        self._rebase(head, pose, params)
        self.following = True

    def _desired_head(self, head, params):
        if params.epoch != self.base_epoch:
            self._rebase(head, self.target, params)
            return list(self.target)
        current = input_pose_to_arm(_head_components(head), params.calibration)
        target = self.mapping.target(current, params.scale, params.scale)
        if target is None:
            self._rebase(head, self.target, params)
            return list(self.target)
        return [float(value) for value in target]

    def _dispatch_pose(self, target, feedback, generation):
        self.motion_active = True
        if self.carm.dispatch_pose_target_for_connection(
                target, feedback.ws, lambda: self._admitted(generation)) is not True:
            self._pause("waiting_feedback" if self._input() is not None else "waiting_input")
            return False
        return True

    @staticmethod
    def _stick_velocity(value, deadzone, maximum):
        if abs(value) <= deadzone:
            return 0.0
        return math.copysign((abs(value) - deadzone) / (1.0 - deadzone) * maximum, value)

    def _dispatch_stick(self, stick, feedback, generation):
        now = self.clock()
        elapsed = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if elapsed > INPUT_TIMEOUT_S:
            self._pause("waiting_input")
            return
        dt = max(0.0, min(elapsed, 0.05))
        target = list(self.target)
        # 前推为关节 0 正向，左推为关节 1 正向；x 为左右，y 为前后。
        for index, axis, sign in ((0, stick[1], 1), (1, stick[0], -1)):
            target[index] += sign * self._stick_velocity(axis, self.config.deadzone,
                                                         self.max_velocity[index]) * dt
            target[index] = max(self.lower[index], min(self.upper[index], target[index]))
        self.motion_active = True
        if self.carm.dispatch_joint_target_for_connection(
                target, feedback.ws, lambda: self._admitted(generation)) is not True:
            self._pause("waiting_feedback" if self._input() is not None else "waiting_input")
            return
        self.target = target
        self._status("following")

    def _start_return(self, feedback):
        self.following = False
        if self.motion_active or self.stop_pending:
            if not self.hold():
                self.pending_home = True
                self._status("waiting_feedback")
                return
            feedback = self._feedback()
            if feedback is None:
                self._status("waiting_feedback")
                return
        result = self.carm.move_joint_for_connection(
            self.config.teleop_reset_joint, feedback.ws, MOVE_JOINT_WINDOW_S, False)
        if result is not True and result != 1:
            logging.warning("[WaistControl] 回位命令未被接收：%r", result)
            self._status("idle")
            return
        self.return_watch = JointMotionWatch(
            self.carm, self.carm.last_motion_task_key, self.config.teleop_reset_joint)
        self.return_ws = feedback.ws
        self.target = None
        self.motion_active = self.stop_pending = False
        self._status("returning_init")

    def _step_return(self):
        if (self.carm.is_connected() and self.carm.ws is self.return_ws
                and not self.motion_cancel_event.is_set()):
            feedback = self._feedback(allow_error=True)
            if feedback is None:
                return
            arrived = self.return_watch.poll(
                self.clock(), feedback.position, feedback.velocity,
                feedback.task.get("exe_flag", False))
            if arrived is None:
                return
            if not arrived:
                logging.warning("[WaistControl] 回位结束但未到位：%s", feedback.position)
        self.return_watch = self.return_ws = None
        self.target = None
        self.motion_active = self.stop_pending = False
        self.follow_sequence = self.home_sequence = None
        self._status("idle")

    def step(self):
        if self.return_watch is not None:
            return self._step_return()
        publication = self._input()
        if publication is None:
            return self._pause("waiting_input")
        sample, body = publication.waist_input, publication.body_intent
        generation = publication.transport_generation
        mode = sample.mode
        if self.mode is not None and mode != self.mode:
            # 切换方式：停住腰并回到空闲，下一次腰跟随事件按新方式从新鲜反馈接管。
            held = self.hold()
            self.mode = mode
            return self._status("idle" if held else "waiting_feedback")
        self.mode = mode
        rebased = (generation != self.generation or self.follow_sequence is None
                   or body.waist_follow_sequence < self.follow_sequence
                   or body.waist_home_sequence < self.home_sequence)
        clicked = not rebased and body.waist_follow_sequence > self.follow_sequence
        home_clicked = not rebased and body.waist_home_sequence > self.home_sequence
        jumped = (mode == "head" and sample.head_valid and self.last_head is not None
                  and _head_jumped(self.last_head, sample.head))
        self.generation = generation
        self.follow_sequence = body.waist_follow_sequence
        self.home_sequence = body.waist_home_sequence
        self.last_head = sample.head if mode == "head" and sample.head_valid else None
        if mode == "head" and sample.recenter_count != self.recenter_count:
            rebased = True
        self.recenter_count = sample.recenter_count
        # 换代、序号回退或重定位时两种方式都中断控制（含等待回中），必须再触发一次腰跟随事件。
        if rebased and (self.following or self.status == "waiting_center"):
            return self._pause("waiting_input")
        if mode == "head" and (not sample.head_valid or (self.following and jumped)):
            return self._pause("waiting_input")
        feedback = self._feedback()
        if feedback is None:
            return self._pause("waiting_feedback")
        if self.stop_pending and not self.hold():
            self.pending_follow = self.pending_follow or clicked
            self.pending_home = self.pending_home or home_clicked
            return self._status("waiting_feedback")
        clicked = clicked or self.pending_follow
        home_clicked = home_clicked or self.pending_home
        self.pending_follow = self.pending_home = False
        if home_clicked:
            return self._start_return(feedback)
        stopped_by_click = False
        if clicked:
            if self.following:
                self.following = False
                self._status("idle")
                stopped_by_click = True
            elif mode == "head":
                self._engage_head(sample.head, feedback, self.read_params())
            else:
                self.following = False
                self.last_tick = self.clock()
                self.target = list(feedback.position)
                self._status("waiting_center")
                return
        if mode == "stick":
            if not self.following:
                if not stopped_by_click and self.status == "waiting_center":
                    if (abs(body.waist_stick[0]) <= self.config.deadzone
                            and abs(body.waist_stick[1]) <= self.config.deadzone):
                        self.target = list(feedback.position)
                        self.following = True
                        self.last_tick = self.clock()
                        self._status("following")
                    else:
                        self._status("waiting_center")
                elif self.status != "idle":
                    self._status("idle")
                return
            self._dispatch_stick(body.waist_stick, feedback, generation)
            return
        if not self.following:
            # 不再下发，控制器停在最后目标；motion_active 保留，重新跟随以该目标为基线，
            # 回位与切换方式仍先停止。
            self._status("idle")
            return
        target = self._desired_head(sample.head, self.read_params())
        if self._dispatch_pose(target, feedback, generation):
            self.target = target
            self._status("following")

    def start(self):
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.run, daemon=True, name="JoystickWaistLoop")
        self.thread.start()

    def run(self):
        try:
            signal = self.publication_signal
            while not self.stop_event.is_set():
                # 步前取发布序号：步内到达的新帧会让下一次等待立即返回。
                seen = signal.sequence if signal is not None else None
                self.step()
                if signal is not None:
                    signal.wait_newer(seen, LOOP_INTERVAL_S)
                else:
                    self.stop_event.wait(LOOP_INTERVAL_S)
        except Exception as exc:
            logging.exception("[WaistControl] 控制循环异常")
            self.following = False
            self._status("fault", str(exc))
            self.hold()
            self.on_failure(self, str(exc))

    def request_stop(self):
        self.stop_event.set()

    def stop_loop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=4.0)
            if self.thread.is_alive():
                return False
            self.thread = None
        self.following = False
        return True

    def close(self):
        if not self.stop_loop():
            return False
        if self.carm is not None and self.carm.disconnect() is not True:
            return False
        self._status("stopped")
        return True


def create_waist_runtime(owner, runtimes):
    """双臂连接后决定是否启用腰控制；不满足条件时发布原因并返回 None。

    条件依次为：同板 D3 双臂、本连接协商了 ``waist_control_v1`` 且声明了 ``wf``/``wh``/``ws``、配置启用、硬件存在两轴腰。

    Raises: ValueError D3 双臂不在同一控制板；JoystickConfigError 腰配置非法；
        RuntimeError 腰硬件识别失败（缺反馈不能当作无腰）。
    """
    if len(runtimes) != 2 or any(r.controller.arm_model != "D3" for r in runtimes):
        owner._set_waist_state("unavailable")
        return None
    if len({r.control_host for r in runtimes}) != 1:
        raise ValueError("D3 腰与双臂必须在同一控制板")
    with owner._data_lock:
        binding = owner.active_source_binding
        module = owner.source_registry.active_module()
    if binding is None or module is None:
        owner._set_waist_state("unavailable")
        return None
    lease = binding.lease
    if dict(module.status(lease).details).get("waist_control_ready") is not True:
        owner._set_waist_state("legacy_input")
        return None
    config = owner.joystick_config.resolve_waist()
    if not config.enabled:
        owner._set_waist_state("disabled")
        return None
    if not detect_waist(runtimes[0].controller.carm_, lambda: not owner.is_input_connected()):
        owner._set_waist_state("absent")
        return None

    def read_input():
        """只返回当前 lease 下、输入可用时的最新发布。"""
        current = owner.active_source_binding
        if (current is None or current.lease != lease or current.phase is not BindingPhase.ACTIVE
                or not current.input_ready or not owner.state.ble_connected):
            return None
        publication = owner._input_publication.load()
        return publication if publication is not None and publication.lease == lease else None

    def failed(runtime, reason):
        """腰控制器故障沿用会话异常清理。"""
        if owner.session is not None and owner.session.waist is runtime:
            owner._mark_disconnected_and_request_cleanup("腰：" + reason, expected_lease=lease)

    owner._set_waist_state("connecting")
    def read_params():
        """腰与手柄共用当前发布的 scale/标定参数。"""
        return owner._control_params

    links = owner._shared_carm_links()
    carm_factory = WaistCarm if links is None else (
        lambda host, arm_index: links.channel(host, arm_index, WaistChannel))
    return WaistRuntime(
        runtimes[0].control_host, config, read_input, read_params, owner._set_waist_state, failed,
        carm_factory=carm_factory, motion_cancel_event=owner._motion_cancel,
        publication_signal=owner._publication_signal,
    )
