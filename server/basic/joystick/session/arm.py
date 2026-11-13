"""单双臂遥操 session 编排：落地 ArmRuntime 并编排连接/回零/启停，任一步失败即回收。"""

import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, normalize_hand, opposite_hand
from ..core.command_controller import CommandController
from ..end_effectors import EndEffectorFactory, validate_end_effector_driver
from .end_effector_sender import EndEffectorSender
from .control_transaction import ControlTransaction


LOCAL_ARM_CONTROL_HOST = "127.0.0.1"
ArmEndpointResolver = Callable[[str, str, bool], str]


def active_arm_profile(owner):
    """当前激活 provider 的 arm_profile（无激活模块时为 ``None``）。"""
    registry = getattr(owner, "source_registry", None)
    module = None if registry is None else registry.active_module()
    return None if module is None else getattr(module, "arm_profile", None)


def apply_profile_device_indexes(bindings, profile) -> None:
    """按 arm_profile 的手侧 → 臂索引映射覆盖 binding 的 ``device_index``。"""
    if profile is None:
        return
    for arm_binding in bindings:
        index = profile.device_index_by_hand.get(arm_binding["hand"])
        if index is not None:
            arm_binding["device_index"] = index


def board_arm_endpoint(role: str, management_ip: str, same_board_dual: bool) -> str:
    """控制板侧 endpoint 解析：master 或同板双臂走本机回环。"""
    if role == "master" or same_board_dual:
        return LOCAL_ARM_CONTROL_HOST
    return management_ip


def desktop_arm_endpoint(role: str, management_ip: str, same_board_dual: bool) -> str:
    """桌面侧 endpoint 解析：始终使用管理网 IP。"""
    del role, same_board_dual
    return management_ip


def build_arm_bindings(
    mode: str,
    master_ip: str,
    master_side: str,
    slave_ip: Optional[str] = None,
    endpoint_resolver: ArmEndpointResolver = board_arm_endpoint,
) -> List[Dict[str, Any]]:
    """把管理网地址解析为显式的单臂控制 endpoint 绑定列表。

    Args: mode "single"/"dual"；master_ip 主臂管理网 IP；master_side 主臂手侧
        （同板双臂忽略此值，固定 index 0 为左、index 1 为右）；
        slave_ip 双臂模式下的从臂 IP；endpoint_resolver endpoint 解析策略。
    Returns: binding dict 列表，字段为 role/ip/control_host/device_index/hand。
    Raises: ValueError mode 不受支持，或双臂模式缺少 slave_ip。
    """
    normalized_mode = (mode or "single").strip().lower()
    normalized_master_side = normalize_hand(master_side, HAND_RIGHT)

    if normalized_mode == "single":
        return [
            {
                "role": "master",
                "ip": master_ip,
                "control_host": endpoint_resolver("master", master_ip, False),
                "device_index": None,
                "hand": normalized_master_side,
            }
        ]

    if normalized_mode != "dual":
        raise ValueError(f"Unsupported joystick mode: {mode}")
    if not slave_ip:
        raise ValueError("slave_ip is required for dual mode")

    # 先按非同板规则解析 endpoint，再判断两个臂是否实际指向同一控制端点。
    # 控制板侧主臂固定走本机回环，因此从臂填写 127.0.0.1 时，即使管理 IP
    # 文本不同，两个 Carm 仍属于同一服务；桌面侧远端主臂和回环从臂仍保持区分。
    master_endpoint = endpoint_resolver("master", master_ip, False)
    slave_endpoint = endpoint_resolver("slave", slave_ip, False)
    same_board_dual = (
        master_ip == slave_ip or master_endpoint == slave_endpoint
    )
    # 同板双臂（人形双臂）的 Carm 索引与手侧是物理固定的：index 0 为左臂、
    # index 1 为右臂，左手柄控制左臂、右手柄控制右臂，不随 master_side 变化。
    master_hand = HAND_LEFT if same_board_dual else normalized_master_side
    return [
        {
            "role": "master",
            "ip": master_ip,
            "control_host": endpoint_resolver("master", master_ip, same_board_dual),
            "device_index": 0 if same_board_dual else None,
            "hand": master_hand,
        },
        {
            "role": "slave",
            "ip": slave_ip,
            "control_host": endpoint_resolver("slave", slave_ip, same_board_dual),
            "device_index": 1 if same_board_dual else None,
            "hand": opposite_hand(master_hand),
        },
    ]


@dataclass
class ArmRuntime:
    """单臂运行态：controller、控制事务、end-effector 与线程句柄（含幂等进度标记）。"""

    role: str
    ip: str
    control_host: str
    device_index: Optional[int]
    hand: str
    controller: object
    transaction: Optional[ControlTransaction] = None
    end_effector: Optional[object] = None
    # 控制环看到的末端驱动：包装 end_effector，去重与保活后在控制线程内只发不等。
    end_effector_sender: Optional[EndEffectorSender] = None
    thread: Optional[threading.Thread] = None
    carm_disconnected: bool = False
    home_completed: bool = False
    arm_angle: Optional[object] = None


class BaseArmSession:
    """单/双臂 session 的公共编排基类（由具体 profile 派生）。"""

    def __init__(
        self,
        owner,
        bindings: List[Dict[str, Any]],
        controller_factory: Callable[[str, str], object],
        end_effector_factory: Optional[EndEffectorFactory] = None,
        waist_factory: Optional[Callable[[List[ArmRuntime]], object]] = None,
        arm_angle_factory: Optional[Callable[[List[ArmRuntime]], object]] = None,
    ):
        """绑定 owner、binding 列表与 controller/end-effector/腰/臂型角工厂。

        ``waist_factory`` 在双臂连接后被调用，返回腰运行时或 None（不启用腰）。
        ``arm_angle_factory`` 同样在双臂连接后被调用，按准入条件把每臂运行时挂到
        ``ArmRuntime.arm_angle``，不满足时只发布原因、不启用。
        """
        self.owner = owner
        self.bindings = bindings
        self.controller_factory = controller_factory
        self.end_effector_factory = end_effector_factory
        self.waist_factory = waist_factory
        self.arm_angle_factory = arm_angle_factory
        self.arms: List[ArmRuntime] = []
        self.waist = None
        self._home_requested = False

    @property
    def has_resources(self) -> bool:
        """是否仍持有未释放的臂或腰。"""
        return bool(self.arms or self.waist is not None)

    @property
    def home_pending(self) -> bool:
        """是否仍有已请求但未完成的回零（含腰）。"""
        return self._home_requested and (
            any(not runtime.home_completed for runtime in self.arms)
            or (self.waist is not None and not self.waist.home_completed)
        )

    def initialize(self) -> bool:
        """完成连接、准备、end-effector 装配与回零，随后启动控制线程。

        Returns: 全部运行时就绪且控制线程已启动时为 True。
        Raises: RuntimeError end-effector 工厂缺失。
        """
        runtimes: List[ArmRuntime] = []
        self.arms = runtimes
        try:
            for binding in self.bindings:
                controller = self.controller_factory(binding["role"], binding["hand"])
                controller.carm_device_index = binding["device_index"]
                runtime = ArmRuntime(
                    role=binding["role"],
                    ip=binding["ip"],
                    control_host=binding["control_host"],
                    device_index=binding["device_index"],
                    hand=binding["hand"],
                    controller=controller,
                )
                runtimes.append(runtime)

            for runtime in runtimes:
                if (not self.owner.is_input_connected()
                        or not runtime.controller.connectArm(runtime.control_host)):
                    self._cleanup_started(runtimes)
                    return False
                self.owner.configure_arm_initialization(runtime)

            if self.waist_factory is not None:
                self.waist = self.waist_factory(runtimes)
                if self.waist is not None:
                    self.waist.prepare()
            if self.arm_angle_factory is not None:
                self.arm_angle_factory(runtimes)

            for runtime in runtimes:
                if not self.owner.is_input_connected():
                    self._cleanup_started(runtimes)
                    return False
                if not runtime.controller.prepareArm():
                    self._cleanup_started(runtimes)
                    return False

            for runtime in runtimes:
                if self.end_effector_factory is None:
                    raise RuntimeError("end-effector factory is required")
                controller = runtime.controller
                runtime.end_effector = validate_end_effector_driver(
                    self.end_effector_factory(
                        carm=controller.carm_,
                        side=runtime.hand,
                        role=runtime.role,
                    )
                )
                runtime.end_effector_sender = EndEffectorSender(
                    runtime.end_effector, runtime.role,
                )
                controller.end_effector_driver = runtime.end_effector_sender

            if not self._move_arms(runtimes):
                self._cleanup_started(runtimes)
                return False

            if not self.owner.is_input_connected():
                logging.warning("[ArmSession] Input disconnected before control loops started")
                self._cleanup_started(runtimes)
                return False

            for runtime in runtimes:
                if not self.owner.publish_source_controller(runtime.controller):
                    logging.warning("[ArmSession] Input disconnected before controller ownership publication")
                    self._cleanup_started(runtimes)
                    return False

            self.arms = runtimes
            for runtime in self.arms:
                controller = runtime.controller
                controller.isStop = False
                runtime.transaction = ControlTransaction(self.owner, runtime)
                # 臂型角由本臂控制线程步进，须在线程启动前完成激活。
                if runtime.arm_angle is not None:
                    runtime.arm_angle.start()
                runtime.thread = threading.Thread(
                    target=runtime.transaction.run,
                    daemon=True,
                    name=f"JoystickArmLoop-{runtime.role}",
                )
                runtime.thread.start()
                self.owner.set_arm_runtime_state(
                    runtime.role,
                    runtime.ip,
                    runtime.hand,
                    True,
                    control_host=runtime.control_host,
                    device_index=runtime.device_index,
                )
            if self.waist is not None:
                self.waist.start()
            return True
        except Exception:
            self._cleanup_started(runtimes)
            raise

    def _move_arms(self, runtimes: List[ArmRuntime], home: bool = False) -> bool:
        """调用线程依次触发各臂，再共等一个窗口；回零成功按臂保留。"""
        triggered = []
        for runtime in runtimes:
            try:
                controller = runtime.controller
                sent = (
                    controller.trigger_home_motion() if home
                    else self.owner.is_input_connected() and controller.trigger_init_motion()
                )
                if sent:
                    triggered.append(runtime)
            except Exception:
                logging.exception(
                    "[ArmSession] %s movement failed for role=%s",
                    "Home" if home else "Initialization", runtime.role,
                )

        # 腰在双臂触发后下发并等待自身窗口；双臂窗口从触发算起，三路总共约一个窗口。
        waist_ok = True
        if self.waist is not None:
            try:
                waist_ok = bool(
                    self.waist.move_home() if home
                    else self.owner.is_input_connected() and self.waist.move_to_init()
                )
            except Exception:
                logging.exception("[ArmSession] 腰%s运动失败", "回零" if home else "初始化")
                waist_ok = False

        completed = CommandController.wait_joint_motion_window(
            [runtime.controller for runtime in triggered]
        )
        if home:
            for runtime in triggered:
                if runtime.controller in completed:
                    runtime.home_completed = True
        return waist_ok and len(completed) == len(runtimes)

    def stop(self, move_home: bool = False) -> bool:
        """停止控制线程、按需回零，并释放 Carm 连接与 owner 归属。

        Args: move_home 是否在停止前把 pending 的机械臂送回零点。
        Returns: 全部运行时已干净释放（``self.arms`` 为空）时为 True。
        """
        success = True
        threads_stopped = True
        self._home_requested = bool(move_home)
        if self.waist is not None:
            self.waist.request_stop()

        for runtime in self.arms:
            try:
                if runtime.transaction is not None:
                    runtime.transaction.request_stop()
            except Exception:
                logging.exception(
                    "[ArmSession] Failed to request control stop for role=%s",
                    runtime.role,
                )
                success = False

        stopped_runtimes = []
        for runtime in self.arms:
            try:
                if runtime.thread and runtime.thread.is_alive():
                    runtime.thread.join(timeout=2.0)
                if runtime.thread and runtime.thread.is_alive():
                    threads_stopped = False
                    success = False
                else:
                    runtime.thread = None
                    stopped_runtimes.append(runtime)
            except Exception:
                threads_stopped = False
                success = False

        # join 超时或异常时保留运行时，不能改写退出状态尚未确认的映射。
        for runtime in stopped_runtimes:
            try:
                runtime.controller.halt_control()
            except Exception:
                logging.exception(
                    "[ArmSession] Failed to invalidate control mapping for role=%s",
                    runtime.role,
                )
                success = False

        # 控制线程停止后停用末端发送器，回零/断开 Carm 期间不再下发末端值。
        for runtime in self.arms:
            if runtime.end_effector_sender is not None:
                runtime.end_effector_sender.stop()

        if self.waist is not None and not self.waist.stop_loop():
            threads_stopped = False
            success = False
        # 臂型角由所属臂的控制线程步进；只有线程已退出的臂才能标记停止。
        for runtime in stopped_runtimes:
            if runtime.arm_angle is not None:
                runtime.arm_angle.stop()

        # 在每个控制循环停止前保持臂集合完整，避免手动双臂回零
        # 退化为单臂操作。
        if not threads_stopped:
            return False

        if self._home_requested and self.has_resources:
            pending_home = [
                runtime for runtime in self.arms if not runtime.home_completed
            ]
            home_clean = self._move_arms(pending_home, home=True)
            if not home_clean:
                return False
            self._home_requested = False

        remaining: List[ArmRuntime] = []
        for runtime in self.arms:
            if not runtime.carm_disconnected:
                try:
                    carm = getattr(runtime.controller, "carm_", None)
                    if carm is not None and carm.disconnect() is not True:
                        remaining.append(runtime)
                        success = False
                        continue
                    runtime.carm_disconnected = True
                except Exception:
                    logging.exception(
                        "[ArmSession] Carm disconnect failed for role=%s",
                        runtime.role,
                    )
                    remaining.append(runtime)
                    success = False
                    continue

            try:
                self.owner.set_arm_runtime_state(
                    runtime.role,
                    runtime.ip,
                    runtime.hand,
                    False,
                    control_host=runtime.control_host,
                    device_index=runtime.device_index,
                )
                self.owner.release_source_controller(runtime.controller)
            except Exception:
                logging.exception(
                    "[ArmSession] Host owner release failed for role=%s",
                    runtime.role,
                )
                remaining.append(runtime)
                success = False

        self.arms = remaining
        if self.waist is not None:
            if self.waist.close():
                self.waist = None
            else:
                success = False
        return success and not self.has_resources

    def get_primary_controller(self):
        """返回首臂 controller（无臂时为 None）。"""
        return self.arms[0].controller if self.arms else None

    def _cleanup_started(self, started: List[ArmRuntime]) -> bool:
        """把已启动运行时登记为 arms 后执行不回零的停止。"""
        self.arms = list(started)
        return self.stop(move_home=False)
