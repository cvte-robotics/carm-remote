"""单双臂遥操 session 编排：落地 ArmRuntime 并编排连接/回零/启停，任一步失败即回收。"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from carm_teleop_input_contracts import HAND_LEFT, HAND_RIGHT, normalize_hand, opposite_hand
from ..end_effectors import EndEffectorFactory, validate_end_effector_driver
from .end_effector_sender import AsyncEndEffectorSender
from .control_transaction import ControlTransaction


LOCAL_ARM_CONTROL_HOST = "127.0.0.1"
ArmEndpointResolver = Callable[[str, str, bool], str]


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
    # 控制环看到的末端驱动：包装 end_effector，SDK 阻塞调用在独立线程执行。
    end_effector_sender: Optional[AsyncEndEffectorSender] = None
    thread: Optional[threading.Thread] = None
    carm_disconnected: bool = False
    home_completed: bool = False


class BaseArmSession:
    """单/双臂 session 的公共编排基类（由具体 profile 派生）。"""

    def __init__(
        self,
        owner,
        bindings: List[Dict[str, Any]],
        controller_factory: Callable[[str, str], object],
        end_effector_factory: Optional[EndEffectorFactory] = None,
    ):
        """绑定 owner、binding 列表与 controller/end-effector 工厂。"""
        self.owner = owner
        self.bindings = bindings
        self.controller_factory = controller_factory
        self.end_effector_factory = end_effector_factory
        self.arms: List[ArmRuntime] = []
        self._home_requested = False

    @property
    def home_pending(self) -> bool:
        """是否仍有已请求但未完成的回零。"""
        return self._home_requested and any(
            not runtime.home_completed for runtime in self.arms
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

            for runtime in runtimes:
                if not self.owner.is_input_connected():
                    self._cleanup_started(runtimes)
                    return False
                if not runtime.controller.prepareArm(runtime.control_host):
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
                runtime.end_effector_sender = AsyncEndEffectorSender(
                    runtime.end_effector, runtime.role,
                )
                controller.end_effector_driver = runtime.end_effector_sender

            if len(runtimes) == 1:
                if (
                    not self.owner.is_input_connected()
                    or not runtimes[0].controller.moveToInitPosition()
                ):
                    self._cleanup_started(runtimes)
                    return False
            elif not self._move_to_init_concurrently(runtimes):
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
                if runtime.end_effector_sender is not None:
                    runtime.end_effector_sender.start()
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
            return True
        except Exception:
            self._cleanup_started(runtimes)
            raise

    def _move_to_init_concurrently(self, runtimes: List[ArmRuntime]) -> bool:
        """并发下发双臂初始化运动，并对齐命令下发时刻。"""
        start_gate = threading.Event()
        cancel_event = threading.Event()
        results: Dict[str, bool] = {}
        start_times: Dict[str, float] = {}

        def move(runtime: ArmRuntime) -> None:
            """等待启动栅栏后下发单臂初始化运动。"""
            start_gate.wait()
            if cancel_event.is_set():
                results[runtime.role] = False
                return
            start_times[runtime.role] = time.monotonic()
            try:
                results[runtime.role] = bool(
                    self.owner.is_input_connected()
                    and runtime.controller.moveToInitPosition()
                )
            except Exception:
                logging.exception("[ArmSession] Initialization movement failed for role=%s", runtime.role)
                results[runtime.role] = False

        workers = [
            threading.Thread(target=move, args=(runtime,), daemon=True, name=f"JoystickArmInit-{runtime.role}")
            for runtime in runtimes
        ]
        try:
            for worker in workers:
                worker.start()
        except Exception:
            cancel_event.set()
            start_gate.set()
            for worker in workers:
                if worker.is_alive():
                    worker.join(timeout=2.0)
            raise

        start_gate.set()
        for worker in workers:
            worker.join()

        if len(start_times) > 1:
            skew_ms = (max(start_times.values()) - min(start_times.values())) * 1000.0
            logging.info("[ArmSession] Dual-arm initialization command start skew: %.3f ms", skew_ms)

        return len(results) == len(runtimes) and all(results.values())

    def stop(self, move_home: bool = False) -> bool:
        """停止控制线程、按需回零，并释放 Carm 连接与 owner 归属。

        Args: move_home 是否在停止前把 pending 的机械臂送回零点。
        Returns: 全部运行时已干净释放（``self.arms`` 为空）时为 True。
        """
        success = True
        threads_stopped = True
        self._home_requested = bool(move_home)

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

        for runtime in self.arms:
            try:
                if runtime.thread and runtime.thread.is_alive():
                    runtime.thread.join(timeout=2.0)
                if runtime.thread and runtime.thread.is_alive():
                    threads_stopped = False
                    success = False
                else:
                    runtime.thread = None
            except Exception:
                threads_stopped = False
                success = False

        for runtime in self.arms:
            try:
                runtime.controller.halt_control()
            except Exception:
                logging.exception(
                    "[ArmSession] Failed to invalidate control mapping for role=%s",
                    runtime.role,
                )
                success = False

        # 控制线程停止后丢弃待发末端值并等待在途 SDK 调用结束，再回零/断开 Carm。
        for runtime in self.arms:
            sender = runtime.end_effector_sender
            try:
                if sender is not None and not sender.stop():
                    threads_stopped = False
                    success = False
            except Exception:
                logging.exception(
                    "[ArmSession] Failed to stop end-effector sender for role=%s",
                    runtime.role,
                )
                threads_stopped = False
                success = False

        # 在每个控制循环停止前保持臂集合完整，避免手动双臂回零
        # 退化为单臂操作。
        if not threads_stopped:
            return False

        if self._home_requested and self.arms:
            pending_home = [
                runtime for runtime in self.arms if not runtime.home_completed
            ]
            if len(pending_home) > 1:
                home_clean = self._move_home_concurrently(pending_home)
            elif pending_home:
                try:
                    pending_home[0].home_completed = bool(
                        pending_home[0].controller.move_home()
                    )
                except Exception:
                    logging.exception("[ArmSession] Single-arm home movement failed")
                home_clean = pending_home[0].home_completed
            else:
                home_clean = True
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
        return success and not self.arms

    def _move_home_concurrently(self, runtimes: List[ArmRuntime]) -> bool:
        """并发下发回零命令，并标记成功的运行时。"""
        start_gate = threading.Event()
        cancel_event = threading.Event()
        results: Dict[str, bool] = {}
        start_times: Dict[str, float] = {}

        def move_home(runtime: ArmRuntime) -> None:
            """等待启动栅栏后下发单臂回零运动。"""
            start_gate.wait()
            if cancel_event.is_set():
                results[runtime.role] = False
                return
            start_times[runtime.role] = time.monotonic()
            try:
                results[runtime.role] = bool(runtime.controller.move_home())
            except Exception:
                logging.exception("[ArmSession] Home movement failed for role=%s", runtime.role)
                results[runtime.role] = False

        workers = [
            threading.Thread(target=move_home, args=(runtime,), daemon=True, name=f"JoystickArmHome-{runtime.role}")
            for runtime in runtimes
        ]
        try:
            for worker in workers:
                worker.start()
        except Exception:
            cancel_event.set()
            start_gate.set()
            for worker in workers:
                if worker.is_alive():
                    worker.join(timeout=2.0)
            logging.exception("[ArmSession] Failed to start concurrent home workers")
            return False

        start_gate.set()
        for worker in workers:
            worker.join()

        if len(start_times) > 1:
            skew_ms = (max(start_times.values()) - min(start_times.values())) * 1000.0
            logging.info("[ArmSession] Dual-arm home command start skew: %.3f ms", skew_ms)

        for runtime in runtimes:
            if results.get(runtime.role) is True:
                runtime.home_completed = True
        return len(results) == len(runtimes) and all(results.values())

    def get_primary_controller(self):
        """返回首臂 controller（无臂时为 None）。"""
        return self.arms[0].controller if self.arms else None

    def _cleanup_started(self, started: List[ArmRuntime]) -> bool:
        """把已启动运行时登记为 arms 后执行不回零的停止。"""
        self.arms = list(started)
        return self.stop(move_home=False)
