"""joystick 运行时依赖探测与 Carm SDK 封装。

映射数学仅使用 Python 标准库；Carm SDK 是否可用由 ARM_SDK_AVAILABLE 单独表示。
"""

import inspect
import itertools
import logging
import random
import threading
import time
import uuid
from contextlib import contextmanager

_SEND_ONLY_PREFIX = uuid.uuid4().hex[:8]
_SEND_ONLY_COUNTER = itertools.count()

# 遥操 move_joint 速度档位（SDK speed 百分比）；desire_time 固定为 -1，到达时间由控制器按速度规划。
MOVE_SPEED_LEVELS = {"low": 20, "medium": 50, "high": 80}
# SDK 的 move_joint 没有 speed 参数时，各档位速度百分比改用的 desire_time（-1 为 SDK 默认规划）。
MOVE_SPEED_FALLBACK_DESIRE_TIME = {
    MOVE_SPEED_LEVELS["low"]: 5,
    MOVE_SPEED_LEVELS["medium"]: 2,
    MOVE_SPEED_LEVELS["high"]: -1,
}

# 只发不等作用域内改为只写入连接的 SDK 请求命令：遥操期间的运动类命令（末端、关节运动、
# 停止）。查询类命令（限位、末端配置等）即使在作用域内也照常同步等待。
_NOWAIT_COMMANDS = frozenset({"setEffectorCtr", "webRecieveTasks", "stopSignals"})
_nowait_scope = threading.local()


@contextmanager
def sdk_nowait():
    """在当前线程内让 SDK 运动类业务方法只发不等。

    报文构造、参数校验与限位截断仍由 SDK 业务方法完成，只有最终的 ``request()``
    改为只写入连接；此时业务方法得到的 ``Task_Recieve`` 只表示已写入连接，控制器
    拒绝经状态帧错误异步上报。依赖 SDK 业务方法统一经 ``self.request`` 发送。
    """
    depth = getattr(_nowait_scope, "depth", 0)
    _nowait_scope.depth = depth + 1
    try:
        yield
    finally:
        _nowait_scope.depth = depth


class PendingReply:
    """登记不等的请求：发送后由调用方周期性 ``poll``，不阻塞调用线程。"""

    def __init__(self, carm, task_key, event, deadline):
        self._carm = carm
        self._task_key = task_key
        self._event = event
        self._deadline = deadline

    def poll(self, now):
        """回复已到返回回复字典，超时返回 SDK 同格式的拒绝，尚未到达返回 None。"""
        if self._event.is_set():
            data = self._carm.res_pool.pop(self._task_key, None) or {}
            return data.get("res", {"recv": "Task_Reject", "errMsg": "No response data"})
        if now >= self._deadline:
            self._carm.res_pool.pop(self._task_key, None)
            return {"recv": "Task_Reject", "errMsg": "Request timed out"}
        return None

    def cancel(self):
        """放弃等待并移除回复槽。"""
        self._carm.res_pool.pop(self._task_key, None)

ARM_SDK_AVAILABLE = False
Carm = None
CarmSdk = None

try:
    from carm import Carm as CarmSdk

    class Carm(CarmSdk):
        """连接世代（generation）约束下的 Carm SDK 封装，串行化 WebSocket 替换与 pose 下发。"""

        def __init__(self, *args, **kwargs):
            """初始化连接世代锁与生命周期锁。"""
            self._connection_generation_lock = threading.RLock()
            self._lifecycle_lock = threading.RLock()
            self._disconnect_reader = None
            self.last_motion_task_key = None
            self.last_finished_task_key = None
            super().__init__(*args, **kwargs)
            self.on_task_finish(self._record_task_finished)

        def _record_task_finished(self, task_key):
            """SDK 读线程回调：保留最近结束的任务 key，控制线程按 key 比较。"""
            self.last_finished_task_key = task_key

        def connect(self, *args, **kwargs):
            """在上一代 SDK reader 真正退出前拒绝重连。"""
            with self._lifecycle_lock:
                if not self.disconnect():
                    return False
                return super().connect(*args, **kwargs)

        def _create_connection(self):
            """仅在运行标志仍为真时创建连接，避免陈旧世代复活。"""
            with self._connection_generation_lock:
                if not self._running:
                    return None
                result = super()._create_connection()
                # 控制板与控制器在局域网直连且无中间代理，掩码无需密码学随机。
                self.ws.get_mask_key = lambda n: random.getrandbits(8 * n).to_bytes(n, "little")
                return result

        def send_only(self, req):
            """仅发送请求，依赖 SDK 私有方法 _Carm__send。"""
            req["task_key"] = f"{_SEND_ONLY_PREFIX}:{next(_SEND_ONLY_COUNTER)}"
            return self._Carm__send(req)

        def request(self, req, timeout=1):
            """``sdk_nowait()`` 作用域内的运动类命令只发不等，其余照常同步请求。"""
            if getattr(_nowait_scope, "depth", 0) and req.get("command") in _NOWAIT_COMMANDS:
                if self.send_only(req):
                    if req["command"] == "webRecieveTasks":
                        self.last_motion_task_key = req["task_key"]
                    return {"recv": "Task_Recieve"}
                return {"recv": "Task_Reject", "errMsg": "WebSocket not connected"}
            return super().request(req, timeout)

        def request_nowait(self, req, timeout=1.0):
            """登记回复槽后发送，不等待；返回 ``PendingReply``，发送失败返回 None。

            回复由 SDK 接收路径按 task_key 写入 ``res_pool``（依赖 SDK 的 res_pool 格式）。
            """
            event = threading.Event()
            req["task_key"] = f"{_SEND_ONLY_PREFIX}:{next(_SEND_ONLY_COUNTER)}"
            self.res_pool[req["task_key"]] = {"req": req, "event": event}
            if not self._Carm__send(req):
                self.res_pool.pop(req["task_key"], None)
                return None
            return PendingReply(self, req["task_key"], event, time.monotonic() + timeout)

        def disconnect(self):
            """仅当确切的 SDK reader 与 socket 均已消失时返回 True。"""
            with self._lifecycle_lock:
                reader = self._disconnect_reader or getattr(self, "reader", None)
                with self._connection_generation_lock:
                    self._running = False
                    self._reconnect_event.set()
                super().disconnect()
                reader_stopped = bool(
                    reader is None or not reader.is_alive()
                )
                self._disconnect_reader = None if reader_stopped else reader
                return bool(
                    reader_stopped
                    and not getattr(self, "_running", False)
                    and getattr(self, "reader", None) is None
                    and getattr(self, "ws", None) is None
                    and not self.open_ready.is_set()
                )

        def move_joint(self, pos, desire_time=-1, is_sync=True, tool=0, speed=None):
            """下发关节运动；SDK 的 move_joint 没有 speed 参数时按速度档位换算 desire_time。

            Args: speed 速度百分比，None 表示不指定；其余参数同 SDK。
            Returns: SDK 下发结果。
            """
            sdk_move_joint = super().move_joint
            if speed is None:
                return sdk_move_joint(pos, desire_time, is_sync, tool)
            if "speed" in inspect.signature(sdk_move_joint).parameters:
                return sdk_move_joint(pos, desire_time, is_sync, tool, speed=speed)
            return sdk_move_joint(
                pos, MOVE_SPEED_FALLBACK_DESIRE_TIME[speed], is_sync, tool,
            )

        def dispatch_joint_for_connection(
            self,
            joints,
            expected_ws,
            on_admit,
            speed,
            wait,
        ):
            """在连接世代锁内复核连接身份，通过后只发不等地下发关节运动（控制线程使用）。

            目标与反馈帧无关；SDK 读线程约 15 ms 换一帧，这里不比较 state，否则按键会被静默丢弃。

            Args: joints 关节目标；expected_ws 期望的连接；on_admit 准入回调；
                speed 速度百分比；wait 必须为 False（不等待任务完成）。
            Returns: 连接身份失配或未获准入时为 False，否则为是否已写入连接。
            """
            with self._connection_generation_lock:
                if (
                    not self._running
                    or self.ws is not expected_ws
                    or not self.is_connected()
                    or not on_admit()
                ):
                    return False
                with sdk_nowait():
                    return self.move_joint(joints, -1, wait, speed=speed)

        def dispatch_joint_target_for_connection(self, joints, expected_ws, on_admit, on_return=None):
            """在世代锁内下发关节目标（track_joint），成功后回调 on_return。

            Args: joints 关节目标；expected_ws 期望连接；on_admit 准入回调；
                on_return 下发成功且连接仍有效时的回调。
            Returns: 连接失配、未准入或下发失败时为 False 或 SDK 原始结果；
                成功且未提供回调时为 True。
            """
            with self._connection_generation_lock:
                if (not self._running or self.ws is not expected_ws
                        or not self.is_connected() or not on_admit()):
                    return False
                result = super().track_joint(list(joints))
                if result is not True and result != 1:
                    return result
                if on_return is None:
                    return True
                if (
                    not self._running
                    or self.ws is not expected_ws
                    or not self.is_connected()
                ):
                    return False
                on_return()
                return True

        def dispatch_pose_target_for_connection(self, pos, expected_ws, on_admit):
            """在世代锁内按准入回调下发 pose 目标（track_pose），供腰使用。

            Args: pos pose 目标 [x,y,z,qx,qy,qz,qw]；expected_ws 期望连接；on_admit 准入回调。
            Returns: 连接失配或未准入时为 False；否则为 True 或 SDK 原始失败结果。
            """
            with self._connection_generation_lock:
                if (not self._running or self.ws is not expected_ws
                        or not self.is_connected() or not on_admit()):
                    return False
                result = super().track_pose(list(pos))
                return True if result is True or result == 1 else result

        def dispatch_pose_for_connection(
            self,
            pos,
            expected_ws,
            on_return,
        ):
            """在世代锁内下发 pose（track_pose），成功后回调 on_return。

            Args: pos pose 目标；expected_ws 期望连接；on_return 下发成功且连接仍有效时的回调。
            Returns: 连接失配或下发失败时为 False 或 SDK 原始结果，成功时为 True。
            """
            with self._connection_generation_lock:
                if (
                    not self._running
                    or self.ws is not expected_ws
                    or not self.is_connected()
                ):
                    return False
                result = super().track_pose(pos)
                if result is not True:
                    return result
                if (
                    not self._running
                    or self.ws is not expected_ws
                    or not self.is_connected()
                ):
                    return False
                on_return()
                return True

    ARM_SDK_AVAILABLE = True
    logging.info("[Joystick] ✓ Pure Python Carm SDK imported successfully")
except ImportError as e:
    logging.error(f"[Joystick] ✗ Carm SDK import failed: {e}")
