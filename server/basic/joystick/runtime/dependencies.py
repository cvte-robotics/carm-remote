"""joystick 运行时第三方依赖探测与 Carm SDK 封装。

numpy/scipy 从系统镜像导入；Carm SDK 缺失时降级为 ARM_SDK_AVAILABLE=False，
再由 DEPENDENCIES_AVAILABLE 汇总对外暴露。
"""

import logging
import threading

# 导入标准库和第三方库（numpy、scipy 从系统镜像中导入）
# 注意：numpy 和 scipy 已经安装在系统镜像中，不需要从 deb 包导入
try:
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    logging.info(f"[Joystick] ✓ Successfully imported numpy and scipy (numpy version: {np.__version__})")
except ImportError as e:
    logging.error(f"[Joystick] ✗ Failed to import numpy/scipy: {e}")
    logging.error(f"[Joystick] Please ensure numpy and scipy are installed in the system image")
    np = None
    R = None
except Exception as e:
    logging.error(f"[Joystick] ✗ Error importing numpy/scipy: {e}")
    np = None
    R = None

# 导入依赖
DEPENDENCIES_AVAILABLE = False
CORE_DEPENDENCIES_AVAILABLE = np is not None and R is not None
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
            super().__init__(*args, **kwargs)

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
                return super()._create_connection()

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

        def dispatch_joint_for_connection(
            self,
            joints,
            expected_ws,
            expected_state,
            on_admit,
            velocity,
            wait,
        ):
            """在连接世代锁内复核连接身份，通过后下发关节运动。

            Args: joints 关节目标；expected_ws/expected_state 期望的连接身份；
                on_admit 准入回调；velocity/wait 下发动参。
            Returns: 连接身份失配或未获准入时为 False，否则为 SDK 下发结果。
            """
            with self._connection_generation_lock:
                if (
                    not self._running
                    or self.ws is not expected_ws
                    or self.state is not expected_state
                    or not self.is_connected()
                    or not on_admit()
                ):
                    return False
                return super().move_joint(joints, velocity, wait)

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


DEPENDENCIES_AVAILABLE = CORE_DEPENDENCIES_AVAILABLE or ARM_SDK_AVAILABLE
if DEPENDENCIES_AVAILABLE:
    logging.info(
        "[Joystick] Dependencies ready (core=%s, arm_sdk=%s)",
        CORE_DEPENDENCIES_AVAILABLE,
        ARM_SDK_AVAILABLE,
    )
