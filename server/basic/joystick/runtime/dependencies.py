import logging
import threading

from .paths import quest_only_providers_enabled

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
BLE_DEPENDENCIES_AVAILABLE = False
CORE_DEPENDENCIES_AVAILABLE = np is not None and R is not None
ARM_SDK_AVAILABLE = False
Carm = None
CarmSdk = None

try:
    from carm import Carm as CarmSdk

    class Carm(CarmSdk):
        """Serialize SDK WebSocket replacement with generation-bound pose sends."""

        def __init__(self, *args, **kwargs):
            self._connection_generation_lock = threading.RLock()
            self._lifecycle_lock = threading.RLock()
            self._disconnect_reader = None
            super().__init__(*args, **kwargs)

        def connect(self, *args, **kwargs):
            """Refuse reconnect until the previous SDK reader has exited."""
            with self._lifecycle_lock:
                if not self.disconnect():
                    return False
                return super().connect(*args, **kwargs)

        def _create_connection(self):
            with self._connection_generation_lock:
                if not self._running:
                    return None
                return super()._create_connection()

        def disconnect(self):
            """Return True only after the exact SDK reader and socket are gone."""
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

        def dispatch_pose_for_connection(
            self,
            pos,
            expected_ws,
            on_return,
        ):
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


if not quest_only_providers_enabled():
    try:
        from surreal.module import SurrealInputModule

        BLE_DEPENDENCIES_AVAILABLE = SurrealInputModule.availability().available
    except ImportError as e:
        logging.warning("[Joystick] Input module availability check failed: %s", e)

DEPENDENCIES_AVAILABLE = CORE_DEPENDENCIES_AVAILABLE or BLE_DEPENDENCIES_AVAILABLE or ARM_SDK_AVAILABLE
if DEPENDENCIES_AVAILABLE:
    logging.info(
        "[Joystick] Dependencies ready (core=%s, ble=%s, arm_sdk=%s)",
        CORE_DEPENDENCIES_AVAILABLE,
        BLE_DEPENDENCIES_AVAILABLE,
        ARM_SDK_AVAILABLE,
    )
