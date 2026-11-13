"""Quest 直连管理器：板端自动维持待命链路，会话由头显内的连接/初始化/停止请求驱动。"""

import ipaddress
import logging
import queue
import socket
import threading
import time

from quest.adb import is_quest_adb_device, list_adb_devices
from ..runtime.config import JoystickConfigError, identify_arm_model
from ..runtime.dependencies import Carm
from ..runtime.direct_config import load_config, save_config
from .state import SessionPhase

LOOPBACK = "127.0.0.1"
STANDBY_RETRY_SECONDS = 10.0

# 会话相位到直连阶段的映射；IDLE 由链路是否就绪决定（standby / waiting_app）。
_PHASE_STAGES = {
    SessionPhase.SCANNING: "connecting",
    SessionPhase.CONNECTING: "connecting",
    SessionPhase.CONNECTED: "connected",
    SessionPhase.INITIALIZING: "initializing",
    SessionPhase.READY: "ready",
    SessionPhase.STOPPING: "stopping",
    SessionPhase.CLEANUP_FAILED: "error",
}
_PARAM_STAGES = frozenset({"standby", "connected", "ready"})


class QuestDirectManager:
    """Board 端 Quest 直连管理器。

    监视线程只负责发现 Quest 并请求维持待命链路；所有状态读写与生命周期操作都在
    工作线程中串行执行。接收器线程上的回调（控制记录、握手、断线）只入队。
    """

    def __init__(self, controller, interval=1.0, device_lister=list_adb_devices):
        self.controller = controller
        self.interval = interval
        self._device_lister = device_lister
        self._requests = queue.Queue()
        self._serial = None
        self._standby_retry_at = 0.0
        # 当前会话是否由头显发起；Web 会话不属于直连。
        self._owned = False
        # 请求处理期间覆盖显示的过渡阶段（connecting/initializing/stopping）。
        self._busy_stage = None
        self._family = None
        self._model = None
        self._error = (None, None)
        self._config = load_config()
        self._seq = 0
        self._last_payload = None

    def start(self):
        """启动监视线程和工作线程（守护线程，随后端进程退出）。"""
        threading.Thread(target=self._monitor_loop, daemon=True, name="QuestDirectMonitor").start()
        threading.Thread(target=self._worker_loop, daemon=True, name="QuestDirectWorker").start()

    # ---- 接收器线程与控制器回调：只入队 ----

    def enqueue_control_record(self, payload, receiver, generation):
        """接收器线程：转交 APK 上传的控制记录。"""
        self._requests.put(("record", payload))

    def on_negotiated(self, receiver, generation):
        """接收器线程：APK 完成握手，需要重发完整状态。"""
        self._requests.put(("sync", True))

    def on_standby_disconnect(self, reason):
        """接收器线程：待命链路上的 APK 断开。"""
        self._requests.put(("sync", False))

    def update_parameter(self, name, value):
        """控制器在灵敏度/yaw 设置成功后调用；直连会话中（含 Web 修改）写回板端配置。"""
        self._requests.put(("param_changed", name, value))

    # ---- 线程主循环 ----

    def _monitor_loop(self):
        while True:
            time.sleep(self.interval)
            # 工作线程忙（例如初始化移动）时不堆积周期同步。
            if self._requests.empty():
                self._requests.put(("sync", False))

    def _worker_loop(self):
        while True:
            item = self._requests.get()
            try:
                self._dispatch(item)
            except Exception:
                logging.exception("[QuestDirect] Request failed: %s", item[0])

    def _dispatch(self, item):
        kind = item[0]
        if kind == "sync":
            self._sync(force_publish=item[1])
        elif kind == "param_changed":
            if self._owned:
                self._save_parameter(item[1], item[2])
        else:
            self._handle_record(item[1])

    # ---- 链路与状态 ----

    def _quest_module(self):
        return self.controller.source_registry.get_or_create("quest")

    def _sync(self, force_publish=False):
        """维持待命链路，并按需发布直连状态。"""
        serial = next(
            (
                device["serial"] for device in self._device_lister()
                if device.get("state") == "device" and is_quest_adb_device(device)
            ),
            None,
        )
        module = self._quest_module()
        if serial != self._serial:
            if self._serial is not None:
                module.stop_standby()
            self._serial = serial
            self._standby_retry_at = 0.0
        with self.controller._data_lock:
            idle = self.controller.state.phase is SessionPhase.IDLE
        if idle:
            self._owned = False
            self._family = self._model = None
            if serial is not None and time.monotonic() >= self._standby_retry_at:
                self._ensure_standby(module, serial)
        self._publish(force=force_publish)

    def _ensure_standby(self, module, serial):
        # 与 Web 操作互斥：Web 正在扫描/连接时跳过本轮。
        if not self.controller._lifecycle_lock.acquire(blocking=False):
            return
        try:
            if module.start_standby(serial, self):
                return
            logging.warning("[QuestDirect] Failed to set up standby link for %s", serial)
        except OSError as exc:
            logging.warning("[QuestDirect] Standby listener unavailable: %s", exc)
        finally:
            self.controller._lifecycle_lock.release()
        self._standby_retry_at = time.monotonic() + STANDBY_RETRY_SECONDS

    def _stage(self):
        if self._busy_stage is not None:
            return self._busy_stage
        with self.controller._data_lock:
            phase = self.controller.state.phase
        if phase is SessionPhase.IDLE:
            return "standby" if self._quest_module().direct_link_ready() else "waiting_app"
        return _PHASE_STAGES[phase] if self._owned else "waiting_app"

    def _publish(self, force=False):
        stage = self._stage()
        if stage == "connected" and self._family == "A3":
            editable = ["mode", "hand", "other_arm_ip", "scale", "yaw"]
        elif stage in _PARAM_STAGES:
            editable = ["scale", "yaw"]
        else:
            editable = []
        a3 = self._config["a3"]
        payload = {
            "active": self._owned,
            "stage": stage,
            "family": self._family,
            "model": self._model,
            "config": {
                "mode": a3["mode"], "hand": a3["hand"], "other_arm_ip": a3["other_arm_ip"],
                "scale": self._config["scale"], "yaw": self._config["yaw"],
            },
            "editable": editable,
            "error_code": self._error[0],
            "message": self._error[1],
        }
        if payload == self._last_payload and not force:
            return
        self._last_payload = payload
        self.controller.set_direct_state(payload)
        self._seq += 1
        self._quest_module().send_direct_record(dict(payload, type="direct_state", seq=self._seq))

    def _reply(self, request_id, error=None):
        """回复一次请求；error 为 (code, message) 时表示失败，并显示在状态里。"""
        self._error = error or (None, None)
        code, message = self._error
        self._quest_module().send_direct_record({
            "type": "request_result", "request_id": request_id,
            "ok": error is None, "code": code, "message": message,
        })
        self._publish()

    # ---- 头显请求 ----

    def _handle_record(self, payload):
        request_id = payload["request_id"]
        stage = self._stage()
        record_type = payload["type"]
        if record_type == "connect":
            if stage != "standby":
                return self._reply(request_id, ("connect_locked", "当前状态不能连接"))
            self._run_busy("connecting", self._connect, request_id)
        elif record_type == "initialize":
            if stage != "connected":
                return self._reply(request_id, ("initialize_locked", "请先连接后再初始化"))
            self._run_busy("initializing", self._initialize, request_id, payload)
        elif record_type == "stop":
            if stage not in ("connected", "ready", "error") or not self._owned:
                return self._reply(request_id, ("stop_locked", "当前没有可停止的会话"))
            self._run_busy("stopping", self._stop_session, request_id)
        else:
            if stage not in _PARAM_STAGES:
                return self._reply(request_id, ("param_locked", "当前状态不能修改参数"))
            self._set_parameter(request_id, payload)

    def _run_busy(self, stage, action, request_id, *args):
        self._busy_stage = stage
        self._error = (None, None)
        self._publish()
        try:
            error = action(*args)
        except ValueError as exc:
            # user_operation 在 Web 正执行操作时拒绝准入。
            error = ("busy", str(exc))
        finally:
            self._busy_stage = None
        self._reply(request_id, error)

    def _connect(self):
        """扫描 Quest 并接管待命链路，再探测本机臂型号（相当于 Web 的扫描 + 连接）。"""
        controller = self.controller
        with controller.user_operation("scan", {}):
            registry = controller.source_registry
            descriptors = registry.scan_provider("quest", registry.begin_scan())
            descriptor = next((d for d in descriptors if d.candidate.address == self._serial), None)
            if descriptor is None:
                return ("connect_failed", "未找到可用的 Quest")
            result = controller.connect_device(self._serial, descriptor.candidate.name, direct=True)
            if result.outcome.value != "ready":
                return ("connect_failed", result.message)
            self._owned = True
            # 板端保存的灵敏度与 yaw 以配置文件为准。
            controller.set_scale(self._config["scale"])
            controller.set_yaw(self._config["yaw"])
            try:
                self._model = probe_arm_model(LOOPBACK)
            except Exception as exc:
                self._owned = False
                controller.stop_session(stop_mode="hold", reason="quest_direct_probe_failed")
                return ("probe_failed", "读取机械臂型号失败：%s" % exc)
            self._family = "D3" if self._model == "D3" else "A3"
        return None

    def _initialize(self, payload):
        """校验拓扑后初始化（使能并移到初始位，与 Web 初始化一致）。"""
        if self._family == "D3":
            mode, hand, other = "dual", "right", LOOPBACK
        else:
            mode, hand = payload.get("mode"), payload.get("hand")
            other = payload.get("other_arm_ip") if mode == "dual" else None
            if mode not in ("single", "dual") or hand not in ("left", "right"):
                return ("invalid_config", "会话配置不合法")
            if mode == "dual":
                peer_error = check_peer_arm(other)
                if peer_error is not None:
                    return ("invalid_peer", peer_error)
            self._config["a3"] = {"mode": mode, "hand": hand, "other_arm_ip": other}
            save_config(self._config)
        with self.controller.user_operation("init_arm", {}):
            ok = self.controller.init_arm(LOOPBACK, mode=mode, slave_ip=other, master_side=hand)
        if ok:
            return None
        return ("initialize_failed", self.controller.get_last_error() or "初始化失败")

    def _stop_session(self):
        """回零并结束会话；Quest 侧链路保留为待命。"""
        with self.controller.user_operation("disconnect", {}):
            ok = self.controller.stop_session(stop_mode="home", reason="quest_direct_stop")
        if not ok:
            return ("stop_failed", self.controller.get_last_error() or "停止失败")
        self._owned = False
        self._family = self._model = None
        return None

    def _set_parameter(self, request_id, payload):
        name = "scale" if "scale" in payload else "yaw"
        setter = self.controller.set_scale if name == "scale" else self.controller.set_yaw
        if not setter(payload.get(name)):
            return self._reply(request_id, ("invalid_param", "参数不合法"))
        self._save_parameter(name, payload[name])
        self._reply(request_id)

    def _save_parameter(self, name, value):
        value = float(value) if name == "scale" else int(value)
        if self._config[name] == value:
            return
        self._config[name] = value
        save_config(self._config)
        self._publish()


def probe_arm_model(address, timeout=5.0):
    """连接 Carm 读取型号后断开，不使能。

    Raises:
        RuntimeError: SDK 不可用或读取超时。
        JoystickConfigError: 型号不受支持。
    """
    if Carm is None:
        raise RuntimeError("Carm SDK 不可用")
    carm = Carm(addr=address, arm_index=0, port=8090)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            name, dof = str(carm.arm_name or "").strip(), int(carm.arm_dof or 0)
            if name and dof:
                return identify_arm_model(name, dof)
            time.sleep(0.05)
        raise RuntimeError("读取机械臂型号超时")
    finally:
        carm.disconnect()


def check_peer_arm(address):
    """校验异板双臂的另一臂；合格返回 None，否则返回原因。"""
    try:
        ip = ipaddress.IPv4Address(address)
    except (ipaddress.AddressValueError, TypeError):
        return "另一臂 IP 不是合法的 IPv4 地址"
    if ip.is_loopback or ip.is_unspecified or _is_local_address(str(ip)):
        return "另一臂 IP 是本机地址"
    try:
        model = probe_arm_model(str(ip), timeout=3.0)
    except JoystickConfigError:
        return "另一臂型号不受支持"
    except Exception:
        return "另一臂不可达"
    return None if model != "D3" else "另一臂型号不是 A3"


def _is_local_address(ip):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((ip, 0))
        except OSError:
            return False
    return True
