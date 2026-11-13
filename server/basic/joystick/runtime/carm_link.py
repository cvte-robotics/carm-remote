"""同一控制端点的 Carm SDK 共享连接。

一个 ``control_host`` 只建一条 WebSocket、一个读线程、每帧一次 ``json.loads``；各
``arm_index`` 以 ``CarmChannel`` 视图使用这条连接。控制器按请求报文里的 ``arm_index``
路由，状态帧 ``webSendRobotState`` 本身包含全部臂。
"""

import json
import logging
import random
import socket
import threading

from .dependencies import Carm

try:
    import websocket
except ImportError:  # pragma: no cover - 与 SDK 一同缺失时共享连接不可用
    websocket = None


_RECV_CHUNK = 65536


def _mask_key(length):
    # 控制板与控制器在局域网直连且无中间代理，掩码无需密码学随机。
    return random.getrandbits(8 * length).to_bytes(length, "little")


def _install_prefetch(ws):
    """把 websocket-client 帧缓冲的下层 recv 换成 64 KiB 预读。

    依赖 websocket-client 内部属性 ``frame_buffer.recv``（0.5x 起各版本均有）。读帧按
    帧头/扩展长度/正文分 3 次取数据，预读后积压的多帧只需一次系统调用。本连接由读线程
    阻塞循环读取、不做 select，缓冲里剩余的完整帧会在下一次读取时直接被解析。
    """
    recv = ws.frame_buffer.recv
    buffer = [b"", 0]

    def prefetch(bufsize):
        data, offset = buffer
        if offset >= len(data):
            data, offset = recv(_RECV_CHUNK), 0
        end = offset + bufsize
        buffer[0], buffer[1] = data, end
        return data[offset:end]

    ws.frame_buffer.recv = prefetch


class CarmLink:
    """一个 control_host 的 WebSocket、读线程与按臂分发。"""

    def __init__(self, host, port=8090):
        self.host, self.port = host, port
        self.ws = None
        self.state = None
        self.open_ready = threading.Event()
        self.channels = {}
        self._connection_generation_lock = threading.RLock()
        self._running = True
        self._stop_event = threading.Event()
        self.reader = threading.Thread(
            target=self._recv_loop, name="CarmLink-%s" % host, daemon=True)
        self.reader.start()

    @property
    def running(self):
        return self._running

    def register(self, channel):
        """登记通道；连接已就绪时为它补取限位（未就绪时由建连统一获取）。"""
        with self._connection_generation_lock:
            self.channels[channel.arm_index] = channel
            ready = self.open_ready.is_set()
        if ready:
            self._start_fetch_limits([channel])

    def unregister(self, channel):
        with self._connection_generation_lock:
            if self.channels.get(channel.arm_index) is channel:
                del self.channels[channel.arm_index]
            return bool(self.channels)

    def _start_fetch_limits(self, channels):
        # 与 SDK on_open 相同：限位请求的回复要由读线程分发，不能在读线程里同步等待。
        threading.Thread(target=self._fetch_limits, args=(channels,),
                         name="CarmLinkLimits-%s" % self.host, daemon=True).start()

    @staticmethod
    def _fetch_limits(channels):
        for channel in channels:
            try:
                if not channel.is_connected():
                    continue
                limits = channel.get_limits()
                if limits:
                    channel.limit = limits
                eeff = channel.get_eeff_config()
                if eeff:
                    channel.eeff_limit = eeff
            except Exception:
                logging.exception("[CarmLink] 获取 arm_index=%s 限位失败", channel.arm_index)

    def _open(self):
        """建立新连接；每次重连都是新的 WebSocket 对象（后端按对象身份识别换连接）。"""
        ws = websocket.WebSocket(get_mask_key=_mask_key, skip_utf8_validation=True)
        ws.connect("ws://%s:%s" % (self.host, self.port), timeout=1.0)
        # 建连超时只用于握手；之后与 SDK run_forever 一样阻塞读取。
        ws.settimeout(None)
        _install_prefetch(ws)
        return ws

    def _serve(self, ws):
        """发布连接并在读线程内阻塞收帧，直到断开或关闭。"""
        with self._connection_generation_lock:
            if not self._running:
                ws.shutdown()
                return
            self.ws = ws
            self.open_ready.set()
            channels = list(self.channels.values())
        self._start_fetch_limits(channels)
        try:
            while self._running:
                # 对端 ping 由 websocket-client 自动回 pong；close 帧回 close 后返回。
                opcode, frame = ws.recv_data_frame()
                if opcode == websocket.ABNF.OPCODE_CLOSE:
                    break
                self._dispatch(frame.data)
        except Exception as error:
            if self._running:
                logging.warning("[CarmLink] %s 连接断开：%r", self.host, error)
        finally:
            with self._connection_generation_lock:
                self.open_ready.clear()
                channels = list(self.channels.values())
            for channel in channels:
                channel._Carm__abort_all_tasks()
            ws.shutdown()

    def _dispatch(self, payload):
        """每帧解析一次，按 SDK 自身的单臂语义交给各通道处理。"""
        try:
            message = json.loads(payload)
            channels = list(self.channels.values())
            if message.get("command") == "webSendRobotState":
                # SDK __cbk_status：替换 state、更新回调、本臂错误中止、本臂 last_task_key 唤醒。
                for channel in channels:
                    channel._Carm__cbk_status(message)
            else:
                # SDK __response_op：各通道只在自己的 res_pool 里按 task_key 匹配。
                for channel in channels:
                    channel._Carm__response_op(message)
        except Exception:
            logging.exception("[CarmLink] %s 消息处理失败", self.host)

    def _recv_loop(self):
        delay = 1.0
        while self._running:
            try:
                ws = self._open()
            except Exception as error:
                logging.debug("[CarmLink] %s 连接失败：%s", self.host, error)
            else:
                delay = 1.0
                self._serve(ws)
            if self._stop_event.wait(delay):
                break
            delay = min(delay * 1.5, 10.0)

    def close(self):
        """停止重连并关闭连接；读线程确实退出时返回 True，否则可再次调用重试。"""
        with self._connection_generation_lock:
            self._running = False
            ws = self.ws
        self._stop_event.set()
        sock = ws.sock if ws is not None else None
        if sock is not None:
            # 只有 shutdown 能唤醒另一线程里阻塞的 recv；socket 可能已被读线程关闭。
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.reader is not threading.current_thread():
            self.reader.join(timeout=3.0)
        if self.reader.is_alive():
            return False
        with self._connection_generation_lock:
            self.ws = None
            self.open_ready.clear()
        return True


class CarmLinkRegistry:
    """按 control_host 复用 CarmLink，限制同一 host/arm_index 只有一个活跃通道。"""

    def __init__(self, port=8090):
        self.port = port
        self._links = {}
        self._lock = threading.Lock()

    def channel(self, host, arm_index=0, channel_cls=None):
        """返回 host 上 arm_index 的通道，必要时新建 Link。

        Raises: RuntimeError 上次关闭失败的 Link 仍未退出，或同一 host/arm_index 已有活跃通道。
        """
        with self._lock:
            link = self._links.get(host)
            if link is not None and not link.running:
                if not link.close():
                    raise RuntimeError("%s 的旧 SDK 连接仍未退出" % host)
                link = None
            if link is None:
                link = self._links[host] = CarmLink(host, self.port)
            if arm_index in link.channels:
                raise RuntimeError("%s arm_index=%s 已有活跃 SDK 通道" % (host, arm_index))
            channel = (channel_cls or CarmChannel)(link, arm_index, self)
            link.register(channel)
            return channel

    def release(self, channel):
        """注销通道；最后一个通道释放时关闭 Link，关闭失败返回 False 并保留 Link 以便重试。"""
        link = channel.link
        with self._lock:
            if link.unregister(channel):
                return True
            if not link.close():
                return False
            if self._links.get(link.host) is link:
                del self._links[link.host]
            return True


_RELEASED = threading.Event()


class CarmChannel(Carm if Carm is not None else object):
    """一个 arm_index 的 Carm SDK 视图：保留 SDK 业务方法与后端 dispatch_*，不拥有连接。

    不调用 SDK ``__init__``（不建连、不起读线程），只初始化 SDK 方法读取的实例属性；
    ``ws``/``state``/``open_ready``/世代锁委托给所属 Link，``res_pool``/``task_pool``
    每通道独立，因此 SDK 的按臂错误中止与任务等待语义不变。
    """

    def __init__(self, link, arm_index, registry):
        self.link, self.registry = link, registry
        self.arm_index = arm_index
        self.addr, self.port = link.host, link.port
        self.low_state_data = None
        self.last_msg = None
        self.limit = None
        self.eeff_limit = None
        self.res_pool = {}
        self.task_pool = {}
        self._released = False
        self.last_motion_task_key = None
        self.last_finished_task_key = None
        self.call_back = {
            "taskFinished": self._record_task_finished,
            "onCarmError": self._Carm__cbk_error,
            "updateRobotState": self._Carm__cbk_update,
        }

    @property
    def ws(self):
        return self.link.ws

    @property
    def state(self):
        return self.link.state

    @state.setter
    def state(self, value):
        # SDK __cbk_status 会写回同一帧对象，所有通道看到的是同一份 state。
        self.link.state = value

    @property
    def open_ready(self):
        # SDK __send / is_connected 只看这个事件；释放后的通道永远未连接。
        return _RELEASED if self._released else self.link.open_ready

    @property
    def _running(self):
        return self.link.running and not self._released

    @property
    def _connection_generation_lock(self):
        return self.link._connection_generation_lock

    def connect(self, timeout=1.0):
        """等待共享连接就绪。"""
        return not self._released and self.link.open_ready.wait(timeout)

    def disconnect(self):
        """释放本通道；与 Carm.disconnect 一致，底层连接未能关闭时返回 False，可重试。"""
        with self._connection_generation_lock:
            self._released = True
        self._Carm__abort_all_tasks()
        return self.registry.release(self)
