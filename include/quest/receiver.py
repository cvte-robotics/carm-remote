"""Quest control v2 协议的 TCP/NDJSON 接收器（严格状态机）。

任何协议违规都发 ``protocol_error`` 并断开；发送侧经队列异步排空，不阻塞 I/O 线程。
"""

import json
import logging
import math
import queue
import socket
import threading
import time
import uuid
from typing import Any, Callable, Dict, Optional

from .constants import (
    QUEST_PROTOCOL_PREFIX, QUEST_LINE_LIMIT, QUEST_SEND_QUEUE_LIMIT,
    QUEST_PROTOCOL_VERSION, QUEST_HANDSHAKE_TIMEOUT, QUEST_RX_TIMEOUT,
)


class QuestTcpReceiver:
    """严格实现 Quest control v2 的 TCP/NDJSON 接收器。

    单实例可接受多轮客户端连接（每轮递增 ``connection_generation``）。``stop``
    在线程仍存活时返回 False，由调用方重试。
    """

    def __init__(
        self,
        host: str,
        port: int,
        frame_callback: Callable[..., None],
        disconnect_callback: Callable[..., None],
        status_callback: Optional[Callable[..., None]] = None,
    ):
        self.host = host
        self.port = port
        self.frame_callback = frame_callback
        self.disconnect_callback = disconnect_callback
        self.status_callback = status_callback
        self._server_socket = None
        self._client_socket = None
        self._thread = None
        self._running = False
        self._connected = False
        self._send_queue = queue.Queue(maxsize=QUEST_SEND_QUEUE_LIMIT)
        self._send_lock = threading.Lock()
        self._connection_lock = threading.Lock()
        self._connection_generation = 0
        self.protocol_state = "Listening"
        self.protocol_version = None
        self.connection_id = None
        self.client_id = None
        self.connection_negotiated = False
        self.last_valid_rx_monotonic = None
        self.last_heartbeat_seq = None
        self.handshake_event = threading.Event()
        self.first_frame_event = threading.Event()
        self.disconnect_notified = False
        self.last_error = None

    @property
    def connection_generation(self) -> int:
        """返回当前连接 generation（线程安全；每次 accept 递增）。"""
        with self._connection_lock:
            return self._connection_generation

    def start(self):
        """绑定监听端口并启动 I/O 线程（先确保旧线程已停止）。

        Raises:
            RuntimeError: 上一个 receiver 线程尚未停止。
            OSError: 端口绑定/监听失败。
        """
        if self._running:
            return
        if self.stop() is False:
            raise RuntimeError("previous Quest receiver thread did not stop")
        self.first_frame_event.clear()
        self.handshake_event.clear()
        self.last_error = None
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, self.port))
        self._server_socket.listen(1)
        self._server_socket.settimeout(0.2)
        self._running = True
        self.protocol_state = "Listening"
        self._thread = threading.Thread(target=self._run, daemon=True, name="QuestTcpReceiver")
        self._thread.start()

    def stop(self) -> bool:
        """停止接收器并回收线程与 socket。

        Returns:
            True 表示线程已停止；False 表示线程仍存活，需重试。
        """
        self._running = False
        self._connected = False
        current_thread = threading.current_thread()
        server_socket = self._server_socket
        client_socket = self._client_socket
        self._server_socket = None
        if server_socket is not None:
            try:
                server_socket.close()
            except Exception:
                pass
        if client_socket is not None:
            try:
                # 唤醒 recv，但保留写半边，供 receiver I/O 线程排空控制队列。
                client_socket.shutdown(socket.SHUT_RD)
            except OSError:
                pass
        thread = self._thread
        if thread and thread.is_alive() and thread is not current_thread:
            thread.join(timeout=1.0)
        if client_socket is not None:
            try:
                client_socket.close()
            except Exception:
                pass
        self._client_socket = None
        thread_stopped = bool(thread is None or not thread.is_alive())
        if thread_stopped:
            self._thread = None
            self.protocol_state = "Disconnected"
        else:
            self.protocol_state = "Stopping"
        self._clear_send_queue()
        return thread_stopped

    def _clear_send_queue(self):
        """丢弃队列中尚未发出的控制帧（新连接/停止时重置控制通道）。"""
        while True:
            try:
                self._send_queue.get_nowait()
            except queue.Empty:
                return

    def _notify(self, callback, *args):
        """调用单个回调（回调内部异常由调用点自行隔离）。"""
        callback(*args)

    def _notify_status(self, status, generation):
        """向 status_callback 上报协议状态（未注册时静默忽略）。"""
        if self.status_callback:
            self._notify(self.status_callback, status, generation)

    def _queue_control(self, payload: Dict[str, Any]) -> bool:
        """把一条控制帧入队。

        Returns:
            True 表示已入队；队列满时返回 False。
        """
        try:
            self._send_queue.put_nowait(payload)
            return True
        except queue.Full:
            logging.error("[QuestTcpReceiver] Control send queue is full")
            return False

    def _flush_send_queue(self, client):
        """把队列中所有控制帧写入客户端。

        Returns:
            True 表示队列已排空；无客户端或写入失败时返回 False。
        """
        if client is None:
            return False
        while True:
            try:
                payload = self._send_queue.get_nowait()
            except queue.Empty:
                return True
            record = QUEST_PROTOCOL_PREFIX + json.dumps(
                payload, separators=(",", ":"), ensure_ascii=True
            ).encode("utf-8") + b"\n"
            try:
                with self._send_lock:
                    client.sendall(record)
            except OSError as exc:
                self.last_error = str(exc)
                return False

    @staticmethod
    def _decode_record(raw: bytes):
        """解码一行 NDJSON 记录。

        Args:
            raw: 去掉换行后的一行字节。

        Returns:
            (是否控制帧, payload dict)；控制帧以 QUEST_PROTOCOL_PREFIX 开头。

        Raises:
            ValueError: 行过长、JSON 非法或控制帧缺少 type 字段。
        """
        if len(raw) > QUEST_LINE_LIMIT:
            raise ValueError("line_too_long")
        text = raw.decode("utf-8")
        is_control = text.startswith(QUEST_PROTOCOL_PREFIX.decode("ascii"))
        json_text = text[len(QUEST_PROTOCOL_PREFIX):] if is_control else text
        payload = json.loads(json_text)
        if not isinstance(payload, dict):
            raise ValueError("invalid_json_record")
        if is_control and not isinstance(payload.get("type"), str):
            raise ValueError("invalid_control_record")
        return is_control, payload

    @staticmethod
    def _is_valid_pose_payload(payload: Dict[str, Any]) -> bool:
        """校验一帧 pose 数据的字段齐备性与数值合法性。

        Returns:
            True 表示可作为合法输入交给上层。
        """
        if "type" in payload:
            return False
        timestamp = payload.get("ts")
        frame_index = payload.get("fi")
        if (
            isinstance(timestamp, bool)
            or not isinstance(timestamp, (int, float))
            or not math.isfinite(timestamp)
            or isinstance(frame_index, bool)
            or not isinstance(frame_index, (int, float))
            or not math.isfinite(frame_index)
            or int(frame_index) != frame_index
        ):
            return False

        validity = {}
        for key in ("rpv", "lpv"):
            value = payload.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value not in (0, 1)
            ):
                return False
            validity[key] = value == 1

        right_keys = {"px", "py", "pz", "qx", "qy", "qz", "qw"}
        prefixed_right_keys = {f"r{key}" for key in right_keys}
        left_keys = {f"l{key}" for key in right_keys}
        if (
            not right_keys.issubset(payload)
            or not left_keys.issubset(payload)
            or prefixed_right_keys.intersection(payload)
        ):
            return False
        required_keys = right_keys | left_keys
        if any(
            isinstance(payload[key], bool)
            or not isinstance(payload[key], (int, float))
            or not math.isfinite(payload[key])
            for key in required_keys
        ):
            return False

        right_norm_sq = sum(float(payload[f"q{axis}"]) ** 2 for axis in "xyzw")
        left_norm_sq = sum(float(payload[f"lq{axis}"]) ** 2 for axis in "xyzw")
        return (
            (not validity["rpv"] or right_norm_sq > 1e-24)
            and (not validity["lpv"] or left_norm_sq > 1e-24)
        )

    def _send_protocol_error(self, code: str) -> bool:
        """把一条 protocol_error 控制帧入队。"""
        return self._queue_control({"type": "protocol_error", "code": code})

    def send_session_end(self, reason: str = "backend_stop", fatal: bool = True) -> bool:
        """在本地清理之前直接写入会话结束通知。

        Args:
            reason: 结束原因。
            fatal: 是否为致命结束。

        Returns:
            True 表示已写入 socket；未连接或写入失败时返回 False。
        """
        client = self._client_socket
        if not self._connected or client is None:
            return False
        record = QUEST_PROTOCOL_PREFIX + json.dumps(
            {"type": "session_end", "reason": reason, "fatal": fatal},
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8") + b"\n"
        try:
            with self._send_lock:
                client.sendall(record)
            return True
        except OSError as exc:
            self.last_error = str(exc)
            return False

    def _run(self):
        """I/O 线程主循环：accept → 握手 → 收帧 → 断开并通知。"""
        try:
            while self._running:
                server_socket = self._server_socket
                if server_socket is None:
                    if self._running:
                        raise OSError("Quest listener socket is unavailable")
                    break
                try:
                    client, _ = server_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._running:
                        raise
                    break

                with self._connection_lock:
                    self._connection_generation += 1
                    generation = self._connection_generation
                self._client_socket = client
                self._client_socket.settimeout(0.1)
                self._connected = True
                self.disconnect_notified = False
                self.protocol_state = "Negotiating"
                self.protocol_version = None
                self.connection_id = None
                self.client_id = None
                self.connection_negotiated = False
                self.last_valid_rx_monotonic = time.monotonic()
                self.last_heartbeat_seq = None
                self.handshake_event.clear()
                self.first_frame_event.clear()
                self._clear_send_queue()
                self._notify_status("Negotiating", generation)
                buffer = bytearray()
                close_reason = "quest_stream_disconnected"
                negotiated = False

                while self._running:
                    if not self._flush_send_queue(self._client_socket):
                        close_reason = "quest_send_failed"
                        break
                    try:
                        chunk = self._client_socket.recv(4096)
                        if not chunk:
                            self._flush_send_queue(self._client_socket)
                            close_reason = "quest_stream_disconnected"
                            break
                        buffer.extend(chunk)
                    except socket.timeout:
                        pass
                    except (UnicodeError, OSError):
                        close_reason = "quest_stream_read_failed"
                        break

                    if len(buffer) > QUEST_LINE_LIMIT and b"\n" not in buffer:
                        self._send_protocol_error("line_too_long")
                        close_reason = "quest_protocol_error"
                        self._flush_send_queue(self._client_socket)
                        break

                    while b"\n" in buffer:
                        raw, _, remainder = buffer.partition(b"\n")
                        buffer = bytearray(remainder)
                        raw = raw.rstrip(b"\r")
                        if not raw:
                            continue
                        try:
                            is_control, payload = self._decode_record(raw)
                        except UnicodeDecodeError:
                            self._send_protocol_error("invalid_utf8")
                            close_reason = "quest_protocol_error"
                            break
                        except json.JSONDecodeError:
                            self._send_protocol_error("invalid_json")
                            close_reason = "quest_protocol_error"
                            break
                        except ValueError as exc:
                            code = str(exc)
                            self._send_protocol_error(code)
                            close_reason = "quest_protocol_error"
                            break

                        record_type = payload.get("type") if is_control else None
                        now = time.monotonic()
                        if not negotiated:
                            if (
                                not is_control
                                or record_type != "hello"
                                or not self._is_strict_int(payload.get("protocol"))
                                or payload.get("protocol") != QUEST_PROTOCOL_VERSION
                            ):
                                self._send_protocol_error("version_mismatch")
                                close_reason = "quest_protocol_error"
                                break
                            if (
                                not isinstance(payload.get("client_id"), str)
                                or not payload.get("client_id")
                                or not isinstance(payload.get("app_version"), str)
                                or not payload.get("app_version")
                                or not self._is_strict_int(payload.get("connection_seq"))
                                or payload.get("connection_seq") <= 0
                            ):
                                self._send_protocol_error("invalid_hello")
                                close_reason = "quest_protocol_error"
                                break
                            self.client_id = payload["client_id"]
                            self.protocol_version = QUEST_PROTOCOL_VERSION
                            self.connection_id = str(uuid.uuid4())
                            if not self._queue_control({
                                "type": "hello_ack",
                                "protocol": QUEST_PROTOCOL_VERSION,
                                "connection_id": self.connection_id,
                            }):
                                close_reason = "quest_send_failed"
                                break
                            negotiated = True
                            self.connection_negotiated = True
                            self.last_valid_rx_monotonic = now
                            self.protocol_state = "Healthy"
                            self.handshake_event.set()
                            self._notify_status("Healthy", generation)
                            continue

                        if not is_control:
                            if not self._is_valid_pose_payload(payload):
                                self._send_protocol_error("invalid_pose_frame")
                                close_reason = "quest_protocol_error"
                                break
                            self.last_valid_rx_monotonic = now
                            try:
                                self.frame_callback(payload, generation)
                            except Exception:
                                logging.exception("[QuestTcpReceiver] Invalid Quest pose callback")
                                self._send_protocol_error("invalid_pose_frame")
                                close_reason = "quest-invalid-frame"
                                break
                            self.first_frame_event.set()
                            continue
                        if record_type == "heartbeat":
                            seq = payload.get("seq")
                            if (
                                not self._is_strict_int(seq)
                                or seq <= 0
                                or (
                                    self.last_heartbeat_seq is not None
                                    and seq <= self.last_heartbeat_seq
                                )
                            ):
                                self._send_protocol_error("invalid_heartbeat")
                                close_reason = "quest_protocol_error"
                                break
                            self.last_heartbeat_seq = seq
                            self.last_valid_rx_monotonic = now
                            self._notify_status("Healthy", generation)
                            if not self._queue_control({"type": "heartbeat_ack", "seq": seq}):
                                close_reason = "quest_send_failed"
                                break
                            continue
                        if record_type in {"hello", "hello_ack", "heartbeat_ack", "protocol_error", "session_end"}:
                            self._send_protocol_error("invalid_control_record")
                            close_reason = "quest_protocol_error"
                            break

                        self._send_protocol_error("invalid_control_record")
                        close_reason = "quest_protocol_error"
                        break

                    if close_reason in {"quest_protocol_error", "quest-invalid-frame"}:
                        self._flush_send_queue(self._client_socket)
                        break
                    now = time.monotonic()
                    if not negotiated and now - self.last_valid_rx_monotonic >= QUEST_HANDSHAKE_TIMEOUT:
                        close_reason = "quest-handshake-timeout"
                        break
                    if negotiated and now - self.last_valid_rx_monotonic >= QUEST_RX_TIMEOUT:
                        self.protocol_state = "Degraded"
                        self._notify_status("Degraded", generation)
                        close_reason = "quest-rx-timeout"
                        break

                self._connected = False
                try:
                    self._client_socket.close()
                except Exception:
                    pass
                self._client_socket = None
                if self._running and not self.disconnect_notified:
                    self.disconnect_notified = True
                    self.protocol_state = "Disconnected"
                    self._notify_status("Disconnected", generation)
                    self._notify(self.disconnect_callback, close_reason, generation)
        except Exception as exc:
            self.last_error = str(exc)
            logging.exception("[QuestTcpReceiver] Quest receiver failed")
            was_running = self._running
            self._running = False
            self.protocol_state = "ListenerFailed"
            generation = self.connection_generation
            client_socket = self._client_socket
            server_socket = self._server_socket
            self._client_socket = None
            self._server_socket = None
            for current_socket in (client_socket, server_socket):
                if current_socket is None:
                    continue
                try:
                    current_socket.close()
                except Exception:
                    pass
            try:
                self._notify_status("ListenerFailed", generation)
            except Exception:
                logging.exception("[QuestTcpReceiver] Listener failure status callback failed")
            # listener 失败是新事件，即使旧连接已上报断开也要送达会话持有者。
            if was_running:
                self.disconnect_notified = True
                try:
                    self._notify(self.disconnect_callback, "quest_receiver_error", generation)
                except Exception:
                    logging.exception("[QuestTcpReceiver] Listener failure cleanup callback failed")
        finally:
            self._connected = False

    @staticmethod
    def _is_strict_int(value) -> bool:
        """严格整数判定（排除 bool，供 seq/protocol/connection_seq 校验）。"""
        return isinstance(value, int) and not isinstance(value, bool)
