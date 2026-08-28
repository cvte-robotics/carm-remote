"""Reusable HTTP/SSE adapter for the desktop joystick composition."""

from __future__ import annotations

import json
import logging
import time
import uuid
from typing import Callable, Dict, Optional, Tuple

from flask import Response, request, stream_with_context


ScanHandler = Callable[[object, Dict[str, object]], Dict[str, object]]
OperationRunner = Callable[
    [Dict[str, object], Callable[[], object]],
    Tuple[bool, object],
]


def _json_response(msg: str = "successed", code: int = 1, body=None, status: int = 200):
    payload = dict(body or {})
    payload["msg"] = msg
    payload["code"] = code
    response = Response(
        json.dumps(payload, ensure_ascii=False),
        status=status,
        content_type="application/json",
    )
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
    return response


def _failed(msg: str):
    return _json_response(msg=msg, code=0)


def _default_scan(controller, body: Dict[str, object]) -> Dict[str, object]:
    del body
    return {"data": controller.scan_devices()}


def register_desktop_joystick_routes(
    app,
    controller,
    *,
    scan_handler: Optional[ScanHandler] = None,
    operation_runner: Optional[OperationRunner] = None,
) -> None:
    """Register the desktop API subset against one injected controller."""

    scan_handler = scan_handler or _default_scan

    def run_operation(body, operation):
        if operation_runner is None:
            return True, operation()
        return operation_runner(body, operation)

    def require_available(handler):
        def wrapped(*args, **kwargs):
            if not getattr(controller, "available", False):
                return _failed("手柄遥操功能不可用")
            return handler(*args, **kwargs)

        wrapped.__name__ = handler.__name__
        return wrapped

    @app.post("/api/joystick/scan")
    @require_available
    def desktop_joystick_scan():
        try:
            body = request.get_json(silent=True) or {}
            accepted, result = run_operation(
                body,
                lambda: scan_handler(controller, body),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            devices = result.get("data") or []
            if not devices:
                error_msg = controller.get_last_error()
                if error_msg:
                    return _failed(error_msg)
            return _json_response("扫描完成", body=result)
        except Exception as exc:
            logging.exception("[DesktopAPI] Scan failed")
            return _failed(f"扫描设备失败: {exc}")

    @app.post("/api/joystick/connect")
    @require_available
    def desktop_joystick_connect():
        try:
            body = request.get_json(silent=True) or {}
            address = body.get("address")
            name = body.get("name") or "Unknown"
            if not address:
                return _failed("设备地址不能为空")
            accepted, success = run_operation(
                body,
                lambda: controller.connect_device(address, name),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            if success:
                return _json_response("连接成功")
            return _failed(
                controller.get_last_error()
                or "连接失败，请检查设备是否在范围内或设备是否已连接"
            )
        except Exception as exc:
            logging.exception("[DesktopAPI] Connect failed")
            return _failed(f"连接设备失败: {exc}")

    @app.post("/api/joystick/session_config")
    @require_available
    def desktop_joystick_session_config():
        try:
            body = request.get_json(silent=True) or {}
            accepted, success = run_operation(
                body,
                lambda: controller.set_session_config(
                    mode=body.get("mode"),
                    master_side=body.get("master_side"),
                    view_side=body.get("view_side"),
                    slave_ip=body.get("slave_ip"),
                ),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            return _json_response("设置成功") if success else _failed("设置会话配置失败")
        except Exception:
            logging.exception("[DesktopAPI] Session config failed")
            return _failed("设置会话配置失败")

    @app.post("/api/joystick/init_arm")
    @require_available
    def desktop_joystick_init_arm():
        try:
            body = request.get_json(silent=True) or {}
            mode = body.get("mode", "single")
            master_ip = body.get("master_ip") or body.get("ip", "10.42.0.101")
            slave_ip = body.get("slave_ip")
            master_side = body.get("master_side", "right")
            accepted, success = run_operation(
                body,
                lambda: controller.init_arm(
                    master_ip,
                    mode=mode,
                    slave_ip=slave_ip,
                    master_side=master_side,
                ),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            if success:
                return _json_response("初始化成功")
            return _failed(
                controller.get_last_error()
                or f"机械臂初始化失败 (IP: {master_ip})，请检查 IP 地址和网络连接"
            )
        except Exception as exc:
            logging.exception("[DesktopAPI] Arm initialization failed")
            return _failed(f"初始化机械臂失败: {exc}")

    @app.post("/api/joystick/calibration")
    @require_available
    def desktop_joystick_calibration():
        try:
            value = (request.get_json(silent=True) or {}).get("value")
            if value is None or value < 0 or value > 3:
                return _failed("标定值必须在 0-3 之间")
            return _json_response("设置成功") if controller.set_calibration(value) else _failed("设置失败")
        except Exception:
            logging.exception("[DesktopAPI] Calibration failed")
            return _failed("设置标定失败")

    @app.post("/api/joystick/scale")
    @require_available
    def desktop_joystick_scale():
        try:
            value = (request.get_json(silent=True) or {}).get("value")
            if value is None or value < 0.1 or value > 1.0:
                return _failed("灵敏度值必须在 0.1-1.0 之间")
            return _json_response("设置成功") if controller.set_scale(value) else _failed("设置失败")
        except Exception:
            logging.exception("[DesktopAPI] Scale update failed")
            return _failed("设置灵敏度失败")

    @app.post("/api/joystick/view_side")
    @require_available
    def desktop_joystick_view_side():
        try:
            value = (request.get_json(silent=True) or {}).get("value")
            if value not in ("left", "right"):
                return _failed("查看侧必须为 left 或 right")
            return _json_response("设置成功") if controller.set_view_side(value) else _failed("设置失败")
        except Exception:
            logging.exception("[DesktopAPI] View side update failed")
            return _failed("设置查看侧失败")

    @app.post("/api/joystick/disconnect")
    @require_available
    def desktop_joystick_disconnect():
        try:
            body = request.get_json(silent=True) or {}
            reason = body.get("reason", "unknown")
            stop_mode = body.get("stop_mode", "hold")
            accepted, success = run_operation(
                body,
                lambda: controller.stop_session(
                    stop_mode=stop_mode,
                    reason=reason,
                ),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            if success:
                return _json_response(f"已停止手柄遥操 ({reason})")
            return _failed("停止手柄遥操失败")
        except Exception:
            logging.exception("[DesktopAPI] Disconnect failed")
            return _failed("停止手柄遥操异常")

    @app.get("/api/joystick/status")
    def desktop_joystick_status():
        try:
            if not getattr(controller, "available", False):
                return _json_response(body={"data": {"available": False, "status": "not_available"}})
            return _json_response(body={"data": controller.get_status()})
        except Exception:
            logging.exception("[DesktopAPI] Status failed")
            return _failed("获取状态失败")

    @app.get("/api/joystick/events")
    @require_available
    def desktop_joystick_events():
        connection_id = str(uuid.uuid4())[:8]

        def build_payload(snapshot, heartbeat=False):
            status = snapshot["status"]
            transport = snapshot["transport"]
            payload = {
                "sequence": snapshot["sequence"],
                "pose": snapshot["pose"],
                "button": snapshot["button"],
                "gripper_pose": snapshot["gripper_pose"],
                "gripper_tau": snapshot["gripper_tau"],
                "status": {
                    "ble_connected": status["ble_connected"],
                    "status": status["status"],
                    "battery_level": status["battery_level"],
                    "rssi": status["rssi"],
                    "source_type": status["source_type"],
                    "mode": status["mode"],
                    "master_side": status["master_side"],
                },
                "device": snapshot["device"],
                "arms": snapshot["arms"],
                "hands": snapshot["hands"],
                "stream_terminal": snapshot["stream_terminal"],
                "emit_disconnect_frame": snapshot["emit_disconnect_frame"],
                "transport": {
                    "state": transport["state"],
                    "protocol": transport["protocol"],
                    "connection_id": transport["connection_id"],
                    "last_rx_age_ms": transport["last_rx_age_ms"],
                    "disconnect_reason": transport["disconnect_reason"],
                },
                "timestamp": time.time(),
            }
            if heartbeat:
                payload["heartbeat"] = True
                payload["pose"] = None
                payload["button"] = None
            return payload

        def event_stream():
            last_sequence = -1
            last_heartbeat = time.time()
            try:
                while True:
                    snapshot = controller.get_stream_snapshot()
                    sequence = snapshot["sequence"]
                    emit_disconnect = bool(
                        snapshot["emit_disconnect_frame"] and sequence != last_sequence
                    )
                    if snapshot["stream_terminal"] and not emit_disconnect:
                        return
                    if sequence != last_sequence:
                        yield f"data: {json.dumps(build_payload(snapshot), ensure_ascii=False)}\n\n"
                        last_sequence = sequence
                        last_heartbeat = time.time()
                        if emit_disconnect:
                            return
                    elif time.time() - last_heartbeat > 3:
                        yield f"data: {json.dumps(build_payload(snapshot, heartbeat=True), ensure_ascii=False)}\n\n"
                        last_heartbeat = time.time()
                    time.sleep(0.05)
            except GeneratorExit:
                logging.info("[DesktopSSE-%s] Connection closed", connection_id)
            except Exception:
                logging.exception("[DesktopSSE-%s] Stream failed", connection_id)

        response = Response(
            stream_with_context(event_stream()),
            mimetype="text/event-stream",
        )
        response.headers["Cache-Control"] = "no-cache"
        response.headers["Connection"] = "keep-alive"
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["X-Accel-Buffering"] = "no"
        return response
