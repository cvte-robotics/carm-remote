"""控制板 Web 与桌面 joystick profile 复用的 HTTP/SSE 路由。"""

from __future__ import annotations

from functools import wraps
from contextlib import nullcontext
import json
import logging
import time
import uuid
from typing import Callable, Dict, Optional, Tuple

from flask import Response, request, stream_with_context
from basic.joystick.session.observation import project_sse_payload


ScanHandler = Callable[[object, Dict[str, object]], Dict[str, object]]
OperationRunner = Callable[..., Tuple[bool, object]]


def _json_response(msg: str = "successed", code: int = 1, body=None, status: int = 200):
    """构造带 CORS 头的统一 JSON 响应。"""
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
    """构造 code=0 的失败响应。"""
    return _json_response(msg=msg, code=0)


def _default_scan(controller, body: Dict[str, object]) -> Dict[str, object]:
    """默认扫描处理器：直接调用 controller.scan_devices()。"""
    del body
    return {"data": controller.scan_devices()}


def register_joystick_routes(
    app,
    controller,
    scan_handler: Optional[ScanHandler] = None,
    operation_runner: Optional[OperationRunner] = None,
    include_maintenance_routes: bool = True,
    allow_get_disconnect: bool = True,
    *,
    dependencies_available=True,
) -> None:
    """注册一套 profile 感知的 joystick HTTP/SSE 路由。

    Args:
        app: Flask 应用。
        controller: 控制器实例或其 getter。
        scan_handler: 可覆盖的扫描处理器。
        operation_runner: 桌面端用户操作准入包装。
        include_maintenance_routes: 是否注册维护类路由。
        allow_get_disconnect: 是否允许用 GET 停止会话。
        dependencies_available: 依赖可用性（布尔或可调用）。
    """

    custom_scan_handler = scan_handler is not None
    scan_handler = scan_handler or _default_scan

    def dependencies_ready() -> bool:
        """解析 dependencies_available 的布尔/可调用两种形态。"""
        return bool(
            dependencies_available()
            if callable(dependencies_available)
            else dependencies_available
        )

    def current_controller():
        """解析 controller 的实例/getter 两种形态。"""
        return controller() if callable(controller) else controller

    def run_operation(body, operation, *, cancel=False):
        """在用户操作准入上下文内执行操作。

        Returns: (是否被准入接受, 操作结果)。
        """
        def admitted_operation():
            """在准入上下文内执行操作本体。"""
            owner = current_controller()
            admission = getattr(owner, "user_operation", None)
            with admission(request.path.rsplit("/", 1)[-1], body) if admission else nullcontext():
                return operation()
        if operation_runner is None:
            return True, admitted_operation()
        return operation_runner(body, admitted_operation, cancel=cancel)

    def require_available(handler):
        """包装 handler：统一处理 OPTIONS 预检与依赖/功能不可用。"""
        @wraps(handler)
        def wrapped(*args, **kwargs):
            """执行预检与可用性检查后进入原 handler。"""
            if request.method == "OPTIONS":
                return _json_response()
            if not dependencies_ready():
                return _failed("手柄遥操依赖模块不可用")
            joystick_controller = current_controller()
            if not getattr(joystick_controller, "available", False):
                return _failed("手柄遥操功能不可用")
            return handler(joystick_controller, *args, **kwargs)

        return wrapped

    post_methods = ["POST", "OPTIONS"] if include_maintenance_routes else ["POST"]
    get_methods = ["GET", "OPTIONS"] if include_maintenance_routes else ["GET"]
    disconnect_methods = list(post_methods)
    if allow_get_disconnect:
        disconnect_methods.insert(1, "GET")

    @app.route("/api/joystick/scan", methods=post_methods)
    @require_available
    def joystick_scan(joystick_controller):
        """POST/OPTIONS /api/joystick/scan：扫描输入设备并返回候选列表。"""
        try:
            body = request.get_json(silent=True) or {}
            accepted, result = run_operation(
                body,
                lambda: scan_handler(joystick_controller, body),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            devices = result.get("data") or []
            if not devices:
                error_msg = joystick_controller.get_last_error()
                if error_msg:
                    logging.info("[JoystickAPI] Scan failed: %s", error_msg)
                    return _failed(error_msg)
            message = "扫描完成" if custom_scan_handler else "successed"
            return _json_response(message, body=result)
        except Exception as exc:
            logging.exception("[JoystickAPI] Scan failed")
            return _failed(f"扫描设备失败: {exc}")

    @app.route("/api/joystick/connect", methods=post_methods)
    @require_available
    def joystick_connect(joystick_controller):
        """POST/OPTIONS /api/joystick/connect：按 address/name 连接设备。"""
        try:
            body = request.get_json(silent=True) or {}
            address = body.get("address")
            name = body.get("name") or "Unknown"
            if not address:
                return _failed("设备地址不能为空")
            accepted, result = run_operation(
                body,
                lambda: joystick_controller.connect_device(address, name),
            )
            if not accepted:
                return _failed("桌面请求已失效")
            outcome = result.outcome.value
            response_body = {"data": {"outcome": outcome}}
            if outcome == "ready":
                return _json_response(result.message or "连接成功", body=response_body)
            if outcome == "pending":
                return _json_response(result.message or "等待设备连接", body=response_body)
            return _json_response(
                msg=(
                    result.message
                    or joystick_controller.get_last_error()
                    or "连接失败，请检查设备是否在范围内或设备是否已连接"
                ),
                code=0,
                body=response_body,
            )
        except Exception as exc:
            logging.exception("[JoystickAPI] Connect failed")
            return _failed(f"连接设备失败: {exc}")

    @app.route("/api/joystick/disconnect", methods=disconnect_methods)
    @require_available
    def joystick_disconnect(joystick_controller):
        """GET/POST /api/joystick/disconnect：停止会话（GET 走 query，默认 hold）。"""
        try:
            body = request.get_json(silent=True) or {}
            if request.method == "GET":
                reason = request.args.get("reason", "unknown")
                stop_mode = "hold"
            else:
                reason = body.get("reason", "unknown")
                stop_mode = body.get("stop_mode", "hold")
            accepted, success = run_operation(
                body,
                lambda: joystick_controller.stop_session(
                    stop_mode=stop_mode,
                    reason=reason,
                ),
                cancel=True,
            )
            if not accepted:
                return _failed("桌面请求已失效")
            status_body = {"data": {"status": joystick_controller.get_status()}}
            if success:
                return _json_response(
                    f"已停止手柄遥操 ({reason})",
                    body=status_body,
                )
            return _json_response(
                msg=joystick_controller.get_last_error() or "停止手柄遥操失败",
                code=0,
                body=status_body,
            )
        except Exception:
            logging.exception("[JoystickAPI] Disconnect failed")
            return _json_response(
                msg="停止手柄遥操异常",
                code=0,
                body={"data": {"status": joystick_controller.get_status()}},
            )

    @app.route("/api/joystick/init_arm", methods=post_methods)
    @require_available
    def joystick_init_arm(joystick_controller):
        """POST/OPTIONS /api/joystick/init_arm：初始化单/双臂与手侧绑定。"""
        try:
            body = request.get_json(silent=True) or {}
            mode = body.get("mode", "single")
            master_ip = body.get("master_ip") or body.get("ip", "10.42.0.101")
            slave_ip = body.get("slave_ip")
            master_side = body.get("master_side", "right")
            accepted, success = run_operation(
                body,
                lambda: joystick_controller.init_arm(
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
                joystick_controller.get_last_error()
                or f"机械臂初始化失败 (IP: {master_ip})，请检查 IP 地址和网络连接"
            )
        except Exception as exc:
            logging.exception("[JoystickAPI] Arm initialization failed")
            return _failed(f"初始化机械臂失败: {exc}")

    def register_value_route(path, endpoint, validator, setter, invalid_message, exception_message):
        """注册 /api/joystick/* 单值设置路由（校验 body.value 后调用 setter）。"""
        def handler(joystick_controller):
            """校验 body.value 后调用 setter 并返回统一响应。"""
            try:
                body = request.get_json(silent=True) or {}
                value = body.get("value")
                if not validator(value):
                    return _failed(invalid_message)
                accepted, success = run_operation(
                    body,
                    lambda: setter(joystick_controller, value),
                )
                if not accepted:
                    return _failed("桌面请求已失效")
                return _json_response("设置成功") if success else _failed("设置失败")
            except Exception:
                logging.exception("[JoystickAPI] %s failed", endpoint)
                return _failed(exception_message)

        handler.__name__ = endpoint
        app.add_url_rule(
            path,
            endpoint=endpoint,
            view_func=require_available(handler),
            methods=post_methods,
        )

    register_value_route(
        # POST/OPTIONS /api/joystick/calibration：设置历史 0-3 标定档位。
        "/api/joystick/calibration",
        "joystick_calibration",
        lambda value: isinstance(value, int) and not isinstance(value, bool)
        and 0 <= value <= 3,
        lambda joystick_controller, value: joystick_controller.set_calibration(value),
        "标定值必须在 0-3 之间",
        "设置标定失败",
    )
    register_value_route(
        # POST/OPTIONS /api/joystick/yaw：设置 -180..180 的整数 yaw 角度。
        "/api/joystick/yaw",
        "joystick_yaw",
        lambda value: isinstance(value, (int, float)) and not isinstance(value, bool)
        and -180 <= value <= 180 and int(value) == value,
        lambda joystick_controller, value: joystick_controller.set_yaw(value),
        "标定角度必须为 -180 到 180 的整数",
        "设置标定失败",
    )
    register_value_route(
        # POST/OPTIONS /api/joystick/scale：设置 0.1-2.0 灵敏度。
        "/api/joystick/scale",
        "joystick_scale",
        lambda value: isinstance(value, (int, float)) and not isinstance(value, bool)
        and 0.1 <= value <= 2.0,
        lambda joystick_controller, value: joystick_controller.set_scale(value),
        "灵敏度值必须在 0.1-2.0 之间",
        "设置灵敏度失败",
    )
    register_value_route(
        # POST/OPTIONS /api/joystick/view_side：设置查看侧 left/right。
        "/api/joystick/view_side",
        "joystick_view_side",
        lambda value: value in ("left", "right"),
        lambda joystick_controller, value: joystick_controller.set_view_side(value),
        "查看侧必须为 left 或 right",
        "设置查看侧失败",
    )

    @app.route("/api/joystick/session_config", methods=post_methods)
    @require_available
    def joystick_session_config(joystick_controller):
        """POST/OPTIONS /api/joystick/session_config：更新 mode/master_side/view_side/slave_ip。"""
        try:
            body = request.get_json(silent=True) or {}
            accepted, success = run_operation(
                body,
                lambda: joystick_controller.set_session_config(
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
            logging.exception("[JoystickAPI] Session config failed")
            return _failed("设置会话配置失败")

    @app.route("/api/joystick/status", methods=["GET"])
    def joystick_status():
        """GET /api/joystick/status：返回可用性与会话状态（不可用也返回 200）。"""
        try:
            if not dependencies_ready():
                return _json_response(
                    body={"data": {"available": False, "status": "dependencies_not_available"}}
                )
            joystick_controller = current_controller()
            if not getattr(joystick_controller, "available", False):
                return _json_response(
                    body={"data": {"available": False, "status": "not_available"}}
                )
            return _json_response(body={"data": joystick_controller.get_status()})
        except Exception:
            logging.exception("[JoystickAPI] Status failed")
            return _failed("获取状态失败")

    if include_maintenance_routes:
        @app.route("/api/joystick/confirm_init_position", methods=post_methods)
        @require_available
        def joystick_confirm_init_position(joystick_controller):
            """POST/OPTIONS /api/joystick/confirm_init_position：确认并记录零点位置。"""
            try:
                body = request.get_json(silent=True) or {}
                ip = body.get("ip", "10.42.0.101")
                if joystick_controller.confirm_init_position(ip):
                    return _json_response("确认初始化位置成功")
                return _failed("确认初始化位置失败")
            except Exception:
                logging.exception("[JoystickAPI] Confirm init position failed")
                return _failed("确认初始化位置失败")

        @app.route("/api/joystick/get_init_position", methods=get_methods)
        @require_available
        def joystick_get_init_position(joystick_controller):
            """GET/OPTIONS /api/joystick/get_init_position：读取已记录的零点。"""
            try:
                position = joystick_controller.get_init_position()
                if position.get("error"):
                    return _failed(position.get("error", "获取初始化位置失败"))
                return _json_response(body={"data": position})
            except Exception:
                logging.exception("[JoystickAPI] Get init position failed")
                return _failed("获取初始化位置失败")

        @app.route("/api/joystick/get_current_joints", methods=get_methods)
        @require_available
        def joystick_get_current_joints(joystick_controller):
            """GET/OPTIONS /api/joystick/get_current_joints：读取指定 IP 的当前关节值。"""
            try:
                ip = request.args.get("ip", "10.42.0.101")
                joints = joystick_controller.get_current_joints(ip)
                if joints.get("error"):
                    return _failed(joints.get("error", "获取当前关节值失败"))
                if joints.get("current_joints") is not None:
                    return _json_response(
                        body={"data": {"joints": joints.get("current_joints")}}
                    )
                return _failed("获取当前关节值失败：未知错误")
            except Exception as exc:
                logging.exception("[JoystickAPI] Get current joints failed")
                return _failed(f"获取当前关节值失败: {exc}")

    @app.route("/api/joystick/events", methods=get_methods)
    @require_available
    def joystick_events(joystick_controller):
        """GET/OPTIONS /api/joystick/events：SSE 推送观察帧、心跳与断连终帧。"""
        try:
            connection_id = str(uuid.uuid4())[:8]

            def event_stream():
                """SSE 生成器：序列号变化即推送；空闲超时发心跳；终止帧后结束流。"""
                last_sequence = -1
                last_heartbeat = time.time()
                try:
                    while True:
                        sequence_reader = getattr(joystick_controller, "get_observation_sequence", None)
                        if (sequence_reader is not None and sequence_reader() == last_sequence
                                and time.time() - last_heartbeat <= 3):
                            time.sleep(0.05)
                            continue
                        snapshot = joystick_controller.get_observation_snapshot()
                        sequence = snapshot.sequence
                        emit_disconnect = bool(
                            snapshot.emit_disconnect_frame
                            and sequence != last_sequence
                        )
                        if snapshot.stream_terminal and not emit_disconnect:
                            return
                        if sequence != last_sequence:
                            payload = project_sse_payload(
                                snapshot,
                                include_terminal_fields=not include_maintenance_routes,
                            )
                            payload["timestamp"] = time.time()
                            yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                            last_sequence = sequence
                            last_heartbeat = time.time()
                            if emit_disconnect:
                                return
                        elif time.time() - last_heartbeat > 3:
                            payload = project_sse_payload(
                                snapshot,
                                heartbeat=True,
                                include_terminal_fields=not include_maintenance_routes,
                            )
                            payload["timestamp"] = time.time()
                            yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                            last_heartbeat = time.time()
                        time.sleep(0.05)
                except GeneratorExit:
                    logging.info("[JoystickSSE-%s] Connection closed", connection_id)
                except Exception:
                    logging.exception("[JoystickSSE-%s] Stream failed", connection_id)

            response = Response(
                stream_with_context(event_stream()),
                mimetype="text/event-stream",
            )
            response.headers["Cache-Control"] = "no-cache"
            response.headers["Connection"] = "keep-alive"
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["X-Accel-Buffering"] = "no"
            return response
        except Exception:
            logging.exception("[JoystickAPI] Create stream failed")
            return _failed("创建数据流失败")
