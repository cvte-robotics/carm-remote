"""Joystick-only local Flask service owned by the PyQt desktop process."""

from __future__ import annotations

import argparse
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import sys
import tempfile
import threading

from flask import Flask
from werkzeug.serving import make_server


SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))
os.environ["CARM_JOYSTICK_PROVIDERS"] = "quest"


class _LoggerStream:
    def __init__(self, logger: logging.Logger, level: int) -> None:
        self.logger = logger
        self.level = level
        self.encoding = "utf-8"
        self._buffer = ""
        self._lock = threading.Lock()

    def write(self, message: str) -> int:
        if not message:
            return 0
        with self._lock:
            self._buffer += str(message)
            while "\n" in self._buffer:
                line, self._buffer = self._buffer.split("\n", 1)
                if line.strip():
                    self.logger.log(self.level, line.rstrip())
        return len(message)

    def flush(self) -> None:
        with self._lock:
            if self._buffer.strip():
                self.logger.log(self.level, self._buffer.rstrip())
            self._buffer = ""

    def isatty(self) -> bool:
        return False


def _log_directory_candidates(configured_dir: str | None):
    if configured_dir:
        yield Path(configured_dir)
    yield Path.cwd() / "logs"
    yield Path(tempfile.gettempdir()) / "carm-quest-desktop" / "logs"


def _configure_backend_logging() -> Path:
    file_handler = None
    log_path = None
    for log_directory in _log_directory_candidates(
        os.environ.get("CARM_DESKTOP_LOG_DIR")
    ):
        try:
            log_directory.mkdir(parents=True, exist_ok=True)
            candidate_path = log_directory / "desktop-backend.log"
            candidate_handler = RotatingFileHandler(
                candidate_path,
                maxBytes=5 * 1024 * 1024,
                backupCount=3,
                encoding="utf-8",
            )
            file_handler = candidate_handler
            log_path = candidate_path
            break
        except OSError:
            continue
    if file_handler is None or log_path is None:
        raise RuntimeError("No writable desktop backend log file")
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s %(name)s %(message)s"
    )
    file_handler.setFormatter(formatter)
    logging.basicConfig(
        level=logging.INFO,
        handlers=[file_handler],
        force=True,
    )
    sys.stdout = _LoggerStream(logging.getLogger("desktop.stdout"), logging.INFO)
    sys.stderr = _LoggerStream(logging.getLogger("desktop.stderr"), logging.ERROR)
    logging.getLogger("desktop.backend").info(
        "Backend logging initialized: %s", log_path
    )
    return log_path


BACKEND_LOG_PATH = _configure_backend_logging()

from api.joystick_routes import _json_response, register_joystick_routes
from basic.joystick.end_effectors import (
    EndEffectorBase,
    resolve_end_effector_factory,
)
from desktop.runtime import DesktopRuntime


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 1999


def create_app(runtime: DesktopRuntime | None = None):
    runtime = runtime or DesktopRuntime()
    app = Flask("carm_quest_desktop")

    @app.after_request
    def add_cors_headers(response):
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
        return response

    register_joystick_routes(
        app,
        runtime.controller,
        scan_handler=runtime.scan,
        operation_runner=runtime.run_operation,
        include_maintenance_routes=False,
        allow_get_disconnect=False,
    )

    @app.get("/api/desktop/health")
    def desktop_health():
        return _json_response(
            body={
                "data": {
                    "ready": True,
                    "pid": os.getpid(),
                    "instance_token": os.environ.get(
                        "CARM_DESKTOP_INSTANCE_TOKEN", ""
                    ),
                    "log_file": str(BACKEND_LOG_PATH),
                    **runtime.lifecycle_status(),
                }
            }
        )

    @app.post("/api/desktop/heartbeat")
    def desktop_heartbeat():
        runtime.heartbeat()
        return _json_response("heartbeat")

    @app.post("/api/desktop/shutdown")
    def desktop_shutdown():
        status = runtime.request_shutdown("desktop-requested-shutdown")
        return _json_response("正在清理", body={"data": status})

    return app, runtime


def main(argv=None) -> int:
    EndEffectorBase.load_all()
    parser = argparse.ArgumentParser(description="CARM Quest desktop backend")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--end-effector",
        default="carm",
        help="包内末端实现名称（按名称动态导入）",
    )
    args = parser.parse_args(argv)

    end_effector_factory = resolve_end_effector_factory(args.end_effector)
    app, runtime = create_app(
        DesktopRuntime(end_effector_factory=end_effector_factory)
    )
    try:
        server = make_server(DEFAULT_HOST, args.port, app, threaded=True)
    except (OSError, SystemExit) as exc:
        logging.error("Desktop backend cannot bind %s:%s: %s", DEFAULT_HOST, args.port, exc)
        return 2

    runtime.attach_server(server)
    runtime.start()
    logging.info(
        "Desktop backend listening on http://%s:%s end_effector=%s",
        DEFAULT_HOST,
        args.port,
        args.end_effector,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Desktop backend interrupted")
    finally:
        cleanup_complete = runtime.shutdown("desktop-server-exit")

        if cleanup_complete:
            server.server_close()
        else:
            logging.error("Desktop backend exited with incomplete cleanup")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
