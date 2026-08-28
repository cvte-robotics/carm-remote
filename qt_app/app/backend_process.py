from __future__ import annotations

from collections import deque
import logging
import sys
from pathlib import Path
import uuid

from PyQt6.QtCore import QObject, QProcess, QProcessEnvironment, pyqtSignal


class BackendProcess(QObject):
    failed = pyqtSignal(str)
    exited = pyqtSignal(int, object)
    diagnostic = pyqtSignal(str)

    def __init__(self, root: Path, log_directory: Path, parent=None):
        super().__init__(parent)
        self.root = root
        self.log_directory = log_directory
        self.backend_log_file = log_directory / "desktop-backend.log"
        self._instance_token = uuid.uuid4().hex
        self._diagnostics: deque[str] = deque(maxlen=40)
        self._stdout_buffer = ""
        self._stderr_buffer = ""
        self.process = QProcess(self)
        self.process.setWorkingDirectory(str(root))

        environment = QProcessEnvironment.systemEnvironment()
        environment.insert("CARM_DESKTOP_LOG_DIR", str(log_directory))
        environment.insert("CARM_DESKTOP_INSTANCE_TOKEN", self._instance_token)
        self.process.setProcessEnvironment(environment)

        self.process.readyReadStandardError.connect(self._read_error)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.errorOccurred.connect(self._error)
        self.process.finished.connect(self._finished)

    def start(self) -> None:
        logging.getLogger("desktop.backend_process").info(
            "Starting backend: %s -m server.desktop.app (cwd=%s)",
            sys.executable,
            self.root,
        )
        self.process.start(sys.executable, ["-m", "server.desktop.app"])

    def process_id(self) -> int:
        return int(self.process.processId())

    def instance_token(self) -> str:
        return self._instance_token

    def is_running(self) -> bool:
        return self.process.state() != QProcess.ProcessState.NotRunning

    def last_diagnostic(self) -> str:
        try:
            with self.backend_log_file.open("rb") as log_file:
                log_file.seek(0, 2)
                size = log_file.tell()
                log_file.seek(max(0, size - 16 * 1024))
                lines = log_file.read().decode("utf-8", errors="replace").splitlines()
            if lines:
                return lines[-1].strip()
        except OSError:
            pass
        return self._diagnostics[-1] if self._diagnostics else ""

    def _read_output(self) -> None:
        self._record_output(
            bytes(self.process.readAllStandardOutput()),
            logging.INFO,
            "_stdout_buffer",
        )

    def _read_error(self) -> None:
        self._record_output(
            bytes(self.process.readAllStandardError()),
            logging.WARNING,
            "_stderr_buffer",
        )

    def _record_output(self, raw: bytes, level: int, buffer_name: str) -> None:
        text = getattr(self, buffer_name) + raw.decode("utf-8", errors="replace")
        lines = text.split("\n")
        setattr(self, buffer_name, lines.pop())
        for line in lines:
            self._record_line(line, level)

    def _record_line(self, line: str, level: int) -> None:
        line = line.strip()
        if not line:
            return
        self._diagnostics.append(line)
        logging.getLogger("desktop.backend_output").log(level, line)
        self.diagnostic.emit(line)

    def _finished(self, exit_code: int, exit_status) -> None:
        self._read_output()
        self._read_error()
        self._record_line(self._stdout_buffer, logging.INFO)
        self._record_line(self._stderr_buffer, logging.WARNING)
        self._stdout_buffer = ""
        self._stderr_buffer = ""
        logging.getLogger("desktop.backend_process").info(
            "Backend exited: code=%s status=%s",
            exit_code,
            exit_status,
        )
        self.exited.emit(exit_code, exit_status)

    def _error(self, error) -> None:
        message = f"后端进程错误: {error}"
        logging.getLogger("desktop.backend_process").error(message)
        self.failed.emit(message)
