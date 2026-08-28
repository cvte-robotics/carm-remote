from __future__ import annotations

import json

from PyQt6.QtCore import QObject, QTimer, QUrl, pyqtSignal
from PyQt6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply


class SseClient(QObject):
    message_received = pyqtSignal(dict)
    disconnected = pyqtSignal(str)

    def __init__(self, base_url: str = "http://127.0.0.1:1999", parent=None):
        super().__init__(parent)
        self.url = QUrl(base_url.rstrip("/") + "/api/joystick/events")
        self.manager = QNetworkAccessManager(self)
        self.reply: QNetworkReply | None = None
        self.buffer = b""
        self.attempt = 0
        self.max_attempts = 5
        self.stopped = True
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.open_stream)

    def open_stream(self) -> None:
        self.stopped = False
        self.timer.stop()
        if self.reply is not None:
            old_reply = self.reply
            self.reply = None
            old_reply.abort()
            old_reply.deleteLater()
        request = QNetworkRequest(self.url)
        request.setRawHeader(b"Accept", b"text/event-stream")
        self.buffer = b""
        reply = self.manager.get(request)
        self.reply = reply
        reply.readyRead.connect(lambda current=reply: self._read(current))
        reply.finished.connect(lambda current=reply: self._finished(current))

    def stop(self) -> None:
        self.stopped = True
        self.timer.stop()
        self.attempt = 0
        if self.reply is not None:
            reply = self.reply
            self.reply = None
            reply.abort()
            reply.deleteLater()

    def _read(self, reply: QNetworkReply) -> None:
        if reply is not self.reply:
            return
        self.buffer += bytes(reply.readAll())
        while b"\n\n" in self.buffer or b"\r\n\r\n" in self.buffer:
            separator = b"\r\n\r\n" if b"\r\n\r\n" in self.buffer and (b"\n\n" not in self.buffer or self.buffer.index(b"\r\n\r\n") < self.buffer.index(b"\n\n")) else b"\n\n"
            raw, self.buffer = self.buffer.split(separator, 1)
            data_lines = []
            for line in raw.replace(b"\r\n", b"\n").split(b"\n"):
                if line.startswith(b"data:"):
                    data_lines.append(line[5:].lstrip())
            if data_lines:
                try:
                    value = json.loads(b"\n".join(data_lines).decode("utf-8"))
                    if isinstance(value, dict):
                        self.attempt = 0
                        self.message_received.emit(value)
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue

    def _finished(self, reply: QNetworkReply) -> None:
        if reply is not self.reply:
            reply.deleteLater()
            return
        self.reply = None
        error = reply.errorString() if reply.error() else "连接已断开"
        reply.deleteLater()
        if self.stopped:
            return
        self.disconnected.emit(error)
        if self.attempt < self.max_attempts:
            self.attempt += 1
            self.timer.start(min(5000, 250 * (2 ** (self.attempt - 1))))
