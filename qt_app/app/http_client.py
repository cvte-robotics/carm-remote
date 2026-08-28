from __future__ import annotations

import json
from typing import Any

from PyQt6.QtCore import QObject, QUrl, pyqtSignal
from PyQt6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply


class HttpClient(QObject):
    response = pyqtSignal(str, int, dict, str)

    def __init__(self, base_url: str = "http://127.0.0.1:1999", parent=None):
        super().__init__(parent)
        self.base_url = base_url.rstrip("/")
        self.manager = QNetworkAccessManager(self)
        self._replies: set[QNetworkReply] = set()
        self._next_request_id = 1

    def _allocate_request_id(self) -> int:
        request_id = self._next_request_id
        self._next_request_id += 1
        return request_id

    def request(self, method: str, path: str, payload: dict[str, Any] | None = None) -> int:
        request_id = self._allocate_request_id()
        request = QNetworkRequest(QUrl(self.base_url + path))
        request.setHeader(QNetworkRequest.KnownHeaders.ContentTypeHeader, "application/json")
        body = json.dumps(payload or {}).encode("utf-8")
        reply = self.manager.sendCustomRequest(request, method.encode("ascii"), body)
        self._replies.add(reply)
        reply.finished.connect(lambda: self._finished(path, request_id, reply))
        return request_id

    def get(self, path: str) -> int:
        request_id = self._allocate_request_id()
        request = QNetworkRequest(QUrl(self.base_url + path))
        reply = self.manager.get(request)
        self._replies.add(reply)
        reply.finished.connect(lambda: self._finished(path, request_id, reply))
        return request_id

    def post(self, path: str, payload: dict[str, Any] | None = None) -> int:
        return self.request("POST", path, payload)

    def _finished(self, path: str, request_id: int, reply: QNetworkReply) -> None:
        self._replies.discard(reply)
        error = ""
        if reply.error() != QNetworkReply.NetworkError.NoError:
            error = reply.errorString()
        try:
            data = json.loads(bytes(reply.readAll()).decode("utf-8"))
            if not isinstance(data, dict):
                data = {}
        except (UnicodeDecodeError, json.JSONDecodeError):
            data = {}
            error = error or "后端返回了无效 JSON"
        if not error and data.get("code") not in (None, 1):
            error = str(data.get("msg") or "后端请求失败")
        self.response.emit(path, request_id, data, error)
        reply.deleteLater()
