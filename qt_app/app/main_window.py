from __future__ import annotations

import html
import json
from typing import Any

from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import QCloseEvent, QDesktopServices, QFontDatabase
from PyQt6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QPushButton,
    QSlider,
    QStyle,
    QTextEdit,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from .models import Device, ShutdownState, Snapshot, UiState
from .view_model import ViewModel


PROFILE_NAMES = {"a3_single": "六轴单臂", "a3_dual": "六轴双臂", "d3_dual": "人形双臂"}
# 状态栏只在输入设备已连接时显示；离线（stale）项用灰色表示，不加文字。
DEVICE_STATUS_STATES = {"Connected", "Initializing", "Teleoperating"}
DEVICE_STATUS_COLOR = "#15803d"
DEVICE_STATUS_STALE_COLOR = "#9ca3af"
# 状态指示点颜色只反映 UI 状态名，不参与任何启用/禁用判断。
STATE_TONES = {
    "Scanning": "busy", "Connecting": "busy", "Initializing": "busy", "Stopping": "busy",
    "BackendStarting": "busy", "ShuttingDown": "busy",
    "Connected": "info", "Teleoperating": "success",
    "BackendFailed": "error", "BackendStopped": "error",
}

STYLE_SHEET = """
QMainWindow, QWidget#root { background: #f3f5f8; }
QWidget { color: #1f2937; font-size: 14px; }
QLabel { background: transparent; }
QToolTip { color: #1f2937; background: #ffffff; border: 1px solid #d6dbe3; padding: 4px 6px; }

QGroupBox {
    background: #ffffff; border: 1px solid #e2e6ec; border-radius: 10px;
    margin-top: 10px; padding: 14px 12px 10px 12px; font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 12px; padding: 0 6px; color: #334155;
}
QGroupBox QLabel { font-weight: normal; }
QLabel#fieldLabel { color: #475569; }
QLabel#hint, QLabel#mutedText { color: #64748b; font-size: 13px; }
QLabel#valueBadge {
    background: #eff6ff; color: #1d4ed8; border-radius: 6px;
    padding: 3px 0; font-weight: 600;
}

QLineEdit, QComboBox {
    background: #ffffff; border: 1px solid #cfd6df; border-radius: 6px;
    padding: 4px 10px; min-height: 22px;
    selection-background-color: #2563eb;
}
QLineEdit:hover, QComboBox:hover { border-color: #94a3b8; }
QLineEdit:focus, QComboBox:focus, QComboBox:on { border-color: #2563eb; }
QLineEdit:disabled, QComboBox:disabled { background: #f1f5f9; color: #94a3b8; border-color: #e2e8f0; }
QComboBox::drop-down { border: none; width: 28px; }
QComboBox::down-arrow {
    image: url(:/qt-project.org/styles/commonstyle/images/arrow-down-16.png);
    width: 12px; height: 12px;
}
QComboBox::down-arrow:disabled { image: none; }
QComboBox QAbstractItemView {
    background: #ffffff; border: 1px solid #d6dbe3; outline: none; padding: 4px;
    selection-background-color: #dbeafe; selection-color: #1e3a8a;
}

QPushButton {
    background: #ffffff; border: 1px solid #cfd6df; border-radius: 8px;
    padding: 0 20px; font-weight: 600;
}
QPushButton:hover { background: #f8fafc; border-color: #94a3b8; }
QPushButton[variant="primary"] { background: #2563eb; color: #ffffff; border: none; }
QPushButton[variant="primary"]:hover { background: #1d4ed8; }
QPushButton[variant="primary"]:pressed { background: #1e40af; }
QPushButton[variant="danger"] { background: #dc2626; color: #ffffff; border: none; }
QPushButton[variant="danger"]:hover { background: #b91c1c; }
QPushButton[variant="danger"]:pressed { background: #991b1b; }
QPushButton:disabled, QPushButton[variant="primary"]:disabled, QPushButton[variant="danger"]:disabled {
    background: #e5e9ef; color: #9aa4b2; border: none;
}

QToolButton { background: transparent; border: none; border-radius: 6px; padding: 4px; }
QToolButton:hover { background: #e2e8f0; }

QListWidget {
    background: #ffffff; border: 1px solid #e2e6ec; border-radius: 8px;
    padding: 4px; outline: none;
}
QListWidget::item { padding: 8px 10px; border-radius: 6px; color: #1f2937; }
QListWidget::item:hover { background: #f1f5f9; }
QListWidget::item:selected { background: #dbeafe; color: #1e3a8a; }
QListWidget:disabled { background: #f8fafc; }
QListWidget::item:disabled { color: #94a3b8; }

QSlider { min-height: 22px; }
QSlider::groove:horizontal { height: 6px; background: #e2e8f0; border-radius: 3px; }
QSlider::sub-page:horizontal { background: #3b82f6; border-radius: 3px; }
QSlider::handle:horizontal {
    background: #ffffff; border: 2px solid #2563eb;
    width: 14px; height: 14px; margin: -6px 0; border-radius: 9px;
}
QSlider::handle:horizontal:hover { background: #eff6ff; }
QSlider::sub-page:horizontal:disabled { background: #cbd5e1; }
QSlider::handle:horizontal:disabled { border-color: #cbd5e1; background: #f8fafc; }

QLabel#sectionTitle { color: #334155; font-weight: 600; padding-left: 4px; }
QTextEdit {
    background: #ffffff; border: 1px solid #e2e6ec; border-radius: 8px;
    padding: 6px; color: #0f172a;
}

QFrame#statusPanel { background: #ffffff; border: 1px solid #e2e6ec; border-radius: 8px; }
QLabel#statusText { font-weight: 600; color: #1f2937; }
QLabel#statusDot { border-radius: 5px; background: #94a3b8; }
QLabel#statusDot[tone="busy"] { background: #f59e0b; }
QLabel#statusDot[tone="info"] { background: #2563eb; }
QLabel#statusDot[tone="success"] { background: #16a34a; }
QLabel#statusDot[tone="error"] { background: #dc2626; }

QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: #cbd5e1; border-radius: 3px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: #94a3b8; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 2px; }
QScrollBar::handle:horizontal { background: #cbd5e1; border-radius: 3px; min-width: 24px; }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
"""


def _field_label(text: str) -> QLabel:
    label = QLabel(text)
    label.setObjectName("fieldLabel")
    return label


def _set_plain_text_keep_scroll(editor: QTextEdit, text: str) -> None:
    """只在内容变化时刷新，并保留滚动位置，避免高频 SSE 帧导致闪烁和跳回顶部。"""
    if editor.toPlainText() == text:
        return
    vertical = editor.verticalScrollBar().value()
    horizontal = editor.horizontalScrollBar().value()
    editor.setPlainText(text)
    editor.verticalScrollBar().setValue(vertical)
    editor.horizontalScrollBar().setValue(horizontal)


def _localized(text: Any, fallback: str) -> str:
    """输入源按语言给出的文本：Qt 界面为中文，取 zh-CN，依次回退到 en-US、任一语言。"""
    if not isinstance(text, dict) or not text:
        return fallback
    return str(text.get("zh-CN") or text.get("en-US") or next(iter(text.values())))


def format_device_status(items: Any) -> str:
    """把 SSE status.device_status 渲染为状态栏富文本；无内容时返回空串。"""
    parts = []
    for item in items if isinstance(items, list) else []:
        if not isinstance(item, dict):
            continue
        label = _localized(item.get("label"), str(item.get("id", "")))
        value = item.get("value")
        if value is None:
            text = "未知"
        elif isinstance(value, bool):
            text = "是" if value else "否"
        elif isinstance(value, dict):
            text = _localized(value, "未知")
        else:
            text = str(value)
        unit = item.get("unit")
        if value is not None and unit:
            text += unit if unit == "%" else f" {unit}"
        color = DEVICE_STATUS_STALE_COLOR if item.get("stale") else DEVICE_STATUS_COLOR
        parts.append(f'<span style="color:{color}">{html.escape(label)}: {html.escape(text)}</span>')
    return "&nbsp;&nbsp;&nbsp;".join(parts)


class MainWindow(QMainWindow):
    def __init__(self, parent=None, *, end_effector: str = "carm"):
        super().__init__(parent)
        self.setWindowTitle("CARM 手柄遥操")
        self.resize(1000, 800)
        self.vm = ViewModel(end_effector=end_effector, parent=self)
        self._build_ui()
        self.vm.state_changed.connect(self.render_state)
        self.vm.snapshot_changed.connect(self.render_snapshot)
        self.vm.toast.connect(self.show_message)
        self.vm.closed.connect(self.close)
        self.render_state(self.vm.state)
        self.vm.start()

    def _build_ui(self) -> None:
        self.setStyleSheet(STYLE_SHEET)
        root = QWidget(self)
        root.setObjectName("root")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(16, 10, 16, 12)
        layout.setSpacing(8)

        connection = QGroupBox("设备连接")
        connection_layout = QVBoxLayout(connection)
        connection_layout.setSpacing(8)
        ip_row = QHBoxLayout()
        ip_row.setSpacing(10)
        ip_row.addWidget(_field_label("当前臂 IP:"))
        self.current_ip = QLineEdit("10.42.0.101")
        self.current_ip.setMinimumHeight(40)
        self.current_ip.textChanged.connect(self.invalidate_scan)
        ip_row.addWidget(self.current_ip, 1)
        connection_layout.addLayout(ip_row)

        actions = QHBoxLayout()
        actions.setSpacing(10)
        self.scan_button = QPushButton("扫描设备")
        self.scan_button.clicked.connect(lambda: self.vm.scan(self.current_ip.text().strip()))
        self.init_button = QPushButton("初始化机械臂")
        self.init_button.clicked.connect(self.vm.initialize)
        self.stop_button = QPushButton("停止手柄遥操")
        self.stop_button.clicked.connect(self.vm.stop)
        for button, variant in (
            (self.scan_button, "primary"),
            (self.init_button, "primary"),
            (self.stop_button, "danger"),
        ):
            button.setProperty("variant", variant)
            button.setMinimumHeight(40)
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            actions.addWidget(button)
        connection_layout.addLayout(actions)

        self.devices = QListWidget()
        self.devices.setToolTip("扫描后双击设备进行连接")
        self.devices.setMinimumHeight(88)
        self.devices.itemDoubleClicked.connect(self.device_double_clicked)
        connection_layout.addWidget(self.devices)
        layout.addWidget(connection, 1)

        status_panel = QFrame()
        status_panel.setObjectName("statusPanel")
        status_row = QHBoxLayout(status_panel)
        status_row.setContentsMargins(12, 6, 12, 6)
        status_row.setSpacing(8)
        self.status_dot = QLabel()
        self.status_dot.setObjectName("statusDot")
        self.status_dot.setFixedSize(10, 10)
        status_row.addWidget(self.status_dot)
        self.status = QLabel("正在启动后端...")
        self.status.setObjectName("statusText")
        status_row.addWidget(self.status, 1)
        self._device_status_items: Any = []
        self.device_status = QLabel()
        self.device_status.setTextFormat(Qt.TextFormat.RichText)
        self.device_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        status_row.addWidget(self.device_status)
        layout.addWidget(status_panel)

        log_row = QHBoxLayout()
        log_row.setContentsMargins(4, 0, 0, 0)
        log_title = QLabel("日志:")
        log_title.setObjectName("mutedText")
        log_row.addWidget(log_title)
        self.log_location = QLabel(str(self.vm.log_directory))
        self.log_location.setObjectName("mutedText")
        self.log_location.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        self.log_location.setWordWrap(True)
        log_row.addWidget(self.log_location, 1)
        self.open_logs_button = QToolButton()
        self.open_logs_button.setIcon(
            self.style().standardIcon(QStyle.StandardPixmap.SP_DirOpenIcon)
        )
        self.open_logs_button.setToolTip("打开日志目录")
        self.open_logs_button.setCursor(Qt.CursorShape.PointingHandCursor)
        self.open_logs_button.clicked.connect(self.open_log_directory)
        log_row.addWidget(self.open_logs_button)
        layout.addLayout(log_row)

        config = QGroupBox("遥操配置")
        form = QFormLayout(config)
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(8)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        self._config_form = form
        self.profile = QComboBox()
        self.profile.currentIndexChanged.connect(self.profile_changed)
        form.addRow(_field_label("遥操类型:"), self.profile)
        self.other_ip = QLineEdit("10.42.0.102")
        form.addRow(_field_label("另一臂 IP:"), self.other_ip)
        self.side = QComboBox()
        self.side.addItem("作为左臂", "left")
        self.side.addItem("作为右臂", "right")
        form.addRow(_field_label("当前机械臂:"), self.side)
        self.view_side = QComboBox()
        self.view_side.addItem("左手柄", "left")
        self.view_side.addItem("右手柄", "right")
        self.view_side.currentIndexChanged.connect(lambda: self.vm.set_view_side(self.view_side.currentData()))
        form.addRow(_field_label("手柄数据:"), self.view_side)
        scale_row = QHBoxLayout()
        scale_row.setSpacing(12)
        self.scale = QSlider(Qt.Orientation.Horizontal)
        self.scale.setRange(10, 200)
        self.scale.setValue(40)
        self.scale.valueChanged.connect(self.scale_changed)
        self.scale_value = QLabel("0.40")
        self.scale_value.setObjectName("valueBadge")
        self.scale_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.scale_value.setFixedWidth(56)
        scale_row.addWidget(self.scale, 2)
        scale_row.addWidget(self.scale_value)
        form.addRow(_field_label("灵敏度:"), scale_row)
        layout.addWidget(config)

        calibration = QGroupBox("标定")
        cal_layout = QVBoxLayout(calibration)
        cal_layout.setSpacing(6)
        instruction = QLabel("Yaw(°):请根据手柄(头盔)与机械臂的X轴夹角调整校准角度")
        instruction.setObjectName("hint")
        instruction.setWordWrap(True)
        cal_layout.addWidget(instruction)
        cal_row = QHBoxLayout()
        cal_row.setSpacing(12)
        self.calibration_slider = QSlider(Qt.Orientation.Horizontal)
        self.calibration_slider.setRange(-180, 180)
        self.calibration_slider.setSingleStep(1)
        self.calibration_slider.setTracking(False)
        self.calibration_slider.setValue(-90)
        self.calibration_slider.valueChanged.connect(self.vm.calibration)
        self.calibration_value = QLabel("-90°")
        self.calibration_value.setObjectName("valueBadge")
        self.calibration_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.calibration_value.setFixedWidth(56)
        self.calibration_slider.sliderMoved.connect(lambda value: self.calibration_value.setText(f"{value}°"))
        self.calibration_slider.valueChanged.connect(lambda value: self.calibration_value.setText(f"{value}°"))
        cal_row.addWidget(self.calibration_slider, 1)
        cal_row.addWidget(self.calibration_value)
        cal_layout.addLayout(cal_row)
        layout.addWidget(calibration)

        data = QHBoxLayout()
        data.setSpacing(10)
        mono = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
        mono.setPointSizeF(max(mono.pointSizeF(), 10.0))
        self.pose = QTextEdit(); self.pose.setReadOnly(True)
        self.buttons = QTextEdit(); self.buttons.setReadOnly(True)
        for title, editor in (("位置信息:", self.pose), ("按钮信息:", self.buttons)):
            editor.setFont(mono)
            editor.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
            editor.setMinimumHeight(72)
            caption = QLabel(title)
            caption.setObjectName("sectionTitle")
            box = QVBoxLayout()
            box.setSpacing(4)
            box.addWidget(caption)
            box.addWidget(editor)
            data.addLayout(box, 1)
        layout.addLayout(data, 2)
        form.setRowVisible(self.other_ip, False)
        self._controls = [self.scan_button, self.init_button, self.stop_button, self.profile, self.other_ip, self.side, self.view_side, self.scale, self.calibration_slider]

    def invalidate_scan(self) -> None:
        state = self.vm.state
        if state.devices or state.allowed_profiles or state.probe is not None:
            self.vm.clear_scan_state()
            self.devices.clear()
            self.profile.clear()
            self.vm._set("Idle", "当前臂 IP 已变化，请重新扫描")

    def render_state(self, state: UiState) -> None:
        if [self.profile.itemData(i) for i in range(self.profile.count())] != state.allowed_profiles:
            self.render_profiles(state.allowed_profiles)
        if self.devices.count() != len(state.devices):
            self.render_devices(state.devices)
        self.status.setText(state.message)
        name = state.name
        self.scan_button.setEnabled(name in {"Idle", "ScanReady"})
        self.current_ip.setEnabled(name in {"Idle", "ScanReady"})
        self.init_button.setEnabled(name == "Connected")
        self.stop_button.setEnabled(name in {"Connecting", "Connected", "Initializing", "Teleoperating"})
        can_config = name == "ScanReady"
        self.profile.setEnabled(can_config)
        self.other_ip.setEnabled(can_config)
        self.side.setEnabled(can_config)
        active_adjustment = name in {"Connected", "Teleoperating"}
        self.view_side.setEnabled(active_adjustment)
        self.scale.setEnabled(active_adjustment)
        self.calibration_slider.setEnabled(active_adjustment)
        self.devices.setEnabled(name == "ScanReady")
        self.render_status_tone(name)
        self.render_device_status()

    def render_status_tone(self, name: str) -> None:
        tone = STATE_TONES.get(name, "idle")
        if self.status_dot.property("tone") != tone:
            self.status_dot.setProperty("tone", tone)
            self.status_dot.style().unpolish(self.status_dot)
            self.status_dot.style().polish(self.status_dot)

    def render_snapshot(self, snapshot: Snapshot) -> None:
        angle = snapshot.status.get("yaw")
        if isinstance(angle, (int, float)) and not self.calibration_slider.isSliderDown():
            self.calibration_slider.blockSignals(True)
            self.calibration_slider.setValue(int(angle))
            self.calibration_value.setText(f"{int(angle)}°")
            self.calibration_slider.blockSignals(False)
        _set_plain_text_keep_scroll(self.pose, json.dumps(snapshot.pose, ensure_ascii=False, indent=2) if snapshot.pose is not None else "暂无位置数据")
        _set_plain_text_keep_scroll(self.buttons, json.dumps(snapshot.button, ensure_ascii=False, indent=2) if snapshot.button is not None else "暂无按钮信息")
        self._device_status_items = snapshot.status.get("device_status") or []
        self.render_device_status()

    def render_device_status(self) -> None:
        connected = self.vm.state.name in DEVICE_STATUS_STATES
        self.device_status.setText(format_device_status(self._device_status_items) if connected else "")

    def render_profiles(self, profiles: list[str]) -> None:
        self.profile.blockSignals(True)
        self.profile.clear()
        for value in profiles:
            self.profile.addItem(PROFILE_NAMES.get(value, value), value)
        self.profile.blockSignals(False)
        self.profile_changed()

    def profile_changed(self) -> None:
        self._config_form.setRowVisible(self.other_ip, self.profile.currentData() == "a3_dual")
        # 人形双臂固定左手柄控左臂（index 0）、右手柄控右臂（index 1），无需选择当前臂。
        self._config_form.setRowVisible(self.side, self.profile.currentData() != "d3_dual")

    def device_double_clicked(self, item) -> None:
        device = item.data(Qt.ItemDataRole.UserRole)
        if self.vm.state.name == "ScanReady" and isinstance(device, Device) and self.profile.currentData():
            self.vm.connect(device, self.profile.currentData(), self.current_ip.text().strip(), self.other_ip.text().strip(), self.side.currentData(), self.view_side.currentData())

    def scale_changed(self, value: int) -> None:
        self.scale_value.setText(f"{value / 100:.2f}")
        if self.vm.state.name in {"Connected", "Teleoperating"}:
            self.vm.set_scale(value)

    def show_message(self, message: str) -> None:
        self.status.setText(message)

    def open_log_directory(self) -> None:
        opened = QDesktopServices.openUrl(
            QUrl.fromLocalFile(str(self.vm.log_directory))
        )
        if not opened:
            self.status.setText(f"日志目录：{self.vm.log_directory}")

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.vm.shutdown_state is ShutdownState.COMPLETE:
            event.accept()
        else:
            event.ignore()
            self.vm.shutdown()

    def render_devices(self, devices: list[Device]) -> None:
        self.devices.clear()
        for device in devices:
            self.devices.addItem(f"{device.name}（{device.address}）")
            self.devices.item(self.devices.count() - 1).setData(Qt.ItemDataRole.UserRole, device)
