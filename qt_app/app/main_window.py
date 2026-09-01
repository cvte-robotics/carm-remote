from __future__ import annotations

import json
from typing import Any

from PyQt6.QtCore import Qt, QUrl
from PyQt6.QtGui import QCloseEvent, QDesktopServices
from PyQt6.QtWidgets import (
    QComboBox,
    QFormLayout,
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
        root = QWidget(self)
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        ip_row = QHBoxLayout()
        ip_row.addWidget(QLabel("当前臂 IP:"))
        self.current_ip = QLineEdit("10.42.0.101")
        self.current_ip.setMinimumHeight(40)
        self.current_ip.textChanged.connect(self.invalidate_scan)
        ip_row.addWidget(self.current_ip, 1)
        layout.addLayout(ip_row)

        actions = QHBoxLayout()
        self.scan_button = QPushButton("扫描设备")
        self.scan_button.clicked.connect(lambda: self.vm.scan(self.current_ip.text().strip()))
        self.init_button = QPushButton("初始化机械臂")
        self.init_button.clicked.connect(self.vm.initialize)
        self.stop_button = QPushButton("停止手柄遥操")
        self.stop_button.clicked.connect(self.vm.stop)
        for button in (self.scan_button, self.init_button, self.stop_button):
            button.setMinimumHeight(48)
            actions.addWidget(button)
        layout.addLayout(actions)

        self.devices = QListWidget()
        self.devices.itemDoubleClicked.connect(self.device_double_clicked)
        layout.addWidget(self.devices)
        self.status = QLabel("正在启动后端...")
        layout.addWidget(self.status)

        log_row = QHBoxLayout()
        log_row.addWidget(QLabel("日志:"))
        self.log_location = QLabel(str(self.vm.log_directory))
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
        self.open_logs_button.clicked.connect(self.open_log_directory)
        log_row.addWidget(self.open_logs_button)
        layout.addLayout(log_row)

        config = QGroupBox("遥操配置")
        form = QFormLayout(config)
        self.profile = QComboBox()
        self.profile.currentIndexChanged.connect(self.profile_changed)
        form.addRow("遥操类型:", self.profile)
        self.other_ip = QLineEdit("10.42.0.102")
        form.addRow("另一臂 IP:", self.other_ip)
        self.side = QComboBox()
        self.side.addItem("作为左臂", "left")
        self.side.addItem("作为右臂", "right")
        form.addRow("当前机械臂:", self.side)
        self.view_side = QComboBox()
        self.view_side.addItem("左手柄", "left")
        self.view_side.addItem("右手柄", "right")
        self.view_side.currentIndexChanged.connect(lambda: self.vm.set_view_side(self.view_side.currentData()))
        form.addRow("手柄数据:", self.view_side)
        scale_row = QHBoxLayout()
        self.scale = QSlider(Qt.Orientation.Horizontal)
        self.scale.setRange(10, 100)
        self.scale.setValue(40)
        self.scale.valueChanged.connect(self.scale_changed)
        self.scale_value = QLabel("0.4")
        scale_row.addWidget(self.scale, 1)
        scale_row.addWidget(self.scale_value)
        form.addRow("灵敏度:", scale_row)
        layout.addWidget(config)

        calibration = QGroupBox("标定")
        cal_layout = QVBoxLayout(calibration)
        instruction = QLabel("手柄前后左右平移，观察 X 轴坐标变化，确定 X 轴朝向，点击相应按钮")
        instruction.setWordWrap(True)
        cal_layout.addWidget(instruction)
        cal_buttons = QHBoxLayout()
        self.calibration_buttons = []
        for index, title in enumerate(("X向左", "X向右", "X向前", "X向后")):
            button = QPushButton(title)
            button.clicked.connect(lambda checked=False, i=index: self.vm.calibration(i))
            self.calibration_buttons.append(button)
            cal_buttons.addWidget(button)
        cal_layout.addLayout(cal_buttons)
        layout.addWidget(calibration)

        data = QHBoxLayout()
        self.pose = QTextEdit(); self.pose.setReadOnly(True)
        self.buttons = QTextEdit(); self.buttons.setReadOnly(True)
        pose_box = QVBoxLayout(); pose_box.addWidget(QLabel("位置信息:")); pose_box.addWidget(self.pose)
        button_box = QVBoxLayout(); button_box.addWidget(QLabel("按钮信息:")); button_box.addWidget(self.buttons)
        data.addLayout(pose_box, 1); data.addLayout(button_box, 1)
        layout.addLayout(data, 1)
        self.other_ip.setVisible(False)
        self._controls = [self.scan_button, self.init_button, self.stop_button, self.profile, self.other_ip, self.side, self.view_side, self.scale]

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
        for button in self.calibration_buttons:
            button.setEnabled(active_adjustment)
        self.devices.setEnabled(name == "ScanReady")

    def render_snapshot(self, snapshot: Snapshot) -> None:
        self.pose.setPlainText(json.dumps(snapshot.pose, ensure_ascii=False, indent=2) if snapshot.pose is not None else "暂无位置数据")
        self.buttons.setPlainText(json.dumps(snapshot.button, ensure_ascii=False, indent=2) if snapshot.button is not None else "暂无按钮信息")

    def render_profiles(self, profiles: list[str]) -> None:
        self.profile.blockSignals(True)
        self.profile.clear()
        for value in profiles:
            self.profile.addItem(PROFILE_NAMES.get(value, value), value)
        self.profile.blockSignals(False)
        self.profile_changed()

    def profile_changed(self) -> None:
        self.other_ip.setVisible(self.profile.currentData() == "a3_dual")

    def device_double_clicked(self, item) -> None:
        device = item.data(Qt.ItemDataRole.UserRole)
        if self.vm.state.name == "ScanReady" and isinstance(device, Device) and self.profile.currentData():
            self.vm.connect(device, self.profile.currentData(), self.current_ip.text().strip(), self.other_ip.text().strip(), self.side.currentData(), self.view_side.currentData())

    def scale_changed(self, value: int) -> None:
        self.scale_value.setText(f"{value / 100:.1f}")
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
