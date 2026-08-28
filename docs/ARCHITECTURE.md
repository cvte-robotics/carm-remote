# CARM Quest Desktop 整体架构

本文面向维护者，描述当前仓库中可以从源码确认的运行结构、模块边界、数据流、状态管理、Quest 输入链、机械臂控制链和 HTTP/SSE API。

## 1. 系统组成

应用运行时包含两个本机进程：

1. PyQt6 桌面进程：显示界面、管理用户工作流、发送命令并展示状态。
2. Flask 后端进程：管理 Quest 连接、输入数据、机械臂会话、控制循环和资源清理。

桌面进程通过 `QProcess` 启动后端。两者使用 `127.0.0.1:1999` 上的 HTTP 和 SSE 通信。Quest 数据通过 ADB reverse 到达后端监听的 `127.0.0.1:9999`，不会经过 Qt 事件循环。

```mermaid
flowchart LR
    User[操作员] --> Window[PyQt MainWindow]
    Window --> VM[ViewModel]
    VM -->|QProcess| Backend[Flask 后端]
    VM -->|HTTP 命令| Backend
    Backend -->|SSE 状态| VM

    Quest[Quest] -->|ADB reverse / TCP| Receiver[QuestTcpReceiver]
    Receiver --> Module[QuestInputModule]
    Module --> Controller[JoystickController]
    Controller --> Session[ArmSession]
    Session --> Command[CommandController]
    Command --> Carm[Carm SDK]
    Carm --> Arm[机械臂]
```

## 2. 模块边界

| 层级 | 主要代码 | 职责 |
|---|---|---|
| 桌面视图 | `qt_app/app/main_window.py` | 创建控件、接收用户操作、根据状态控制可用性 |
| 桌面编排 | `qt_app/app/view_model.py` | 启动后端、组织扫描/连接/初始化/停止流程、过滤迟到响应 |
| 桌面通信 | `http_client.py`、`sse_client.py`、`backend_process.py` | HTTP、SSE 与后端子进程生命周期 |
| 后端组合 | `server/desktop/app.py`、`runtime.py` | 创建 Flask 服务、注入唯一 controller、机械臂探测、心跳和进程级清理 |
| API 适配 | `server/api/joystick_routes.py` | 把 HTTP/SSE 请求映射到 controller |
| 会话主体 | `server/basic/joystick/session/controller.py` | 输入会话、共享状态、SSE 快照、机械臂会话和停止编排 |
| 输入注册 | `server/basic/joystick/sources/registry.py` | 设备扫描身份、source lease 和模块生命周期 |
| Quest 适配 | `include/quest/` | ADB 扫描、TCP 协议、输入解析、重连和资源回收 |
| 机械臂会话 | `server/basic/joystick/session/arm.py` | 单臂/双臂 binding、初始化、控制线程和停止 |
| 控制计算 | `server/basic/joystick/core/` | 坐标变换、目标计算、命令门禁和末端命令 |
| 输入合同 | `include/carm_teleop_input_contracts.py` | provider、设备、lease、输入批次和传输事件的数据结构 |

组合关系由 `server/desktop/app.py` 建立：`DesktopRuntime` 创建一个 `JoystickController`，随后将同一个 controller 注入所有 joystick 路由。

## 3. 启动流程

根目录脚本只负责转发参数：

- Windows：`run_windows.ps1` 调用 `qt_app/run_windows.ps1`
- Linux：`run_linux.sh` 调用 `qt_app/run_linux.sh`

平台脚本依次执行：

1. 检查 Python 版本是否为 3.11 或 3.12。
2. 创建 `qt_app/.venv-windows` 或 `qt_app/.venv-linux`。
3. 检查依赖，缺失时从 `qt_app/requirements.txt` 安装。
4. 执行 `qt_app/preflight.py`。
5. 从仓库根目录运行 `qt_app/main.py`。

`preflight.py` 检查 Python 依赖、Carm 包版本、ADB 命令、端口 `1999` 和 `9999`、运行所需文件以及 Quest 模块导入。

`qt_app/main.py` 创建 `QApplication`、配置日志、再次执行启动检查，然后创建 `MainWindow`。`ViewModel.start()` 通过 `BackendProcess` 启动：

```text
<当前虚拟环境 Python> -m server.desktop.app
```

后端启动后监听 `127.0.0.1:1999`。Qt 每 250 ms 请求一次 `/api/desktop/health`，返回的 `instance_token` 与本窗口生成的 token 相同时，才进入 `Idle` 状态并开始每秒 heartbeat。

## 4. 用户工作流

```mermaid
sequenceDiagram
    actor User as 操作员
    participant UI as MainWindow
    participant VM as ViewModel
    participant API as Desktop API
    participant JC as JoystickController
    participant Quest as QuestInputModule
    participant Arm as ArmSession

    User->>UI: 输入当前臂 IP，点击扫描
    UI->>VM: scan(ip)
    VM->>API: POST /api/joystick/scan
    API->>JC: 探测机械臂并扫描 Quest
    API-->>VM: devices + arm_probe + allowed_profiles

    User->>UI: 选择模式并双击 Quest
    VM->>API: POST /api/joystick/session_config
    VM->>API: POST /api/joystick/connect
    API->>Quest: ADB reverse、启动接收器并等待输入
    API-->>VM: 连接结果
    VM->>API: GET /api/joystick/events

    User->>UI: 点击初始化机械臂
    VM->>API: POST /api/joystick/init_arm
    API->>Arm: 创建单臂或双臂会话
    Arm-->>VM: 状态通过 SSE 返回

    User->>UI: 点击停止
    VM->>API: POST /api/joystick/disconnect
    API->>JC: stop_session(home)
```

扫描先调用 `probe_current_arm()` 读取当前机械臂的 `arm_name` 和 `arm_dof`，再扫描 ADB 设备：

| 探测结果 | 界面可选模式 |
|---|---|
| 名称包含 `A3` 且自由度为 6 | `a3_single`、`a3_dual` |
| 名称包含 `D3` 且自由度为 7 | `d3_dual` |

单臂只创建 master binding。双臂创建 master 和 slave binding：

- 两个 IP 不同时，分别连接两个管理 IP。
- 两个 IP 相同时，master 使用 `arm_index=0`，slave 使用 `arm_index=1`，并选用七轴初始化配置。
- master 绑定界面选择的手柄侧，slave 绑定另一侧。

## 5. 数据流

### 5.1 命令流

```text
MainWindow
  -> ViewModel
  -> HttpClient
  -> Flask route
  -> DesktopRuntime operation gate
  -> JoystickController
```

`HttpClient` 为每个请求分配递增的 `request_id`。对于扫描、会话配置、连接、初始化、断开和关闭等关键工作流请求，ViewModel 只处理当前保存的 request ID，已被新操作替代的响应不会推进工作流状态。

扫描、会话配置、连接、初始化和断开请求还携带 `_desktop_generation`。后端要求它是非负整数，并拒绝小于已接收最新 generation 的操作。

### 5.2 Quest 输入流

```text
Quest TCP 帧
  -> QuestTcpReceiver
  -> QuestInputModule
  -> RawInputBatch（同一帧内的左右手数据）
  -> JoystickController
  -> QuestInputPolicy
  -> 对应手柄绑定的 CommandController
```

`QuestTcpReceiver` 接收换行分隔的 JSON 记录。连接必须先完成 protocol 2 的 `hello` / `hello_ack` 协商，之后才接收位姿帧和 heartbeat。

`QuestInputModule` 将一帧输入转换成不可变的 `RawInputBatch`。批次带有 source lease、transport generation 和 batch sequence，用来阻止旧连接的数据进入当前会话。

`QuestInputPolicy` 检查双手帧的一致性、位姿有效性和配置的位姿跳变阈值，并为控制派发产生与当前输入状态绑定的 token。

### 5.3 机械臂控制流

```text
CommandController 输入快照
  -> 坐标变换与灵敏度
  -> 生成相对位姿目标
  -> 校验输入/控制/连接代际
  -> Carm.dispatch_pose_for_connection()
  -> Carm.track_pose()
  -> 当前机械臂连接
```

每个机械臂对应一个 `CommandController` 和一个控制线程。控制循环频率由 `CONTROL_LOOP_HZ` 定义为 72 Hz。

控制代码中的主要手柄语义：

| 输入 | 当前行为 |
|---|---|
| Grip 大于 100 | 允许生成位姿目标；首次按下时同步机械臂反馈基准 |
| 松开 Grip | 清除待发目标并等待下次重新同步 |
| Trigger | 映射为 `0..0.08` 的末端开合位置 |
| A/X 且未按 Grip | 发送配置中的初始化关节位置 |
| B/Y | 切换旋转锁定状态并重建映射基准 |

末端类型在机械臂准备阶段读取：

- `flange`：不发送末端命令。
- `hand`：调用 `set_hand`。
- `gripper`：调用 `set_gripper`。
- 其他值或读取异常：按 `gripper` 处理。

### 5.4 状态展示流

```text
JoystickState
  -> JoystickController.get_stream_snapshot()
  -> /api/joystick/events
  -> SseClient
  -> ViewModel._event()
  -> MainWindow.render_snapshot()
```

SSE 以 `data_sequence` 判断状态是否变化。序列变化时立即发送快照；连续 3 秒无变化时发送 heartbeat 帧。Qt 当前将顶层 `pose` 和 `button` 展示在界面中，并根据 transport 和 terminal 状态更新提示。

## 6. 状态与生命周期

### 6.1 Qt 工作流状态

`UiState.name` 使用以下状态：

| 状态 | 含义 |
|---|---|
| `BackendStarting` | 后端正在启动和认领 |
| `Idle` | 可以扫描 |
| `Scanning` | 正在探测机械臂并扫描 Quest |
| `ScanReady` | 已有扫描结果，可以选择配置并连接 |
| `Connecting` | 正在提交配置并连接 Quest |
| `Connected` | Quest 已连接，可以初始化机械臂 |
| `Initializing` | 正在初始化机械臂 |
| `Teleoperating` | 机械臂初始化完成 |
| `Stopping` | 正在停止会话 |
| `ShuttingDown` | 窗口关闭流程正在清理后端 |
| `BackendFailed` / `BackendStopped` | 后端启动失败或已停止 |

窗口关闭还使用独立的 `ShutdownState`：`NONE`、`STOPPING_SESSION`、`REQUESTING_BACKEND_SHUTDOWN`、`WAITING_PROCESS_EXIT`、`COMPLETE`。

### 6.2 后端生命周期

`DesktopRuntime` 使用以下生命周期：

```text
running
  -> shutdown_requested
  -> cleaning
  -> cleaned
  -> server_stopping
  -> stopped
```

清理失败时进入 `retry_wait`，按 1 秒、2 秒、5 秒的间隔继续重试，后续保持 5 秒间隔。

Qt 每秒调用一次 `/api/desktop/heartbeat`。后端首次启动有 10 秒等待时间；收到 heartbeat 后，连续 3 秒未刷新会触发进程级 `hold` 清理。

### 6.3 Source binding

输入连接使用 `SourceLease` 和 `ActiveSourceBinding` 标识所有权：

```text
PROVISIONAL -> ACTIVE -> CLOSED
```

只有 ACTIVE 且 `input_ready=true` 的 binding 可以参与控制。lease 中的 provider、module instance 和 input generation 必须与当前连接匹配。

## 7. Quest 连接与协议

Quest 连接由 `QuestInputModule.connect()` 完整管理：

1. 检查 ADB 设备状态和 Quest 身份。
2. 创建并启动 `QuestTcpReceiver`。
3. 执行 `adb reverse tcp:9999 tcp:9999`。
4. 启动源码中配置的 Quest 包。
5. 等待 protocol handshake 和首个有效输入帧。

协议常量位于 `include/quest/constants.py`：

| 项目 | 当前值 |
|---|---|
| TCP 地址 | `127.0.0.1:9999` |
| 协议版本 | `2` |
| 控制记录前缀 | `@carm-control/2 ` |
| handshake 超时 | 5 秒 |
| 接收超时 | 5 秒 |
| 首帧等待时间 | 20 秒 |
| 单行上限 | 64 KiB |

握手完成后支持：

- 普通双手输入帧。
- `heartbeat` / `heartbeat_ack`。
- 后端清理时尽力发送 `session_end`。
- 无效记录返回 `protocol_error` 并终止当前连接。

普通传输断开进入 5 秒恢复窗口。协议错误、无效输入帧或 listener 错误属于 terminal 错误，不进入普通恢复路径。

## 8. 停止与清理

停止模式只有 `home` 和 `hold`：

| 模式 | 行为 |
|---|---|
| `home` | 停止控制线程后，将机械臂移动到零关节位置，再断开 |
| `hold` | 停止控制线程、清除目标并断开，不发送回零运动 |

界面“停止手柄遥操”请求 `home`。如果输入已经断开，controller 会将 `home` 降级为 `hold`。窗口关闭请求 `hold`，桌面 heartbeat 超时和后端退出也固定执行 `hold`。

会话清理包含：

1. 停止机械臂控制线程。
2. 根据 stop mode 决定是否回零。
3. 关闭 source binding。
4. 删除 ADB reverse。
5. 通过 ADB 对源码中配置的 Quest 包执行 `force-stop`。
6. 停止并回收 `QuestTcpReceiver`。
7. 重置 joystick 状态。

进程级清理还会关闭 source registry、joystick 清理线程和 Flask server。资源清理返回失败时，`DesktopRuntime` 不会把 lifecycle 标记为完成，而是继续重试。

## 9. HTTP API

### 9.1 通用约定

- Base URL：`http://127.0.0.1:1999`
- JSON 接口的结果字段位于响应顶层。
- `code: 1` 表示业务成功，`code: 0` 表示业务失败。
- 业务失败通常仍返回 HTTP 200，调用方必须检查 `code`。
- 服务绑定 loopback；响应包含 `Access-Control-Allow-Origin: *`。

成功响应：

```json
{
  "msg": "操作结果",
  "code": 1
}
```

失败响应：

```json
{
  "msg": "错误原因",
  "code": 0
}
```

### 9.2 接口总表

| 方法与路径 | 请求字段 | 主要响应数据 | 作用 |
|---|---|---|---|
| `GET /api/desktop/health` | 无 | `data.ready`、`pid`、`instance_token`、`log_file`、lifecycle 字段 | Qt 认领后端并读取清理状态 |
| `POST /api/desktop/heartbeat` | 空 JSON | 无额外数据 | 刷新桌面 heartbeat |
| `POST /api/desktop/shutdown` | 空 JSON | `data.lifecycle`、`cleanup_complete`、`retryable`、`attempt` | 请求进程级清理 |
| `POST /api/joystick/scan` | `current_arm_ip`、`_desktop_generation` | `data`、`arm_probe`、`allowed_profiles` | 探测机械臂并扫描 Quest |
| `POST /api/joystick/session_config` | `mode`、`master_side`、`view_side`、`slave_ip`、`_desktop_generation` | 无额外数据 | 保存连接前配置 |
| `POST /api/joystick/connect` | `address`、可选 `name`、`_desktop_generation` | 无额外数据 | 连接扫描到的 Quest |
| `POST /api/joystick/init_arm` | `mode`、`master_ip`、`slave_ip`、`master_side`、`_desktop_generation` | 无额外数据 | 创建并初始化机械臂会话 |
| `POST /api/joystick/calibration` | `value` | 无额外数据 | 设置标定方向，范围 `0..3` |
| `POST /api/joystick/scale` | `value` | 无额外数据 | 设置灵敏度，范围 `0.1..1.0` |
| `POST /api/joystick/view_side` | `value` | 无额外数据 | 设置兼容视图侧，`left` 或 `right` |
| `POST /api/joystick/disconnect` | `stop_mode`、`reason`、`_desktop_generation` | 无额外数据 | 停止当前会话 |
| `GET /api/joystick/status` | 无 | `data` 状态对象 | 获取当前状态快照 |
| `GET /api/joystick/events` | 无 | SSE 数据流 | 订阅状态和输入快照 |

### 9.3 扫描响应

```json
{
  "data": [
    {
      "name": "设备名称",
      "address": "ADB_SERIAL"
    }
  ],
  "arm_probe": {
    "ip": "10.42.0.101",
    "arm_name": "机械臂名称",
    "arm_dof": 6,
    "family": "A3"
  },
  "allowed_profiles": [
    "a3_single",
    "a3_dual"
  ],
  "msg": "扫描完成",
  "code": 1
}
```

### 9.4 Status 数据

`GET /api/joystick/status` 的 `data` 包含：

- `ble_connected`、`arm_initialized`、`status`
- `calibration`、`scale`
- `battery_level`、`rssi`
- `source_type`、`is_quest_device`
- `mode`、`master_side`、`view_side`
- `master_arm`、`slave_arm`
- 当前 view side 的 `pose`、`button`
- `transport.state`、`protocol`、`connection_id`、`last_rx_age_ms`、`disconnect_reason`

## 10. SSE 契约

订阅：

```http
GET /api/joystick/events
Accept: text/event-stream
```

变化帧的顶层结构：

```json
{
  "sequence": 1,
  "pose": null,
  "button": null,
  "gripper_pose": null,
  "gripper_tau": null,
  "status": {
    "ble_connected": true,
    "status": "connected",
    "battery_level": null,
    "rssi": null,
    "source_type": "quest",
    "mode": "single",
    "master_side": "right"
  },
  "device": {
    "address": "ADB_SERIAL",
    "name": "设备名称"
  },
  "arms": {
    "master": {},
    "slave": {}
  },
  "hands": {
    "left": {
      "pose": null,
      "button": null,
      "gripper_pose": null,
      "gripper_tau": null
    },
    "right": {
      "pose": null,
      "button": null,
      "gripper_pose": null,
      "gripper_tau": null
    }
  },
  "stream_terminal": false,
  "emit_disconnect_frame": false,
  "transport": {
    "state": "healthy",
    "protocol": 2,
    "connection_id": "连接标识",
    "last_rx_age_ms": 0,
    "disconnect_reason": null
  },
  "timestamp": 0
}
```

`pose` 字段由以下成员组成：

```text
x, y, z, qx, qy, qz, qw, timestamp, confidence,
linear_velocity, angular_velocity, acceleration
```

`button` 字段由以下成员组成：

```text
trigger, grip, btn_xa, btn_yb, btn_menu, btn_system,
joystick_x, joystick_y, joystick_z, menu_capture, timestamp
```

连续 3 秒没有新的 sequence 时，服务发送 `heartbeat: true`，并将该帧顶层 `pose`、`button` 设为 `null`。会话终止时 `stream_terminal=true`；需要发送最终断连帧时 `emit_disconnect_frame=true`，发送后关闭 SSE 流。

## 11. 配置与日志

`config/joystick_config.json` 在后端导入 joystick runtime 时读取一次，运行期间不自动重新加载。配置内容包括：

- `quest_pose_jump.position_threshold_m`
- `quest_pose_jump.rotation_threshold_deg`
- `single_arm.gripper_tau` 与六轴 `init_joint`
- `dual_arm.left/right.gripper_tau`
- 双臂左右侧的 `six_axis` 与 `seven_axis` 初始化关节

数值与数组长度由 `JoystickConfigSnapshot` 校验。

日志分为：

- `desktop-ui.log`：Qt 工作流、HTTP/SSE 和界面异常。
- `desktop-backend.log`：后端、Quest、控制和清理过程。

日志使用 5 MiB 轮转文件并保留 3 个备份。实际目录由 `qt_app/app/runtime_paths.py` 选择，并显示在界面中。

## 12. 代码事实与验证边界

本文中的端口、状态、字段、超时和调用关系来自当前源码与配置。启动前检查只执行软件环境和导入检查；Quest 连接、机械臂初始化、控制运动以及 `home` / `hold` 的物理结果仍需要在相应设备上验证。
