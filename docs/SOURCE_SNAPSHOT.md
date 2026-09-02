# 源码文件说明

本文按当前仓库文件树说明各文件职责。这里只记录仓库中实际存在的文件；运行关系与 API 详见 [整体架构](ARCHITECTURE.md)。

## 1. 文件树

```text
carm_quest_desktop/
├── .gitattributes
├── .gitignore
├── LICENSE
├── README.md
├── run_linux.sh
├── run_windows.ps1
├── config/
│   └── joystick_config.json
├── docs/
│   ├── ARCHITECTURE.md
│   ├── END_EFFECTOR_DEVELOPMENT.md
│   └── SOURCE_SNAPSHOT.md
├── include/
│   ├── carm_teleop_input_contracts.py
│   └── quest/
│       ├── __init__.py
│       ├── adb.py
│       ├── config.py
│       ├── constants.py
│       ├── module.py
│       ├── parser.py
│       ├── policy.py
│       └── receiver.py
├── qt_app/
│   ├── .gitignore
│   ├── main.py
│   ├── preflight.py
│   ├── pyproject.toml
│   ├── requirements.txt
│   ├── run_linux.sh
│   ├── run_windows.ps1
│   ├── scripts/
│   │   └── collect_data.py
│   └── app/
│       ├── __init__.py
│       ├── backend_process.py
│       ├── http_client.py
│       ├── logging_config.py
│       ├── main_window.py
│       ├── models.py
│       ├── runtime_paths.py
│       ├── sse_client.py
│       └── view_model.py
└── server/
    ├── __init__.py
    ├── api/
    │   └── joystick_routes.py
    ├── desktop/
    │   ├── __init__.py
    │   ├── app.py
    │   ├── runtime.py
    │   └── watchdog.py
    └── basic/
        └── joystick/
            ├── __init__.py
            ├── end_effectors/
            │   ├── __init__.py
            │   ├── carm.py
            │   ├── blank.py
            │   └── passthrough.py
            ├── core/
            │   ├── __init__.py
            │   ├── command_controller.py
            │   ├── constants.py
            │   ├── models.py
            │   └── pose_math.py
            ├── runtime/
            │   ├── __init__.py
            │   ├── config.py
            │   ├── context.py
            │   ├── dependencies.py
            │   └── paths.py
            ├── session/
            │   ├── __init__.py
            │   ├── arm.py
            │   ├── cleanup.py
            │   ├── constants.py
            │   ├── control_transaction.py
            │   ├── controller.py
            │   ├── observation.py
            │   └── state.py
            └── sources/
                ├── __init__.py
                ├── adapters.py
                └── registry.py
```

## 2. 根目录

| 文件 | 职责 |
|---|---|
| `.gitattributes` | 定义文本换行规则以及图片、压缩包、音视频等二进制文件类型 |
| `.gitignore` | 排除虚拟环境、缓存、日志和本地编辑器目录 |
| `LICENSE` | 定义当前仓库源码使用的 MIT License；不覆盖仓库外部依赖和设备软件 |
| `README.md` | 面向使用者的项目介绍、快速启动和基本操作 |
| `run_windows.ps1` | Windows 根入口，将参数转发给 `qt_app/run_windows.ps1` |
| `run_linux.sh` | Linux 根入口，将参数转发给 `qt_app/run_linux.sh` |

## 3. 配置与文档

| 文件 | 职责 |
|---|---|
| `config/joystick_config.json` | 保存 Quest 位姿跳变阈值以及单臂/双臂初始化关节配置 |
| `docs/ARCHITECTURE.md` | 记录整体架构、数据流、生命周期、控制链和 API |
| `docs/END_EFFECTOR_DEVELOPMENT.md` | 记录新增末端实现的扩展合同、加载方式、调用语义和验证方法 |
| `docs/SOURCE_SNAPSHOT.md` | 记录当前文件树和逐文件职责 |

## 4. Qt 桌面应用

### 4.1 启动与依赖

| 文件 | 职责 |
|---|---|
| `qt_app/main.py` | 创建 `QApplication`、配置 UI 日志、运行 preflight 并显示主窗口 |
| `qt_app/preflight.py` | 检查 Python、依赖、Carm 版本、ADB、端口、仓库文件和 Quest 模块导入 |
| `qt_app/run_windows.ps1` | 创建 Windows 虚拟环境、安装依赖、运行 preflight 和桌面入口 |
| `qt_app/run_linux.sh` | 创建 Linux 虚拟环境、安装依赖、运行 preflight 和桌面入口 |
| `qt_app/requirements.txt` | 启动脚本使用的 Python 依赖列表 |
| `qt_app/pyproject.toml` | 项目名称、Python 版本范围、依赖范围和 Ruff 行宽配置 |

### 4.2 `qt_app/scripts/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `collect_data.py` | `build_parser`、`create_arms`、`collect_plan_data`、`run` | 连接并校验 A3 单臂、A3 双臂或 D3 双臂，按固定间隔采集机械臂和末端计划位置/速度/力矩，并写入 CSV |

### 4.3 `qt_app/app/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | 将目录声明为 Python package |
| `main_window.py` | `MainWindow` | 创建主窗口、控件和信号绑定；根据 `UiState` 控制按钮和配置项 |
| `view_model.py` | `ViewModel` | 编排后端启动、扫描、会话配置、Quest 连接、机械臂初始化、停止和窗口关闭 |
| `models.py` | `UiState`、`ShutdownState`、`Device`、`ArmProbe`、`Snapshot` | 定义 Qt 层状态和展示数据 |
| `backend_process.py` | `BackendProcess` | 使用 `QProcess` 启动 `server.desktop.app`，传递实例 token 和日志目录，收集标准输出与错误 |
| `http_client.py` | `HttpClient` | 使用 `QNetworkAccessManager` 发送异步 GET/POST，为请求分配 ID 并解析 JSON 响应 |
| `sse_client.py` | `SseClient` | 连接 `/api/joystick/events`，按 SSE 记录解析 JSON，处理断开与重试 |
| `logging_config.py` | `configure_ui_logging` | 配置 UI 控制台日志、轮转文件日志和未捕获异常记录 |
| `runtime_paths.py` | `backend_root`、`log_dir`、日志路径函数 | 解析仓库根目录和可写日志目录 |

Qt 层的依赖方向是：

```text
MainWindow
  -> ViewModel
     -> BackendProcess
     -> HttpClient
     -> SseClient
     -> models
```

## 5. 桌面后端

### 5.1 `server/desktop/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | 桌面后端组合 package |
| `app.py` | `create_app`、`main` | 配置后端日志、创建 Flask app、注册路由、绑定 `127.0.0.1:1999` 并运行 Werkzeug server |
| `runtime.py` | `DesktopRuntime`、`Lifecycle`、`probe_current_arm` | 创建唯一 controller、探测机械臂、执行 generation 门禁、管理 heartbeat 与进程级清理 |
| `watchdog.py` | `HeartbeatWatchdog` | 用独立线程监视 Qt heartbeat，超时后通知 runtime |

### 5.2 `server/api/`

| 文件 | 职责 |
|---|---|
| `joystick_routes.py` | 注册扫描、连接、会话配置、初始化、校准、灵敏度、数据显示侧、停止、状态和 SSE 路由；统一生成 `code/msg` 响应 |

桌面管理路由位于 `server/desktop/app.py`，joystick 路由位于 `server/api/joystick_routes.py`。两者共同构成 Qt 使用的本机 API。

## 6. 遥操主体

### 6.1 Package facade

| 文件 | 职责 |
|---|---|
| `server/__init__.py` | 后端顶层 package 标记 |
| `server/basic/joystick/__init__.py` | 组织运行路径、依赖、配置、Quest 常量、数据模型、会话与控制类的公开导出；提供 controller facade |

### 6.2 `end_effectors/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | `EndEffectorBase`、`EndEffectorContext`、加载和解析函数 | 定义末端合同、动态发现包内实现并提供统一构造入口 |
| `blank.py` | `BlankEndEffector` | 提供不控制硬件的最小开发示例 |
| `carm.py` | `CarmEndEffector` | 根据 Carm SDK 末端配置执行原生末端控制 |
| `passthrough.py` | `PassthroughEndEffector` | 将触发值转换为六指 CAN-FD passthrough 帧 |

末端实现通过启动参数选择，默认是 `carm`；当前仓库还提供 `passthrough` 和不控制硬件的 `blank` 开发示例。新增实现放入此目录并由启动时扫描发现，具体合同见 [新增末端控制方式](END_EFFECTOR_DEVELOPMENT.md)。

### 6.3 `core/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | 控制核心 package |
| `constants.py` | `CONTROL_LOOP_HZ` 等 | 定义 72 Hz 控制循环、末端控制速度和位姿分段阈值 |
| `models.py` | `pose_to_dict`、`button_to_dict`、创建函数 | 在输入对象、兼容模型和 JSON 字典之间转换 |
| `pose_math.py` | `project_input_to_arm`、`apply_pose_delta`、连续位姿差函数 | 完成坐标投影、四元数处理、缩放和相对位姿计算 |
| `command_controller.py` | `CommandController` | 管理一个机械臂连接、反馈同步、初始化运动、目标计算和 home/stop 所需的机械臂操作 |

`CommandController` 负责计算控制目标并封装单臂操作；会话层的 `ControlTransaction` 负责实时采样、策略门禁、物理派发和派发后的身份确认。

### 6.4 `runtime/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | joystick runtime package |
| `config.py` | `JoystickConfigSnapshot`、`ArmControlConfig` | 加载并校验 `joystick_config.json`，按单臂/双臂和六轴/七轴解析会话配置 |
| `context.py` | `JOYSTICK_CONFIG` | 在模块导入时创建当前后端进程使用的配置快照 |
| `dependencies.py` | `Carm` adapter、依赖可用性标记 | 导入 NumPy、SciPy 和 Carm，并为 Carm 连接增加连接身份绑定的位姿派发 |
| `paths.py` | 输入模块路径函数 | 选择输入模块根目录、设置 import path，并校验桌面运行使用的 Quest 模块来源 |

### 6.5 `session/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | joystick session package |
| `constants.py` | 线程超时常量 | 定义控制线程和清理线程等待时间 |
| `state.py` | `JoystickState` | 保存设备、输入、双手、机械臂、transport、控制代际和 SSE sequence 等共享状态 |
| `arm.py` | `ArmRuntime`、`BaseArmSession` | 创建 arm binding，初始化一个或两个 `CommandController`，启动控制线程并执行 stop/home |
| `control_transaction.py` | `ControlTransaction`、`LoopTermination` | 为一个 ArmRuntime 运行完整的 72 Hz 实时控制事务，执行采样、策略门禁、派发和终止回报 |
| `cleanup.py` | `SessionCleanup`、`CleanupContext`、`CleanupResult` | 按固定上下文停止机械臂和输入源，失败时保留原 owner 供同一清理事务重试 |
| `observation.py` | `ObservationSnapshot`、HTTP/SSE 投影函数 | 生成不可变 Host 状态快照，并分别投影为 HTTP 状态和 SSE 载荷 |
| `controller.py` | `JoystickController` | 遥操会话主控制器；拥有 source registry、共享状态、机械臂 session、生命周期门禁和清理编排 |

`JoystickController` 的主要责任链：

```text
scan_devices
  -> connect_device
  -> set_session_config
  -> init_arm
  -> publish_input / publish_transport
  -> get_observation_snapshot
  -> stop_session
```

### 6.6 `sources/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | joystick source package |
| `registry.py` | `SourceRegistry`、`SourceLeaseAllocator`、`ActiveSourceBinding`、`BindingPhase` | 注册输入模块、生成扫描 descriptor 和 lease、维护 provisional/active/closed binding |
| `adapters.py` | sample/legacy 转换函数 | 严格校验输入对象，把中立 `PoseSample` / `ButtonSample` 转换为控制层使用的兼容对象 |

## 7. 中立输入合同

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `include/carm_teleop_input_contracts.py` | `SourceCapabilities`、`DeviceCandidate`、`DeviceDescriptor`、`SourceLease`、`RawInputBatch`、`TransportEvent`、Protocols | 定义输入模块与 host 之间不可变的数据合同、身份字段和 provider 接口 |

关键合同分为四组：

- 设备发现：`SourceAvailability`、`DeviceCandidate`、`DeviceDescriptor`
- 连接身份：`SourceLease`、`ConnectResult`
- 输入数据：`PoseSample`、`ButtonSample`、`HandInputUpdate`、`RawInputBatch`
- Host 接口：`SourceEventSink`、`InputSourceModule`

## 8. Quest 输入模块

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `include/quest/__init__.py` | `QuestInputModule` 导出 | 声明 Quest module ID、API 版本并导出模块实现 |
| `include/quest/constants.py` | Quest 协议与超时常量 | 定义 TCP 地址、端口、包名、协议版本、heartbeat、handshake、接收超时和记录上限 |
| `include/quest/adb.py` | ADB 辅助函数 | 执行 adb、解析设备列表、识别 Quest、读取设备状态、重启 ADB 服务和启动 Quest 包 |
| `include/quest/receiver.py` | `QuestTcpReceiver` | 监听 `127.0.0.1:9999`，处理 protocol 2 协商、输入帧、heartbeat、超时和连接 generation |
| `include/quest/parser.py` | `parse_quest_frame` | 把一条 Quest payload 转换为左右手位姿、按钮和 frame index 数据 |
| `include/quest/config.py` | `QuestPoseJumpConfig` | 解析并校验位置与旋转跳变阈值 |
| `include/quest/policy.py` | `QuestInputPolicy`、decision/token 数据类 | 判断输入是否可接受、处理跳变与 rebase，并为控制派发生成可验证 token |
| `include/quest/module.py` | `QuestInputModule` | 实现 `InputSourceModule`；拥有 ADB reverse、receiver、serial、lease、重连 timer 和 Quest 资源清理 |

Quest 文件之间的调用关系：

```text
QuestInputModule
  ├── adb.py       设备扫描、reverse、启动与停止
  ├── receiver.py  TCP 生命周期和协议
  ├── parser.py    输入帧解析
  ├── policy.py    输入接受、rebase 与派发门禁
  ├── config.py    位姿跳变配置
  └── constants.py 端口、协议和超时
```

## 9. 维护约定

- README 只保留面向使用者的介绍和快速使用。
- 架构、数据流、状态、API 和源码职责放在 `docs/`。
- 文档中的文件、路由、字段、状态和功能必须能在当前仓库中找到对应实现。
- 新增、删除或移动源码后，应同步更新本文件树。
