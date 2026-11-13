# 源码文件说明

本文按 [整体架构](ARCHITECTURE.md) 中记录的源码同步基线说明公开桌面投影的文件树与职责。

## 1. 文件树

```text
carm-remote/
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
│   ├── carm_teleop_intent.py
│   └── quest/
│       ├── __init__.py
│       ├── adb.py
│       ├── config.py
│       ├── constants.py
│       ├── device_status.py
│       ├── factory.py
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
            │   ├── joint_motion.py
            │   ├── mailbox.py
            │   ├── models.py
            │   ├── pose_mapping.py
            │   └── pose_math.py
            ├── runtime/
            │   ├── __init__.py
            │   ├── carm_link.py
            │   ├── config.py
            │   ├── dependencies.py
            │   ├── direct_config.py
            │   └── paths.py
            ├── session/
            │   ├── __init__.py
            │   ├── arm.py
            │   ├── arm_angle.py
            │   ├── cleanup.py
            │   ├── control_stats.py
            │   ├── control_transaction.py
            │   ├── controller.py
            │   ├── direct.py
            │   ├── end_effector_sender.py
            │   ├── maintenance.py
            │   ├── observation.py
            │   ├── state.py
            │   └── waist.py
            └── sources/
                ├── __init__.py
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
| `config/joystick_config.json` | 保存 Quest 位姿跳变阈值、各臂初始化/停止/遥操回位关节目标、D3 腰与臂型角参数 |
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
| `main_window.py` | `MainWindow`、`format_device_status`、`WAIST_STATUS_TEXT` | 创建主窗口、控件、样式表和信号绑定；根据 `UiState` 控制按钮和配置项，渲染设备电量与 D3 腰状态 |
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
| `joystick_routes.py` | 注册扫描、连接、会话配置、初始化、yaw 与兼容四档标定、灵敏度、关节速度档位、数据显示侧、停止、状态和 SSE 路由；统一生成 `code/msg` 响应，并接入排他准入；Desktop 关闭维护路由 |

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
| `mailbox.py` | `LatestValueCell`、`PublicationSignal`、`HandInputFrame`、`InputPublication` | 保存双手同批的最新 publication、头盔/机身输入、冻结输入帧及发布唤醒信号 |
| `models.py` | `pose_to_dict`、`button_to_dict`、`intent_to_dict`、`head_pose_to_dict`、`body_intent_to_dict` | 把冻结输入与功能结果转换为 HTTP/SSE JSON 字典 |
| `pose_math.py` | `build_calibration_matrix`、`legacy_mode_to_yaw`、`input_pose_to_arm`、`apply_pose_delta` | 用标准库构建 yaw 标定矩阵、完成坐标投影、连续四元数处理、缩放和相对位姿计算 |
| `pose_mapping.py` | `PoseMapping`、`select_output_baseline` | 手臂与头盔腰共用的输入/输出基线、目标计算和重建基线选择；由各自控制线程独占 |
| `joint_motion.py` | `JointMotionWatch` | 根据任务结束标识、新鲜位置与静止反馈判定遥操中关节回位是否到位 |
| `command_controller.py` | `CommandController`、`ControlParams`、`ControlSample`、`JointMoveAttempt` | 单臂映射、反馈、关节回位与 SDK 派发；映射和回位状态由所属控制线程独占 |

`CommandController` 负责计算控制目标并封装单臂操作；会话层的 `ControlTransaction` 负责实时采样、策略门禁、物理派发和派发后的身份确认。

### 6.4 `runtime/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | joystick runtime package |
| `config.py` | `JOYSTICK_CONFIG`、`JoystickConfigSnapshot`、`ArmControlConfig`、`WaistControlConfig`、`ArmAngleControlConfig`、`identify_arm_model` | 加载配置快照，识别型号，解析各臂三类关节目标、腰、臂型角与共享连接开关 |
| `dependencies.py` | `Carm`、`PendingReply`、`sdk_nowait`、`ARM_SDK_AVAILABLE` | 封装连接身份门禁、只发不等请求、登记回复与关节速度兼容；不再导入 NumPy/SciPy |
| `carm_link.py` | `CarmLink`、`CarmLinkRegistry`、`CarmChannel` | 同一控制端点共用 WebSocket、读线程与状态帧，各 `arm_index` 通过通道视图独立路由 |
| `direct_config.py` | `load_config`、`save_config` | 板端 Quest 直连配置校验与原子保存；Desktop 不启动对应管理器 |
| `paths.py` | 输入模块路径函数 | 选择输入模块根目录、设置 import path，并校验桌面运行使用的 Quest 模块来源 |

### 6.5 `session/`

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `__init__.py` | 包标记 | joystick session package |
| `state.py` | `SessionPhase`、`JoystickState` | 定义会话相位，保存设备、输入、双手、机械臂、transport、标定/灵敏度、设备状态项和 SSE sequence 等共享状态 |
| `arm.py` | `build_arm_bindings`、`ArmRuntime`、`BaseArmSession` | 创建 arm binding（同板双臂 index 0 左、index 1 右），装配末端、腰与臂型角运行时，管理各臂控制线程和 stop/home |
| `arm_angle.py` | `ArmAngleRuntime`、`ArmAngleInput`、`create_arm_angle_runtimes` | 按 SDK/APK/配置条件启用 D3 臂型角；由本臂控制线程以 20 Hz 步进，登记查询回复后跨周期读取 |
| `waist.py` | `WaistRuntime`、`FreshRobotState`、`create_waist_runtime` | 检测两轴腰，以独立控制线程处理头盔/摇杆方式、反馈新鲜度、跟随重触发和回位/停止 |
| `control_transaction.py` | `ControlTransaction`、`LoopTermination`、`ControlStepOutcome` | 按输入节拍或 72 Hz 固定节拍运行单臂采样、门禁、派发和终止回报；同周期调用本臂臂型角 tick |
| `control_stats.py` | `ControlLoopStats`、`percentile` | 按 5 秒窗口统计控制周期、步时、派发与输入频率、跳帧和发布到派发延迟；当前周期日志输出已注释 |
| `end_effector_sender.py` | `EndEffectorSender` | 在本臂控制线程内按值变化、0.5 秒保活或失败重试条件调用只发不等驱动；没有独立 worker 或队列 |
| `cleanup.py` | `SessionCleanup`、`CleanupContext`、`CleanupResult` | 按固定上下文停止机械臂和输入源，失败时保留原 owner 供同一清理事务重试 |
| `observation.py` | `ObservationSnapshot`、HTTP/SSE 投影函数 | 生成不可变 Host 状态快照，并分别投影为 HTTP 状态和 SSE 载荷 |
| `controller.py` | `JoystickController` | 遥操会话主控制器；拥有 source registry、共享状态、机械臂 session、生命周期门禁和清理编排 |
| `maintenance.py` | `ArmMaintenance` | 封装初始化位置确认、缓存和当前关节读取；对应 HTTP 维护路由在 Desktop 组合中不注册 |
| `direct.py` | `QuestDirectManager` | 板端自动维持 Quest 待命链路并处理头显会话请求；包含在投影中但 Desktop 不创建或启动它 |

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

## 7. 中立输入合同

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `include/carm_teleop_input_contracts.py` | `SourceCapabilities`、`ArmProfile`、`DeviceCandidate`、`SourceLease`、`RawInputBatch`、`HandIntentSample`、`BodyIntentSample`、`DeviceStatusItem`、`BaseInputSourceModule` | 定义输入模块与 Host 之间的冻结数据、功能结果、能力位、身份与 provider 接口 |
| `include/carm_teleop_intent.py` | `LegacyHandIntentAdapter` | 每手持有的旧按键到功能结果适配；累计回位事件序号，供未协商 intent 的旧 APK 使用 |

关键合同分为五组：

- 设备发现与能力：`SourceAvailability`、`DeviceCandidate`、`SourceCapabilities`、`ArmProfile`
- 连接身份：`SourceLease`、`ConnectOutcome`、`ConnectResult`
- 输入数据：`PoseSample`、`ButtonSample`、`JointTargetSample`、`HandInputUpdate`、`RawInputBatch`
- 功能与检测结果：`HandIntentSample`、`BodyIntentSample`、`WaistControlSample`（含头盔 `PoseSample`）、`InputUpdateDecision`
- Host 接口与遥测：`SourceEventSink`、`InputSourceModule`、`BaseInputSourceModule`、`DeviceStatusItem`

## 8. Quest 输入模块

| 文件 | 主要对象 | 职责 |
|---|---|---|
| `include/quest/__init__.py` | `QuestInputModule` 导出 | 声明 Quest module ID、API 版本并导出模块实现 |
| `include/quest/factory.py` | `create_provider` | 在 registry 首次需要时延迟构造 Quest provider |
| `include/quest/constants.py` | Quest 协议、能力与功能字段表 | 定义 TCP、超时、协议、包名与 `QUEST_INTENT_FIELDS`；核心手臂字段和可选腰/臂型角字段共用校验表 |
| `include/quest/adb.py` | ADB 辅助函数 | 执行 adb、解析设备列表、识别 Quest、读取设备信息、重启 ADB 服务和启动 Quest 包 |
| `include/quest/device_status.py` | `QuestDeviceStatusPoller`、`read_quest_device_status` | 每 15 秒经 ADB `dumpsys` 读取头盔与左右手柄电量，发布为设备状态项 |
| `include/quest/receiver.py` | `QuestTcpReceiver` | 监听 `127.0.0.1:9999`，处理 protocol 2 协商、输入帧、heartbeat、超时和连接 generation |
| `include/quest/parser.py` | `parse_quest_frame`、`parse_waist_control` | 把已校验的输入转换为左右手位姿、原始按钮、功能结果、头盔/机身输入与帧序号 |
| `include/quest/config.py` | `QuestPoseJumpConfig` | 解析并校验位置与旋转跳变阈值 |
| `include/quest/policy.py` | `QuestInputPolicy` | 输入线程独占相邻帧比较状态，输出跳变、旋转锁边沿与传输换代的断续事件，不读写控制器映射、不生成派发 token |
| `include/quest/module.py` | `QuestInputModule` | 继承 `BaseInputSourceModule`，声明能力位与 `arm_profile`；拥有 ADB reverse、receiver、serial、lease、重连 timer、电量轮询和 Quest 资源清理 |

Quest 文件之间的调用关系：

```text
QuestInputModule
  ├── adb.py       设备扫描、reverse、启动与停止
  ├── receiver.py  TCP 生命周期和协议
  ├── parser.py    输入帧解析
  ├── policy.py    输入接受、rebase 与派发门禁
  ├── config.py    位姿跳变配置
  ├── device_status.py 头盔与手柄电量轮询
  └── constants.py 端口、协议和超时
```

## 9. 维护约定

- README 只保留面向使用者的介绍和快速使用。
- 架构、数据流、状态、API 和源码职责放在 `docs/`。
- 文档中的文件、路由、字段、状态和功能必须能在当前仓库中找到对应实现。
- 新增、删除或移动源码后，应同步更新本文件树。
