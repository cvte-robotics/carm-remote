# CARM Quest Desktop 整体架构

本文面向维护者，描述当前仓库中可以从源码确认的运行结构、模块边界、数据流、状态管理、Quest 输入链、机械臂控制链和 HTTP/SSE API。

文档同步基线：后端 `master.dev` 的 `1d1c103`（2026-11-10）经公开白名单导出的桌面源码，并包含本次桌面依赖声明与启动探测清理。根目录 README、源码索引与末端扩展说明使用同一基线。桌面入口只启用 Quest provider；虽然导出目录包含 `QuestDirectManager` 与配置模块，Desktop 不创建或启动板端直连管理器，连接与初始化仍由 Qt 工作流发起。

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
    Session --> Transaction[ControlTransaction]
    Transaction --> Command[CommandController]
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
| 实时事务 | `server/basic/joystick/session/control_transaction.py` | 单个 ArmRuntime 的输入采样、策略门禁、物理派发和终止回报 |
| 输入信箱 | `server/basic/joystick/core/mailbox.py` | 双手同批的单槽 publication、冻结输入帧和输入发布唤醒信号 |
| 末端发送 | `server/basic/joystick/session/end_effector_sender.py` | 每臂一个 `EndEffectorSender`，在本臂控制线程内去重、保活并调用只发不等驱动 |
| 腰与臂型角 | `session/waist.py`、`session/arm_angle.py` | 腰独立控制线程；臂型角由本臂控制线程步进 |
| 共用映射 | `server/basic/joystick/core/pose_mapping.py` | 手臂与头盔腰共用的增量位姿基线与连续四元数分支 |
| SDK 连接 | `server/basic/joystick/runtime/carm_link.py` | 同一控制端点共用一条 WebSocket，各 arm_index 使用独立通道视图 |
| 运行统计 | `server/basic/joystick/session/control_stats.py` | 按 5 秒窗口汇总控制周期、步时、派发和输入频率 |
| 清理事务 | `server/basic/joystick/session/cleanup.py` | 冻结清理身份、编排 arm/provider 清理并保留失败 owner 供重试 |
| 状态投影 | `server/basic/joystick/session/observation.py` | 生成不可变 Host 快照，并分别投影 HTTP 和 SSE 兼容载荷 |
| 输入注册 | `server/basic/joystick/sources/registry.py` | 设备扫描身份、source lease 和模块生命周期 |
| Quest 适配 | `include/quest/` | ADB 扫描、TCP 协议、输入解析、重连、设备电量读取和资源回收 |
| 机械臂会话 | `server/basic/joystick/session/arm.py` | 单臂/双臂 binding、初始化、控制线程和停止 |
| 控制计算 | `server/basic/joystick/core/` | 坐标变换、目标计算、命令门禁和末端命令 |
| 输入合同 | `include/carm_teleop_input_contracts.py` | provider 能力位、设备、lease、输入批次、设备状态项和传输事件的数据结构 |

组合关系由 `server/desktop/app.py` 建立：`DesktopRuntime` 创建一个 `JoystickController`，随后将同一个 controller 注入所有 joystick 路由。

## 3. 启动流程

根目录脚本只负责转发参数：

- Windows：`run_windows.ps1` 调用 `qt_app/run_windows.ps1`
- Linux：`run_linux.sh` 调用 `qt_app/run_linux.sh`

平台脚本依次执行：

1. 检查 Python 版本是否在 3.8 – 3.12 之间。
2. 创建 `qt_app/.venv-windows` 或 `qt_app/.venv-linux`。
3. 检查依赖，缺失时从 `qt_app/requirements.txt` 安装。
4. 执行 `qt_app/preflight.py`。
5. 从仓库根目录运行 `qt_app/main.py`。

`preflight.py` 检查 Python 依赖、Carm 包版本、ADB 命令、端口 `1999` 和 `9999`、运行所需文件以及 Quest 模块导入。

位姿计算只使用标准库。`requirements.txt`、`pyproject.toml`、Windows/Linux 启动脚本的依赖探测与 preflight 已统一移除 NumPy/SciPy；启动时不会因缺少这两个包而重复进入安装分支。

`qt_app/main.py` 加载包内可用的末端实现，创建 `QApplication`、配置日志、再次执行启动检查，然后创建 `MainWindow`。`ViewModel.start()` 通过 `BackendProcess` 启动：

```text
<当前虚拟环境 Python> -m server.desktop.app --end-effector <name>
```

默认模式是 `carm`。Windows 包装器接受 `--end-effector` 和 `-EndEffector`，Linux 包装器原样转发 `--end-effector`。启动时会从 `server/basic/joystick/end_effectors/` 按名称动态加载包内实现；当前仓库提供 `carm`、`passthrough`，以及不控制硬件的 `blank` 开发示例。名称只受 Python 标识符格式约束；模块不存在或实现不符合末端合同时，启动会明确失败。

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

扫描先调用 `probe_current_arm()` 读取当前机械臂的 `arm_name` 和 `arm_dof`，由 `identify_arm_model()` 识别型号，再扫描 ADB 设备：

| 探测结果 | 识别型号 | 界面可选模式 |
|---|---|---|
| 名称唯一包含 `A3`，且唯一包含 `C`/`T`/`S` 之一，自由度为 6 | `A3_C` / `A3_T` / `A3_S` | `a3_single`、`a3_dual` |
| 名称唯一包含 `D3`，自由度为 7 | `D3` | `d3_dual` |

名称同时包含 `A3` 和 `D3`、A3 分型缺失或不唯一、自由度与型号不符时，扫描直接失败。连接机械臂时同样按型号识别，并用 `型号 -> 模式 -> 臂侧` 解析初始化关节配置；D3 只支持双臂。

单臂只创建 master binding。双臂创建 master 和 slave binding：

- 两个 IP 及其控制 endpoint 都不同时，分别连接两个管理 IP；master 绑定界面选择的手柄侧，slave 绑定另一侧。
- 两个 IP 相同或解析到同一控制 endpoint 时视为同板双臂（D3 人形双臂）：master 使用 `arm_index=0` 并固定绑定左手柄，slave 使用 `arm_index=1` 并固定绑定右手柄，忽略界面选择的手柄侧。
- 桌面端所有机械臂都通过管理 IP 的 `8090` 端口连接。

每个 ArmRuntime 在 Carm 连接准备完成后创建自己的末端 Driver。输入源、单/双臂和同/异板拓扑不决定末端实现：`carm` 等价封装原生 flange/gripper/hand，`passthrough` 将一个归一化 trigger 映射为六指相同目标，并依次发送六个 `mode=0` CAN-FD 帧。

Driver 由每臂一个 `EndEffectorSender` 包装：位姿派发成功并通过连接身份复核后，在同一个控制线程内按值变化、0.5 秒保活或失败重试条件调用驱动。原生末端使用 `sdk_nowait()`，透传使用 `send_only()`；没有独立末端线程或待发队列。自定义驱动也必须只发不等，避免拖慢位姿和臂型角控制；返回成功只表示驱动定义的提交结果，不表示实体已经完成动作。

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

`JoystickController.user_operation()` 对用户操作做排他准入：同一时间只允许一个操作执行，另一个操作正在进行时新请求直接失败（“另一个遥操操作正在执行”），不排队。各操作还受会话相位约束：

| 操作 | 允许的会话相位 |
|---|---|
| `scan`、`connect` | `IDLE` |
| `init_arm` | `CONNECTED` |
| `session_config` | `IDLE`、`CONNECTED` |
| `disconnect` | `IDLE`、`CONNECTED`、`READY`、`CLEANUP_FAILED` |
| 其他设置（yaw、灵敏度、数据显示侧等） | `IDLE`、`CONNECTED`、`READY` |

`connect` 还要求地址来自本次扫描结果，否则返回“请先扫描并选择设备”。

### 5.2 Quest 输入流

```text
Quest TCP 帧
  -> QuestTcpReceiver
  -> QuestInputModule
  -> RawInputBatch（同一帧内的左右手数据）
  -> JoystickController
  -> QuestInputPolicy
  -> 双手同批的 InputPublication 单槽 + 发布信号（PublicationSignal）
  -> 对应 ArmRuntime 的 ControlTransaction
  -> CommandController 目标计算
```

`QuestTcpReceiver` 接收换行分隔的 JSON 记录。连接必须先完成 protocol 2 的 `hello` / `hello_ack` 协商，之后才接收位姿帧和 heartbeat。

`QuestInputModule` 将一帧输入转换成不可变的 `RawInputBatch`。批次带有 source lease、transport generation 和 batch sequence，用来阻止旧连接的数据进入当前会话。

接收边界校验输入帧，`QuestInputPolicy` 比较相邻输入、旋转锁边沿和传输世代，输出接收决策与断续事件。它不再生成派发 token，也不读写控制器的映射基线；断续计数交给控制线程消费。

通过检查的左右手数据与机身输入封装成同一个冻结 `InputPublication`，一次写入最新值单槽；新批次覆盖旧批次，控制环每步只读一次 publication。写入后 `PublicationSignal` 唤醒等待新输入的控制环；信号只做唤醒，不携带数据。每手再接入状态仍分别保存。

### 5.3 机械臂控制流

```text
ControlTransaction 输入快照与策略门禁
  -> CommandController 目标计算
  -> 坐标变换与灵敏度
  -> 生成相对位姿目标
  -> 校验输入/控制/连接代际
  -> Carm.dispatch_pose_for_connection()
  -> Carm.track_pose()
  -> 当前机械臂连接
```

每个机械臂对应一个 `CommandController`、一个 `ControlTransaction` 和一个控制线程。`ControlTransaction` 拥有完整实时循环；`CommandController` 负责目标计算和单臂操作，不拥有循环线程。

`CommandController` 的位姿映射与回位状态只由所属控制线程推进。手臂与头盔腰共用 `PoseMapping`：目标由输出基线叠加输入增量；手臂仅对位置应用灵敏度，姿态按 1:1 映射，头盔腰的位置和姿态均应用灵敏度。重建基线时结合当前是否仍在跟踪、最近目标和新鲜反馈，避免重接入时跳回旧目标。

同一控制端点默认由 `CarmLinkRegistry` 共用一条 WebSocket 和一个接收线程，各机械臂及腰通过 `CarmChannel` 按 `arm_index` 路由。设置 `sdk.shared_connection=false` 可改用独立 SDK 连接。控制周期内运动类命令只发不等；臂型角查询登记回复槽后由后续周期取结果，初始化和清理仍保留必要的同步确认。

控制循环的节拍由输入源能力位 `input_clocked_control` 决定，周期常量为 `control_transaction.py` 中的 `_CONTROL_LOOP_HZ = 72.0`：

- Quest 声明了该能力位：每次输入发布后立即唤醒控制环；步内已有新帧时直接进入下一步，没有新帧时最长等待 1/72 秒兜底，用于反馈刷新、运动开关释放和停止检查。输入策略与控制事务的帧屏障保证每帧最多派发一次。
- 未声明的输入源按 72 Hz 绝对时间节拍运行，避免“执行后再 sleep”造成的周期漂移；落后时不追赶。

`ControlLoopStats` 在控制线程内按 5 秒窗口统计控制周期、步时、派发与输入频率、跳帧数和“帧发布到物理派发”延迟，不参与控制判断；当前源码中它的周期日志输出已被注释，不会写入日志。末端发送器的 `[EndEffectorStats]` 统计仍按 5 秒窗口写入后端日志。

控制代码消费 `HandIntentSample`，原始按钮保留作兼容观测。新版 APK 的绑定与按住/切换方式由头显决定：

| 输入 | 当前行为 |
|---|---|
| `motion_engaged=true` | 允许生成位姿目标；重新进入跟随时建立映射基线 |
| `motion_engaged=false` | 清除待发目标，下一次开启时重新同步 |
| `end_effector` | 交给启动时选择的末端实现；原生按 SDK 上下限插值，透传映射为六个 `0..1000` 目标 |
| 新的 `home_sequence` 事件且运动关闭 | 下发 `teleop_reset_joint`，由任务结束及位置反馈判定到位；不在忙碌期间排队补发 |
| `rotation_locked` 变化 | 更新旋转锁并重建映射基线 |

未协商 `intent_v1` 的旧 APK 由每手 `LegacyHandIntentAdapter` 转换：Grip 大于 100 为运动开启，B/Y 为旋转锁状态，Trigger 截断到 `0..127`，A/X 上升沿累计为回初始位事件。同 lease 的传输重连只重建事件基线，不把旧事件当作新动作。

末端实现由启动参数选择，并在机械臂准备阶段创建：

- `carm`：根据 Carm SDK 的末端配置执行原生末端控制；flange 或零自由度时不发送末端命令。
- `passthrough`：将触发值映射为六指目标，依次发送六个 `mode=0` CAN-FD 帧。
- 其他包内实现：由其 `EndEffectorBase` 子类负责参数校验和派发语义。

末端实现不再由核心控制代码写死。新增实现放入 `server/basic/joystick/end_effectors/`，定义唯一的 `NAME` 和 `set_eff`，由启动时的包扫描发现。完整扩展合同与验证方法见 [新增末端控制方式](END_EFFECTOR_DEVELOPMENT.md)。

### 5.4 腰与臂型角

腰控制仅在同板 D3 双臂、配置启用、APK 已协商 `waist_control_v1` 与 `intent_v1` 且声明 `wf`/`wh`/`ws`、新鲜硬件反馈确认存在两轴腰时启用。明确无腰或旧 APK 仅禁用腰；反馈缺失、异常或超时不能当作无腰，初始化会报错。

`WaistRuntime` 使用 `arm_index=2`，有独立的按输入帧唤醒线程。`wm` 选择 `head` / `stick`：头盔方式经共用位姿映射调用 `track_pose`，摇杆方式按死区和角速度积分生成关节目标后调用 `track_joint`。摇杆方式先回中再跟随；输入/反馈中断、重定位、模式或传输身份变化后需重新触发跟随。回位事件使用 `teleop_reset_joint`；`home` 停止使用 `home_joint`，`hold` 不回零。

臂型角仅在同板 D3 双臂、SDK 提供 `get_arm_angle` / `set_arm_angle`、APK 声明 `ia` / `lia` 且配置启用时生效；不满足条件则发布未启用原因，双臂继续遥操。每臂 `ArmAngleRuntime.tick()` 由本臂控制线程调用，以 20 Hz 步进，不创建线程。输入需先回中，运动功能开启且近期确有位姿派发时才调节；松键后异步读取并对齐实际臂型角。`arm_angle_state` 回传 Quest，`angle_deg` / `target_deg` 当前恒为 `null`，Qt 不显示实际角度。

### 5.5 状态展示流

```text
JoystickState
  -> JoystickController.get_observation_snapshot()
  -> project_sse_payload()
  -> /api/joystick/events
  -> SseClient
  -> ViewModel._event()
  -> MainWindow.render_snapshot()
```

SSE 每 50 ms 检查一次观察序列号（observation sequence）。序列变化时立即发送快照；连续 3 秒无变化时发送 heartbeat 帧。Qt 当前将顶层 `pose` 和 `button` 展示在界面中，在连接期间把 `status.device_status` 渲染到状态行右侧，回填 `status.yaw` 到标定滑条，并根据 transport 和 terminal 状态更新提示。

D3 模式还展示 `status.waist` 中的模式、跟随状态、未启用原因和错误；停止期间不会把最后一帧继续显示为跟随中。

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

手动停止时，Qt 先关闭本地 SSE 订阅，再请求 `home` 清理。若后端因 Carm reader 或其他 owner 尚未释放而返回失败，界面保持 `Stopping`，使用同一 workflow generation 按 1 秒、2 秒、5 秒上限继续请求清理；只有后端确认全部 owner 已释放后才进入 `Idle`。

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

`hello.capabilities` 可协商 `intent_v1`、`session_setup_v1` 和 `waist_control_v1`。`hello.intents` 声明具体功能字段；旧 `intent_v1` APK 未上传该声明时仅默认核心手臂字段，新功能不会被隐式开启。

| 功能 | 线字段 | 后端语义 |
|---|---|---|
| 右手核心功能 | `im` / `il` / `ie` / `ih` | 运动开关、旋转锁、末端值、回初始位事件 |
| 左手核心功能 | `lim` / `lil` / `lie` / `lih` | 同上，作用于左手 |
| 腰功能 | `wf` / `wh` / `ws`，配合 `wm` | 跟随事件、回位事件、摇杆、头盔/摇杆方式 |
| 臂型角 | `ia` / `lia` | 右/左手臂型角摇杆输入 |

模块将事件线计数转换为 lease 内累计序号，传输重连时保留累计值并重建基线。`session_setup_v1` 的板端待命与头显发起会话由 Board 组合启用；Desktop 不启动该管理器，不能据此省略 Qt 扫描和连接流程。

普通传输断开进入 5 秒恢复窗口。协议错误、无效输入帧或 listener 错误属于 terminal 错误，不进入普通恢复路径。

### 7.1 设备状态

OpenXR 运行时不向应用暴露手柄电量，因此 Quest 连接就绪后由 `QuestDeviceStatusPoller` 每 15 秒通过 ADB 读取一次（单次命令超时 5 秒）：

| 状态项 `id` | 中文标签 | 来源 |
|---|---|---|
| `headset_battery` | 头盔 | `dumpsys battery` 的 `level` / `scale` |
| `left_controller_battery` | 左手柄 | `dumpsys OVRRemoteService` 的 `Battery` |
| `right_controller_battery` | 右手柄 | `dumpsys OVRRemoteService` 的 `Battery` |

单位均为 `%`。`ExternalStatus` 不是 `CONNECTED*` 的手柄标记为 `stale`；读不到的项值为 `null`，不报错。轮询随 source lease 摘除而停止，迟到的发布由身份复核丢弃；会话断开或停止时 Host 清空状态项。

## 8. 停止与清理

停止模式只有 `home` 和 `hold`：

| 模式 | 行为 |
|---|---|
| `home` | 停止控制线程后，将机械臂及已启用的腰移动到各自配置的 `home_joint`，再断开 |
| `hold` | 停止控制线程、清除目标并断开，不发送回零运动 |

界面“停止手柄遥操”先关闭本地 SSE 订阅，再请求 `home`。如果输入已经断开，controller 会将 `home` 降级为 `hold`。窗口关闭请求 `hold`，桌面 heartbeat 超时和后端退出也固定执行 `hold`。

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
| `POST /api/joystick/connect` | `address`、可选 `name`、`_desktop_generation` | `data.outcome`：`ready`、`pending` 或 `failed` | 连接扫描到的 Quest |
| `POST /api/joystick/init_arm` | `mode`、`master_ip`、`slave_ip`、`master_side`、`_desktop_generation` | 无额外数据 | 创建并初始化机械臂会话及符合条件的腰、臂型角运行时 |
| `POST /api/joystick/yaw` | `value` | 无额外数据 | 设置 Yaw 标定角度，`-180..180` 的整数；0 为前，+90 为左，-90 为右 |
| `POST /api/joystick/calibration` | `value` | 无额外数据 | 兼容旧版四档标定，`0..3` 分别映射为 yaw `+90`、`-90`、`0`、`180` |
| `POST /api/joystick/scale` | `value` | 无额外数据 | 设置灵敏度，范围 `0.1..2.0` |
| `GET /api/joystick/move_speed` | 无 | `data.level`、`data.speed` | 读取关节运动速度档位 |
| `POST /api/joystick/move_speed` | `value`、`_desktop_generation` | 无额外数据 | 设置 `low` / `medium` / `high`，Qt 暂无对应控件 |
| `POST /api/joystick/view_side` | `value` | 无额外数据 | 设置兼容视图侧，`left` 或 `right` |
| `POST /api/joystick/disconnect` | `stop_mode`、`reason`、`_desktop_generation` | `data.status` 状态对象 | 停止当前会话 |
| `GET /api/joystick/status` | 无 | `data` 状态对象 | 获取当前状态快照 |
| `GET /api/joystick/events` | 无 | SSE 数据流 | 订阅状态和输入快照 |

设置类 POST 在 Desktop 组合中同样需要有效的 `_desktop_generation`。速度默认 `medium`，三档对应 SDK `speed=20/50/80`；旧 SDK 的 `move_joint` 无 `speed` 参数时分别使用 `desire_time=5/2/-1`。该接口控制手臂关节运动档位，不改变位姿灵敏度、腰速度或臂型角速度。Desktop 不注册 `include_maintenance_routes` 控制的初始化位置维护与关节读取接口。

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
- `yaw`（当前标定角度）、`calibration`（yaw 恰为四档值时为 `0..3`，否则为 `null`）、`scale`
- `device_status`：输入源声明的设备状态项列表，见 [7.1 设备状态](#71-设备状态)
- `source_type`、`is_quest_device`
- `mode`、`master_side`、`view_side`
- `capabilities`：当前输入源的能力位；`arm_profile`：当前输入源支持的型号与模式
- `master_arm`、`slave_arm`
- 当前 view side 的 `pose`、`button`
- 当前 view side 的 `intent`：运动、旋转锁、末端、回位累计序号与臂型角摇杆
- `waist`：腰模式、状态、跟随标记及错误；`head_pose` / `body_intent`：头盔腰输入和腰功能结果
- `direct`：板端直连状态结构；Desktop 不启动直连管理器，其 `active` 保持 `false`
- `transport.state`、`protocol`、`connection_id`、`last_rx_age_ms`、`disconnect_reason`
- controller 不可用时附带 `available: false`

`device_status` 每一项的结构：

```json
{
  "id": "headset_battery",
  "label": {"zh-CN": "头盔", "en-US": "Headset"},
  "value": 82,
  "unit": "%",
  "stale": false
}
```

`label` 和文本型 `value` 以 `{语言: 文本}` 给出，至少包含 `zh-CN` 和 `en-US`；数值、布尔和单位与语言无关。Qt 界面取 `zh-CN`。

## 10. SSE 契约

订阅：

```http
GET /api/joystick/events
Accept: text/event-stream
```

变化帧的结构（机械臂绑定、直连配置等对象省略内部细节）：

```json
{
  "sequence": 1,
  "pose": null,
  "button": null,
  "intent": null,
  "head_pose": null,
  "body_intent": null,
  "direct": {"active": false},
  "status": {
    "ble_connected": true,
    "status": "connected",
    "calibration": 1,
    "yaw": -90,
    "scale": 0.4,
    "arm_initialized": false,
    "device_status": [],
    "waist": {"status": "unavailable", "following": false, "error": null},
    "source_type": "quest",
    "mode": "single",
    "master_side": "right",
    "capabilities": {
      "provides_pose": true,
      "provides_buttons": true,
      "can_control_arms": true,
      "connect_only": false,
      "supports_reconnect_grace": true,
      "provides_joint_targets": false,
      "supports_per_arm_switch": false,
      "configures_arm_session": true,
      "supports_reengagement": false,
      "input_clocked_control": true
    },
    "arm_profile": {
      "models": ["A3", "D3"],
      "dof": null,
      "modes": ["a3_dual", "a3_single", "d3_dual"],
      "require_same_board_dual": false,
      "device_index_by_hand": {}
    }
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
      "intent": null
    },
    "right": {
      "pose": null,
      "button": null,
      "intent": null
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

顶层 `intent` 与视角手的 `hands.<side>.intent` 一致，包含 `motion_engaged`、`rotation_locked`、`end_effector`、`home_sequence`、`arm_angle_stick`，功能结果与原始 `button` 分开发布。`head_pose` 含 `mode`、七分量位姿、`valid`、`recenter_count`；`body_intent` 含两个腰事件累计序号和 `waist_stick`。无相应输入时为 `null`。`status.waist` 发布腰状态，运行时还带 `mode`；`direct` 保留板端直连状态结构，但 Desktop 的 `active` 为 `false`。

未连接输入源时 `capabilities` 和 `arm_profile` 为 `null`。`hands` 中只有输入源提供关节目标时才出现 `joint`。

连续 3 秒没有新的 sequence 时，服务发送 `heartbeat: true`，并将该帧顶层 `pose`、`button`、`intent`、`head_pose`、`body_intent` 设为 `null`。会话终止时 `stream_terminal=true`；需要发送最终断连帧时 `emit_disconnect_frame=true`，发送后关闭 SSE 流。

## 11. 配置与日志

`config/joystick_config.json` 在后端导入 joystick runtime 时读取一次，运行期间不自动重新加载。配置内容包括：

- `quest_pose_jump.position_threshold_m`
- `quest_pose_jump.rotation_threshold_deg`
- `models.<型号>.single.init_joint`：`A3_C`、`A3_T`、`A3_S` 的单臂初始化关节
- `models.<型号>.dual.left|right.init_joint`：各型号双臂左右侧初始化关节；`D3` 只有 `dual`
- 各臂配置同层的 `home_joint` 与 `teleop_reset_joint`：停止回零与遥操中回位目标，省略时分别为全零与 `init_joint`
- `models.D3.waist`：`enabled`、两轴 `init_joint` / `teleop_reset_joint` / `home_joint`、`max_speed_deg_s`、`deadzone`
- `models.D3.arm_angle`：`enabled`、`deadzone`、`max_speed_deg_s`、`min_deg` / `max_deg`、`min_step_deg`
- 可选 `sdk.shared_connection`：默认 `true`

关节目标使用弧度；角度范围和角速度以字段中的 `_deg` / `_deg_s` 为单位。当前配置开启 D3 腰和臂型角，但硬件与 APK 准入仍逐次检查。腰目标默认均为 `[0, 0]`，摇杆死区 `0.15`、最大速度 `10°/s`；臂型角死区 `0.15`、最大速度 `45°/s`、范围 `-170..170°`、最小下发步长 `0.2°`。腰与手臂共用灵敏度和 Yaw，旧腰 `scale` 字段已被忽略。

`JoystickConfigSnapshot.resolve_arm(model, mode, side)` 按 `型号 -> 模式 -> 臂侧` 解析并校验数值与数组长度。旧版 `single_arm` / `dual_arm`（`six_axis` / `seven_axis`）结构不再被读取。

日志分为：

- `desktop-ui.log`：Qt 工作流、HTTP/SSE 和界面异常。
- `desktop-backend.log`：后端、Quest、控制和清理过程。

日志使用 5 MiB 轮转文件并保留 3 个备份。实际目录由 `qt_app/app/runtime_paths.py` 选择，并显示在界面中。

## 12. 代码事实与验证边界

本文中的端口、状态、字段、超时和调用关系来自当前源码与配置。启动前检查只执行软件环境和导入检查；Quest 连接、机械臂初始化、控制运动以及 `home` / `hold` 的物理结果仍需要在相应设备上验证。
