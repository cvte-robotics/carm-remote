# CARM Quest Desktop

CARM Quest Desktop 是一个 Quest 手柄遥操桌面应用，可在图形界面中完成机械臂探测、Quest 连接、机械臂初始化、遥操状态查看和会话停止。

当前界面支持：

- A3 六轴单臂、A3 六轴双臂（A3_C / A3_T / A3_S 分型）
- D3 人形双臂
- 左右手柄数据显示切换
- 灵敏度（0.1–2.0）和 Yaw 标定角度（-180°–180°）调整
- Quest 头盔与左右手柄电量显示
- 手柄位置、按钮数据和运行日志查看

## 运行要求

### 系统环境

| 项目 | 要求 |
|---|---|
| 操作系统 | Windows 或 Linux |
| Python | 3.11 或 3.12 |
| ADB | `adb` 命令已加入系统 `PATH` |
| Quest | 已开启调试并完成 ADB 授权 |
| 机械臂 | 与运行本程序的电脑网络连通 |
| Python 包下载 | 首次运行时能够访问配置的软件包下载源 |

Quest 中需要安装能够与本程序连接的应用。连接前可运行以下命令确认设备状态：

```text
adb devices -l
```

Quest 对应行的状态应为 `device`。

### 网络端口

| 端口 | 用途 |
|---|---|
| `127.0.0.1:1999` | 桌面界面与本机后端通信 |
| `127.0.0.1:9999` | 接收 Quest 输入数据 |
| 机械臂 IP 的 `8090` | 连接机械臂 |

启动前请确保本机端口 `1999` 和 `9999` 未被其他程序占用。

## Python 依赖

依赖版本以 [qt_app/requirements.txt](qt_app/requirements.txt) 和 [qt_app/pyproject.toml](qt_app/pyproject.toml) 为准。

| 依赖 | 版本 | 用途 |
|---|---|---|
| PyQt6 | `>=6.7,<7` | 桌面界面和本机数据通信 |
| Flask | `>=2.2,<4` | 本机后台接口 |
| Werkzeug | `>=2.2,<4` | 运行本机后端服务 |
| carm | `>=0.1.20260819` | 机械臂连接与控制 |
| NumPy | `==1.26.4` | 位姿和矩阵计算 |
| SciPy | `==1.11.4` | 旋转与四元数计算 |
| websocket-client | `>=1.6,<2` | 启动检查要求的运行依赖 |
| zeroconf | `>=0.130,<1` | 启动检查要求的运行依赖 |

无需提前手动安装这些 Python 包。启动脚本会创建独立虚拟环境，并在依赖缺失时自动执行安装：

- Windows：`qt_app/.venv-windows`
- Linux：`qt_app/.venv-linux`

## 快速开始

### Windows

1. 安装 Python 3.11 或 3.12，并确认 `python` 命令可用。
2. 安装 ADB，并确认 `adb` 命令可用。
3. 在仓库根目录打开 PowerShell。
4. 运行：

```powershell
.\run_windows.ps1
```

### Linux

1. 安装 Python 3.11 或 3.12，并确保支持创建 venv。
2. 安装 ADB。
3. 在仓库根目录运行：

```bash
./run_linux.sh
```

Linux 启动脚本默认使用 `python3`。需要指定其他解释器时，可设置 `PYTHON`：

```bash
PYTHON=/path/to/python3 ./run_linux.sh
```

### 选择末端实现

默认使用 Carm 原生末端控制：

```text
carm
```

使用 EtherCAT/CAN-FD 六指透传时，在启动时显式选择 `passthrough`。

Windows：

```powershell
.\run_windows.ps1 --end-effector passthrough
```

PowerShell 风格的 `.\run_windows.ps1 -EndEffector passthrough` 也受支持。

Linux：

```bash
./run_linux.sh --end-effector passthrough
```

末端名称按包内模块动态导入；名称必须是合法 Python 标识符格式，模块不存在或实现不符合末端合同时，启动会明确失败，不会静默回退。

### 只检查运行环境

Windows：

```powershell
.\run_windows.ps1 -PreflightOnly
```

Linux：

```bash
./run_linux.sh --preflight-only
```

检查内容包括 Python 版本、Python 依赖、Carm 包版本、ADB、运行端口和仓库运行文件。

### 采集机械臂计划数据

`qt_app/scripts/collect_data.py` 用于在遥操过程中按固定间隔采集机械臂和末端的计划位置、速度和力矩，并写入 CSV。脚本会先连接并校验指定型号的机械臂；结束时按 `Ctrl+C`，或使用 `--duration` / `--count` 设置采集边界。

在仓库根目录执行时，建议使用 Qt 应用创建的虚拟环境中的 Python：

```powershell
qt_app\.venv-windows\Scripts\python.exe qt_app\scripts\collect_data.py --single-a3 --ip 192.168.1.10 --duration 30
```

支持的连接方式如下：

| 模式 | 必需参数 |
|---|---|
| A3 六轴单臂 | `--single-a3 --ip <控制器 IP>` |
| A3 六轴双臂 | `--dual-a3 --left-ip <左臂 IP> --right-ip <右臂 IP>` |
| D3 七轴双臂 | `--dual-d3 --ip <控制器 IP>`，可用 `--left-index` 和 `--right-index` 指定臂索引 |

默认每 `100` ms 采集一帧，输出到当前执行目录下带时间戳的 `collect_data_*.csv`。可通过 `--interval-ms`、`--duration`、`--count` 和 `--output` 调整采样间隔、时长、帧数和输出路径。CSV 中的计划数据字段以 JSON 字符串保存；双臂模式会分别使用 `left` 和 `right` 作为 `arm` 字段值。

## 使用流程

1. 输入当前机械臂的 IP 地址。
2. 点击“扫描设备”。扫描时会读取机械臂型号，并据此列出可选的遥操类型。
3. 根据扫描结果选择遥操类型。
4. 按界面提示设置当前机械臂侧别；A3 双臂模式还需填写另一臂 IP。D3 人形双臂固定由左手柄控制左臂（索引 0）、右手柄控制右臂（索引 1），侧别选项不影响绑定。
5. 双击扫描到的 Quest，等待连接完成。连接后状态行右侧显示头盔和左右手柄电量，离线的手柄以灰色显示。
6. 点击“初始化机械臂”。
7. 根据需要调整手柄数据显示侧、灵敏度和 Yaw 标定角度。Yaw 为手柄（头盔）与机械臂 X 轴的夹角：0° 为前，+90° 为左，-90° 为右，默认 -90°。
8. 使用完成后，点击“停止手柄遥操”。

后端同一时间只执行一个用户操作（扫描、连接、初始化、停止等）；操作进行中再次提交会被直接拒绝，不会排队。

## 配置

运行配置位于 [config/joystick_config.json](config/joystick_config.json)，包括：

- Quest 位姿变化阈值（`quest_pose_jump`）
- 按机械臂型号划分的初始化关节位置（`models.<型号>.<模式>[.<臂侧>].init_joint`）：
  - `A3_C`、`A3_T`、`A3_S`：`single` 单臂，以及 `dual.left` / `dual.right` 双臂，各 6 个关节
  - `D3`：仅 `dual.left` / `dual.right`，各 7 个关节

机械臂型号由扫描时读取的名称和自由度识别：名称须唯一包含 `A3` 或 `D3`，A3 还须唯一包含 `C`、`T`、`S` 之一；A3 为 6 轴，D3 为 7 轴。识别失败或轴数不符时扫描会报错。

旧版的 `single_arm` / `dual_arm`（`six_axis` / `seven_axis`）格式已不再支持，升级时请按上述结构迁移自定义配置。

配置在后端启动时读取。修改后需要重新启动应用。

## 项目结构

```text
config/                     遥操运行配置
docs/                       架构、数据流、API 和源码说明
include/                    输入合同与 Quest 输入模块
qt_app/                     PyQt6 桌面界面、启动脚本和启动检查
server/api/                 本机数据接口
server/desktop/             桌面后端入口与生命周期管理
server/basic/joystick/      遥操会话、控制算法和输入注册
```

## 常见问题

### 找不到 Quest

运行 `adb devices -l`。如果设备显示为未授权，请在 Quest 中确认调试授权；如果设备不在列表中，请检查 ADB 和设备连接。

### 启动时提示端口被占用

关闭其他正在运行的 CARM Quest Desktop 实例，并确认本机端口 `1999` 和 `9999` 可用。

### Python 依赖安装失败

确认当前网络可以访问 Python 软件包下载源，然后重新运行根目录启动脚本。启动脚本会再次检查依赖。

### 如何查看日志

界面会显示实际日志目录。点击目录按钮即可打开。

## 文档

- [整体架构、数据流与 API](docs/ARCHITECTURE.md)
- [源码文件树与职责](docs/SOURCE_SNAPSHOT.md)
- [新增末端控制方式](docs/END_EFFECTOR_DEVELOPMENT.md)

## 许可证

本项目使用 [MIT License](LICENSE)。Carm SDK、Quest 应用和机械臂端软件不属于本仓库，分别遵循其各自的授权条款。

## 安全提示

操作机械臂前，请确认设备周围环境安全，并确保随时可以停止设备。首次使用或修改配置后，应先在可控环境中验证 Quest 连接、机械臂初始化、遥操和停止行为。
