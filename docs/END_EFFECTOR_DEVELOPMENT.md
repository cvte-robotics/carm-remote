# 新增末端控制方式

本文面向需要为遥操链路接入新末端设备的开发者，说明末端实现的扩展合同、加载方式、调用时机和验证方法。

## 1. 扩展边界

末端实现位于 `server/basic/joystick/end_effectors/`。启动参数 `--end-effector <name>` 选择一个实现；系统为每个 `ArmRuntime` 创建一个实例，并向它提供当前机械臂连接、绑定手侧和主从臂角色。

新增末端方式通常只需要增加一个 Python 模块，不需要修改 `CommandController`、`ArmSession`、Qt 界面或启动脚本。现有实现可作为参考：

| 模块 | 用途 |
|---|---|
| `carm.py` | 通过 Carm SDK 的 `set_eeff` 控制原生 flange、gripper 或 hand |
| `passthrough.py` | 通过现有 Carm 连接发送六指 EtherCAT/CAN-FD 透传帧 |
| `blank.py` | 最小开发示例 |

## 2. 实现约束

每个实现必须满足以下约束：

1. 模块放在 `server/basic/joystick/end_effectors/` 下。
2. 模块文件名与实现名一致，例如 `vacuum.py` 对应 `NAME = "vacuum"`。
3. 类继承 `EndEffectorBase`，并定义唯一的 `NAME`。

   ```python
   from . import EndEffectorBase, EndEffectorContext

   class VacuumEffector(EndEffectorBase):
       NAME = "vacuum"
   ```

4. 构造函数接收 `EndEffectorContext` 和可选的 `**options`。

   ```python
   from . import EndEffectorBase, EndEffectorContext

   class VacuumEffector(EndEffectorBase):
       NAME = "vacuum"

       def __init__(self, context: EndEffectorContext, **options):
           super().__init__(context, **options)
           self._sdk = context.carm
   ```

5. 实现 `set_eff(trigger)`，返回严格的布尔值。

   ```python
   from . import EndEffectorBase, EndEffectorContext

   class VacuumEffector(EndEffectorBase):
       NAME = "vacuum"

        def __init__(self, context: EndEffectorContext, **options):
           super().__init__(context, **options)
           self._sdk = context.carm

       def set_eff(self, trigger):
           if not isinstance(trigger, (int, float)):
               return False
           value = max(0, min(127, int(trigger)))
           self._sdk.send_end_effector(value)
           return True
   ```

名称会转为小写，并且必须匹配 `[a-z][a-z0-9_]*`。以下名称合法：`vacuum`、`two_finger`；`two-finger`、`2finger` 不合法。

```python
    name = str(getattr(cls, "NAME", "") or "").strip().lower()
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise TypeError("end-effector subclass must define a valid NAME")
```

`EndEffectorContext` 提供：
| 字段 | 含义 |
|---|---|
| `carm` | 当前 `ArmRuntime` 已建立连接的 Carm 实例 |
| `side` | 当前机械臂绑定的手侧，`left` 或 `right` |
| `role` | 当前机械臂角色，`master` 或 `slave` |

不要根据单臂、双臂、同板、异板或输入源重新寻找 Carm 连接。末端实例已经绑定到正确的 `carm`、`side` 和 `role`。

## 3. 调用时机与返回语义

末端实例在 Carm 连接准备完成后、机械臂初始化运动前创建。构造函数适合校验当前 SDK 能力和静态参数；构造失败会中止机械臂会话初始化。

控制循环把 provider 归一化的 trigger 值传给 `set_eff()`。Quest 当前使用 `0..127`；实现仍应自行检查类型、有限性并钳制或拒绝越界值，不能假设未来输入源一定提供合法值。

末端命令只在 `track_pose()` 返回严格的 `True` 且连接身份仍有效后提交。Host 用每臂一个 `AsyncEndEffectorSender`（`server/basic/joystick/session/end_effector_sender.py`）包装末端实例：控制循环只把最新 trigger 放入单槽，不等待 SDK 应答；`set_eff()` 在该发送器的独立线程中被调用，同一实例不会被并发调用。发送规则：

- trigger 与上次成功发送的值不同时发送。
- 值相同时，距上次成功发送不足 0.5 秒则跳过，超过 0.5 秒才再次发送（保活）。
- 上次未被接受或抛出异常时，下一次提交必定重新发送。
- 停止、回零或控制循环异常时丢弃尚未发送的值；回零和断开前等待正在进行的 `set_eff()` 结束。

因此 `set_eff()` 的调用次数少于控制循环的步数，实现不应依赖“每个控制周期调用一次”。其返回值和异常按 best-effort 处理：

- 返回 `True`：本次末端命令被实现接受。
- 返回其他值：记录警告，不把已成功的位姿派发改判为失败，下一次提交会重发。
- 抛出异常：记录异常，不把已成功的位姿派发改判为失败，下一次提交会重发。

因此，返回 `True` 只应表达当前实现定义的“已接受”语义，不能用来宣称实体末端已经完成动作。需要逐通道尽力发送时，应像 `passthrough.py` 一样在实现内部隔离单次异常，避免一个通道阻止其余通道发送。

```python
class MyEndEffector(EndEffectorBase):
    NAME = "my_end_effector"

    def set_eff(self, trigger):
        try:
            self._sdk.send_channel_a(int(trigger))
            self._sdk.send_channel_b(int(trigger))
            return True
        except Exception:
            # 每个通道单独处理异常，不要让一个失败阻止其他通道继续发送
            return False
```

当前合同没有 `start`、`stop` 或 `close` 生命周期钩子。异步发送线程由 Host 统一持有，实现本身应保持轻量，避免再创建 worker、队列、后台线程或需要单独释放的外部资源；若新设备必须拥有这类资源，应先扩展并验证统一生命周期合同，而不是只在新模块中隐式持有资源。

```python
# 推荐：保持实现轻量，不在模块顶层创建后台线程或持久队列
class MyEndEffector(EndEffectorBase):
    NAME = "my_end_effector"

    def __init__(self, context, **options):
        super().__init__(context, **options)
        self._sdk = context.carm
```

## 4. 新增末端控制流程

1. 新增末端文件
    在 `server/basic/joystick/end_effectors/`下新增末端控制文件，例如`vacuum.py`.

2. 实现对应末端控制方式
    最小开发示例参考[blank.py](../server/basic/joystick/end_effectors/blank.py)

    > [!IMPORTANT]
    > 不需要手工修改注册表。`EndEffectorBase.load_all()` 会扫描包内非下划线开头的模块；类定义在导入时自动注册。启动时如果模块不存在、`NAME` 非法、类不继承基类或构造失败，应用会明确失败，不会回退到 `carm`。
    >
    > 桌面和后端启动时都会导入目录中的全部公开实现模块。模块顶层应只定义常量、纯函数和实现类，不应连接设备、启动线程或无保护地导入仅某个实现需要的可选依赖。新增必需的第三方包时，还要同步更新 `qt_app/requirements.txt`、`qt_app/pyproject.toml`、启动依赖检查、preflight 和 README 依赖表。

3. 选择新末端控制方式
    新增 `vacuum.py` 后，可通过现有启动入口选择：
    Windows：

    ```powershell
    .\run_windows.ps1 --end-effector vacuum
    ```

    Linux：

    ```bash
    ./run_linux.sh --end-effector vacuum
    ```

    Qt 会把名称转发给后端；后端使用相同名称解析工厂。默认实现仍是 `carm`。

## 5. 验证清单

在连接真实设备前，至少完成以下检查：

1. 发现检查：`EndEffectorBase.load_all()` 的结果包含新名称。
2. 构造检查：使用假的 Carm 对象分别构造需要支持的 `side` 和 `role`。
3. 输入检查：覆盖 `0`、`127`、越界值、布尔值、非数值和非有限数值。
4. 派发检查：确认一次 `set_eff()` 产生预期的 SDK 调用、参数和调用次数。
5. 失败检查：确认 SDK 返回失败或抛出异常时，行为符合该实现声明的 best-effort 或拒绝策略。
6. 启动检查：分别通过 Windows 或 Linux 根启动脚本选择新名称，确认 Qt 和后端使用同一实现。
7. 设备验收：在可控环境验证左右侧、单臂和双臂，以及停止、断连和重新初始化后的物理行为。

软件导入、构造或假对象测试通过，不等于 Carm SDK、总线、末端固件和实体动作已经验收。涉及真实硬件时，应保留日志，并明确区分“命令已提交”和“设备已执行”。
