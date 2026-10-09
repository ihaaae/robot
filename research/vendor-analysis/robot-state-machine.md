# 厂家的机器人状态机（`Juxie::State*`）

厂家 `libjuxie_controller` 里 `power_off / ready / idle / running / fault` 这 5 个状态，逐状态、
逐方法的行为。开发计划 2D.2 把它当**场景清单**用，不当规格（[`development-plan.md`](../../docs/development-plan.md)）。

**怎么得出来的**：

- 静态阅读 `libjuxie_controller.so.0.6.4` 与 `libexecutor.so.0.6.4` 的反汇编，片段在
  `research/evidence/disasm/controller.*.txt` 和 `executor.is*.txt`，由
  `research/vendor-tools/probes/disasm_excerpts.sh` 重新生成。很多状态方法只有 4–8 字节，被链接器合并成同一个地址
  （identical code folding），哪个符号对应哪个地址见 `controller.state-symbols.txt`。
- 在 qemu 下实测（`research/vendor-tools/cpp/state_machine_probe.cpp`）。实测覆盖了 `power_off` 和 `fault`
  两列，以及 §4 里的三个崩溃。`ready / idle / running` 三列要让 `OnRobot()` 成功，也就是要有真实的
  CAN 反馈，**只有静态阅读，没有实测**。

## 1. 结构

```
ControllerJuxie ──► ControllerJuxieImpl ──► m_state_（shared_ptr<State>，Impl+0x68）──► 虚函数
                         │
                         ├── changeState(新状态)：加锁（Impl+0x78 的互斥量）后替换 m_state_
                         └── UpdateStateThread：每 5 ms 轮询 executor，按结果改状态（§3）
```

- `ControllerJuxie` 的每个方法都只是把调用转给 `Impl`，`Impl` 再调当前状态对象的同名虚函数。
  分派时**不加锁**；只有 `changeState` 加锁。
- `GetRobotState()` 直接读 `m_state_->m_state`，同样不加锁
  （`controller.GetRobotState.txt`）。
- 受状态控制的方法有 15 个：`OnRobot / OffRobot / EnableRobot / DisableRobot / Stop / ClearFault`
  加 9 个运动/抱闸/IK 方法。其余方法（`GetJointPositions`、`GetTCPPose`、`getFKpose`、`getDof`、
  `setJointZeroPosition` 等）是基类 `State` 的实现，所有状态行为相同。
- `SelfCheck()` 在每个状态里只做一件事：打一行提示日志，然后返回 `false`（`idle` 返回 `true`）。
  表里写「拒绝（SelfCheck）」的格子就是它，比如 `fault` 下调 `EnableRobot()` 会打出
  `The robot hardware is in fault, please call ClearFault first`。

## 2. 状态 × 方法

返回码：`-101` = `RobotConnectFailed`，`-19` = `ArmNotEnabled`，`-103` = `ArmMoving`，
`-1` = `Error`（`state/error_code.h`）。「执行」指调用 `State` 基类里真正干活的那个 `m_*` 函数。
标 ✅ 的格子已在 qemu 下实测，与读出的结果一致。

| 方法 | `power_off`(0) | `ready`(1) | `idle`(2) | `running`(3) | `fault`(4) |
|---|---|---|---|---|---|
| `OnRobot` | 上电（§2.1） | 转 `InitRobot()` ¹ | 转 `InitRobot()` ¹ | 转 `InitRobot()` ¹ | 转 `InitRobot()` ¹ ✅ |
| `OffRobot` | `true`，不做事 ✅ | 释放 executor → `power_off` | 失能 → 释放 → `power_off` | **见 §2.2** | 释放 → `power_off`（不先失能）✅ |
| `EnableRobot` | 拒绝（SelfCheck）✅ | 使能 → `idle` / `fault` | `true`，不做事 | 拒绝（SelfCheck） | 拒绝（SelfCheck）✅ |
| `DisableRobot` | `true`，不做事 ✅ | `true`，不做事 | 失能 → `ready` / `fault` | 等运动结束，再同 `idle` ² | 拒绝（SelfCheck）✅ |
| `Stop` | `true`，不做事 ✅ | `true`，不做事 | `disableServo()` → `idle` / `fault` | `disableServo()` → `idle` / `fault` | 拒绝（SelfCheck）✅ |
| `ClearFault` | `true`，不做事 ✅ | 清错 → `ready` | 清错 → `ready` | 清错 → `ready` | 清错 → `ready` ✅ ³ |
| `MoveJ` / `MoveJ_P` / `MoveL` | `-101` ✅ | `-19` | 执行 | `-103` | `-1` ✅ |
| `MoveJ_Canfd` | `-101` ✅ | `-19` | 执行 | 执行 | `-1` ✅ |
| `MoveP_Canfd` | `-101` ✅ | **`-101`** ⁴ | 执行 | 执行 | `-1` ✅ |
| `MoveEnd` | `-101` ✅ | `-19` | 执行 | 执行 | `-1` ✅ |
| `BreakEngage` / `BreakRelease` | `-101` ✅ | 执行 | 执行 | `-103` | `-1` ✅ |
| `IK` | 执行 | 执行 | 执行 | 执行 | 执行 |

「→ A / B」的意思是：动作成功后问 executor 的 `isInFault()`，是就进 B 并返回 `false`，否则进 A
并返回 `true`。动作本身失败时返回 `false`，状态不变（`Stop` 例外：它不看 `disableServo()` 的结果）。

1. 状态类自己什么都不做，直接返回 `true`；于是 `Impl::OnRobot()` 接着调 `InitRobot()`（§2.1），
   公开返回值是 `InitRobot()` 的结果。实测在 `fault` 下（没有反馈）返回 `false`，状态不变。
   **线程已经在跑时再走到 `InitRobot()`，进程会被 `std::terminate()` 杀掉**（§4.1）。能到
   `ready / idle / running` 通常意味着线程已经启动，所以在这三列调 `OnRobot()` 基本就是终止进程。
2. `running` 下的 `DisableRobot()` 以 1 ms 为间隔轮询 `isMoving()`，直到运动结束，**没有超时**，
   也不会让运动停下来。
3. `ClearFault` 没有被任何状态重写，用的都是基类 `State::ClearFault()`：状态不是 `power_off`
   就调 `clearErrorsForJoint()`，成功后一律切到 `ready`，**连 `idle` 和 `running` 也不例外**。
   之后由 §3 的轮询线程在 5 ms 内纠正回 `idle` 或 `running`。实测在 `fault` 下（没有反馈）它返回
   `false`，状态不变。
4. `ready` 下的 `MoveP_Canfd` 返回 `-101`，同一行的其他运动方法都返回 `-19`。这不是读错：它的地址
   就是 `power_off` 那一组 `-101` 桩（`controller.state-stubs.a8538.txt`）。像是厂家写错了。

`IK` 在所有状态下都直接调 `i_IK`，不受状态控制。它在 `OnRobot()` 之前会段错误
（[`sdk-usage.md`](sdk-usage.md) §3），原因是 `m_impl` 还是空指针，与状态无关。

### 2.1 `power_off` 下的 `OnRobot()`

`StatePowerOff::OnRobot()` → `m_on_robot()`：按环境变量 `DUAL_ARM_SDK_CONFIG` 和机型（`ARM_73` /
`ARM_62` / `ARM_53`）选 `executor_arm.yml`，创建 `ExecutorJuxie`。创建失败返回 `false`，停在
`power_off`。创建成功就 `usleep(1 s)`，再问 `isInFault()`：是就进 `fault` 并返回 `false`，否则进
`ready`，返回 `true`。

然后回到 `Impl::OnRobot()`：

- 如果此刻是 `fault`，先 `ReadFault()`，等 5 ms，读出 14 个轴的故障码并写日志；
- 状态类返回了 `true`，才调 `InitRobot()`：**启动 `UpdateStateThread`**；此时若仍 `isInFault()`，
  就调一次 `clearErrorsForJoint()`，之后仍有故障则返回 `false`。

`isInFault()` 的初值是 `true`（`ExecutorJuxie.hpp` 里 `JuxieState::isInFault {true}`），只有收到
关节反馈才会变成 `false`。所以没有 CAN 总线时 `OnRobot()` 必然停在 `fault` 并返回 `false`，这正是
[`sdk-usage.md`](sdk-usage.md) §3 观察到的「`OnRobot()` 返回 0、状态为 4」。这时轮询线程**没有启动**，
因为 `InitRobot()` 没被调用。

### 2.2 `running` 下的 `OffRobot()`：条件写反了

`StateRunning::OffRobot()`（`controller.StateRunning.OffRobot.txt`）：

1. 以 1 ms 为间隔轮询 `isMoving()`，直到运动结束，没有超时；
2. `setEnableForJoint(false)`；
3. **失能失败**：释放 executor，切到 `power_off`，返回 `true`；
4. **失能成功**：`isInFault()` 为真就切到 `fault`；否则 `isMoving()` 为真就切到 `running`；
   否则什么都不做。三种情况都返回 `false`，而且**不释放 executor**。

对照 `StateIdle::OffRobot()`：失能（不看结果）→ 释放 → `power_off` → `true`。`running` 版本的第 3、4
步像是把判断条件写反了。**静态阅读，未实测。**

### 2.3 `Stop()` 做了什么

`Stop()` 调 executor 的 `disableServo()`：把 `+0x4b8` / `+0x4b9` 两个原子量置 1（按头文件成员顺序是
`isLeftStop_` / `isRightStop_`），然后最多等 1 s，等某个状态位变化。从这里看不出是减速停还是立即停，
这要在真机上测（[`hardware-bringup.md`](../../docs/hardware-bringup.md) P1-4）。`fault` 下 `Stop()` 被拒绝。

## 3. 自动转换：`UpdateStateThread`

第一次成功的 `OnRobot()` 启动这个线程（§2.1）。它每 5 ms 按下面的优先级判定一次，结果和当前状态
不同才切换（`controller.Impl.UpdateStateThread.txt`，周期见 `rodata.txt`）：

| 优先级 | 条件（executor 的方法） | 切到 | 日志 |
|---|---|---|---|
| 1 | `!isConnected()` | `power_off` | `Connection Lost` |
| 2 | `isInFault()` | `fault`（先等 5 ms、读 14 轴故障码写日志） | `hardware is InFault` |
| 3 | `isMoving()` | `running` | `hardware  is Moving` |
| 4 | `isEnabled()` | `idle` | `hardware is Idle` |
| 5 | 其他 | `ready` | `hardware is Ready` |

这 4 个谓词在 executor 里是什么（`executor.is*.txt`）：

- `isConnected()`：**恒返回 `true`**，所以第 1 行永远不会触发，「掉线 → `power_off`」不存在；
- `isInFault()` / `isEnabled()`：读 `JuxieState` 里的 `isInFault` / `isEnabled`，由反馈更新；
- `isMoving()`：`isLeftSending_ || isRightSending_`，即「有一条臂正在执行轨迹」
  （[`l2-executor.md`](../../docs/components/l2-executor.md) 附录 A.2 的 `SetSending` 一行）。

由此得到三条结论：

1. **在轮询线程跑起来之后，状态实际上是 `(isInFault, isMoving, isEnabled)` 的函数。** 状态类里的
   `changeState` 只是提前 5 ms 给出同一个答案；两者不一致时，线程的判定会盖掉状态类的判定。
   `ClearFault()` 切到 `ready` 后又回到 `idle`（注 3），就是一个例子。
2. **进入 `running` 不是 `MoveJ` 做的**，而是线程看到 `isMoving()` 变真。所以在 `idle` 下，两个
   相隔不到 5 ms 的 `MoveJ` 都能通过状态检查，`-103` 挡不住它们；真正的互斥只能靠 executor。
3. **`fault` 不是锁存的。** 只要反馈里的故障位自己消失，线程就会离开 `fault`，不需要调 `ClearFault()`。
   反过来，`ClearFault()` 成功但故障位还在时，5 ms 后又会回到 `fault`。

## 4. 生命周期缺陷（qemu 实测）

三个都是**厂家 SDK 的缺陷**，正本在 [`sdk-usage.md`](sdk-usage.md) §6.5–§6.7，这里只说它们和状态机的关系。

### 4.1 第三次 `OnRobot()` 终止进程

`InitRobot()` 每次都把一个新的 `std::thread` 赋给同一个成员；该成员已经持有线程时，C++ 规定调用
`std::terminate()`。没有 CAN 时第一次 `OnRobot()` 不启动线程（§2.1），第二次启动，**第三次终止进程**：

```
OnRobot        ->    0   state=4
OnRobot        ->    0   state=4
terminate called without an active exception
```

有 CAN、第一次就成功时，按同样的逻辑**第二次**就会终止进程。

### 4.2 线程启动后 `OffRobot()` 段错误

`m_off_robot()` 释放 executor，把 `Impl` 里的 executor 指针清零，却不停 `UpdateStateThread`。线程在
下一次轮询时解引用空指针：

```
OnRobot        ->    0   state=4
OnRobot        ->    0   state=4
OffRobot       ->    1   state=0
qemu: uncaught target signal 11 (Segmentation fault)
```

### 4.3 `OffRobot()` 之后再 `OnRobot()`，析构时段错误

不需要轮询线程：`OnRobot()` → `OffRobot()` → `OnRobot()` 三次都正常返回，销毁 controller 时 SIGSEGV
（`state_machine_probe offon`）。原因没有追查。

也就是说，厂家 SDK 的**一个 controller 实例**只支持一轮 `OnRobot()`（/ `OffRobot()`）。析构函数会停掉并
`join` 轮询线程，所以销毁后新建一个实例可以重来（模拟环境实测）。Python 垫片按这条规则拒绝第二轮
（`sdk-usage.md` §7）。

## 5. 对我们 L4（2D.2）的含义

**要兼容的**（这是 8.1「同名方法、同号返回码」的具体内容）：

- 5 个状态的编号 0..4 与 `GetRobotState()` 一致；
- §2 表里每一格的返回值。唯一的例外是注 4，要不要照抄 `ready` 下 `MoveP_Canfd` 的 `-101`，需要
  做个决定。建议改成 `-19`，并在兼容性说明里写明；
- 「不做事也返回 `true`」的那些格子（例如 `power_off` 下的 `Stop()` / `DisableRobot()`）。应用代码
  可能依赖它们不报错。

**不兼容的**（它们是缺陷，不是契约）：

- §4 的三个崩溃，以及 §2.2 写反的条件；
- `running` 下 `DisableRobot()` / `OffRobot()` 无超时的忙等（注 2）；
- 分派不加锁：我们的状态读写要么全在一把锁下，要么用原子量加单一写者。

**结构上的取舍**：厂家的状态由两个来源写入，状态类和轮询线程，而且轮询线程说了算。我们的 L4 建议
只保留一个写者：**状态由 L2 的快照推导**，L4 的方法只负责发出请求，再等状态变到位（或超时）。这样
§3 的三条结论自然成立，不会出现「先切到 `ready`、5 ms 后又切回来」这种闪变。L4 需要 L2 提供的输入：

| 厂家谓词 | 我们从 L2 拿什么 |
|---|---|
| `isMoving()` | `motion_active(Part::Both)` |
| `isEnabled()` | `snapshot()` 里各关节的 `enabled` |
| `isInFault()` | `snapshot()` 里各关节的 `error` / `fault`，加上 `ArmHealth`（新鲜度失效也算故障） |
| `isConnected()` | `ArmHealth::bus_ok`。厂家这一项恒为真；我们要真的实现「掉线」 |

这张表也写进了 [`l2-executor.md`](../../docs/components/l2-executor.md) 附录 A.3。

## 6. 还需要真机确认的

- `ready / idle / running` 三列的全部格子（§2 表里没有 ✅ 的格子）；
- §2.2 写反的条件是否真的会发生（需要失能失败的场景）；
- `Stop()` 是减速停还是立即停（§2.3）；
- 故障位自己消失时是否真的会自动离开 `fault`（§3 结论 3）。

这些都并入 [`hardware-bringup.md`](../../docs/hardware-bringup.md) P1-4。
