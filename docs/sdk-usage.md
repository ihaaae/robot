# C++ SDK 实际可用性（实测）

结论：**缺头文件不影响使用 SDK**。我实测编译、链接并运行了只包含 `juxie_controller.h` 的程序，能跑。但有一个**没写进任何文档的强制初始化步骤**，漏了它一半的 API 会直接段错误。

## 1. 缺头文件到底影响什么

| 场景 | 是否受影响 | 说明 |
|---|---|---|
| 运行厂家已有的程序（`dual_arm_app_interface_node` / `test_controller`） | ❌ 不受影响 | `.so` 已经编译好，运行期不需要头文件 |
| 用 `juxie_controller.h` 写自己的程序 | ❌ 不受影响 | 这个头只依赖 Eigen，**实测编译+链接+运行都通过** |
| 包含 `juxie_state.hpp` | ✅ 会失败 | 它 include `bot_executor/ExecutorJuxie.hpp` → 拖进缺失的 `bot_subscriber/multi_thread_version.hpp` 等 |
| 包含 `bot_servo/ArmServoMode.hpp` | ✅ 会失败 | 同上 |
| 包含 `executor/ExecutorJuxie.hpp` | ✅ 会失败 | 同上 |

所以：**高层 API 就是 `juxie_controller.h`，用它就够了**。缺的那三个头（`bot_subscriber/multi_thread_version.hpp`、`thread_safe_container/thread_safe_deque.hpp`、`bot_validator/validator_base.h`）只被伺服/执行器**内部实现**的头文件引用，不是公开 API 的一部分。别去 include `juxie_state.hpp` —— 那是实现细节（状态机类）。

## 2. 编译和链接（实测命令）

Eigen 没随包发布，需要自己装（`libeigen3-dev`，纯头文件，跨架构通用）：

```bash
aarch64-linux-gnu-g++ -std=c++17 -O1 \
    -I<sdk>/usr/include -I/usr/include/eigen3 \
    sdk_probe.cpp \
    -L<sdk>/usr/lib -Wl,-rpath-link,<sdk>/usr/lib \
    -Wl,--disable-new-dtags -Wl,-rpath,<sdk>/usr/lib \
    -ljuxie_controller -o sdk_probe
```

关键点有两个，分别管链接期和运行期：

- **链接期必须加 `-Wl,-rpath-link`。** `libjuxie_controller.so` 依赖 `libexecutor`、`libbot_servo`、`libbot_planner`、`libbot_traj_planner`、`libbot_kinematics`，不加搜索路径会报一堆 `undefined reference` —— 那是**链接期找不到传递依赖**，不是头文件缺失，别被误导。
- **运行期要么加 `-Wl,--disable-new-dtags`，要么设 `LD_LIBRARY_PATH`。** 那五个库自己都不带 RPATH，只能靠可执行文件带。现代工具链默认发 `DT_RUNPATH`，而 `DT_RUNPATH` **不用于传递依赖**：`libjuxie_controller` 能找到，它依赖的 `libexecutor` 找不到，程序直接死在 `libexecutor.so.3: cannot open shared object file`。`--disable-new-dtags` 让链接器改发 `DT_RPATH`（可传递搜索）。两条路都实测过。

**你的程序运行期不需要 Boost。** SDK 的 10 个库没有一个在 `DT_NEEDED` 里列 Boost（`readelf -d` 实测）。
需要 `libboost_thread` / `libboost_system` / `libboost_regex` 1.74.0 的是厂家自己的 WebSocket 应用
`dual_arm_app_interface_node`（deb 的 control 里没声明，见主报告 §8），不是 SDK。

## 3. ⚠️ 必须先调 `OnRobot()`

这是最容易踩的坑。构造完 `ControllerJuxie` 后**直接调用某些方法会 SIGSEGV**：

| 方法 | 不调 `OnRobot()` | 先调 `OnRobot()` |
|---|---|---|
| `getDof()` | ❌ SIGSEGV | ✅ `dof = 7` |
| `IK(...)` | ❌ SIGSEGV | ✅ 两臂都传有效位姿时返回 **0** 并解出 14 维关节角；任一臂为 `-100` 时返回 **-1** |
| `getJointerrcode()` | ❌ SIGSEGV | ✅ 全 0 |
| `setJointZeroPosition()` | ❌ SIGSEGV | ✅ 返回 1 |
| `GetRobotState()` | ✅ 返回 0 (`power_off`) | ✅ 返回 4 (`fault`) |
| `GetJointPositions()` | ✅ 全 `-100.0`（哨兵） | ✅ 全 `0.0` |
| `GetTCPPose()` | ✅ 全 `-100.0` | ✅ `(0,0,0.6752,1,0,0,0, …)` |
| `getConfig()` | ✅ ±3.1415 | ✅ |
| `getFKpose()` | ✅ `0.6755` | ✅ —— 但**输入元素个数必须 ≤ 14**，≥ 15 会堆溢出并 abort，见 §6.1 |
| `GetFaultType(1)` | ✅ `"Robot goes well"` | ✅ |
| `GetAxisFault(0x0001)` | ✅ `"过压"` | ✅ |
| `ClearFault()` / `Stop()` / `DisableRobot()` | ✅ 返回 1 | ✅ |
| `EnableRobot()` | ✅ 返回 0 | ✅ |
| `MoveJ()` / `MoveEnd()` | ✅ 返回 **-1** | ✅ 同样返回 **-1** |

原因：`State::getDof()` 之类的实现会解引用 `State::m_impl`（在 `juxie_state.hpp` 里定义为 `static`，构造函数里通过 `updateImpl()` 赋值）。没走 `OnRobot()` 时这条链路是空的，代码不做检查就直接解引用。

**所以正确的用法是：**

```cpp
Juxie::ControllerJuxie robot;
robot.OnRobot();               // 必须！否则一半 API 段错误
// 之后随便调
```

`OnRobot()` 返回 0 是正常的 —— 它表示"机器人没能真正 ready"（因为没有 CAN 连接），但**它已经把内部状态初始化好了**，后续调用就安全了。

## 4. 实测结果的意义

- `GetJointPositions` / `GetTCPPose` 在未初始化时返回 `-100.0`，**说明 `-100` 不只是命令侧的哨兵，也是"暂无数据"的返回值**。你的上位机必须判这个值，别直接拿去算。`OnRobot()` 之后即使没有 CAN，这两个读也变成全 `0.0` —— 所以 `0` 和 `-100` 都得判。
- `GetTCPPose()` 在 `OnRobot()` 后返回 `z = 0.6752`，与 yml 的 `M` 完全一致。注意 `getFKpose()` 在同一零位返回 **0.6755**，两者差约 0.3 mm —— 这是三方模型不一致的一部分，不是笔误，见 [kinematics.md](kinematics.md)。
- 运动指令（`MoveJ` / `MoveEnd`）在无 CAN 时返回 **-1**（`bot_common::ErrorCode::Error`）。**`IK` 不是这个行为**：它不碰 CAN，两臂位姿都有效时照常解出 14 维关节角（模拟环境实测）；只有位姿里带 `-100` 时才返回 -1。
  真机上成功应该是 0。返回码的完整表见 [`error-codes.md`](error-codes.md)。
- 逐方法探针里 `getFKpose` 传 17 个元素**没有崩**，但这不说明它安全 —— 它确实堆溢出（§6.1）。探针每个进程只调一个方法，越界之后没有下一次分配，glibc 没机会发现。

## 5. 关节向量布局（实测确定）

`getFKpose(v, 7, 7)`：

- `v[0..6]` → **左臂**，输出前 6 个值
- `v[7..13]` → **右臂**，输出后 6 个值
- `v[14..16]` → **不是"忽略"，是溢出**（见 §6.1）。传 17 个会堆损坏。

逐关节激励验证：`v[0]=0.5` 只让左臂 `rz` 变化（绕基座 z 的偏航），`v[1]=0.5` 让左臂 `x/z/ry` 变化（肩部俯仰）—— 与 `kinematics_leftArm.yml` 里 `screws[0] = [0,0,1,0,0,0]`、`screws[1] = [0,1,0,…]` 一致。

## 6. 调用顺序与参数的硬性约束

都是实测出来的，违反任何一条都不会给你清晰的报错。

### 6.1 `getFKpose` 的输入最多 14 个元素（只验证过 14 元素 / 7+7 这一组）

`getFKpose` 分配一个**固定的 7 个 double（56 字节）**缓冲区，然后把 **`n - 7`** 个 double `memmove` 进去（`n` = 传入 vector 的元素个数）。所以界内条件就是 **`n ≤ 14`**，与 `LeftNum` / `RightNum` 无关。

AddressSanitizer 下的实测（`-fsanitize=address -static-libasan`，这是**确定性**的判定，不依赖分配器运气）：

| 输入 `n` | `memmove` 长度 | 结果 |
|---|---|---|
| 14 | 56 字节 | 界内（这是**唯一**被实测验证过的参数组合：14 元素、LeftNum=RightNum=7） |
| 15 | 64 字节 | heap-buffer-overflow WRITE |
| 17 | 80 字节 | heap-buffer-overflow WRITE |
| 20 | 104 字节 | heap-buffer-overflow WRITE |

`LeftNum` 取 3 / 5 / 7 / 9 / 10 / 11 都遵循同一条边界。

**不加 sanitizer 时会怎样**：`getFKpose` 照常返回一个看起来正常的位姿，不报错；越界破坏堆元数据，**在下一次分配时才可能被发现**。能不能被发现取决于二进制和之前的分配历史 —— 所以「没崩」不代表安全，也可能是**静默的内存损坏**。这正是逐方法探针漏掉它的原因（每个进程只调一个方法，越界后没有下一次分配）。

复现程序：`examples/cpp/fk_overflow_repro.cpp`，头部带 ASan 编译命令。**本节是这个缺陷的正本**；它在厂家 WebSocket 应用里表现为整个进程崩溃，那一侧的观察见 [sdk.md](sdk.md) §9「已知问题」。

### 6.2 `IK()` 返回的向量析构时越界读（独立问题）

单独调 `IK()`、完全不碰 `getFKpose`，ASan 也会报：

```
READ of size 8 at ... 8 bytes to the left of 112-byte region
  #0 Eigen::DenseStorage<double, -1, -1, 1, 0>::~DenseStorage()
allocated by ... malloc inside Juxie::State::i_IK(...)
```

`i_IK` 用普通 `malloc` 分配输出向量的存储，交回来的却是 `Eigen::VectorXd`，析构时走 Eigen 的对齐释放路径去读指针前的记账字节。属于分配方/释放方不匹配。

这是越界**读**而非写，读的是相邻的堆元数据。实测中 `IK` 一直没崩（几十次调用），所以**目前不认为它会立刻破坏内存**；但它是未定义行为，换分配器、编译选项或 glibc 版本都可能变成崩溃：

- 不要依赖「`IK` 很稳」这个观察；
- 上真机后把它和 §6.1 一起报给厂家；
- 自己重新实现 IK 就不受这条影响。

**归属存疑**：这是 ASan 下的观察。也可能是 ASan 构建与厂家的 Eigen 用法 / 构建配置不兼容造成的，不一定是厂家 SDK 的固有问题。要在真机上用厂家自己的构建复现一次才能定性。

### 6.3 `IK` 要求两条臂都带有效位姿

`CartesianSpaceData` 是 14 个值（每臂 `x, y, z, qw, qx, qy, qz`）。头文件说 `DEFAULT_INVALID_VALUE` 表示"保持当前位置"，**这条对 `MoveJ` / `MoveJ_P` 成立，对 `IK` 不成立**：

```cpp
// 两臂都给有效位姿 -> 返回 0，解出 14 维关节角
// 任一臂留成 -100   -> 返回 -1，整条调用失败
```

它不抛异常、不打印，只返回非 0。所以**必须判返回值**，否则会拿着 14 个未填的关节角继续跑。`pose` 必须是 **14 个值**（每臂 `x, y, z, qw, qx, qy, qz`，四元数）且两臂都有效：任何一边填 `-100`，整条调用就失败（实测返回 -1）。注意别和 `getFKpose` 的 12 个 RPY 值搞混 —— 两者格式不同。

### 6.4 一个进程里只能有一个 `ControllerJuxie`

构造第二个实例并调 `OnRobot()` 会 **SIGSEGV**：

```cpp
Juxie::ControllerJuxie a;
a.OnRobot();
Juxie::ControllerJuxie b;   // 构造本身没问题
b.OnRobot();               // 这里段错误
```

`State::m_impl` 是 `static`（见 `juxie_state.hpp`），第二个实例的 `OnRobot()` 会踩掉第一个已经建立的状态。写测试或写长驻服务时注意：**不要每个用例都新建一个 controller**，用一个实例反复调用。

### 6.5 同一个实例也不能反复 `OnRobot()`

`Impl::InitRobot()` 每次都把一个新的 `std::thread`（状态轮询线程 `UpdateStateThread`）赋给同一个成员；成员已经持有线程时，C++ 规定调用 `std::terminate()`。线程只在状态类的 `OnRobot()` 返回 `true` 时才启动，所以：

- 没有 CAN（qemu 下）：第一次 `OnRobot()` 停在 `fault`、不启动线程；第二次启动；**第三次终止进程**（`terminate called without an active exception`，SIGABRT）；
- 有 CAN、第一次就成功：按同样的逻辑，**第二次**就会终止进程（静态推断，未实测）。

复现：`examples/cpp/state_machine_probe.cpp onrobot3`。

### 6.6 线程启动之后不能 `OffRobot()`

`OffRobot()` 释放 executor，把 `ControllerJuxieImpl` 里的 executor 指针清零，但不停状态轮询线程。线程每 5 ms 解引用一次这个指针，所以 `OffRobot()` 返回 `1` 之后，进程很快就会 **SIGSEGV**。复现：`state_machine_probe offrobot`（两次 `OnRobot()` 让线程启动，然后 `OffRobot()`）。

§6.5 和 §6.6 合起来的意思是：**一个进程里 `OnRobot()` 只调一次，并且不调 `OffRobot()`**，要断电就结束进程。两者在状态机里的位置见 [robot-state-machine.md](robot-state-machine.md) §4。Python 垫片目前**没有**替这两条做守卫（§7 的表）。

## 7. 从 Python 调用（`python/` + `shensi_robot.sdk`）

C++ 那边靠 `cmake/juxie-sdk.cmake` 接入；Python 这边对应的是 `python/juxie_sdk_bridge.cpp` —— 一个把 SDK 包成 C ABI 的垫片，配合 `src/shensi_robot/sdk.py` 用 ctypes 调用。原因很直接：SDK 的公开签名里有 `std::array` / `std::vector` / `Eigen`，ctypes 一个都表达不了。

**为什么是 C ABI + ctypes，而不是 pybind11。** 不是因为 pybind11 表达不了这些类型 —— 它表达得很好。差别在构建输入：pybind11 的扩展模块要**目标架构的 `Python.h`**，还要对上 CPython 的 minor 版本（头文件布局、`pyconfig.h`、扩展名后缀都得匹配），所以从 x86 交叉编译就得额外准备板子的 Python 开发头文件 —— 这是本仓库没有、也没法从宿主机推出来的东西。C ABI 垫片对 Python 一点依赖都没有：它就是个普通的 aarch64 共享库，任何 CPython minor 版本都能用 ctypes 加载。这既是「一条命令，两台机器都适用」成立的前提，也让阻塞的 `MoveJ` / `MoveL` 顺带拿到了 GIL 释放 —— `ctypes.CDLL` 每次调用都会放掉 GIL，而 pybind11 要显式加 `py::call_guard<py::gil_scoped_release>()` 才有同样的行为。

**那三个崩溃守卫和绑定方式无关**，别指望换个绑定库就少写它们：一个进程一个 controller、`OnRobot()` 之前不能调的四个方法、`getFKpose` 的 7~14 限制，都得自己实现。pybind11 能省掉的是别的东西 —— 25 个 `argtypes` 声明、512 字节的字符串缓冲、`ik()` 的 `out[:n]` 切分、`_call`/`_value`/`_flag` 三套返回约定，以及 `guarded()` 那层「把异常编码成整数」的搬运（它会自动把 C++ 异常转成 Python 异常）。代价是上面那套目标 Python 开发环境。**什么时候值得换**：如果板上固定一个 Python 版本、并且能离线提供对应的 aarch64 开发头文件（Debian 上是 `libpython3.11-dev:arm64`），那 pybind11 是更好的长期接口；否则现在这套更划算。

两半都是 aarch64，和 SDK 一样 —— 跑在机器人板子上（或模拟环境里），不是在笔记本上。

**一条命令，两台机器都适用。** `build_bridge.sh` 自己判断该用哪个编译器（`uname -m` 是 aarch64 就用板子自己的 `g++` 原生编译，否则用 `aarch64-linux-gnu-g++` 交叉编译），自己找 SDK 树（`$SDK` → 仓库里的 `vendor/sdk/...` → 部署好的 `/opt/juxie` → `/usr`）和 Eigen，然后**把它用的是哪棵 SDK 树记在桥旁边**（`juxie_sdk_bridge.sdk`）。模块读这个记录来设 `DUAL_ARM_SDK_CONFIG` —— 库和它的配置永远来自同一棵树，不是运行时猜的。

```bash
./python/build_bridge.sh
python3 examples/python/sdk_min_example.py
```

不需要 export 任何东西。要覆盖的话：`JUXIE_SDK_BRIDGE` 指定加载哪个桥，`DUAL_ARM_SDK_CONFIG` 指定配置根（后者优先级最高，模块不会覆盖你显式设的值）。

**装包时也会自动编译一次。** `setup.py` 里挂了个 best-effort 的 `build_py`：`pip install -e .` 会顺带跑一遍 `build_bridge.sh`，所以板上装完包就能直接用。三条边界：

- 它**永远不会让安装失败** —— 没有编译器、没有 Eigen、没有 SDK 树，安装照常成功，只是桥没编出来（那时按模块的报错提示手动跑一次脚本即可）。
- 它写到 `python/build/`（和脚本同一个位置），所以**可编辑安装**（`pip install -e .`）能自动找到；**非可编辑安装**（`pip install .`）把包装到 site-packages，找不到那个目录 —— 那种情况用 `JUXIE_SDK_BRIDGE` 指过去。
- 设 `SHENSI_SKIP_BRIDGE=1` 可以跳过。

> **关于 `pyproject` extra**：加 extra 解决不了这件事。extra 只能声明 **Python** 依赖，而这个桥需要的是 **C++ 编译器 + Eigen 头 + SDK 树**，pip 装不了。所以这里没有加 extra，而是把「编译」变成一条命令、把「找到它」变成自动 —— 这两件事才是原来那三步里真正烦人的部分。

```python
from shensi_robot.sdk import Controller

with Controller() as robot:
    print(robot.state_name(), robot.joint_positions())
    print(robot.fk_pose([0.0] * 14))
    robot.on_robot()        # 注意：这一步会给底层板子上电
    print(robot.get_dof(), robot.ik([0.2, 0, 0.5, 1, 0, 0, 0] * 2))
```

**垫片顺手把三个「会让解释器崩掉」的坑变成了异常**，这是它比单纯绑一层更有价值的地方：

| 坑 | C++ 里的表现 | Python 里的表现 |
|---|---|---|
| 一个进程里第二个 `ControllerJuxie` + `OnRobot()` | SIGSEGV（§6.4） | `juxie_create()` 返回 NULL → `SdkError` |
| 没调 `OnRobot()` 就调 `getDof` / `IK` / `getJointerrcode` / `setJointZeroPosition` | SIGSEGV | `SdkError`，code `-1002` |
| `getFKpose` 传超过 14 个元素 | 堆溢出（§6.1） | `SdkError`，code `-1003`，根本不会调进去 |
| 反复 `on_robot()`；线程启动后 `off_robot()` | `std::terminate` / SIGSEGV（§6.5、§6.6） | **未守卫**，同样会让解释器崩掉 |
| SDK 里抛出 C++ 异常 | 穿过 C ABI 边界是未定义行为，实际会终止进程 | 垫片在每个导出函数外面接住，转成 code `-1004`（分配失败）/ `-1005`（其他异常） |

中间那一行是逐方法实测出来的：不带 `OnRobot()` 跑一遍 `sdk_probe`，**正好这 4 个** SIGSEGV，其余（`state` / `joints` / `tcp` / `torques` / `config` / `fk` / `faulttype` / `axisfault` / `clearfault` / `stop` / `enable` / `disable` / `movej` / `moveend`）都正常返回。

**返回值的约定**（跟 SDK 自己的语义对齐，不另造一套）：

- 返回 `bool` 的方法（`on_robot()` / `off_robot()` / `enable_robot()` / `disable_robot()` / `stop()` / `clear_fault()` / `set_joint_zero_position()`）返回 `bool`。**`on_robot()` 返回 `False` 是正常结果**（没有 CAN 时板子起不来），不是失败；对后面那几个方法来说要紧的是**调用过**它 —— 实测的崩溃条件正是这个。
- 返回状态/数值的查询（`state()`、`get_dof()`）直接返回那个数：`0` 是 `power_off`、`4` 是 `fault`、`get_dof()` 是 `7`。这些是答案，不是错误码。
- 操作类（`move_j()` / `move_j_p()` / `move_l()` / `move_j_canfd()` / `move_p_canfd()` / `move_end()` / `ik()` / `break_*()`）返回非 0 就抛 `SdkError`，`SdkError.code` 是 SDK 自己的码（`0` 成功、`-1` 被拒），可以用 `fault_type(code)` 解码。
- 垫片自己的码是 `-1000`…`-1005`，SDK 自己的码范围是 `-104`…`0`（`state/error_code.h`），两者不会撞。

**线程**：垫片只把「创建/销毁」串行化（一个互斥量 + 拒绝非当前句柄的销毁），**调用中的方法不受保护**。所以一个 `Controller` 用一个线程，也不要在别的线程还在调用时 `close()`。之所以不做「整调用加锁」：那会让阻塞的 `MoveJ` 把并发的 `stop()` 也一起挡住 —— 在机器人上那是更糟的取舍。

**验证方式**：垫片和这个模块在模拟环境里端到端跑过 —— aarch64 的 python3.11 + numpy，`qemu-aarch64-static -L run/sysroot`：`fk_pose([0]*14)` 返回 `z=0.6755`（与 §4 记录的零位值一致）、`ik()` 返回 14 个关节角、`fault_type(-1)` 给出 SDK 自己的文案、`axis_fault(0x0004)` 解出「过温报错」、`move_j` 在没有 CAN 时抛出 code `-1`。arm64 解释器怎么准备见 [`sdk.md`](sdk.md) §9。

## 8. 文件

| 文件 | 说明 |
|---|---|
| `examples/cpp/01_offline_kinematics.cpp` | **Demo**：离线 FK / IK / 限位，不需要机器人 |
| `examples/cpp/02_read_telemetry.cpp` | **Demo**：状态、关节、TCP、力矩、三套错误编码 |
| `examples/cpp/04_guarded_motion.cpp` | **Demo**：带前置检查的运动序列，默认 dry-run，要 `--yes` |
| `examples/cpp/06_vendor_cyclic_motion.cpp` | **Demo**：复现厂家「循环运动」按钮的动作，直接用 SDK |
| `examples/cpp/07_replay_trajectory.cpp` | **Demo**：按 50 Hz 流式回放厂家录制的轨迹 |
| `examples/cpp/fk_overflow_repro.cpp` | **故障复现程序（会崩）**：证明 §6.1 的越界，头部带 ASan 编译命令 |
| `examples/cpp/sdk_probe.cpp` | 逐方法探针，每个方法一次运行，可选 `onrobot` 参数 |
| `examples/cpp/state_machine_probe.cpp` | 状态机探针：`power_off` / `fault` 两列逐方法，以及 §6.5、§6.6 两个崩溃。只用于模拟环境 |
| `examples/cpp/sdk_min_example.cpp` | 最小可用示例（只含 `juxie_controller.h`） |
| `python/juxie_sdk_bridge.cpp` | **C ABI 垫片**：把 SDK 包成 Python 能调的 C 接口（§7） |
| `python/build_bridge.sh` | 编译垫片，产出 `python/build/juxie_sdk_bridge.so` |
| `src/shensi_robot/sdk.py` | ctypes 封装，对外就是 `Controller` 一个类 |
| `examples/python/sdk_min_example.py` | 最小 Python 示例（对应 `sdk_min_example.cpp`） |
| `tools/probes/validate_fk_direct.py` | 用厂家 `getFKpose` 直接校验离线 FK 的脚本 |

构建好的 aarch64 二进制在 `run/sysroot/usr/bin/sdk_probe`，用 qemu 跑：

```bash
DUAL_ARM_SDK_CONFIG=/usr/etc LD_LIBRARY_PATH=/usr/lib \
qemu-aarch64-static -L run/sysroot run/sysroot/usr/bin/sdk_probe fkvec onrobot 0 0 0 0 0 0 0 0 0 0 0 0 0 0
```

Python 那侧同理，只是要一个 aarch64 的解释器（`docs/sdk.md` §9）：

```bash
./python/build_bridge.sh
PYTHONPATH=$PWD/src qemu-aarch64-static -L run/sysroot run/sysroot/usr/bin/python3.11 \
    examples/python/sdk_min_example.py
```
