# dual_arm_app 0.6.4 逆向分析报告

对象：`dual_arm_app-0.6.4-Linux-1.deb`（arm64，14.4 MB，2026-09-07 构建）
目的：搞清楚厂家到底提供了什么层次的控制接口，以及不依赖厂家 SDK 时能拿到什么。

---

## 0. 结论摘要

**这个 deb 里已经带了完整的高层 controller SDK，不需要靠逆向去「还原」它。**
`libjuxie_controller.so` 导出了 `Juxie::ControllerJuxie`，头文件也一并装在 `/usr/include/juxie_controller/`，接口包含 MoveJ / MoveL / MoveJ_P / MoveP_Canfd / MoveJ_Canfd / IK / FK / MoveEnd / 抱闸 / 零位标定。

除此之外还额外得到一样东西：**完整未 strip 的符号表**（14 个 .so + 3 个可执行文件），所有类名、函数名、模板实例都在，架构基本可以 1:1 还原。

包里还有一个厂家自带的 WebSocket 应用，那是**建在 SDK 之上**的东西，不是 SDK —— 本仓库不记录它，见 §4。

另外：运动学是旋量（PoE）形式，全套参数（螺旋轴、home、连杆、TCP）都在 yml 里，可以在你自己的机器上离线复现 FK/IK。

---

## 1. 包里到底有什么

| 路径 | 内容 |
|---|---|
| `/usr/bin/dual_arm_app_interface_node` | 主程序：控制器 + Web 界面服务（5566） |
| `/usr/bin/test_controller` | GoogleTest 用例，**展示了 SDK 的预期用法** |
| `/usr/bin/test_rk3576_can_canfd` | CAN 层测试 |
| `/usr/lib/libjuxie_controller.so.0.6.4` | **高层 controller 实现** |
| `/usr/lib/libexecutor.so.0.6.4` | 关节执行器（组 CAN 帧、位置换算） |
| `/usr/lib/libbot_{math,kinematics,planner,path_planner,traj_planner,validator,servo,utils,interface,communication}.so` | 运动学 / 规划 / 校验 / 伺服 / 通信（`math` / `interface` / `communication` 未进本仓库，见下） |
| `/usr/lib/librk3576_can_canfd.so.0.6.4` | RK3576 CAN-FD 驱动 |
| `/usr/lib/libweb_interface.so.0.6.4` | WebSocket 服务（websocketpp）；**未进本仓库**，见下 |
| `/usr/include/**` | **全套头文件**（含 `juxie_controller.h`、`kinematics_base.hpp`、`ExecutorBase.hpp` 等）+ 第三方（Eigen 除外：fmt、json、taskflow、websocketpp、piqp、zlgcanfd）。第三方那部分（`usr/include/third_party/`，6.2 MB）**已从本仓库的 SDK 树里移除**，只在 `.deb` 里还有 |
| `/usr/etc/*.yml` | 运动学 / 规划 / 执行器参数，按机型分 `juxie_53 / 62 / 62KML / 73` |
| `/usr/etc/error.json` | 应用层故障文案表 |
| `/usr/etc/data/array0_all/*.csv` | 厂家录制的关节轨迹（14 列 = 左臂 7 + 右臂 7，弧度）。`array04_2.csv` 727 行是 `MoveJCanfdTest` 的输入；`array04_all.csv` 228 行左臂相同、右臂不同 |
| `/usr/etc/dist/**` | 前端构建产物 + **sourcemap（含原始源码）** |
| `/usr/etc/dist/urdf/**` | URDF + 8 个 STL |

**`usr/lib` 里留了什么。** 本仓库的 `usr/lib` 只放 **SDK 自己的运行期闭包**，10 个库：`libjuxie_controller` / `libexecutor` / `libbot_{servo,planner,traj_planner,path_planner,kinematics,validator,utils}` / `librk3576_can_canfd`。

包里另外 4 个库没有进仓库，因为它们服务的不是 SDK：

| 库 | 为什么不在 |
|---|---|
| `libweb_interface`、`libbot_interface`、`libbot_communication`（共 2.8 MB） | 只被厂家那个 WebSocket 应用 `dual_arm_app_interface_node` 需要。那个应用建在 SDK **之上**（见 `running-on-the-robot.md`），而且它的可执行文件本身也不在仓库里 —— 在 `.deb` 里。 |
| `libbot_math.so`（8.4 KB） | 谁都不引用：导出 0 个动态符号，没有任何 `DT_NEEDED` 指向它。 |

实测（干净 sysroot + 去掉 `LD_LIBRARY_PATH`）：只留这 10 个库时，自研程序照常编译、加载、运行。剩下的 22 MB 之所以大，是因为库没 strip（符号表就是本仓库的证据来源），不是因为有冗余。

---

## 2. 原始工程结构（从 DWARF / 符号表 / 日志还原）

构建机路径：`/home/wwh/work/juxie_arm/`

```
juxie_arm/
├── bot_core/bot_common/          bot_math, bot_kinematics, bot_planner,
│                                 bot_path_planner, bot_traj_planner,
│                                 bot_validator, bot_servo, bot_utils
├── entities/controller/src/juxie_controller.cpp   → libjuxie_controller.so
├── entities/executor/src/ExecutorJuxie.cpp        → libexecutor.so
├── bot_driver/rk3576_can_canfd/src/rk3576_can_canfd.cpp → librk3576_can_canfd.so
├── bot_applications/
│   ├── interface/src/SimpleClientManager.cpp      → libbot_interface.so
│   └── web_interface/src/{WebSocketServer,WebSocketTaskServer}.cpp → libweb_interface.so
├── src/interface_adaptor.cpp                      → dual_arm_app_interface_node
└── third_party/{taskflow,fmt,piqp,websocketpp,zlgcanfd,...}
```

分层：

```
interface_adaptor.cpp (Adaptor: 25 条命令的 lambda 表)
        │
        ├── Interface / MiddleWareServer  (libbot_interface, TCP :30485 @192.168.10.95)
        ├── WebSocketServer/WebSocketTaskServer (libweb_interface, HTTP+WS :5566)
        └── Juxie::ControllerJuxie (libjuxie_controller)
                 │
                 ├── State 状态机: power_off → ready → idle → running → fault
                 │    (StatePowerOff / StateReady / StateIdle / StateRunning / StateFault)
                 ├── bot_planner / bot_path_planner / bot_traj_planner / bot_validator
                 └── bot_executor::ExecutorJuxie  (libexecutor)
                          └── rk3576_can_canfd::RK3576CanCanfd  (CAN-FD)
```

状态机语义（`juxie_state.hpp`）的主干：

- `OnRobot()` power_off→ready（有故障则 fault）；`EnableRobot()` ready→idle；`DisableRobot()` idle→ready
- `Move*` idle→running（实际由每 5 ms 一次的轮询线程切换）；`Stop()` running→idle
- `ClearFault()` →ready（之后由轮询线程纠正）；`OffRobot()` → power_off

逐状态、逐方法的完整行为、返回码和厂家自己的缺陷见 [robot-state-machine.md](robot-state-machine.md)。

---

## 3. 高层 SDK：`Juxie::ControllerJuxie`

完整接口见 `/usr/include/juxie_controller/juxie_controller.h`。数据结构（**这是最容易踩坑的地方**）：

| 类型 | 长度 | 布局 |
|---|---|---|
| `JointSpaceData` | 17 | `腰 1 + 左臂 7 + 右臂 7 + 头 2`，单位 rad |
| `ArmSpaceData` | 14 | `左臂 7 + 右臂 7`，单位 rad |
| `CartesianSpaceData` | 14 | `左臂 (x,y,z,qw,qx,qy,qz) + 右臂 同`，单位 m |
| `CartesianSpaceDataRPY` | 12 | `左臂 (x,y,z,rx,ry,rz) + 右臂 同` |

**哨兵值**：`DEFAULT_INVALID_VALUE = -100.`
- 关节空间：某关节填 `-100` → 保持该关节当前位置不变
- 笛卡尔空间：某个 TCP 的任意一个分量填 `-100` → 该臂不动
- 例：`{-100, 1,...,1, -100*7}` 只动左臂

主要方法：`OnRobot/OffRobot/EnableRobot/DisableRobot/Stop/ClearFault`、`GetRobotState`、`GetJointPositions`、`GetTCPPose`、`GetSingleTorques`、`getFKpose`、`IK`、`getDof`、`setJointZeroPosition`、`getConfig`、`MoveJ`、`MoveL`、`MoveJ_P`、`MoveJ_Canfd`、`MoveP_Canfd`、`MoveEnd`、`BreakEngage/BreakRelease`、`GetFaultType`、`GetAxisFault`、`getJointerrcode`。

> ⚠️ 两个 `GetTCPPose` / `getFKpose` 的文档注释都写明：**TCP 位姿是相对各自臂的 base 系，不是机器人 world 系。**

### 能不能直接链接

- 导出符号齐全（`nm -D` 可见全部 `ControllerJuxie::*`），库和头文件都在，理论可以直接链接。
- `juxie_controller.h` 自身只依赖 Eigen，**可以单独编译**。
- 但**部分头文件不自洽**，厂家漏发了三个：
  - `bot_subscriber/multi_thread_version.hpp`
  - `thread_safe_container/thread_safe_deque.hpp`
  - `bot_validator/validator_base.h`

  缺了它们，`bot_servo/ArmServoMode.hpp` 和 `executor/ExecutorJuxie.hpp` 编不过。要用伺服/执行器层得自己补这几个头（或直接链接 `.so` 而不编译这些头）。
- `libbot_interface.so` 对应的头目录 `/usr/include/bot_interface/` **是空的**，`Interface` / `MiddleWareServer` 的声明没给。

### 厂家自己的用例（`test_controller --gtest_list_tests`）

```
ControllerTest.StopTest            ClearFaultTest      GetRobotStateTest
GetFaultType                       GetJointPostionsTest GetTcpPoseTest
GohomeTest                         DisableRobotTest    MoveJTest
GoHomeTest                         MoveJArray04Test    MoveJ_P_Test
MoveLRectTest                      MoveJCanfdTest      Move_StopTest
```

**注意这两条容易搞混**：

- `MoveJCanfdTest` 才是**读 CSV 的那一条**：它按 `getenv("DUAL_ARM_SDK_CONFIG") + "/data/array0_all/array04_2.csv"` 拼路径，用 `csv::CSVReader` 逐行读、`strtod` 转数值，然后 `MoveJ_Canfd(joints17, **50**)` 以 50 Hz 流式下发。**这是包内唯一使用在线流控接口的地方**，也是那份 CSV 存在的意义。
- `MoveJArray04Test` **不读 CSV**：它只是 `memcpy` 两个写死的 17 维位姿，各 `MoveJ(..., 100)` 一次，中间 `sleep(1)`。

CSV 列到 17 槽的映射（从 `MoveJCanfdTest` 的指令里读出来的，不是猜的）：`joints[0] = -100.0`（腰，保持不动），`joints[1..7] = csv[0..6]`（左臂），`joints[8..14] = csv[7..13]`（右臂），`joints[15..16]` 厂家**没有写**（未定义行为）。

回放这段轨迹的 demo：[`research/vendor-tools/cpp/07_replay_trajectory.cpp`](../vendor-tools/cpp/07_replay_trajectory.cpp)。

---

## 4. 不属于 SDK 的部分：厂家自带的 WebSocket 应用

包里还有一个应用 `dual_arm_app_interface_node`：`interface_adaptor.cpp`（25 条命令的 lambda 表）+ `libbot_interface` + `libweb_interface`，在 5566 端口提供 JSON-over-WebSocket 接口和一套网页 UI，另有 30485 端口的 TCP 服务。

它**建在 SDK 之上**，不是 SDK 的一部分：

| 证据 | 结果 |
|---|---|
| 依赖方向 | node 的 `DT_NEEDED` 里有 `libjuxie_controller`；`libjuxie_controller` 反过来不依赖任何 web / interface 库 |
| 25 条命令名 | 只出现在 node 可执行文件里；14 个库里一个都没有 |
| WebSocket / JSON 符号 | `libjuxie_controller.so` 里为 **0** |

所以本仓库既不记录它的接口，也不保留它的库（`libweb_interface` / `libbot_interface` / `libbot_communication` 已从 `vendor/sdk/` 移除）—— 需要时从 `.deb` 里取。

它确实是分析阶段的入口：25 条命令的行为是借它实测的，`getFKpose` 那个堆溢出也是它的 `get_FK_pose` 崩出来的（见 §9 的已知问题）。下面各节里出现的 `get_*` / `move*` 命令名，指的就是这个应用暴露的那一套，用来标注"我们是怎么观察到的"。

---

## 5. 运动学（可离线复现）

`/usr/etc/juxie_73/kinematics_leftArm.yml` 给的是**旋量法（PoE / product of exponentials）**全套参数，不是 DH：

- `screws`: 7 条螺旋轴（ω + v），例如 `[0,0,1, 0,0,0]`、`[0,1,0, -0.1313,0,0]`
- `M`: 零位下的末端位姿（指数坐标），`[0,0,0,0,0,0.6752]`
- `links` / `linksXYZ`: 各连杆的指数坐标与平移
- `limits`: 每关节上下限
- `ik_tolerance_*`、`time_out`、`max_size`

实测：`get_FK_pose` 在零位返回 `[0,0,0.6755, 0,-0,0]`；`get_tcp_pose` 零位返回 `z = 0.6752`，与 `M` 完全一致。两者相差 0.3 mm —— 这不是笔误，是三方模型不一致的一部分，别把它当成相符。

> **已经实现了**：见 [`vendor_model/kinematics.py`](../vendor-tools/python/vendor_model/kinematics.py)（FK + 数值 IK，不依赖机器人）和 [`kinematics.md`](kinematics.md)（模型、RPY 约定、TCP 偏置、校验结果）。**只部分校验**：本库 FK 加拟合的 84.721 mm 偏置能复现厂家 IK 的目标位姿；本库 IK 未与厂家比过；与厂家 FK 对不上。范围见 kinematics.md 开头。

`/usr/etc/params.yml` 里注意 `MaxVelocityFactor: 0.04`（默认速度系数被压到 4%）、`UseLimit: false`（关掉的是每拍的速度 / 加速度限制，`LeftLimits` / `RightLimits` 的 `[1.5, 6.5]` 就是这两个上限；不是位置限位，见 `hardware-bringup.md` P0-4）。

---

## 6. CAN 层

详细比对见 [`can-protocol-comparison.md`](can-protocol-comparison.md)。摘要：

- 不走 Linux SocketCAN。通过 `/dev/mem` 直接映射 RK3576 CAN-FD 控制器寄存器（`0x2AC00000` / `0x2AC10000`），用 `/dev/misc_shm_can0/1` 做共享内存帧队列。
- 每路 CAN 挂 7 个关节；两路共 14 个（对应 14 个臂关节）。另有 Dev_ID 8（`MoveEnd` 的目标），是什么未知。
- 厂家栈实际发的报文：`0x600`（SDO，状态控制；没找到读应答的代码）、`0x100`（单轴快控，抱闸）、`0x108`（`MoveEnd`，给 Dev_ID 8 的单轴速度帧）、`0x200`（多轴广播，运动主通道，每 `Resample` 一帧，在 `libexecutor` 里组包）、`0x80`（同步帧，空闲拍代替 `0x200`）；收 `0x301`…`0x307`（反馈）。文档里的 `0x110` MIT 帧厂家栈没用。
- 单位换算：下发 `cnt = θ_rad/2π × 65536`，反馈 `θ_rad = cnt × π/32768`（与厂家文档一致，已从 `.rodata` 常量确认）。

---

## 7. 控制链路

**整个进程只创建 2 个 socket，都是 TCP。** 从进程启动开始 `strace -f -e trace=socket,bind,connect` 抓全程：

```
socket(AF_INET, SOCK_STREAM)  ×2
bind AF_INET, sin_port=htons(5566),    sin_addr=inet_addr("0.0.0.0")
bind AF_INET, sin_port=htons(30485),   sin_addr=inet_addr("192.168.10.95")
```

`AF_PACKET` / `AF_CAN` / `SOCK_RAW` / UDP / IPv6 / Unix socket / netlink 的命中数**全部为 0**。

**链路是两条：**

```
上位机 ──TCP/IP──> dual_arm_app_interface_node
  · :5566  WebSocket + HTTP（JSON API / Web UI）
  · :30485 自定义 TCP 服务（MiddleWareServer，绑 192.168.10.95）

dual_arm_app_interface_node ──CAN-FD──> 14 个关节模组
  · 直接映射 RK3576 CAN-FD 控制器寄存器：/dev/mem @ 0x2AC00000 / 0x2AC10000
  · 帧队列走共享内存 /dev/misc_shm_can0、/dev/misc_shm_can1
  · 每路 7 个电机，两路共 14 个（双臂 ×7）
```

其他佐证：

| 检查项 | 结果 |
|---|---|
| SDK 里出现的设备节点 | 只有 `/dev/mem`、`/dev/misc_shm_can0`、`/dev/misc_shm_can1` |
| 串口 / SPI / I2C / SocketCAN 节点 | 无 |
| 链接的库 | boost thread/regex + 厂家自有库 + libc/libstdc++ |
| 厂家自己的文档 | PR0002 与 CANopen-V0.6.xlsx 通篇讲 CAN FD / CANopen |

厂家文档把协议写成：

> 巨蟹关节模组支持 CAN 2.0 和 CAN FD 通信方式，协议基于 CANopen 402，也称 CiA 402 或 DS402。

`CiA 402` / `DS402` 是**设备行规**（drive profile，规定状态机与对象字典），不是总线 —— 读文档时不要把行规当成总线。

> 补充：`JointSpaceData` 是 17 维（腰 1 + 左臂 7 + 右臂 7 + 头 2），但 CAN 层只有 14 个电机，代码里也没有 waist/head 的概念。所以 17 是产品族的超集 API，这个安装包只管双臂 14 个关节（另外 `MoveEnd` 会给每路的 Dev_ID 8 发帧，可能是末端，见 `can-protocol-comparison.md` §2）。**没有第三条未解释的传输路径。**

## 8. 打包问题（建议反馈给厂家）

1. **control 文件里没有 `Depends` 字段**。实测缺 `libboost_thread` / `libboost_system` / `libboost_regex`（1.74.0）才能启动。按 `dpkg -i` 装完直接跑会 `error while loading shared libraries`。
2. 头文件不完整：漏发 `bot_subscriber/multi_thread_version.hpp`、`thread_safe_container/thread_safe_deque.hpp`、`bot_validator/validator_base.h`；`bot_interface/` 目录为空。导致 `ArmServoMode.hpp`、`ExecutorJuxie.hpp` 无法编译。
3. 包描述是 `cpack test program`，`Maintainer: xxx@163.com`，`Section: devel` —— 明显是没整理的 CPack 输出。
4. 依赖 `DUAL_ARM_SDK_CONFIG` 环境变量指向配置根目录（默认 `/usr/etc`），但没有任何文档或启动脚本设置它；不设就直接 `basic_string::_M_construct null not valid` 崩溃。
5. 库用 `libfoo.so.0.6.4` + `libfoo.so.3` 双 symlink，但 `SONAME` 是 `.so.3` —— 版本管理混乱。
6. `GetAxisFault()` 的故障文案停留在旧版固件（详见比对文档 §5），新固件下会给出错误描述。

---

## 9. 复现我的测试环境

不需要真机，用 qemu-user 就能把这个 arm64 程序跑起来（CAN 部分用伪造文件顶替，因此机器人会处于 `fault` 态）：

```bash
# 1) 运行环境
sudo dpkg --add-architecture arm64 && sudo apt-get update
sudo apt-get install -y qemu-user-static libc6:arm64 libstdc++6:arm64 libgcc-s1:arm64 \
                        libboost-thread1.74.0:arm64 libboost-system1.74.0:arm64 libboost-regex1.74.0:arm64

# 2) 组 sysroot。这是手工复现模拟环境的步骤，仓库本身不解包 .deb：
#    直接把已提交的 SDK 树拷进去，或用 dpkg-deb -x 从证据包里解一份。
dpkg-deb -x vendor/originals/dual-arm-app/0.6.4/*.deb extracted
ROOT=run/sysroot; mkdir -p $ROOT/usr/lib/aarch64-linux-gnu $ROOT/lib
cp -a /usr/lib/aarch64-linux-gnu/. $ROOT/usr/lib/aarch64-linux-gnu/
ln -sf usr/lib/aarch64-linux-gnu $ROOT/lib/aarch64-linux-gnu
cp -a extracted/usr/{bin,etc,lib} $ROOT/usr/

# 3) 顶替硬件：日志目录 + /dev/mem + 两个共享内存节点（任何一个 open 失败都会 exit(1)）
mkdir -p "$HOME/logs" $ROOT/dev              # 日志写到 $HOME/logs，目录不存在会抛异常终止
truncate -s 721420288 $ROOT/dev/mem          # 稀疏文件，mmap 出来全 0
truncate -s 4096 $ROOT/dev/misc_shm_can0 $ROOT/dev/misc_shm_can1   # 驱动各 mmap 4 KB

# 4) 运行（配置根目录必须通过环境变量指定）
DUAL_ARM_SDK_CONFIG=/usr/etc qemu-aarch64-static -L $ROOT $ROOT/usr/bin/dual_arm_app_interface_node
```

启动约 20 秒后监听 `0.0.0.0:5566`（HTTP+WS）和 `192.168.10.95:30485`（另一个 TCP 服务，需要先把该 IP 加到网卡上才能 bind 成功）。

（那个应用不属于 SDK，本仓库不提供驱动它的客户端；下面 `get_FK_pose` 那个已知问题是通过它观察到的。）

**要在模拟环境里跑 Python 那条路**（`juxie_sdk`，见 `sdk-usage.md` §7），sysroot 里还需要一个 aarch64 的解释器。它和 SDK 一样是 arm64，所以宿主机的 python 不行：

```bash
for p in python3.11-minimal libpython3.11-minimal libpython3.11-stdlib libffi8 \
         python3-numpy python3-yaml libblas3 liblapack3 libgfortran5 zlib1g libexpat1 \
         liblzma5 libbz2-1.0 libsqlite3-0 libncursesw6 libtinfo6 libreadline8; do
    apt-get download "$p:arm64"
done
for d in *.deb; do dpkg-deb -x "$d" $ROOT; done
ln -sf blas/libblas.so.3 $ROOT/usr/lib/aarch64-linux-gnu/libblas.so.3      # Debian 把 BLAS 放在子目录里
ln -sf lapack/liblapack.so.3 $ROOT/usr/lib/aarch64-linux-gnu/liblapack.so.3

./research/vendor-tools/python/build_bridge.sh
DUAL_ARM_SDK_CONFIG=$PWD/vendor/sdk/dual-arm-app/0.6.4/usr/etc \
JUXIE_SDK_BRIDGE=$PWD/research/vendor-tools/python/build/juxie_sdk_bridge.so PYTHONPATH=$PWD/src:$PWD/research/vendor-tools/python \
qemu-aarch64-static -L $ROOT $ROOT/usr/bin/python3.11 research/vendor-tools/python/sdk_min_example.py
```

（qemu-user 的 guest 进程直接跑在宿主内核上，所以 `PYTHONPATH`、`JUXIE_SDK_BRIDGE` 这些用宿主绝对路径就行；`-L $ROOT` 只影响动态库的搜索前缀。）

### 已知问题：`get_FK_pose` 会让整个应用崩溃

**根因与机制的正本在 [`sdk-usage.md`](sdk-usage.md) §6.1**：`getFKpose` 分配固定 7 个 double，却拷入 `n - 7` 个，
所以输入超过 14 个元素就堆溢出（ASan 确定性复现，复现程序 `research/vendor-tools/cpp/fk_overflow_repro.cpp`）。
这一节只记录**从厂家应用这一侧**观察到的现象，因为它们只能在应用里看到。

`get_FK_pose` 会**直接 abort 整个进程**。用 qemu gdbstub + gdb-multiarch 抓到的栈：

```
#0  abort ()
#6  malloc ()
#7  operator new (unsigned long)
#8  Adaptor::Adaptor()::{lambda(json const&)#9}::operator()(json const&) const   ← get_FK_pose 处理器
```

glibc 的报错原文（journal）：

```
Fatal glibc error: malloc assertion failure in sysmalloc:
(old_top == initial_top (av) && old_size == 0) || ((unsigned long) (old_size) >= MINSIZE
 && prev_inuse (old_top) && ((unsigned long) old_end & (pagesize - 1)) == 0)
```

栈顶的 `operator new` 是**受害者**：处理器构造响应的那段代码本身是对的（12 个 `double` 装进 12 元素的 json 数组，
分配 192 字节，写偏移最大 184）。越界写发生在它之前调用的 `getFKpose` 里 —— 处理器传的正是 17 个元素
（LeftNum = RightNum = 7），glibc 在下一次 `malloc` 才发现堆被破坏。

应用侧的其他观察：

- **空闲 60 秒不崩**（systemd 重启计数 +0），不是自发的定时崩溃。
- 一旦开始调用 `get_FK_pose`，往往第一次就崩（新起的进程也一样），因此**无法用它做闭环校验**。
- 崩掉的是整个 `dual_arm_app_interface_node` 进程，不只是那一个 WebSocket 连接。systemd 会重启它（观察到重启计数涨到 11），期间界面和所有控制通道都不可用。
- `get_device_state` / `get_joint_position` / `get_tcp_pose` / `get_config` / `get_IK_joint_position` 都稳定，几十次调用没问题。
- 早先偶尔能成功调用 `get_FK_pose`（零位返回 `[0,0,0.6755,0,-0,0]`），所以不是 100% 必崩。这个 0.6755 与直接调 `getFKpose` 的零位结果一致（而 `get_tcp_pose` 是 0.6752），印证 `get_FK_pose` 是 `getFKpose` 的薄包装。

WebSocket 的 `get_FK_pose` 没有任何请求形态是安全的，只能不调。真机上的复现步骤见 [`hardware-bringup.md`](../../docs/hardware-bringup.md) P0-3。

### 已知问题（另一个，独立）：`IK()` 返回的 Eigen 向量析构时越界读

见 [`sdk-usage.md`](sdk-usage.md) §6.2（正本，含归属存疑的说明）。

### 其他观察到的崩溃

还见过 SIGSEGV，以及两个客户端并发轮询时的一次崩溃。这些没拿到栈。建议在真机上做一轮稳定性测试（长时间轮询 + 并发客户端）。

---

## 10. 交付物索引

文档索引只维护一份：[`index.md`](../../docs/index.md)。代码与示例见 README 的 Layout / Demos 两节。

证据文件（本报告的结论出自这里）：

| 文件 | 说明 |
|---|---|
| `research/evidence/syms/*.syms` | 全部库与可执行文件的 demangle 符号表 |
| `research/evidence/dwarf_sources.txt` | DWARF 里的编译单元 / 源文件清单 |
| `research/vendor-derived/doc_canopen.txt` | `CANopen-V0.6.xlsx` 文本化 |
| `research/vendor-derived/doc_v13.txt` | `V1.3.xlsx` 文本化 |
| `research/vendor-derived/doc_v101.txt` | `_V1.0.1.docx` 文本化 |
