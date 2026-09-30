# L0 接口草案

> **草稿 v0。** 目的是让 6 个人在真机到货前就能并行推进，所以先把 L0 的边界和签名冻结下来。
> 内容会变；变的时候改这一份，不要各自在代码里另立一套。
>
> 依据：[`can-protocol-comparison.md`](can-protocol-comparison.md)（协议对照与字节序）、
> PR0002（协议正本）、[`hardware-acceptance.md`](hardware-acceptance.md)（真机验收门）。
> 相关：[`development-plan.md`](development-plan.md)（任务划分）。

## 0. 已定的决定

| 决定 | 结论 |
|---|---|
| 语言 | **C++ 为第一公民**；对外导出 C ABI，Python 通过 ctypes 使用（沿用仓库现有 `python/juxie_sdk_bridge.cpp` + `src/shensi_robot/sdk.py` 的套路）。C ABI 还顺带让 Python 侧拿到 GIL 释放。 |
| 总线独占 | **L0 在任何情况下都不与 `Juxie::ControllerJuxie` 同进程共存。** 见 §5。 |
| 分层 | L0 只做"字节 ↔ 线上结构体 + 收发 + trace"，**不含策略、时序、状态**。 |
| RX 边界 | **原始帧。** 不走厂家 `setReadFunction` 解好的 `JointState`——那个结构没有温度字段，且解码必须能独立验证。见 §4.3。 |
| 厂家 `.so` | **允许链接**，但 `VendorShmTransport` 只作为 TX / 时序 / capture 后端，不是必选路径。见 §4.2。 |
| 公共 API | 新 SDK 按**与 `Juxie::ControllerJuxie` 兼容**设计：同名方法、同名返回码。 |
| 原始帧来源 | **未定，等真机。** 见 §4.4。 |

## 实现状态

`cpp/` 下已经落地 **wire + transport + trace/差分** 三块，`./cpp/build.sh` 一条命令构建并跑测：
1069 个断言，0 失败，`-Wall -Wextra -Wpedantic` 零警告，ASan + UBSan 下干净。
golden vector 全部取自 PR0002 自带的例子，并已并入 `tools/verify.sh` 第 6 步（离线，只需宿主 C++ 编译器）。

| 文件 | 内容 |
|---|---|
| `cpp/include/shensi/can/frame.hpp` | `Frame`、标识符映射、`classify()` / `dev_id_of()`、`to_hex` / `from_hex` |
| `cpp/include/shensi/can/wire.hpp` | SDO / 控制子帧 / 广播 / 反馈 / NMT / 单位换算 |
| `cpp/include/shensi/can/transport.hpp` | `Transport` 接口（含总线独占规则） |
| `cpp/include/shensi/can/fake_transport.hpp` | 脚本化假后端 |
| `cpp/include/shensi/can/trace.hpp` | `Trace` 文本格式、`diff()`、`RecordingTransport`、`ReplayTransport` |
| `cpp/tests/test_wire.cpp` | golden vector、字节序、往返、文档矛盾 |
| `cpp/tests/test_transport.cpp` | 收发、responder、`send_raw` |
| `cpp/tests/test_trace.cpp` | 文本往返、解析报错、六类差分、左右臂互换、录制回放 |
| `cpp/CMakeLists.txt`, `cpp/build.sh` | 构建（cmake 优先，无 cmake 时直接 g++） |

还没做：C ABI 导出（Python 侧）；`0x110` MIT 单轴编解码（§8 未确认，故意先不做）。

## 1. 边界：L0 拥有什么，不拥有什么

**拥有**

- 字节 ↔ 线上结构体的双向映射（纯函数，无 I/O、无状态）
- 原始帧的发送与接收
- trace 的记录、回放、归一化、差分

**不拥有**

- 什么时候发什么（L1 的时序 / DS402 序列）
- 关节索引到机器人的映射（L2：14 个设备 ↔ `Dev_ID` ↔ 通道；17 维 API ↔ 14 个设备归 L5，见 §10）
- 什么时候必须发帧（L3 的节拍与喂狗，见 [`l3-executor-interface.md`](l3-executor-interface.md)）
- 单位换算的**语义**（上层决定用 rad 还是 deg）；L0 只提供 `cnt ↔ rad` 的纯函数
- 运动学（84.721 mm 偏置属于任务 4，不属于 L0）

## 2. 三块结构

| 块 | 内容 | 能否离线验证 |
|---|---|---|
| **wire** | 编解码纯函数 | ✅ 完全离线，golden vector 在这里 |
| **transport** | 原始帧收发，多后端 | ✅ fake / replay 后端离线可跑 |
| **trace** | 记录 / 回放 / 归一化 / 差分 | ✅ 对冻结语料离线可跑 |

## 3. wire

### 3.1 基本类型

```cpp
namespace shensi::can {

enum class Bus : uint8_t { Can0 = 0, Can1 = 1 };

struct Frame {
    Bus      bus;
    uint32_t id;                        // 11-bit standard id；不塞 EFF/RTR 标志
    uint8_t  len;                       // 0..64
    bool     brs;                       // 数据段加速
    bool     fdf;                       // CAN FD 帧
    std::array<uint8_t, 64> data;
};

}  // namespace shensi::can
```

### 3.2 帧类别与字节序（**最容易写错的地方**）

| 类 | CAN ID | DLC | 字节序 |
|---|---|---|---|
| SDO 请求 | `0x600 + Dev_ID` | 8 | **小端** |
| SDO 应答 | `0x580 + Dev_ID` | 8 | **小端** |
| 单轴控制 | `0x100 + Dev_ID` | 7 | **大端**（16 位字段）|
| MIT 单轴 | `0x110 + Dev_ID` | 9 | 未确认（§6）|
| 多轴广播 | `0x200` | 64 | 大端 |
| MIT 多轴 | `0x210` | 64 | 未确认 |
| 执行器反馈 | `0x300 + Dev_ID` | 12 | **大端** |
| 同步帧 | `0x80` | 0 | — |
| 心跳 / 上线 | `0x700 + Dev_ID` | 1 | — |
| `MoveEnd` 用帧 | `0x108` | 7 | 未文档化，见 §4.5 |

**同一个协议两套字节序。** 已实测确认：SDO 的索引 `0x6040` 编成 `40 60`，而控制帧里的目标位置 16384 编成 `40 00`。

### 3.3 SDO

```cpp
enum class SdoCmd : uint8_t {
    Write1 = 0x2F, Write2 = 0x2B, Write3 = 0x27, Write4 = 0x23,
    Read   = 0x40,
    AckWrite = 0x60, Read1 = 0x4F, Read2 = 0x4B, Read3 = 0x47, Read4 = 0x43,
};

struct SdoRequest  { uint8_t dev_id; SdoCmd cmd; uint16_t index; uint8_t sub; uint32_t value; };
struct SdoResponse { uint8_t dev_id; SdoCmd cmd; uint16_t index; uint8_t sub; uint32_t value; };

Frame encode(const SdoRequest&);
bool  decode(const Frame&, SdoResponse& out);   // false 表示不是 SDO 应答
```

索引与值小端；`value` 占 `[4..7]`，只有低 N 字节有意义（N 由 cmd 决定）。

### 3.4 单轴控制子帧（`0x100 + Dev_ID`，也是 `0x200` 的子帧格式）

```cpp
struct ControlSubframe {        // 7 字节
    bool    enable;             // byte0 bit7  1 = 上使能
    bool    brake_release;      // byte0 bit6  1 = 抱闸释放
    bool    clear_error;        // byte0 bit5  1 = 复位错误
    uint8_t mode;               // byte0 bit4..1  项目真实模式值 1..7
    int16_t target1;            // bytes1..2 大端；位置(cnt)/速度(RPM)/电流(mA)/力矩(0.1Nm)
    int16_t target2;            // bytes3..4 大端；轮廓位置与轮廓速度模式下为加减速(RPM/s)
    int16_t feedforward;        // bytes5..6 大端；位置模式下为输出端轮廓速度
};

std::array<uint8_t, 7> encode(const ControlSubframe&);
ControlSubframe        decode(const std::array<uint8_t, 7>&);
```

模式值：`1` PP / `2` PV / `3` CSP / `4` CSV / `5` 电流 / `6` MIT / `7` 力矩传感器闭环。

### 3.5 执行器反馈（`0x300 + Dev_ID`）

```cpp
struct JointFeedback {          // 12 字节
    int16_t  pos_cnt;           // bytes0..1 大端，-32768..32767 == -180..180 deg
    int16_t  vel_rpm;           // bytes2..3 大端
    int16_t  current_ma;        // bytes4..5 大端
    uint16_t fault;             // bytes6..7 大端
    int16_t  temp_dc;           // bytes8..9 大端，0.1 degC
    uint8_t  mode;              // byte10，项目模式值
    bool     enabled;           // byte11 bit7
    bool     brake_released;    // byte11 bit6
    bool     error;             // byte11 bit5
    bool     in_position;       // byte11 bit4
};

JointFeedback decode(const Frame&);
```

### 3.6 单位换算（放在 L0，因为它贴着线格式且有实测陷阱）

```cpp
constexpr double CNT_PER_REV = 65536.0;

// 向零截断，与厂家驱动一致（cast / fcvtzs），不是四舍五入。
// 这条差异在 cnt 边界上会差 1，任何性质测试都看不见。
int16_t rad_to_cnt(double rad);     // trunc(rad / 2pi * 65536)
double  cnt_to_rad(int16_t cnt);    // cnt * pi / 32768
```

注意 `rad = pi` 会得到 32768，超出 int16 范围；上限处要夹紧。

### 3.7 原始逃生口

```cpp
void send_raw(Bus, uint32_t id, uint8_t len, const uint8_t* data, bool brs);
```

`MoveEnd` 的 `0x108` / 6000 cnt/rev 帧没有文档（`v` 的物理单位未知），不值得为它建模。任何未文档化的帧都走这里，不污染上面那套类型。

**注意 `0x108` 落在单轴命令区间里。** `0x108 = 0x100 | 0x08`，而 PR0002 §5.3 的关节只用 `Dev_ID 01..07`，所以 `08` 是空的。也就是说这条帧在标识符和长度上，和「发给 8 号设备的单轴控制帧」**完全无法区分**——`classify()` 会把它判成 `SingleAxisCommand`，`dev_id_of()` 返回 8。它到底是不是一条单轴命令（8 号设备是手/末端？）**未解决**，这正是它走 `send_raw` 而不是类型化编码器的原因。真机抓帧时优先确认这条。

## 4. transport

### 4.1 接口

```cpp
class Transport {
public:
    virtual ~Transport();
    virtual void send(const Frame&) = 0;                                  // 非阻塞
    virtual void set_receiver(std::function<void(const Frame&)>) = 0;
    virtual bool healthy() const = 0;
};
```

形状对齐厂家 `RK3576CanCanfd`（`can_send_frame(Can_If, uint32_t, uint32_t, uint8_t*)` + `setReadFunction`），使厂家寄存器驱动可以作为其中一个后端接入。

### 4.2 后端

| 后端 | 说明 | 何时可用 |
|---|---|---|
| `FakeTransport` | 脚本化应答 | 现在 |
| `ReplayTransport` | 读 trace 语料 | 现在 |
| `RecordingTransport` | 装饰器，把任意后端录成 trace | 现在 |
| `VendorShmTransport` | 走 `librk3576_can_canfd.so` 的 `can_send_frame` | 板子；需确认是否允许依赖厂家 .so（见 §7）|
| `SocketCanTransport` | `can0` / `can1` | 未确认内核是否暴露 |
| `ZlgUsbCanTransport` | PC 侧 USB-CAN | 以后 |

### 4.3 RX 边界（**待拍板**）

厂家驱动的 `setReadFunction` 回调签名是 `void(JointState&, JointState&)`——它给的是**已经解码好的** `JointState`，不是原始帧。而且 vendored 头里的 `JointState` 只有
`MotionState / ControlType / Current / Vel / single_torque / six_axis_torque / FaultData / origPosAct / isUpdated`，**没有温度字段**——走它的解码就永远读不到反馈帧 `[8..9]`。

因此 L0 的 RX 边界定为**原始帧**（推荐）：解码借厂家的，等于把自己的 L1 / L2 交给一个无法独立验证的实现。反馈帧的字节解码已经落在 L0 的 `decode_feedback`，物理量换算是 L1（任务 2.6）。`VendorShmTransport` 只作为 TX / 时序 / capture 的便利后端。

若改选"用厂家解码"，则 `decode_feedback` 与任务 2.6 整个消失，且 L0 的 `Frame` 类型对 RX 侧失去意义。

### 4.4 原始帧从哪来（**现在拍不了，等真机**）

三条候选路径：

1. 自己读 `/dev/misc_shm_can0|1`——需要那个未确认的 4 KB 结构（`can-protocol-comparison.md` §6）
2. SocketCAN——取决于真机上 `ls /sys/class/net` 有没有 `can0`/`can1`（`hardware-acceptance.md` P2 的一项）
3. PC + USB-CAN 适配器

注意 vendored 头 `rk3576_can_canfd/rk3576_can_canfd.h` 里**已经有完整的寄存器映射**（`CAN0_PHYADDR 0x2AC00000`、`CAN1_PHYADDR 0x2AC10000`、`CANFD_NBTP 0x100`、`CANFD_DBTP 0x104`、`CANFD_BRS_CFG 0x10c`、`CANFD_TXID 0x204`、`CANFD_TXDAT0 0x208`…），所以"自己写寄存器级驱动"是可行的，不必依赖那个 `.so`。

L0 的接口对四条路都成立，所以**先落 `FakeTransport` + `ReplayTransport`，离线就能推进**。

## 5. 总线独占（已定）

L0 在任何情况下都不与 `Juxie::ControllerJuxie` 同进程共存。这条要写进接口的文档注释，否则一定有人踩。

三个理由：

1. **进程内两个 transport 明确是坏的。** shm 映射指针是全局的、每通道一个 RX 线程；第二个实例会重映射同一块 `/dev/mem` 区域、mmap 同一块 shm、并成为同一帧队列的第二个消费者——帧会在两个读者之间非确定性地分裂。
2. **协议不允许两个主站。** SDO `0x600|id` 是请求/应答且无源地址，两个主站的会话会撞。更糟的是看门狗靠周期控制帧喂（< 500 ms，否则自锁）：两个部分 owner 时"对方在喂狗"是运动中途锁死的失败模式。
3. **`OnRobot()` 之后厂家栈直到进程退出都不静止。** 它没有可以插入外来调用的空闲窗口。

推论：**增量只存在于 SDK 内部的覆盖度里，总线级是原子的。** 应用要么整体用厂家 SDK，要么整体用新的。"再实现几个 API → 测 → 再来几个"作为开发节奏成立，但"测"要么是离线 diff，要么是独占的实机差分跑（跑厂家 → 退出 → 跑新的 → 比）。

## 6. trace 与差分

**已实现。** 文本格式是行式的（本仓库没有 JSON 依赖，也不为一个只有这个台子读的格式引入）：

```
# shensi can trace v1
# bus 0 = controller CAN1 (left arm), bus 1 = controller CAN2 (right arm)
# columns: t_ns direction bus id len brs fdf data_hex tag
1000 tx 0 601 8 1 1 2b40600006000000 enable
4000 rx 0 301 12 1 1 09e1fde5ff51000000f001c0 feedback
5000 tx 1 080 0 1 1 - sync
```

表头是**自描述**的：总线语义写在文件里，因为左右臂搞反是这里最现实的失败。空数据段写 `-`（同步帧长度为 0）。

```cpp
struct TraceEntry { std::uint64_t t_ns; Direction direction; Frame frame; std::string tag; };

struct NormalizeOptions {
    bool include_tx = true;
    bool include_rx = false;   // 反馈帧带位置/电流/温度，永远不稳定
    bool collapse_consecutive_duplicates = false;   // 见下
    std::int64_t timing_tolerance_ns = -1;          // <0 表示不比较时序
};

enum class DiffKind { MissingCommand, ExtraCommand, PayloadMismatch, BusMismatch,
                      TimingOutOfTolerance, RxMismatch };

DiffResult diff(const Trace& golden, const Trace& actual, const NormalizeOptions& = {});
```

比较算子**不是逐字节相等**：过滤到关注的通道 → 保留相对顺序而非绝对时间 → 按 `(bus, id, len)` 对齐（窗口内先找**完全一致**的帧，找不到才判 payload 不同）→ 比 payload 序列。

三个设计决定各有理由：

1. **`include_rx` 默认 `false`。** 反馈帧的 payload 物理相关，不可能复现；把它当失败会产生无意义的红。
2. **`collapse_consecutive_duplicates` 默认 `false`。** 控制器手册明说**停止需要多点几下**才会生效，所以重复的停止帧可能是真实协议行为而不是重试。默认折叠会掩盖真实差异。
3. **`timing_tolerance_ns` 默认关闭。** 真实周期从没测过（`can-protocol-comparison.md` §8.4），给个阈值等于编。

`BusMismatch` 是独立一类，不是"缺帧 + 多帧"：同 id 同长度但换了总线，是左右臂互换，必须一眼看出来。

**必须从定义好的初始状态抓。** DS402 序列依赖关节当前状态字（`0x06` 是否先于 `0x0F`）。

配套两个后端：

| 类 | 用途 |
|---|---|
| `RecordingTransport` | 装饰器，套在任意 `Transport` 外面，记录双向。**记录用的 receiver 在构造时装好**，不依赖使用者是否注册了 receiver |
| `ReplayTransport` | 回放 golden 语料：`replay_rx()` 注入录到的反馈，`sent_trace()` 交出被测栈发出的帧，直接喂给 `diff()` |

差分回路长这样：

```cpp
ReplayTransport bus(golden);
drive_your_implementation(bus);          // 它发的帧进了 bus.sent()
bus.replay_rx();                         // 把录到的反馈喂进去
DiffResult result = diff(golden, bus.sent_trace());
```

## 7. 离线就能写的测试

- 全帧类 `encode(decode(x)) == x` 往返
- **照 PR0002 写 golden vector**（从文档写，不从二进制写），包括文档自带的例子：
  - SDO：`2B 40 60 00 06 00 00 00`、`2B 40 60 00 07 00 00 00`、`2B 40 60 00 0F 00 00 00`
  - 单轴：`C2 00 00 07 D0 00 05` → PP、目标 0、加减速 2000、速度 5
  - 反馈：`09 E1 FD E5 FF 51 00 00 00 F0 01 C0` → +13.89°、-539 RPM、-175 mA、无故障、24.0 °C、PP、使能+抱闸释放+运行中
  - 7 轴广播：`0x200` 那条 64 字节示例
- 字节序断言：SDO 索引小端（`0x6040` → `40 60`）、控制帧目标大端（16384 → `40 00`）
- `FakeTransport` 驱动的时序测试

## 8. 待解项（需真机帧或二进制定案）

`can-protocol-comparison.md` §8 四项：

1. `0x200` 逐字节组包
2. `0x110` MIT 单轴 9 字节顺序（12 位字段跨字节，容易错位）
3. 反馈 `byte[10]` / `byte[11]` 在驱动里的落地字段
4. 实际控制周期（`SLEEP_TIME` 是 200000 ns，实际循环周期要实测）

写 wire 层时又钉出四条**文档自身**的问题，都已经写成可执行断言（`cpp/tests/test_wire.cpp`
的 `pr0002_documented_anomalies` 与 `frame_classification_and_device_ids`），不会随时间被遗忘：

1. **§5.3 的 7 轴示例控制字节是 `0xD0`**，按 §5.1 位域解出 mode = **8**，而模式表只有 1–7；
   该示例的加速度与速度字段还都是 0，不像一条 PP 指令。
2. **§4.7 的限位子索引是反的**：设正限位用 sub `02`、读正限位用 `01`；设负限位用 `01`、
   读负限位用 `02`。CiA 402 里 `607Dh:01` = min、`:02` = max，所以两行「读」看起来写反了。
3. **§7 有一条应答的子索引与请求不一致**：请求 `2F 00 16 00`（清空 rxPDO1，sub `00`），
   应答 `60 00 16 01`（sub `01`）。§7 是 PDO 配置的 golden 来源，用之前得逐对核。
4. **§5.2 出现一条 `0x081` DLC 8 的「简单反馈」帧**，文档没有任何地方定义这个标识符。
   `classify()` 判为 `Unknown` 而不是去猜。

另外 `0x108` 的标识符歧义见 §3.7。

## 9. 拍板状态

| # | 事项 | 状态 |
|---|---|---|
| 1 | RX 边界：原始帧 | **已定**：原始帧 |
| 2 | 允许链接厂家 `.so` | **已定**：允许，但只作 TX / 时序 / capture |
| 3 | 公共 API 兼容 `Juxie::ControllerJuxie` | **已定**：兼容 |
| 4 | 原始帧从哪来（shm / SocketCAN / USB-CAN） | **未定**，等真机 `ls /sys/class/net` |

## 10. 控制器手册带来的信息

`vendor/originals/documents/controller-user-manual.pdf`（整理稿：
`research/vendor-derived/document-text/controller-user-manual.md`）是**控制器**那一层的文档，
本仓库此前只有关节模组那一层。其中三条直接影响 L0 / L1 / L2：

1. **左臂 CAN1、右臂 CAN2。** 这是任务 3.3 缺的那一半。⚠️ 但手册用 **1 基**的 `CAN1`/`CAN2`，
   而 `rk3576_can_canfd.h` 和 `/dev/misc_shm_can*` 用 **0 基**的 `CAN0`/`CAN1`；若两者对应，
   则**左臂 = `Bus::Can0`**。这个推断必须真机确认——它是左右臂互换最可能的来源，所以
   trace 表头把它写进文件，`diff()` 也把 `BusMismatch` 单列一类。
2. **应用层错误码 14 个**（`0x01<NN>0001` 掉线 / `0x01<NN>0002` 报错 / `0x02<NN>0001` 超限位，
   `NN` = `01`..`0e`）。14 = 左 7 + 右 7，说明 **17 关节里的腰和头不在这 14 个 CAN 电机里**——
   `JointSpaceData` 的 17 维与总线上的 14 个设备不是一一对应。
3. **`executor.yml` 的 `MotorDirect` 是逐轴且 load-bearing 的**：手册明说 `movel` 轨迹不直时
   要对照 URDF 检查各轴正转方向。⚠️ 手册截图里的符号序列与启动日志里的那行 14 个 `±1`
   对不上，至少一处读错，真机要实测。

还有两条属于安全/行为，记在这里以免丢：

- **循环运动**按钮点多次会导致点位异常，停止需要多点几下才生效——这是
  `collapse_consecutive_duplicates` 默认为 `false` 的直接依据。
- 板子上部署根是 `/home/root/DualArm`，配置在 `usr/etc`，且**应该自启动**（用 `top` 验证）。

## 本文件的历史

- v0：初稿。记录语言（C++ 一等公民 + C ABI + Python ctypes）与总线独占两个决定。
- v1：补上 RX 边界（原始帧）、厂家 `.so` 允许链接、公共 API 兼容三个决定；wire + transport
  落地并跑通（`cpp/`）。新增 §8 的文档矛盾清单。
- v2：trace / 录制回放 / 归一化差分落地（§6）；新增 §10，记录控制器手册带来的
  总线↔臂映射、14 个应用层错误码、`MotorDirect` 三条信息。
- v3：开发计划改为 L0–L5 六层（`development-plan.md` v1）。同步更新本文里对 L1 / L2 的引用；
  新增 L3 的接口草案 [`l3-executor-interface.md`](l3-executor-interface.md)。
