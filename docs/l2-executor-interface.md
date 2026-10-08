# L2 Executor 接口草案

> **草稿。** 和 [`l0-interface.md`](l0-interface.md) 一样：先把边界和签名冻结，让 L3 / L4
> 能对着一个假 executor 开工，不必等 L1 做完。内容会变；变的时候改这一份。
>
> 依据：PR0002、[`hardware-facts.md`](hardware-facts.md)（下文 `HF x.y` 指它的第 x.y 行）、
> [`hardware-acceptance.md`](hardware-acceptance.md)。
> 相关：[`ARCHITECTURE.md`](../ARCHITECTURE.md)（分层）、[`development-plan.md`](development-plan.md)（L2 的任务是 0D.1、1C、1E.1）。
>
> 这是**我们自己的**接口。厂家 `ExecutorBase` 的职责边界和这一层大致对应，对照放在附录 A，
> 只作参考；正文的决定不以「和厂家一致」为理由。

## 0. 为什么要单独一层

L2 回答的是：**谁拥有时钟。**

看门狗、反馈新鲜度、`0x200` 组包、实际控制周期，这四件事不能分开做，理由有三条：

1. **看门狗靠周期控制帧喂**（约 500 ms，否则关节自锁；HF 4.2）。
   「每一拍都有一帧出去」必须由**唯一一个**组件保证。如果交给上层，流式控制停止喂点的时候、
   离线规划结束一条轨迹的时候、上层线程卡住的时候，都会断流。
2. **反馈是总线上的帧触发的。** 按 PR0002，`0x300` 由同步帧 `0x80`、多控报文或单轴报文触发，
   模块不会自己周期上报（HF 2.6）。所以反馈什么时候回来，取决于我们什么时候发什么。新鲜度判定和发送节拍
   本来就是同一个循环的两面。
3. **最后一道安全门只能放在唯一的出口。** 流式控制不经过离线规划器。限位如果
   只放在规划器里，流式路径就没有任何兜底。

厂家 SDK 也有对应的一层（`ExecutorJuxie`），说明这个切法在这台硬件上可行；细节见附录 A。

## 1. 边界

**拥有**

- 固定节拍的循环：每一拍组包并发出控制帧，没有新设定点时发保持帧
- 接收 `0x300` 反馈，维护带时间戳的关节快照
- 反馈新鲜度与总线健康判定
- **最后一道安全门**：位置限位、单拍步长（速度）限制
- 运动源仲裁：同一时刻只允许一个运动源写设定点
- 有界停止（`halt`）：不依赖 L3 仍然活着
- 轴请求（使能、失能、清错、抱闸）：由 L1 落成每拍子帧控制字节里的位，随 `0x200` 发出（§2.3）
- 构造时注入的整机配置：关节 ↔ (bus, `Dev_ID`) 映射、限位、周期（§3.2）

**不拥有**

| 事情 | 归谁 |
|---|---|
| 轨迹怎么生成：平滑、速度 / 加速度 / 加加速度约束、把 10–50 Hz 的稀疏输入加密成带时间戳的点 | L3。L2 只在相邻两点间线性插值（§3.3） |
| 机器人状态机（`power_off / ready / idle / running / fault`） | L4 |
| 每轴的使能 / 抱闸 / 清错逻辑（本拍置哪些位、等哪个状态位、SDO 诊断） | L1。L2 只负责按拍调用它 |
| 单轴的方向、零偏、单位换算 | L1 |
| 运动学 | 2A |

**L2 是 L0 之上唯一带线程、带时钟的层。** L1 做成无线程、无时钟的库，由 L2 在节拍里
调用。这样 L1 能完全离线地做单测，时序问题也只需要在一处审查。

## 2. 节拍

### 2.1 周期：可配置，实测后定

周期由实测决定，不照搬厂家的配置。和周期有关的事实：

| 事实 | 值 | 出处 |
|---|---|---|
| 厂家栈在这台硬件上用的周期 | 2 ms（只说明「这样能跑」） | HF 8.8 |
| 看门狗 | 约 500 ms，来源冲突 | HF 4.2 |
| 「位置跃迁过大」 | CSP 下单帧 Δ > 500 cnt 报错（文档按 1 ms 周期写） | HF 4.4 |

最后一条给出单拍步长的硬件上限：2 ms 时约 500/65536 × 2π / 0.002 ≈ 24 rad/s，远高于任何合理的
关节限速，不是瓶颈；但安全门的步长上限（§5）必须始终低于它。

**带宽不是瓶颈。** 按 1 Mbps 仲裁段 / 5 Mbps 数据段（HF 1.1）粗估，每条总线每拍的流量是一帧 64 字节的
`0x200`（约 0.15–0.2 ms）加 7 帧 12 字节的 `0x300`（每帧约 0.05–0.06 ms），合计约 0.6 ms。按 2 ms 一拍算，
总线负载约 30%。两条总线是各自独立的物理总线，SocketCAN 和 USB-CAN 的吞吐都远高于这个量。

所以周期要测的不是吞吐，而是**端到端时延与抖动**：发出 `0x200` 到收齐 7 帧 `0x300` 要多久、分布多宽。
主机调度和后端的批量收发都会影响这一项，具体取决于后端。周期选在这段时延的上尾之外，留出余量。

**周期是构造参数**，起始值 2 ms，实测后改。L3 不需要知道周期：它交的是带时间戳的点，按拍取样是 L2 的事（§3.3）。

### 2.2 线程模型

推荐：**一个节拍线程驱动两条总线**，再加 transport 自带的 RX 线程。RX 线程只做一件事：写快照。

理由是双臂协调运动需要同一个时基：每条总线一个线程各自调度时，
左右臂的设定点会有不确定的相对相位。代价是一条总线出问题时会拖累另一条，这一点由
§4 的每臂独立判定来弥补。**这是一个可以推翻的决定**：如果真机上两条总线的发送耗时相差很大，
就改回每通道一个线程。

不设单独的看门狗线程：节拍线程自己检测误拍（`missed_ticks`），关节模组自己的 500 ms 看门狗是
硬件兜底。另起一个线程去「补发」只会制造第二个总线主人，这正是 `l0-interface.md` §5 禁止的情况。

### 2.3 一拍里做什么

```
tick(now):
  1. 取 RX 快照（上一拍以来到达的 0x300 / 0x580）
  2. 新鲜度判定 → 每臂健康状态（§4）
  3. 让 L1 前进一步：每个轴按请求的状态和反馈里的状态位，给出本拍控制字节；需要诊断时产出 SDO
  4. 取设定点：按本拍时刻在当前运动源的相邻两点间线性插值（§3.3）；没有运动源就保持上一拍的目标
  5. 安全门：限位 → 步长（§5）
  6. L1 把 rad 换成 cnt（叠加方向与零偏），映射表（§3.2）给出每个关节的 (bus, Dev_ID)
  7. 组 0x200（每个子帧 = 控制字节 + 目标），发出；再发第 3 步产出的 SDO
  8. 记录本拍：是否误拍、是否有步长被截断
```

**使能、失能、清错、抱闸都走控制字节（PR0002 §5.1），不走 SDO。** 它们是 `0x200` 子帧第一个字节里的位
（HF 2.2），每拍随目标一起发出，再用反馈 `byte[11]` 的状态位确认（HF 2.5）。这样轴请求和控制帧天然串行，
不需要额外的同步。代价是 L1 的这部分进了每拍的热路径，所以它必须是纯计算、不阻塞。
只靠控制字节能否完成使能、清错位是电平还是边沿触发，都还是未知（HF 4.6、4.7），真机第一批确认。

SDO 只用于配置和诊断：读故障码、零位标定、通信参数。应答同样在这个循环里处理，要读，不能只发（HF 8.2）。

**位置模式用 CSP**（HF 2.4）。上层每拍给一个目标位置，这正是 CSP 的语义；PP 模式自带轨迹规划，
会和 L3 的插值叠在一起，两层规划器的行为很难推理。

**空闲拍发什么。** 有两种都能让总线上每拍有帧、让模块回 `0x300` 的做法：发目标为当前位置的
`0x200` 保持帧，或者只发 `0x80` 同步帧（HF 2.7、8.1）。我们选保持帧，理由是它同时把「保持在哪」
说清楚了，而且不依赖「只发 `0x80` 也算喂狗」这条未知事实（HF 4.3）。
**这是可以推翻的决定**：如果真机上保持帧会带来抖动，就改成发 `0x80`。

## 3. 接口

```cpp
namespace shensi::exec {

inline constexpr std::size_t kArmJoints = 7;
inline constexpr std::size_t kJoints = 2 * kArmJoints;   // 14：左 0..6，右 7..13

using JointVector = std::array<double, kJoints>;          // 关节空间，rad，已含方向与零偏
using JointMask   = std::bitset<kJoints>;

enum class Part : std::uint8_t { Left, Right, Both };

// L2 自己的结果码。L4 映射到对外错误码；这里不承诺任何数值。
enum class Status : std::uint8_t {
    Ok,
    NotRunning,          // start() 之前或 shutdown() 之后
    InvalidArgument,     // 空轨迹、空 mask、NaN 等
    NotEnabled,          // 目标臂未全部使能
    Busy,                // 该臂已有另一个运动源
    TargetOutOfLimits,   // 目标越过位置限位（§5）
    StepLimitHalted,     // 持续步长截断，安全门已让该臂 halt（§5）
    ArmStale,            // 该臂反馈不新鲜或已判失效（§4）
    BusFault,            // transport 不健康或 bus-off
};

struct JointSample {
    // 物理量，由 L1 从 JointFeedback 换算（cnt → rad、RPM → rad/s、方向、零偏）
    double   position = 0;        // rad
    double   velocity = 0;        // rad/s
    double   current  = 0;        // A
    double   temperature = 0;     // degC
    std::uint16_t fault = 0;      // 原始故障码；映射在 L1
    std::uint8_t  mode = 0;
    bool enabled = false, brake_released = false, error = false, in_position = false;

    std::uint64_t stamp_ns = 0;   // 这帧反馈到达的时刻（单调时钟）
    bool valid = false;           // 从未收到过就是 false。不用 -100 这种哨兵
};

struct ArmHealth {
    bool fresh = false;                 // 所有关节反馈都在新鲜度窗口内
    bool bus_ok = false;                // transport healthy() 且没有 bus-off
    std::uint64_t stale_ticks = 0;      // 连续不新鲜的拍数
};

struct Snapshot {
    std::array<JointSample, kJoints> joints;
    ArmHealth left, right;
    std::uint64_t tick = 0;
    std::uint64_t missed_ticks = 0;     // 累计误拍
    std::uint64_t clamped_ticks = 0;    // 累计步长被截断的拍数
    std::uint64_t stream_underruns = 0; // 累计流断档次数：取样时刻越过了最后一个点（§3.3）
    JointVector commanded{};            // 本拍实际发出的目标（安全门之后）
};

// 带时间戳的点。L2 每拍按本拍时刻在相邻两点间线性插值（§3.3）。
struct TimedPoint {
    std::chrono::nanoseconds t{};       // 含义见 Trajectory 和 stream()
    JointVector q{};
};

// 相邻两点的最大间隔。线性插值误差因此可以忽略（§3.3）；平滑和加速度约束都归 L3。
inline constexpr std::chrono::nanoseconds kMaxPointSpacing = std::chrono::milliseconds(10);

// 离线轨迹。t 相对轨迹起点：points[0].t == 0，严格递增，相邻间隔 ≤ kMaxPointSpacing。
// 起点是 execute() 被接受后的下一拍。
struct Trajectory {
    std::vector<TimedPoint> points;
    JointMask mask;                     // 未选中的关节保持当前位置
};

class Executor {
public:
    virtual ~Executor() = default;

    // ---- 生命周期 -------------------------------------------------------
    virtual Status start() = 0;         // 开始打拍。成功后每一拍都有控制帧出去
    virtual void   shutdown() = 0;      // 先 halt，再失能，最后停拍

    // ---- 轴请求（交给 L1 的序列器，在节拍线程上执行）-----------------------
    virtual Status request_enable(Part, bool enable) = 0;
    virtual Status request_clear_faults(Part) = 0;
    virtual Status request_brake(JointMask, bool engage) = 0;

    // ---- 运动源 ---------------------------------------------------------
    // 流式：追加一个点。point.t 是单调时钟（steady_clock）上的绝对时刻，
    // L2 在 point.t + stream_delay 那一拍播放它，和 execute 用同一个插值器（§3.3）。
    // 同一段流内 t 严格递增、相邻间隔 ≤ kMaxPointSpacing，mask 不变。
    virtual Status stream(const TimedPoint& point, JointMask mask) = 0;
    // 离线：整条轨迹入队。已有轨迹在跑时返回 Busy。
    virtual Status execute(Trajectory trajectory) = 0;
    // 有界停止：按配置的最大减速度把速度降到 0，然后保持。不依赖 L3。
    virtual Status halt(Part) = 0;
    virtual bool   motion_active(Part) const = 0;

    // ---- 状态 -----------------------------------------------------------
    virtual Snapshot snapshot() const = 0;               // 无锁或短锁；可在任意线程调用
    virtual std::chrono::nanoseconds tick_period() const = 0;
};

// ---- 构造配置（§3.2）-------------------------------------------------------
struct JointAddress {
    can::Bus      bus;
    std::uint8_t  dev_id;
};

struct ExecutorConfig {
    std::array<JointAddress, kJoints> joints;   // 关节 i 在哪条总线、哪个 Dev_ID。无默认值
    std::array<l1::AxisConfig, kJoints> axes;   // 方向、零偏、力矩常数（1B.4）
    JointVector lower, upper;                   // 位置限位（§5）。无默认值
    double v_max = 0;                           // 单拍步长上限对应的速度，rad/s（§5）
    std::chrono::nanoseconds tick_period{};     // §2.1
    std::chrono::nanoseconds stream_delay{};    // stream() 的固定延迟（§3.3）
};

}  // namespace shensi::exec
```

### 3.1 L4 状态机从这里读什么

L4 的机器人状态从 L2 推导（`development-plan.md` 2D.2），不另起轮询线程。所以上面的接口必须能回答
这几个问题：

| L4 要知道 | 这里 | 备注 |
|---|---|---|
| 有没有运动在执行 | `motion_active(part)` | |
| 是否全部使能 | `snapshot().joints[i].enabled` | 全部关节都使能才算 |
| 是否有故障 | `snapshot().joints[i].error` / `fault`，以及 `ArmHealth` | 新鲜度失效（§4）也算故障 |
| 是否连着 | `ArmHealth::bus_ok` | 要真的实现，不能恒为真 |

现有字段已经够用，不需要新增接口。要保证的是 `snapshot()` 里这几项取自**同一拍**，否则 L4 可能看到
「已失能但仍在运动」这种不存在的组合。

### 3.2 整机配置

关节 ↔ (bus, `Dev_ID`) 的映射不单列一层。它只是一张表加一个查表函数，没有自己的状态和时序，
所以并入 `ExecutorConfig`，在构造时注入。构造时校验：14 项齐全、(bus, `Dev_ID`) 不重复、
每条总线恰好 7 个关节，否则构造失败。

映射**不提供默认值**，和限位一样。左臂接哪条总线还是推断（HF 1.5，可信度低），一旦配错就是左右臂互换。
真机逐轴点动确认之前，配置文件必须显式写出来。

「是否全部使能、是否有轴报错、是否有轴掉线」这类聚合不再是单独的函数：L2 的 `Snapshot` 和 `ArmHealth`
已经给出了原始数据，聚合由 L4 的状态机在读快照时做（`development-plan.md` 2D.2）。

### 3.3 运动源：带时间戳的点，L2 线性插值

L3 交给 L2 的是**带时间戳的点**，不是「每拍一个点」。L2 每拍拿本拍时刻在相邻两点间线性插值，
这是 L2 唯一的运动原语，`execute` 和 `stream` 共用。

**为什么这样分。** 周期要实测后才定（§2.1），而且会因为后端不同而不同。如果 L3 按 `tick_period()`
采样，L3 就和节拍绑死了：周期一改，L3 的输出要跟着改；流式输入还得有一个和节拍同步的实时线程。
带时间戳以后，L3 只管「什么时刻在哪」，不管节拍，也不需要实时线程。

**分工。** 所有平滑和速度 / 加速度 / 加加速度约束归 L3。L2 只做线性插值，不做任何整形；
安全门（§5）照常对插值结果截断步长。

**点间隔上限 `kMaxPointSpacing` = 10 ms**，提交时校验，超了返回 `InvalidArgument`。
线性插值的误差上界是 a·h²/8：a = 10 rad/s²、h = 10 ms 时约 1.3×10⁻⁴ rad，和一个编码器计数
（2π/65536 ≈ 9.6×10⁻⁵ rad）同一量级，可以忽略。这个上限可以随真机数据调，但它是 L2 和 L3 之间的契约，
不随周期变。

**`execute`。** `t` 相对轨迹起点，`points[0].t == 0`，起点是被接受后的下一拍。
越过最后一个点之后保持在最后一个点，运动源释放。

**`stream`。** `t` 是单调时钟上的绝对时刻，L2 在 `t + stream_delay` 播放它，
也就是每拍拿 `now − stream_delay` 去插值。固定延迟吸收 L3 一侧的抖动，所以 L3 可以成批、
不定时地追加点，只要追加得比播放快。

- 第一个点开始一段新的流，起点应当是当前位置；离得远时由安全门截断，持续截断会 halt（§5）
- 同一段流内 `t` 严格递增、间隔 ≤ `kMaxPointSpacing`、`mask` 不变，否则返回 `InvalidArgument`
- `t` 早于已经播放过的时刻：返回 `InvalidArgument`
- **断档**：取样时刻越过了最后一个点，就保持在最后一个点，`stream_underruns` 加一，这段流结束、
  运动源释放。之后的 `stream` 调用开始新的一段。保持是突然停下，所以 `stream_delay` 要选得让 L3
  正常工作时不断档

`stream_delay` 是构造参数，和 L3 的输出节奏一起定（§8 第 8 项）。

**否决过的方案。**

| 方案 | 为什么不要 |
|---|---|
| 保持「L3 按拍采样」 | L3 和实测周期绑死；流式需要 L3 有一个和节拍同步的实时线程 |
| L3 交一个 `q(t)` 回调，L2 每拍调用 | L3 的代码跑进热路径，耗时没有上界，还可能加锁 |

## 4. 新鲜度与失效

每臂独立判定。一条臂不新鲜时：

1. 该臂的运动源立即中止（`execute` 的轨迹丢弃，`stream` 的目标作废）
2. 该臂继续发**保持帧**，目标是最后一次新鲜反馈里的位置，以免因为断流而自锁
3. 连续 N 拍仍不新鲜，就认定该臂失效，上报给 L4，由 L4 转入 `fault`

另一条臂不受影响。**N 和新鲜度窗口都未定**，唯一的硬约束是：从最后一次新鲜反馈到判定失效的时间
必须明显小于 500 ms 看门狗，否则关节会先于我们的判定自锁。

**绝不在没有新鲜反馈的情况下下发新目标。** 使能的那一刻，设定点必须以实测位置作种子。

失能状态下是否仍需发控制帧才能拿到反馈，**未知**：反馈由控制帧触发，而一个 `enable = 0` 的子帧
会不会触发反馈，文档没写（HF 4.5）。真机上第一批要确认的就是这一条。

## 5. 安全门

作用于每一拍即将发出的目标，不论目标来自 `stream`、`execute` 还是 `halt`：

| 检查 | 超限时 | 理由 |
|---|---|---|
| 位置在 `[lower, upper]` 内 | 在 `stream` / `execute` 提交时直接拒绝，返回 `TargetOutOfLimits` | 静默夹紧会让轨迹悄悄变形 |
| 单拍步长 `|Δq| ≤ v_max · T` | 截断到上限，`clamped_ticks` 加一 | 截断只会让运动变慢，不会让它变危险 |
| 连续截断超过 M 拍 | 该臂 `halt`，此后该臂的提交返回 `StepLimitHalted`，直到上层重新使能 | 上层持续给出过快的目标，说明上层有 bug |

安全门**不可关闭**，也没有「调试时绕过」的开关。

位置限位**没有可用的随包值**（HF 6.3），真实位置限位要在真机上标定（P0-4）。在那之前，
`SimExecutor` 用测试自己给的限位即可；真实的 `Executor` 构造时必须显式传入限位，**不提供默认值**。
步长上限 `v_max` 先取保守值（例如 1.5 rad/s），同样要真机确认；它必须始终低于 §2.1 的硬件上限。

## 6. 两个假实现

| 假实现 | 在哪一层假 | 给谁用 |
|---|---|---|
| `SimExecutor` | 实现 `Executor` 接口本身。关节理想跟随目标（可选一阶滞后），可注入故障、掉帧 | L3 / L4 开发。不需要 L1，不需要写 CAN 应答脚本 |
| 虚拟关节模组 | 挂在 `FakeTransport` 的 responder 上，按 PR0002 模拟关节模组：收控制帧回 `0x300`，按控制字节的使能 / 抱闸 / 清错位改变自身状态，收 SDO 回 `0x580`，超过 500 ms 没收到控制帧就自锁。规格与任务见 `development-plan.md` 0D.2、1A | 测真实的 `Executor` + L1 整条链 |

`SimExecutor` 和真实 `Executor` 必须跑**同一套**接口契约测试，否则 L3 在假实现上验证过的东西，
换到真实实现上可能不成立。

## 7. 离线就能写的测试

- 节拍：用可控时钟驱动，断言每一拍都恰好发出一帧 `0x200`，包括没有运动源的时候
- 保持：`start()` 之后不给任何设定点，发出的目标恒等于最后一次实测位置
- 仲裁：`execute` 进行中调用 `stream`，返回 `Busy`
- 插值：给两个点，断言中间各拍的目标正好落在连线上；点的时刻不和拍对齐时也成立；周期换一个值，同一条轨迹的路径不变
- 时间戳校验：`points[0].t != 0`、不递增、间隔超过 `kMaxPointSpacing` 都返回 `InvalidArgument`
- 流：按 `stream_delay` 延迟播放；成批提前追加和逐个追加结果相同；停止追加后保持在最后一个点、`stream_underruns` 加一、运动源释放
- 安全门：越限目标被拒绝；过大步长被截断且计数
- 新鲜度：虚拟关节模组停止回复一条臂的反馈，该臂中止并保持，另一条臂照常运动
- `halt`：从最大速度停下所用的拍数有上界
- 看门狗：让节拍线程停 600 ms，虚拟关节模组应进入自锁。这条是用来证明虚拟模组本身是对的

## 8. 待解项

| # | 事项 | 怎么定 |
|---|---|---|
| 1 | 控制周期：`0x200` → 收齐 `0x300` 的时延分布、帧间隔抖动 | 真机上用我们自己的后端测，按 §2.1 定周期 |
| 2 | 失能状态下反馈是否仍由控制帧触发 | 真机：失能后发 `enable = 0` 子帧，看有没有 `0x300` 回来 |
| 3 | 新鲜度窗口与失效拍数 N | 真机测反馈延迟分布后定 |
| 4 | 步长上限 `v_max` 与 `halt` 的最大减速度 | 与 P0-4 限位一起标定 |
| 5 | 单线程双总线是否够用 | 真机测每拍的发送耗时 |
| 6 | 空闲拍发保持帧还是 `0x80`（§2.3） | 真机：两种都试，看保持精度与反馈节奏 |
| 7 | 只靠控制字节能否使能、清错位的触发方式（HF 4.6、4.7） | 真机：失能状态下直接置 `enable = 1`，看反馈状态位；清错位分别发一拍和持续多拍 |
| 8 | ~~L3 按 `tick_period()` 采样~~ 已定：带时间戳的点，L2 线性插值（§3.3）。剩下的是 `stream_delay` 的取值和 `kMaxPointSpacing` 是否要收紧 | 用 L3 的真实输入（遥操作 10–50 Hz）测加密后的到达抖动，延迟取在上尾之外 |

## 附录 A：厂家 `ExecutorJuxie` 参考

> 只作参考，不是规格。出自 `vendor/sdk/dual-arm-app/0.6.4/usr/include/`、`usr/etc/*/executor_arm.yml`
> 和反汇编（片段在 `research/evidence/disasm/`）的静态阅读，没有在真机上确认过。

### A.1 结构

`ExecutorJuxie` 一个类同时拥有每通道的发送线程（实际启动的是 `ucas_can0/1_task_send_thread`；
头文件里的 `sendCommandThread0/1` 和 `listenStateThread` 是死代码，没人调用）、`watchdog` 线程、
每通道一个命令队列（`ThreadSafeDeque<Eigen::VectorXd> motor0_commands / motor1_commands`）和每拍的限速
（`JointVelocityPlanner`，`UseLimit` 打开时生效）。`checkJointsWithLimits` 也在这个类里，但只在构造时
调用一次，不在每拍的路径上。上层的 `ArmServoMode`（在线）和 `PathPlanner`（离线）都只通过
`ExecutorBase` 这个接口往下走。

发送周期取 yml 的 `Resample`（本机型 0.002，其他机型 0.005）；驱动里的 `SLEEP_TIME`（0.2 ms）是
驱动层线程的轮询间隔，不决定 `0x200` 的节拍。空闲时发 `0x80`。`params.yml` 的 `[1.5, 6.5]` 是
`JointVelocityPlanner` 的速度 / 加速度上限，不是位置限位。没找到解析 `0x580` 的代码。

和我们的主要差别：我们一个节拍线程驱动两条总线（§2.2）、使能 / 清错 / 抱闸走控制字节而不是 SDO（§2.3）、不设单独的看门狗线程、空闲拍发保持帧
（§2.3）、读 SDO 应答、安全门不可关（§5）、用 mask 而不是 `-100` 哨兵表达「保持」。

### A.2 与厂家 `ExecutorBase` 的方法对照

| 厂家 | 这里 | 差异 |
|---|---|---|
| `sendServo(arm_joint, part)` | `stream(point, mask)` | 用 mask 代替 `-100` 哨兵；点带时间戳，按固定延迟播放，而不是「最新值下一拍生效」 |
| `move(traj)` + `executeCurrentTrajectory(..., renew_time, consecutive)` | `execute(trajectory)` | 不接收连续轨迹对象，只接收间隔 ≤ 10 ms 的带时间戳点（§3.3）；续接（renew）交给 L3 |
| `waitForFinish()` / `waitForFinishSignal(t)` | `motion_active(part)` | 不提供阻塞等待。阻塞语义归 L4（`MoveJ` 阻塞是厂家 API 的约定，不是 executor 的） |
| `getCurrentJointValues()` / `getCurrentJointError()` / `getJointFault()` / `getCurrentSingleTorques()` | `snapshot()` | 一次拿到一致的一整份，而不是四次调用拿到四个不同时刻的值 |
| `isMoving()` / `isInFault()` / `isEnabled()` / `isConnected()` | `snapshot()` 里的字段 | 同上 |
| `setEnableForJoint(bool)` / `clearErrorsForJoint()` / `disableServo()` | `request_enable` / `request_clear_faults` | 按臂，不是全体 |
| `BreakEngage` / `BreakRelease` | `request_brake(mask, engage)` | |
| `getControlFrequency()` | `tick_period()` | |
| `SetSending(bool, part)` | 无 | 它写的两个原子标志（按头文件成员顺序是 `isLeftSending_` / `isRightSending_`）由 `move()` 和 `isMoving()` 读，看起来是「这条臂正在执行轨迹」的状态位，不是开关总线发送。对应物是 `motion_active(part)`，不需要单独的写接口 |
| `MoveEnd(v, part)` | 无 | 发给 Dev_ID 8 的普通单轴速度帧（`can-protocol-comparison.md` §2）。L2 目前不管 Dev_ID 8；要支持时加一个面向它的轴请求，不走 `send_raw` |
| `getDof()` / `getJointNames()` / `getHomeJointValues()` | 无 | 配置，不是运行时状态；归 `ExecutorConfig`（§3.2）和 L4 |
| `setJointZeroPosition()` | 无 | 标定操作，须先失能（1B.2）；由 L4 在停拍状态下调 L1 |

### A.3 厂家状态机读的四个谓词

厂家的机器人状态在轮询线程跑起来之后，实际上是 executor 四个谓词的函数：每 5 ms 按
`isInFault → isMoving → isEnabled` 的优先级判定一次（[`robot-state-machine.md`](../research/vendor-analysis/robot-state-machine.md) §3）。
它们和 §3.1 的四个问题一一对应：

| 厂家谓词 | 这里 | 备注 |
|---|---|---|
| `isMoving()` | `motion_active(Part::Both)` | 厂家是 `isLeftSending_ \|\| isRightSending_` |
| `isEnabled()` | `snapshot().joints[i].enabled` | 全部关节都使能才算 |
| `isInFault()` | `snapshot().joints[i].error` / `fault`，以及 `ArmHealth` | 新鲜度失效（§4）也要算故障，厂家没有这一项 |
| `isConnected()` | `ArmHealth::bus_ok` | 厂家恒返回 `true`，「掉线」从不触发；我们要真的实现 |
