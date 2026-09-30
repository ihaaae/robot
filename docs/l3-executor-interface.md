# L3 Executor 接口草案

> **草稿 v0。** 和 [`l0-interface.md`](l0-interface.md) 一样：先把边界和签名冻结，让 L4 / L5
> 能对着一个假 executor 开工，不必等 L1 / L2 做完。内容会变；变的时候改这一份。
>
> 依据：厂家 `bot_executor::ExecutorBase` / `ExecutorJuxie`（头文件 + 符号表 + 配置）、
> [`can-protocol-comparison.md`](can-protocol-comparison.md)、[`hardware-acceptance.md`](hardware-acceptance.md)。
> 相关：[`development-plan.md`](development-plan.md)（分层与任务 7）。
>
> 下文凡标「厂家」的结论，都出自 `vendor/sdk/dual-arm-app/0.6.4/usr/include/`、
> `research/evidence/syms/libexecutor.so.0.6.4.syms` 和 `usr/etc/*/executor_arm.yml` 的静态阅读，
> 没有在真机上确认过。

## 0. 为什么要单独一层

L3 回答的是：**谁拥有时钟。**

在 v0 计划里，这件事被分散在四个任务里：看门狗在 2.5，反馈新鲜度在 3.4，`0x200` 组包在 5.1，
实际控制周期在 1.5。但这四件事不能分开做，理由有三条：

1. **看门狗靠周期控制帧喂**（< 500 ms，否则关节自锁；`can-protocol-comparison.md` §7.3）。
   「每一拍都有一帧出去」必须由**唯一一个**组件保证。如果交给上层，流式控制停止喂点的时候、
   离线规划结束一条轨迹的时候、上层线程卡住的时候，都会断流。
2. **反馈是控制帧触发的。** 驱动不发 `0x80` 同步帧（`can-protocol-comparison.md` §1），所以
   `0x300` 反馈的节奏取决于 TX 的节奏。新鲜度判定和发送节拍本来就是同一个循环的两面。
3. **最后一道安全门只能放在唯一的出口。** 流式控制（`MoveJ_Canfd`）不经过离线规划器。限位如果
   只放在规划器里（v0 计划的 6.5），流式路径就没有任何兜底。

厂家也是这么切的：`ExecutorJuxie` 一个类同时拥有每通道的发送线程（`sendCommandThread0/1`）、
状态监听线程（`listenStateThread`）、`watchdog` 线程、每通道一个命令队列
（`ThreadSafeDeque<Eigen::VectorXd> motor0_commands / motor1_commands`）、限位检查
（`checkJointsWithLimits`）和限速（`JointVelocityPlanner`）。上层的 `ArmServoMode`（在线）和
`PathPlanner`（离线）都只通过 `ExecutorBase` 这个接口往下走。

## 1. 边界

**拥有**

- 固定节拍的循环：每一拍组包并发出控制帧，没有新设定点时发保持帧
- 接收 `0x300` 反馈，维护带时间戳的关节快照
- 反馈新鲜度与总线健康判定
- **最后一道安全门**：位置限位、单拍步长（速度）限制
- 运动源仲裁：同一时刻只允许一个运动源写设定点
- 有界停止（`halt`）：不依赖 L4 仍然活着
- 在节拍线程上执行 L1 的轴请求（使能、失能、清错、抱闸），与控制帧串行

**不拥有**

| 事情 | 归谁 |
|---|---|
| 轨迹怎么生成、稀疏点怎么插值 | L4 |
| 机器人状态机（`power_off / ready / idle / running / fault`） | L5 |
| DS402 序列本身（先发什么、等什么应答） | L1。L3 只负责按拍调用它 |
| 单轴的方向、零偏、单位换算 | L1 |
| 关节索引 ↔ `Dev_ID` ↔ 总线 | L2，作为配置注入 |
| 运动学 | 任务 4 |
| 17 维 `JointSpaceData` 与 14 个设备的对应 | L5（腰和头不在这 14 个 CAN 设备里，见 `l0-interface.md` §10） |

**L3 是 L0 之上唯一带线程、带时钟的层。** L1 和 L2 都做成无线程、无时钟的库，由 L3 在节拍里
调用。这样 L1 / L2 能完全离线地做单测，时序问题也只需要在一处审查。

## 2. 节拍

### 2.1 周期：可配置，不写死

周期没有定论，候选值有三个，而且互相不一致：

| 来源 | 值 | 说明 |
|---|---|---|
| `ExecutorJuxie.hpp`：`double resample_delta {0.005}; //200hz` | 5 ms | 头文件里的默认值 |
| `usr/etc/juxie_73/executor_arm.yml`：`Resample: 0.002` | **2 ms** | 本机型（`params.yml` 里 `type: "ARM_73"`）的配置。`juxie_53` / `62` / `62KML` 都是 0.005 |
| 驱动里的 `SLEEP_TIME` | 0.2 ms | 驱动收发循环里的休眠。它是否等于控制周期没有证据，实际循环周期要实测（`hardware-acceptance.md` P1-1）。本文**推测**它不是：200 µs 与两份 yml 的量级都对不上 |

`Resample` 究竟是「轨迹重采样步长」还是「发送周期」，或者两者都是，还没有确认。所以
**周期是构造参数**，默认取 2 ms（与本机型的配置一致），真机测过之后再定。L4 不得假设周期，
一律通过 `tick_period()` 读取。

### 2.2 线程模型

推荐：**一个节拍线程驱动两条总线**，再加 transport 自带的 RX 线程。RX 线程只做一件事：写快照。

厂家是每通道一个发送线程。我们不照搬，原因是双臂协调运动需要同一个时基：两条线程各自调度，
左右臂的设定点会有不确定的相对相位。代价是一条总线出问题时会拖累另一条，这一点由
§4 的每臂独立判定来弥补。**这是一个可以推翻的决定**：如果真机上两条总线的发送耗时相差很大，
就改回每通道一个线程。

厂家的 `watchdog` 线程在我们这里没有对应物：节拍线程自己检测误拍（`missed_ticks`），关节模组
自己的 500 ms 看门狗是硬件兜底。另起一个线程去「补发」只会制造第二个总线主人，这正是
`l0-interface.md` §5 禁止的情况。

### 2.3 一拍里做什么

```
tick(now):
  1. 取 RX 快照（上一拍以来到达的 0x300 / 0x580）
  2. 新鲜度判定 → 每臂健康状态（§4）
  3. 让 L1 前进一步：每个轴的序列器看到新反馈后，可能产出 SDO / 0x100 帧
  4. 取设定点：当前运动源的下一个采样；没有就保持上一拍的目标
  5. 安全门：限位 → 步长（§5）
  6. L1 把 rad 换成 cnt（叠加方向与零偏），L2 给出每个关节的 (bus, Dev_ID)
  7. 组 0x200，发出；再发 L1 在第 3 步产出的帧
  8. 记录本拍：是否误拍、是否有步长被截断
```

SDO 应答也在这个循环里处理，与厂家一致：`ucas_can0_task_send_thread` 里比较的正是 `0x580`
（`can-protocol-comparison.md` §1）。也就是说，厂家同样是由发送线程串行处理 SDO 请求和应答，
而不是另开一条线程。

## 3. 接口

```cpp
namespace shensi::exec {

inline constexpr std::size_t kArmJoints = 7;
inline constexpr std::size_t kJoints = 2 * kArmJoints;   // 14：左 0..6，右 7..13

using JointVector = std::array<double, kJoints>;          // 关节空间，rad，已含方向与零偏
using JointMask   = std::bitset<kJoints>;

enum class Part : std::uint8_t { Left, Right, Both };

// 与 bot_common::ErrorCode 同号，L5 可以原样透传给 ControllerJuxie 兼容 API。
enum class Status : int {
    Ok                    =    0,
    Error                 =   -1,
    TargetExceedsLimits   =   -4,    // ArmJointTargetExceedLimits
    BusWrong              =   -7,    // ArmJointBusWrong
    VelocityExceedsLimits =  -10,    // ArmJointVelExceedLimits
    NotEnabled            =  -19,    // ArmNotEnabled
    CommandStep           =  -40,    // ArmJointCmdsStep
    DroppedFrames         =  -42,    // ArmJointConnectionDropFrames
    Moving                = -103,    // ArmMoving：已有另一个运动源
    EmergencyStop         = -104,
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
    JointVector commanded{};            // 本拍实际发出的目标（安全门之后）
};

// 采样好的轨迹：第 k 个点在 start + k * tick_period() 下发。
// L4 负责按 tick_period() 采样；L3 不做插值。
struct SampledTrajectory {
    std::vector<JointVector> points;
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
    // 流式：最新值覆盖（latest wins），下一拍生效。L4 负责把 10–50 Hz 的输入插值成每拍一个点。
    virtual Status stream(const JointVector& target, JointMask mask) = 0;
    // 离线：整条轨迹入队。已有轨迹在跑时返回 Moving。
    virtual Status execute(SampledTrajectory trajectory) = 0;
    // 有界停止：按配置的最大减速度把速度降到 0，然后保持。不依赖 L4。
    virtual Status halt(Part) = 0;
    virtual bool   motion_active(Part) const = 0;

    // ---- 状态 -----------------------------------------------------------
    virtual Snapshot snapshot() const = 0;               // 无锁或短锁；可在任意线程调用
    virtual std::chrono::nanoseconds tick_period() const = 0;
};

}  // namespace shensi::exec
```

### 3.1 与厂家 `ExecutorBase` 的对照

| 厂家 | 这里 | 差异 |
|---|---|---|
| `sendServo(arm_joint, part)` | `stream(target, mask)` | 用 mask 代替 `-100` 哨兵 |
| `move(traj)` + `executeCurrentTrajectory(..., renew_time, consecutive)` | `execute(trajectory)` | 不接收连续轨迹对象，只接收采样好的点；续接（renew）交给 L4 |
| `waitForFinish()` / `waitForFinishSignal(t)` | `motion_active(part)` | 不提供阻塞等待。阻塞语义归 L5（`MoveJ` 阻塞是厂家 API 的约定，不是 executor 的） |
| `getCurrentJointValues()` / `getCurrentJointError()` / `getJointFault()` / `getCurrentSingleTorques()` | `snapshot()` | 一次拿到一致的一整份，而不是四次调用拿到四个不同时刻的值 |
| `isMoving()` / `isInFault()` / `isEnabled()` / `isConnected()` | `snapshot()` 里的字段 | 同上 |
| `setEnableForJoint(bool)` / `clearErrorsForJoint()` / `disableServo()` | `request_enable` / `request_clear_faults` | 按臂，不是全体 |
| `BreakEngage` / `BreakRelease` | `request_brake(mask, engage)` | |
| `getControlFrequency()` | `tick_period()` | |
| `SetSending(bool, part)` | 无 | 语义不明。L3 总是在发；「不发」就等于让看门狗自锁 |
| `MoveEnd(v, part)` | 无 | 未文档化的 `0x108` 帧；需要时由 L5 通过 `send_raw` 走，见 `l0-interface.md` §3.7 |
| `getDof()` / `getJointNames()` / `getHomeJointValues()` | 无 | 配置，不是运行时状态；归 L2 |
| `setJointZeroPosition()` | 无 | 标定操作，须先失能（任务 2.3）；由 L5 在停拍状态下调 L1 |

## 4. 新鲜度与失效

每臂独立判定。一条臂不新鲜时：

1. 该臂的运动源立即中止（`execute` 的轨迹丢弃，`stream` 的目标作废）
2. 该臂继续发**保持帧**，目标是最后一次新鲜反馈里的位置，以免因为断流而自锁
3. 连续 N 拍仍不新鲜，就认定该臂失效，上报给 L5，由 L5 转入 `fault`

另一条臂不受影响。**N 和新鲜度窗口都未定**，唯一的硬约束是：从最后一次新鲜反馈到判定失效的时间
必须明显小于 500 ms 看门狗，否则关节会先于我们的判定自锁。

**绝不在没有新鲜反馈的情况下下发新目标。** 使能的那一刻，设定点必须以实测位置作种子
（这就是厂家 `-100`「保持当前位置」语义的来源）。

失能状态下是否仍需发控制帧才能拿到反馈，**未知**：反馈由控制帧触发，而一个 `enable = 0` 的子帧
会不会触发反馈，文档没写。真机上第一批要确认的就是这一条。

## 5. 安全门

作用于每一拍即将发出的目标，不论目标来自 `stream`、`execute` 还是 `halt`：

| 检查 | 超限时 | 理由 |
|---|---|---|
| 位置在 `[lower, upper]` 内 | 在 `stream` / `execute` 提交时直接拒绝，返回 `TargetExceedsLimits` | 静默夹紧会让轨迹悄悄变形 |
| 单拍步长 `|Δq| ≤ v_max · T` | 截断到上限，`clamped_ticks` 加一 | 截断只会让运动变慢，不会让它变危险 |
| 连续截断超过 M 拍 | 该臂 `halt`，返回 `CommandStep` | 上层持续给出过快的目标，说明上层有 bug |

限位值**不能**取随包 `params.yml` 里的值：那里 `UseLimit: false`，每轴限位都是 `[1.5, 6.5]`
这样的占位值（下限连零位都不包含）。真实限位要在真机上标定（`hardware-acceptance.md` P0-4）。在那之前，
`SimExecutor` 用 `getConfig()` 在模拟环境里返回的值做测试即可，真实的 `Executor` 构造时必须
显式传入限位，**不提供默认值**。

## 6. 两个假实现

| 假实现 | 在哪一层假 | 给谁用 |
|---|---|---|
| `SimExecutor` | 实现 `Executor` 接口本身。关节理想跟随目标（可选一阶滞后），可注入故障、掉帧 | L4 / L5 开发。不需要 L1 / L2，不需要写 CAN 应答脚本 |
| 虚拟关节模组 | 挂在 `FakeTransport` 的 responder 上：收到 `0x200` / `0x100` 就回 `0x300`，收到 SDO `0x6040` 就回 `0x580` 并推进 DS402 状态，超过 500 ms 没收到控制帧就自锁 | 测真实的 `Executor` + L1 + L2 整条链 |

厂家也有前者：`ExecutorBase::isConnected()` 的注释写着 "the simulation will always return true"。

`SimExecutor` 和真实 `Executor` 必须跑**同一套**接口契约测试，否则 L4 在假实现上验证过的东西，
换到真实实现上可能不成立。

## 7. 离线就能写的测试

- 节拍：用可控时钟驱动，断言每一拍都恰好发出一帧 `0x200`，包括没有运动源的时候
- 保持：`start()` 之后不给任何设定点，发出的目标恒等于最后一次实测位置
- 仲裁：`execute` 进行中调用 `stream`，返回 `Moving`
- 安全门：越限目标被拒绝；过大步长被截断且计数
- 新鲜度：虚拟关节模组停止回复一条臂的反馈，该臂中止并保持，另一条臂照常运动
- `halt`：从最大速度停下所用的拍数有上界
- 看门狗：让节拍线程停 600 ms，虚拟关节模组应进入自锁。这条是用来证明虚拟模组本身是对的

## 8. 待解项

| # | 事项 | 怎么定 |
|---|---|---|
| 1 | 控制周期：5 ms / 2 ms，还是别的 | 真机抓 `0x200` 的帧间隔（`can-protocol-comparison.md` §8） |
| 2 | 失能状态下反馈是否仍由控制帧触发 | 真机：失能后发 `enable = 0` 子帧，看有没有 `0x300` 回来 |
| 3 | 新鲜度窗口与失效拍数 N | 真机测反馈延迟分布后定 |
| 4 | 步长上限 `v_max` 与 `halt` 的最大减速度 | 与 P0-4 限位一起标定 |
| 5 | 单线程双总线是否够用 | 真机测每拍的发送耗时 |
| 6 | `SetSending(bool, part)` 在厂家那边到底做什么 | 反汇编或真机 trace；目前我们选择不提供对应物 |

## 本文件的历史

- v0：初稿。从 v0 开发计划里的 2.5 / 3.4 / 5.1 / 6.5 抽出「节拍、新鲜度、组包、安全门」合成一层；
  接口对照厂家 `ExecutorBase`；新增两个假实现。
- v0.1：更正两处——占位限位是 `[1.5, 6.5]`（不是 `1.5 / 6.5` 的含糊写法）；`SLEEP_TIME`
  不是控制周期只是推测，不是 `hardware-acceptance.md` 的结论。
