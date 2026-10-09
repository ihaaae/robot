# 架构

> 这份讲**系统由哪些部分组成、各自跑在哪里、为什么这样切**。按 arc42 的视图组织：
> 上下文 → 约束 → 部署视图 → 逻辑视图（分层）→ 运行时视图 → 关键决定 → 待定问题。
>
> 不在这里的内容：
> 需求（用例、功能 / 非功能需求）见 [`docs/requirements.md`](docs/requirements.md)；
> 安全概念（safety limiter、controlled stop、失效处理）见 [`docs/safety-concept.md`](docs/safety-concept.md)；
> 各组件的接口与内部设计见 [`docs/components/`](docs/index.md#our-own-controller-sdk)；
> 两台机器之间的接口见 [`docs/interfaces/rpc.md`](docs/interfaces/rpc.md)；
> 怎么离线验证见 [`docs/verification.md`](docs/verification.md)；
> 阶段、任务与接口冻结计划见 [`docs/development-plan.md`](docs/development-plan.md)。
> 术语见 [`docs/glossary.md`](docs/glossary.md)。

## 1. 上下文

自己实现巨蟹双臂机器人的 controller SDK，跑在关节模组公开的 CAN / CAN-FD 协议（PR0002）上，提供和厂家 SDK
（`dual_arm_app` 0.6.4，`Juxie::ControllerJuxie`）**同一抽象层次的能力**。需求从我们自己的用例推出
（[`requirements.md`](docs/requirements.md)），厂家 SDK 的功能清单只用来查漏。

**对标只在抽象层次上。** 不追求和厂家 SDK 同名的方法、同号的返回码、同样的状态机格子或同样的总线形态。
厂家 SDK 有不少缺陷，内部细节不透明，照着细节做时，模糊甚至自相矛盾的地方反过来在干扰设计。所以：

- **规格的优先级**：PR0002 + 真机实测 > 我们自己的需求 > 厂家 SDK 的行为（只当提示：「这样做至少能跑」）。
- 设计只从 [`docs/hardware-facts.md`](docs/hardware-facts.md) 取硬件事实，不直接引用厂家实现细节。
  和厂家不同的地方，写我们的理由，不写「为什么和厂家不一样」的辩护。
- 厂家 SDK 的分析在 [`research/`](research/README.md)，冻结，是参考材料。

系统边界：

| 外部参与者 | 和系统的交互 |
|---|---|
| 应用（示教、遥操作、策略） | 在外部计算机上，经 L3 和 RPC Client 使用系统 |
| 14 个关节模组 | 两条 CAN FD 总线，每条一条臂、7 个关节；协议 PR0002 |
| 硬件急停回路 | 独立于软件，系统不控制也不依赖它 |
| 外部电源 | 关节供电；PR0002 里没有供电控制，不进我们的 API |

## 2. 约束

外部强加、不由我们选择的条件。设计决定（§6）都要在这些约束之内。

| # | 约束 | 来源 | 推论 |
|---|---|---|---|
| C1 | **一条总线只能有一个 bus master。** SDO `0x600\|id` 无源地址，看门狗靠周期控制帧维持 | PR0002（[`l0-can-io.md`](docs/components/l0-can-io.md) §5） | 我们的栈和厂家栈不能同时挂在总线上，切换是整体的；L4 只允许一个实例、RPC 只允许一个 control session |
| C2 | 关节模组的看门狗：超过约 500 ms 没有控制帧就自锁 | `hardware-facts.md` 4.2（来源冲突） | 总线上必须持续有帧，由唯一一个组件保证 |
| C3 | 反馈只由总线上的帧触发，模块不自己上报 | `hardware-facts.md` 2.6 | 反馈新鲜度和发帧节奏是同一个循环 |
| C4 | 控制板算力不够做完整的运动规划 | 厂家板卡 | 系统分两台机器 |
| C5 | 仓库里还没有任何东西在真机上跑过 | 现状 | 离线的正确性只能对着一个照 PR0002 写的 Joint Emulator 判定（[`verification.md`](docs/verification.md)） |

设计原则（我们选的，不是外部强加的）：**按层切，不按 API 切。** API 表面跨好几层，规格质量差很远：
L1 有完整文档（PR0002），L3 没有任何文档（只能写行为契约）。混在一起做会给出虚假的信心。

## 3. 部署视图

系统分两台机器，**网络边界放在 Executor 接口之上**：

```
外部计算机（上位机）                          控制板（机器人上）
┌──────────────────────────┐              ┌───────────────────────────────────┐
│ 应用：示教、遥操作、策略  │              │ RPC Service                        │
│ L3 Motion Planning        │── 有线以太网 ─▶│  control session、心跳、时间换算  │
│ RPC Client                │◀─ 快照 / 事件 ─│ L4 Control API + 机器人状态机     │
└──────────────────────────┘              │ L2 Executor                        │
                                          │ L1 Joint Driver、L0 CAN I/O        │
                                          └──────────────┬────────────────────┘
                                                         │ CAN FD ×2
                                                      14 个关节模组
```

| 组件 | 节点 | 理由 |
|---|---|---|
| L0 / L1 / L2 | 控制板 | 必须靠近总线，见下文「为什么不经网络转发 CAN 帧」 |
| L4 状态机 | 控制板 | 只能有一个 bus master（C1）；状态由 L2 快照推导，唯一写者要和总线在一起（`development-plan.md` 2D.2） |
| L4 Control API | 控制板，经 RPC Service 暴露 | 外部程序都通过它访问机器人 |
| L3 离线规划、streaming smoothing、运动学 | **外部计算机**（v1） | 重的规划本来就在外面；L3 只往 L2 交带时间戳的点，不需要知道周期，也不需要实时线程（`l2-executor.md` §3.3），所以隔一层网络不改接口 |

外部程序（含 Python）一律经 RPC 访问，不提供 C ABI 绑定——调用者在另一台机器上，进程内绑定用不上。
RPC 的契约（操作、时间换算、断连、control session）见 [`rpc.md`](docs/interfaces/rpc.md)。

L3 v1 整体放在外部，板上就不需要 IK 和规划代码。如果实测发现 streaming 经网络的到达抖动大到
`stream_delay` 不可接受，再把 streaming smoothing（2B.1）搬到板上，让网络上只走 10–50 Hz 的稀疏设定点。
接口不变，只是加密那一步换了位置（§7 Q1）。

### 为什么不经网络转发 CAN 帧

考虑过的方案是外部机器算出每个 tick 的 `0x200`，经网络（例如 SocketCAN over Ethernet）送到板上的总线。
不采用，理由都来自 L2 那个循环必须靠近总线：

| 事实 | 放到外部的后果 |
|---|---|
| 关节模组看门狗：PR0002 写帧间隔超过 500 ms 就自锁，厂家 V1.0.1 写告警级、可配（`hardware-facts.md` 4.2，来源冲突，1A.4 做成开关） | 按自锁算：网线松动、Wi-Fi 抖动、上位机卡顿都会直接让关节自锁 |
| 反馈只由总线上的帧触发，模块不自己上报（2.6） | 新鲜度判定和发帧节奏是同一个循环；中间夹一层网络，网络时延和抖动就进了控制周期 |
| CSP 下单帧 Δ > 500 cnt 报「位置跃迁过大」（4.4；文档按 1 ms 周期写，本机型周期 2 ms，见 8.8） | 网络丢几帧再补上，就可能触发 |
| safety limiter 在 L2，且不可关闭（[`safety-concept.md`](docs/safety-concept.md)） | 板上没有兜底；断网时没有人能做 controlled stop |

这个方案只适合台架调试，不作为部署形态。

## 4. 逻辑视图：分层

```
外部计算机   应用
             L3 Motion Planning：streaming smoothing、离线规划、FK / IK
             RPC Client
──────────── 网络边界 ────────────
控制板       RPC Service
             L4 Control API：我们自己的 API、错误码、机器人状态机
             L2 Executor：tick、watchdog keep-alive、新鲜度、safety limiter、joint mapping   ← 唯一拥有时钟的层
             L1 Joint Driver：控制字节、诊断、方向 / 零偏、单位                     ← 无线程，由 L2 每个 tick 调用
             L0 CAN I/O：wire codec + Transport + trace
──────────── CAN FD ×2 ────────────
             14 个关节模组
```

| 层 | 职责 | 线程 / 时钟 | 文档 | 状态 |
|---|---|---|---|---|
| L0 CAN I/O | 字节 ↔ 线上结构体（wire codec），收发（`Transport`），trace 记录与比对 | `Transport` 后端可有 RX 线程 | [`l0-can-io.md`](docs/components/l0-can-io.md) | **已实现，接口已冻结**（`cpp/`） |
| L1 Joint Driver | **一个**关节模组：控制字节里的使能 / 抱闸 / 清错 / 模式、SDO 诊断与零位标定、故障码、方向与零偏、单位 | 无。由 L2 每个 tick 调用 | `development-plan.md` 1B | 未开始 |
| L2 Executor | 控制周期：tick、组 `0x200`、watchdog keep-alive、新鲜度、**safety limiter**、motion source 仲裁、controlled stop；joint mapping 作为构造配置 | **有，且只有这一层有** | [`l2-executor.md`](docs/components/l2-executor.md) | 接口草案 |
| L3 Motion Planning | streaming（10–50 Hz 输入 → 平滑、加密成带时间戳的点）、离线规划、FK / IK；所有速度 / 加速度 / jerk 约束 | 无自有 tick，不需要知道周期；只往 L2 交带时间戳的点，L2 每个 tick 线性插值 | `development-plan.md` 2A–2C | 未开始 |
| L4 Control API | 对外 API、机器人状态机、阻塞语义；经 RPC Service 暴露给外部计算机 | 无（RPC Service 有自己的 I/O 线程，不带控制时钟） | `development-plan.md` 2D | 未开始 |

这个切法和厂家的 `librk3576_can_canfd` / `ExecutorJuxie` / `ArmServoMode` + `PathPlanner` / `ControllerJuxie`
大致对应——这是「对标」的含义：同样的职责边界，不是同样的实现。

## 5. 运行时视图：线程与时钟

| 线程 | 所在 | 做什么 |
|---|---|---|
| Executor tick 线程 | L2，控制板 | **唯一拥有控制时钟的线程。** 一个线程驱动两条总线；每个 tick 读快照、调 L1、插值、过 safety limiter、发 `0x200`（`l2-executor.md` §2） |
| `Transport` RX 线程 | L0 后端，控制板 | 只做一件事：写快照 |
| RPC Service I/O 线程 | 控制板 | 收发 RPC 消息、心跳、时间换算；不带控制时钟，调用 L4 / L2 的线程安全接口 |
| L3 / 应用线程 | 外部计算机 | 不需要实时；成批、不定时地追加点，只要追加得比播放快 |

两条主要的数据流：

- **命令**：应用 → L3（带时间戳的点）→ RPC → L4 → L2 `execute` / `stream` → 每个 tick 插值 → safety limiter
  → L1 换算成 cnt + 控制字节 → L0 `0x200`
- **反馈**：关节模组 `0x300` → L0 RX 线程 → L2 快照（L1 换算成物理量）→ L4 状态机在读快照时推导状态 → RPC 快照订阅

## 6. 关键决定

| 决定 | 理由 | 细节 |
|---|---|---|
| **L2 单独一层，拥有时钟** | 看门狗要求总线上持续有帧，反馈又只由总线上的帧触发（C2、C3），tick、watchdog keep-alive、新鲜度、组包是同一个循环 | `l2-executor.md` §0 |
| **safety limiter 在 L2，不可关闭** | streaming 不经过离线规划器，限位只放在规划器里等于 streaming 没有兜底。规划期的限位检查只是为了尽早报错 | [`safety-concept.md`](docs/safety-concept.md) |
| **joint mapping 不单列一层，并入 `ExecutorConfig`** | 解码已经在 L0（`decode_feedback`），状态机在 L4，剩下的只是一张映射表和一个查表函数，没有状态和时序 | `l2-executor.md` §3.2 |
| **使能 / 清错 / 抱闸走控制字节，不走 SDO** | PR0002 §5.1 的原生路径：这些位每个 tick 随 `0x200` 发出，和控制帧天然串行，用反馈状态位确认。SDO 只做配置与诊断 | `l2-executor.md` §2.3 |
| **机器人状态机在 L4，只有一个写者** | 状态由 L2 快照推导，L4 不另起轮询线程；每臂健康等聚合状态在读快照时算 | `l2-executor.md` §3.1 |
| **方向与零偏在 L1** | 配置里有逐轴方向与软零位，方向配错会让直线走不直（`hardware-facts.md` 3.5） | `development-plan.md` 1B.4 |
| **L3 只交带时间戳的点** | L3 不需要实时线程，也不需要知道周期；所以 L3 可以隔一层网络放在外部计算机上，接口不变 | `l2-executor.md` §3.3 |
| **网络边界在 Executor 之上** | 带时钟的层和 bus master 必须靠近总线 | §3 |
| **运动学从零重写，FK / IK 共用一个模型文件** | 随包的三份模型（YAML、厂家 FK、厂家 IK）互相不一致（[`kinematics.md`](research/vendor-analysis/kinematics.md)）。模型文件是我们自己的格式，每个参数标明来源；真值是真机 | `development-plan.md` 2A |
| **Joint Emulator 是独立组件，不是 L2 的测试附件** | trace 比对只是调试工具，离线判断 L1 / L2 对不对只剩它，所以 L1 / L2 的「做完」都依赖它 | [`verification.md`](docs/verification.md) |

## 7. 待定问题

跨组件、影响部署的问题。各组件自己的待定项在组件文档里，RPC 的在 `rpc.md` §5。

| # | 问题 | 怎么定 |
|---|---|---|
| Q1 | streaming smoothing 留在外部还是搬到板上 | 由网络到达抖动决定（`rpc.md` §5 第 1 项；2B.4） |
| Q2 | 板上 CAN 访问是 SocketCAN 还是寄存器 | `hardware-facts.md` 1.8，真机（0C.3；实现 1D.1）。只影响 L0 `Transport` 后端，不影响分层 |
| Q3 | 板上的空闲算力与调度抖动 | 真机上在满载下测 L2 tick 的迟到分布（`l2-executor.md` §2.1；0C.4） |
| Q4 | 开发板侧的实现语言 | 阶段 0 结束时定（0C.4）；L0 沿用 C++ 或移植 |

## 8. 代码地图

```
cpp/include/shensi/can/   L0 公共头：frame、wire、transport、fake_transport、trace
cpp/src/                  L0 实现
cpp/tests/                L0 测试（PR0002 golden vector、字节序、trace 往返与差分）
tools/verify.sh           离线检查：厂家原件哈希、SDK 树完整、L0 测试
```

L1 以上还没有代码。
