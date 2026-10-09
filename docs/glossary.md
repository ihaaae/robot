# 术语表

> 文档里的技术术语一律用这里的英文写法，不另加中文译名；每个术语在这里给一次中文定义。
> 按字母排序。缩写见表末。

| 术语 | 定义 | 主文档 |
|---|---|---|
| **bus master** | 总线上发控制帧和 SDO 请求的那一方。一条总线只能有一个 | `l0-can-io.md` §5 |
| **control session** | RPC 上有权让机器人运动的会话。同一时刻只有一个；只读的快照订阅不算 | `rpc.md` §4 |
| **Control API** | L4：我们自己的对外 API、错误码、机器人状态机 | `ARCHITECTURE.md` §4 |
| **controlled stop** | `halt()`：按配置的最大减速度把速度降到 0，然后保持。不依赖 L3 和网络。大致对应 IEC 60204-1 的 stop category 2（停下后保持使能） | `safety-concept.md` |
| **Executor** | L2：唯一拥有控制时钟的层 | `l2-executor.md` |
| **freshness**（新鲜度） | 一条臂所有关节的反馈都在新鲜度窗口内。不新鲜的臂不接受新目标 | `l2-executor.md` §4 |
| **hardware bring-up** | 真机到货后、第一次运动前后要过的验证门 | `hardware-bringup.md` |
| **hold frame**（保持帧） | 目标为当前位置的 `0x200`。没有 motion source 时每个 tick 发它 | `l2-executor.md` §2.3 |
| **Joint Driver** | L1：**一个**关节模组的控制字节、SDO 诊断、方向 / 零偏、单位换算。无线程，由 L2 每个 tick 调用 | `development-plan.md` 1B |
| **Joint Emulator** | 照 PR0002 写的虚拟关节模组，挂在 `FakeTransport` 上。离线判定 L1 / L2 对不对的唯一依据 | `verification.md` §2 |
| **joint mapping** | 14 个关节 ↔ (bus, `Dev_ID`) 的映射表，`ExecutorConfig` 的一部分，无默认值 | `l2-executor.md` §3.2 |
| **Motion Planning** | L3：streaming smoothing、离线规划、FK / IK | `ARCHITECTURE.md` §4 |
| **motion source** | 写设定点的来源：一条离线轨迹（`execute`）或一段流（`stream`）。每条臂同一时刻只有一个 | `l2-executor.md` §3.3 |
| **reference motion** | 阶段 0 用厂家栈采到的运动（总线抓包 + 反馈曲线），阶段 1 的验收依据。是数据，不是规格 | `development-plan.md` 阶段 0 |
| **RPC Service / RPC Client** | 控制板上 / 外部计算机上的 RPC 端点 | `rpc.md` |
| **safety limiter** | L2 里对每个 tick 即将发出的目标做的最后检查：位置限位、单 tick 步长。不可关闭 | `safety-concept.md` |
| **stream underrun** | 取样时刻越过了 stream 的最后一个点：保持并释放 motion source，`stream_underruns` 加一 | `l2-executor.md` §3.3 |
| **streaming** | 上位机以 10–50 Hz 实时给设定点的运动方式；L3 把它平滑、加密成带时间戳的点（streaming smoothing） | `l2-executor.md` §3.3 |
| **test double** | 替代真实依赖的测试实现：`FakeTransport`、`ReplayTransport`、`SimExecutor`、Joint Emulator | `verification.md` §3 |
| **tick** | Executor 的一个控制周期。每个 tick 恰好发出一帧 `0x200`；周期可配置，起始值 2 ms | `l2-executor.md` §2 |
| **trace** | L0 记录的双向原始帧，文本格式。用于调试与回归，不是验收标准 | `l0-can-io.md` §6 |
| **watchdog keep-alive** | 通过每个 tick 持续发控制帧，让关节模组的看门狗（约 500 ms）不触发自锁 | `l2-executor.md` §0 |
| **wire codec** | L0 里字节 ↔ 线上结构体的编解码纯函数 | `l0-can-io.md` §3 |

| 缩写 | 全称 |
|---|---|
| CSP | Cyclic Synchronous Position，周期同步位置模式（CiA 402） |
| FK / IK | 正 / 逆运动学 |
| HF x.y | `hardware-facts.md` 的第 x.y 行 |
| PP | Profile Position，带轨迹规划的位置模式 |
| PR0002 | 关节模组 CAN / CAN-FD 协议文档，协议正本 |
| SDO | Service Data Object，CANopen 的请求 / 应答式对象字典访问 |
