# 需求

> 系统**要做什么**：目标与范围、用例、功能需求、非功能需求。怎么做见 [`ARCHITECTURE.md`](../ARCHITECTURE.md)。
> L4 Control API 与 RPC 操作集合（[`rpc.md`](interfaces/rpc.md) §1）都以这里为来源。
> 编号（UC / FR / NFR）稳定，不重排；其他文档引用编号。

## 1. 目标与范围

自己的 controller SDK，提供和厂家 SDK（`dual_arm_app` 0.6.4）同一抽象层次的能力。需求从我们自己的用例推出；
厂家 SDK 的功能清单只用来查漏（§5）。

**范围内**：两条臂共 14 个关节的使能、运动、监控、故障恢复、维护，经 RPC 从外部计算机访问。

**范围外**：

- **供电控制。** PR0002 里没有任何供电控制；关节供电由外部电源和急停回路负责，不进我们的 API。
- **Dev_ID 8 的设备。** 它的身份不明（`hardware-facts.md` 1.6），真机上确认有末端设备时再加。
- **认证。** v1 不做，是有意接受的风险（`rpc.md` §4）。

## 2. 用例

| # | 用例 | 涉及的功能需求 |
|---|---|---|
| UC1 | 开机后把臂带到可动状态 | FR1、FR6 |
| UC2 | 示教、预设动作 | FR2 |
| UC3 | 上位机实时控制（遥操作、策略输出，10–50 Hz） | FR3 |
| UC4 | 出了问题，停下来 | FR4 |
| UC5 | 故障恢复 | FR5 |
| UC6 | 监控与记录 | FR6、FR7 |
| UC7 | 维护 | FR8 |
| UC8 | 离线计算 | FR9 |

## 3. 功能需求

| # | 需求 |
|---|---|
| FR1 | 连接 / 断开总线，按臂使能 / 失能 |
| FR2 | 关节点到点、笛卡尔点到点、直线；提交 + 等待 |
| FR3 | 关节 streaming、笛卡尔 streaming |
| FR4 | controlled stop（按臂或全部），从任意线程生效（[`safety-concept.md`](safety-concept.md)） |
| FR5 | 读故障（原始码 + 我们的映射）、清错、重新使能 |
| FR6 | 状态与遥测快照：位置、速度、电流 / 力矩、温度、每臂健康、机器人状态 |
| FR7 | 总线 trace |
| FR8 | 零位标定（须失能）、手动抱闸 |
| FR9 | FK / IK |

## 4. 非功能需求与 API 硬性要求

| # | 需求 |
|---|---|
| NFR1 | 任何调用顺序都返回明确的错误而不是崩溃 |
| NFR2 | 输入一律带长度检查 |
| NFR3 | 数据形状是 14 个关节（左 7 + 右 7），mask 表达「保持」；读侧用 `valid` 标志表示「暂无数据」，不用哨兵值 |
| NFR4 | 一条总线只允许一个实例，第二个实例显式拒绝；RPC 同一时刻只有一个 control session（`ARCHITECTURE.md` §2 C1） |
| NFR5 | safety limiter 不可关闭；限位与 joint mapping 不提供默认值（[`safety-concept.md`](safety-concept.md)） |
| NFR6 | RPC 断连在 `link_timeout` 内 controlled stop（`rpc.md` §3） |
| NFR7 | 外部程序（含 Python）经 RPC 访问，不提供 C ABI 绑定 |
| NFR8 | 默认检查（`tools/verify.sh`）离线可跑，不连网、不碰机器人 |

## 5. 和厂家 SDK 的查漏

拿厂家 SDK 的功能清单查漏，名字上只多出「上下电」（`OnRobot` / `OffRobot`）。但厂家的 `OnRobot()` 实际做的是
创建执行器、启动状态轮询线程、必要时清一次错（[`robot-state-machine.md`](../research/vendor-analysis/robot-state-machine.md) §2.1），
并不切电源——对应的是我们的 FR1「连接 / 断开总线」。
