# 安全概念

> 软件这一侧**怎么防止机械臂做危险的动作、出了问题怎么停**。这里只汇总各层的安全机制和它们之间的关系；
> 每个机制的细节在它所在组件的文档里，本文链接过去，不重复论证。
>
> 本文不是功能安全认证意义上的安全文档。硬件急停回路独立于软件，不在本文范围内，本文也不依赖它。

## 1. 原则

1. **最后一道检查在唯一的出口。** 所有 motion source（离线轨迹、streaming、`halt`）都经过 L2 的每个 tick 发出，
   safety limiter 就放在这里。上层的检查只为尽早报错，不是兜底。
2. **不可关闭，没有默认值。** safety limiter 没有「调试时绕过」的开关。位置限位和 joint mapping
   不提供默认值：配错就是越限或左右臂互换，真机标定 / 确认之前配置文件必须显式写出来。
3. **不依赖上游活着。** controlled stop 不依赖 L3、不依赖网络。
4. **没有新鲜反馈就不下发新目标。** 使能的那一刻，设定点必须以实测位置作种子。

## 2. 机制

| 机制 | 在哪 | 触发 | 动作 | 细节 |
|---|---|---|---|---|
| **safety limiter**：位置限位 | L2 | `stream` / `execute` 提交的目标越过 `[lower, upper]` | 提交时拒绝，返回 `TargetOutOfLimits` | `l2-executor.md` §5 |
| **safety limiter**：单 tick 步长 | L2 | `\|Δq\| > v_max · T` | 截断到上限并计数；持续截断 M 个 tick 则该臂 `halt` | `l2-executor.md` §5 |
| **controlled stop**（`halt`） | L2 | 上层调用、safety limiter、RPC 断连 | 按配置的最大减速度把速度降到 0，然后保持 | `l2-executor.md` §3；`development-plan.md` 1C.4 |
| **反馈新鲜度** | L2 | 一条臂的反馈超出新鲜度窗口 | 该臂 motion source 中止、发保持帧；连续 N 个 tick 不新鲜则判失效，L4 转 `fault` | `l2-executor.md` §4 |
| **watchdog keep-alive** | L2 | 每个 tick | 总线上每个 tick 都有一帧，没有 motion source 时发保持帧，避免关节自锁 | `l2-executor.md` §2.3 |
| **RPC 断连** | RPC Service | 超过 `link_timeout` 没有消息 | 对该 control session 占用的臂 `halt`，离线轨迹执行中也一样 | `rpc.md` §3 |
| **单一 bus master / control session** | L0、L4、RPC | 第二个实例或第二个 control session | 拒绝 | `l0-can-io.md` §5；`rpc.md` §4 |
| 规划期限位检查 | L3 | 规划时 | 尽早报错。**不是兜底** | `development-plan.md` 2C.3 |
| 关节模组自身的看门狗 | 硬件 | 约 500 ms 没有控制帧 | 自锁 | `hardware-facts.md` 4.2 |

## 3. 时间上的约束

- 从最后一次新鲜反馈到判定失效的时间必须**明显小于** 500 ms 看门狗，否则关节会先于我们的判定自锁。
- safety limiter 的步长上限必须始终低于硬件的「位置跃迁过大」上限（CSP 下单帧 Δ > 500 cnt，`hardware-facts.md` 4.4；
  `l2-executor.md` §2.1 有换算）。
- `link_timeout`（起始值 200 ms）要让 RPC 断连的 controlled stop 早于 L2 的 stream underrun（突然保持）。

## 4. 待标定的参数

都没有可用的随包值，都要在真机上定：

| 参数 | 怎么定 |
|---|---|
| 位置限位 `lower` / `upper` | P0-4（`hardware-bringup.md`；0C.1） |
| 步长上限 `v_max`、持续截断 tick 数 M、`halt` 的最大减速度 | 和 P0-4 一起标定；`v_max` 先取保守值（例如 1.5 rad/s）（`l2-executor.md` §8 第 4 项） |
| 新鲜度窗口、失效 tick 数 N | 真机测反馈延迟分布后定（`l2-executor.md` §8 第 3 项） |
| `link_timeout`、心跳周期 | 实测网络后定（`rpc.md` §5） |
| joint mapping（左右臂、`Dev_ID`、方向） | 真机逐轴点动（0C.2；`hardware-facts.md` 1.4、1.5） |

## 5. 有意接受的风险

| 风险 | 缓解 | 何时重新评估 |
|---|---|---|
| RPC 没有认证，能连上端口的设备就能让机器人运动 | 只监听专用的有线网口，不监听 Wi-Fi 和公网（`rpc.md` §4） | 真机上线前 |
