# 验证策略

> 离线时和在真机上，**怎么判定代码是对的**。各组件的具体测试清单在组件文档里
> （`l0-can-io.md` §7、`l2-executor.md` §7），真机上的验证门在 [`hardware-bringup.md`](hardware-bringup.md)；
> 本文讲它们之间的关系和判定依据。

## 1. 判定依据

| 依据 | 判定什么 | 地位 |
|---|---|---|
| PR0002 + 真机实测 | 一切 | 正本（规格优先级见 `ARCHITECTURE.md` §1） |
| **Fake Joint** | 离线时 L1 / L2 的逻辑 | 测试替身，行为照真机实测；L1 / L2 的验收在真机上 |
| golden vector（照 PR0002 写） | L0 wire codec | 已写成断言（`cpp/tests/test_wire.cpp`） |
| 接口契约测试 | `Executor` 的真实实现和 `SimExecutor` 行为一致 | 两者跑同一套 |
| trace 差分 | 「这次和上次有什么不同」 | **调试工具，不是验收标准** |
| 参考运动（厂家栈的反馈曲线） | 阶段 1 真机回放 | 验收按反馈曲线，不按帧 |

仓库里还没有任何东西在真机上跑过（`ARCHITECTURE.md` §2 C5）。真机已到位，L1 / L2 对不对最终在真机上判定；
Fake Joint 用来跑离线回归，以及测真机上难以或不宜制造的故障。

## 2. Fake Joint

挂在 L0 `FakeTransport` 的 responder 上，模拟 14 个关节模组（两条总线，每条 7 个）：
收控制帧回 `0x300`，按控制字节的使能 / 抱闸 / 清错位改变自身状态，收 SDO 回 `0x580`。
可注入故障：不回复、报错、看门狗超时。

- **行为照真机实测**：L1 要用到的未知项（`hardware-facts.md` 4.2、4.3、4.6、4.7）先在真机上答掉，
  Fake Joint 只实现实测的那一种行为，不做开关（`development-plan.md` T1.2）。和真机不一致时以真机为准，改 Fake Joint。
- **它不是验收标准**：L1 / L2 先在 Fake Joint 上过测试，再上真机验收。它的用处是离线回归（`tools/verify.sh` 不碰机器人）、
  上真机前挡住明显的错、测真机上难以或不宜制造的故障（模块不回复、看门狗超时）。

## 3. Test double

每个冻结的接口配一个 test double，上层就能不等下层推进。冻结计划见 `development-plan.md`「接口冻结」。

| Test double | 替代什么 | 给谁用 | 细节 |
|---|---|---|---|
| `FakeTransport` | L0 `Transport` 的真实后端 | L0 测试；Fake Joint 挂在它上面 | `l0-can-io.md` §4.2 |
| `ReplayTransport` | 真实总线（回放录好的 trace） | trace 差分回路 | `l0-can-io.md` §6 |
| Fake Joint | 14 个关节模组 | 测真实的 `Executor` + L1 整条链 | §2 |
| `SimExecutor` | L2 `Executor` 本身。关节理想跟随目标（可选一阶滞后），可注入故障、掉帧 | L3 / L4 开发，不需要 L1 | `l2-executor.md` §6 |

## 4. 离线测试

- **L0**：PR0002 golden vector、字节序、往返、文档矛盾、trace 文本往返与六类差分（`l0-can-io.md` §7）。
  已在 `tools/verify.sh` 第 3 步，只需宿主 C++ 编译器。
- **L2**：tick、保持、仲裁、插值、时间戳校验、stream、safety limiter、新鲜度、`halt`、看门狗（`l2-executor.md` §7）。
  契约测试清单在 T0.4。
- **RPC**：注入 ±10⁻⁴ 的时钟漂移和网络抖动，断言长时间流不出现 stream underrun（T2.2）。

默认检查必须离线：`./tools/verify.sh` 不连网、不碰机器人（`requirements.md` NFR8）。

## 5. 真机上的验证

- **「第一次动」之前的门**：限位、方向与左右臂映射、反馈新鲜度与失效处理……见 [`hardware-bringup.md`](hardware-bringup.md) 主线。
- **阶段 1 验收**：回放参考运动，每个关节反馈曲线的最大偏差在重复性容差以内（T0.2、T1.6）。
  帧不一样而运动一样算通过，帧一样而运动不一样不算。
- 结论一律回填 [`hardware-facts.md`](hardware-facts.md)，来源改成「真机」。
