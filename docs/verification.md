# 验证策略

> 真机到货前、以及真机上线后，**怎么判定代码是对的**。各组件的具体测试清单在组件文档里
> （`l0-can-io.md` §7、`l2-executor.md` §7），真机上的验证门在 [`hardware-bringup.md`](hardware-bringup.md)；
> 本文讲它们之间的关系和判定依据。

## 1. 判定依据

| 依据 | 判定什么 | 地位 |
|---|---|---|
| PR0002 + 真机实测 | 一切 | 正本（规格优先级见 `ARCHITECTURE.md` §1） |
| **Joint Emulator** | 离线时 L1 / L2 对不对 | 离线唯一的判定依据 |
| golden vector（照 PR0002 写） | L0 wire codec | 已写成断言（`cpp/tests/test_wire.cpp`） |
| 接口契约测试 | `Executor` 的真实实现和 `SimExecutor` 行为一致 | 两者跑同一套 |
| trace 差分 | 「这次和上次有什么不同」 | **调试工具，不是验收标准** |
| 参考运动（厂家栈的反馈曲线） | 阶段 1 真机回放 | 验收按反馈曲线，不按帧 |

仓库里还没有任何东西在真机上跑过（`ARCHITECTURE.md` §2 C5），所以 Joint Emulator 先于 L1 建好。

## 2. Joint Emulator

挂在 L0 `FakeTransport` 的 responder 上，按 PR0002 模拟 14 个关节模组（两条总线，每条 7 个）：
收控制帧回 `0x300`，按控制字节的使能 / 抱闸 / 清错位改变自身状态，收 SDO 回 `0x580`，
超过 500 ms 没收到控制帧就自锁。可注入故障：不回复、报错、超限位、反馈延迟或丢帧、bus-off。

- **行为规格是可执行的**：模块对每类帧的反应，每条注明出处（PR0002 章节或 `hardware-facts.md` 的行）（0D.2）。
- **文档没写的行为做成开关**（`hardware-facts.md` 4.3、4.5–4.7 等「未知」项）。L1 / L2 的测试在每种设置下都要通过；
  真机给出答案时，只是把开关定下来，不用改被测代码。
- **规格、实现和 L1 由三个不同的人写**（0D.2、1A、1B）：三者都照 PR0002 写，同一个人对文档的误读
  会同时进实现和 test double，测试照样通过。
- 真机到货后逐条对照，修正规格、定下开关（1A.5）；之后才是可信的 test double。

它是独立组件，不是 L2 的测试附件（`ARCHITECTURE.md` §6）。

## 3. Test double

每个冻结的接口配一个 test double，上层就能不等下层推进。冻结计划见 `development-plan.md`「接口冻结」。

| Test double | 替代什么 | 给谁用 | 细节 |
|---|---|---|---|
| `FakeTransport` | L0 `Transport` 的真实后端 | L0 测试；Joint Emulator 挂在它上面 | `l0-can-io.md` §4.2 |
| `ReplayTransport` | 真实总线（回放录好的 trace） | trace 差分回路 | `l0-can-io.md` §6 |
| Joint Emulator | 14 个关节模组 | 测真实的 `Executor` + L1 整条链 | §2 |
| `SimExecutor` | L2 `Executor` 本身。关节理想跟随目标（可选一阶滞后），可注入故障、掉帧 | L3 / L4 开发，不需要 L1 | `l2-executor.md` §6 |

## 4. 离线测试

- **L0**：PR0002 golden vector、字节序、往返、文档矛盾、trace 文本往返与六类差分（`l0-can-io.md` §7）。
  已在 `tools/verify.sh` 第 3 步，只需宿主 C++ 编译器。
- **L2**：tick、保持、仲裁、插值、时间戳校验、stream、safety limiter、新鲜度、`halt`、看门狗（`l2-executor.md` §7）。
  契约测试清单在 0D.3。
- **RPC**：注入 ±10⁻⁴ 的时钟漂移和网络抖动，断言长时间流不出现 stream underrun（2E.1）。

默认检查必须离线：`./tools/verify.sh` 不连网、不碰机器人（`requirements.md` NFR8）。

## 5. 真机上的验证

- **「第一次动」之前的门**：限位、方向与左右臂映射、反馈新鲜度与失效处理……见 [`hardware-bringup.md`](hardware-bringup.md) 主线。
- **阶段 1 验收**：回放参考运动，每个关节反馈曲线的最大偏差在重复性容差以内（0B.3、1E.4）。
  帧不一样而运动一样算通过，帧一样而运动不一样不算。
- 结论一律回填 [`hardware-facts.md`](hardware-facts.md)，来源改成「真机」。
