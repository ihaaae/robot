# 验证策略

> 离线时和在真机上，**怎么判定代码是对的**。各组件的具体测试清单在组件文档里
> （`l0-can-io.md` §7、`l2-executor.md` §6），真机上的验证门在 [`hardware-bringup.md`](hardware-bringup.md)；
> 本文讲它们之间的关系和判定依据。

## 1. 判定依据

| 依据 | 判定什么 | 地位 |
|---|---|---|
| PR0002 + 真机实测 | 一切 | 正本（规格优先级见 `ARCHITECTURE.md` §1） |
| golden vector（照 PR0002 写） | L0 wire codec | 随 L0 实现一起写（T0.7） |
| 接口契约测试 | `Executor` 的真实实现 | 清单在 T0.4，在真机上跑（T1.4） |
| trace 差分 | 「这次和上次有什么不同」 | **调试工具，不是验收标准** |
| 参考运动（厂家栈的反馈曲线） | 阶段 1 真机回放 | 验收按反馈曲线，不按帧 |

仓库里还没有任何东西在真机上跑过（`ARCHITECTURE.md` §2 C5）。真机已到位，L1 及以上对不对只在真机上判定。

## 2. 离线测试

- **L0**：PR0002 golden vector、字节序、往返、文档矛盾、trace 文本往返与六类差分（`l0-can-io.md` §7）。
  随 L0 实现一起写（T0.7），并入 `tools/verify.sh`。
- **L2**：插值、时间戳校验、stream 缓冲、safety limiter（`l2-executor.md` §6）。tick、保持、仲裁、新鲜度、`halt`、
  看门狗要收发帧，在真机上测；契约测试清单在 T0.4。
- **RPC**：注入 ±10⁻⁴ 的时钟漂移和网络抖动，断言长时间流不出现 stream underrun（T2.2）。

默认检查必须离线：`./tools/verify.sh` 不连网、不碰机器人（`requirements.md` NFR8）。

## 3. 真机上的验证

- **「第一次动」之前的门**：限位、方向与左右臂映射、反馈新鲜度与失效处理……见 [`hardware-bringup.md`](hardware-bringup.md) 主线。
- **L1 / L2**：L1 逐轴确认，L2 的契约测试与 `l2-executor.md` §6 里要收发帧的各项，在我们的栈独占总线后上真机跑（T1.3、T1.4）。
- **阶段 1 验收**：回放参考运动，每个关节反馈曲线的最大偏差在重复性容差以内（T0.2、T1.6）。
  帧不一样而运动一样算通过，帧一样而运动不一样不算。
- 结论一律回填 [`hardware-facts.md`](hardware-facts.md)，来源改成「真机」。
