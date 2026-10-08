# 控制器使用说明 — PDF 整理稿

| | |
|---|---|
| 来源 | `vendor/originals/documents/controller-user-manual.pdf` |
| sha256 | `3fa67165e56e5bb7ddc34ddff8d62a9041cdf3dcd645c8932e07de20b7ea31c6` |
| 页数 | 8 |
| 文档编号 | 无（封面直接进入「一、控制器基础配置部分」） |
| 水印 | 无锡巨蟹智能 |
| 整理日期 | 2026-09-29 |

**为什么重要**：这是本仓库一直缺的那一层——**控制器**（板子 + Web/JSON 应用）的文档，
而不是关节模组的协议。它回答了几个我们只能猜的问题：CAN 通道到左右臂的映射、
`params.yaml` 里 `Part` 的含义、臂型号命名、以及应用层错误码的完整表。

**这不是权威副本**，是整理稿。原文里大量内容是**截图**（ssh 输出、vim 里的 yml、
`top` 输出），下面的数值是从截图读出来的，**比正文散文部分可靠性低**。凡是从截图读出来的
都标了 ⚠️。另：文档里含板子的默认 SSH 口令，本仓库不复制该凭据。

---

## 一、控制器基础配置

- 左臂通信接 **CAN1**，右臂通信接 **CAN2**；支持单臂和双臂运动。
- 网口连 PC，控制器默认 IP `192.168.10.95`。PC 侧最后一位主机号不能也是 `95`。
- 通过 `ssh root@192.168.10.95` 进入板子；换板子后可能需要清掉旧密钥。
- 配置文件目录：`cd DualArm/usr/etc`，编辑 **`params.yaml`**。
  ⚠️ 注意文件名是 `.yaml`，而仓库里 vendored 的树用的是 `params.yml`——**两个名字不一致，
  上真机时以板子上的实际文件为准**。
  - `Part`：`0` 双臂 / `1` 左臂 / `2` 右臂。
  - 臂型号：六轴选 `ARM_62`，七轴选 `ARM_73`。
  - 格式敏感：改动时不能删多余空格，否则文件解析失败、控制器连不上。
- 改完重新断上电，下发 JSON 指令；**上电后直接上使能即表示适配成功**。
- 若没使能：可能是电机报警或程序没起来。用 `top` 看首行是不是
  `dual_arm_app_interface_node`；不是就进 `DualArm/usr/bin/` 手动
  `./dual_arm_app_interface_node` 启动。
  ⚠️ 手动启动时 cmd 窗口不能关、不能输入其他字符，否则程序会挂。

启动日志片段（截图）⚠️，注意其中出现了 `updateImpl` 与 `Config file path`：

```
helo main...
updateImpl
Config file path  : /home/root/DualArm/usr/etc
create_directory : /home/root/logs/_0505005141
path /home/root/DualArm/usr/etc
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
1, 1, 1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, -1
leftLimits.size() : 7
runing...
```

> `updateImpl` 出现在启动日志里，与 `research/vendor-analysis/sdk-usage.md` §3 里「`State::m_impl` 由
> `updateImpl()` 赋值」的推断一致。
> 第三行 14 个 `±1` 与 `executor.yml` 的 `MotorDirect` 是同一类东西（见下）。
> ⚠️ 日志里的符号序列 `1,1,1,-1,1,1,-1,1,1,-1,1,1,1,-1` 与下面 `executor.yml` 截图里读到的
> `1,1,-1,1,1,-1,1,-1,1,-1,1,1,1,1` **对不上**。至少有一处是读错的，真机要实测。

### 运动指令失败的两种常见原因

1. 目标点位超限（关节角度是**弧度制**）。
2. 当前关节位置已超限。可下使能把轴转回限位内，或在各轴不干涉的前提下改限位值。

限位在 `DualArm/usr/etc/juxie_73/kinematics.yml`（六轴为 `juxie_62`）的 `limit` 段，
⚠️「每个关节的前两行即为该关节的限位数值」：

```
limits:
  0: [-3.14, -3.14, 1.5, 6.5, 100]
  1: [ 3.14, -0.09, 1.5, 6.5, 100]
```

⚠️ 每个关节 5 个数，后三个在所有关节上都是 `1.5, 6.5, 100`（像是速度/加速度类上限）。
前两个数才是限位，但 `0: [-3.14, -3.14]` 是退化区间，说明要么截图读错，要么顺序是
`[上限, 下限]` 而非 `[下限, 上限]`。**这与 `docs/hardware-acceptance.md` P0-4 记的
「limits 全是 ±3.1415」需要一起重核。**

同一个文件夹下的 `executor.yml` 可改**各轴正转方向** `MotorDirect`（14 项）。
文档明确说：**若 `movel` 走出的轨迹不直，就对照 URDF 检查各轴正转方向是否正确**，
错了直接改对应符号。

> 这条对 L2/L3 是硬信息：`MotorDirect` 是**逐轴、且 load-bearing** 的配置，
> 不是装饰。厂家自己的说明承认它配错会让 `MoveL` 不直。

## 二、Web 界面

- `192.168.10.95:5566`，账户 `admin`（口令见原文档）。
- 三个页面：显示（关节角度、笛卡尔位姿、机器人状态、错误反馈）、示教、设置（零位标定）。
- **机器人状态**（应用层命名）：`ready`（无报错未使能）/ `idle`（无报错已使能）/
  `running`（运动中）/ `error`（报错，报错码显示在下方）。
  > 注意这是 **4 个**状态，没有 SDK `GetRobotState()` 里的 `power_off`（0）。
  > `research/vendor-analysis/error-codes.md` 记的是 SDK 的 5 个（0..4），两者命名也不同（`fault` vs `error`）。
- 未使能时可手动推动机器人，观察监控数值变化来验证轴方向。
- 示教页：`movej`（关节角）、`movej_p`（笛卡尔的关节运动）、`movel`（直线运动）。
- **循环运动**是控制器内预设轨迹，点一次即可。文档明确警告：**点多次会导致点位异常造成危险；
  停止需要多点几下，否则后续轨迹填充会导致停不下来。**
  > 这条直接影响 `Stop()` 的重实现与 trace 语料设计：厂家自身的停止路径可能合法地重复下发
  > 停止帧，见 `docs/l0-interface.md` §6 里 `collapse_consecutive_duplicates` 默认为 `false` 的理由。

## 三、程序刷新

- 更新以 `.deb` 交付，`scp` 到板子后：
  `dpkg -i --instdir="/home/root/DualArm" dual_arm_app-<version>-Linux.deb`
- 因此板子上的部署根目录是 `/home/root/DualArm`，配置在 `/home/root/DualArm/usr/etc`，
  可执行文件在 `/home/root/DualArm/usr/bin/`，日志在 `/home/root/logs/`。

> 这回答了 `research/vendor-analysis/running-on-the-robot.md` 里「node 是怎么起来的、装在哪」的一部分：
> 部署根是 `/home/root/DualArm`，且**应该自启动**（用 `top` 验证；没起来就手动跑）。

## 四、错误码

网页端报错形如 `0x01010001`。完整表（`etc/error.json` 的自造格式，**与伺服的 `0x603F`
无关**，见 `research/vendor-analysis/can-protocol-comparison.md` §5 末尾）：

| 格式 | 含义 |
|---|---|
| `0x01<NN>0001` | 电机 `NN` 掉线 |
| `0x01<NN>0002` | 电机 `NN` 报错 |
| `0x02<NN>0001` | 电机 `NN` 超限位 |

`NN` 取值 `01`..`0e`，即 **14 个电机**。

> **这是整机映射（`development-plan.md` 1C.1）缺的那块拼图的一半**：14 = 左 7 + 右 7。
> 结合「左臂 CAN1、右臂 CAN2」，全局电机号 `1..7` = 左臂、`8..14` = 右臂。
> 也就是说 **17 关节里的腰（1 个）和头（2 个）不在这 14 个 CAN 电机里**——
> `JointSpaceData` 的 17 维与 CAN 上的 14 个设备不是一一对应。
>
> ⚠️ 还有一处命名陷阱要确认：本文件用 **1 基**的 `CAN1`/`CAN2`，而
> `rk3576_can_canfd.h` 与 `/dev/misc_shm_can*` 用 **0 基**的 `CAN0`/`CAN1`。
> 若两者对应，则**左臂 = 代码里的 `Bus::Can0`**。这是最容易左右臂搞反的地方，
> 真机上必须先确认再动。

电机报错的具体内容要去日志里看：`cd logs`，最新的是编号最大的文件夹，
进去打开 `ErrorData`。
