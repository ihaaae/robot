# 离线运动学（部分校验：只对齐了厂家 IK 的目标位姿）

[`vendor_model/kinematics.py`](../vendor-tools/python/vendor_model/kinematics.py) 是从 `/usr/etc/juxie_73/kinematics_*.yml` 还原出的独立运动学库，**不需要机器人**就能算 FK / IK。

**校验范围，一句话**：本库的 **FK** 加一个**拟合**出来的 84.721 mm 工具偏置，能复现厂家 IK 被要求到达的目标位姿（0.089 mm）。本库的**数值 IK 从没和厂家比过**；本库 FK 与厂家 `getFKpose` 在一般位姿下差 200–700 mm。详见下文「三方不一致」。本文件是这个话题的**唯一正本**，其他文档只引用这里。

## 模型

厂家用的是**旋量法（PoE / product of exponentials）**，不是 DH：

\[ T(q) = e^{[S_1]q_1}\, e^{[S_2]q_2} \cdots e^{[S_7]q_7}\, M \]

- `screws[i]` = 第 i 个关节的**单位螺旋轴** `[ω(3), v(3)]`，表示在**臂基座系**里，`v = -ω × q`（q 为轴线上一点）。
- `M` = 零位下的工具系，以 **SE(3) 指数坐标**给出。本机型 `M = [0,0,0,0,0,0.6752]`，即纯平移 (0, 0, 0.6752)。
- 每臂 **7 自由度**（`screws` 有 7 条）。

验证过 `screws[i]` 与 `linksXYZ[i]` 自洽：例如 `screws[1] = [0,1,0, -0.1313,0,0]`，ω=(0,1,0)、v=(-0.1313,0,0) ⇒ 轴线过点 `(0, 0, 0.1313)`，而 `linksXYZ[1] = [0,0,0.1313]` ✓。

## 两个必须知道的约定

### 1. RPY 是 ZYX 顺序

\[ R = R_z(r_z)\, R_y(r_y)\, R_x(r_x) \]

用 XYZ 顺序会有 **0.687°** 的定向误差。这是实测出来的：同一批 IK 结果，`--rpy-convention zyx` 旋转误差 0.0000°，`xyz` 则 0.6871°。

### 2. `get_IK_joint_position` 与 `get_tcp_pose` / `get_FK_pose` 用的不是同一个 TCP

实测（8 个目标位姿，双臂；与下面「校验结果」的 7 个是**两次不同的测量**，记录里位姿数不同，产生它们的脚本 `validate_kin.py` 已不在仓库里，无法再核对是否同一批位姿）：

| 命令 | 零位 TCP z (m) | 说明 |
|---|---|---|
| `get_tcp_pose` | **0.6752** | 等于 yml 的 `M`，不含偏置 |
| `get_FK_pose` | 0.6755 | 与 `M` 差 0.3 mm，同样不含偏置 |
| `get_IK_joint_position` | 0.7599（推算） | **多出 84.721 mm** |

测法：让厂家 IK 解出关节角，再用本库的 FK 正算，在**工具坐标系**下残差是一个**恒定平移** `(0, 0, 84.721) mm`，跨位姿标准差仅 0.02–0.05 mm，旋转偏差 1.3e-3（≈0.07°，在 IK 容差内）。

所以本库提供 `tcp_offset` 参数：

```python
arm.fk_pose(q, "zyx", tcp_offset=0.0)          # 对齐 get_tcp_pose / get_FK_pose
arm.fk_pose(q, "zyx", tcp_offset=IK_TCP_OFFSET) # 对齐 get_IK_joint_position
```

`IK_TCP_OFFSET = 0.084721`。

> yml 里 `linksXYZ[5] = [0, 0, -0.084668]`（84.668 mm）与此量级吻合，怀疑厂家的 IK 前向模型比 FK 多算了一节连杆。

**这个结论的适用范围（重要）**：上面的 84.721 mm 是用**厂家 IK 解出的关节角**代进**本库的 yml FK** 拟合出来的，它只说明「往本库的 yml FK 上加这个偏置，能让它和厂家 IK 的目标位姿对上」。它**没有**证明：

- 厂家 IK 与**一般位姿**下的 `get_tcp_pose` / `get_FK_pose` 之间存在恒定 84.721 mm 偏置 —— 那条链子没有测过；
- 同一位姿分别走 `get_IK_joint_position` 和 `MoveJ_P` 会差 84.7 mm。`MoveJ_P` 收的是笛卡尔位姿而不是关节角，两者不是可以直接串联的调用链，这个说法**只是推测**，真机要单独验证（`hardware-acceptance.md` P0-2）。

另外注意：那一节校验调的是本库的 **FK**（`fk_pose`），**没有**调用本库的数值 IK（`ArmKinematics.ik`）。所以「本库 IK 与厂家 IK 一致到 0.089 mm」不成立；0.089 mm 是「本库 FK + 该偏置 vs 厂家 IK 的目标位姿」。

## 校验结果

用厂家的 `get_IK_joint_position` 作为真值来源（`get_FK_pose` 有堆溢出，见 [sdk-usage.md](sdk-usage.md) §6.1），对 7 个目标位姿、双臂做闭环：

```
tcp_offset=0 (matches get_tcp_pose):
  max position error : 84.758 mm      ← 恒定偏置
  max rotation error : 0.0000 deg
tcp_offset=0.084721 (matches get_IK_joint_position):
  max position error : 0.089 mm
  max rotation error : 0.0000 deg
```

复现：产生上表的校验脚本（`validate_kin.py`）随 WebSocket 那层一起移出了仓库，没有保留下来；它当时是通过厂家应用暴露的 IK 驱动的。**FK 侧的对齐校验仍在仓库里**：`research/vendor-tools/probes/validate_fk_direct.py`，走 `sdk_probe fkvec` 直接对比两边的 `getFKpose`。

另外：

- RPY 提取/构造互为逆运算，5000 组随机旋转回环误差 ≤ 1.3e-14。
- 本库自带数值 IK（LM 阻尼最小二乘）：60 组随机目标全部收敛，位置误差 0.00000000 mm。
- 厂家的 IK 解本身有 0.07–0.09 mm 残差（受 `time_out: 0.002` / `max_size: 50` 限制），本库能算到机器精度。

## ⚠️ 三方不一致：yml / `getFKpose` / `get_IK_joint_position`

用 C++ 探针直接调 `getFKpose`（见 [sdk-usage.md](sdk-usage.md)）可以看出，**上面这套校验只证明了模型与厂家的 IK 一致，并没有证明它与厂家的 FK 一致**。实测下来这三者互不一致：

| 对比 | 结果 |
|---|---|
| 我的模型（yml + 84.72 mm 偏置） vs 厂家 `get_IK_joint_position` | ✅ 0.089 mm |
| 我的模型（yml） vs 厂家 `get_FK_pose`，一般位姿 | ❌ **200–700 mm / 100°+** |
| 我的模型（yml） vs 厂家 `get_FK_pose`，单关节激励 | 见下 |

> 上表第 3 行只说「单关节激励」：**未经修正的 yml 模型**在单关节激励下平移差 9–24 mm（`screws[1/3/5]` 那三个轴），旋转 0.0000°。**0.05 mm / 0.3 mm 那组数字属于下面那个"反解修正过的实验模型"**，不是 `vendor_model/kinematics.py` 里实际实现的那套。

逐关节单独给 0.5 rad 时：

- **旋转全部吻合**（7 个关节都是 0.0000°）→ ω（螺旋轴方向）和关节顺序是对的。
- **平移只在 `screws[0/2/4/6]`（轴过原点，v=0）吻合**；`screws[1/3/5]`（v≠0）差 9–24 mm。

从厂家的 FK 数据反解这三个轴的实际位置：

| 关节 | yml 的 `v` 隐含轴高 | 从 `getFKpose` 反解出的轴高 |
|---|---|---|
| 1 | 0.1313 | **0.1119** |
| 3 | 0.3919 | **0.3440** |
| 5 | 0.6013 | **0.6205** |

用这三组修正值 + 把 `M` 的 z 改成 0.6755 后，**单关节激励全部收敛到 0.05 mm**。但**多关节组合仍然对不上**。我试过：

- 空间系 / 体系两种 PoE 写法
- 正序 / 逆序组合
- 全部 5040 种螺旋轴排列

最好的一组仍有 403 mm 误差。所以厂家的 `getFKpose` 不是"这些螺旋轴的某种简单乘积"。

**结论：yml、`get_FK_pose`、`get_IK_joint_position` 是三份互相矛盾的模型。**

实际影响：

1. 本库目前**只对齐了厂家的 IK**。如果你的用法是"给目标位姿 → IK → 下发关节角"，本库与厂家 IK 一致，可用。
2. 如果你要**预测某个关节角组合下 TCP 在哪**（例如离线做碰撞检查、可视化），本库目前**不可信**，必须先确定厂家 FK 的真实模型。
3. 真机到手后最直接的判定办法：让机器人运动到若干已知关节角，读 `get_tcp_pose`，看哪套模型对得上。这是唯一能一锤定音的实验。

## 用法

```python
# PYTHONPATH=research/vendor-tools/python
import numpy as np, vendor_model as K

arm = K.load_arm("left", "/usr/etc")          # 或 --config-root 指向你的 etc 目录
q   = [0.0]*7

T = arm.fk(q)                                  # 4x4，臂基座系
pose = arm.fk_pose(q)                          # [x,y,z,rx,ry,rz]，ZYX
q_sol, ok = arm.ik([0.2, 0.0, 0.5, 0, 0, 0], tcp_offset=K.IK_TCP_OFFSET)
```

命令行：

```bash
PYTHONPATH=research/vendor-tools/python python3 -m vendor_model.cli --arm left fk --joints "0 0 0 0 0 0 0"
PYTHONPATH=research/vendor-tools/python python3 -m vendor_model.cli --arm right fk --joints "0.1 -0.2 0.3 -0.4 0.5 -0.6 0.7"
```

## 还没做的

- **IK 不做碰撞检查，也不做限位规避**（只按 yml 的 `limits` 做夹紧）。随包的位置限位只有 yml 里这组 ±3.1415，像占位值；`params.yml` 的 `UseLimit` 管的是速度 / 加速度限制，不是位置限位（`hardware-acceptance.md` P0-4）。真机上要注意。
- **没有多解选择**：数值解依赖种子，返回的是种子附近的解，不保证与厂家 IK 选同一个分支。厂家那侧可能有肘部/腕部构型偏好（`ik_tolerance_emog_elbow` / `ik_tolerance_ev_elbow` 等参数暗示了按肘/腕分类的解析分支）。
- **没做轨迹规划/插值**：`bot_traj_planner` 里用的是时间最优（TOTP）+ 迭代样条（`planner_*.yml` 里 `TrajectoryType: 1`），要复现得另做。
- **没有校验右臂**：`kinematics_rightArm.yml` 已加载并且验证里双臂都通过了，但右臂的 `M` 与左臂相同（都是 0.6752），如果真机上两臂 TCP 不同需要修正。
