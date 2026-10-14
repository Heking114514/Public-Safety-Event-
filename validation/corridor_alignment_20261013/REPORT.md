# 20 cm 直过道内的航向摆正：数学与独立仿真验证

**结论：在当前配置和下述模型条件内，初始航向误差 −40°～+40°、满足 2 mm 安全余量的横向起点，存在前进／后退调整后摆正至 ±3°并停车的轨迹，初始位置前后各 20 cm 的活动空间足够。**

本次只新增独立验证文件，没有修改现有导航控制器、参数或地图。验证的是动作的可行性；现有避障恢复流程尚未接入这一动作。

## 验证范围与结果

过道沿 x 方向，边界为 `y = ±0.100 m`。令初始车身中心为 `x = 0`，活动范围为相对初始位置的 `x ∈ [−0.200, +0.200] m`。车身中心可以有不同的初始 y，初始速度为零。除车身中心外，实验还要求**整个含余量的车身矩形**留在上述纵向范围内，这是比只限制车身中心更强的检查。

“合法横向起点”按现有规划余量定义：125 mm 方形车身的每条边外扩 2 mm，得到 129 mm 方形检查轮廓，起点的完整轮廓须在过道内。

| 指标 | 当前回环运动模型 | 增加物理加速度限制的模型 |
|---|---:|---:|
| 网格起点 | 32,421 | 1,701 |
| 随机起点 | 10,000 | 1,000 |
| 通过 / 总数 | **42,421 / 42,421** | **2,701 / 2,701** |
| 最大车身中心累计路程 | 6.193 cm | 5.857 cm |
| 最大车身中心纵向偏移 | 5.463 cm | 单独输出整体轮廓范围 |
| 含余量轮廓的最大纵向绝对坐标 | 11.913 cm | 11.736 cm |
| 最长完成时间，含停车窗口 | 2.133 s | 2.133 s |
| 最大终态航向误差 | 3° | 3° |
| 碰撞或活动范围越界 | 0 | 0 |

初始误差已经在 ±3° 内的案例直接保持停车，所以总体最大终态误差为 3°。主模型中其余案例转到 0°，最大数值残差约 `5.41×10⁻¹⁴°`；这只是确定性仿真的计算残差，不代表实车定位精度。

所有指标的最大值分别取自各自最差案例，不要求出现在同一个案例中。主模型全程最小“扣除配置余量后的间隙”为 0，来自包含在集合中的边界起点；没有侵入检查边界。实际 125 mm 矩形在这些姿态下仍至少保留 2 mm 侧向几何间隙。

原始结果：[主模型汇总](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/summary.json)、[主模型逐案例数据](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/cases.csv)、[加速度受限模型汇总](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/physical_accel_summary.json)、[加速度受限模型逐案例数据](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/physical_accel_cases.csv)。

## 几何和运动模型

尺寸与轮轴位置读取自 [vehicle_physical.yaml](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/vehicle_config/config/vehicle_physical.yaml)：车身长宽均为 0.125 m，车身中心位于轮轴中点前方 `d = 0.030 m`，轮距 0.1466 m。配置中横向轮轴偏移为零。

令 `a = 0.125/2 + 0.002 = 0.0645 m`，航向相对过道为 θ。含余量轮廓的横向半宽为：

```text
h(θ) = a (|cos θ| + |sin θ|)
合法横向偏移：|y₀| ≤ 0.100 − h(θ₀)
```

40° 时，裸车身横向占用约 176.1 mm，含余量轮廓占用约 181.739 mm。因此此时车身中心的合法横向偏移仅为 **±9.130 mm**。不能把居中起点的结果直接用于所有靠墙起点。

轮轴速度为 v、角速度为 ω，车身中心的真实偏移运动方程为：

```text
ẋ = v cos θ − d ω sin θ
ẏ = v sin θ + d ω cos θ
θ̇ = ω
```

独立实验直接编译使用现有 [motion.cpp](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/vehicle_loopback_sim/src/motion.cpp) 中的 `integrate_base_link` 和 [checker.cpp](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/navigation_safety/src/checker.cpp) 中的车身轮廓与安全检查，包含轮轴偏移。

约束读取自 [waypoint_navigation.yaml](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/visual_navigation/config/waypoint_navigation.yaml) 和 [contrast_controller.yaml](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/contrast_controller/config/contrast_controller.yaml)：控制频率 30 Hz，最小非零线速度指令 0.035 m/s。本实验采用角速度上限 0.65 rad/s，角速度加速和回零均使用 1.80 rad/s²，满足现有数值限幅器两个方向的约束，并低于 3.00 rad/s² 的另一方向限值。每个转动步骤都伴随前进指令；没有使用零线速度原地旋转。

当前 [output.cpp](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/contrast_navigation_core/src/output.cpp) 的最小指令速度规则会覆盖起步时的线速度速率限幅，而 [vehicle_loopback.yaml](/home/hjh/zhongzhouyanda/Public-Safety-Event-/src/vehicle_loopback_sim/config/vehicle_loopback.yaml) 默认关闭物理线速度、角速度加速度限幅。因此主模型忠实复现指令限幅和现有回环运动模型；另外增加实际线速度加速 0.25 m/s²、减速 0.80 m/s²、实际角速度变化率 1.80 rad/s² 的离散物理模型。第二组保持 30 Hz 指令更新，以 600 Hz 更新实际速度，允许起步和制动中的实际速度小于最小非零指令速度。

## 可行轨迹的构造

1. 初始误差已经在 ±3° 内时保持停车。
2. 其余情况先生成一个角速度先增加、后回零的离散脉冲，使航向单调从 θ₀ 变到 0。非零角速度步骤的前进指令为 0.035 m/s。角速度峰值不超过 0.65 rad/s，逐周期变化不超过 `1.80/30 rad/s`。
3. 用完整车身扫掠计算该弧线允许的起始横向位置区间。若当前 y 不在区间内，先保持原航向，沿原朝向前进或后退少量距离，把 y 挪入区间。该区间额外保留 1 mm 转弯准备余量。
4. 准备动作结束后发送零速度命令，经过停车窗口，再执行摆正弧线。最终发送零速度命令至少 7 个控制周期，即约 0.233 s，并检查终态速度为零。加速度受限模型也计算停车过程中的剩余运动。

在固定 θ 的准备动作中，y 单调变化，起终点都满足初始合法横向范围，因此整段直线的横向车身轮廓也安全。主模型中最大准备动作累计行程为 1.867 cm。

这套构造能根据靠墙方向选择前进或后退，不要求横向回到过道中心线。两个运动模型分别按自己的实际运动轨迹计算安全起点区间与准备距离；第二组不是把第一组的开环指令直接套给一个响应不同的车辆。

## 连续扫掠和连续角度范围

扫掠检查没有只看控制周期末端。每段恒定 v、ω 的运动中，车角坐标可写成：

```text
轮轴初始位置为 (xA₀, yA₀)，r = v/ω
车角相对轮轴为 (cx, cy) = (d ± a, ±a)
x角(θ) = xA₀ − r sin θ₀ + cx cos θ + (r − cy) sin θ
y角(θ) = yA₀ + r cos θ₀ + (cy − r) cos θ + cx sin θ
```

程序检查每个车角的区间端点和区间内全部导数为零的位置，得到连续运动的真正四角极值。直线段检查端点即可。此处过道为单个凸矩形，因此四角始终在矩形内就保证整个矩形轮廓在内。

此外，主模型进行了网格采样之间的连续角度包络。固定角速度脉冲长度 N 时，`θ(t) = f(t) θ₀`，其中 `0 ≤ f(t) ≤ 1`。任一车角坐标相对初始航向的导数绝对值可由下式保守约束：

```text
L = v N Δt + 2d + sqrt(a² + a²)
```

对时间取极大／极小后仍保留该 Lipschitz 界。安全转弯起点区间两端各保留 L 界，区间宽度保留 2L 界。程序在每个脉冲长度变化处明确分段，再细分为不超过 0.01° 的 3,725 个小区间，使用两端数值和解析导数界包络整个区间，另加 1 nm 数值垫量。

连续范围包络结果为：

| 数学包络量 | 结果 |
|---|---:|
| 全范围安全转弯起点区间宽度下界 | 12.840 mm |
| 最大单周期准备动作横向位移 | 0.750 mm |
| 准备动作累计路程上界 | 2.070 cm |
| 车身中心全程累计路程上界 | 6.680 cm |
| 含余量轮廓纵向绝对坐标上界 | 15.802 cm |
| 可用纵向绝对坐标范围 | 20.000 cm |

区间宽度大于整数控制周期引起的横向步长，因此准备动作不会因为 30 Hz 步长而跳过整个安全区间。最后一项由“中心路径长度上界 + 轮廓外接半径”得到，较仿真实测的 11.913 cm 保守。

在主模型的确定性条件下，上述包络支持全体 `θ₀ ∈ [−40°,40°]` 及合法横向偏移的可行性。它是解析导数界结合双精度数值包络的验证，不是形式化证明工具的证书。加速度受限模型另有 2,701 个起点的仿真证据，本次没有把主模型的连续区间包络外推到第二个模型。

证据：[连续角度范围结果](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/continuous_certificate.json)。

## 对照与交叉核验

主网格使用 0.25° 的角度步长：321 个航向 × 101 个合法横向比例，包括正负两侧的边界起点，共 32,421 个；另加种子 `20261013` 的 10,000 个随机起点。物理加速度受限网格使用 1° 步长与 21 个横向比例，共 1,701 个，加 1,000 个随机起点。

对同一批 42,421 个主模型起点，只执行摆正弧线、跳过准备挪动时，**1,692 个起点违反 2 mm 安全余量边界**；最差侵入该检查边界约 3.388 mm。这说明航向误差相同并不意味着可以采用相同起步动作，也不能把准备阶段简化为固定方向的一次后退。这里的对照是本次实验的固定弧线模板，不是现有导航控制器的运行结果。

选取 ±10°、±30°、±40°，分别配合左右边界和居中起点，共 18 条轨迹，再用独立的车身中心微分方程 RK4 积分进行交叉核验。每个控制周期细分 100 次，共 85,600 个子步；与现有 C++ 精确积分的最大终点状态差约 `2.34×10⁻¹⁴`，所有轨迹满足几何和终态检查。

证据：[独立积分核验结果](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/independent_verification.json)。

![合法起点、安全转弯区间、靠边轨迹和纵向活动范围](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/validation_overview.png)

图右上红色表示跳过准备动作时失败的网格起点；完整动作在这些起点全部通过。左下轮廓包含 2 mm 余量，横向黑线为过道边界。

## 结论的适用边界

- 初始完整轮廓必须满足上述 2 mm 余量条件。仅裸车身不碰墙、但不满足配置余量的起点没有被计入“全部合法起点”的结论。
- 活动窗口始终相对初始车身中心；本结果不表示在同一固定 40 cm 矩形中任意靠近纵向端点的初始位置都可完成。
- 验证对象是无额外障碍的 20 cm 直过道，采用现有 125 mm 方形轮廓、30 mm 轮轴偏移和确定性运动模型。没有加入定位误差、通信延时、轮胎打滑或实车执行偏差。
- 终态目标为航向误差不超过 3°并停车，没有要求横向回中。已满足 ±3° 的边界起点可以原地保持停车。
- 前后各 20 cm 在模型内有足够空间。现有恢复控制器是否选择正确动作、与障碍物和拐角联动后是否完成全任务，需要后续集成验证。

## 复现

从项目根目录运行：

```bash
python3 validation/corridor_alignment_20261013/run_validation.py
```

依赖项目已有 C++ 编译器、yaml-cpp、Python 3、numpy 和 matplotlib。命令不启动 ROS 节点、不接管正在运行的车辆、不修改现有配置；它编译独立实验、运行两组模型、生成连续角度包络、交叉核验和图表。

仅重新核验现有结果并绘图：

```bash
python3 validation/corridor_alignment_20261013/run_validation.py --verify-only
```

复现入口：[run_validation.py](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/run_validation.py)。实现：[主模型与连续角度包络](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/corridor_alignment.cpp)、[物理加速度受限模型](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/physical_accel_validation.cpp)。[输入指纹](/home/hjh/zhongzhouyanda/Public-Safety-Event-/validation/corridor_alignment_20261013/results/input_manifest.json) 记录本次使用的几何、配置和核心源文件 SHA-256。
