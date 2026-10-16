# Contrast 局部避障恢复验收报告

日期：2026-10-13（Asia/Shanghai）  
范围：`final_navigation` 的 Contrast 控制器、`navigation_mission_manager` 总控、`new_map_planner` 和 `vehicle_loopback_sim` 现有仿真环境。  
证据目录：最终针对性分支为 `/tmp/contrast_fallback_measured_final/`，最终六场正式地图回归为 `/tmp/contrast_final_measured_regression/`；历史严格转弯交接复验为 `/tmp/contrast_strict_turn_all_final/`，较早前进调整回归为 `/tmp/contrast_forward_adjust_fix_all/`。

## 结论

本轮完成了基于实际 ROS 2 仿真的 Contrast 局部避障恢复实现，并运行了六个独立域场景。六个场景全部通过，满足“发现障碍 → 停车 → 候选出口评估 → 倒车到合法转弯点 → 换向停车 → `CAPTURE` 停车确认 → `APPROACH_ENTRY` → `TURN_60` → `DIAGONAL_CREEP` → `TURN_30` → 实际出口完成 → 正常直行”的验收条件；其中包括正常 90° 转弯后再遇障碍的场景。恢复转弯不再由独立目标跟踪弧线合成，而是复用 Contrast 的唯一固定转弯控制器。

## 最终前进调整分支验证

本轮仅修改 Contrast 包内的恢复状态机和验收工具，正常控制与恢复 YAML 参数保持原值。修正两个已复现的问题：前进时不能朝身后的锚点计算 `atan2`，否则航向误差接近 π 而持续大角速度转向；前进结束不能直接倒车，需要再次确认零速。前进调整期间禁止到点交接，调整后的停车不消耗回撤重试时间预算，且只允许一次有限时长重试。

针对性真实 ROS 2 回环证据：`/tmp/contrast_fallback_measured_final/retreat_sweep_fallback/`，域 226，`result.json` 为 PASS。实测如下：

| 检查项 | 结果 |
|---|---|
| 首次回撤扫掠拒绝 / 前进开始 / 前进结束 / 停车后重试 | 各 1 次 |
| 有界前进阶段 | 0.233392 s（配置计算值 0.225 s，按 30 Hz 周期终止） |
| 实测前进样本轨迹长度 | 0.007798 m（12 个运动样本间累计距离） |
| 调整前 / 调整后实测零速样本 | 11 / 12 |
| 回撤重试实测倒车样本 | 847 |
| 严格转弯启动次数 | 1 |
| 转弯交接位置误差 | 0.019931 m |
| 完成后实测连续直行 | 0.200526 m |
| 故障 / 碰撞保护介入 / 协议错误 | 均无 |

停车样本要求 `abs(linear)<0.002 m/s`、`abs(angular)<0.03 rad/s`；控制器自身还检查左右轮速及现有 `recovery_switch_stop_s=0.22 s`。源代码保留 `recovery_linear_speed=0.035`、`recovery_angular_speed=0.65`、`recovery_primitive_s=0.45`、`recovery_margin_m=0.002` 等现有参数。所有前进与重试命令仍执行限幅后车身扫掠预测。

### 专用夹具和适用范围

该夹具从 `(1.2,-1.1,-96°)` 出发，标准动态矩形障碍位于 `(1.2,-1.4)`。生成的测试地图额外加入 arena 坐标 `[0.562,3.030,0.563,3.031]` 的 1 mm 角点障碍。规划器正式直行余量为 15 mm，会在恢复前将这个边界夹具判为起点碰撞；因此仅在输出目录中的 `fixture_arena.yaml` 将规划余量设为 2 mm，与恢复检查一致。生产地图、车辆物理尺寸、控制参数和碰撞保护均不修改。这个场景是控制器扫掠边界测试，不能作为正式 15 mm 规划配置下全部障碍可恢复的结论。

可复现入口自动保存测试地图、原始地图路径、场景配置、原始 ROS 事件、里程计、命令、日志及轨迹：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/vehicle_loopback_sim/scripts/run_pose_recovery_scenarios.py \
  --forward-adjustment-fixture --base-domain 226 \
  --output /tmp/contrast_forward_adjustment_new --timeout 75
```

`--forward-adjustment-fixture` 自动要求完整调整分支，普通恢复通过不能满足验收。默认运行器仍只运行六个正式地图场景。

持续阻断的反例保存在 `/tmp/contrast_fallback_corrected_trigger5/`：正式 15 mm 规划余量，附加角点矩形 `[0.580,3.033,0.582,3.035]`。该例只调整一次，第二次确认停车后重试仍被拒绝，进入 `FAULT_POSE_RECOVERY_SWEEP`。它验证安全拒绝和无无限重试，不能记为恢复成功。

## 最终六场正式配置回归

使用最终版本验收器与正式地图运行，证据为 `/tmp/contrast_final_measured_regression/`。六场均通过新增的回撤换向停车检查和单次严格转弯检查；无故障、碰撞保护介入或协议错误。标准六场没有触发前进调整，其分支覆盖以单独的角点夹具为准。

| 场景 | ROS 域 | 恢复后实测连续直行 |
|---|---:|---:|
| `south_nominal` | 215 | 0.200524 m |
| `north_mirror` | 216 | 0.200693 m |
| `south_yaw_plus6` | 217 | 0.200528 m |
| `south_yaw_minus6` | 218 | 0.200393 m |
| `south_less_space` | 219 | 0.200527 m |
| `after_normal_90deg_turn` | 220 | 0.200732 m |

两个 Contrast 包构建通过；`ctest --test-dir build/final_navigation --output-on-failure` 的 3 个注册测试全部通过；最终验收脚本语法检查与 `git diff --check` 通过。`contrast_navigation_core` 无单独 CTest，其恢复状态机由真实 ROS 回环验证。

## 较早前进调整回归（历史证据）

针对旧版本在 `RETREAT_TO_TURN_POINT` 扫掠阻断后直接报 `FAULT_POSE_RECOVERY_SWEEP` 的问题，重新构建后运行六个独立 ROS 域场景。六场均完成障碍检测、恢复、实际出口交接和恢复后连续直行，没有出现 `FAULT`、静态保护拦截或新的前进调整安全故障。

| 场景 | ROS 域 | 结果 | 恢复后实测连续直行 | 出口道路 |
|---|---:|---|---:|---|
| `south_nominal` | 215 | 通过 | 0.20073 m | `ROAD_X1_Y2` |
| `north_mirror` | 216 | 通过 | 0.20043 m | `ROAD_X1_Y2` |
| `south_yaw_plus6` | 217 | 通过 | 0.20044 m | `ROAD_X1_Y2` |
| `south_yaw_minus6` | 218 | 通过 | 0.20057 m | `ROAD_X1_Y3` |
| `south_less_space` | 219 | 通过 | 0.20056 m | `ROAD_X1_Y2` |
| `after_normal_90deg_turn` | 220 | 通过 | 0.20056 m | `ROAD_X1_Y2` |

该补丁保留了原有 `recovery_switch_stop_s` 和恢复速度参数；前进调整只在倒车扫掠拒绝后触发一次，先停车，再沿入路航向结合实时横向误差执行有界前进，结束后再次停车确认，最后只重试一次有限时长回撤。每条命令继续使用同一车身扫掠安全检查。六场标准夹具未触发该兜底分支，因为其倒车预测本身安全；这组证据验证了修复没有改变正常恢复协议和既有验收行为。

## 严格转弯交接复验

旧的恢复 `TURN_INTO_EXIT` 会根据目标位置和航向自行合成带角速度的运动命令，可能在转弯点附近形成自由弧线。本轮将其替换为已有的 Contrast `TurnManeuverController`，恢复流程固定为：

```text
RETREAT_TO_TURN_POINT
    -> 零速度和轮速停止确认
    -> CAPTURE
    -> APPROACH_ENTRY
    -> TURN_60
    -> DIAGONAL_CREEP
    -> TURN_30
    -> COMPLETE
```

`CAPTURE` 只在合法转弯点附近的停车确认之后启动；`APPROACH_ENTRY`、`TURN_60`、`DIAGONAL_CREEP` 和 `TURN_30` 的输出全部来自同一个严格控制器。每条非零输出在发布前都使用实时位姿、限速后的速度和短时域运动样本执行 `poseRecovery_.safe(...)` 车身扫掠检查；检查失败会停车并报告故障。目标航向不属于单次固定转弯范围时也会安全停止，不降级为自由弧线。

最新六场证据目录为 `/tmp/contrast_strict_turn_all_final/`，六场均通过：

| 场景 | ROS 域 | 结果 | 恢复后实测连续直行 | 出口道路 |
|---|---:|---|---:|---|
| `south_nominal` | 227 | 通过 | 0.20053 m | `ROAD_X1_Y2` |
| `north_mirror` | 228 | 通过 | 0.20070 m | `ROAD_X1_Y2` |
| `south_yaw_plus6` | 229 | 通过 | 0.20051 m | `ROAD_X1_Y2` |
| `south_yaw_minus6` | 230 | 通过 | 0.20040 m | `ROAD_X1_Y3` |
| `south_less_space` | 231 | 通过 | 0.20069 m | `ROAD_X1_Y2` |
| `after_normal_90deg_turn` | 232 | 通过 | 0.20073 m | `ROAD_X1_Y2` |

六场都记录了停车后的 `POSE_RECOVERY_ALIGN_FOR_TURN` 兼容状态和 `POSE_RECOVERY_TURN_INTO_EXIT` 严格动作状态，没有 `FAULT`、协议错误或碰撞保护拦截。

## 前一轮实际场景结果（历史证据）

| 场景 | ROS 域 | 起始姿态/构造 | 结果 | 实测连续直行 | 出口道路 |
|---|---:|---|---|---:|---|
| `south_nominal` | 187 | `(1.2,-0.66,-90°)`，障碍 `(1.2,-1.02)` | 通过 | 0.20074 m | `ROAD_X1_Y2` |
| `north_mirror` | 188 | `(1.2,-0.34,+90°)`，北向镜像 | 通过 | 0.20073 m | `ROAD_X1_Y2` |
| `south_yaw_plus6` | 189 | 南向起点，航向 `-84°` | 通过 | 0.20057 m | `ROAD_X1_Y2` |
| `south_yaw_minus6` | 190 | 南向起点，航向 `-96°` | 通过 | 0.20043 m | `ROAD_X1_Y3` |
| `south_less_space` | 191 | 南向，障碍间距缩小到 0.31 m | 通过 | 0.20044 m | `ROAD_X1_Y2` |
| `after_normal_90deg_turn` | 192 | 先完成正常 90° 转弯，再在直路遇障 | 通过 | 0.20073 m | `ROAD_X1_Y2` |

六个历史场景都记录到了 `braking`、`candidates`、`planning_motion`、`motion`、`complete`，并在控制状态历史中出现 `POSE_RECOVERY_RETREAT_TO_TURN_POINT`、`POSE_RECOVERY_DIRECTION_STOP`、`POSE_RECOVERY_ALIGN_FOR_TURN`、`POSE_RECOVERY_TURN_INTO_EXIT`、`POSE_RECOVERY_COMPLETE`。这些数据用于确认前进调整修复；当前严格转弯交接以“严格转弯交接复验”章节和 `/tmp/contrast_strict_turn_all_final/` 为准。

## 场景前置条件与历史失败记录

本次严格运行中 `after_normal_90deg_turn` 已完成正常 90° 转弯前置条件，并随后完成障碍恢复；对应证据保存在 `/tmp/contrast_pose_recovery_strict_20261013_0540/after_normal_90deg_turn/`。

此前版本曾记录 `south_yaw_minus6` 的 `FAULT_POSE_RECOVERY_SWEEP` 安全拒绝和一次正常转弯前置条件超时。这些历史失败没有通过放宽正常控制参数处理；最新实现通过实时二维回中、入口门限和启动就绪同步后，六场均已通过。
## 实现要点

- 独立的 `contrast_navigation_core` 使用真实 `base_link` 与轮轴偏移积分，按地图节点/车道识别 blocked lane、T/X 路口和 left/right/reverse 出口，生成合法 `RETREAT_TO_TURN_POINT` 以及固定严格转弯的道路语义。
- 控制器每个周期根据融合里程计重新计算倒车目标距离；倒车到点后先确认停车，再捕获转弯点上下文并调用 Contrast `TurnManeuverController`。该控制器唯一推进 `APPROACH_ENTRY`、`TURN_60`、`DIAGONAL_CREEP` 和 `TURN_30`，恢复控制器不再合成自由 `linear/angular` 出口弧线。
- 每个命令及短时域预测都使用车身扫掠检查；换向和严格动作启动前都确认停止，暂停、定位无效、无进展或危险扫掠统一停车报错。单次固定动作不支持的目标转角也进入安全故障，而不改用另一种转弯。
- 控制器在刹停结束时优先锁存真实动态障碍多边形；首个 `candidates` 消息锁存障碍几何和双向 blocked lane。候选选择服务校验 `recovery_id`、障碍版本、出口和借用车道状态。
- 总控在 `candidates` 后保存实际已完成的 DRIVE/RFID 进度，逐个进行 `activate=false`、NORMAL、`ordinary_nodes_since_task=0` 的只读规划评估；候选失败时尝试下一个，不能把评估路线直接作为执行路线。
- 完成恢复后，总控使用控制器回报的实际出口位姿重新规划，普通道路计数归零；被堵普通道路单独记录为 skipped，不伪造 covered，任务点和隧道仍保持未完成。
- `vehicle_loopback_sim` 将手动障碍纳入最终动态障碍碰撞保护；运行器还会把 `collision guard blocked` 作为失败证据，避免保护介入被误报为成功。

## 构建和测试

本轮增量构建和验证命令为：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select contrast_navigation_core final_navigation --symlink-install
python3 -m py_compile src/vehicle_loopback_sim/scripts/run_pose_recovery_scenarios.py
```

回归测试命令和结果：

```bash
ctest --test-dir build/visual_navigation --output-on-failure              # 18/18
ctest --test-dir build/navigation_mission_manager --output-on-failure      # 2/2
ctest --test-dir build/final_navigation \
  -R 'contrast_(launch_contract|checkpoint)_test|turn_maneuver_test' \
  --output-on-failure                                                       # 3/3
```

`contrast_navigation_core` 当前没有单独注册 CTest；其恢复行为由核心编译和六场 ROS loopback 场景执行真实控制闭环验证。`git diff --check` 通过。六场严格场景运行器结果为 6/6，证据目录为 `/tmp/contrast_strict_turn_all_final/`，恢复后连续直行距离为 `0.20040–0.20073 m`，所有场景均无 `bad_log_lines`。

## 限制

本报告验证的是现有 loopback 的手动多边形障碍、当前地图拓扑和当前车辆参数，不替代实车制动距离或真实视觉感知认证。六个通过场景证明了本地恢复、换向停车、固定严格转弯、出口交接和恢复后续行；它们不等价于 12 个任务点、4 条隧道、24 条普通道路及返航的完整任务终态。长距离超过当前 `recovery_max_radius_m` 的真实道路场景、更多障碍组合和实车制动距离仍需单独增加证据。
