# Contrast 控制器 A/B 仿真报告

## 测试条件

- 正式控制器与 `contrast_controller` 接收同一个 `/waypoint_navigation/route_input`。
- 两辆 `vehicle_loopback_sim` 使用同一初始位姿、`0.125 m × 0.125 m` 车体、`30 mm` 轴心偏移和相同地图。
- 两侧静态车体碰撞保护均开启，未关闭或放宽地图保护。
- 两侧除 `TurnManeuverController` 实现和本包转弯参数外，共用同一个 `waypoint_navigator_core`。
- 任务点使用 A/B 专用真值检查点：匹配当前 route ACK、目标距离不超过 `0.05 m`、实际线/角速度分别不超过 `0.005 m/s` 和 `0.02 rad/s`，连续保持至少 `1 s` 后授权继续。该流程不模拟物理 RFID、位置修正或播报。

## 结果

| 路线 | 正式控制器 | Contrast 控制器 |
| --- | --- | --- |
| 首个右转，到目标 1 `(0.2, -0.5)` | 卡在 `TURN_60`；真值速度为零，持续触发静态碰撞保护 | `GOAL_REACHED`；终点约 `(0.1955, -0.4612, -88.6°)` |
| 首个左转，到目标 12 `(0.2, 0.5)` | 卡在 `TURN_60`；真值速度为零，持续触发静态碰撞保护 | `GOAL_REACHED`；终点约 `(0.1975, 0.4602, 89.2°)` |

正式控制器在两侧都继续发布约 `0.035 m/s` 的线速度和 `±0.17 rad/s` 的角速度，但 loopback 真值速度保持为零。Contrast 控制器以 `0.035 m/s` 和 `0.50–0.65 rad/s` 执行固定动作，通过完整车体碰撞检查后进入出口道路，并完成检查点停车。

## 复现

```bash
source install/setup.bash
ros2 launch contrast_controller contrast_comparison.launch.py
```

默认 GUI：

- Production：`http://127.0.0.1:8765/`
- Contrast：`http://127.0.0.1:8766/`

只比较单个方向：

```bash
# 右转到目标 1
ros2 launch contrast_controller contrast_comparison.launch.py \
  planner_mode:=layer1 target_ids:=1 return_home:=false

# 左转到目标 12
ros2 launch contrast_controller contrast_comparison.launch.py \
  planner_mode:=layer1 target_ids:=12 return_home:=false
```

## 结论边界

该结果证明当前 loopback 地图、车体和轴心模型下，Contrast 动作可以通过 launch 路口左右两个方向，而正式动作会被静态碰撞保护拦截。它不等于实车标定结论，也不覆盖全地图所有路口、真实电机滞后/打滑、物理 RFID 或定位纠偏。实车应用前仍需低速分阶段验证并保留转弯无进展锁停。
