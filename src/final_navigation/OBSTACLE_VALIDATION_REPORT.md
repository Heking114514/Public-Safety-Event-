# Contrast 避障闭环验证报告

运行日期：2026-10-12，Asia/Shanghai。目标是在保留实车物理参数和静态碰撞保护的 loopback 仿真中，验证 Contrast 的动态障碍检测、停车、上报、重规划绕行和任务恢复。

## 结论

修复后验证通过。有效实跑捕获了以下完整链路：

`front_blocked=true + front_range` -> `OBSTACLE_BRAKING` -> `OBSTACLE_REVERSING` -> `OBSTACLE_REPLAN_REQUIRED` -> 总控携带动态障碍重新规划 -> 新路线 ACK -> `FOLLOWING`。

车辆没有撞击障碍；重规划路线离开原路，经过相邻道路绕行，并继续执行后续路线。静态碰撞保护仍为开启状态，车辆尺寸和相机前向偏移由 `vehicle_config/config/vehicle_physical.yaml` 注入。

## 实现修复

1. 新增 `vehicle_loopback_sim/launch/final_navigation_obstacle_validation.launch.py`。它启动单个 Contrast、loopback 仿真、新地图规划器和 `navigation_mission_manager`；仿真检测开关为 true，`enforce_static_collision` 保持 true。
2. 避障验证启动将 `plan_on_start=false`，因此 Comparison helper 不再发起第二套规划。正式总控是唯一任务、路线和检查点所有者，使用 loopback 的 `task_confirmation_mode=route`。
3. 总控将 Contrast 前缀下的 odom、status、ACK、motion hold、retreat、dynamic obstacle、front bool/range 全部接线到正式总控。
4. 修复 `navigation_mission_manager/src/planning.cpp`：每个规划请求设置 `validate_geometry=true` 和 `online_geometry_validation=true`。此前虽然请求带有 `dynamic_obstacles=1`，规划器却因两个碰撞校验开关默认 false 而可能返回穿过障碍的路线。
5. 增加 Comparison helper 的外部总控兼容字段和路线 ACK/旧路线保护，并添加两项回归测试。

## 有效实跑证据

证据文件：[final_navigation_obstacle_validation_final_20261012.json](/tmp/final_navigation_obstacle_validation_final_20261012.json)。最终运行使用修复后的 mission manager，启动后在车辆进入目标直路前放置障碍：

- 障碍中心：ROS map `(2.75, 1.50)` 米；GUI 实际矩形约 `x=2.70..2.80, y=1.40..1.60`。
- 所在直路：`ROAD_Y1_RIGHT`，车辆沿 `-x` 通过；障碍中心距两端约 `0.45 m`，不在路口、拐角或任务点。
- 触发时车辆真值：`(3.105, 1.500)`，线速度约 `0.050 m/s`，front range `0.2496 m`。
- `OBSTACLE_BRAKING`：记录器相对时间 `65.396 s`；`front_blocked=true` 和 range 同时收到了。
- `OBSTACLE_REVERSING`：`65.623 s`，刹停确认后开始倒车；此时 odom 线速度为约 `-0.050 m/s`。
- `OBSTACLE_REPLAN_REQUIRED`：`68.090 s`；倒退约 `2.467 s` 后上报新规划请求。
- 重规划路线：新 `plan_id=1791805479121851419`，长度 `3.025 m`，路径为
  `(3.225,1.500) -> (3.2,1.5) -> (3.2,0.5) -> (2.2,0.5) -> (2.2,1.5)`。
  该路线不再沿障碍所在的 `y=1.5, x=2.7..2.8` 直线穿越，而是经 `y=0.5` 相邻道路绕行。
- 新路线在 `68.352 s` 收到 ACK 并进入 `FOLLOWING`；之后车辆继续到达后续检查点，记录结束时仍在运行新路线而不是 FAULT。
- 传感器清除：`front_blocked=false` 在 `70.595 s`，并不撤销已经确认的 recovery 或已发布的新路线。

有效运行的过程中，关键管理器日志还记录了 `obstacles=1`、`replanning=true`。这证明障碍观测已经进入总控和规划器，而不是只有控制器本地停车。

## 先失败后修复的证据

第一次开启检测的短任务中，障碍放置太晚，车辆已经越过第一处障碍；该次只证明了时序选择不足，不作为避障成功证据。随后第二处障碍成功触发了停车、倒退和重规划，但当时路线仍接近障碍。检查发现规划请求只携带多边形，没有显式打开碰撞校验；因此补上上述两个字段并重新构建、重新实跑。最终实跑才作为有效绕行结果。

## 验证条件与复现

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select final_navigation navigation_mission_manager --symlink-install
ros2 launch vehicle_loopback_sim final_navigation_obstacle_validation.launch.py \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 gui:=true
```

等待 GUI `http://127.0.0.1:8766/` 显示车辆沿直路行驶后，向 GUI HTTP 接口添加手动障碍。接口坐标是 ROS map 坐标，示例：

```bash
python3 - <<'PY'
import json, urllib.request
body=json.dumps({'x_m': 2.75, 'y_m': 1.50}).encode()
request=urllib.request.Request(
    'http://127.0.0.1:8766/manual_obstacles', data=body,
    headers={'Content-Type': 'application/json'}, method='POST')
print(urllib.request.urlopen(request).read().decode())
PY
```

记录以下话题：

- `/comparison/final_navigation/obstacle/front_blocked`：best-effort Bool
- `/comparison/final_navigation/obstacle/front_range`：best-effort Range
- `/comparison/final_navigation/navigator_status`：durable String
- `/comparison/final_navigation/navigation_recovery_state`：durable recovery 锚点
- `/comparison/mission_status`：总控状态文本
- `/new_map_planner/route_plan`：路线和重规划几何
- `/comparison/final_navigation/odometry/fused`：实际速度和位姿

障碍应在直路中部放置，距离路线两端至少约 `0.4 m`；不要在拐角或任务点放置。Bool 和 Range 使用仿真 SensorDataQoS，记录器必须使用 best-effort，否则会出现 QoS 不兼容而漏收消息。

## 未验证和限制

- 本次是仿真手工多边形检测，不是实车相机/视觉感知验证。
- 本次没有完成整套 12 点、4 隧道、28 条道路加返航的“障碍插入后完整任务终态”；有效实跑验证了障碍触发、恢复、绕行和后续路线继续，未宣称完整任务最终完成。
- 没有覆盖规划器确认“无路”后的多次 `continue_obstacle_retreat` 扩展和最终 FAULT 分支。
- 没有做实车制动距离认证。有效运行记录了检测 range、odom 速度和状态转换，但没有单独采集 ControlTelemetry 的零速确认样本、制动延迟统计或车头到障碍的最终毫米级安全裕量。
- GUI 手动障碍不属于静态地图碰撞模型；静态地图碰撞保护本身保持启用，并未把手工障碍混入静态地图。
- 之前的完整 Contrast 无障碍任务报告仍然有效，但不能证明自动避障；本报告的 JSON 才是动态避障证据。
