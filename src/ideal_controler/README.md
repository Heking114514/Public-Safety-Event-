# ideal_controler

`ideal_controler` 是全局避障规划的隔离验证包。它把车辆等效为质点，按前视点
计算理想圆弧速度，不执行真实车体 footprint、轮距、死区或转弯动作门控。

它订阅总控下发的 `route_input`，发布路线 ACK、当前航点和导航状态。遇到
`/obstacle/front_blocked` 时只停车并报告 `OBSTACLE_REPLAN_REQUIRED`，跳过正式车体的
历史轨迹倒退恢复；重规划由
`navigation_mission_manager` 和 `new_map_planner` 完成，避免重复请求。

## 启动

```bash
ros2 launch ideal_controler ideal_controler.launch.py
```

需要同时启动 `new_map_planner`、`navigation_mission_manager` 和仿真器。

一键启动隔离仿真：

```bash
ros2 launch ideal_controler ideal_obstacle_test.launch.py
```

该入口只启动规划器、任务总控、loopback 底盘和理想控制器，不启动正式
`final_navigation`。理想控制器向 loopback 底盘的 `/cmd_vel_nav` 发布命令，并关闭
loopback 的真实车体静态碰撞保护，以保持质点验证假设。GUI 中放置的障碍仍通过现有仿真
感知链发布。
