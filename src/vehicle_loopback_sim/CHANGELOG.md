# vehicle_loopback_sim Changelog

## Unreleased

- 增加离线真值测试工具，明确区分闭环验收和加速回放，并校验状态 JSON 与时间单调性。
- 增加仿真时间契约：默认 `closed_loop` 模式强制 `playback_speed=1.0`；只有显式的 `offline` 模式允许加速，并在日志中标记为非验收结果。
- 增加 `scripts/start_loopback_sim.sh`，启动规划器/控制器仿真前清理残留进程，并检查 GUI 端口是否被无关进程占用。
- Migrated `vehicle_loopback_sim_node`, its motion/obstacle models, and the
  optional `loopback_route_controller` from Python to modular C++17 while
  preserving executable names, topics and parameters.
- Switched the package to `ament_cmake_python`: realtime nodes are compiled,
  while the browser GUI and legacy comparison helper remain Python tools.
- Added `playback_speed` to accelerate the formal controller timer and truth
  integration together without changing real-vehicle parameters.
- Static map geometry is no longer connected to the simulated sensor. Only
  GUI/manual obstacles can publish simulated obstacle events.
- Added simulated front-obstacle recognition for GUI/manual obstacles through
  the existing `/obstacle/front_range` and `/obstacle/front_blocked` interfaces.
- Added GUI placement and clearing for temporary simulated obstacles.
- Tuned simulated obstacle recognition to use a fixed `0.25 m` trigger distance.
- Snapped GUI temporary obstacles to the nearest road centerline and oriented
  them crosswise to the lane so they block the clicked passage.
- Matched GUI temporary obstacle width to the `0.20 m` road width, set obstacle
  depth to `0.10 m`, and drew a dashed forward detection fan in the GUI.
- Disabled the simulated obstacle stable-detection delay by default so front
  obstacle reports are published immediately.
- Fixed fixed-distance obstacle detection so obstacles already inside the
  `0.25 m` trigger range are reported instead of requiring an exact boundary
  crossing.
- Published detected simulated obstacles on `/perception/dynamic_obstacle` so
  the production mission manager can replan with an obstacle polygon.
- Switched the default planner/control loopback launch to the production
  `navigation_mission_manager` instead of the compatibility
  `loopback_mission_node`.
## 2026-10-08

- GUI 红色路线图例改为“导航器当前路线”，与黑色实际车迹明确区分；路线摘要展示 frame、
  ROS 时间戳、点数和首末坐标，便于对照规划器输出与导航器接收内容。
