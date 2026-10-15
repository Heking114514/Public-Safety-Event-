# vehicle_loopback_sim Changelog

## Unreleased

- 2026-10-15：Contrast 避障闭环新增唯一 /clock 时钟源；运行中 GUI 可在 0.1x～10x 调节时间倍率。物理积分不额外乘倍率，车辆速度、转弯参数和碰撞逻辑不变。
- GUI 显示时钟源确认后的实际倍率，支持页面重连；滑块请求合并并按顺序发送，防止拖动时被后台刷新抢回。界面刷新保持墙钟频率。
- 旧生产/双车回环保留原时间机制；倍率控件等待统一时钟时禁用。中文说明明确统一仿真时钟与离线 playback 的区别。

- comparison mission 在规划器返回 `planned_only` 时按原样发布同一 `navigation_path`，恢复双控制器 A/B 的实际执行链；正式任务总控不受影响。
- 2026-10-11：loopback 默认仿真起点沿初始车头反方向后移 `20 mm`，仅用于当前转角 A/B 实验；可通过 `initial_x:=0.0` 恢复原起点。
- 2026-10-11：静态真值碰撞检查不再重复叠加规划器的 `2 mm` 转弯余量；该余量会在车体实际与障碍仅相差 `0.052 mm` 时误停车。仿真器现在只用真实车体外廓判断是否进入静态障碍，规划安全余量仍由规划器和控制器负责。
- 2026-10-11：loopback 仿真器读取 `arena_config`，使用共享车体 footprint、场地自由区域和静态障碍进行真值碰撞检查；候选姿态越界时拒绝积分并保持停车，避免仿真车辆穿出场地后再由规划器被动报碰撞。
- 2026-10-11：静态碰撞余量默认调整为 `2 mm`，与规划器闭环转弯扫掠保持一致，避免用直行 `15 mm` 余量误拦截合法转弯。
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
