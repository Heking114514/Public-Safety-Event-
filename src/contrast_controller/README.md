# contrast_controller

`contrast_controller` 是独立的 ROS 2 对照控制包。可执行文件链接
`contrast_navigation_core` 导出的 `contrast_waypoint_navigator_core`，保留正式导航的
话题、服务、路线处理、定位健康检查和停车协议；对照动作与恢复修复均位于 Contrast
包内，不影响其他控制器的调参。

## 运行边界

- 可执行文件名和运行节点名均为 `contrast_controller`。
- 参数文件使用 `contrast_controller` 节点键。
- 车辆尺寸、轮距、轴心偏移和传感器安装位姿仍由 `vehicle_config` 的标准启动注入，
  本包不复制物理标定值。
- 转弯控制参数全部来自 `config/contrast_controller.yaml`；统一线速度低于兼容入口或
  蠕行速度时配置校验失败，因此未加载本包 YAML 不会静默使用共享头中的旧低速值。
- 转弯阶段保持为 `CAPTURE`、`APPROACH_ENTRY`、`TURN_60`、
  `DIAGONAL_CREEP`、`TURN_30` 和 `COMPLETE`。

## 控制策略

入口调整在动作开始时固定前进或倒退方向，并同时闭环修正纵向、横向和航向误差；入口位置与航向条件同时满足后才进入第一段转弯，越过入口捕获盒则停车请求新路线，不反复换向。两段转弯保持实车可响应的固定线速度，角速度由目标航向反馈计算并限制在配置的最小、最大值之间；中间直行段使用 YAML 中独立的 `25 mm` 正投影距离，达到上限仍未对准则停车。

每个运动阶段都有独立阶段超时和无进展检查。转向阶段只认可朝目标减小的航向误差，平移不能掩盖停转；超时或无实际进展时输出零命令并返回 `NEED_REPLAN`，共享导航器要求新的 route id 后才能恢复。直接调用核心时支持暂停预算，但实节点的任务 hold 会复位 maneuver，恢复后从当前路线状态重新捕获，不承诺原阶段无缝续跑。

## 避障恢复的严格转弯交接

Contrast 避障恢复先沿当前道路倒车到候选路口转弯点附近，确认车轮和融合速度已经停止，再启动同一个 `TurnManeuverController`。恢复阶段严格按 `CAPTURE`、`APPROACH_ENTRY`、`TURN_60`、`DIAGONAL_CREEP`、`TURN_30`、`COMPLETE` 执行；恢复控制器不另行合成目标跟踪弧线。严格动作的每条非零命令都经过实时车身扫掠安全检查，单次固定动作无法表示的目标转角会停车进入故障保护。恢复使用的 `recovery_linear_speed`、`recovery_angular_speed`、`recovery_primitive_s`、`recovery_max_radius_m` 和 `recovery_max_expansions` 参数保持原值。

## 回撤受阻后的有界前进调整

首次回撤命令被车身扫掠拒绝时，先停车并确认轮速，再沿入路航向与实时横向误差闭环前进一次。前进时间为 `clamp(0.5 * recovery_primitive_s, 0.20, 0.45)`，当前配置为 0.225 s；每条命令照常执行扫掠检查。结束后再次停车确认，再给一次原有有限预算的回撤重试。前进不安全或重试仍受阻时安全报错，不循环换向，也不会在调整过程中启动转弯。现有正常及恢复参数不变。

默认六场正式地图回归与专用分支验证分别运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/contrast_controller/scripts/run_pose_recovery_scenarios.py \
  --output /tmp/contrast_six_new
python3 src/contrast_controller/scripts/run_pose_recovery_scenarios.py \
  --forward-adjustment-fixture --base-domain 226 \
  --output /tmp/contrast_adjustment_new --timeout 75
```

专用夹具只在输出目录生成测试地图，加入 1 mm 角点障碍，并将测试规划余量由正式 15 mm 设为与恢复一致的 2 mm，以避免初始规划直接拒绝该边界姿态。正式地图文件不变。验收必须记录一次调整、两次实测停车、安全回撤、一次严格转弯和完成后的 0.20 m 连续直行；详情见 [POSE_RECOVERY_VALIDATION_REPORT.md](POSE_RECOVERY_VALIDATION_REPORT.md)。

## 构建与测试

在工作空间根目录执行：

```bash
colcon build --packages-up-to contrast_controller
colcon test --packages-select contrast_controller --event-handlers console_direct+
colcon test-result --verbose
```

单元测试直接积分控制器发布的线速度和角速度，不向状态机人工写入中间姿态。名义
用例检查完整阶段顺序，停滞用例保持里程计不动并检查 `NEED_REPLAN`。

## 对比仿真

包内安装了双控制器对比入口：

```bash
ros2 launch contrast_controller contrast_comparison.launch.py gui:=false
```

该入口让正式控制器和对照控制器接收同一条规划路线，同时隔离两侧的里程计、IMU、
执行器反馈、障碍输入、速度命令和状态话题。默认可启用两套独立 GUI，端口分别为
`8765` 和 `8766`；无图形环境可使用上面的 `gui:=false`。左右首个路口的实际闭环结果见
`AB_REPORT.md`。

## 完整三层任务

正式控制器在首个路口被碰撞保护拦截时，双车模式会等待两侧完成同一路线。验证 Contrast 的完整任务应使用单车入口：

```bash
source install/setup.bash
ros2 launch contrast_controller contrast_comparison.launch.py \
  controllers:=contrast planner_mode:=layer1,layer2,layer3 \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 return_home:=true
```

浏览器打开 `http://127.0.0.1:8766/`。每条部分路线执行完成后，任务脚本依据这辆车的实际里程计和已确认任务/道路进度请求下一条规划路线，依次推进任务点、隧道、道路补漏与返航；静态车体碰撞保护保持开启。

`controllers` 可取 `both`（默认双车对照）、`production` 或 `contrast`。单车模式只启动所选控制器、仿真车和 GUI，不等待另一辆车；真值检查点仍要求停车一秒，不模拟物理 RFID、播报或定位纠偏。

完整实跑已完成全部 12 个任务点、4 条隧道、24 条普通巡检道路与返航，记录见 [FULL_MISSION_REPORT.md](FULL_MISSION_REPORT.md)。对比入口默认关闭仿真障碍检测，GUI 手动添加障碍物不能直接作为自动避障验证。

## 避障闭环验证

使用专用入口开启 loopback 障碍检测，并由正式 `navigation_mission_manager` 独占路线规划、检查点授权和动态障碍重规划：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch contrast_controller obstacle_validation.launch.py \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 gui:=true
```

该入口保持 `enforce_static_collision=true`，物理参数仍来自 `vehicle_config`；手工障碍须放在直路中段，不能用 GUI 障碍直接替代实车视觉感知验证。状态时间线和限制见 [OBSTACLE_VALIDATION_REPORT.md](OBSTACLE_VALIDATION_REPORT.md)。

## 实车替换启动

实车测试使用独立入口，将 Contrast 可执行文件以生产节点名 `waypoint_navigator` 启动，因此总控继续使用 `/waypoint_navigator/*` 服务；串口桥仍将唯一速度话题 `/cmd_vel_nav` 转发到底盘：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch contrast_controller contrast_serial_bringup.launch.py \
  serial_device:=auto autostart:=false
```

入口保留 `require_actuator_health=true`、`use_sim_time=false`、`serial_command_timeout_s=0.4` 和 `serial_control_telemetry_timeout_s=0.90`。默认不自动开始路线，必须先确认融合定位、执行器健康、串口设备和急停状态。该入口只启动 Contrast 和串口桥，不启动规划器或任务总控；实车现有规划器和 `navigation_mission_manager` 按原生产流程单独启动。

回滚时停止该入口，再按原生产入口启动：

```bash
ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  serial_device:=auto autostart:=false
```

同一时刻只能运行一个导航控制器和一个串口速度桥，避免两个节点同时向 `/cmd_vel_nav` 发布。

## 使用现有实车启动脚本切换 Contrast

完整的传感器、里程计、录包、清理和锁管理仍由工作区已有的 `scripts/start_visual_navigation.sh` 负责。加上 `--contrast` 即可只替换导航控制器，其他启动流程保持原样：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
scripts/start_visual_navigation.sh --contrast --no-build
```

生产模式仍使用原命令；Contrast 模式会复用相同的 D455/ORB-SLAM3、轮速与融合里程计、串口安全门、录包和退出清理逻辑，并将导航启动入口切换为 `contrast_serial_bringup.launch.py`。默认仍为 `autostart=false`，需要任务管理器和规划器时继续按原脚本提示启动 `scripts/start_new_map_planner.sh`。


部署启动文件应以 `contrast_controller` 替换正式导航可执行文件，并按以下顺序向同一
节点注入参数：正式导航通用参数、本包转弯参数、`vehicle_config` 生成的物理参数。
这样对照动作与恢复控制的改动集中于 Contrast 包内，不影响其他控制器的现有参数。

## 出门与柔性纠偏实验

Contrast 出门首次转弯在 `base_link` 沿地图出口方向距起点 0.11 m 处停车，软件到位容差 1 mm；保留轮速/定位停稳确认，实际停距受定位与制动误差影响。距离是代码中的固定值，不受普通路口 `braking_safety_margin` 和 `turn_maneuver_entry_target_x` 调节。转弯入口同步采用该位置，避免停下后又前进到 13 cm。起点和出口路口从现有地图读取；后续路口与返航不使用出门特例。

仅本控制器启用柔性纠偏：偏差增大时平滑增强转向与 P 增益、降低线速度；停车前收紧角速度与线速度比例，RFID 扫描继续使用横向反馈。正式 `visual_navigation` 不变。

验收方式与结果见工作区《实验预算.md》。本分支尚未替换生产运行文件。
