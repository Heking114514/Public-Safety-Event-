# final_navigation

`final_navigation` 是最终 ROS 2 导航控制包。可执行文件链接
`contrast_navigation_core` 导出的 `contrast_waypoint_navigator_core`，保留正式导航的
话题、服务、路线处理、定位健康检查和停车协议；Contrast 动作与恢复逻辑集中于本包；生产启动只保留这一套控制器。

## 运行边界

- 可执行文件名和运行节点名均为 `final_navigation`。
- 参数文件使用 `final_navigation` 节点键。
- 车辆尺寸、轮距、轴心偏移和传感器安装位姿仍由 `vehicle_config` 的标准启动注入，
  本包不复制物理标定值。
- 转弯控制参数全部来自 `config/final_navigation.yaml`；统一线速度低于兼容入口或
  蠕行速度时配置校验失败，因此未加载本包 YAML 不会静默使用共享头中的旧低速值。
- 转弯阶段保持为 `CAPTURE`、`APPROACH_ENTRY`、`TURN_60`、
  `DIAGONAL_CREEP`、`TURN_30` 和 `COMPLETE`。

## 控制策略

入口调整在动作开始时固定前进或倒退方向，只按入弯道路航向上的纵向投影判断位置；剩余距离进入 YAML 的 `turn_maneuver_entry_tolerance` 或跨过目标后进入第一段转弯，不反复换向。横向偏差不参与到点验收，入口角速度只根据实时航向误差计算，不尝试纠正横向位置。两段转弯保持固定线速度，角速度由目标航向反馈计算并限幅；中间直行段使用 YAML 中独立的 `25 mm` 正投影距离，距离达到后继续等待航向到达。最终控制器报告转到目标航向即完成，不增加出弯横向验收。

转弯恢复中的直线入位、返回入口和停车后位置复核同样只使用纵向位置容差，保留航向反馈与独立航向到达判定。道路是否合法、车身是否碰撞以及制动扫掠仍由道路几何安全检查决定，不作为横向到点容差使用。

历史横向入口容差、出口容差和入口横向增益参数仅兼容读取，不参与上述到点判定或入口纠偏。

每个运动阶段都有独立阶段超时和无进展检查。转向阶段只认可朝目标减小的航向误差，平移不能掩盖停转；超时或无实际进展时输出零命令并返回 `NEED_REPLAN`，共享导航器要求新的 route id 后才能恢复。任务 hold 和定位暂停保留动作上下文；暂停时输出零速，恢复后的阶段计时不计入暂停时间。

## 本轮道路避障恢复

直路障碍先按真实道路航向倒退到车后最近路口的合法入口，确认停车，再执行固定 `CAPTURE → APPROACH_ENTRY → TURN_60 → DIAGONAL_CREEP → TURN_30` 动作。T/X 的左右出口使用一次固定转弯，反向出口借用旁支执行两次固定转弯。双转之间按已确认的设计执行：旁支前进、停车、短倒车入位、停车、第二次转弯。每次换向都需要开始停车之后的新遥测、连续停止样本和保持时间。

若直接倒车恢复不成立，允许先朝障碍方向前移回中，前移量按道路和障碍位置确定，取消固定 8 cm 上限。`recovery_forward_obstacle_clearance_m: 0.03` 表示车身外廓到障碍物的最小净距；考虑偏航角点、控制周期及制动距离，不将相机或车中心距离当作车身净距。选择足以恢复的最短前移量，不要求每次走到 3 cm。候选发布前预检前移、停车、倒退及转弯整条链，执行时也检查 3 cm 净距和制动扫掠。固定入口自身越界时仍报告 `FAULT_POSE_RECOVERY_FIXED_ENTRY_UNSAFE`。

恢复采用 `recovery_turn_*` 独立配置，正常行驶参数保持隔离基线。旧 `recovery_max_radius_m` 仅保留参数兼容性，长倒车终点由道路拓扑决定；候选在发布前及被选中时均预检完整动作。预检和执行都使用真实输出限幅、底盘死区和中央车辆物理参数，逐周期检查车身轨迹及制动尾迹。

正常转弯中出现异常障碍时保存原转弯上下文，安全完成原动作、停车、执行固定倒转返回原车道、停车，再按后方路口 T/X 模板恢复。原动作或返回动作不安全会明确停车失败。

完成恢复前检查真实道路、出口侧位置、横向偏差、目标航向和车身安全。新障碍使当前事务失效并停车，不继续使用旧的安全计划。恢复动作不提交 RFID、隧道或道路覆盖进度。

本轮验证结果见 [RECOVERY_REVIEW.md](RECOVERY_REVIEW.md)。此前报告中“六场全通过”的历史记录不作为当前分支的验收结果。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/vehicle_loopback_sim/scripts/run_pose_recovery_scenarios.py --list
python3 src/vehicle_loopback_sim/scripts/run_pose_recovery_scenarios.py \
  --scenario south_nominal --output /tmp/contrast_recovery_new
```

运行器记录真实内部转弯阶段、独立道路投影、全程实测换向停车、候选编号、恢复完成和恢复后连续前进；总控路线 ACK 需在任务管理器日志中单独核对。方向场景必须实际选中对应模板；没有选中时记录覆盖不足。碰撞保护介入、故障或缺少证据均不能通过。

## 构建与测试

在工作空间根目录执行：

```bash
colcon build --packages-up-to final_navigation
colcon test --packages-select contrast_navigation_core final_navigation --event-handlers console_direct+
colcon test-result --verbose
```

单元测试直接积分控制器发布的线速度和角速度，不向状态机人工写入中间姿态。名义
用例检查完整阶段顺序，停滞用例保持里程计不动并检查 `NEED_REPLAN`。

## 单车仿真

`vehicle_loopback_sim` 提供 final_navigation 单车仿真入口：

```bash
ros2 launch vehicle_loopback_sim final_navigation_comparison.launch.py gui:=false
```

该入口使用 final_navigation 控制器、独立仿真车和规划器；无图形环境可使用 `gui:=false`。

## 完整三层任务

完整任务使用 final_navigation 单车入口：

```bash
source install/setup.bash
ros2 launch vehicle_loopback_sim final_navigation_comparison.launch.py \
  planner_mode:=layer1,layer2,layer3 \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 return_home:=true
```

浏览器打开 `http://127.0.0.1:8765/`。每条部分路线执行完成后，任务脚本依据这辆车的实际里程计和已确认任务/道路进度请求下一条规划路线，依次推进任务点、隧道、道路补漏与返航；静态车体碰撞保护保持开启。

真值检查点仍要求停车一秒，不模拟物理 RFID、播报或定位纠偏。

完整实跑已完成全部 12 个任务点、4 条隧道、24 条普通巡检道路与返航，记录见 [FULL_MISSION_REPORT.md](FULL_MISSION_REPORT.md)。对比入口默认关闭仿真障碍检测，GUI 手动添加障碍物不能直接作为自动避障验证。

## 避障闭环验证

Loopback 入口额外加载 `config/final_navigation_simulation_turn.yaml`：正常转弯入口沿用固定动作的路口前 70 mm，角速度为 0.40 rad/s，中间直行 5 mm，入口纵向容差为 3 mm；到入口后先保持直行入位并停车确认。它用于让车辆通过正常路口后继续测试避障；实车串口入口同样沿用 70 mm 固定入口。车辆尺寸和地图碰撞检查保持原配置。

使用专用入口开启 loopback 障碍检测，并由 `navigation_mission_manager` 独占路线规划、检查点授权和动态障碍重规划：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_loopback_sim final_navigation_obstacle_validation.launch.py \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 gui:=true
```

该入口保持 `enforce_static_collision=true`，物理参数仍来自 `vehicle_config`；手工障碍须放在直路中段，不能用 GUI 障碍直接替代实车视觉感知验证。状态时间线和限制见 [OBSTACLE_VALIDATION_REPORT.md](OBSTACLE_VALIDATION_REPORT.md)。

当前严格逐项核对及未通过分支见 [RECOVERY_REVIEW.md](RECOVERY_REVIEW.md)。顶部 L 的下游无解、边界 T 双转越界和实车原正常转弯碰撞测试失败仍保留，不能按仿真局部通过判为全部验收完成。

## 实车替换启动

实车测试使用 final_navigation 可执行文件，节点名和总控服务统一使用 `/final_navigation/*`；串口桥仍将唯一速度话题 `/cmd_vel_nav` 转发到底盘：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_navigation final_navigation_serial_bringup.launch.py \
  serial_device:=auto autostart:=false
```

入口保留 `require_actuator_health=true`、`use_sim_time=false`、`serial_command_timeout_s=0.4` 和 `serial_control_telemetry_timeout_s=0.90`。默认不自动开始路线，必须先确认融合定位、执行器健康、串口设备和急停状态。该入口只启动 final_navigation 和串口桥，不启动规划器或任务总控；实车现有规划器和 `navigation_mission_manager` 按原流程单独启动。

同一时刻只能运行一个导航控制器和一个串口速度桥，避免两个节点同时向 `/cmd_vel_nav` 发布。

## 使用现有实车启动脚本

完整的传感器、里程计、录包、清理和锁管理由工作区的 `scripts/start_final_navigation.sh` 负责：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
scripts/start_final_navigation.sh --no-build
```

该脚本使用 D455/ORB-SLAM3、轮速与融合里程计、串口安全门、录包和退出清理流程，并启动 `final_navigation_serial_bringup.launch.py`。默认仍为 `autostart=false`，需要任务管理器和规划器时继续按原脚本提示启动 `scripts/start_new_map_planner.sh`。


部署启动文件统一使用 `final_navigation` 可执行文件，并向节点注入本包控制参数和
`vehicle_config` 生成的物理参数。本工作空间使用自己的控制参数文件，不读取 dm2325
的导航配置。
