# final_navigation

`final_navigation` 是 dm2325 工作空间唯一安装的路线导航控制器，节点、功能包和可执行
文件统一使用此名称。控制实现链接 `contrast_navigation_core` 导出的
`contrast_waypoint_navigator_core`；本包使用独立的 `config/final_navigation.yaml`，不加载
其他工作空间的控制参数。

## 参数与控制

- 车辆尺寸、轮距、轴心偏移和传感器安装位姿由 `vehicle_config` 在标准启动中注入。
- 本包路线、速度、制动、转弯、纠偏和避障恢复参数均来自自身 YAML。普通直线路线使用
  独立的 `max_path_linear_speed=0.10 m/s`，输出经过 `max_linear_acceleration` 和
  `max_linear_deceleration` 限幅；转弯、读卡、倒车和恢复阶段仍遵循各自参数。
- 转弯阶段依次为 `CAPTURE`、`APPROACH_ENTRY`、`TURN_60`、`DIAGONAL_CREEP`、
  `TURN_30` 和 `COMPLETE`。任一阶段超时或无进展都会停车并请求新路线。
- 避障恢复沿当前行驶轨迹回撤到候选路口，停车后复用相同的转弯控制器。恢复运动继续
  经过车身扫掠安全检查；无法安全执行的动作会停车并进入故障保护。

## 构建和测试

在工作空间根目录执行：

```bash
colcon build --packages-up-to final_navigation
colcon test --packages-select final_navigation --event-handlers console_direct+
colcon test-result --verbose
```

转弯测试读取本包 YAML，并使用 `vehicle_config` 的物理标定和地图几何积分车辆运动。

## Loopback 仿真

安装在 `vehicle_loopback_sim` 中的 `final_navigation_comparison.launch.py` 是历史命名的
单车闭环入口：只启动 final_navigation、真值仿真器、规划器和任务节点。它没有旧控制器
选择项，两侧控制参数不会合并。默认 GUI 地址为 `http://127.0.0.1:8765/`：

```bash
source install/setup.bash
ros2 launch vehicle_loopback_sim final_navigation_comparison.launch.py gui:=false
```

运行完整三层任务：

```bash
ros2 launch vehicle_loopback_sim final_navigation_comparison.launch.py \
  planner_mode:=layer1,layer2,layer3 \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 return_home:=true
```

真值检查点要求路线 ACK 匹配、车体靠近目标并连续停车一秒后才授权路线继续；它不模拟
实物 RFID 读取、定位修正或语音播报。避障闭环使用专用入口：

```bash
ros2 launch vehicle_loopback_sim final_navigation_obstacle_validation.launch.py \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 gui:=true
```

## 实车启动

直接启动 final_navigation 和串口桥：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_navigation final_navigation_serial_bringup.launch.py \
  serial_device:=auto autostart:=false
```

该入口加载本包 YAML 和 `vehicle_config` 物理参数，并开启执行器健康检查。默认不会自动
开始路线；启动任务前应确认融合定位、串口设备和急停状态。工作区的
`scripts/start_final_navigation.sh` 负责传感器、里程计、录包、进程清理和锁管理：

```bash
scripts/start_final_navigation.sh --no-build
```

首个出口转弯的位置与阶段速度由本包的实现和 YAML 控制；后续普通路口、读卡、倒车和
恢复阶段保留各自独立的速度限制。实跑证据和历史仿真记录见本目录的验证报告。
