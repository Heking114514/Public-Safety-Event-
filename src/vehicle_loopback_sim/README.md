# vehicle_loopback_sim

本包提供导航闭环真值仿真，供地图规划器和 `final_navigation` 在不连接实车底盘、轮速计、
ORB-SLAM 或融合定位门控时使用。车辆运动、传感器和障碍仿真为 C++17 节点；任务辅助、
启动文件和浏览器界面使用 Python。

仿真节点订阅速度命令，并发布 `map -> base_link` 真值里程计、控制角速度、融合健康
状态、理想轮速反馈和前向障碍检测。车辆尺寸、轮距和轴心偏移从 `vehicle_config` 读取，
仿真限速与加速度参数位于 `config/vehicle_loopback.yaml`。

## 启动闭环

只运行仿真器：

```bash
ros2 launch vehicle_loopback_sim vehicle_loopback.launch.py
```

运行规划器、final_navigation、任务总控和仿真器：

```bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py
```

也可以运行工作空间包装脚本；脚本会先清理之前启动的仿真、规划器、导航器和任务管理器：

```bash
scripts/start_loopback_sim.sh simulate_obstacle_recognition:=true
```

闭环验收固定使用 `simulation_mode:=closed_loop` 和 `playback_speed:=1.0`。加速只用于明确
标记为非验收的离线冒烟检查：

```bash
ros2 launch vehicle_loopback_sim vehicle_loopback.launch.py \
  simulation_mode:=offline playback_speed:=3.0 gui:=false
```

`planner_control_loopback.launch.py` 固定启动正式 `final_navigation`；可用
`plan_on_start:=false` 只检查空闲话题，也可传入逗号分隔的 `target_ids`。车辆碰撞保护
默认开启；需要单独验证质点控制器时，使用 `ideal_controler` 包提供的专用入口。

浏览器界面默认位于 `http://127.0.0.1:8765/`，显示地图、规划路线、导航路线、车辆外形、
实际轨迹、速度命令和导航状态。可用 `gui:=false` 关闭界面，或用 `gui_port:=8766` 更改
端口。发布 `/initialpose` 可在运行中重置仿真位姿。

## 单车验证入口

`final_navigation_comparison.launch.py` 保留历史文件名，目前只启动一个
`final_navigation` 节点、对应的真值仿真器、地图规划器和路线任务辅助节点；启动参数不
再选择控制器。规划器与控制器读取各自功能包的 YAML，车辆物理参数由 `vehicle_config`
注入。默认关闭障碍检测，可用 `obstacle_detection:=true` 打开 GUI 临时障碍的模拟检测。

```bash
ros2 launch vehicle_loopback_sim final_navigation_comparison.launch.py gui:=false
```

真值检查点只在路线确认、接近目标且连续停车一秒后授权路线继续，不模拟实物 RFID、定位
修正或语音播报。需要由正式任务总控处理障碍重规划时，使用专用入口：

```bash
ros2 launch vehicle_loopback_sim final_navigation_obstacle_validation.launch.py \
  target_ids:=1,2,3,4,5,6,7,8,9,10,11,12 gui:=true
```

GUI 中临时放置的障碍不会改写静态地图，只发布模拟前向检测和动态障碍消息。静态地图碰撞
保护保持开启；该功能用于仿真接口验证，不能替代实车视觉感知和制动验收。详细验证记录见
`final_navigation` 包的报告文件。
