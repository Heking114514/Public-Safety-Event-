# 回环仿真器

无需实车即可运行规划器、控制器和真值车辆。物理积分、传感器与碰撞检查使用 C++，浏览器 GUI 和测试工具使用 Python。

## Contrast 避障仿真与时间倍率

在已构建并 source 的工作区启动：

~~~bash
ros2 launch contrast_controller obstacle_validation.launch.py simulation_time_scale:=3.0 gui:=true
~~~

浏览器打开 http://127.0.0.1:8766/，在“仿真时间”中拖动倍率滑块，运行中可在 0.1x～10.0x 调整，无需重启或重置车辆。页面显示时钟节点确认的实际倍率；推荐先用 3x，高倍率所需计算量也会增加。

倍率改变统一 /clock 的推进速度，车辆线速度、角速度、加速度、固定转弯参数均不变。车辆积分、控制周期、入口停车、RFID 检查点驻停和避障超时使用相同仿真时间。例如 3x 时，仿真中的 1 秒约对应现实中的 1/3 秒。GUI 刷新保持墙钟 12 Hz，避免高倍率重复渲染。修改倍率不会重置仿真时间或现有动作。

每个 ROS domain 只启动一个仿真时钟源。当前统一时间模式用于 obstacle_validation.launch.py 和 contrast_comparison.launch.py controllers:=contrast；旧生产/双车对照入口仍保持原时间机制，GUI 不启用倍率调节。旧 playback_speed 仅用于离线积分回放，闭环仍固定为 1.0，不能用它替代统一仿真时间。

HTTP 接口 POST /simulation_time_scale 接受 JSON {"value":3.0}，收到统一时钟回执才返回成功；非法值返回 400，未收到回执返回 503。ROS 控制话题是 /simulation/time_scale（std_msgs/msg/Float64），实际倍率回执是 /simulation/time_scale_state（可靠、持久化，GUI 刷新后仍可读）。

## 界面与障碍

GUI 展示地图、规划路线、导航器当前路线、真实车迹、命令速度、轮速反馈和任务状态。点击 Obstacle 再点击道路可放置临时障碍，Clear 清除临时障碍。障碍吸附到最近道路中心线，默认横跨道路、纵向深度 0.10 m。静态地图障碍只参与已知地图与碰撞判断，不伪装成视觉新发现障碍。临时障碍在相机前方有效视野内、距离不超过 0.25 m 时触发检测。

Contrast 避障入口启动正式总控、规划器、Contrast 控制器和真值车辆，按三层任务执行。仿真使用路线确认方式；这不替代真实 RFID 定位和语音播报验收。可用 auto_start:=false 保持空闲，或者用 target_ids:=2 缩小测试任务。

## 其他入口

仅启动真值车辆与 GUI：

~~~bash
ros2 launch vehicle_loopback_sim vehicle_loopback.launch.py
~~~

旧生产控制器的完整回环，GUI 默认端口为 8765：

~~~bash
ros2 launch vehicle_loopback_sim planner_control_loopback.launch.py simulate_obstacle_recognition:=true
~~~

该入口默认运行正式 visual_navigation/waypoint_navigator 和任务总控；plan_on_start:=false 可关闭任务自动启动。需要排查仿真器时可用 use_waypoint_navigator:=false use_debug_controller:=true 启动简化调试控制器。保留的 loopback_mission_node 只用于兼容测试，不由完整回环默认启动。

旧控制器双车对照：

~~~bash
ros2 launch vehicle_loopback_sim controller_comparison.launch.py
~~~

新、旧页面分别在 8765、8766，命令与里程计使用独立命名空间，两车共享规划路线。两车均到达终点才请求下一阶段。

## 配置与复位

速度与加速度上限在 config/vehicle_loopback.yaml；加速度上限为 0 表示禁用该限幅。车体尺寸和轴心偏移来自 vehicle_config/config/vehicle_physical.yaml。静态碰撞检查拒绝越界积分，不允许车体穿墙。

向相应车辆的 initialpose 话题发布位姿可复位真值位置；Contrast 使用 /comparison/contrast/initialpose。默认起点沿车头反方向偏移 20 mm，Contrast 单车启动可用 initial_x:=0.0 覆盖。
