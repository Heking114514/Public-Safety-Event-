# D455 视觉定位与小车控制使用手册

本工作空间支持两种控制方式：

1. **航点自动控制**：D455 + ORB-SLAM3 提供定位，导航节点读取 CSV 航点并发布速度。
2. **手动速度控制**：不启动航点导航节点，使用键盘或命令行直接发布速度。

两种方式最终都使用标准 ROS 2 速度消息：

```text
话题：/cmd_vel_nav
类型：geometry_msgs/msg/Twist
线速度：linear.x，单位 m/s
角速度：angular.z，单位 rad/s
```

工作空间中的 `cup_car_serial` 包会订阅 `/cmd_vel_nav`，并将速度转换为 `vx,az\r\n` 后写入串口。

## 1. 安全要求

- 首次实车测试时将驱动轮架空，并准备急停。
- 从 `0.03 m/s`、`0.10 rad/s` 以下开始测试。
- 下位机必须实现通信看门狗，建议 `100～300 ms` 未收到命令时自动停车。
- `/cmd_vel_nav` 上只能保留一个有效速度发布者。
- ROS 约定：正线速度前进，正角速度左转，负角速度右转。

## 2. 终端环境

每个新终端都先执行：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查功能包：

```bash
ros2 pkg list | grep -E 'orbslam3|visual_navigation|cup_car_serial|realsense2_camera'
```

也可以在工作空间根目录直接一键检查、构建缺失组件并启动完整系统：

```bash
./scripts/start_visual_navigation.sh
```

默认不会自动开始行驶。常用选项：

```bash
./scripts/start_visual_navigation.sh --no-serial
./scripts/start_visual_navigation.sh --no-imu --visualization
./scripts/start_visual_navigation.sh --serial-device /dev/ttyACM0
./scripts/start_visual_navigation.sh --route /absolute/path/to/route.csv
```

查看全部选项：

```bash
./scripts/start_visual_navigation.sh --help
```

---

# 模式一：航点自动控制

数据链路：

```text
D455 图像和 IMU
  → ORB-SLAM3
  → /odom、/tracking_state
  → waypoint_navigator
  → /cmd_vel_nav
  → 下位机桥接节点
  → 小车底盘
```

## 3. 启动完整程序

### 3.1 终端 1：一键启动相机、定位、航点导航和串口

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  route_file:=/home/j/colcon_ws/src/visual_navigation/routes/example_route.csv \
  route_frame:=map \
  body_frame_id:=camera_link \
  cmd_vel_topic:=/cmd_vel_nav \
  serial_device:=/dev/ttyUSB0 \
  serial_baud_rate:=115200 \
  visualization:=true \
  use_imu:=true \
  autostart:=false
```

保持该终端运行，不要按 `Ctrl+C`。

`autostart:=false` 表示航点已经加载，但必须手动调用服务才开始导航。启动后应存在：

```text
/camera/camera
/orbslam3_stereo_inertial
/waypoint_navigator
/cmd_vel_serial_node
```

检查：

```bash
ros2 node list
```

如果 D455 序列号不同，先查询：

```bash
rs-enumerate-devices | grep 'Serial Number'
```

然后在 launch 命令中增加：

```text
serial_no:=_实际序列号
```

序列号前的 `_` 不能省略。

### 3.2 检查串口设备

连接下位机后执行：

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

根据实际设备修改启动参数，例如：

```text
serial_device:=/dev/ttyACM0
```

当前用户已经属于 `dialout` 组。启动日志中应出现：

```text
Forwarding /cmd_vel_nav to /dev/ttyUSB0 at 115200 baud
```

如果看到 `Cannot open serial port`，检查设备名、USB 连接和设备权限。串口不存在时节点会每秒重试，不会导致其他导航节点退出。

## 4. 启动前检查

### 4.1 相机数据

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

期望频率：

```text
左右红外图像：约 30 Hz
合并 IMU：约 200 Hz
```

### 4.2 ORB-SLAM3 跟踪

```bash
ros2 topic echo /tracking_state
```

有效状态：

```text
data: 2    # OK
data: 5    # OK_KLT
```

如果一直为 `-1`、`0` 或 `1`，保证场景有纹理，并缓慢平移、转动相机完成视觉惯性初始化。

### 4.3 里程计和 TF

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map camera_link
```

手动移动相机时，`/odom` 中的位置和姿态应合理变化。

### 4.4 航点和导航状态

```bash
ros2 topic echo /waypoint_path --once
ros2 topic echo /waypoint_navigation/current_waypoint
ros2 topic echo /waypoint_navigation/status
```

启动任务前状态应为：

```text
IDLE
```

默认示例路线：

```text
(0,0) → (1,0) → (1,1) → (0,1)
```

CSV 格式：

```csv
x,y,yaw,speed,tolerance,stop_time
```

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `x`、`y` | 航点位置 | m |
| `yaw` | 目标偏航角 | rad |
| `speed` | 最大线速度 | m/s |
| `tolerance` | 到达容差 | m |
| `stop_time` | 到达后停留时间 | s |

航点和 `/odom` 必须使用同一个坐标系，当前均为 `map`。

## 5. 启动航点任务并查看速度

### 5.1 终端 2：监控速度

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /cmd_vel_nav
```

未启动导航时应持续输出零速度。控制频率可用下面命令检查：

```bash
ros2 topic hz /cmd_vel_nav
```

期望约为 `30 Hz`。

### 5.2 终端 3：开始导航

确认 `/tracking_state` 为 `2` 或 `5`，且 `/odom` 正常后执行：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call \
  /waypoint_navigator/start \
  std_srvs/srv/Trigger '{}'
```

成功返回：

```text
success=True
message='Waypoint navigation started'
```

导航状态应进入 `FOLLOWING`：

```bash
ros2 topic echo /waypoint_navigation/status
```

查看当前航点：

```bash
ros2 topic echo /waypoint_navigation/current_waypoint
```

### 5.3 速度含义

```text
linear.x > 0：前进
angular.z > 0：左转
angular.z < 0：右转
```

方向误差较大时，控制器会令 `linear.x=0`，先原地转向；方向接近目标后才输出正线速度。

### 5.4 只有相机时模拟车辆

没有底盘时，可手动搬动相机测试完整控制逻辑：

- `angular.z > 0`：向左转动相机。
- `angular.z < 0`：向右转动相机。
- `linear.x > 0`：沿相机正前方缓慢平移。
- 到达航点附近后，`current_waypoint` 应递增。

同时观察：

```bash
ros2 topic echo /waypoint_navigation/status
ros2 topic echo /waypoint_navigation/current_waypoint
ros2 topic echo /cmd_vel_nav
```

## 6. 停止和复位自动导航

停止任务并保持当前航点：

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

停车并回到第 0 个航点：

```bash
ros2 service call /waypoint_navigator/reset std_srvs/srv/Trigger '{}'
```

停止后 `/cmd_vel_nav` 应持续输出零速度。

完全关闭程序：

1. 调用 `/waypoint_navigator/stop`。
2. 回到 launch 终端。
3. 按 `Ctrl+C`。

> 仅调用 `stop` 后，导航节点仍会持续发布零速度。要切换到人工控制，必须关闭完整 launch 或把导航输出重映射到其他话题。

## 7. 检查自动导航到串口的连接

联合 launch 已经启动 `/cmd_vel_serial_node`。执行：

```bash
ros2 topic info /cmd_vel_nav --verbose
```

正常应看到：

```text
Publisher count: 1
Subscription count: 1
```

- 发布者：`/waypoint_navigator`
- 订阅者：`/cmd_vel_serial_node`

串口发送格式为：

```text
linear.x,angular.z\r\n
```

例如速度 `0.25 m/s`、角速度 `-1.5 rad/s` 会发送：

```text
0.250,-1.500\r\n
```

串口节点以 `20 Hz` 发送；超过 `0.4 s` 没收到新的 `/cmd_vel_nav` 后会发送零速度。

---

# 模式二：手动速度控制

手动模式不读取航点，不启动 `/waypoint_navigator`。人工控制节点直接发布 `/cmd_vel_nav`。

```text
键盘或 ros2 topic pub
  → /cmd_vel_nav
  → 下位机桥接节点
  → 小车底盘
```

## 8. 从自动导航切换到手动模式

如果完整航点程序正在运行，先停止任务：

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

然后回到 launch 终端按 `Ctrl+C`，并确认：

```bash
ros2 node list
```

手动模式下不应存在：

```text
/waypoint_navigator
```

这是为了防止导航节点发布的零速度覆盖人工命令。

## 9. 可选：只启动相机和 ORB-SLAM3

如果手动开车时仍需要观察视觉定位，在终端 1 执行：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  serial_no:=_038122250473 \
  body_frame_id:=camera_link \
  map_frame_id:=map \
  visualization:=true \
  use_imu:=true
```

该 launch 只启动：

```text
/camera/camera
/orbslam3_stereo_inertial
```

它不会发布速度，也不会与人工控制冲突。

如果手动控制时不需要定位，可以不启动相机和 ORB-SLAM3，只启动串口节点和人工控制节点。

## 10. 启动下位机桥接节点

终端 1 启动串口节点：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav
```

根据实际设备将 `/dev/ttyUSB0` 改为 `/dev/ttyACM0` 或 `/dev/serial/by-id/...`。

该节点订阅：

```text
/cmd_vel_nav
geometry_msgs/msg/Twist
```

人工发布启动后检查：

```bash
ros2 topic info /cmd_vel_nav --verbose
```

如果显示：

```text
Subscription count: 0
```

说明串口节点没有启动或话题名称不一致，小车不会运动。

### 使用 Python 模拟导航节点测试串口

不要启动 `/waypoint_navigator`。终端 1 持续模拟导航速度：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/test_cmd_vel_nav.py \
  --linear 0.03 \
  --angular 0.10 \
  --rate 10 \
  --enable-output
```

终端 2 启动串口包：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=auto \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav
```

检查串口状态和下位机编码器回传：

```bash
ros2 topic echo /cup_car_serial/connected --once
ros2 topic echo /cup_car_serial/rx
```

按 `Ctrl+C` 结束 Python 发布器时，脚本会自动发送零速度。

## 11. 键盘手动控制

系统已经安装 `teleop_twist_keyboard`。

终端 2：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_nav
```

常用按键以程序显示为准：

| 按键 | 动作 |
| --- | --- |
| `i` | 前进 |
| `,` | 后退 |
| `j` | 左转 |
| `l` | 右转 |
| `u` / `o` | 前进并转弯 |
| `m` / `.` | 后退并转弯 |
| `k` | 停车 |
| `w` / `x` | 增大/减小线速度 |
| `e` / `c` | 增大/减小角速度 |

另开终端监控：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic info /cmd_vel_nav --verbose
```

手动模式正常应只有一个速度发布者，并至少有一个下位机订阅者。

## 12. 命令行发送常用速度

以下命令以 `10 Hz` 持续发送，按 `Ctrl+C` 结束。

### 12.1 低速前进

```bash
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.03}, angular: {z: 0.0}}"
```

### 12.2 低速后退

```bash
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: -0.03}, angular: {z: 0.0}}"
```

### 12.3 原地左转

```bash
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.10}}"
```

### 12.4 原地右转

```bash
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.10}}"
```

### 12.5 前进并左转

```bash
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.03}, angular: {z: 0.10}}"
```

### 12.6 前进并右转

```bash
ros2 topic pub -r 10 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.03}, angular: {z: -0.10}}"
```

### 12.7 停车

先按 `Ctrl+C` 停止持续发送，再执行：

```bash
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

真实下位机仍必须有通信超时自动停车功能。

## 13. 下位机使用 `/cmd_vel` 时

如果下位机订阅 `/cmd_vel`，键盘直接执行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

命令行示例：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.03}, angular: {z: 0.0}}"
```

停车：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

上位机发布话题和下位机订阅话题必须完全一致。

## 14. 防止控制源冲突

```bash
ros2 topic info /cmd_vel_nav --verbose
```

手动控制时理想状态：

```text
Publisher count: 1
```

如果发布者大于 1：

1. 立即发送零速度。
2. 关闭人工发布。
3. 停止航点任务。
4. 关闭完整航点 launch。
5. 确认 `/waypoint_navigator` 消失后再开始手动控制。

多个发布者会让下位机收到交错的零速度和非零速度，可能导致抖动或失控。

---

## 15. 测试通过标准

### 15.1 航点自动控制

- [ ] 左右红外图像约 30 Hz，IMU 约 200 Hz。
- [ ] `/tracking_state` 为 `2` 或 `5`。
- [ ] `/odom` 持续发布并随相机运动变化。
- [ ] `/waypoint_path` 正确显示 CSV 航点。
- [ ] 调用 `start` 后状态进入 `FOLLOWING`。
- [ ] `/cmd_vel_nav` 输出合理的 `linear.x` 和 `angular.z`。
- [ ] 到达航点后当前航点编号递增。
- [ ] 调用 `stop` 或定位丢失后速度归零。
- [ ] 下位机桥接节点能够收到 `/cmd_vel_nav`。

### 15.2 手动速度控制

- [ ] `/waypoint_navigator` 没有运行。
- [ ] `/cmd_vel_nav` 只有一个人工控制发布者。
- [ ] 下位机桥接节点订阅 `/cmd_vel_nav`。
- [ ] 正负线速度对应前进和后退。
- [ ] 正负角速度对应左转和右转。
- [ ] 零速度能够停车。
- [ ] 停止发布后下位机看门狗能够自动停车。

## 16. 常见问题

### 找不到功能包

```bash
source /opt/ros/humble/setup.bash
source /home/j/colcon_ws/install/setup.bash
```

### 一直没有 `/odom`

```bash
ros2 topic echo /tracking_state
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

保证场景有纹理，并缓慢移动和转动相机。

### 自动导航启动成功但速度为零

```bash
ros2 topic echo /waypoint_navigation/status --once
ros2 topic echo /tracking_state --once
ros2 topic echo /odom --once
```

只有跟踪状态为 `2` 或 `5` 且 `/odom` 未超时，导航节点才会输出非零速度。

### 手动发布后小车不动

```bash
ros2 topic info /cmd_vel_nav --verbose
```

如果 `Subscription count: 0`，说明没有下位机桥接节点接收命令。还要确认下位机使用的是 `/cmd_vel_nav` 还是 `/cmd_vel`。

### 手动控制时小车抖动

```bash
ros2 topic info /cmd_vel_nav --verbose
```

如果 `Publisher count` 大于 1，关闭完整航点 launch，只保留一个人工控制源。

## 17. 最短启动命令

### 航点自动控制

```bash
ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  route_file:=/home/j/colcon_ws/src/visual_navigation/routes/example_route.csv \
  route_frame:=map \
  body_frame_id:=camera_link \
  cmd_vel_topic:=/cmd_vel_nav \
  serial_device:=/dev/ttyUSB0 \
  serial_baud_rate:=115200 \
  visualization:=true \
  use_imu:=true \
  autostart:=false

ros2 topic echo /cmd_vel_nav
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

### 手动速度控制

```bash
# 可选：只启动相机和 ORB-SLAM3
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  serial_no:=_038122250473 \
  body_frame_id:=camera_link \
  map_frame_id:=map \
  visualization:=true \
  use_imu:=true

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_nav

# 停车
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```
