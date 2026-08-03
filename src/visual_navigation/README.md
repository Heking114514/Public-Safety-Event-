# visual_navigation

该功能包使用现有 ORB-SLAM3 ROS 2 节点发布的 `/odom` 和 `/tracking_state`，按照 CSV 航点文件依次导航，并通过 `/cmd_vel_nav` 输出给后续下位机通信节点。

详细文档：

- `NAVIGATION_CODE_GUIDE.md`：导航代码结构、算法和参数说明。
- 工作空间根目录的 `README.md`：纯上位机测试、真实视觉联调和下位机对接操作手册。

系统不执行 A*、动态避障或局部路径搜索。航点文件中的路线必须提前确认安全。

## 接口

订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 当前位姿 |
| `/tracking_state` | `std_msgs/msg/Int32` | ORB-SLAM3 跟踪状态 |

发布：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 导航速度命令，下位机桥接输入 |
| `/waypoint_path` | `nav_msgs/msg/Path` | CSV 航点路线 |
| `/waypoint_navigation/status` | `std_msgs/msg/String` | 导航状态 |
| `/waypoint_navigation/current_waypoint` | `std_msgs/msg/Int32` | 当前目标航点索引 |

服务：

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/waypoint_navigator/start` | `std_srvs/srv/Trigger` | 从当前索引开始导航 |
| `/waypoint_navigator/stop` | `std_srvs/srv/Trigger` | 停车并保持当前索引 |
| `/waypoint_navigator/reset` | `std_srvs/srv/Trigger` | 停车并回到第 0 个航点 |

只有跟踪状态为 `OK`（2）或 `OK_KLT`（5），且 `/odom` 没有超时时，节点才会输出非零速度。运行过程中丢失定位时默认中止任务并持续输出零速度。

## 航点文件

CSV 列顺序：

```text
x, y, yaw, speed, tolerance, stop_time
```

- `x`、`y`：目标位置，单位为米，必须填写。
- `yaw`：目标朝向，单位为弧度；中间航点可以留空，最终航点留空时不执行终点朝向控制。
- `speed`：该航点对应路段的最大线速度，单位为米每秒；留空使用参数默认值。
- `tolerance`：航点到达半径，单位为米；留空使用参数默认值。
- `stop_time`：到达航点后的停车时间，单位为秒；留空或 `0` 表示不停留。

示例：

```csv
x,y,yaw,speed,tolerance,stop_time
0.0,0.0,0.0,0.10,0.12,0.0
1.0,0.0,0.0,0.20,0.12,0.0
1.0,1.0,1.570796,0.15,0.12,1.0
```

航点必须与 `/odom.header.frame_id` 使用同一坐标系。当前 ORB-SLAM3 包默认把第一次有效位姿设置为 `map` 原点，因此第一版应从固定位置和固定方向启动，并按该启动坐标记录航点。

如果需要使用已有地图中的绝对航点，需要后续增加 `外部地图 -> ORB里程计坐标` 的对齐节点，不能仅修改 CSV 中的 `frame_id`。

## 构建

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select visual_navigation
source install/setup.bash
```

如果 `orbslam3` 尚未构建，可以一起构建：

```bash
colcon build --symlink-install --packages-select orbslam3 visual_navigation \
  --cmake-args \
  -DSophus_DIR=$PWD/src/deps/share/sophus/cmake \
  -DPangolin_DIR=$PWD/src/deps/lib/cmake/Pangolin
```

## 只启动航点导航

先单独启动 ORB-SLAM3，确认 `/odom` 和 `/tracking_state` 正常，然后执行：

```bash
ros2 launch visual_navigation waypoint_navigation.launch.py \
  route_file:=/absolute/path/to/route.csv
```

默认不会自动行驶。确认车辆架空或场地安全后启动任务：

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

停止或复位：

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_navigator/reset std_srvs/srv/Trigger '{}'
```

## 启动完整视觉导航

下面的 Launch 同时启动 D455、ORB-SLAM3 和航点导航节点：

```bash
ros2 launch visual_navigation visual_navigation_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  body_frame_id:=camera_link \
  autostart:=false
```

需要同时启动串口桥接时使用：

```bash
ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  body_frame_id:=camera_link \
  serial_device:=/dev/ttyUSB0 \
  serial_baud_rate:=115200 \
  autostart:=false
```

安装到小车后，建议准确发布 `base_link` 与相机的静态 TF，并将 `body_frame_id` 设置为 `base_link`。如果不存在该静态变换，ORB-SLAM3 将无法输出车体中心位姿。

## 下位机连接

`/cmd_vel_nav` 是导航系统的最终 ROS 输出接口：

```text
linear.x  -> 目标线速度，m/s
angular.z -> 目标角速度，rad/s
```

`cup_car_serial` 节点订阅 `/cmd_vel_nav`，并按照 `vx,az\r\n` 格式向串口发送。默认设备为 `/dev/ttyUSB0`，波特率为 115200，发送频率为 20 Hz，命令超时为 0.4 秒。下位机仍必须实现独立通信看门狗。

## 安全说明

- 示例路线仅用于格式演示，禁止未确认场地时直接让实车执行。
- 当前系统不检测路线上的人员、车辆或临时障碍物。
- `autostart` 默认关闭。
- 调试初期应架空驱动轮或把最大速度调到很低。
- ORB-SLAM3 严重跟踪丢失后可能重新建立坐标原点，此时必须重新对齐航点坐标再启动任务。
