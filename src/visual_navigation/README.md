# visual_navigation

该功能包使用通用融合里程计 `/odometry/fused` 和字符串健康状态 `/odometry/fusion_status`，按照航点依次导航，并通过 `/cmd_vel_nav` 输出速度命令。它不依赖具体的视觉、IMU、轮速或融合实现。

详细文档：

- `NAVIGATION_CODE_GUIDE.md`：导航代码结构、算法和参数说明。
- 工作空间根目录的 `README.md`：纯上位机测试、真实视觉联调和下位机对接操作手册。

系统不执行 A*、动态避障或局部路径搜索。航点文件中的路线必须提前确认安全。

## 接口

订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/odometry/fused` | `nav_msgs/msg/Odometry` | 融合后的当前位姿 |
| `/odometry/fusion_status` | `std_msgs/msg/String` | `FULL`、允许的 `DEGRADED_*` 或 `FAULT` |
| `/cup_car_serial/connected` | `std_msgs/msg/Bool` | 可选的执行器连接心跳，串口 bringup 强制启用 |
| `/tracking_state` | `std_msgs/msg/Int32` | 可选的旧 ORB 兼容检查，默认关闭 |
| `/waypoint_navigation/route_input` | `nav_msgs/msg/Path` | 动态替换当前路线 |

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

只有里程计和融合健康消息均未超时，且健康状态位于 `allowed_fusion_states` 中，节点才会输出非零速度。`DEGRADED_*` 状态按 `degraded_speed_scale` 同时限制线速度和角速度；`FAULT`、未知状态、消息陈旧、无效位姿或坐标系不一致都会停车。旧 `/tracking_state` 检查由 `require_tracking_state` 参数选择性启用。

普通 bringup 的 `require_actuator_health` 默认为 `false`，便于不连接下位机时测试；`visual_navigation_serial_bringup.launch.py` 强制设为 `true`。此时启动前必须收到新鲜的 `connected=true`，运行中断连或心跳超过 `actuator_health_timeout`（默认 `0.8 s`）会立即停车并锁止任务，重连后不会自动续走。

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

航点必须与 `/odometry/fused.header.frame_id` 使用同一坐标系；节点不做 TF 转换，坐标系不一致时会拒绝里程计并停车。

如果需要使用已有地图中的绝对航点，需要后续增加 `外部地图 -> ORB里程计坐标` 的对齐节点，不能仅修改 CSV 中的 `frame_id`。

## 构建

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select visual_navigation
source install/setup.bash
```

## 只启动航点导航

先由外部总启动拉起传感器和融合包，确认 `/odometry/fused` 与 `/odometry/fusion_status` 正常，然后执行：

```bash
ros2 launch visual_navigation waypoint_navigation.launch.py \
  route_file:=/absolute/path/to/route.csv
```

CSV 模式默认不会自动行驶。需要时可用现有服务启动：

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

停止或复位：

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_navigator/reset std_srvs/srv/Trigger '{}'
```

## 图形路线编辑器

导航节点运行后，另开终端启动编辑器：

```bash
source install/setup.bash
ros2 launch visual_navigation route_editor.launch.py
```

网格每格固定为 `0.6 m`，原点箭头为车头初始的 `+X` 方向。左键添加航点，按住航点拖动可调整箭头方向，右键删除最后一个点。顶部显示 `ODOM READY` 后，点击“发布并启用路线”会通过 `/waypoint_navigation/route_input` 一次发布全部航点。

导航节点收到合法动态路线后会立即发零速度、清零航点索引并进入 `IDLE`。编辑器等待 `/waypoint_path` 回显确认本次路线后，会自动调用启动服务；确认或启动失败会在界面显示，不需要手动执行服务命令。历史 transient-local 路线只会被加载，不会自行启动。

动态路线中的速度和到达容差使用 `default_speed` 和 `waypoint_tolerance` 参数。重启导航节点后仍会先加载 `route_file` 指定的 CSV；需要再次点击发布按钮才能重新启用编辑路线。

## 导航 Bringup

下面的 Launch 只启动导航节点，传感器和融合由外部总启动编排：

```bash
ros2 launch visual_navigation visual_navigation_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  autostart:=false
```

需要同时启动串口桥接时使用：

```bash
ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
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
