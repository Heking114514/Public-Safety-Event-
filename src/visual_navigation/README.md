# visual_navigation

该功能包使用通用融合里程计 `/odometry/fused` 和字符串健康状态 `/odometry/fusion_status`，按照收到的路线依次导航，并通过 `/cmd_vel_nav` 输出速度命令。它不依赖具体的视觉、IMU、轮速或融合实现。

详细文档：

- `NAVIGATION_CODE_GUIDE.md`：导航代码结构、算法和参数说明。
- 工作空间根目录的 `README.md`：纯上位机测试、真实视觉联调和下位机对接操作手册。

`visual_navigation` 是路线执行器，不是路径规划器。系统本身不执行 A*、动态避障或局部路径搜索；路线由以下任一入口提供：

1. **手动路点 / CSV 模式**：预先编辑 `route_file` 指向的 CSV，导航节点加载后按固定航点跟随；也可以启动 `route_editor.py` 在图形网格上手动标点并直接发布动态路线。编辑器发布 `nav_msgs/msg/Path`，不会自动保存 CSV。
2. **自动规划模式**：由 `arena_path_planner` 根据地图、当前位姿、目标点和障碍物生成 `nav_msgs/msg/Path`；通过安全检查且能带来新进展的完整或阶段路线发布到 `/waypoint_navigation/route_input`，导航节点收到后自动激活执行。

自动规划模式的启动入口和目标设置见工作空间根目录 `README.md` 以及
`src/arena_path_planner/README.md`。

## 接口

订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/odometry/fused` | `nav_msgs/msg/Odometry` | 融合后的当前位姿 |
| `/odometry/fusion_status` | `std_msgs/msg/String` | `WAITING_FOR_INITIALIZATION`、`FULL`、允许的 `DEGRADED_*` 或 `FAULT_*` |
| `/imu/control` | `sensor_msgs/msg/Imu` | fusion gate 校正后的 `base_link` 平面角速度，用于转弯阻尼与到点判断 |
| `/cup_car_serial/actuator_healthy` | `std_msgs/msg/Bool` | 控制遥测健康状态，串口 bringup 强制启用；异常时停车等待并在恢复后续跑 |
| `/tracking_state` | `std_msgs/msg/Int32` | 可选的旧 ORB 兼容检查，默认关闭 |
| `/waypoint_navigation/route_input` | `nav_msgs/msg/Path` | 动态替换当前路线 |
| `/waypoint_navigation/route_ack` | `std_msgs/msg/UInt64` | 导航器接受的路线 ID（Path 时间戳纳秒） |

Odometry and IMU messages are accepted only when their ROS timestamps are
positive, monotonic, within the configured age limit, and no farther in the
future than `stamp_future_tolerance`.

发布：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 导航速度命令，下位机桥接输入 |
| `/waypoint_path` | `nav_msgs/msg/Path` | 当前执行路线（CSV、手动标点或自动规划） |
| `/waypoint_navigation/status` | `std_msgs/msg/String` | 导航状态 |
| `/waypoint_navigation/current_waypoint` | `std_msgs/msg/Int32` | 当前目标航点索引 |
| `/waypoint_navigation/motion_hold_state` | `mission_control_interfaces/msg/MotionHoldState` | 驻停状态与所有驻停来源 |

服务：

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/waypoint_navigator/start` | `std_srvs/srv/Trigger` | 从当前索引开始导航 |
| `/waypoint_navigator/stop` | `std_srvs/srv/Trigger` | 停车并保持当前索引 |
| `/waypoint_navigator/reset` | `std_srvs/srv/Trigger` | 停车并回到第 0 个航点 |
| `/waypoint_navigator/set_motion_hold` | `mission_control_interfaces/srv/SetMotionHold` | 按来源申请或释放任务驻停 |

任务驻停只冻结路线控制并持续输出零速度，不关闭 ORB-SLAM3、相机、IMU、轮式里程计或融合定位。多个来源同时申请时，必须全部释放后才会恢复运动。

```bash
ros2 service call /waypoint_navigator/set_motion_hold \
  mission_control_interfaces/srv/SetMotionHold \
  "{source: 'inspection', hold: true, reason: 'recognizing clue'}"

ros2 service call /waypoint_navigator/set_motion_hold \
  mission_control_interfaces/srv/SetMotionHold \
  "{source: 'inspection', hold: false, reason: ''}"
```

正常情况下，只有里程计和融合健康消息均未超时、且健康状态位于 `allowed_fusion_states` 中，节点才会输出非零速度。ORB 暖机时融合状态为 `WAITING_FOR_INITIALIZATION`：路线可以保持激活但只输出零速等待；连续有效视觉样本到来后会自动转入允许状态，无需再次 `start`。超过融合端 `initialization_timeout_s` 才会变为 `FAULT_INIT_TIMEOUT` 并锁存。最近一次定位有效后发生的短暂丢失，可在 `transient_localization_grace` 内按 `transient_fault_speed_scale` 降速维持；超过该窗口仍会停车。控制 IMU 超时则将角速度反馈置零并按 `no_imu_speed_scale` 限速；不同 `DEGRADED_*` 状态使用各自的 `*_speed_scale`，未单独配置的状态使用 `degraded_speed_scale`；`FAULT`、未知状态、消息陈旧、无效位姿或坐标系不一致都会停车。旧 `/tracking_state` 检查由 `require_tracking_state` 参数选择性启用。导航不得直接订阅原始 `/imu/filtered`，以免绕过融合 gate 的 bias 修正。

普通 bringup 的 `require_actuator_health` 默认为 `false`，便于不连接下位机时测试；`visual_navigation_serial_bringup.launch.py` 强制设为 `true`。此时启动前必须收到新鲜的 `actuator_healthy=true`，即下位机控制遥测新鲜、处于导航模式、未急停且命令没有超时。运行中状态变为 `false` 或心跳超过 `actuator_health_timeout`（默认 `0.8 s`）会立即发布零速度并进入 `WAITING_FOR_ACTUATOR_RECOVERY`，但保留当前路线和进度；健康状态恢复后自动续跑。

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

航点必须与 `/odometry/fused.header.frame_id` 使用同一坐标系，且默认要求
`/odometry/fused.child_frame_id` 为 `base_link`（可通过 `odom_child_frame` 配置）。
节点不做 TF 转换；任一 frame 为空或不一致时会丢弃该样本并保留上一条有效里程计。
如果合法样本持续缺失并超过 `odom_timeout`，车辆才会停车等待；收到下一条合法样本后
自动恢复。

最终航点首次进入位置容差后会锁定终点对角阶段。此后原地旋转造成的
`base_link` 位置摆动不会让导航重新追踪最后一段路径方向；达到最终
`yaw` 容差后直接发布零速度并进入 `GOAL_REACHED`。

路径控制连续 `10 s` 未取得 `0.05 m` 沿线进展，且车辆相对监督起点
确实位移过 `0.25 m` 时，先停车一拍并重置路径反馈；另外，只有在直线前进阶段
持续发出至少 `0.08 m/s` 的线速度命令、而实测速度和位移都接近零超过 `3 s` 时，
才按同样的恢复次数处理完全卡死。原地转向、制动、任务驻停和短时定位恢复不触发这条
低运动检查。连续两次恢复仍无效才进入 `FAULT_NO_PATH_PROGRESS`。转向使用独立的航向误差
进度监督，持续改善的大角度慢转不会因固定短超时被中止；连续无改善时同样先恢复两次，再进入
`FAULT_NO_TURN_PROGRESS`。
制动、任务驻停、航点等待和短时定位恢复期间不累计进度超时。

如果需要使用已有地图中的绝对航点，需要后续增加 `外部地图 -> ORB里程计坐标` 的对齐节点，不能仅修改 CSV 中的 `frame_id`。

## 构建

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to visual_navigation
source install/setup.bash
```

## 手动路点 / CSV 模式

### 使用统一脚本自动执行 CSV

实车固定路线统一读取工作空间根目录的 `scripts/waypoints.csv`。确认路线坐标、速度和场地安全后执行：

```bash
./scripts/start_visual_navigation.sh --autostart
```

该入口同时启动定位、融合、导航和串口，设置 `autostart=true`，并自动录制 rosbag。运行结束后严格只保留按时间排序最近 3 份 bag 数据，旧运行的 manifest 和参数快照仍保留。

### 包级 CSV 调试

下面的命令只用于单包开发调试，不会自动启动传感器、融合、串口或 rosbag recorder：

```bash
ros2 launch visual_navigation waypoint_navigation.launch.py \
  route_file:=/absolute/path/to/route.csv
```

包级 CSV 调试默认不会自动行驶。需要时可用现有服务启动：

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

停止或复位：

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_navigator/reset std_srvs/srv/Trigger '{}'
```

### 使用图形编辑器手动标点

导航节点运行后，另开终端启动编辑器：

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch visual_navigation route_editor.launch.py
```

网格每格固定为 `0.6 m`，原点箭头为车头初始的 `+X` 方向。左键添加航点，按住航点拖动可调整箭头方向，右键删除最后一个点。顶部显示 `ODOM READY` 后，点击“发布并启用路线”会通过 `/waypoint_navigation/route_input` 一次发布全部航点。

该编辑器是临时路线编辑工具，手动点选的路线直接作为 `Path` 发布，不会写入
`routes/*.csv`。需要保存为固定路线时，请按“航点文件”中的 CSV 格式自行保存，并在下次启动时通过 `route_file:=...` 加载。

“暂停小车/继续行驶”按钮使用 `route_editor` 作为驻停来源。`Space` 切换该来源的驻停状态，`Esc` 只申请驻停，不会误触恢复。若还有其他任务来源处于驻停状态，UI 释放后小车仍保持停止。

导航节点收到合法动态路线后会立即发零速度、清零航点索引并开始导航。编辑器等待 `/waypoint_path` 回显确认本次路线；确认失败会在界面显示，不需要手动执行服务或发布启动话题。历史 transient-local 路线回显不会触发编辑器重复发布。

动态路线中的速度和到达容差使用 `default_speed` 和 `waypoint_tolerance` 参数。当前默认巡航速度和最高线速度均为 `0.5 m/s`。默认启动不加载 CSV，重启导航节点后需要再次点击发布按钮才能重新启用编辑路线。

## 自动规划模式

自动规划由 `arena_path_planner` 负责，`visual_navigation` 只接收规划结果并执行。该模式不读取
`route_file`，也不需要手动调用 `/waypoint_navigator/start`。

终端 1 使用无参数统一入口启动视觉、融合、导航和串口链路：

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/start_visual_navigation.sh
```

终端 2 启动规划后端和前端：

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/start_arena_planner.sh
```

在规划前端中切换到 `vehicle`，确认目标和障碍物后点击“开始导航”。规划器会把通过碰撞检查、
确实有位移且能带来新任务/道路进展的安全阶段路线自动发布到
`/waypoint_navigation/route_input`；即使有暂时不可达的延迟目标，也先执行可达部分，完成后由
前端从最新位姿自动续规划。没有位移或没有新增进展的结果不会激活，避免重复路线把任务状态弄乱。
确认路线后，导航仍需通过定位、融合状态和执行器健康检查才会输出非零速度。
主启动脚本清理旧节点时会保留已经打开的规划 GUI、路线编辑 GUI、地图编辑 GUI 及路线编辑器的 ROS 父进程，不会替用户关闭窗口。

## 导航 Bringup

下面的 Launch 只启动导航节点，传感器和融合由外部总启动编排；它们是开发入口，不负责统一录包和三份保留策略：

```bash
ros2 launch visual_navigation visual_navigation_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  autostart:=false
```

需要同时启动串口桥接时使用：

```bash
ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  serial_device:=auto \
  serial_baud_rate:=115200 \
  autostart:=false
```

当前完整启动链固定发布 `base_link -> camera_link` 前向 `0.096m` 的二维静态 TF，并将 ORB、轮式里程计和融合结果统一到 `base_link`。重新测量安装位置后应同步修改该静态 TF；导航内部的旧相机位置补偿已经清零，不能重复补偿。

## 下位机连接

`/cmd_vel_nav` 是导航系统的最终 ROS 输出接口：

```text
linear.x  -> 目标线速度，m/s
angular.z -> 目标角速度，rad/s
```

`cup_car_serial` 节点订阅 `/cmd_vel_nav`，并按照 `vx,az\r\n` 格式向串口发送。默认设备参数为 `auto`，只接受唯一的 `/dev/serial/by-id/*` 设备；实车建议将 `serial_device` 显式设为实际的 `/dev/serial/by-id/...` 路径。波特率为 115200，发送频率为 20 Hz，命令超时为 0.4 秒。下位机仍必须实现独立通信看门狗。

## 安全说明

- 示例路线仅用于格式演示，禁止未确认场地时直接让实车执行。
- 当前系统不检测路线上的人员、车辆或临时障碍物。
- 节点参数 `autostart` 默认关闭；只有显式运行统一脚本的 `--autostart` 模式才会加载 `scripts/waypoints.csv` 并自动请求行驶。
- 调试初期应架空驱动轮或把最大速度调到很低。
- ORB-SLAM3 严重跟踪丢失后可能重新建立坐标原点，此时必须重新对齐航点坐标再启动任务。
