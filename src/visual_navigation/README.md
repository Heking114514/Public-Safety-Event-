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
| `/waypoint_navigation/route_plan_input` | `mission_control_interfaces/msg/WaypointRoute` | 接收规划 UI 发布的完整路线 |

发布：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 导航速度命令，下位机桥接输入 |
| `/waypoint_path` | `nav_msgs/msg/Path` | CSV 航点路线 |
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

最终航点首次进入位置容差后会锁定终点对角阶段。此后原地旋转造成的
`camera_link` 位置摆动不会让导航重新追踪最后一段路径方向；达到最终
`yaw` 容差后直接发布零速度并进入 `GOAL_REACHED`。

如果需要使用已有地图中的绝对航点，需要后续增加 `外部地图 -> ORB里程计坐标` 的对齐节点，不能仅修改 CSV 中的 `frame_id`。

## 构建

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-up-to visual_navigation
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

“暂停小车/继续行驶”按钮使用 `route_editor` 作为驻停来源。`Space` 切换该来源的驻停状态，`Esc` 只申请驻停，不会误触恢复。若还有其他任务来源处于驻停状态，UI 释放后小车仍保持停止。

导航节点收到合法动态路线后会立即发零速度、清零航点索引并开始导航。编辑器等待 `/waypoint_path` 回显确认本次路线；确认失败会在界面显示，不需要手动执行服务或发布启动话题。历史 transient-local 路线回显不会触发编辑器重复发布。

动态路线中的速度和到达容差使用 `default_speed` 和 `waypoint_tolerance` 参数。当前默认巡航速度和最高线速度均为 `0.5 m/s`。默认启动不加载 CSV，重启导航节点后需要再次点击发布按钮才能重新启用编辑路线。

## 赛场12点栅格路径规划器

`arena_route_planner.py` 按比赛图纸尺寸生成占用栅格，使用 A* 计算固定出发点、
当前选中默认点和手动目标点之间的可行路径，再使用 Held-Karp 动态规划求精确最短
闭环。规划结果会转换成以启动位置为原点、初始车头为 `+X` 的 `map` 坐标，
可以直接由本功能包的航点导航器读取。

在源码工作空间中打开可视化界面：

```bash
cd /home/j/Public-Safety-Event-hjh
python3 src/visual_navigation/scripts/arena_route_planner.py
```

默认 1 至 12 点启动时全部选中。左键或触摸屏单指点击任意白色可通行道路可增加
手动目标点；右键或触摸屏两指点击默认点或手动点可取消，取消后的默认点变灰，
再次左键点击可恢复。固定起点 `S` 不能取消，规划路线始终从 `S` 出发并返回 `S`。
目标点总数最多为 16 个。

点击“开始规划”后，界面只更新 A* 和 Held-Karp 的文字状态，不再绘制搜索展开、
分段流动或路线回放动画。规划成功后只显示最终红色路线。`shortest` 是目标点
最短闭环模式，`numbered` 按当前默认点编号及手动点添加顺序访问。

规划完成且里程计、融合状态和底盘连接均正常后，“一键发布并启动导航”按钮
变为可用。点击后，规划器通过 `/waypoint_navigation/route_plan_input` 发布完整
路线，保留每个航点的速度、容差和停车时间；导航器回显 `/waypoint_path` 后
界面显示确认结果并直接开始执行，不需要再调用 `/waypoint_navigator/start`。

完整实车运行只需要两个终端：

```bash
# 终端 1：完整定位、导航、串口和 rosbag
cd /home/j/Public-Safety-Event-hjh
./scripts/start_visual_navigation.sh --serial-device /dev/ttyUSB0

# 终端 2：规划、发布和实时监控 UI
cd /home/j/Public-Safety-Event-hjh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/visual_navigation/scripts/arena_route_planner.py
```

终端 1 使用的导航器不预加载 CSV，而是等待 UI 发布路线。UI 的发布按钮只有在
导航器订阅存在、里程计新鲜、融合状态允许且底盘连接为 `true` 时才会启用。
使用 `--no-serial` 调试时，应把 `arena_map.yaml` 中的
`monitoring.require_actuator_health` 改为 `false`。

界面同时作为实时导航监视器，默认订阅：

| 话题 | 显示内容 |
| --- | --- |
| `/odometry/fused` | 修正到车体参考点后的实时位置、朝向和蓝色实际轨迹 |
| `/odometry/fusion_status` | `FULL`、`DEGRADED_*` 或故障状态 |
| `/waypoint_navigation/status` | `IDLE`、`NAVIGATING`、`GOAL_REACHED` 等状态 |
| `/waypoint_navigation/current_waypoint` | 当前稠密航点索引和下一个任务编号 |
| `/waypoint_path` | 导航器实际加载的红色路线，用于索引匹配和任务高亮 |

里程计超过 `0.5 s` 未更新时，车体标记变灰并显示“里程计超时”。监控使用
与导航器相同的相机到车体偏移，然后把启动相对 `map` 位姿逆变换到赛场坐标。
打开界面前需要加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/j/Public-Safety-Event-hjh/install/setup.bash
ros2 run visual_navigation arena_route_planner.py
```

只查看和编辑地图、不连接 ROS 时可以增加 `--no-ros`。监控话题、超时时间、
轨迹点间距和车体偏移位于 `config/arena_map.yaml` 的 `monitoring` 段；其中
`tracking_point_offset_x/y` 必须与 `waypoint_navigation.yaml` 保持一致。
监视器和导航器都会用各自收到的第一帧里程计建立偏移参考朝向，因此应在小车
仍位于出发区且尚未运动时启动两者。

不打开图形界面直接生成：

```bash
python3 src/visual_navigation/scripts/arena_route_planner.py --headless
python3 src/visual_navigation/scripts/arena_route_planner.py --headless --mode numbered
```

默认输出目录为 `routes/arena_generated`，其中：

| 文件 | 用途 |
| --- | --- |
| `arena_route.csv` | 航点导航器读取的启动相对路线 |
| `arena_map.pgm`、`arena_map.yaml` | ROS 标准占用栅格 |
| `arena_route_preview.png` | 场地和最终路线预览 |
| `arena_route_report.json` | 访问顺序、长度和完整性检查结果 |

完成构建后也可以运行安装后的入口：

```bash
source install/setup.bash
ros2 run visual_navigation arena_route_planner.py
```

默认配置位于 `config/arena_map.yaml`。实车前至少核对 `start.position_m`、
`start.heading_deg`、`arena.inflation_radius_m`、速度和停车时间。默认假设小车
位于顶部出发区中心并朝向场内；如果摆放方向不同，CSV 与里程计坐标不会对齐。

只启动导航节点并加载生成路线：

```bash
ros2 launch visual_navigation waypoint_navigation.launch.py \
  route_file:=/home/j/Public-Safety-Event-hjh/src/visual_navigation/routes/arena_generated/arena_route.csv \
  autostart:=false

ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

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
