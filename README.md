# D455 视觉定位与小车控制使用手册

本工作空间支持两种导航模式，另有一种不经过导航规划器的手动速度模式：

1. **手动路点 / CSV 导航**：用户预先编辑 CSV 路点文件，或使用图形编辑器手动标点；`visual_navigation` 只负责跟随收到的路线，不执行 A*、动态避障或局部路径搜索。
2. **自动规划导航**：`arena_path_planner` 根据地图、当前位姿、剩余目标和动态障碍物生成路线，并把通过安全及新增进展检查的完整路线或阶段路线交给 `visual_navigation` 执行，不需要 CSV。
3. **手动速度控制**：不启动 `waypoint_navigator`，键盘或其他节点直接向 `/cmd_vel_nav` 发布速度；这不是导航模式。

两种导航模式和手动速度模式最终都使用标准 ROS 2 速度消息：

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
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

上面的 `/path/to/your/workspace` 仅是占位符，请替换为本仓库根目录。

新环境第一次构建时，在工作空间根目录运行对应架构脚本：

```bash
./x86_build.sh   # Ubuntu 22.04 x86_64
./arm_build.sh   # Ubuntu 22.04 ARM64，二选一
```

脚本会在需要时安装依赖，并在编译前询问 sudo 授权和并行线程数。完成首次原生库构建后，普通 ROS 2 包修改可直接使用 `colcon build`。

检查功能包：

```bash
ros2 pkg list | grep -E 'orbslam3|visual_navigation|cup_car_serial|realsense2_camera'
```

实车统一使用 `start_visual_navigation.sh`。不加参数时进入规划模式：脚本启动视觉、融合、导航和串口链路，等待规划 GUI 发布动态路线，并自动录制本次 rosbag：

```bash
./scripts/start_visual_navigation.sh
```

另开终端启动规划后端和 Python GUI，在界面确认路线后点击“开始导航”：

```bash
./scripts/start_arena_planner.sh
```

该包装脚本启动 C++ 规划服务后执行 `scripts/arena_route_frontend.py`；不要绕过它单独运行 GUI，否则界面没有规划服务可调用。
两个入口共用工作空间构建锁，即使两个终端紧接着启动，也不会并发写入 `build/`、`install/` 和 `log/`。规划入口会等待旧后端完全退出后再发布新服务，但不会关闭已经打开的规划、路线或地图 GUI。

统一入口的设备、传感器和构建默认值集中在 `config/navigation_startup.yaml`；录包保留数量和 topic 清单集中在 `config/navigation_recording.yaml`。两者分别由独立读取器校验，算法节点仍使用各功能包自己的 YAML，避免启动编排与融合、控制实现互相依赖。无参数 GUI 模式和 `--autostart` CSV 模式不允许由 YAML 改写，相关命令行选项优先于 YAML 默认值。

固定 CSV 自动运行时，先检查并编辑 `scripts/waypoints.csv`，再使用：

```bash
./scripts/start_visual_navigation.sh --autostart
```

`--autostart` 是唯一会在节点就绪后自动请求行驶的主启动模式，不要同时打开规划 GUI 或路线编辑器。无参数规划模式必须由 GUI 明确点击启动。常用调试选项：

```bash
./scripts/start_visual_navigation.sh --no-serial
./scripts/start_visual_navigation.sh --visualization
./scripts/start_visual_navigation.sh --serial-device /dev/ttyACM0
```

查看全部选项：

```bash
./scripts/start_visual_navigation.sh --help
```

---

# 导航模式一：手动路点 / CSV 导航

数据链路：

```text
D455 图像和 IMU
  → ORB-SLAM3
  → /odometry/visual_raw、/tracking_state
  → 里程计融合
  → /odometry/fused
  → waypoint_navigator
  → /cmd_vel_nav
  → 下位机桥接节点
  → 小车底盘
```

## 3. 启动完整程序

### 3.1 终端 1：启动相机、ORB-SLAM3 和融合定位

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fused_odometry odometry_bringup.launch.py \
  serial_no:=_038122250473 \
  visualization:=true \
  use_imu:=true \
  use_slam_imu:=false \
  equalize:=false \
  use_wheel:=true
```

保持该终端运行，不要按 `Ctrl+C`。

该 Launch 只负责传感器和融合定位，不启动航点导航或串口桥接。启动后应存在：

```text
/camera/camera
/orbslam3_stereo_inertial
/fused_odometry_gate
/fused_ekf
/map_odom_correction
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

### 3.2 固定 CSV 自动启动（统一入口）

先检查并编辑工作空间中的固定路线：

```text
scripts/waypoints.csv
```

然后只运行统一脚本：

```bash
./scripts/start_visual_navigation.sh --autostart
```

脚本把该 CSV 的规范化绝对路径传给导航节点并设置 `autostart=true`。路线文件及其 SHA-256 会写入本次 `run_manifest.json`；相机、融合、执行器健康条件未通过时仍保持零速度。该模式与无参数规划模式一样自动录包。

下面的包级 Launch 仅供开发调试，不是实车统一入口，也不会自动录制 rosbag：

```bash
ros2 launch visual_navigation visual_navigation_serial_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  route_frame:=map \
  cmd_vel_topic:=/cmd_vel_nav \
  serial_device:=auto \
  serial_baud_rate:=115200 \
  autostart:=false
```

需要临时手动标点时，可在无参数主脚本运行期间另开路线编辑器：

```bash
ros2 launch visual_navigation route_editor.launch.py
```

在网格上手动添加航点并点击“发布并启用路线”。编辑器直接发布动态 `Path`，导航节点会自动
激活该路线，不需要再调用 `/waypoint_navigator/start`；这种方式不会生成或读取 CSV。

### 3.3 检查串口设备

连接下位机后执行：

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

默认 `serial_device:=auto` 只接受唯一的 `/dev/serial/by-id/*` 设备。实车建议显式指定稳定路径；根据实际设备修改启动参数，例如：

```text
  serial_device:=/dev/serial/by-id/usb-your-device
```

请确保运行 ROS 2 的用户属于 `dialout` 组。启动日志中应出现：

```text
Forwarding /cmd_vel_nav and /imu/rpy to serial device 'auto' at 115200 baud
```

随后应看到连接日志中的实际 `/dev/serial/by-id/...` 路径。如果看到 `Cannot open serial port` 或 `No uniquely identified serial device found`，检查设备名、USB 连接和设备权限。串口不存在时节点会每秒重试，不会导致其他导航节点退出。

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

如果一直为 `-1`、`0` 或 `1`，保证场景有纹理并缓慢平移、转动相机完成初始化；双目惯性模式还需要让 IMU 感受到足够的加速度变化。

### 4.3 里程计和 TF

```bash
ros2 topic hz /odometry/fused
ros2 topic echo /odometry/fused --once
ros2 run tf2_ros tf2_echo map base_link
```

手动移动相机时，`/odometry/fused` 中的位置和姿态应合理变化。

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

航点和 `/odometry/fused` 必须使用同一个坐标系，当前均为 `map`。

## 5. 启动航点任务并查看速度

### 5.1 终端 2：监控速度

```bash
cd /path/to/your/workspace
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

确认 `/odometry/fusion_status` 为允许导航的状态且 `/odometry/fused` 正常后执行。若显式启用了 `require_tracking_state`，还应确认 `/tracking_state` 为 `2` 或 `5`：

```bash
cd /path/to/your/workspace
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

任务侦察期间驻停但保留导航任务、定位和当前航点：

```bash
ros2 service call /waypoint_navigator/set_motion_hold \
  mission_control_interfaces/srv/SetMotionHold \
  "{source: 'inspection', hold: true, reason: 'recognizing clue'}"
```

侦察完成后释放该任务来源：

```bash
ros2 service call /waypoint_navigator/set_motion_hold \
  mission_control_interfaces/srv/SetMotionHold \
  "{source: 'inspection', hold: false, reason: ''}"
```

`route_editor.py` 顶部按钮提供相同控制；`Space` 切换暂停/继续，`Esc` 只暂停。

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

# 导航模式二：自动规划导航

默认地图已经按正式比赛图转换为当前规划器使用的 YAML：总包络
`3.2 × 4.4 m`、主场地 `3.2 × 3.2 m`、10 个 `800 × 800 mm` 障碍块、
`200 mm` 道路和 4 段隧道。车辆采用实测的 `143.4 × 143.7 mm` 完整外廓，
每侧保留 `15 mm` 余量后直行包络为 `173.4 × 173.7 mm`，可以通过名义道路；
`124.7 mm` 运动学轮距是单独参数，与整车外廓不混用。图纸本身没有
任务编号，配置中的 12 个编号点是项目现有的
道路中心访问点。原 6×6 地图保存在
`src/arena_path_planner/config/arena_map_6x6_legacy.yaml`，需要时可通过
`./scripts/start_arena_planner.sh --map <文件>` 显式加载。

自动规划模式不读取 `route_file`，也不需要调用 `/waypoint_navigator/start`。规划前端向
`/arena_path_planner/plan` 提交当前位姿、剩余目标、已完成道路、障碍物和规划模式。规划器找到完整路线时直接执行；后续目标暂时不可达时，只要已得到的阶段路线通过碰撞检查、确实有位移并能增加任务或道路进度，也会在 `activate_navigation=true` 时发布到 `/waypoint_navigation/route_input`。导航节点收到后自动激活。定位、融合健康状态和执行器健康检查仍然是实际发车的必要条件。

当前只保留在线障碍话题，障碍识别本身尚未实现。导航始终订阅 `std_msgs/msg/Bool /obstacle/front_blocked` 和 `sensor_msgs/msg/Range /obstacle/front_range`；没有节点发布就等于没有障碍，不需要额外开关。后续独立感知节点先用 YOLO 确认目标，再从同一目标框内取得对齐深度并同步发布上述结果。默认启动不运行感知生产者，也不会用深度 ROI 猜障碍。

`success` 只表示规划结果合法，`navigation_activated` 才表示路线真的已发布，`all_targets_reached` 表示整场任务完成。前端必须收到匹配的 route ACK、看到导航实际运行并最终收到 `GOAL_REACHED`，才记录本段完成的任务、隧道和道路；随后使用最新融合位姿与剩余访问项自动规划下一段。隧道只有入口、出口都跑到才算完成，单纯绕行不会被记成道路覆盖。

## 自动规划启动步骤

### 终端 1：无参数启动视觉、融合、导航和串口

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/start_visual_navigation.sh
```

该终端会启动 D455、ORB-SLAM3、融合定位、`waypoint_navigator` 和
`cup_car_serial`，并等待 `/waypoint_navigation/route_input`。此模式不加载 CSV。

### 终端 2：启动自动规划前端

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/start_arena_planner.sh
```

前端启动后：

1. 将模式切换为 `vehicle`，确认融合里程计已经就绪。
2. 确认任务目标、目标顺序和必要的动态障碍物。
3. 点击“开始规划”可只查看规划结果，不会驱动车辆。
4. 检查预览没有明显穿障碍、坐标系或目标配置错误后，点击“开始导航”。
5. 前端会请求并激活第一条安全可执行路线；收到导航确认后，车辆才会在健康 gate 通过时运动。
6. 每段路线实际跑完后，前端自动提交本段进度并从当前位置续规划，直到剩余任务清零并完成回程。

存在暂时无法到达的目标不会直接结束任务。能带来新进展的安全阶段路线会先执行，其余目标保留为 `deferred_targets`；当前没有安全新进展、规划失败、里程计尚未恢复或路线未被导航确认时，前端会有界退避并自动重新探测。与路线有关的导航故障只让同一路线短时冷却，之后仍允许重新规划尝试，避免一次普通失败永久停车。planner 服务请求超过 30 秒未响应时，前端会取消旧请求、释放等待状态并继续处理最新请求，不会永久卡在 `in_flight`。

自动规划模式的停止方式：

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

停止后关闭两个启动终端即可。不要同时启动 `route_editor.py`，否则它也可能向同一个
`/waypoint_navigation/route_input` 发布路线。

---

# 附加模式：手动速度控制

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
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  serial_no:=_038122250473 \
  body_frame_id:=camera_link \
  map_frame_id:=map \
  visualization:=true \
  use_slam_imu:=true
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
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=auto \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav
```

默认 `device:=auto` 只接受唯一的 `/dev/serial/by-id/*` 设备；实车建议改为实际的
`/dev/serial/by-id/...` 路径。只有在设备枚举受控时，才额外启用
`allow_generic_auto_device:=true` 回退探测 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。

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
cd /path/to/your/workspace
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
cd /path/to/your/workspace
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

键盘控制需要 `teleop_twist_keyboard`。如果新环境尚未安装，请先执行：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

终端 2：

```bash
cd /path/to/your/workspace
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

### 15.0 rosbag 回放诊断数据

`scripts/start_visual_navigation.sh` 默认先增量构建 ORB-SLAM3 和本次启动所需 ROS 包，构建成功后才停止旧节点和操作相机。`--no-build` 只接受源码指纹一致、当前启动模式所需产物均可验证且哈希匹配上次成功构建的结果，否则直接退出；根目录两份导航 YAML 也属于运行源码指纹，修改后需要重新执行一次默认构建入口。

规划 GUI 和 `--autostart` CSV 两种实车模式都会无条件录包。每次运行先保存在独立的 `navigation_runs/<run-id>/bag`；结束时严格只保留 `config/navigation_recording.yaml` 中 `retain_bags` 指定数量的最新实际 bag，默认是 3 份。较旧运行只删除 `bag/`，`run_manifest.json` 和参数快照仍保留。`latest_navigation_run` 始终指向最近一次运行；`latest_navigation_bag` 只会原子指向最近一次完整或正常中断且已生成元数据、并且仍在保留范围中的 bag。若连续失败使旧有效 bag 退出保留范围，该兼容链接会移除，最近一次原始记录仍可从 `latest_navigation_run/bag` 检查。

启动脚本清理旧 ROS 节点时会明确保留 `arena_route_frontend.py`、`route_editor.py`、地图编辑器以及路线编辑器的 `ros2 launch/run` 父进程，不会关闭用户已经打开的 GUI。规划后端也保持独立，由第二终端的 `start_arena_planner.sh` 管理。

`run_manifest.json` 记录 commit、branch、dirty 状态、运行源码指纹、递归 submodule 状态、原始参数、有效设置、两份导航 YAML 及其读取器的 SHA-256、运行文件和二进制 SHA-256、bag topics、退出码，以及 `parameters/` 中从真实节点抓取的参数快照。可直接检查：

```bash
python3 -m json.tool latest_navigation_run/run_manifest.json | less
```

默认清单不录制任何原始/压缩图像或点云，只保留两路 `camera_info`、IMU、ORB 跟踪与视觉里程输出、融合输入/输出、控制与执行器状态、规划路径/栅格、`/diagnostics`、`/parameter_events`、`/rosout` 和 TF。这样不会再产生两路 848x480 Y8 30 Hz 图像约 `1.47 GB/min` 的主要负载，也没有重复录制 `/odom`、`/pose` 等兼容输出或每帧增长的 ORB `/path`。代价是该 bag 只能分析在线 ORB 的结果，不能离线重新运行 ORB；比赛默认优先保证磁盘空间和在线导航稳定。

ROS 2 Humble 的 rosbag 不记录 service 请求/响应。当前 bag 足以分析在线视觉定位结果，并排查 IMU、轮速融合、导航控制、执行器和规划结果，但不能离线重跑 ORB，也不能单独重建 `/arena_path_planner/plan` 请求中的目标标签、动态障碍和访问顺序。规划器算法回归仍应同时保留任务输入或测试用例。

回放前可确认录制内容：

```bash
ros2 bag info latest_navigation_bag
ros2 bag play latest_navigation_bag
```

启动阶段融合尚未初始化或尚未发布时，航点导航保持零速度等待，不锁存运行故障；连续有效定位到来后，已激活路线会自动继续。已经正常运行后发生的真实定位故障仍按导航监督策略降速、停车恢复或锁存，不能把运行中失去可信定位当成普通暖机。

### 15.1 手动路点 / CSV 导航

- [ ] 左右红外图像约 30 Hz，IMU 约 200 Hz。
- [ ] `/tracking_state` 为 `2` 或 `5`。
- [ ] `/odometry/fused` 持续发布并随小车运动变化。
- [ ] `/waypoint_path` 正确显示 CSV 路线或手动标点路线。
- [ ] `--autostart` 模式加载 `scripts/waypoints.csv`，健康检查通过后进入 `FOLLOWING`。
- [ ] `/cmd_vel_nav` 输出合理的 `linear.x` 和 `angular.z`。
- [ ] 到达航点后当前航点编号递增。
- [ ] 调用 `stop` 或定位丢失后速度归零。
- [ ] 下位机桥接节点能够收到 `/cmd_vel_nav`。

### 15.2 自动规划导航

- [ ] `arena_path_planner` 提供 `/arena_path_planner/plan` 服务。
- [ ] 前端切换为 `vehicle`，并显示融合里程计就绪。
- [ ] “开始规划”能生成完整或安全阶段路线；有延迟目标但能新增真实进度时允许激活，没有位移、起点不匹配或没有新进展时不激活。
- [ ] 点击“开始导航”后 `/arena_path_planner/navigation_path` 和 `/waypoint_path` 内容一致。
- [ ] `/waypoint_navigation/route_ack` 收到本次路线确认。
- [ ] `GOAL_REACHED` 前不提交本段任务/道路；完成后从最新位姿自动续规划，隧道双检查点和道路覆盖结果正确。
- [ ] 暂无安全路线、odom 暂失或 ACK 超时时自动等待重试，不需要人工重新点击；迟到 ACK 不提交旧路线进度。
- [ ] 自动规划模式不传 `route_file`，也不同时启动 `route_editor.py`。

### 15.3 手动速度控制

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
source /path/to/your/workspace/install/setup.bash
```

### 一直没有 `/odometry/fused`

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
ros2 topic echo /odometry/fused --once
```

默认情况下，导航节点依据融合状态、`/odometry/fused` 新鲜度和位姿有效性决定是否输出非零速度；最近一次定位有效后的短暂异常可在宽限期内按配置降速，允许的 `DEGRADED_*` 状态也会按各自配置降速。只有显式启用 `require_tracking_state` 时，才额外要求 `/tracking_state` 为 `2` 或 `5`。

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

### 固定 CSV 自动导航

```bash
# 先检查 scripts/waypoints.csv
./scripts/start_visual_navigation.sh --autostart

ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

### 自动规划导航

```bash
# 终端 1：视觉、融合、导航和串口
./scripts/start_visual_navigation.sh

# 终端 2：自动规划后端和前端
./scripts/start_arena_planner.sh

# 在前端切换 vehicle，确认目标后点击“开始导航”
```

### 手动速度控制

```bash
# 可选：只启动相机和 ORB-SLAM3
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  serial_no:=_038122250473 \
  body_frame_id:=camera_link \
  map_frame_id:=map \
  visualization:=true \
  use_slam_imu:=true

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_nav

# 停车
ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```
