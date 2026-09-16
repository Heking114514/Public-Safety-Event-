# visual_navigation 导航代码说明

## 1. 文档目的

本文档说明 `visual_navigation` 功能包中航点导航代码的设计和运行逻辑，主要面向：

- 后续维护导航代码的开发人员
- 需要调整控制参数的调试人员
- 需要接入串口或 CAN 下位机的电控人员
- 需要制作航点路线文件的使用人员

当前实现是一个轻量级固定航点导航系统：

```text
融合里程计 /odometry/fused + /odometry/fusion_status
ORB-SLAM3 /tracking_state
                 │
                 ▼
          waypoint_navigator
                 │
                 ├── /waypoint_path
                 ├── /waypoint_navigation/status
                 └── /cmd_vel_nav
                            │
                            ▼
                      下位机桥接节点
```

系统不使用 Nav2。导航节点根据融合里程计提供的当前位置依次跟踪规划器或 CSV 给出的航点；可选接收独立感知节点的前方静态障碍事件，负责立即停车、沿实际轨迹倒退到恢复锚点，并通知任务层重规划。

---

## 2. 功能包目录

```text
visual_navigation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── NAVIGATION_CODE_GUIDE.md
├── config/
│   └── waypoint_navigation.yaml
├── launch/
│   ├── waypoint_navigation.launch.py
│   ├── route_editor.launch.py
│   ├── visual_navigation_bringup.launch.py
│   └── visual_navigation_serial_bringup.launch.py
├── routes/
│   └── example_route.csv
├── include/visual_navigation/
│   ├── waypoint_navigator.hpp
│   ├── control_state.hpp
│   ├── obstacle_recovery.hpp
│   └── 控制器、输入缓存与监督器头文件
└── src/
    ├── waypoint_navigator.cpp
    ├── node_init.cpp
    ├── route.cpp
    ├── runtime.cpp
    ├── obstacle_recovery.cpp
    ├── control.cpp
    ├── maneuvers.cpp
    ├── path_following.cpp
    ├── progress.cpp
    ├── navigation.cpp
    └── output.cpp
```

各文件作用：

| 文件 | 作用 |
| --- | --- |
| `include/visual_navigation/waypoint_navigator.hpp` | ROS 节点声明及节点内部共享状态 |
| `include/visual_navigation/control_state.hpp` | 互斥控制阶段、终点父状态和状态文本映射 |
| `include/visual_navigation/obstacle_recovery.hpp` | 实际轨迹缓存、倒车目标、超时/无进展监督和纯控制逻辑 |
| `src/waypoint_navigator.cpp` | 程序入口，只负责启动节点 |
| `src/node_init.cpp` | 参数、ROS 接口和初始状态 |
| `src/route.cpp` | CSV/动态路线加载、发布和接收 |
| `src/runtime.cpp` | 运行时输入回调、启停复位、任务驻停和时间戳校验 |
| `src/obstacle_recovery.cpp` | 障碍分类、立即停车、停稳确认、专用倒车和重规划状态 |
| `src/control.cpp` | 安全门控、周期上下文和状态机分派 |
| `src/maneuvers.cpp` | 等待、制动、航点恢复和终点动作 |
| `src/path_following.cpp` | 路径跟随和路径方向对齐 |
| `src/progress.cpp` | 路径与转向进度监督 |
| `src/navigation.cpp` | 航段、航点和控制器生命周期辅助 |
| `src/output.cpp` | 速度、航点、任务驻停和状态发布 |
| `config/waypoint_navigation.yaml` | 导航节点默认参数 |
| `routes/example_route.csv` | 航点文件格式示例 |
| `launch/waypoint_navigation.launch.py` | 只启动航点导航节点 |
| `launch/route_editor.launch.py` | 启动图形路线编辑器 |
| `launch/visual_navigation_bringup.launch.py` | 启动航点导航节点；传感器和融合由外部启动 |
| `launch/visual_navigation_serial_bringup.launch.py` | 启动航点导航节点和串口桥接 |
| `CMakeLists.txt` | 安装导航节点、Launch 文件、路线资源和包内的 `route_editor.py` |
| `README.md` | 构建和快速使用说明 |
| `NAVIGATION_CODE_GUIDE.md` | 代码设计与详细逻辑说明 |

`route_editor.launch.py` 启动的是由 `CMakeLists.txt` 安装到包前缀的包内 `scripts/route_editor.py`。
工作空间根目录的同名脚本只是源码树中的便捷启动入口，不再承载另一份实现。

---

## 3. 节点职责

导航核心目前是一个 C++ 节点，另有一个用于发布动态路线的 Python 图形编辑器：

```text
waypoint_navigator
```

该节点负责：

1. 从 CSV 文件读取航点。
2. 订阅融合节点发布的最终导航里程计。
3. 订阅 ORB-SLAM3 跟踪状态。
4. 接收启动、停止和复位服务。
5. 按顺序选择当前目标航点。
6. 根据位置误差和航向误差计算速度。
7. 发布 `geometry_msgs/msg/Twist`。
8. 定位失效、陈旧或融合状态不允许时立即发布零速度。
9. 接收可选前障碍事件，执行停车和退回路口，不承担障碍检测与新路线搜索。
10. 发布路线、当前航点编号和导航状态。

它不负责：

- 建图
- 地图加载
- 障碍物检测
- 路径搜索
- 障碍物几何建图和绕行路径搜索
- 串口或 CAN 数据发送
- 轮速闭环控制

下位机通信由单独的 `cup_car_serial` 节点完成；后续也可以替换为 CAN 或 UDP 桥接节点。

---

## 4. ROS 接口

### 4.1 订阅话题

#### `/odometry/fused`

消息类型：

```text
nav_msgs/msg/Odometry
```

使用字段：

```text
header.frame_id
pose.pose.position.x
pose.pose.position.y
pose.pose.orientation
twist.twist.linear.x
```

节点把四元数转换为平面偏航角 `yaw`，用纵向速度反馈提前制动。停车确认优先使用刹车开始后收到的新鲜 MCU 左右轮瞬时速度；MCU 遥测缺失或过期时，才回退到约 `0.2 s` 的融合位置位移窗口。轮速只参与停车确认，不进入位置融合。

当前代码不使用 Z 方向位置、横滚角和俯仰角，也不积分里程计速度来计算位置。

#### `/tracking_state`

消息类型：

```text
std_msgs/msg/Int32
```

只有下面两个状态被认为定位有效：

| 数值 | ORB-SLAM3 状态 |
| --- | --- |
| `2` | `OK` |
| `5` | `OK_KLT` |

只有在参数 `require_tracking_state` 设置为 `true` 时，其他状态才会阻止运动命令。默认配置下，`/tracking_state` 作为兼容性观测，导航主要依据融合状态和融合里程计判断是否允许运动。

#### `/odometry/fusion_status`

消息类型：

```text
std_msgs/msg/String
```

默认必须收到新鲜且位于 `allowed_fusion_states` 中的融合健康状态；`FAULT`、未知状态或状态超时会阻止运动。`WAITING_FOR_INITIALIZATION` 是 ORB 暖机期间的可恢复等待，路线保持激活但不发速度，初始化完成后自动继续。允许的 `DEGRADED_*` 状态不降低速度；它只表示融合基础设施已降级但仍可向控制器提供可用位姿。

### 4.2 发布话题

#### `/cmd_vel_nav`

消息类型：

```text
geometry_msgs/msg/Twist
```

当前只使用：

```text
linear.x   目标线速度，单位 m/s
angular.z  目标角速度，单位 rad/s
```

其他字段始终为零。

该话题是导航系统提供给下位机桥接节点的最终控制接口。

#### `/waypoint_path`

消息类型：

```text
nav_msgs/msg/Path
```

节点启动并成功读取路线后发布一次完整航点路径。该发布器使用 `transient_local` QoS，因此 RViz 或其他节点稍后启动也能收到最后一条路径。

该路径只用于显示和监控，不参与在线路径搜索。

#### `/waypoint_navigation/status`

消息类型：

```text
std_msgs/msg/String
```

只有状态发生变化时才重新发布。发布器使用 `transient_local` QoS。

#### `/waypoint_navigation/current_waypoint`

消息类型：

```text
std_msgs/msg/Int32
```

表示当前正在跟踪的航点索引。第一个航点索引为 `0`。

### 4.3 服务

#### `/waypoint_navigator/start`

类型：

```text
std_srvs/srv/Trigger
```

作用：

- 启动导航任务。
- 从当前航点索引继续执行。
- 如果定位尚未有效，则进入等待定位状态。

调用命令：

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

#### `/waypoint_navigator/stop`

作用：

- 停止导航。
- 发布零速度。
- 保留当前航点索引。

再次调用 `start` 时从保留的索引继续。

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

#### `/waypoint_navigator/reset`

作用：

- 停止导航。
- 发布零速度。
- 把当前航点索引恢复为 `0`。

```bash
ros2 service call /waypoint_navigator/reset std_srvs/srv/Trigger '{}'
```

任务完成后如需重新从头执行，应先调用 `reset`，再调用 `start`。

---

## 5. 航点数据结构

代码中每个航点由 `Waypoint` 结构体保存：

```cpp
struct Waypoint
{
  double x;
  double y;
  double yaw;
  double speed;
  double tolerance;
  double stopTime;
};
```

字段含义：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `x` | m | 航点 X 坐标 |
| `y` | m | 航点 Y 坐标 |
| `yaw` | rad | 航点期望朝向，可留空 |
| `speed` | m/s | 前往该航点时允许的最大线速度 |
| `tolerance` | m | 判断到达航点的距离半径 |
| `stopTime` | s | 到达航点后的停车时间 |

`yaw` 留空时使用 `NaN` 表示没有指定朝向。

当前控制逻辑只对最终航点执行精确朝向对准。中间航点的 `yaw` 主要用于发布路径时显示方向，不会让车辆在每个中间航点都强制转到该角度。

---

## 6. CSV 航点文件

### 6.1 文件格式

```csv
x,y,yaw,speed,tolerance,stop_time
0.0,0.0,0.0,0.10,0.12,0.0
1.0,0.0,0.0,0.20,0.12,0.0
1.0,1.0,1.570796,0.15,0.12,0.0
```

最少必须填写：

```text
x,y
```

其余字段可以留空：

```csv
x,y,yaw,speed,tolerance,stop_time
0.0,0.0,,,,
1.0,0.0,,0.20,,
1.0,1.0,1.570796,0.10,0.08,2.0
```

留空字段的处理方式：

| 字段 | 留空行为 |
| --- | --- |
| `yaw` | 不要求该航点朝向；发布路径时根据相邻航点推导显示朝向 |
| `speed` | 使用参数 `default_speed` |
| `tolerance` | 使用参数 `waypoint_tolerance` |
| `stop_time` | 使用 `0.0` |

支持：

- 第一行表头
- 空行
- 以 `#` 开头的注释
- 数据行末尾注释
- 普通小数和科学计数法

### 6.2 文件加载时机

路线只在节点构造时加载一次。

如果运行期间修改 CSV 文件，节点不会自动重新加载；需要重启节点或重新启动
Launch 才会读取新的文件。运行中也可以通过 `/waypoint_navigation/route_input`
发布合法的 `nav_msgs/msg/Path` 动态替换路线，路线编辑器使用的就是这个接口。

### 6.3 文件校验

以下情况会导致路线加载失败：

- `route_file` 为空
- 文件不存在或无法打开
- 数据行少于两列
- 数值无法转换为浮点数
- `speed < 0`
- `tolerance <= 0`
- `stop_time < 0`
- 文件中没有有效航点

加载失败后节点状态为：

```text
FAULT_ROUTE_NOT_LOADED
```

启动服务也会返回失败。

---

## 7. 坐标系要求

### 7.1 当前实现不执行 TF 转换

参数 `route_frame` 表示航点所在坐标系，默认值为：

```text
map
```

导航节点会比较：

```text
/odometry/fused.header.frame_id
route_frame
```

如果 `header.frame_id` 为空或与 `route_frame` 不同，或者 `child_frame_id` 为空或与
`odom_child_frame`（默认 `base_link`）不同，节点会丢弃该样本，但不会覆盖上一条有效
里程计或推进时间戳基线。节点不会自动查询 TF，也不会转换坐标；合法样本持续缺失超过
`odom_timeout` 后进入 `WAITING_FOR_ODOMETRY` 并停车等待。

因此必须保证：

```text
航点坐标系 == /odometry/fused 位姿坐标系
```

仅修改 CSV、RViz 或消息中的 `frame_id` 字符串不会真正完成坐标转换。

### 7.2 与当前 ORB-SLAM3 节点的关系

当前 ORB-SLAM3 ROS 节点会在第一次定位成功时把当前车体位姿作为发布坐标原点。因此使用本导航包时，第一版建议：

1. 每次把小车放在固定启动位置。
2. 保证启动朝向一致。
3. ORB-SLAM3 初始化完成后再启动导航。
4. 航点以该固定启动原点为基准记录。

如果需要使用已有二维地图中的绝对航点，应增加坐标对齐节点：

```text
外部地图 map
   └── ORB里程计坐标 vio_map
          └── base_link
```

当前导航节点没有实现该对齐功能。

### 7.3 车体坐标系

控制算法假设当前 `yaw` 表示小车正前方方向，并假设：

```text
linear.x > 0
```

会让车辆沿 `base_link` X 正方向前进。

如果 ORB-SLAM3 输出的是相机坐标而不是车体中心坐标，必须准确配置相机与 `base_link` 的静态 TF，否则可能出现：

- 前进方向不正确
- 左右转方向错误
- 转弯时路径偏移
- 到达位置与车体中心不一致

---

## 8. 节点初始化流程

节点构造函数执行顺序如下：

```text
声明并读取参数
        │
        ▼
创建发布器、订阅器和服务
        │
        ▼
读取 CSV 航点文件
        │
        ├── 失败：FAULT_ROUTE_NOT_LOADED
        │
        └── 成功：发布 /waypoint_path
                         │
                         ▼
                  创建控制定时器
                         │
                         ▼
          route_file 为空：WAITING_FOR_ROUTE
          route_file 已加载且 autostart=false：按手动启动流程等待
          autostart=true ：等待有效定位
```

控制定时器默认运行频率为 `30 Hz`。

`route_file` 成功加载且 `autostart=false` 时进入 `IDLE`，表示路线已就绪但尚未请求
运动；只有指定文件加载失败才进入 `FAULT_ROUTE_NOT_LOADED`。调用 `start` 后再按定位、
融合状态和执行器健康状态进行检查。

---

## 9. 定位有效性检查

导航启动和运行监督依次检查：

1. 是否收到过 `/odometry/fused`。
2. 最近一次 `/odometry/fused` 到达时间是否超过 `odom_timeout`。
3. 里程计坐标系、平面位姿和四元数是否有效。
4. 是否要求融合健康状态，以及状态是否新鲜、允许且非故障。
5. 是否显式要求检查 `/tracking_state`；若要求，是否收到状态 `2` 或 `5`。
6. 串口 bringup 是否要求执行器健康，以及执行器遥测是否新鲜且健康。

消息回调同时拒绝未设置、非单调、过期或未来超限的测量时间戳；输入新鲜度使用消息到达节点的时间：

```text
now() - last_odom_arrival
```

因此不能仅靠重复发布带旧时间戳的消息绕过超时。

默认配置：

```yaml
odom_timeout: 0.40
require_tracking_state: false
abort_on_tracking_loss: false
```

默认导航以 `/odometry/fused` 和 `/odometry/fusion_status` 作为定位接口；`/tracking_state` 仅在显式启用 `require_tracking_state` 时参与放行判断。若需要在 ORB 跟踪丢失后立即锁存任务，再将 `abort_on_tracking_loss` 设为 `true`。

### 初始阶段没有定位

启动服务已经调用，但 ORB-SLAM3 尚未初始化时：

```text
WAITING_FOR_LOCALIZATION
```

节点持续发布零速度，定位有效后自动进入 `FOLLOWING`。

### 运行中定位丢失

如果导航曾经获得有效定位，之后定位失效：

- 默认 `abort_on_tracking_loss=false` 时，节点发布零速度并等待恢复；收到新鲜、有效且允许的融合位姿后自动继续。
- 允许的 `DEGRADED_*` 状态仍按路线速度和全局限幅正常行驶，不再通过融合健康状态改写速度。
- 只有同时显式启用跟踪状态检查，并设置：

```yaml
require_tracking_state: true
abort_on_tracking_loss: true
```

节点才会：

1. 发布零速度。
2. 关闭当前导航任务。
3. 进入 `FAULT_TRACKING_LOST`。
4. 不自动恢复运动。

如果是融合硬故障，则进入对应的 `FAULT_FUSION_STATUS` 或执行器/里程计故障状态。
这是因为 ORB-SLAM3 严重丢失后可能重新建立发布坐标原点，旧航点可能已经失效。

### 内部控制状态机

任务驻停、导航未启动和输入监督属于外层安全门控，不与运动阶段混在一起。
安全门控允许运动后，`ControlState` 保证每个周期只有一个活动阶段：

```text
FOLLOW
  ├── ALIGN_PATH ───────────────┐
  ├── RECOVER_WAYPOINT ── BRAKE ├── FOLLOW / WAIT
  └── BRAKE ── ALIGN_GOAL ──────┴── GOAL_REACHED
                    └── RECOVER_GOAL ── BRAKE ── ALIGN_GOAL
```

`GoalStage` 是终点动作的父状态，用于区分尚未到达终点、终点待停稳、已经停稳和
最终位置回收。路径对齐、回收、制动和等待不再由可同时为真的布尔标志表达。
本航段是否完成过路径对齐、转向锚点和等待截止时间仍作为阶段上下文保存。

重置也按生命周期区分：路径反馈重置只清 PID；动作中断另外清转向稳定计时、
制动计时和当前动作阶段；整次任务重置才清终点父状态与等待动作。
为兼容原有控制逻辑，输入监督中断会保留终点位置恢复父状态，但把当前回收动作
退回 `FOLLOW`，恢复输入后重新按终点距离和路径投影选择动作。

---

## 10. 导航状态

当前代码可能发布以下状态：

| 状态 | 含义 |
| --- | --- |
| `IDLE` | 已停止或复位，当前未执行路线 |
| `WAITING_FOR_ROUTE` | 尚未加载路线，等待 CSV 或动态路线 |
| `WAITING_FOR_LOCALIZATION` | 已请求启动，正在等待有效定位 |
| `WAITING_FOR_ODOMETRY` | 等待新鲜且有效的融合里程计 |
| `WAITING_FOR_ALLOWED_FUSION_STATUS` | 融合状态未进入允许列表 |
| `WAITING_FOR_ACTUATOR_RECOVERY` | 执行器不健康，保持零速度并等待恢复 |
| `RESUME_PENDING_LOCALIZATION` | 任务驻停释放后等待重新确认定位 |
| `HELD_FOR_MISSION` | 任务驻停来源仍存在，持续发布零速度 |
| `WAITING_FOR_INITIALIZATION` | ORB 暖机等待，不锁存，保持零速 |
| `FOLLOWING` | 正在向当前航点运动 |
| `ROTATING_TO_PATH` | 正在原地旋转以对齐当前路径 |
| `TURN_HALF_SETTLED` | 180 度掉头的第一个 90 度已停稳，随后执行第二个 90 度 |
| `PATH_ALIGNED` | 路径方向已对齐，准备进入路径跟随 |
| `BRAKING_APPROACH` | 按剩余距离和实际车速执行航点接近制动 |
| `BRAKING_AT_WAYPOINT` | 已收点，持续发零速并等待车体停稳后再转向 |
| `WAITING_AT_WAYPOINT` | 已到达航点，按照 `stop_time` 停车等待 |
| `RECOVERING_FINAL_POSITION` | 最终朝向调整后发现位置超差，开始终点位置回收；后续回收周期沿用 `RECOVERING_WAYPOINT` |
| `ALIGNING_FINAL_YAW` | 已到达最终位置，正在调整最终朝向 |
| `RECOVERING_WAYPOINT` | 航点越过或位置超差，正在低速回收 |
| `REVERSING_PLANNED_RETREAT` | 路段末端不可继续前进，保持车头方向并沿原路倒回路口 |
| `RETREATING_TO_JUNCTION` | 路中出现过大方向误差，正沿实走轨迹倒回最后一个已确认路口 |
| `RETREATING_TO_ROUTE_START` | 尚未经过路口时出现过大方向误差，正沿实走轨迹倒回本次路线起点 |
| `RECOVERING_NO_PATH_PROGRESS` | 路径进度不足，正在重置路径反馈并尝试恢复 |
| `RECOVERING_NO_TURN_PROGRESS` | 转向进度不足，正在重置转向控制并尝试恢复 |
| `RECOVERING_WAYPOINT_BRAKE` | 未在单次时间窗内确认停稳，继续发零速并自动重新确认，不结束任务 |
| `GOAL_REACHED` | 已完成全部航点 |
| `FAULT_ROUTE_NOT_LOADED` | 航点文件加载失败 |
| `FAULT_ODOMETRY_FRAME` | 融合里程计坐标系与 `route_frame` 不一致 |
| `FAULT_ODOMETRY_INVALID` | 融合里程计位姿或四元数无效 |
| `FAULT_FUSION_STATUS_STALE` | 融合健康状态未收到或已超时 |
| `FAULT_FUSION_STATUS` | 融合报告故障状态 |
| `FAULT_INIT_TIMEOUT` | ORB 在限定时间内未完成初始化 |
| `FAULT_TRACKING_LOST` | 启用跟踪检查且丢失后锁存任务 |
| `FAULT_ACTUATOR_STALE` | 执行器健康消息未收到或已超时 |
| `FAULT_ACTUATOR_DISCONNECTED` | 执行器连接或健康状态无效 |
| `FAULT_NO_PATH_PROGRESS` | 路径进度恢复尝试耗尽 |
| `FAULT_NO_TURN_PROGRESS` | 转向进度恢复尝试耗尽 |

### 典型状态转换

```text
IDLE
  │ start
  ▼
WAITING_FOR_LOCALIZATION
  │ 定位有效
  ▼
FOLLOWING
  │ 到达带停留时间的航点
  ▼
WAITING_AT_WAYPOINT
  │ 等待结束
  ▼
FOLLOWING
  │ 到达最终位置且指定最终 yaw
  ▼
ALIGNING_FINAL_YAW
  │ 朝向误差满足要求
  ▼
GOAL_REACHED
```

异常分支：

```text
FOLLOWING
  │ 里程计陈旧、位姿无效或融合状态不允许
  ▼
WAITING_FOR_ODOMETRY / WAITING_FOR_ALLOWED_FUSION_STATUS / 对应 FAULT_* 状态
  │ 输入恢复且未锁存
  ▼
FOLLOWING
```

---

## 11. 航点切换逻辑

当前目标航点由：

```text
currentWaypointIndex
```

记录。

对于当前航点，计算欧氏距离：

```text
dx = target_x - current_x
dy = target_y - current_y
distance = sqrt(dx² + dy²)
```

当：

```text
distance <= target.tolerance
```

认为已经到达该航点。

### 中间航点

- `stop_time == 0`：索引立即加一。
- `stop_time > 0`：先停车等待，等待结束后索引加一。

### 最终航点

1. 先检查位置距离。
2. 如果最终航点填写了 `yaw`，执行最终朝向控制。
3. 如果配置了 `stop_time`，停车等待。
4. 进入 `GOAL_REACHED`。

航点索引只向前推进，不会自动倒退，也不会在整条路线中重新搜索最近航点。

普通非停车采样点一旦被纵向越过，索引继续前进，由下一段的跟线反馈收敛横向误差，不再回头追逐车后的旧采样点。需要停车的转角和最终点仍执行严格的径向/横向到达检查。

这可以避免路线交叉时跳到错误路段，也避免密集路径点触发反复回追；车辆如果偏离路线很远，系统仍只会沿当前索引之后的路线恢复，不会全局重搜最近路段。

---

## 12. 当前控制算法

当前实现是逐航点的直线路径跟随器。每一段从开始导航或到达上一航点时的当前位置开始，终点为当前航点；航向误差和横向误差进入 PID，持续补偿左右轮机械差异造成的跑偏。

### 12.1 目标方向

```text
path_heading = atan2(target_y - segment_start_y,
                     target_x - segment_start_x)
```

### 12.2 航向误差

```text
heading_error = normalize(path_heading - current_yaw)
```

`normalize()` 把角度限制到：

```text
[-π, π]
```

避免从 `179°` 到 `-179°` 时产生接近 `360°` 的错误旋转。

### 12.3 角速度

```text
cross_track_error = cos(path_heading) × (current_y - segment_start_y)
                    - sin(path_heading) × (current_x - segment_start_x)
path_error = heading_error - cross_track_gain × cross_track_error
angular_z = path_pid_kp × path_error
          + path_pid_ki × integral(path_error)
          + path_pid_kd × derivative(path_error)
```

并限制到：

```text
[-max_angular_speed, max_angular_speed]
```

### 12.4 原地转向判断

只有车辆已经在计划拐点停稳，且该拐点方向变化超过约 40 度时，才允许原地转向。普通直线路段的横向误差不会再触发原地转；180 度掉头固定拆成两个 90 度，每段都发零速确认停稳。

当：

```text
abs(heading_error) >= rotate_in_place_threshold
```

执行：

```text
linear_x = 0
```

车辆只旋转，不向前移动。

当误差小于阈值后才允许前进。

### 12.5 线速度

允许前进时，基础线速度为：

```text
requested_speed = min(waypoint.speed, max_linear_speed)
heading_scale = max(0, cos(heading_error))
d_available = max(remaining_distance - braking_safety_margin, 0)
v_limit = -a × delay + sqrt((a × delay)^2 + 2 × a × d_available)
linear_x = min(requested_speed, v_limit, cross_track_limit) × heading_scale
```

该计算实现了以下行为：

1. 不超过航点指定速度。
2. 不超过全局最大速度。
3. 需要转弯或结束的航点根据剩余距离进入物理制动曲线，直线连续航点可直接通过。
4. 到达需要转弯的中间航点或最终点后，用融合里程计实际纵向速度确认停稳。
5. 航向误差或横向误差越大，前进速度越低。

相邻路径方向变化小于 `pre_turn_stop_heading_threshold` 时可以连续通过；超过阈值时先停稳再转向。
需要停车的拐点进站时，路径纠偏角速度还会被 `pre_turn_path_angular_ratio × linear_x`
限制，避免刹车阶段演变成提前切弯；到点停稳后再进入原地转向。

### 12.6 最终航点

最终航点可能位于路中，因此路径消息携带的切线朝向不再授权原地调整；位置到达并确认停车后直接完成。需要改变行驶方向时，规划器必须把转向安排在前一个真实路口。

动态 `nav_msgs/Path` 是二维路径。`pose.position.z=0.001` 表示该点是规划器确认的真实转向路口，`z=0` 表示普通路径点。导航只把这个标记当作普通路口转向授权；规划器不再发布 180 度掉头或短距离连续 90 度转向。遇到前方障碍需要退出时，由障碍恢复模块停车、倒回上一个恢复锚点/路口，再等待重规划；其他 z 值会被拒绝。

---

## 13. 控制循环伪代码

```text
每个控制周期：

  如果任务驻停有效：
      重置进度监督器
      发布零速度
      状态 = HELD_FOR_MISSION
      返回

  如果导航未启动：
      发布零速度
      返回

  运行 NavigationSupervisor：
    如果执行器、里程计或融合状态不可用：
      发布零速度
      按故障类型设置 WAITING_* 或 FAULT_* 状态
      返回

    如果融合处于允许的 DEGRADED_* 状态：
      不改写速度，继续进入路线控制

  如果正在航点等待：
      发布零速度
      如果等待时间未结束：
          返回
      根据等待类型推进航点或完成任务

  读取当前目标航点
  计算距离

  如果到达航点：
      如果是最终航点且需要朝向对准：
          原地旋转
          返回

      如果需要停车等待：
          开始计时
          返回

      如果是最终航点：
          完成任务
      否则：
          current_waypoint_index++
      返回

  计算目标方向和航向误差
  计算角速度

  如果路径或转向进度监督器判定无进展：
      发布零速度并重置控制反馈
      进入 RECOVERING_NO_PATH_PROGRESS 或 RECOVERING_NO_TURN_PROGRESS
      恢复次数耗尽后进入对应的 FAULT_NO_PATH_PROGRESS 或 FAULT_NO_TURN_PROGRESS

  如果航向误差小：
      计算线速度
  否则：
      线速度 = 0

  发布 Twist
```

---

## 14. 参数说明

参数文件：

```text
config/waypoint_navigation.yaml
```

### 14.1 文件和话题参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `route_file` | 空 | CSV 航点文件路径，Launch 会覆盖 |
| `route_frame` | `map` | 航点坐标系 |
| `odom_topic` | `/odometry/fused` | 最终导航里程计输入 |
| `odom_child_frame` | `base_link` | 最终导航里程计必须声明的车体坐标系 |
| `fusion_status_topic` | `/odometry/fusion_status` | 融合健康状态输入 |
| `actuator_health_topic` | `/cup_car_serial/actuator_healthy` | 执行器健康输入 |
| `tracking_state_topic` | `/tracking_state` | ORB 跟踪状态输入 |
| `cmd_vel_topic` | `/cmd_vel_nav` | 导航速度输出 |
| `route_input_topic` | `/waypoint_navigation/route_input` | 动态路线输入 |
| `path_topic` | `/waypoint_path` | 航点路径显示话题 |
| `status_topic` | `/waypoint_navigation/status` | 状态输出 |
| `current_waypoint_topic` | `/waypoint_navigation/current_waypoint` | 航点索引输出 |
| `motion_hold_state_topic` | `/waypoint_navigation/motion_hold_state` | 任务驻停状态输出 |
| `route_ack_topic` | `/waypoint_navigation/route_ack` | 动态路线确认输出 |

### 14.2 控制参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `control_frequency` | `30.0` | 控制频率，Hz |
| `default_speed` | `0.15` | CSV 未填写速度时的默认速度，m/s |
| `max_linear_speed` | `0.15` | 最终速度发布口强制执行的全局最大线速度，m/s |
| `max_angular_speed` | `0.75` | 最大角速度，rad/s |
| `effective_braking_deceleration` | `0.35` | 实车有效制动减速度，m/s^2 |
| `braking_control_delay` | `0.20` | 串口、控制与执行总延迟估计，s |
| `braking_safety_margin` | `0.015` | 航点前预留的制动距离，m |
| `braking_distance_feedback_gain` | `1.00` | 实际停止距离超出剩余距离时的速度压低增益 |
| `angular_gain` | `1.40` | 终点朝向对齐的比例增益 |
| `path_pid_kp` | `1.20` | 路径误差 PID 的比例增益 |
| `path_pid_ki` | `0.08` | 路径误差 PID 的积分增益，用于补偿持续机械跑偏 |
| `path_pid_kd` | `0.00` | 路径误差 PID 的微分增益 |
| `path_yaw_rate_damping` | `0.45` | 实际角速度阻尼，用于抑制横向纠偏过冲 |
| `cross_track_gain` | `1.50` | 横向偏差到航向误差的换算增益，rad/m |
| `pre_turn_path_angular_ratio` | `1.20` | 停车拐点进站段角速度相对线速度的最大比例，rad/m |
| `path_pid_integral_limit` | `0.20` | PID 积分限幅，避免定位跳变后持续过度转向 |
| `path_progress_command_threshold` | `0.08` | 触发完全不动检查的最小线速度命令，m/s |
| `path_progress_stationary_speed_threshold` | `0.02` | 认定实测近似静止的速度阈值，m/s |
| `path_progress_no_motion_timeout` | `3.0` | 持续命令但无位移时的恢复超时，s |
| `path_progress_no_motion_displacement` | `0.05` | 无运动检查窗口内允许的最大位移，m |
| `rotate_in_place_threshold` | `0.18` | 超过该航向误差时禁止前进，rad |
| `precision_turn_threshold` | `0.45` | 原地转向进入精调最低角速度区间的航向误差，rad |
| `turn_cruise_speed` | `0.75` | 原地转向巡航角速度命令，rad/s |
| `min_precision_turn_speed` | `0.50` | 精调阶段最低角速度命令，需高于实测底盘转向死区 |
| `waypoint_tolerance` | `0.04` | 动态路线未填写容差时的默认值，m |
| `waypoint_pass_longitudinal_tolerance` | `0.01` | 距终点平面的提前收点余量，m |
| `waypoint_pass_lateral_tolerance` | `0.06` | 停车点或最终点越过终点时允许收点的横向走廊，m |
| `waypoint_recovery_speed` | `0.10` | 停车点或最终点超出走廊后回收至航点的最高线速度，m/s |
| `waypoint_recovery_heading_tolerance` | `0.12` | 停车点或最终点回收时允许开始低速前进的朝向误差，rad |
| `waypoint_recovery_max_angular_speed` | `0.40` | 停车点或最终点回收的最高角速度，rad/s |
| `pre_turn_stop_heading_threshold` | `0.18` | 相邻路径转角超过此值时要求先停稳，rad |
| `pre_turn_stop_speed` | `0.03` | 判断车体停稳的纵向速度阈值，m/s |
| `pre_turn_stop_dwell` | `0.10` | 速度连续低于阈值的确认时间，s |
| `stop_motion_window` | `0.20` | 从融合位姿计算实际移动速度的窗口，s |
| `pre_turn_minimum_stop_time` | `0.20` | 收点后最短制动等待时间，s |
| `pre_turn_brake_timeout` | `1.00` | 单次停车确认时间窗；超时后保持零速并自动重新确认，s |
| `control_telemetry_timeout` | `0.30` | MCU 轮速用于停车确认的最大到达间隔，s |
| `final_yaw_tolerance` | `0.12` | 最终朝向容差，rad |
| `final_position_release_tolerance` | `0.10` | 已进入最终点后允许完成的最大位置误差，m |
| `odom_timeout` | `0.40` | 里程计到达超时时间，s |
| `imu_timeout` | `0.15` | 控制 IMU 到达超时时间，s |
| `fusion_status_timeout` | `0.60` | 融合健康状态超时时间，s |
| `actuator_health_timeout` | `0.80` | 执行器健康状态超时时间，s |

### 14.3 行为和安全参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `require_fusion_status` | `true` | 是否要求融合健康状态有效且允许 |
| `allowed_fusion_states` | 配置列表 | 允许导航的融合状态；允许的 `DEGRADED_*` 不触发额外限速 |
| `require_actuator_health` | `false` | 是否要求执行器健康；串口 bringup 强制为 `true` |
| `require_tracking_state` | `false` | 是否额外要求 ORB 跟踪状态为 `OK` 或 `OK_KLT` |
| `abort_on_tracking_loss` | `false` | 在启用跟踪状态检查时，运行中跟踪丢失是否立即中止任务 |
| `autostart` | `false` | 节点启动后是否自动请求导航 |

该表描述节点参数。实车主入口中，无参数 `start_visual_navigation.sh` 保持其关闭并等待规划 GUI；只有显式 `--autostart` 才加载根目录 `scripts/waypoints.csv` 并开启它。

---

## 15. 参数调节建议

### 15.1 车辆转向过慢

适当增大：

```yaml
angular_gain
```

或：

```yaml
max_angular_speed
```

如果出现左右摆动，说明增益可能过大。

### 15.2 车辆转弯时向外冲

可以：

- 检查 `/odometry/fused.twist.twist.linear.x` 是否能在停车后回到零附近
- 减小 `effective_braking_deceleration`，使系统更早制动
- 增大 `braking_control_delay` 或 `braking_safety_margin`
- 适当增大 `pre_turn_minimum_stop_time`

### 15.3 到达航点后振荡

可以：

- 略微增大该航点的 `tolerance`
- 检查有效制动减速度和延迟是否与实车一致
- 检查停稳速度阈值是否被融合里程计零偏持续触发
- 检查视觉位姿是否抖动

### 15.4 小角度误差时一直缓慢转动

当前中间航点不会执行目标 `yaw` 对齐；如果发生持续转动，应检查：

- 相机到 `base_link` 的外参
- 当前里程计 yaw 方向
- 左右轮控制方向
- 下位机角速度符号

### 15.5 经常触发里程计超时

先检查 `/odometry/fused` 实际频率：

```bash
ros2 topic hz /odometry/fused
```

如果计算负载较高、偶尔超过 `0.40 s`，可以谨慎增大：

```yaml
odom_timeout
```

不能为了避免报警把超时设置得过大，否则定位停止后车辆可能继续运动较长时间。

---

## 16. Launch 文件

实车不要直接拼装下面的包级 Launch。统一入口是：

```bash
# 规划模式：终端 1 录包并等待动态路线，终端 2 启动 Python 规划 GUI
./scripts/start_visual_navigation.sh
./scripts/start_arena_planner.sh

# 固定 CSV 模式：读取 scripts/waypoints.csv 并自动启动
./scripts/start_visual_navigation.sh --autostart
```

两种主入口都自动录制 rosbag，并只保留最近 3 份 bag 数据。以下 Launch 用于模块开发和模拟输入，不包含该录包策略。

### 16.1 只启动导航节点

```bash
ros2 launch visual_navigation waypoint_navigation.launch.py \
  route_file:=/absolute/path/to/route.csv \
  route_frame:=map \
  autostart:=false
```

适用场景：

- ORB-SLAM3 已经单独启动
- 使用 rosbag 或模拟节点发布 `/odometry/fused`
- 单独调试控制器

### 16.2 启动导航节点（传感器和融合由外部启动）

```bash
ros2 launch visual_navigation visual_navigation_bringup.launch.py \
  route_file:=/absolute/path/to/route.csv \
  autostart:=false
```

该 Launch 包含：

```text
waypoint_navigator
```

相机、ORB-SLAM3 和融合定位应由外部启动，例如先运行：

```bash
ros2 launch fused_odometry odometry_bringup.launch.py
```

需要串口桥接时，改用 `visual_navigation_serial_bringup.launch.py`，它在导航节点之外再启动 `cup_car_serial` 桥接节点。

---

## 17. 与下位机的接口

当前导航节点不直接打开串口或 CAN 设备，而是输出：

```text
/cmd_vel_nav
```

当前串口桥接结构：

```text
/cmd_vel_nav
      │
      ▼
cup_car_serial
      │
      └── 串口
      │
      ▼
下位机速度闭环
```

如果后续改用 CAN 或 UDP，应新增对应桥接节点并保持 `/cmd_vel_nav` 接口，不应把
当前 `cup_car_serial` 的能力描述为已经支持这些链路。

对于差速底盘，下位机可以根据轮距 `L` 和目标速度计算左右轮速度：

```text
v_left  = v - w × L / 2
v_right = v + w × L / 2
```

其中：

```text
v = linear.x
w = angular.z
```

建议下位机协议至少包含：

- 帧头
- 序号
- 使能状态
- 线速度
- 角速度
- 急停标志
- CRC 或校验和

下位机必须实现通信看门狗。建议在约 `100～300 ms` 没有收到新命令时自动停车，具体时间根据控制频率和通信链路确定。

---

## 18. 安全逻辑

代码中的已有安全措施：

1. 节点参数 `autostart` 默认关闭；统一脚本只有在显式传入 `--autostart` 时才覆盖为开启。
2. 未启动任务时持续发布零速度。
3. 未收到有效里程计或融合健康状态时不运动。
4. 里程计、融合状态或执行器健康超时时不运动。
5. 坐标系、位姿、四元数和输入测量时间戳无效时不运动。
6. 默认不强制依赖 `/tracking_state`；显式启用后可按参数选择等待恢复或锁存
   `FAULT_TRACKING_LOST`。
7. 融合硬故障、路径进度耗尽或转向进度耗尽时锁存并发布零速度；制动确认超时只保持零速并自动重试，不结束任务。
8. 调用停止、复位或任务驻停服务时立即发布零速度。
9. 节点析构时发布零速度。

仍需要在系统其他层实现：

- 下位机通信看门狗
- 物理急停开关
- 电机驱动故障保护
- 最大轮速限制
- 加速度限制
- 人员和动态障碍物检测

没有避障功能不等于可以省略急停和底层超时停车。

---

## 19. 调试方法

### 19.1 确认定位输入

```bash
ros2 topic echo /tracking_state
ros2 topic echo /odometry/fused
ros2 topic hz /odometry/fused
```

### 19.2 确认路线加载

```bash
ros2 topic echo /waypoint_path --once
```

也可以在 RViz 中添加：

```text
Path -> /waypoint_path
```

### 19.3 查看导航状态

```bash
ros2 topic echo /waypoint_navigation/status
```

### 19.4 查看当前航点

```bash
ros2 topic echo /waypoint_navigation/current_waypoint
```

### 19.5 在不接下位机时查看速度

```bash
ros2 topic echo /cmd_vel_nav
```

### 19.6 手动停止

```bash
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

实车测试时应保持另一个终端随时可以调用停止服务，并保留物理急停。

---

## 20. 当前实现的限制

### 20.1 没有避障

车辆会按照预设航点运动，不知道路线中是否出现人员或临时障碍物。

### 20.2 没有自动路径搜索

节点不会根据任意起点和终点生成新路线。CSV 中的航点顺序就是最终执行顺序。

### 20.3 没有 TF 坐标转换

航点坐标和里程计坐标必须完全一致。

### 20.4 有速度加速度斜坡

输出速度同时受到最大线/角速度以及最大加速度、减速度限制；航点接近阶段还会依据制动模型提前降速。下位机仍应实现独立的速度闭环和通信看门狗，不能把 ROS 层斜坡当作唯一安全措施。

### 20.5 不支持倒车规划

控制器只输出非负线速度，通过原地转向后向前行驶到目标点。

### 20.6 不是连续曲线跟踪器

当前控制器逐个跟踪离散航点。航点太稀疏时可能切弯，航点太密集且容差太小时可能频繁切换或振荡。

### 20.7 CSV 路线不会自动热加载

启动参数中的 `route_file` 只在节点启动时读取，修改对应 CSV 后需要重启节点或重新启动该 Launch。运行中可通过 `/waypoint_navigation/route_input` 发布合法的 `nav_msgs/msg/Path` 动态替换路线，路线编辑器使用的就是这个接口。

### 20.8 单线程执行假设

当前 `main()` 使用普通 `rclcpp::spin()`，回调在单线程执行器中串行运行，因此状态变量未加互斥锁。

如果后续改为 `MultiThreadedExecutor`，需要为里程计回调、服务回调和控制定时器共享的数据增加互斥保护，或使用互斥回调组。

---

## 21. 推荐后续扩展

### 第一阶段：下位机桥接

现有实现 `cup_car_serial` 已完成 `/cmd_vel_nav` 到串口协议的转换，并接收下位机
状态。若需要 CAN 或 UDP，应新增对应桥接节点并保持相同的速度接口。

### 第二阶段：速度平滑

在输出前增加：

- 最大线加速度
- 最大线减速度
- 最大角加速度
- 紧急停车旁路

### 第三阶段：连续路线跟踪

将当前逐航点控制升级为 Pure Pursuit：

1. 对航点折线插值。
2. 计算车辆在路径上的投影位置。
3. 沿路径弧长寻找前视点。
4. 根据前视点曲率计算角速度。
5. 根据曲率自动降低线速度。

### 第四阶段：地图坐标对齐

增加定位适配节点，发布：

```text
map -> vio_map
```

使航点可以直接使用已有地图中的绝对坐标。

### 第五阶段：路线任务接口

增加自定义 Action，例如：

```text
FollowWaypointRoute.action
```

提供：

- 路线名称
- 当前进度
- 剩余距离
- 取消任务
- 成功或失败结果

### 第六阶段：安全感知

即使不采用 Nav2，也可以单独增加简单的急停感知节点，检测前方危险距离后覆盖 `/cmd_vel_nav` 输出。

---

## 22. 构建与安装

```bash
cd /path/to/your/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select visual_navigation
source install/setup.bash
```

构建后主要安装位置：

```text
install/visual_navigation/lib/visual_navigation/waypoint_navigator
install/visual_navigation/share/visual_navigation/config/
install/visual_navigation/share/visual_navigation/launch/
install/visual_navigation/share/visual_navigation/routes/
install/visual_navigation/share/visual_navigation/README.md
install/visual_navigation/share/visual_navigation/NAVIGATION_CODE_GUIDE.md
```

---

## 23. 最小运行流程

```bash
# 1. 终端 1：完整导航栈、运行清单和 rosbag
./scripts/start_visual_navigation.sh

# 2. 终端 2：规划后端和 Python GUI，确认后在 GUI 点击“开始导航”
./scripts/start_arena_planner.sh

# 固定路线测试改用下面一条，并且不要启动规划 GUI：
# ./scripts/start_visual_navigation.sh --autostart

# 3. 确认 ORB-SLAM3 定位有效
ros2 topic echo /tracking_state

# 4. 确认路线正确
ros2 topic echo /waypoint_path --once

# 5. 观察输出
ros2 topic echo /cmd_vel_nav
ros2 topic echo /waypoint_navigation/status

# 8. 必要时停止
ros2 service call /waypoint_navigator/stop std_srvs/srv/Trigger '{}'
```

正式连接实车前，应先在不接电机或架空驱动轮的条件下验证速度方向、转向方向、坐标系和定位丢失停车逻辑。
