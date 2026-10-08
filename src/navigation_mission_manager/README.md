# Navigation Mission Manager

比赛实车的唯一高层任务状态拥有者。该节点不计算路径、不识别图像，也不发布速度；它只
编排现有模块：

运行节点和任务核心均为 C++17；launch 文件保留 ROS 2 Python launch 格式。安装后的
可执行文件仍为 `mission_manager_node`，现有启动命令和 ROS 接口不变。

```text
感知候选 / 障碍观测 / 融合位姿
                 |
                 v
navigation_mission_manager
  三阶段进度、路线 ACK、故障恢复、事件去重与播报确认
        |                         |
        v                         v
new_map_planner             event_reporter
        |
        v
waypoint_navigator -> /cmd_vel_nav
```

## 自主导航

正式任务固定为：

1. `layer1`：访问任务点。
2. `layer2`：显式穿越四条隧道。
3. `layer3`：补齐普通道路并返回发车点。

路线必须收到匹配的 `/waypoint_navigation/route_ack`，并在真正执行后收到
`GOAL_REACHED`，任务节点才提交该段进度。控制器报告
`OBSTACLE_REPLAN_REQUIRED` 时，任务节点使用已经确认的障碍多边形和最新融合位姿自动
重规划当前阶段。若规划器明确确认当前恢复姿态无路，任务节点通过
`/waypoint_navigator/continue_obstacle_retreat` 请求控制器沿已执行轨迹再倒退
`retreat_step_m`；控制器反向执行实际限速命令并在新锚点校验姿态，任务节点等其再次
报告 `OBSTACLE_REPLAN_REQUIRED` 后才规划。默认最多扩展
`max_retreat_extensions` 次；轨迹耗尽、控制器拒绝或达到次数上限才进入 `FAULT` 并驻停。
任务节点始终不发布 `/cmd_vel`，后退跟踪与安全监督仍完全属于控制器。

完成一段路线后若下一段仅因几何衔接无路，总控改用
`/waypoint_navigator/continue_route_retreat`，并保持 `REPLANNING`；控制器发布
`ROUTE_TRANSITION_*` 状态。该流程复用同一条已执行轨迹和安全监督，但不属于障碍恢复，
不会产生 `OBSTACLE_BRAKING`。只有已经确认动态障碍的重规划才使用上述障碍接口和状态。

动态障碍默认保留 `dynamic_obstacle_retention_s=120` 秒，保证停车、后退和近期重规划使用
同一障碍；超时未再次观测则从动态地图移除。障碍仍存在时，感知节点在车辆再次接近后
会重新上报并重新进入恢复，避免一次临时障碍永久封死后续道路覆盖。

正式任务向规划服务请求纯规划结果，任务节点确认请求仍属于当前状态后才发布
`/waypoint_navigation/route_input`。因此暂停、终止或复位期间迟到的规划响应不能发车。

## 事件播报

感知节点向 `/perception/event_candidate` 发布 `EventCandidate`。任务节点负责置信度门限、
同类同位置去重、必要时驻停，并通过 `/event_reporter/report` Action 委托独立播报器。
只有 Action 成功返回后，事件才发布到 `/navigation_mission/reported_event` 并记为已播报。

## RFID 巡逻点

`arena.yaml` 中的 `rfid_supernodes` 将固定物理位置 `s1`–`s12` 映射到地图任务点 `1`–`12`。
读卡器返回的 1–12 卡号是到点事件数据，不代表地图点编号；任何有效卡号只会关联到当前路线
预期的超节点，并且融合位姿必须位于该点的接近范围内。

串口桥只发布 `/cup_car_serial/rfid_number`。导航器到达超节点后沿来向以 `0.02 m/s` 低速
扫描，最多越过地图节点 `0.20 m` 并停等读卡；里程计本身不能确认节点、推进路线或触发转弯。
读到卡后任务总控申请 `navigation_mission_manager/rfid` 驻停，等待驻停生效，向
`/fused_odometry/correct_map_position` 请求校正，并等待融合位姿进入目标点 `0.025 m` 范围且
稳定 `0.20 s`。非播报点驻停 `1 s`，播报点沿用播报和原有驻停时间。最后总控通过导航器
`/waypoint_navigator/confirm_rfid_checkpoint` 服务放行指定路线号、航点序号和超节点 ID。
卡号重复按当前物理点去重，因此相同卡号可在不同物理超节点再次使用。
规划器的 `visit_order` 只描述预计访问顺序，不再直接作为任务点完成凭据。
实车默认使用 `task_confirmation_mode: rfid`，因此只有真实 RFID 事件可以确认任务点。
loopback 显式使用 `task_confirmation_mode: route`，在路线已 ACK、实际执行并收到
`GOAL_REACHED` 后，才使用该路线的 `visit_order` 确认仿真任务点。

其他节点可以向 `/navigation_mission/speech_text` 发布 `std_msgs/String`。总控会排队文本，
申请 `navigation_mission_manager/speech` 驻停，调用 `/cup_car_serial/speak_text` 下发给
下位机，播报完成后驻停 3 秒再释放；RFID 播报使用同一个下发链路。

其他节点需要控制车辆驻停或请求障碍倒退时，应调用总控入口：
`/navigation_mission/set_motion_hold` 和
`/navigation_mission/continue_obstacle_retreat`。总控会校验外部来源并转发给导航器，
外部节点不应直接调用 `/waypoint_navigator` 下的底层服务。

## 操作接口

```bash
ros2 service call /navigation_mission/control \
  mission_control_interfaces/srv/ControlMission \
  '{command: 1, target_ids: []}'
```

命令值：`1 START`、`2 PAUSE`、`3 RESUME`、`4 ABORT`、`5 RESET`。完整状态通过
`/navigation_mission/state` 持久发布。
