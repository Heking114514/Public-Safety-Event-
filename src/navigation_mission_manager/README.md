# Navigation Mission Manager

比赛实车的唯一高层任务状态拥有者。该节点不计算路径、不识别图像，也不发布速度；它只
编排现有模块：

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

动态障碍默认保留 `dynamic_obstacle_retention_s=120` 秒，保证停车、后退和近期重规划使用
同一障碍；超时未再次观测则从动态地图移除。障碍仍存在时，感知节点在车辆再次接近后
会重新上报并重新进入恢复，避免一次临时障碍永久封死后续道路覆盖。

正式任务向规划服务请求纯规划结果，任务节点确认请求仍属于当前状态后才发布
`/waypoint_navigation/route_input`。因此暂停、终止或复位期间迟到的规划响应不能发车。

## 事件播报

感知节点向 `/perception/event_candidate` 发布 `EventCandidate`。任务节点负责置信度门限、
同类同位置去重、必要时驻停，并通过 `/event_reporter/report` Action 委托独立播报器。
只有 Action 成功返回后，事件才发布到 `/navigation_mission/reported_event` 并记为已播报。

## 操作接口

```bash
ros2 service call /navigation_mission/control \
  mission_control_interfaces/srv/ControlMission \
  '{command: 1, target_ids: []}'
```

命令值：`1 START`、`2 PAUSE`、`3 RESUME`、`4 ABORT`、`5 RESET`。完整状态通过
`/navigation_mission/state` 持久发布。
