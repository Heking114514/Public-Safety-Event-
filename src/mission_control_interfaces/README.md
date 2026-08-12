# mission_control_interfaces

任务控制的可复用 ROS 2 接口包。接口定义与任务识别、播报实现解耦，当前提供多来源运动驻停能力。

## 接口

- `srv/SetMotionHold.srv`：按 `source` 申请或释放运动驻停。
- `msg/MotionHoldState.msg`：发布当前驻停状态、来源和原因。
- `msg/WaypointRoute.msg`：发布包含速度、容差和停车时间的完整规划路线。

服务由 `visual_navigation/waypoint_navigator` 实现：

```text
/waypoint_navigator/set_motion_hold
```

一个来源只能释放自己的驻停请求；所有来源均释放后，路线导航才会恢复。
