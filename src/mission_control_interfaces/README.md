# mission_control_interfaces

共享的任务控制接口：

- `ControlTelemetry.msg`：STM32控制状态、命令回显、轮速目标/实测值和PWM。
- `MotionHoldState.msg`：导航驻停状态与来源。
- `SetMotionHold.srv`：按来源申请或释放导航驻停。
- `SpeakText.srv`：向下位机车载语音模块发送一次 UTF-8 播报文本。
- `CorrectMapPosition.srv`：总控在 RFID 驻停期间请求融合节点修正 map 坐标，仅修正位置。
- `ConfirmRfidCheckpoint.srv`：总控在读卡、位置校正和驻停流程完成后，放行指定路线中的 RFID 超节点。
- `ContinueRetreat.srv`：按已执行轨迹请求下一段障碍恢复倒车。

任务控制的可复用 ROS 2 接口包。接口定义与规划、识别、播报实现解耦。

## 接口

- `srv/SetMotionHold.srv`：按 `source` 申请或释放运动驻停。
- `srv/ControlMission.srv`：开始、暂停、继续、终止或复位自主任务。
- `srv/ContinueRetreat.srv`：当前恢复姿态无路时，请求控制器沿已执行轨迹继续倒退。
- `msg/MissionState.msg`：统一发布任务阶段、路线和剩余工作状态。
- `msg/EventCandidate.msg`：感知节点提交带位置、置信度和证据引用的事件候选。
- `action/ReportEvent.action`：任务节点委托独立播报器执行一次可确认的事件播报。
- `msg/MotionHoldState.msg`：发布当前驻停状态、来源和原因。

外部节点通过总控调用：

```text
/navigation_mission/set_motion_hold
/navigation_mission/continue_obstacle_retreat
```

以下服务由 `final_navigation` 实现，供总控内部调用：

```text
/final_navigation/set_motion_hold
/final_navigation/continue_obstacle_retreat
/final_navigation/continue_route_retreat
/final_navigation/confirm_rfid_checkpoint
```

一个来源只能释放自己的驻停请求；所有来源均释放后，路线导航才会恢复。
继续倒退服务只在控制器已经到达恢复锚点并等待新路线时接受请求；它不接收速度命令，
也不会越过控制器保留的实际行驶轨迹。
