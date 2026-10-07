# Changelog

## 2026-10-07

- `layer1` now plans the next pending task point first, falling back to another
  reachable task only when that target has no legal route. Graph search tracks
  arrival heading and the number of consecutive ordinary junctions; the count
  is returned with each route and carried into the next planning request.
- 部分覆盖路线必须至少完成一条当前阶段要求的道路；仅执行厘米级起点吸附、所有阶段
  目标仍 deferred 的零进展路线不再发布。
- 新增 `z=0.002` 反向进站转弯语义；规划器与控制器共用状态格核心，连续验证“探入
  边缘道路、倒车退回路口、转入下一道路”的完整车身扫掠。

## 2026-10-06

- 路口附近的起点吸附段若需要先倒车或横移，不再伪装成普通前进 `Path` 并返回成功；
  规划器明确拒绝当前恢复姿态，让任务层继续退到可执行的更早锚点。

## 0.2.0 - 2026-10-05

- 新增 `layer1` 任务点、`layer2` 隧道和 `layer3` 普通道路补漏回程三阶段规划策略。
- 新增独立 `StagedMission` 任务状态模块，路线只有在 ACK 和 `GOAL_REACHED` 后才提交；
  成功段自动衔接，故障段暂停等待操作员恢复。
- 实车 UI 增加阶段、剩余任务/隧道/道路状态和“继续巡视”入口。
- 实车 UI 从任务状态拥有者降级为操作台，完整巡视通过
  `navigation_mission_manager` 命令和状态接口运行，关闭 UI 不影响后台任务。
- loopback 默认执行三阶段全链路，并从每阶段最新真值位姿续规划。
- 三阶段闭环已用正式 `waypoint_navigator` 跑到 `mission_complete:stages=3`，三段长度分别
  为 `13.70 m`、`9.04 m`、`15.74 m`，最终回程位置误差约 `38 mm`。

## 0.1.0 - 2026-10-04

- 新增道路图主规划、航向状态搜索和独立车身碰撞验证层。
- 新增显式 `Maneuver`、`RoutePlan` 与 `PlanRoute` ROS 接口。
- 正式 12 点任务使用精确集合搜索；道路覆盖保留延期结果。
- 动态障碍改为请求级多边形验证，不重建静态栅格。
- 新增仅面向实车的 Tk 操作界面、路线 ACK 监控和导航驻停。
- 将前方 Bool/Range 观测按导航恢复结果提升为请求级动态障碍，并在 UI 显示和清除。
