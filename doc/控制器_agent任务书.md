# 控制器 agent 任务书：转弯动作拥有者重构

记录日期：2026-10-10（北京时间）  
负责功能包：`visual_navigation`
任务边界：只修改控制器动作封装、控制器测试和必要的中文 changelog；不修改 `new_map_planner`、仿真器、任务管理器和 RFID 业务。

## 目标

将现有 `TurnManeuverController` 收敛为转弯动作的唯一拥有者。保留已经完成的 `geometry_heading` / `arrival_heading` 分离，不再在 `waypoint_navigator` 中追加独立的 `TURN_REPLAN`、trim 或 arc 决策分支。

## 必须实现

1. 定义或整理动作输入 `TurnRequest`：路口 ID/位置、入口几何方向、实际到达方向、出口方向、reverse 标记、车体参数和当前 pose。
2. 动作启动时冻结几何上下文；后续只更新实测 pose、速度和 IMU。
3. 把入口检查、trim、arc、exit align 和 footprint 预测放入动作控制器调用链。
4. 对安全拒绝记录原因和最后安全 pose；同一输入无实际改善时不得反复发送相同 trim。
5. 统一返回 `RUNNING`、`COMPLETED`、`NEED_REPLAN`、`FAULT` 四类结果。
6. `NEED_REPLAN` 必须携带：最后安全 pose、junction ID、geometry heading、arrival heading、target heading、失败原因和建议恢复方向。
7. `waypoint_navigator` 只负责启动、暂停、取消、发布命令和处理结果；动作内部状态不能由导航器重复判断。

## 不得修改

- 不放宽 `footprint_outside_envelope`；
- 不删除道路边界、起点碰撞或转弯安全检查；
- 不给普通节点增加无条件倒车或原地掉头；
- 不修改 RFID 卡识别、任务顺序和全局规划搜索；
- 不修改 `new_map_planner` 的消息和接口；规划器接口需求写入规划器任务书，由主 agent 统一合并。

## 测试要求

至少增加隔离测试覆盖：

- 正常居中入口完成转弯；
- 提前约 2 cm 停车后通过 trim 完成；
- arrival heading 存在残差但 geometry heading 不变；
- 第一次 arc 被拒后仍有改善，动作继续；
- 连续相同拒绝且无改善，只返回一次 `NEED_REPLAN`；
- 当前实测 pose 不安全时返回 `FAULT` 并输出零速；
- `NEED_REPLAN` 之后不会再次进入动作内部 `ARC`。

现有普通转弯、障碍恢复和 launch contract 测试必须保持通过。完成后报告修改文件、测试命令和未解决的动态问题，不直接启动实车。

