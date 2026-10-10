# 规划器 agent 任务书：恢复起点契约

记录日期：2026-10-10（北京时间）  
负责功能包：`new_map_planner`
任务边界：只修改规划器请求/起点种子/恢复路线和对应测试、中文 changelog；不修改 `visual_navigation` 的动作状态机、仿真器和底盘控制。

## 目标

补齐规划器与转弯/障碍恢复之间的起点契约。普通规划行为保持原样；只有显式恢复模式才允许规划器考虑沿来向道路退离路口。

## 接口口径

规划请求增加可选恢复语义，推荐字段：

- `start_mode`: `NORMAL`、`TURN_RECOVERY`、`OBSTACLE_RECOVERY`；
- `recovery_anchor` 或 `anchor_node_id`；
- `allow_reverse_departure`；
- `incoming_lane_id`（可选）；
- 当前起点 pose。

如果现有接口消息不能立即扩展，先在规划器内部建立等价的 request/domain 类型和转换层，不要把恢复语义编码到 yaw、Z 值或特殊坐标中。

## 必须实现

1. `NORMAL` 模式继续执行现有起点 footprint、贴路、航向、正向段碰撞检查。
2. `TURN_RECOVERY` 只允许从控制器提供的最后安全 pose/路口上下文生成退离路线。
3. 恢复路线必须经过同一车辆 footprint、道路边界和动态障碍检查。
4. `OBSTACLE_RECOVERY` 不替代现有历史轨迹回放；只有控制器确认抵达恢复锚点后，才转入普通规划。
5. 失败原因结构化返回，至少区分：起点碰撞、起点不在道路、航向不匹配、正向被阻挡、恢复方向不可行。
6. 规划器输出路线时继续保留现有转弯几何航向修复，不能将 arrival heading 当作 geometry heading。

## 不得修改

- 不降低 `lane_snap_distance`、`start_heading_tolerance` 或 footprint 安全标准来让测试通过；
- 不让 `NORMAL` 模式自动使用倒车起步；
- 不接管 `TurnManeuverController` 的入口、arc、trim 或出口控制；
- 不修改仿真器障碍物模型；
- 不在本任务中迁移 Z 轴标记协议。显式 waypoint metadata 作为后续独立任务，先保留兼容发布。

## 测试要求

至少增加规划器测试覆盖：

- NORMAL 起点行为与当前基线一致；
- TURN_RECOVERY 在车头朝前、正向段被阻挡但来向道路可退时生成受限恢复路线；
- 恢复路线遇到 footprint 或动态障碍时正确拒绝；
- 无 anchor 或恢复上下文不完整时拒绝，而不是猜测方向；
- 规划器输出转弯 waypoint 的 geometry heading 不受 arrival residual 影响；
- 失败原因能被任务管理器区分，不统一伪装成 `START_POSE_COLLISION`。

完成后报告修改文件、接口变化、测试命令和与控制器 agent 的待合并契约；不得直接修改控制器代码。

