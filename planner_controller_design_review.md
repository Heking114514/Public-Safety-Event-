# 规划器、控制器与避障恢复设计审查

更新时间：2026-10-12

本文记录当前代码已经确认的事实，并给出基于“直行 + 固定转弯”运动模型的规划器重构建议。本文只做设计审查，不代表代码已经按建议修改。

## 1. 当前系统的真实分工

当前链路是：

```text
vehicle_loopback_sim 发布 /odometry/fused
        |
        v
navigation_mission_manager 获取当前位姿和任务进度
        |
        v
调用 /new_map_planner/plan
        |
        v
new_map_planner 返回 RoutePlan / navigation_path
        |
        v
任务总控发布 /waypoint_navigation/route_input
        |
        v
visual_navigation ACK、跟踪直行、执行固定转弯
        |
        v
发布 /cmd_vel_nav
```

### new_map_planner

当前规划器同时做了四件事：

- 在道路图上搜索目标点和道路顺序；
- 维护航向、连续普通节点数、任务阶段和回家约束；
- 使用车体 footprint 检查直行段和固定转弯扫掠；
- 编译 `RoutePlan`，并通过 `nav_msgs/Path` 兼容生产控制器。

规划器的动作层包含 `DRIVE`、`TURN_LEFT`、`TURN_RIGHT`、`REVERSE`、`FINISH`。完整说明见 [src/new_map_planner/README.md](src/new_map_planner/README.md)。

### visual_navigation

生产控制器接收的是 `nav_msgs/Path`，而不是按 `RoutePlan.maneuvers` 逐项执行。它从路径点的 `z` 值解析出：

- 普通转弯点；
- 倒车到达转弯点；
- RFID 超节点；
- RFID 转弯超节点。

解析逻辑在 [src/visual_navigation/include/visual_navigation/route_manager.hpp](src/visual_navigation/include/visual_navigation/route_manager.hpp)。

正常巡航转弯由现有固定状态机完成：

```text
FOLLOW
  -> BRAKE
  -> TURN_CAPTURE
  -> APPROACH_ENTRY
  -> TURN_60
  -> DIAGONAL_CREEP
  -> TURN_30
  -> PATH_ALIGNED
  -> FOLLOW
```

实现位于 [src/visual_navigation/src/turn_maneuver.cpp](src/visual_navigation/src/turn_maneuver.cpp)，对应参数在 [src/visual_navigation/src/node_init.cpp](src/visual_navigation/src/node_init.cpp)。这套正常巡航转弯目前执行稳定，后续改造必须保持其参数、阶段顺序和控制逻辑不变。

### navigation_mission_manager

总控负责：

- `layer1 -> layer2 -> layer3` 三阶段任务；
- 路线 ACK 和 `GOAL_REACHED`；
- 超节点停车、位置修正、第一次播报和保持；
- 动态障碍记录；
- 障碍恢复完成后重新规划。

任务提交和三层进度在 [src/navigation_mission_manager/src/mission.cpp](src/navigation_mission_manager/src/mission.cpp) 和 [src/navigation_mission_manager/src/planning.cpp](src/navigation_mission_manager/src/planning.cpp) 中实现。

## 2. 超节点规则已经存在

地图通过 `rfid_supernodes` 将任务点绑定为超节点。规划器会把超节点元数据写入与路径点对齐的 `waypoint_supernode_ids`，再通过路径 `z` 值传给控制器。

总控识别预期超节点后，现有标准是：

```text
识别预期 RFID
  -> 请求控制器保持
  -> 确认车辆已经停止并稳定
  -> 调用位置修正服务
  -> 等待位置误差收敛
  -> layer1 首次任务点播报
  -> 保持一秒
  -> 授权控制器继续
  -> 标记超节点完成
```

具体 RFID phase 是：

```text
idle
waiting_hold
correcting
waiting_correction
announcing
post_speech
authorizing
```

代码在 [src/navigation_mission_manager/src/rfid.cpp](src/navigation_mission_manager/src/rfid.cpp)。因此后续设计直接复用这套标准，不新增第二套“超节点完成”判据。

超节点完成后，普通节点计数清零。正常通过普通节点会增加计数，恢复完成后从零重新开始。当前规划请求和结果已经带有 `ordinary_nodes_since_task`，见 [src/new_map_planner/include/new_map_planner/types.hpp](src/new_map_planner/include/new_map_planner/types.hpp)。

## 3. 当前动态障碍接口的实际情况

地图节点本身没有单独的障碍物字段。`arena.yaml` 中目前分别定义：

```text
free_regions
static obstacles
nodes
lanes
tasks
rfid_supernodes
```

动态障碍是运行时数据，不是某个地图节点的属性。

现有接口是：

```text
/perception/dynamic_obstacle
/obstacle/front_blocked
/obstacle/front_range
```

总控订阅动态障碍多边形，并把它们放进 `PlanRoute` 请求的 `dynamic_obstacles`。相关代码在 [src/navigation_mission_manager/src/node.cpp](src/navigation_mission_manager/src/node.cpp)、[src/navigation_mission_manager/src/planning.cpp](src/navigation_mission_manager/src/planning.cpp) 和 [src/new_map_planner/include/new_map_planner/types.hpp](src/new_map_planner/include/new_map_planner/types.hpp)。

因此当前是：

```text
感知输出多边形
  -> 总控保存动态障碍
  -> 总控把多边形传给规划器
  -> 规划器用 footprint 和 sweep 做几何否决
```

建议改成：

```text
感知输出距离/多边形
  -> 总控根据当前位置推断阻塞边
  -> 保存 blocked_lane_id + blocked_direction
  -> 规划器只把该边或方向视为暂时不可用
```

障碍不会出现在路口或堵死路口，因此不需要设计路口内部避障拓扑；但需要保留边方向信息，避免把只挡一侧的障碍错误地扩展成整条道路永久封闭。

## 4. 当前路径规划算法是否有问题

有问题，但核心问题不是“搜索算法完全错误”，而是当前算法承担了超出实际车辆模型需要的职责。

### 4.1 几何防撞约束和实际执行模型重复且不一致

规划器在 [src/new_map_planner/src/maneuver.cpp](src/new_map_planner/src/maneuver.cpp) 中硬编码了一份转弯 profile，然后对这份 profile 做车体 sweep 检查。

生产控制器在 [src/visual_navigation/src/turn_maneuver.cpp](src/visual_navigation/src/turn_maneuver.cpp) 中又独立实现了一份固定转弯状态机。两边的结构和关键参数基本一致，但不是共享代码。

结果是：

```text
规划器验证一份转弯轨迹
控制器执行另一份转弯状态机
```

只要两边的入口位置、斜向爬行距离、角度阈值、base_link 偏移或退出条件发生漂移，规划器验证的动作就不再是控制器实际执行的动作。

### 4.2 在线规划被 footprint 和 sweep 门控绑住

规划器当前会检查：

- 起点 footprint 是否碰撞；
- 起点到道路入口的前向线段；
- 每一条候选直行边；
- 每一个候选转弯的扫掠姿态；
- 动态障碍多边形与车体的几何关系。

这些检查对于通用移动机器人是合理的，但当前小车只有直行和已经锁死的转弯动作，位姿也有较准确的闭环反馈。在线规划继续搜索这些连续几何细节，既增加失败点，也没有给生产控制器增加对应的实时包络保护。

### 4.3 当前覆盖规划是带覆盖集合的回溯搜索，不保证最短

2026-10-12 重新读取当前实现后更正：当前 `coverage.cpp` 已使用带访问集合的深度优先回溯搜索，不是逐次选择最近未覆盖道路的贪心算法。搜索状态包括覆盖边位掩码、节点、航向、普通节点数和到达边；候选优先选择新增覆盖边，找到首条完整路线即返回。最多展开 2,000,000 个状态。

它会搜索完整覆盖，并在需要时要求终点回家；部分路线要求有新增覆盖、有前向后继，并在要求回家时检查可达性。它不以累计路程作为最优性目标，因此不能宣称最短；达到搜索上限也不能解释为拓扑不可达。

代码在 [coverage.cpp](<src/new_map_planner/src/coverage.cpp>)。此前依据旧说明写出的“确定性贪心”判断不适用于当前实现。

另外，当前目标搜索和覆盖搜索初始化普通起点时均使用 `max(1, ordinary_nodes_since_task)`。恢复后传入 0 仍会变成 1，需要在新设计中区分“已经计过的起点”和“接入后新到达的普通节点”。


### 4.4 动态障碍接口层级偏低

当前规划器收到的是多边形，却没有直接收到“哪一条边、哪个方向暂时阻塞”的业务语义。规划器被迫重新做几何判断。

在当前传感器能力和恢复规则下，更合适的接口是：

```text
blocked_lane_id
obstacle_revision
observation_time
confidence
```

动态障碍推断属于总控/感知融合职责，规划器只需要在道路图上避开被禁用的边。

### 4.5 路线动作语义和实际执行接口不完全一致

规划器可以生成 `Maneuver`，但生产链最终只发布 `navigation_path`。总控没有把 `RoutePlan.maneuvers` 逐项发送给控制器执行。

正常转弯目前靠路径点 `z` 标记触发，这对固定巡航转弯是可行的；但如果未来把复杂避障恢复也塞进普通 `Path`，语义会再次变得隐含。因此避障恢复需要单独的控制器服务或恢复命令，不应通过普通路线点猜测。

### 4.6 当前规划器的结论

当前算法可以在现有系统上工作，但不适合继续向“更复杂的在线几何规划”发展。

建议把它调整为：

```text
静态地图/固定动作：离线验证
在线规划器：道路图和任务状态搜索
生产控制器：固定转弯和避障恢复执行
总控：动态障碍边状态、阶段进度和恢复协调
```

## 5. 建议的状态机

### 5.1 正常巡航控制器状态机

这部分完全保留现状，不修改转弯效果：

```text
FOLLOW
  |
  | 到达需要停车的路口/终点
  v
BRAKE
  |
  v
TURN_CAPTURE
  |
  v
APPROACH_ENTRY
  |
  v
TURN_60
  |
  v
DIAGONAL_CREEP
  |
  v
TURN_30
  |
  v
PATH_ALIGNED
  |
  v
FOLLOW
```

超节点扫描从正常路线状态中插入，但不改变转弯动作：

```text
FOLLOW -> RFID_SCANNING / WAITING_FOR_RFID -> FOLLOW
```

### 5.2 总控状态机

```text
IDLE
  -> WAIT_READY
  -> PLANNING
  -> WAIT_ROUTE_ACK
  -> EXECUTING
  -> SUPER_NODE_SERVICE
  -> EXECUTING
  -> ROUTE_COMPLETED
  -> PLANNING
```

遇到障碍时：

```text
EXECUTING
  -> OBSTACLE_PENDING
  -> OBSTACLE_BRAKING
  -> WAIT_RECOVERY_READY
  -> PLANNING
  -> WAIT_ROUTE_ACK
  -> EXECUTING
```

如果障碍发生在已经开始的正常转弯中：

```text
TURN_ACTIVE
  -> FINISH_CURRENT_TURN
  -> TURN_EXIT_RECOVERY
  -> OBSTACLE_BRAKING
  -> WAIT_RECOVERY_READY
  -> PLANNING
```

这里 `FINISH_CURRENT_TURN` 只表示继续使用当前已经验证过的控制器转弯状态机，不新增另一套转弯算法。

### 5.3 避障恢复控制器状态机

#### 丁字路口掉头

```text
RECOVERY_SELECT_T
  -> RETREAT_TO_TURN_POINT_A
  -> RECOVERY_TURN_1
  -> DRIVE_TO_TURN_POINT_B
  -> RECOVERY_TURN_2
  -> RESUME_FORWARD
  -> EXECUTING
```

#### 丁字路口 90 度转入另一条道路

```text
RECOVERY_SELECT_T
  -> RETREAT_ACROSS_JUNCTION
  -> RECOVERY_TURN
  -> RESUME_FORWARD
  -> EXECUTING
```

#### 十字路口

```text
RECOVERY_SELECT_CROSS
  -> 选择三个恢复方向之一
  -> 调用对应丁字模板
  -> RESUME_FORWARD
  -> EXECUTING
```

#### 恢复完成契约

恢复成功后，控制器向总控报告：

```text
recovery_completed = true
resume_node
resume_lane
resume_heading
ordinary_nodes_since_task = 0
```

当前代码已经有恢复锚点和 `OBSTACLE_REPLAN_REQUIRED` 相关路径，见 [src/visual_navigation/src/obstacle_recovery.cpp](src/visual_navigation/src/obstacle_recovery.cpp)。后续应把现有“沿执行轨迹退回锚点并请求重规划”扩展成上述固定恢复模板，但不要触碰正常巡航转弯状态机。

## 6. 各包职责

| 包 | 保留职责 | 应移出的职责 |
|---|---|---|
| `new_map_planner` | 任务阶段、目标顺序、道路顺序、航向、普通节点计数、阻塞边过滤、转弯语义 | 在线 footprint/sweep 门控、长距离倒车恢复、转弯恢复轨迹生成 |
| `visual_navigation` | 直行跟踪、正常刹车、现有固定转弯、RFID 扫描、速度限制 | 无需移除正常转弯；新增固定避障恢复模板 |
| `navigation_mission_manager` | 三阶段进度、超节点流程、障碍边推断、恢复协调、路线 ACK、阶段切换 | 不直接执行车辆运动 |
| `navigation_safety` | 离线地图验证、固定动作回归、仿真器保护 | 不再作为在线规划的主要门控 |
| `vehicle_loopback_sim` | 真值位姿、传感器模拟、可选车体碰撞验收 | 不参与正式规划决策 |

## 7. 可视化方案

现有仓库已经有两种规划器可视化入口：

- [scripts/visualize_planner.py](scripts/visualize_planner.py)：离线路线和障碍分析；
- [scripts/planner_visualizer.html](scripts/planner_visualizer.html)：浏览器交互地图和路线播放。

当前 HTML 页面主要显示质点路径，且不会绘制 `arena.yaml` 的固定障碍物。为了看清新规划器是否符合规则，可视化分为四层。

### 7.1 地图和图层

画面显示：

- 静态道路和路口；
- 超节点用大圆标识，并显示 `s1...s12` 和任务编号；
- 普通节点用小圆标识；
- 每条边显示 lane id、方向箭头和是否隧道；
- 起点、当前航向和当前任务阶段。

颜色建议：

```text
超节点：蓝色
普通节点：灰色
隧道边：紫色
当前路线：青色
已覆盖边：绿色
动态阻塞边：红色虚线
候选但暂不选择的边：浅灰色
```

### 7.2 路线状态

路线播放时，在每个节点旁显示：

```text
ordinary_count = 0 / 1 / 2
current_stage = layer1 / layer2 / layer3
current_route_step
next_supernode
```

这样可以直接观察：

- 是否连续超过两个普通节点；
- 是否在超节点完成后清零；
- layer1、layer2、layer3 是否按预期接力；
- layer3 是否最终回到 launch。

### 7.3 固定转弯状态机

转弯时不把规划器 footprint 包络当成控制器行为，而显示控制器真实状态：

```text
FOLLOW
BRAKE
APPROACH_ENTRY
TURN_60
DIAGONAL_CREEP
TURN_30
PATH_ALIGNED
```

同时绘制：

- 当前车辆位置和航向；
- 入口目标点；
- 60 度中间目标；
- 斜向爬行段；
- 最终出口方向；
- 实际轨迹与固定动作参考轨迹的偏差。

这样能够确认“规划器只选择转弯语义，控制器仍然按原动作执行”，也能确认正常巡航转弯没有被改动。

### 7.4 避障恢复动画

在障碍模式下，画面显示：

```text
障碍物位置
推断出的 blocked_lane_id
obstacle_revision
最近恢复路口
恢复模板名称
当前恢复状态
恢复后的 resume_node 和 resume_heading
```

例如：

```text
OBSTACLE_BRAKING
  -> RETREAT_TO_TURN_POINT_A
  -> RECOVERY_TURN_1
  -> DRIVE_TO_TURN_POINT_B
  -> RECOVERY_TURN_2
  -> RESUME_FORWARD
```

车辆轨迹使用不同颜色：

```text
正常执行轨迹：青色
倒车恢复轨迹：橙色
恢复后的新路线：绿色
阻塞边：红色虚线
```

### 7.5 输出形式

建议一次生成三种结果：

```text
1. 静态 PNG/SVG：看地图、道路、超节点和路线
2. HTML 动画：看状态机和车辆运动
3. JSON/CSV trace：逐帧记录状态、节点计数、路线和障碍边
```

命令行接口可以扩展为：

```bash
python3 scripts/visualize_planner.py \
  --mode mission \
  --show-supernodes \
  --show-ordinary-count \
  --show-blocked-edges \
  --save /tmp/mission_plan.svg
```

浏览器页面可以增加：

```text
Layer 1 / Layer 2 / Layer 3 切换
Obstacle 开关
Blocked edge 选择
Recovery template 选择
Play / Step / Reset
显示控制器状态
显示 ordinary_count
```

## 8. 改造顺序

### 第一步：先做只读可视化和状态 trace

不改变规划和控制代码，增加：

- 超节点/普通节点显示；
- 路线边方向显示；
- 普通节点计数显示；
- 动态阻塞边显示；
- 当前控制器状态显示；
- 正常转弯状态机显示。

先确认现有路线和当前控制器实际行为符合规则。

### 第二步：把动态障碍转换为阻塞边

总控保留现有障碍多边形输入，但增加边推断结果：

```text
blocked_lane_id
obstacle_revision
```

规划器搜索时只过滤对应道路方向，不再把动态多边形交给每个候选转弯做 footprint sweep。

### 第三步：旁路规划器在线转弯几何门控

保留现有 `navigation_safety` 和 `ManeuverLibrary` 作为离线检查工具，但在线规划默认只做：

- 道路图状态转移；
- 任务和阶段约束；
- 航向约束；
- 普通节点计数；
- 阻塞边过滤。

路线仍然输出原来的 `turn_junction` 和 RFID `z` 标记，保证正常控制器行为不变。

### 第四步：把避障恢复改成固定模板

新增恢复模板状态，不改 `FOLLOW -> TURN_* -> PATH_ALIGNED` 的正常巡航流程。

恢复模板调用现有固定转弯动作，而不是复制新的转弯算法。

### 第五步：最后再删除重复的在线几何代码

经过 loopback 和实车回放确认后，再考虑删除或彻底旁路：

- 在线 `turn_sweep()`；
- 在线动态障碍 footprint 检查；
- 正常规划中的 `REVERSE` 恢复路线；
- 与控制器执行模型重复的路线几何门控。

## 9. 结论

当前路径规划算法不是完全错误，而是把“道路任务规划”和“车辆动作可行性验证”绑定得过紧。对于当前车辆，建议收敛为：

```text
规划器决定走哪条已知合法的直行边；
控制器决定如何执行固定转弯和避障恢复；
总控决定超节点、任务阶段和阻塞边；
正常巡航转弯状态机完全保留。
```

最需要先验证的不是删除碰撞代码，而是通过可视化确认三件事：

1. 超节点完成后普通节点计数是否清零；
2. 恢复完成后普通节点计数是否从零重新开始；
3. 正常巡航路线经过转弯时，控制器现有状态机是否完全不变。

确认这三点后，再把在线规划器从 footprint/sweep 门控中释放出来，风险最低。

## 10. 按当前代码还原的规划流程

一次正常规划请求应理解为“从当前合法道路状态继续选一段正向道路”，而不是从任意位姿重新搜索连续轨迹。

### 10.1 总控准备请求

总控根据当前任务状态生成：

```text
start：最新 map -> base_link 位姿
mode：layer1 / layer2 / layer3
剩余任务点或待覆盖道路
已覆盖道路
ordinary_nodes_since_task
blocked_lane_ids
obstacle_revision
return_home
allow_partial
```

任务阶段由 [src/navigation_mission_manager/src/mission.cpp](src/navigation_mission_manager/src/mission.cpp) 决定：

```text
layer1：访问剩余任务超节点，不回家
layer2：覆盖剩余隧道，不回家
layer3：覆盖剩余普通道路并回到 launch
```

### 10.2 起点接入道路图

规划器把起点转换到 arena 坐标，识别当前可以接入的道路节点和方向。当前实现还会做 footprint、前向线段和转弯 sweep 检查；按新的职责设计，保留的核心应该是：

```text
当前位于哪条道路或哪个节点
当前车辆朝哪个道路方向
起点是否满足正常 DRIVE 的道路状态
```

恢复流程完成后，不应由规划器从一帧位置猜测恢复历史；控制器应向总控报告明确的 `resume_node`、`resume_lane` 和 `resume_heading`。

### 10.3 Layer 1：任务点搜索

Layer 1 使用目标集合搜索。状态包含：

```text
已访问目标集合
当前节点
当前航向
连续普通节点数
```

每次沿一条有向道路前进：

```text
到达超节点：ordinary_count = 0
到达普通节点：ordinary_count += 1
ordinary_count 达到 2 后，下一步必须进入超节点
```

目标搜索会为道路长度和转弯付出代价，寻找能够覆盖目标集合的路线。正式执行后，只有路线 ACK、真实 `GOAL_REACHED` 和现有超节点流程完成，任务进度才提交。

### 10.4 Layer 2 和 Layer 3：覆盖搜索

覆盖搜索状态位于 [src/new_map_planner/src/coverage.cpp](src/new_map_planner/src/coverage.cpp)，包含：

```text
已覆盖道路位掩码
当前节点
当前航向
ordinary_count
到达道路
```

搜索使用回溯，不是单纯的最近边贪心：

1. 删除请求中已经完成的道路；
2. 根据阶段过滤隧道/普通道路类别；
3. 过滤 `blocked_lane_ids`；
4. 生成满足航向和普通节点计数的正向后继；
5. 优先尝试新增覆盖道路；
6. 遇到死路回溯；
7. Layer 3 还检查是否能到达 home；
8. 找到完整状态后编译路线。

当前实现最多展开 2,000,000 个状态，不保证最短路线；达到上限只能报告搜索未完成，不能当作地图必然不可达。

### 10.5 规划器输出

规划器输出的核心应收敛为：

```text
正向道路序列
每段道路的方向
每个节点是否是超节点
路口处的 LEFT / RIGHT 转弯语义
访问任务点顺序
覆盖道路集合
下一次规划需要继承的 ordinary_count
blocked_lane_ids 使用的 obstacle_revision
```

路径仍通过现有 `navigation_path` 交给总控，正常巡航控制器继续从路径点标记触发原来的停车和转弯状态机。

## 11. 总控障碍接口设计

### 11.1 规划服务接口

建议在 `PlanRoute.srv` 中增加一份阻塞道路快照：

```srv
# Current complete snapshot of temporarily blocked physical lanes.
# A lane id blocks both travel directions in the first version.
string[] blocked_lane_ids

# Monotonically increasing version owned by navigation_mission_manager.
uint64 obstacle_revision
```

建议放在 `dynamic_obstacles` 之后，保留旧多边形字段一段兼容期。新规划逻辑优先使用 `blocked_lane_ids`；旧多边形只用于调试或过渡验证。

规划响应的 `RoutePlan` 也应带回：

```text
uint64 obstacle_revision
string[] blocked_lane_ids_used
```

这样总控可以确认路线是基于哪一份障碍快照生成的。若生成期间版本已经变化，路线不得直接激活，应重新规划。

### 11.2 为什么第一版屏蔽整条道路

当前假设是障碍物在道路内部、不在路口，也不会堵住路口。道路宽度有限，感知距离不能证明从反方向接近就能安全通过。因此第一版采用：

```text
blocked_lane_id = 禁止该物理道路的两个方向
```

暂时不把 `blocked_direction` 作为规划器决策字段。以后如果实测证明障碍只影响单向，可以再扩展：

```srv
string[] blocked_directed_lane_ids
```

不要一开始把不可靠的单向判断硬编码进路线规划。

### 11.3 总控内部障碍状态

总控可以保留更丰富的观测记录，但只向规划器提交道路语义：

```text
ObstacleRecord:
  obstacle_id
  lane_id
  first_seen_time
  last_seen_time
  observation_pose
  estimated_position_map
  distance_m
  source
  revision
```

当前已有 `/perception/dynamic_obstacle`、`/obstacle/front_blocked` 和 `/obstacle/front_range`。总控根据障碍发生时的车辆位姿和当前执行道路推断 `lane_id`，而不是等车辆退回后再用新的位姿反推旧距离。

### 11.4 障碍状态生命周期

```text
NONE
  -> PENDING_OBSERVATION
  -> CONFIRMED_BLOCKED_LANE
  -> RECOVERY_REQUIRED
  -> RECOVERY_EXECUTING
  -> REPLAN_READY
  -> BLOCKED_LANE_ACTIVE
```

观测在超时前没有得到可靠道路归属时，保持 `PENDING_OBSERVATION`，控制器停车；不要生成未知边的规划请求。

恢复完成后：

```text
1. 控制器报告 resume_node / resume_lane / resume_heading；
2. 总控将 ordinary_count 置为 0；
3. 总控把确认的 lane_id 加入 blocked_lane_ids；
4. obstacle_revision 自增；
5. 以最新位姿和完整阻塞边快照重新规划。
```

障碍过期时，总控从集合中删除对应 lane，增加 revision，再允许新路线使用它。已确认覆盖的道路不能因为临时阻塞自动变成已完成；它应保持“已覆盖”和“当前阻塞”两个独立属性。

### 11.5 路线激活时的版本检查

总控收到规划响应后检查：

```text
response.obstacle_revision == 当前 obstacle_revision
response.blocked_lane_ids_used == 当前 blocked_lane_ids
```

如果不相等：

```text
丢弃该路线
重新请求规划
```

这样可以避免规划器刚生成路线，感知又发现新的障碍，旧路线仍被发布给控制器。

## 12. 满足任务和避障要求的规划器设计

规划器的新核心不是“寻找一条几何上最安全的连续曲线”，而是搜索一个带业务状态的有向道路图。

### 12.1 规划状态

建议明确使用：

```cpp
struct PlanningState {
  NodeId node;
  Heading heading;
  uint8_t ordinary_count;
  CoverageMask covered_lanes;
  TaskMask completed_targets;
  std::set<LaneId> blocked_lanes;  // 由请求快照提供，不在状态内变动
};
```

`blocked_lanes` 实际上属于本次请求上下文，不需要每个搜索状态重复存储。

### 12.2 合法转移

对每条候选有向道路执行以下判断：

```text
1. lane_id 是否在 blocked_lane_ids 中
2. 是否属于当前阶段允许的道路类别
3. 是否导致道路内掉头
4. 是否会让 ordinary_count 超过 2
5. 是否违反当前阶段的目标/覆盖约束
6. 是否需要控制器之外的长距离倒车
```

通过后更新：

```text
next_node = lane.to
next_heading = lane.heading
next_ordinary_count = is_supernode(next_node) ? 0 : ordinary_count + 1
next_coverage = coverage | lane_bit
next_targets = targets | target_bit(next_node)
```

这里不生成转弯姿态。航向改变只产生一个 `TURN_LEFT` 或 `TURN_RIGHT` 语义，实际动作仍由生产控制器现有状态机完成。

### 12.3 三层目标

```text
Layer 1：
  搜索能够到达剩余任务超节点的正向路线；
  到达超节点后依赖现有 RFID 流程完成确认和清零。

Layer 2：
  搜索覆盖所有剩余隧道的正向路线；
  允许经过其他道路作为连接，但不把非隧道边算作本层目标。

Layer 3：
  搜索覆盖所有剩余普通道路的正向路线；
  过滤 blocked_lane_ids；
  完成覆盖后回到 launch。
```

这里建议重新确认当前 `layer3` 的连接边规则。当前代码禁止隧道作为连接边；如果真实任务只要求“补普通道路”，但允许通过隧道去连接两条普通道路，那么 `allow_tunnel_connectors` 需要改成由任务规则明确控制，而不能隐含禁止。

### 12.4 恢复与正常规划的边界

正常规划永远不生成长距离倒车：

```text
正常规划允许：
  DRIVE
  TURN_LEFT
  TURN_RIGHT
  FINISH

正常规划不生成：
  跨道路长距离 REVERSE
  用倒车绕过 ordinary_count
  用倒车代替非法掉头
```

障碍恢复由控制器执行固定模板：

```text
障碍检测
  -> 停车
  -> 选恢复模板
  -> 沿合法历史/指定方向倒车
  -> 调用原有固定转弯状态机
  -> 恢复正向道路
  -> ordinary_count = 0
  -> 总控更新 blocked_lane_ids
  -> 规划器重新搜索
```

正常巡航的 `FOLLOW -> BRAKE -> TURN_60 -> DIAGONAL_CREEP -> TURN_30 -> PATH_ALIGNED` 完全不动。恢复模板只复用其中的固定转弯动作，不复制或修改它的控制参数。

## 13. 设计收敛补充（本节取代前文冲突的建议）

本文件是设计稿。现有代码事实与拟议行为分开说明；下列约定为本轮设计的最终口径。

- 阻塞接口采用整条物理边双向禁用的完整快照：`blocked_lane_ids + obstacle_revision`。前文关于默认单向禁用的建议作废。规划器不维护障碍寿命，多边形仅保留用于感知和显示。
- RFID 工作流完全复用现有实现，不调整保持时间。当前代码 `post_speech` 根据是否需要播报选择 `rfid_hold_duration_s` 或 `rfid_stop_duration_s`，不能将现状描述成所有场景均严格停一秒；默认普通停留为 1 秒，需要播报的保持默认值为 3 秒，以现有配置为准。位置修正请求只包含 x/y，不修改航向。
- 当前普通节点计数在规划图中以任务节点到达作为预计清零；总控 `finish_rfid_workflow()` 当前没有直接将计数清零的赋值。新实现由实际超节点确认事件驱动执行计数清零，并以节点通过事件逐次递增；规划结果里的终点计数仅作预测。恢复完成事件把实际计数设为 0。
- Layer 1 当前优先下一待办目标，底层目标集合搜索为带道路长度和转弯代价的 Dijkstra；Layer 2/3 当前为覆盖集合回溯，找到首条完整路线，不证明最短。改造沿用这些任务语义，先在离散状态图上保证完成性，再优化道路重复和长度。
- 起点不重复计数。请求起点是计过的节点时直接沿用计数（包括 0）；在边中接入后到达的新普通节点才增加一次。`launch` 是任务终点，是否作为普通路口计数必须独立定义，不能因为没有 RFID 自动计一次；它不能清零，也不能成为绕过超节点规则的中间通道。
- 第三层目标集合为 `普通边 - 已确认覆盖边 - 当前阻塞边`。阻塞边不标成已覆盖，结果单独列 `skipped_blocked_lane_ids`；拓扑不可达的其他边列 `unreachable_lane_ids`，不能当作障碍跳过。完成状态要求目标集合覆盖并返回 home。阻塞隧道/任务点保留未完成状态，不能伪造第一、二层完成。
- 连接边集合与目标边集合分开。沿用当前 Layer 3 不以隧道连接的行为作为初始兼容配置，明确提供配置项；“只覆盖普通道路”本身不等于“禁止经过隧道”。
- 在约束图的状态 `node + heading + count` 上生成后继，再在状态与目标/覆盖 mask 的乘积图中搜索。移除在线 footprint/sweep 否决；保留地图拓扑、禁止直接掉头、计数和阻塞边这些用户指定的业务规则。
- 正常直行、巡航转弯控制代码、参数、入口判据、制动、路径标记和 RFID 握手冻结。文档里的 TURN 状态链是概念图，实际状态名和完成容差以当前控制器实现为准，不新增一个 PATH_ALIGNED 几何收敛门控。
- 恢复不新增 `RECOVERY_FAILED` 状态，本轮采用用户给定理想假设。控制器提供有限的恢复出口候选：候选包含出口状态、模板 ID 和涉及的道路；总控剔除已阻塞道路，规划器仅评估候选出口之后的正常路线。规划器不选择倒车点位，也不生成恢复轨迹。
- 恢复出口评估是一种只读请求，不能发布执行 Path。总控选定出口后下发控制器，控制器完成模板并报告真实出口位姿/道路/航向；总控再以 count=0 发起正式规划。这样避免控制器先选出口、重新规划后又发现没有合法后继。
- 恢复涉及的倒车只退出障碍边的已通过部分，不允许穿过障碍位置；两次转弯所借用的旁支必须没有被阻塞。90 度拐角遇障先完成当前固定转弯，再执行专用倒车返回模板，不通过改变巡航转弯控制律实现。
- 障碍事件锁存一次。发现时立即更新阻塞集合版本，阻止旧路线激活；转弯中先完成当前转弯。感知消失不自动撤销已确认阻塞边，由总控清除/过期策略管理。恢复完成不必再重复增加同一版本。
- 被障碍中断的路线只提交已经实际完成的边和已完成 RFID 握手，尚未走完的边不计覆盖；恢复机动不提交新覆盖。这需要增加边完成事件或实际进度记录，不能沿用整条计划的预计覆盖集合直接提交。

### 选择路线的策略

先构建统一的合法状态图，然后对各层使用带代价的集合搜索（Dijkstra/A* 或具有代价剪枝的回溯）。覆盖 mask 只包含当前阶段未完成且非阻塞的目标边，已覆盖边仍可作为连接边。代价默认以累计道路长度为主，转弯数为次；不改动控制器速度来配合代价。回家请求把 home 纳入真正终止条件。

为控制规模，先使用现有首条完整覆盖搜索保证行为一致，移除无关 arrival_lane 维度前验证未来转移确实与该维度无关，再加代价剪枝、缓存和可采纳下界。部分结果必须有真实新增进度，并有合法后继（或已完成回家）；SEARCH_LIMIT 单独报告，不能混同 NO_ROUTE。不承诺给定时间内所有障碍组合都全局最优。

### 验收示例

| 场景 | 必须满足 |
|---|---|
| 无障碍完整任务 | 12 个超节点完成既有握手，隧道目标完成，普通边补齐并回家 |
| 两个普通节点之后 | 下一计数节点必须是超节点；无法满足时搜索其他路线 |
| 恢复出口在普通节点 | 新请求 count=0，起点不自动升为1 |
| 障碍在未覆盖普通边 | 双向排除、单独记录跳过，其他可达目标覆盖后回家 |
| 障碍中断一段路线 | 保留已执行边进度，不提交未执行后半段 |
| T / 十字路口恢复 | 只选模板提供的出口，借用道路未阻塞，恢复后前向续行 |
| 转弯中发现障碍 | 原转弯完成流程保持一致，再进入专用恢复模板 |
| 规划过程中障碍版本变化 | 旧结果不能激活 |
| 搜索上限到达 | 报告 SEARCH_LIMIT，不宣称无路或任务完成 |

## 14. 实现前验收标准

### 规划搜索

- Layer 1 能按超节点目标完成任务；
- Layer 2 只把隧道计入本层覆盖；
- Layer 3 能补普通道路并回到 launch；
- 连续两个普通节点后不会选择第三个普通节点；
- 超节点完成后计数清零；
- 恢复完成后从零开始计数；
- blocked lane 不出现在新路线中；
- 正常路线不含长距离倒车；
- obstacle revision 发生变化时旧规划不会激活。

### 控制器兼容

- 现有正常巡航转弯状态和参数不变；
- 路径 `z` 标记含义不变；
- 路线 ACK 和 `GOAL_REACHED` 语义不变；
- 超节点停车、位置修正、播报和一秒保持不变；
- 恢复完成后能明确提供新的道路状态。

### 可视化

每次规划输出至少显示：

```text
当前节点和航向
超节点/普通节点分类
ordinary_count
已覆盖边
当前阻塞边及 obstacle_revision
本次路线的道路顺序
每个道路转移的 LEFT/RIGHT 语义
每层任务剩余量
```


## 15. 路线生成：一个宏观目标加一条道路走廊

### 15.1 核心原则

规划器不生成连续微观轨迹，也不一次性为整场任务生成一条超长路线。每次只做：

```text
选择下一个宏观目标点
  +
搜索到这个目标点的正向道路走廊
```

控制器接收道路走廊后负责：

```text
沿当前道路直行
在路口执行现有固定转弯
在超节点执行现有 RFID 流程
到达本段宏观目标后停车/确认
```

因此“规划器选择一个点”在接口上应表示为：

```text
MacroGoal：本段必须到达的宏观目标
RoadCorridor：到达该目标必须经过的有向道路序列
```

只给一个坐标点不够，因为控制器还需要知道中间路口的左转/右转语义。规划器不控制转弯，但必须提供道路走廊；控制器不重新规划，只执行走廊。

### 15.2 每段路线的状态流

```text
PLAN_NEXT_MACRO_GOAL
  -> EXECUTE_ROAD_CORRIDOR
  -> MACRO_GOAL_REACHED
  -> SUPER_NODE_SERVICE（若目标是超节点）
  -> COMMIT_ACTUAL_PROGRESS
  -> PLAN_NEXT_MACRO_GOAL
```

遇到障碍时：

```text
EXECUTE_ROAD_CORRIDOR
  -> OBSTACLE_BRAKING
  -> ENUMERATE_RECOVERY_EXITS
  -> EVALUATE_CONTINUATIONS
  -> EXECUTE_SELECTED_RECOVERY_TEMPLATE
  -> REPLAN_FROM_REAL_EXIT
```

正常巡航的控制器状态机保持不变：

```text
FOLLOW -> BRAKE -> TURN_CAPTURE -> APPROACH_ENTRY
       -> TURN_60 -> DIAGONAL_CREEP -> TURN_30
       -> PATH_ALIGNED -> FOLLOW
```

### 15.3 宏观目标选择

#### Layer 1

宏观目标是未完成的 RFID 超节点：

1. 先尝试任务规定顺序中的下一个超节点；
2. 该目标不可达时尝试其他未完成超节点；
3. 只接受满足普通节点计数和阻塞边约束的候选；
4. 到达后使用现有 RFID 停车、位置修正、播报、保持和授权流程。

#### Layer 2

宏观目标是下一个需要覆盖的未完成隧道。候选隧道必须满足：

- 从当前状态正向可达；
- 不经过 `blocked_lane_ids`；
- 不违反普通节点计数；
- 覆盖该隧道后仍有合法的后续隧道路线。

#### Layer 3

宏观目标是下一个未覆盖且未阻塞的普通边，最后的宏观目标是 `launch`。不能只选择几何距离最近的边；至少要检查完成该边后剩余道路仍能覆盖并回家。

目标边和连接边分开：已经覆盖的边可以作为连接边重复经过，但不再次贡献进度；阻塞边既不能作为目标边，也不能作为连接边。

### 15.4 道路走廊搜索

每次候选目标都从一个离散道路状态开始：

```cpp
struct RouteState {
  NodeId node;
  Heading heading;
  uint8_t ordinary_count;
  TargetMask completed_targets;
  CoverageMask covered_lanes;
};
```

`blocked_lane_ids` 和 `obstacle_revision` 是本次请求的只读上下文，不需要复制到每个状态中。

搜索每条有向道路作为后继：

```text
过滤阻塞道路
过滤不属于当前阶段的目标/连接道路
禁止道路内直接掉头
禁止正常长距离倒车
检查 ordinary_count 是否超过 2
更新节点和航向
更新任务/覆盖集合
```

状态更新规则：

```text
到达普通节点：ordinary_count += 1
到达超节点：ordinary_count = 0（执行时以实际超节点流程确认）
恢复完成后的新规划起点：ordinary_count = 0
```

搜索结果包含：

```text
macro_goal
corridor_lane_ids
corridor_directions
每个路口的 LEFT / RIGHT 语义
waypoint_supernode_ids
covered_lane_ids
predicted_ordinary_nodes_since_task
obstacle_revision
```

它不包含：

```text
车体扫掠轨迹
速度指令
倒车轨迹
转弯的 60 度/30 度微观控制参数
```

### 15.5 障碍恢复前的路线可行性评估

控制器遇到障碍后不能随便退到最近路口。它应提供有限的合法恢复出口候选：

```text
candidate_id
resume_node
resume_lane
resume_heading
recovery_template
recovery_edges
```

规划器对每个候选出口执行一次“后续路线搜索”：

```text
候选恢复出口
  -> ordinary_count = 0
  -> 使用当前任务进度
  -> 加入最新 blocked_lane_ids
  -> 规划到下一个宏观目标
  -> 检查该目标之后当前阶段是否仍可完成
  -> Layer 3 额外检查最终回到 launch
```

只有能够完成当前阶段、且不使用阻塞道路的出口，才允许控制器执行对应恢复模板。规划器不生成倒车动作，也不发布恢复路线；它只判断出口之后有没有可行的道路走廊。

如果多个出口可行，排序为：

```text
完整完成当前阶段
  > 最终可回家
  > 恢复模板动作少
  > 后续道路长度短
  > 转弯次数少
```

恢复模板执行完成后，控制器报告实际 `resume_node / resume_lane / resume_heading`，总控将 `ordinary_count` 设为 0，再用实际位姿重新请求正式的 `RoadCorridor`。这样可以同时保证恢复方向经过规划器筛选，且正式路线使用的是控制器真实到达的位置。

### 15.6 规划服务的目标语义

长期可以把路线结果显式表示为：

```text
macro_goal_type
macro_goal_id
corridor_lane_ids
corridor_directions
turn_actions
waypoint_supernode_ids
obstacle_revision
```

短期继续发布现有 `navigation_path`：

- 路径几何继续给控制器使用；
- `z=0.001/0.002/0.003/0.004/0.005` 的语义保持不变；
- 控制器继续使用现有巡航转弯状态机；
- 总控通过 `RoutePlan` 元数据维护宏观目标和实际进度。

### 15.7 这套算法满足的要求

```text
规划器决定：
  下一步去哪一个宏观点
  经过哪些道路
  哪些道路被障碍排除
  当前阶段是否还能完整完成

控制器决定：
  如何沿道路直行
  如何停车
  如何执行既有固定转弯
  如何执行选定的避障恢复模板

总控决定：
  何时确认超节点
  何时提交实际进度
  何时锁存或清除阻塞边
  何时进入下一阶段
```


## 16. 本轮实现状态（2026-10-12）

本轮已经落地并通过构建的部分：

- `PlanRoute.srv` 增加 `blocked_lane_ids`、`obstacle_revision` 和精确普通节点计数标志；
- `RoutePlan.msg` 回传 `blocked_lane_ids_used` 和 `obstacle_revision`；
- 规划节点把请求快照写回响应；
- 规划器目标/覆盖搜索过滤阻塞物理边；
- 正常起点沿用原 `ordinary_nodes_since_task=1` 语义，恢复请求可显式使用精确 0；
- 总控保存阻塞边集合和单调版本，向规划器发送完整快照，并在路线激活前校验响应快照；
- 总控保留原动态障碍多边形链路，无法归属道路时不生成未知边；
- 控制器增加 ROS 无关的恢复出口候选和状态边界模型，未修改正常巡航转弯实现；
- 可视化输出阻塞边、障碍版本、路线道路序列、节点计数和恢复候选；
- 正常巡航测试的旧 `TURN_ARC` 断言兼容当前已有的 `TURN_APPROACH_ENTRY` 状态名，未改变生产控制器状态发布。

已验证：

```text
colcon build --base-paths src --packages-up-to new_map_planner navigation_mission_manager visual_navigation：通过
new_map_planner C++ planner_test：28/28 通过
navigation_mission_manager：8/8 通过
visual_navigation：17/17 通过（含 launch 集成测试）
obstacle_recovery_test：20/20 通过
可视化 Python 测试：2/2 通过
git diff --check：通过
```

本轮的最终行为边界：

- 正常在线规划只使用道路拓扑、航向、普通节点计数、任务阶段和 `blocked_lane_ids`；不执行 footprint 或固定转弯 sweep 门控。
- `validate_geometry` / `online_geometry_validation=true` 时保留严格几何模式，用于专项验证和恢复路线；默认总控请求使用图规划模式。
- 路线编译仍发布现有 `navigation_path`、转弯标记和 RFID 超节点元数据；生产控制器继续使用原有固定巡航转弯状态机。
- 动态障碍由总控转换成阻塞物理边并携带 `obstacle_revision`，规划结果回传快照并由总控在激活前校验。
- 恢复候选模型保留在控制器侧，正常巡航转弯代码没有被改写。

本轮目标已完成。
