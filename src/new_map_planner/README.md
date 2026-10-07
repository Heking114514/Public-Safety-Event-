# New Map Planner

面向正式赛场实车的分层规划器。功能包不包含 loopback、虚拟里程计、路线播放或模拟
车辆；规划起点只接受请求中的实时 `map -> base_link` 位姿。正式任务编排由独立的
`navigation_mission_manager` 负责，规划 UI 只承担预览、任务命令和状态监控。

## 分层

```text
任务层
  目标集合 / 待覆盖道路 / 已确认进度
            |
道路图层
  状态 = 路口 + 到达航向
  决定道路顺序，禁止隐式掉头
            |
动作层
  DRIVE / TURN_LEFT / TURN_RIGHT / REVERSE / FINISH
  转弯采用 tight_l_turn_v1 标定动作
            |
几何验证层
  直行：完整车身 + 15 mm 膨胀
  紧转：按实车状态格参考轨迹逐姿态连续扫掠 + 8 mm 矩形膨胀
  动态障碍：请求级多边形，不重建静态地图
            |
ROS 适配层
  RoutePlan（完整语义） + nav_msgs/Path（现有实车控制器兼容）
```

道路图负责选择，几何层只负责否决不可执行的直行段或动作。规划器不会把“位于路口”
当成免碰撞条件。`visual_navigation` 的兼容路径只对已经通过扫掠验证的 90 度动作设置
`z=0.001`；边缘道路倒车退回路口后的转弯使用 `z=0.002`。两者由同一个实车
corner-shunt 状态格核心完成并在规划期连续扫掠。起点吸附到邻近路口的短接入段
如果需要先倒车或横移，而兼容 `Path` 又没有对应动作语义，规划器必须返回失败；任务层
会继续退到更早锚点，不能把它伪装成普通前进路线并报告 `success`。

## 规划模式

- `layer1`：访问剩余任务点，不回原点；允许把当前可达部分作为一段执行。
- `layer2`：从第一阶段实测终点继续，显式穿越 4 条隧道，不回原点。
- `layer3`：禁止使用隧道作为连接边，只补齐尚未覆盖的普通道路，最后返回发车点。
- `targets`：对最多 16 个任务点执行带航向的精确集合搜索；正式 12 点不是贪心排序。
- `coverage`：逐段选择最近的可执行未覆盖道路，途经道路同步计入本次计划。
- `mission`：兼容的一次性道路覆盖请求，并报告途经任务点。

正式“完整巡视”由 `navigation_mission_manager` 按 `layer1 -> layer2 -> layer3` 编排。每段使用
最新 `map -> base_link` 位姿重新规划；只有匹配的 route ACK 和真实 `GOAL_REACHED`
到达后才提交该段进度。覆盖模式是确定性的可执行优先策略，不宣称全局最短。无法到达
的道路或任务会进入 `deferred_*`，只有请求设置 `allow_partial=true` 时才返回安全部分路线。
道路图搜索状态还包含连续普通路口数：抵达任务路口后清零，普通路口最多连续经过两个。
该计数随规划请求和路线结果传递，障碍恢复仍由任务管理器与导航控制器执行。
`layer1` 按待办顺序先规划下一个任务点；该点不可达时，再搜索其他待办点。

## 接口

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/new_map_planner/plan` | `new_map_planner/srv/PlanRoute` | 预览或激活路线 |
| `/new_map_planner/route_plan` | `new_map_planner/msg/RoutePlan` | 带显式动作的完整计划 |
| `/new_map_planner/arena_path` | `nav_msgs/msg/Path` | `arena` 坐标预览 |
| `/new_map_planner/navigation_path` | `nav_msgs/msg/Path` | `map` 坐标兼容路线 |
| `/new_map_planner/map` | `nav_msgs/msg/OccupancyGrid` | UI/RViz 静态地图 |
| `/waypoint_navigation/route_input` | `nav_msgs/msg/Path` | 经验证后才发布的实车路线 |

`success` 表示规划与所有几何验证通过，`activated` 表示兼容路线已经发布给导航器。
任务节点等待相同 route id 的 `/waypoint_navigation/route_ack`，不会把服务成功当成
车辆已经接收；规划器本身不保存任务进度或控制车辆。

## 实车启动

终端 1 启动相机、融合、导航控制、串口和 rosbag：

```bash
./scripts/start_visual_navigation.sh
```

终端 2 启动自主任务节点、新规划器和实车 UI：

```bash
./scripts/start_new_map_planner.sh
```

只启动后端用于接口调试：

```bash
./scripts/start_new_map_planner.sh --no-ui
```

UI 中“规划预览”绝不发布行驶路线；完整巡视的预览只显示第一阶段。点击“开始三阶段
巡视”后，UI 只向任务节点发送开始命令。任务节点在后台独立完成阶段衔接、障碍重规划、
路线 ACK 和进度提交；关闭 UI 不会终止或暂停正式任务。

任务节点接收结构化 `/perception/dynamic_obstacle`；同时兼容
`/obstacle/front_blocked` 与 `/obstacle/front_range`。只有控制器安全退回并发布
`OBSTACLE_REPLAN_REQUIRED` 后，观测才进入动态地图并触发当前阶段重规划。

不要同时启动 `arena_path_planner` GUI、CSV `--autostart` 或路线编辑器，它们会竞争
`/waypoint_navigation/route_input`。

## 构建与测试

```bash
colcon build --symlink-install --packages-select new_map_planner
colcon test --packages-select new_map_planner \
  --event-handlers console_direct+ --return-code-on-test-failure
```

正式地图测试覆盖任务点、隧道、普通道路补漏和回程三阶段接力，以及发车路口和顶部
边界路口的完整车身扫掠、动态障碍延期与坐标变换。车身尺寸仍只来自
`vehicle_config/config/vehicle_physical.yaml`。
