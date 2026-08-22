# 上位机问题清单

## 严重程度

- `P0`：已经对目前的里程计和导航系统造成实质影响，使其大幅偏离应有方向，需要立即修复。
- `P1`：已经对目前系统造成影响，但影响范围有限，部分影响系统性能。
- `P2`：设计思路或逻辑存在冲突、矛盾、缺陷，未能发挥系统最佳性能。
- `P3`：系统存在潜在矛盾或冲突，但目前未发现其已经影响系统。
- `P4`：模块过耦合，不利于后续系统维护或扩展。

## P0

### P0-03 最近一次实车控制没有形成有效路径推进

- 证据：`latest_navigation_bag/` 中路线约 `29.33 m`，实际仅推进约 `3.67 m`；运动期约 69% 处于 `PATH_LATERAL_RECOVERY`，出现约 24 次角速度方向反转，最终横向误差约 `12.6 cm`。
- 根因：该 bag 对应的旧运行版本在横向恢复期间冻结路径进度，但仍以约 `0.08 m/s` 前进；冻结参考逐渐落到车后，恢复约 `2.0 s` 超时解冻后仅推进少量路径，约 `0.35 s` 后再次恢复，形成恢复、紧弯和角速度反向循环。
- 未闭环：当前版本只通过单元测试和旧轨迹无底盘回放；尚无当前源码、二进制和参数可追溯的新实车 bag，不能确认首弯后路线索引会持续推进、角速度不再周期性反向，且宽横向误差下不会侵占规划安全边界。
- 影响：最近一次实车轨迹长度约 `5.59 m`，有效路线进度只有约 `3.67 m`，未完成剩余约 `25.66 m` 路线。

## P1

### P1-04 后端按点机器人规划，前端却显示完整车体已参与检查

- 证据：`src/arena_path_planner/include/arena_path_planner/planner.hpp:41-61` 的车长、车宽和边距默认为零，活动配置 `src/arena_path_planner/config/arena_map.yaml` 未覆盖这些值；`scripts/arena_route_frontend.py:173-177,443-448` 默认显示 `0.217 × 0.210 m` 车体和 `0.040 m` 边距。
- 触发：路线靠近障碍物、窄通道或需要原地转向。
- 影响：界面显示的车体轮廓没有进入后端碰撞计算；`RotationIsFree()` 在零尺寸配置下也会退化为点占用检查。

### P1-05 未覆盖全部目标的部分路线可按完整任务启动

- 证据：`src/arena_path_planner/src/planner_node.cpp:206-229` 在 `result.success=true` 时发布路线，没有用 `all_targets_reached` 阻止激活；`scripts/arena_route_frontend.py:603-660` 未正确区分包含 `deferred_targets` 的部分路线。
- 触发：规划成功，但部分任务点被延后。
- 影响：车辆可执行一条未覆盖全部目标的路线，前端同时给出完整路线已经启动的错误状态。

### P1-06 视觉位姿与本地里程计使用最近样本对齐，没有插值

- 证据：`src/fused_odometry/src/map_odom_correction_node.cpp:192-207` 选择时间最近的 local pose，并允许最大约 `0.12 s` 时间差。
- 触发：视觉和本地里程计异步或存在调度抖动。
- 影响：以 `0.5 m/s` 行驶时，最大允许时间错位可形成约 `6 cm` 的位置误差，并进入 map-to-odom 校正。

### P1-07 轮式里程计使用串口到达时间代替真实测量时间

- 证据：编码器消息没有 ROS header；`src/wheel_odometry/src/wheel_odometry_node.cpp:150-188` 使用 MCU 间隔计算速度，但以节点 `now()` 作为 odometry stamp。
- 触发：USB 串口排队或上位机调度延迟变化。
- 影响：wheel 虽不再进入 EKF，但仍参与视觉残差、健康分类和失视距离预算；串口延迟会被混入这些判断的时间差和测量差。

### P1-08 导航和融合使用不同版本的 IMU 角速度

- 证据：`src/visual_navigation/src/waypoint_navigator.cpp:653-659` 直接使用 `/imu/filtered`；`src/fused_odometry/src/fusion_gate_node.cpp:681-746` 对该角速度再做视觉参考 bias 修正。
- 触发：视觉参考 bias 不为零。
- 影响：导航转向阻尼使用的角速度与 `/odometry/fused` 使用的角速度不一致，同一次转弯存在两套反馈口径。

## P2

### P2-01 到点停车超时会被当成停车完成

- 证据：`src/visual_navigation/include/visual_navigation/path_control.hpp:352-370` 在 `total_stop_seconds >= timeout` 时直接返回 true；`src/visual_navigation/src/waypoint_navigator.cpp:1673-1710` 只记录警告并继续状态转换；测试也把该行为作为期望结果。
- 触发：制动超时后实测速度仍高于阈值，或速度值无效。
- 影响：车辆可在没有确认停稳时进入转向或下一路径段。最近 rosbag 没有运行到该状态，当前未形成实车影响证据。

### P2-02 `/odometry/fused` 的位姿和协方差不属于同一状态

- 证据：`src/fused_odometry/src/map_odom_correction_node.cpp:357-372` 复制 `/odometry/local` 后覆盖全局 pose，但保留本地 EKF 的 pose covariance 和 twist covariance。
- 触发：视觉校正、重定位或 map-to-odom 平滑改变全局 pose。
- 影响：消息中的全局位姿与位姿协方差含义不一致；当前导航主要使用 pose 和 twist，尚未发现它使用该 pose covariance 做控制决策。

### P2-03 视觉恢复计数不保证来自连续有效样本

- 证据：`src/fused_odometry/src/fusion_gate_node.cpp:358-460` 对部分无效样本和过大采样间隔没有一致重置 `visual_recovery_count_`。
- 触发：视觉在有效、无效和大间隔样本之间反复切换。
- 影响：零散有效样本可能累计达到恢复门限，使系统早于预期恢复视觉状态。

### P2-04 map 校正会根据控制命令冻结全局平移

- 证据：`src/fused_odometry/src/map_odom_correction_node.cpp:243-279,357-364` 根据转向命令激活 turn hold，并重写全局 pose 与 `map_from_odom_`。
- 触发：命令被判断为原地转向，但车辆实际发生平移或打滑。
- 影响：真实平移可能被估计逻辑抹除，控制意图反向改变定位结果。

### P2-05 重复编码器帧仍会刷新 wheel 健康时间

- 证据：`src/wheel_odometry/src/wheel_odometry_node.cpp:159-181,200-225` 在确认积分结果是否重复前更新时间，duplicate 状态仍可保持诊断正常。
- 触发：串口持续重发同一序号编码器帧。
- 影响：轮式数据已经冻结，但依赖 wheel 的融合模式仍可暂时认为数据新鲜。

### P2-06 路线发布没有任务 ID 和导航端确认

- 证据：`src/arena_path_planner/src/planner_node.cpp:223-229` 发布路线后直接返回；`scripts/arena_route_frontend.py:589-655` 没有等待导航对对应路线的接收或启动确认。
- 触发：导航拒绝路线、导航节点未运行、旧路线残留或发布期间节点重启。
- 影响：前端显示的任务状态可能与导航端实际执行状态不一致。

### P2-07 无视觉降级没有限制在隧道所需的时间或距离内

- 证据：`src/fused_odometry/src/fusion_logic.cpp:553-557` 在 wheel 和 IMU 都健康时直接返回 `DEGRADED_NO_VISION`；`max_dead_reckoning_time_s` 和 `max_dead_reckoning_distance_m` 只约束后面的纯 wheel 分支。导航配置允许该状态以 `0.50` 倍速运行。
- 触发：相机在隧道外永久失效，但 wheel 和 IMU 消息继续发布。
- 影响：系统无法区分约 `0.8 m` 隧道遮挡和无限期视觉故障，可在无视觉状态下继续执行剩余路线。当前没有永久失视实车记录。

## P3

### P3-01 导航和融合主要按消息到达时间判断新鲜度

- 证据：`src/visual_navigation/src/waypoint_navigator.cpp:610-659,932-955` 与 `src/fused_odometry/src/fusion_gate_node.cpp:358-420,529-551,686-705` 没有统一检查 header stamp 的实际年龄、未来时间和完整单调性。
- 触发：跨机时钟错误、长队列延迟或 bag 回放旧消息。
- 影响：旧测量可能在刚到达时被视为新鲜；当前本机实时运行中未发现该问题的实际证据。

### P3-02 IMU 输入的有限值和 frame 校验不完整

- 证据：`src/imu_rpy_filter/src/imu_rpy_filter_node.cpp:442-468` 没有在滤波前完整检查有限值，未知的非 optical frame 会被按 body frame 处理；ORB IMU 入口也没有完整 finite/frame 校验。
- 触发：传感器驱动发布 NaN、Inf 或错误 frame。
- 影响：非法数据可能进入滤波或 ORB；当前固定 D455 接线下未发现该输入异常。

### P3-03 ORB 输出使用固定协方差

- 证据：`src/orbslam3_ros2/src/stereo-inertial/stereo-inertial-node.cpp:137-142,511-512,563-579` 使用固定 pose/twist covariance。
- 触发：视觉质量随弱纹理、快速运动或重定位发生明显变化。
- 影响：协方差不能表达视觉质量变化；当前主要门控还使用 tracking 状态和残差，没有证据表明固定协方差已经单独造成控制失败。

### P3-04 同一 D455 IMU 存在重复进入估计链的可选启动组合

- 证据：`src/fused_odometry/launch/odometry_bringup.launch.py` 可同时启用外部 IMU filter 和 `use_slam_imu=true` 的 ORB stereo-inertial。
- 触发：启动参数同时打开两条路径；当前脚本默认没有同时打开。
- 影响：同一物理 IMU 会通过 ORB pose 和外部 `wz` 两条相关路径影响估计，相关性没有显式表达。

### P3-05 ROS 时间与 wall timer 混用

- 证据：导航控制和串口发送使用 wall timer，部分等待和命令超时使用 ROS time。
- 触发：启用仿真时间后 `/clock` 暂停、回退或跳变；当前实车默认使用系统时间。
- 影响：旧命令或等待状态的年龄可能不再正常增长，wall timer 仍继续运行。

### P3-06 自动串口选择没有设备身份确认

- 证据：`src/cup_car_serial/src/cmd_vel_serial_node.cpp:44-72` 可从通用串口设备中选择第一个匹配项。
- 触发：上位机同时连接多个 USB 串口设备并使用 `auto`。
- 影响：节点可能打开无关设备；固定设备路径时不触发。

### P3-07 最近 rosbag 没有覆盖完整任务关键状态

- 证据：`latest_navigation_bag/` 没有 `BRAKING_AT_WAYPOINT`、最终航向完成、`GOAL_REACHED`、map-change 和完整隧道失视过程，路线只完成约 `3.67/29.33 m`。
- 触发：用该 bag 判断停车、终点、视觉恢复和完整路线能力。
- 影响：现有记录不能验证这些状态，但不等于这些状态已经发生故障。

### P3-09 节点级关键流程缺少自动化覆盖

- 证据：现有测试主要覆盖辅助函数，没有完整 `RunControl()`、隧道失视到恢复、制动到转向再到完成、部分路线激活、时间戳异常和 wheel 隔离的节点级测试。
- 触发：多个辅助函数单独通过，但节点状态和参数组合发生冲突。
- 影响：关键流程只能依赖实车发现问题，现有单元测试不能证明整条链路行为。

### P3-10 无效 IMU 时间戳会先污染滤波状态再被拒绝

- 证据：`src/imu_rpy_filter/src/imu_rpy_filter_node.cpp:447-493` 先更新中值、均值滤波器和 `last_stamp_`，之后才检查无效 `dt`。
- 触发：IMU 出现未来、乱序或重复时间戳。
- 影响：被拒绝的样本仍留在滤波历史中，并可影响后续正常样本；当前 bag 未发现该异常输入。

### P3-11 地图跳变授权与对应视觉帧没有绑定关系

- 证据：`src/fused_odometry/src/fusion_gate_node.cpp:323-329,441-451` 的 `/orbslam3/map_change` 没有 header 或事件 ID；ORB odometry 与 map-change 分别发布。
- 触发：两个 topic 的消息调度顺序变化或 map-change 延迟到达。
- 影响：跳变宽限可能作用于错误帧；当前 bag 没有 map-change，未发现实际影响。

### P3-12 原地转向安全判定阈值在规划器中重复硬编码

- 证据：`src/arena_path_planner/src/planner_node.cpp:41-60` 使用固定 `0.18 rad`；导航当前配置也为 `0.18 rad`，但两者没有共享配置。
- 触发：以后只修改规划器或导航器其中一侧的阈值。
- 影响：两侧会对哪些拐点需要原地转向产生不同判断；当前数值一致，尚未影响现有运行。

## P4

### P4-01 导航节点职责过度集中

- 证据：`src/visual_navigation/src/waypoint_navigator.cpp` 仍约 2029 行、104 个参数；路线接收、跟线、制动、转向、恢复、任务状态和命令发布仍共享同一个节点状态。
- 影响：修改某一运动阶段仍可能通过共享成员影响其他阶段，完整状态迁移难以独立测试。

### P4-02 融合门控节点职责过度集中

- 证据：`src/fused_odometry/src/fusion_gate_node.cpp` 仍约 1059 行；传感器回调同时承担 frame/时间校验、视觉增量计算、IMU bias、wheel 残差和估计输入重发布，传感器适配和门控仍共用大量节点状态。
- 影响：修改某一传感器的校验或残差逻辑仍可能改变其他输入的发布条件。

### P4-03 同一状态由多个节点分别维护

- 证据：视觉可用性由 ORB tracking、fusion gate 的视觉接收状态、map correction 的视觉超时和 navigator 的输入许可分别维护；静止状态由 IMU filter、map correction 和导航停车逻辑分别按不同阈值判断。
- 影响：排查视觉恢复或停车状态时仍需对齐多个节点的独立计时和阈值，单一状态话题不能完整解释各节点行为。
