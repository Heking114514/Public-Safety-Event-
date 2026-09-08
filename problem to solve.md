# 上位机问题清单

## 严重程度

- `P0`：已经对目前的里程计和导航系统造成实质影响，使其大幅偏离应有方向，需要立即修复。
- `P1`：已经对目前系统造成影响，但影响范围有限，部分影响系统性能。
- `P2`：设计思路或逻辑存在冲突、矛盾、缺陷，未能发挥系统最佳性能。
- `P3`：系统存在潜在矛盾或冲突，但目前未发现其已经影响系统。
- `P4`：模块过耦合，不利于后续系统维护或扩展。

## 2026-08-25 最终复核

结论：本轮授权范围内的结构解耦，以及下文明确标为“已解决”的项目已经完成；这份清单**尚未全部解决**，不能用一次构建通过代替实车和长时间数据验证。

- 最终 `colcon build` 成功构建 8 个活动包；工作空间配置只忽略独立维护的第三方 `ORB_SLAM3` 和 `pangolin` 源码，ROS wrapper `orbslam3` 仍参与构建与测试。
- 最终 `colcon test-result --verbose` 为 `235 tests, 0 errors, 0 failures, 0 skipped`；其中导航 launch test 实际运行 2 个节点级用例，并验证有效 CSV、`autostart=false` 的初始状态为 `IDLE`。
- 第一方代码文件均低于 800 行；含统一启动入口时最大为 `scripts/start_visual_navigation.sh` 的 772 行，C++ 最大为 `cup_car_serial/src/cmd_vel_serial_node.cpp` 的 656 行，本轮四个目标包最大为 `fused_odometry/src/fusion_logic.cpp` 的 598 行。目标包 `src/` 下没有头文件。
- 对比 `HEAD`，四个目标包 C++ 节点的参数名称与类型、publisher/subscription/service 契约、节点名和可执行文件名均保留；根目录路线编辑器只是迁入包内，安装名和原有 ROS 接口不变；`PlanArenaPath.srv` 与公开 `planner.hpp` 无差异。此前经授权的功能修复有意改变了部分行为，例如 `direct_visual_tracking` 默认关闭且不再直通，因此“接口保留”不应误读为“所有默认行为与 HEAD 完全相同”。
- `colcon graph --dot` 未发现包依赖环。第一方包之间没有直接包含对方源码；`mission_control_interfaces` 是公共消息契约，其他跨包边仅为 bringup 的运行时依赖。
- 代码或设计仍未闭环的项目主要包括 P1-06、P2-02、P2-03、P2-07、P2-10、P3-01、P3-03、P3-05、P3-09、P3-11、P3-12、P3-15 和 P4-03；P0-03、P2-04、P2-05、P2-08、P3-07 等还需要完整实车路线或长期数据验证。它们涉及时间戳闭环、协方差近似、连续视觉恢复、转向稳定计时、完整任务确认、动态视觉协方差、统一时钟抽象、节点级覆盖、map-change 帧级绑定、四元数姿态解算、规划模式分支和跨节点可观测性，不能根据一次静态重构或构建通过宣称已经解决。

## P0

### P0-01 漂移的编码器速度在正常视觉定位期间仍持续进入 EKF（已解决）

- 历史证据：旧版 `src/fused_odometry/config/fused_odometry.yaml` 曾将 `/fusion/input/wheel_odom` 配置为 EKF 输入。
- 当前状态：默认 EKF 只接收门控视觉前向速度和 `/imu/control`；轮速只用于残差、打滑/失速诊断以及视觉中断的受限距离预算，不修改正常视觉定位的 EKF 状态。
- 残余注意：轮速标定仍影响诊断和失视觉预算，不能因为轮速不进入 EKF 就跳过实车校准。

### P0-03 最近一次实车控制没有形成有效路径推进（旧运行记录）

- 证据：`latest_navigation_bag/` 中路线约 `29.33 m`，实际仅推进约 `3.67 m`；运动期约 69% 处于 `PATH_LATERAL_RECOVERY`，出现约 24 次角速度方向反转，最终横向误差约 `12.6 cm`。
- 当前状态：该 bag 对应旧运行二进制。当前导航已加入 `PathProgressSupervisor` 和 `TurnProgressSupervisor`，在持续无进展时先恢复，恢复次数耗尽后分别进入 `FAULT_NO_PATH_PROGRESS` 或 `FAULT_NO_TURN_PROGRESS`。
- 残余注意：现有 bag 仍不能证明新状态机在完整实车任务中的效果，需补充节点级和完整路线实测覆盖。

## P1

### P1-01 编码器误差会间接改变 IMU 输出和融合状态（已解决，诊断仍保留）

- 历史证据：旧版 `imu_rpy_filter` 曾把轮速参与静止判定，并可能将输出 `wz` 置零；融合门控也曾把轮速参与路径判断。
- 当前状态：`imu_rpy_filter` 中轮速只生成 `external_stationary/external_moving` 诊断，不再改写 IMU `wz`、姿态或协方差。融合门控仍使用轮速做残差、健康和受限失视觉预算判断，但 `/imu/control` 的角速度由 IMU 经过自身校正得到。
- 残余注意：轮速异常仍可能触发融合降级或故障诊断，这是健康监督行为，不等于轮速进入正常 EKF。

### P1-02 导航没有“持续运动但路线无进度”的判定（已解决）

- 历史证据：旧版跟线、航点恢复和精确转向缺少独立进度监督。
- 当前状态：`PathProgressSupervisor` 按沿线进度和实际位移监督路径，`TurnProgressSupervisor` 按航向误差改善和预计角速度监督转向；两者均先进入 `RECOVERING_*`，恢复次数耗尽才进入对应故障状态。制动、驻停和定位宽限阶段不累计进度超时。
- 残余注意：完整节点流程和实车长路线仍缺少覆盖，旧 bag 不能直接代表当前实现。

### P1-03 执行器健康只证明通信和模式正常，不证明命令执行正常（已解决）

- 历史证据：旧版串口节点没有比较目标轮速和实测轮速。
- 当前状态：`ActuatorTrackingMonitor` 按左右轮分别检查无响应、方向错误和严重跟踪误差；异常持续达到配置时长后，`actuator_healthy` 变为 `false`。统一实车入口使用 serial bringup，强制导航端启用 `require_actuator_health`，因此会停车并按有限次数自动恢复，最终可锁存故障。普通 bringup 只有显式启用该监督时才形成同样闭环；串口节点自身只负责发布健康状态，不会独立改写上游速度命令。
- 残余注意：诊断仍依赖下位机 `CTL` 遥测内容和单位正确，不能替代底层急停及通信看门狗。

### P1-04 后端按点机器人规划，前端却显示完整车体已参与检查（已解决）

- 历史证据：旧版活动地图没有覆盖规划器的车体尺寸和安全边距，前端与后端口径不同。
- 当前状态：活动 `src/arena_path_planner/config/arena_map.yaml` 已配置 `0.217 x 0.210 m` 车体、`0.015 m` 安全边距和 `0.020 m` 跟踪边距；平移和原地转向检查均使用车体 footprint。旧的窄走廊配置仅用于测试，不作为实车路线。

### P1-05 未覆盖全部目标的部分路线可按完整任务启动（已解决）

- 历史证据：旧版规划服务只检查 `result.success`，没有阻止含 `deferred_targets` 的路线激活。
- 当前状态：`ActivationAllowed()` 现在同时要求请求激活、规划成功和 `all_targets_reached=true`；部分路线仍可作为结果返回，但不能发布为已激活导航任务。

### P1-06 视觉位姿与本地里程计使用最近样本对齐，没有插值（部分解决）

- 历史证据：旧版 `map_odom_correction_node` 使用最近 local pose 配对。
- 当前状态：节点已按视觉测量时间在保留的 local 历史中做 SE(2) 插值（含最短路径 yaw），视觉样本暂时领先时也会短暂排队，不再使用最近样本近似。
- 未解决：map correction 没有记录最后接受的视觉时间戳，也没有对视觉和 local 测量统一执行年龄、未来时间与视觉乱序校验；仍落在 local 历史范围内的旧视觉帧可能被接受。

### P1-07 轮式里程计使用串口到达时间代替真实测量时间（已解决）

- 历史证据：旧版 wheel odometry 以节点接收时刻作为 odometry stamp。
- 当前状态：节点以 MCU 毫秒计数建立 MCU 到 ROS 的时间锚点生成测量时间；接收时刻用于首次锚定、低频校准和链路新鲜度，不再逐帧直接作为测量时间。校准器从五秒后开始定期以初始锚点为长基线估计 MCU/ROS 频率比并低通更新，补偿晶振频偏，同时避免短窗口的毫秒量化偏差；MCU 重启、乱序、异常间隔或未来时间会重建/拒绝，不发布非单调测量。
- 验证：纯 C++ 测试模拟 MCU 晶振慢 `50 ppm` 连续运行一小时，映射时间保持单调、不超前，最终时间差小于 `20 ms`；另有 MCU 回退后重新锚定测试。

### P1-08 导航和融合使用不同版本的 IMU 角速度（已解决）

- 历史证据：旧版导航直接订阅 `/imu/filtered`，融合门控另行做 bias 修正。
- 当前状态：融合门控发布 `/imu/control`，本地 EKF 和导航都使用该同一条经过校正、`base_link` 平面化的角速度；原始 `/imu/filtered` 只作为 gate 输入。

## P2

### P2-01 到点停车超时会被当成停车完成（已解决）

- 历史证据：旧版制动控制器超时返回完成，导航端只记录警告并继续状态转换。
- 当前状态：航点制动超时会发布零速度、关闭当前导航任务并进入 `FAULT_WAYPOINT_BRAKE_TIMEOUT`，不会把未确认停稳当成可转向或可进入下一段的完成条件。

### P2-02 `/odometry/fused` 的位姿和协方差不属于同一状态（近似修复）

- 历史证据：旧版 map correction 覆盖全局 pose 后保留本地协方差。
- 当前状态：`/odometry/fused` 不再原样保留 local pose covariance，而是把 local 与已接受视觉校正的 X、Y、yaw 三个对角方差相加；twist covariance 仍按 local EKF 语义发布。
- 未解决：当前近似没有把 local XY 协方差旋转到 map 坐标系，也没有保留交叉项或处理两个估计的相关性，现有测试尚未覆盖全局 covariance 语义。

### P2-03 视觉恢复计数不保证来自连续有效样本

- 历史证据：旧版视觉恢复计数在部分无效或中断样本后未统一清零。
- 当前状态：首次从正常视觉进入中断时会清零恢复计数，多数无效 frame/value 和时间戳异常也会进入中断处理。
- 未解决：`mark_visual_interrupted()` 只在 `interrupted=false` 时清零；恢复期间再次出现坏帧、异常增量或 tracking 丢失时，计数可能跨中断继续累计。tracking 超时时 `publish_raw_visual()` 还会直接返回，现有测试没有覆盖这一节点级序列。

### P2-04 map 校正会根据控制命令冻结全局平移（默认关闭，仍需实车验证）

- 当前状态：`hold_global_xy_during_turn` 默认值为 `false`，默认链路不会根据转向命令冻结全局 XY。显式启用时，节点仍会以低线速度、角速度门槛和 local odometry 位移上限监督 turn hold，位移超过 `turn_hold_max_translation_m` 会释放保持并记录告警。
- 残余风险：该选项仍依赖“原地转向”命令与实际车体运动一致；真实打滑场景需要继续实车验证，不能把可选保护逻辑当成通用运动估计。

### P2-05 Direct Visual Tracking 直通未经平滑的 ORB 校正（已解决）

- 历史证据：旧版 `map_odom_correction_node` 在 `direct_visual_tracking=true` 且视觉正常时直接执行 `map_from_odom_ = desired_map_from_odom_`，关键帧/特征跟踪抖动会原样进入 `/odometry/fused`。
- 当前状态：默认配置将 `direct_visual_tracking` 设为 `false`；该参数作为兼容入口保留但已弃用，即使显式开启也只给出告警。首个有效校正会直接建立 map/odom 初始锚点，之后的视觉校正均经过指数插值；map-change、恢复和驻停阶段继续使用各自的平滑时间常数。
- 残余注意：平滑参数仍需结合 ORB 输出频率和导航控制频率实车复核，不能用单元测试完全证明视觉抖动已满足车辆控制要求。

### P2-06 重复编码器帧仍会刷新 wheel 健康时间（已解决）

- 历史证据：旧版 wheel odometry 在判断 duplicate 前更新接收时间。
- 当前状态：重复帧不刷新 wheel 新鲜度，也不发布冻结的测量；只有有效积分或有效基线样本刷新健康时间。

### P2-07 路线发布没有任务 ID 和导航端确认（接收确认已实现，仍非完整任务协议）

- 历史证据：旧版路线发布后没有导航端确认。
- 当前状态：动态 `nav_msgs/msg/Path` 使用 Path 时间戳纳秒作为路线 ID，导航端会通过 `/waypoint_navigation/route_ack` 回传该 ID。路线编辑器当前实际等待的是 transient-local `/waypoint_path`，并比较时间戳与路径指纹来确认导航端已经接收；它尚未消费独立的 route-ack topic。
- 残余注意：这仍是路线接收确认，不是覆盖执行、完成和失败原因的完整任务协议。

### P2-08 无视觉降级没有限制在隧道所需的时间或距离内（已解决）

- 历史证据：旧版 `DEGRADED_NO_VISION` 分支没有应用无视觉预算。
- 当前状态：wheel + IMU 的无视觉运行同时受 `max_dead_reckoning_time_s` 和 `max_dead_reckoning_distance_m` 限制；超过预算进入 `FAULT`。纯 wheel fallback 还受独立的时间、距离、速度和 wheel-yaw 验证条件限制。
- 残余注意：默认参数仍需结合实际隧道长度和车辆速度复核，现有 bag 没有覆盖永久失视觉过程。

### P2-09 手动启动模式把已加载路线报告为加载失败（已解决）

- 历史证据：构造函数原先只有 `autostart && routeLoaded` 与 `else` 两个分支；当路线加载成功但 `autostart=false` 时，因 `route_file` 非空错误进入 `FAULT_ROUTE_NOT_LOADED`。
- 当前状态：初始化状态现在按路线是否加载成功和是否自动启动分别判断：空路线为 `WAITING_FOR_ROUTE`，文件加载失败为 `FAULT_ROUTE_NOT_LOADED`，手动启动且路线就绪为 `IDLE`，自动启动为 `WAITING_FOR_LOCALIZATION`。
- 验证：节点级 launch test 使用有效 CSV 和 `autostart=false`，通过 transient-local 状态话题确认初始状态为 `IDLE`。

### P2-10 转向稳定计时在每个控制周期被重新开始

- 证据：`TurnSettleController::begin_settling()` 每次调用都会清除内部计时器，而路径对齐和最终 yaw 控制在误差进入容差后每个控制周期都会再次调用它。
- 影响：默认 `turn_settle_dwell=0.30 s` 时，连续保持在角度容差内反而可能始终无法累计足够驻留时间，导致路径对齐或最终朝向阶段不能正常完成。
- 当前状态：这是重构前已经存在的控制逻辑缺陷。本轮 HSM 解耦按约束保持了控制行为，现有 `TurnSettleController` 单元测试只调用一次 `begin_settling()`，没有覆盖真实调用方式。

## P3

### P3-01 导航和融合主要按消息到达时间判断新鲜度（部分解决）

- 历史证据：旧版导航和融合只按回调到达时间维护输入新鲜度。
- 当前状态：导航、fusion gate 和 wheel odometry 已检查非零、单调、年龄及未来容差；消息到达时间只用于判断链路是否仍有新数据。
- 未解决：map correction 的 local 输入只检查非零和单调，视觉输入只检查非零与同步容差，尚未形成同样完整的测量时间戳策略。

### P3-02 IMU 输入的有限值和 frame 校验不完整（已解决）

- 历史证据：旧版 IMU filter 和融合入口对 finite、covariance、frame 及 optical frame 的校验不完整。
- 当前状态：IMU filter 在中值/均值滤波前检查时间戳、允许的 frame、向量和协方差；融合 gate 也检查 `frame_id`、有限值、幅值和测量时间戳，异常样本直接拒绝。

### P3-03 ORB 输出使用固定协方差

- 证据：`src/orb_slam3/orbslam3_ros2/src/stereo-inertial/stereo-inertial-node.cpp` 仍以配置的 pose/twist covariance 作为固定基线；当前只会按 KLT 兼容状态对不确定度做缩放。
- 触发：视觉质量随弱纹理、快速运动或重定位发生明显变化。
- 影响：协方差仍不能完整表达实时视觉质量；融合 gate 另外使用 tracking 状态、时间戳、增量残差和恢复门控，因此当前没有证据表明固定基线已单独造成控制失败。这一项仍是待改进项。

### P3-04 同一 D455 IMU 存在重复进入估计链的可选启动组合（已解决）

- 历史证据：旧版 bringup 在 `use_slam_imu=true` 时仍可能启动外部 IMU filter。
- 当前状态：`odometry_bringup.launch.py` 以 `use_imu=true and use_slam_imu=false` 作为外部 filter 的启动条件；开启 ORB 内部 IMU 时会省略外部 filter，默认配置也只选择一条路径。

### P3-05 ROS 时间与 steady clock 仍存在混用（保留设计风险）

- 当前证据：导航和串口周期调度使用节点 ROS clock；部分输入到达、串口健康和融合诊断仍使用 `steady_clock`。消息测量时间使用 ROS 时间并有单调/未来检查。
- 触发：启用仿真时间后 `/clock` 暂停、回退或跳变，ROS 时间等待与 steady-clock 健康计时可能产生不同步行为。
- 影响：这是仍需统一时间抽象和仿真时间测试的设计风险，不应再表述为“使用 wall timer”这一旧实现。

### P3-06 自动串口选择没有设备身份确认（已解决，通用回退仍可显式开启）

- 历史证据：旧版 `auto` 会从 `/dev/ttyUSB*` 或 `/dev/ttyACM*` 选择第一个匹配项。
- 当前状态：默认 `device=auto` 只接受唯一 `/dev/serial/by-id/*` 设备；多个稳定设备、无稳定设备或无法唯一识别时拒绝连接。只有显式设置 `allow_generic_auto_device=true` 才启用旧式通用设备探测。

### P3-07 最近 rosbag 没有覆盖完整任务关键状态

- 证据：`latest_navigation_bag/` 没有 `BRAKING_AT_WAYPOINT`、最终航向完成、`GOAL_REACHED`、map-change 和完整隧道失视过程，路线只完成约 `3.67/29.33 m`。
- 触发：用该 bag 判断停车、终点、视觉恢复和完整路线能力。
- 影响：现有记录不能验证这些状态，但不等于这些状态已经发生故障。

### P3-09 节点级关键流程缺少自动化覆盖

- 证据：新增 `ControlState` 测试覆盖了阶段互斥、等待消费、终点恢复和 reset，但仍属于纯状态对象测试；launch test 只覆盖手动路线初始 `IDLE` 和动态路线确认，没有完整 `RunControl()`、隧道失视到恢复、制动到转向再到完成、部分路线激活、时间戳异常和 wheel 隔离的节点级流程。
- 触发：多个辅助函数单独通过，但节点状态和参数组合发生冲突。
- 影响：关键流程只能依赖实车发现问题，现有单元测试不能证明整条链路行为。

### P3-10 无效 IMU 时间戳会先污染滤波状态再被拒绝（已解决）

- 历史证据：旧版 filter 在更新滤波历史后才检查无效 `dt`。
- 当前状态：现在在更新中值、均值、低通滤波器和 `last_stamp_` 之前检查时间戳是否有限、正值、单调且位于最大间隔内；拒绝样本不会污染后续滤波历史。

### P3-11 地图跳变授权与对应视觉帧没有绑定关系

- 证据：`src/fused_odometry/src/inputs.cpp:42-49` 接收 `/orbslam3/map_change` 的递增事件序号，`src/fused_odometry/src/visual.cpp:108-134` 检查并消费跳变宽限，但视觉 odometry 本身不携带对应序号或事件时间；map-change 与位姿仍是分别发布、分别调度的两个 topic。
- 触发：两个 topic 的消息调度顺序变化或 map-change 延迟到达。
- 影响：跳变宽限可能作用于错误帧；当前 bag 没有 map-change，未发现实际影响。

### P3-12 IMU 姿态解算仍以欧拉角一阶积分为核心（保留设计风险）

- 当前证据：`src/imu_rpy_filter/src/update.cpp:81-106` 根据 `tan(pitch)` 将机体系角速度转换为 `roll_rate/pitch_rate`，再直接更新 `roll_`、`pitch_`；仅对 `cos(pitch)` 做 `1e-3` 截断，并用加速度做互补修正。
- 触发：车辆出现较大俯仰、颠簸/振动或线加速度明显偏离重力时，欧拉坐标积分误差和坐标奇异性会被放大。
- 当前影响判断：D455 当前安装和地面车辆工况通常远离 ±90°，现有测试没有证明它已造成控制故障，因此属于 P3 设计风险，不是已确认的 P0/P1 故障。
- 建议：若要提高姿态解算鲁棒性，应将内部姿态状态改为四元数/旋转矩阵，用 gyro 增量积分并以重力方向做误差校正；保持现有 RPY、`/imu/filtered` 和参数接口不变，并补充大俯仰、振动和长时间积分测试。

### P3-13 原地转向安全判定阈值在规划器中重复硬编码（已解决）

- 历史证据：旧版 `planner_node.cpp` 曾使用固定 `0.18 rad`，与导航参数各自维护。
- 当前状态：规划器从 `arena_map.yaml` 读取 `planning.in_place_turn_heading_threshold_deg`，由 `config.cpp` 转换为弧度；`node.cpp` 将该配置传给覆盖/路径结果判定，不再在节点代码中硬编码 `0.18 rad`。
- 残余注意：规划器和导航仍是不同包，运行时阈值不会自动从一个包同步到另一个包；部署配置变更时需同时审查两侧参数。

### P3-14 普通目标超过 16 个时曾被直接拒绝（已解决）

- 历史证据：旧版 `Plan()` 在进入已有的逐点可达性选择前先拒绝 `planning_targets.size() > 16`，导致后面的非指数级分支无法作为大点集回退。
- 当前状态：已移除该前置拒绝。小点集既有逻辑不变；大点集进入同一有界逐点 A* 选择，并继续保留不可达目标延迟、隧道出入口成对和路径平滑检查。
- 验证：新增 17 个普通目标的 `shortest` 回归测试，确认规划成功、所有目标均进入访问序列且没有延迟目标；未把大点集送入状态压缩 DP。

### P3-15 Layer 1 小点集精确 DP 分支不可达

- 证据：`ArenaPlanner::Plan()` 将非 `layer2` 请求的 `planning_mode` 直接设为原始 `mode`，但精确 DP 条件同时要求 `mode == "layer1"` 和 `planning_mode == "shortest"`，两个条件无法同时成立。
- 影响：P3-14 的大点集回退已经生效，但注释宣称用于 Layer 1 小点集的精确开放 TSP 实际不会执行，当前仍会落入逐点选择逻辑。该条件在本轮拆分前已经存在，需在允许改变规划结果时单独修复并补充路线最优性回归测试。

## P4

### P4-01 导航节点职责过度集中（已解决节点内部耦合）

- 当前状态：`WaypointNavigator` 继续作为唯一 ROS 编排边界，以保留现有节点名、topic/service 和回调时序；实现按职责组织为入口 `waypoint_navigator.cpp`、初始化 `node_init.cpp`、路线 `route.cpp`、输入与运行许可 `runtime.cpp`、控制编排 `control.cpp`、动作 `maneuvers.cpp`、路径跟随 `path_following.cpp`、进度监督 `progress.cpp`、任务迁移 `navigation.cpp` 和输出 `output.cpp`。`ControlState` 以互斥阶段、终点阶段和等待动作取代散落标志，路线管理、输入缓存、路径控制和进度监督仍由独立对象维护并分别测试。
- 边界说明：节点持有一次导航任务的状态属于其本职，不再为了降低表面行数拆成多个互相同步的 ROS 节点。完整任务状态迁移的覆盖不足另列为 P3-09，不再混同为代码耦合问题。

### P4-02 融合门控节点职责和状态过度集中（已解决节点内部耦合）

- 历史证据：原 `FusionGateNode` 在单个源文件和类中平铺维护 40 余个状态标志、计数、残差与时间戳，并在各传感器回调间交叉修改。
- 当前状态：实现按职责组织为入口 `fusion_gate_node.cpp`、初始化 `node_init.cpp`、公共输入 `inputs.cpp`、视觉 `visual.cpp`、轮速 `wheel.cpp`、IMU `imu.cpp` 和健康输出 `health.cpp`；残差门控、鲁棒窗口、IMU bias、运动分类和降级决策继续由已有策略对象负责。节点内部状态已聚合为 `TrackingState`、`VisualState`、`RawVisualState`、`WheelState`、`ImuState`、`CommandState` 和 `MapChangeState`，不再由几十个同层成员表达；输入到达时间与测量时间戳分别通过统一的 `InputState`、`StampState` 管理。
- 边界说明：融合健康本来就需要读取多个传感器的当前状态，因此 `health.cpp` 对这些聚合状态的只读组合属于节点的编排职责，不适合再机械拆成更多 ROS 节点。相对结构拆分前的当前实现，本次纯拆分没有再改变节点名、可执行文件名、topic、参数或判定条件；相对 `HEAD` 的功能变化以 P0-P3 各条为准。

### P4-03 同一状态由多个节点分别维护

- 证据：视觉可用性由 ORB tracking、fusion gate 的视觉接收状态、map correction 的视觉超时和 navigator 的输入许可分别维护；静止状态由 IMU filter、map correction 和导航停车逻辑分别按不同阈值判断。
- 影响：排查视觉恢复或停车状态时仍需对齐多个节点的独立计时和阈值，单一状态话题不能完整解释各节点行为。
