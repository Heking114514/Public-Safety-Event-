# Changelog

## 2026-10-08

- 将 `navigation_mission_manager` 的总控节点、任务模型、障碍/事件/RFID 状态机、服务代理
  和测试播报器完整迁移到 C++17，ROS 接口和可执行文件名保持不变。
- 将 `vehicle_loopback_sim_node`、仿真运动/障碍模型和可选调试路线控制器迁移到 C++17；
  浏览器 GUI、launch、离线分析与兼容工具继续使用 Python。
- 两个包切换到 `ament_cmake`/`ament_cmake_python`，新增对应 GTest，并完成 C++ 总控与
  C++ loopback 的多段规划、ACK、运动、任务提交和驻停恢复组合验证。
- `pure_line_following` 按要求保持现有 Python 实现，不参与本轮迁移。
- 将 `rectangle_odometry_test` 的矩形路线发布/闭环监视节点和几何生成迁移到 C++17；
  primitive A* 实验控制器保留 Python 兼容入口。
- 修复 `start_visual_navigation.sh` 构建清单漏掉 `new_map_planner` 和
  `navigation_mission_manager` 的问题；现在在构建融合里程计前安装其依赖包，并纳入
  `--no-build` 运行产物核验。

## 2026-10-07

- 扩展 `export_rosbag.py`：支持 MJPG AVI 导出，并随分析产物保存 `/rosout` CSV/文本日志、
  下位机串口接收日志和原始运行清单，便于一次导出后复核视频、导航和 RFID 播报链路。
- 新增固定 RFID 任务点与 `/odometry/fused` 的离线误差分析，输出逐次读卡、逐点均值、
  重复读卡离散度、刚体对齐残差和可视化图，明确保留读卡器安装偏移这一测量假设。
- 将统一车辆配置中的左右编码器每圈计数更新为 2026-10-06 实测值：左 `368867.22`、
  右 `368883.7667`，轮式里程计分别注入左右标定参数。
- 在 `map_odom_correction` 增加 RFID 固定点位置校正：只修正 `map -> odom` 的 x/y，
  保留 IMU/局部 EKF 航向；同一点短时间重复上报按 3 秒冷却去重。
- RFID 地标校正现在等待 `/waypoint_navigation/motion_hold_state` 确认 `rfid` 驻停后才开始，
  在 3 秒驻停窗口内完成平滑位置修正，避免车辆尚未停稳时拉动融合位姿。
- 完成临时障碍三阶段闭环：首次封路触发停车、历史退回和 15.34 m 绕行；障碍离场后
  自动完成 12 个任务点、4 条隧道、24 条普通道路并回到距原点约 38 mm 的终点。
- 修复障碍绕行后的阶段交接死端：拒绝零阶段进展 partial 路线，并从刚完成路线有界
  后退到更早路口后规划下一阶段。
- 规划器和控制器新增共享的反向进站转弯语义，修复边缘道路倒回路口后紧接 90 度转弯
  进入 `TURN_BLOCKED`；动态障碍超过 120 秒未重现会释放，仍存在则由感知再次锁定。

## 2026-10-06

- 修复障碍恢复在首个路口锚点仍无可行路线时直接进入 `FAULT` 的问题：任务节点现在会
  请求控制器沿保留的实际行驶轨迹分段退到更早锚点并重复规划；倒退距离、次数和轨迹
  范围都有边界，耗尽后保持停车，不允许任务层绕过控制器直接发速度。
- 扩展倒退从实测姿态匹配历史起点，并按相反时间顺序回放实际限速控制命令，保留紧
  路口轨迹中的前进/倒车/转向微调。loopback 临时封路回归连续退回三段，在发车点生成
  `15.70 m` 绕行路线并恢复执行。
- 修复规划器把“车头朝南但需先向北退 5 cm 到路口”的吸附段编码成普通前进路线并
  反复返回 `COMPLETE` 的问题；不可由 `Path` 表达的起步倒车现在明确触发继续后退。

## 2026-10-05

- 新增独立 `navigation_mission_manager` 实车功能包，作为比赛任务状态的唯一拥有者；统一
  管理三阶段进度、规划触发、route ACK、障碍退回后的自动重规划、故障驻停和人工恢复。
- 扩展 `mission_control_interfaces`：新增任务命令/状态、结构化事件候选和带结果 ACK 的
  播报 Action；任务节点完成置信度门限、位置去重和播报调度，识别与音频实现保持独立。
- `new_map_planner` UI 改为操作台，完整巡视只发送任务命令并显示统一状态；关闭 UI 不再
  结束后台比赛任务。实车入口同步启动规划器、任务节点和 UI。
- 更新实测整车参数：最大外廓宽度由 `153.5 mm` 改为 `125.0 mm`，轮轴中点到
  `base_link` 车体中心的前向偏移由 `50 mm` 改为 `30 mm`；规划器、控制器与
  loopback 真值底盘继续从 `vehicle_config` 同一来源读取。
- `new_map_planner` 增加正式 `layer1 -> layer2 -> layer3` 任务层：先访问任务点，再显式
  穿越四条隧道，最后禁止借隧道连接、补齐普通道路并回程；每段从最新实测位姿重规划，
  仅在 route ACK 与 `GOAL_REACHED` 后提交进度，故障只暂停不自动反复发车。
- `vehicle_loopback_sim` 完整闭环与新旧控制器对比入口默认执行同一套三阶段任务，保留
  单次 `targets/coverage/mission` 模式和多控制器同步完成参数。
- 三阶段正式控制器闭环依次完成 `13.70 m`、`9.04 m`、`15.74 m` 路线并回到发车点，
  最终位置误差约 `38 mm`，全程无 `TURN_BLOCKED` 或导航故障。
- loopback mission 改用 `PlanRoute` 和 `map` 真值位姿，GUI 同时支持新地图的命名节点、
  `lanes`、12 个任务点以及 `/new_map_planner/*` 规划话题。

## 2026-10-04

- 新增独立 `new_map_planner` 实车功能包：道路图负责全局选择，航向进入搜索状态，
  车身几何层独立验证直行、动态障碍和标定转弯扫掠；提供显式动作接口、实车 UI、
  route ACK/驻停闭环、前方障碍映射和独立启动脚本，不包含模拟或路线播放模式。
- 新增正式新控制器与 `71ddfde` 老原地转向控制器的并行闭环比较：两套控制器
  使用同一规划路线、独立真值底盘与隔离话题，分别提供 `8765`/`8766` 实时 GUI；
  mission 等待两边同时到达后才推进下一规划阶段，避免比较过程相互抢占命令或路线。
- 将正式过弯控制器从逐步贪心切换改为停车后异步状态格规划：搜索显式包含轮轴位姿、
  当前受限速度、前进/倒车/原地转向原语和完整车身 8mm 安全包络；运行时每个控制周期
  复核下一步，危险或计划衔接时先硬停车再从零速重规划，避免减速惯性把车推过边界。
- 新控制器已用正式道路图的 19 点、13.70m `layer1` 路线和带
  `base_link` 杠臂的真值底盘完整跑到 `GOAL_REACHED`；覆盖左右转、偏移入口、凹角
  切点续接和越界姿态失败关闭。老控制器仍复现首弯前制动卡死，作为 A/B 基线保留。
- 新增 `scripts/record_camera_samples.sh` 交互式定点相机采样工具：仅启动 D455，
  不启动串口或运动控制；每次放置小车后录制一段独立的 RGB、对齐深度和标定
  rosbag，并在外接录制盘的会话目录保存样本备注与时间索引。
- 为相机采样脚本增加模拟 ROS 进程测试，覆盖完整样本落盘、RGB-only 模式和非法
  时长拒绝，确保采集话题不包含 `cmd_vel` 或串口控制链。
- 保留 `corner_turn_shunt_animation.html` 的停-查-微动作原则与严格出口姿态，但不再
  直接使用其非物理关键帧；正式控制器按真实差速运动学、30Hz 限加速度和 0.20s
  原语在线生成可执行路径。

## 2026-10-03

- 将 `corner_turn_shunt_animation.html` 的停-检查-小弧线/微转/慢爬/小倒退策略移入正式
  `visual_navigation/waypoint_navigator`：新增 `CornerShuntController`、`SHUNT_TURN`
  状态和安全预测，按轮轴中点积分并使用中央车体参数中的 `base_link` 前偏预测整车矩形包络。
- 收紧 `CornerShuntController` 的出弯交棒条件：不再只靠宽松航向误差提前返回
  `PATH_ALIGNED`，必须确认车体中心已经进入出弯通道；第二个右转不再从贴内角姿态进入
  `TURN_BLOCKED`。
- 完整闭环 launch 继续默认使用正式 `waypoint_navigator`，`loopback_route_controller`
  仅保留为 debug；loopback 真值里程计同步改为按轮轴积分后发布 `base_link`，
  避免仿真绕过车体前偏/杠臂效应。
- 新增/更新 `corner_shunt`、`waypoint_navigator` launch、`vehicle_config` 和
  `vehicle_loopback_sim` 契约测试。`planner_control_loopback.launch.py gui:=false`
  已用真实道路图路线和正式 `waypoint_navigator` 跑完整 19 点路线，
  最终到达 `GOAL_REACHED`。
- 新增 `vehicle_loopback_sim` 真值里程计仿真包：订阅 `/cmd_vel_nav`，
  发布 `/odometry/fused`、`/imu/control`、融合健康状态、执行器健康和理想控制遥测，
  便于在不启动实车、ORB 和融合链路时闭环测试规划器与 `waypoint_navigator`。
- 将 loopback mission 从单次 `layer1` 请求改为默认顺序执行
  `layer1,layer2,layer3`：每段 `GOAL_REACHED` 后用真值里程计反算 arena 起点，
  并把已确认道路覆盖传入下一次规划请求。
- 修复 loopback mission 使用 C++ 风格日志参数导致 Python 节点在第一段规划返回后崩溃的问题；
  增加 mission 日志签名单测，避免后续分层接力再次被日志调用打断。
- 收敛正式 `waypoint_navigator` 控制器速度与过弯交棒：默认/最高线速度降至
  `0.10 m/s`，增加制动余量，降低横向偏差最低速度，并将 shunt 内部脱困前进候选
  从最高 `0.040 m/s` 压到 `0.030 m/s`；shunt 几何安全边保持 `6 mm`，避免窄弯误判无解。
- 为 `vehicle_loopback_sim` 增加浏览器实时 GUI：默认随 loopback launch 启动，
  显示规划地图、导航路径、控制器执行路径、真值车体轮廓、历史轨迹、速度命令和导航状态。
- 修正 loopback GUI 的场地显示：按 `arena_map.yaml` 绘制真实道路/障碍边界，
  并把 planner 的 `arena` 路径转换到 odom 使用的 `map` 坐标；完整闭环 launch
  默认发送一次真实 `layer1` planner 请求并激活给正式 `waypoint_navigator`。
- 新增 `loopback_route_controller` 仅作为 debug/demo 路线执行器保留；完整闭环仿真
  默认接入 `visual_navigation/waypoint_navigator`，避免仿真包绕过正式导航控制器。
- 修复 `waypoint_navigator` 在预转弯刹车安全边界上可能卡住的问题：到点/过点
  判断加入微小浮点容差，避免速度已降到 0 但剩余距离刚好等于刹车安全余量时无法进入转弯。
- 整理 `vehicle_config` 中央车体参数包：保留 `vehicle_physical.yaml`
  作为车体尺寸、轮系、跟踪点和相机安装位姿的唯一来源，并补充依赖契约测试，
  防止消费者包漏声明 `vehicle_config`。
- 去除融合节点头文件中残留的轮轴到 `base_link` 物理默认值，并让障碍物投影脚本
  默认读取中心相机前向偏移，避免离线/规划辅助工具继续散落车体参数。

## 2026-10-01

- 纯巡线实车调试确认：此前“走着走着偏右”不是串口掉线或积分项发散，
  而是视觉估计中心与小车真实中心存在稳定静态偏差；在
  `pure_line_following` 中新增 `lateral_error_bias_m`，实车配置设为
  `+0.015 m` 后，小车可稳定走在赛道中间。
- D455 aligned depth 已接入纯巡线，但只作为可信直道中心修正；深度投影被限制在
  RGB 已选主赛道 component 附近，路口/宽白块默认回退 RGB trunk，避免 depth 把
  原本可走的路口帧判成 `UNSAFE_EDGE`。
- 最新实车 rosbag 分析确认：串口连接、执行器健康和命令有效状态全程正常；
  停车来自视觉安全判断，在赛道可见宽度破碎/不足时主动输出零速度。
- 纯巡线白色赛道分割新增浅绿/泛黄地面排除：在原亮度与通道差判断外，
  加入归一化 RGB 绿占比、RGB green-opponent 和 Lab-a 中性色度约束；最新
  rosbag 回归保持主赛道连续，同时剔除远处浅绿色偏色区域。

## 2026-09-29

- 新增独立离线 RGB 白色赛道分类器：使用相机地面投影和绿色背景邻接约束，
  将局部形状分类为直行、左右直角、T 字路口、十字路口、失线和未知；
  新增真实 rosbag 的 AVI/CSV 渲染脚本和合成拓扑测试，不修改生产 ROS 节点。
- 新增 `pure_line_following` ROS 2 功能包：只订阅 RGB 图像和 CameraInfo，
  不订阅里程计、TF、地图或规划器，直接发布独立 `Twist` 寻迹控制话题。
- 联合实车启动文件同时拉起 D455 RGB 相机、纯寻迹节点和串口桥，并在等待
  相机输入时发布明确状态。

## 2026-09-27

- 为 `rectangle_odometry_test` 增加独立的 A*/差速动作原语矩形实验模式。
  统一启动脚本新增 `--primitive-rectangle-test`，实验模式接管 `/cmd_vel_nav`
  并关闭 `waypoint_navigator`，使用 `/odometry/fused` 闭环重规划和安全门控；
  默认导航控制链不变。
- 导航默认继续以 `base_link` 为控制点；启动脚本新增 `--tracking-point-offset-x/y` 并写入 run manifest，便于只在原地 pivot A/B 测试时切到轮轴参考。
- 矩形测试包新增 `corner_mode:=arc`，用密集普通航点生成 `base_link` 中心圆角闭环路线，并将直线段按 `straight_step_m` 拆分，避免第一段长直线提前套用圆弧曲率。
- 根据 2026-09-27 实车 rosbag 的 `0.15 m/s`、`0.65 rad/s` 弯道命令和约 `0.12 m` 横向误差，增加独立慢速圆弧测试 profile；统一启动脚本支持记录式透传 `--navigation-parameters-file`，不改变生产导航默认参数。
- 将矩形圆角测试恢复为 `0.25 m` 半径；测试 profile 让直线保持 `0.04 m/s`，通过曲率限速让圆角降到约 `0.03 m/s`，避免整条路线过慢。
- 调整 `rectangle_odometry_test` 的 primitive 矩形控制：收紧转弯目标航向判定，
  接通规划/重规划容差参数，并在实际航向未达到当前转弯目标时保留低速角向修正，
  避免执行器转角不足后提前恢复直行。
- 将 primitive 控制逻辑与 ROS2 适配节点拆分，避免单文件超过工程规范限制；
  运行入口和参数保持不变。
- 按矩形实车测试调慢 primitive 转弯：直线动作默认提升到 `0.06 m/s`，
  曲率动作改为 `3/6/12°`、原地转动步长改为 `6°`；直线段增加
  `1.5°` 死区、`1.5` 增益和 `8°/s` 限幅的小角度航向闭环，弯道补偿限速改为
  `10~25°/s`。
- 修复融合层把矩形测试 `0.06 m/s` 边走边转误判为原地转弯的问题：
  `wheel_in_place_max_linear_speed_mps` 收紧到 `0.02`，保留真正 pivot 的轮速清零保护。

## 2026-09-16

- 修复视觉导航启动脚本的残留清理顺序：构建完成后先停止旧导航 ROS 进程，再等待 `.navigation.lock` 释放，避免上一次启动残留导致新启动直接报 lock 占用。
- 收紧导航到点释放窗口：终点 release 从 `0.10 m` 收到 `0.05 m`，过点横向走廊从 `0.06 m` 收到 `0.045 m`，默认 CSV 航点 tolerance 从 `0.08 m` 收到 `0.04 m`，减少融合里程略偏大时的提前到点。
- 基于最新实车 rosbag 调整拐点控制：转向监督恢复不再打断 `ALIGN_PATH` 进入直行，精转最低角速度提高到实测可跨过底盘死区的 `0.50 rad/s`，并收紧停车拐点进站段角速度夹限，避免刹车阶段提前切弯。
- 按现场满负载实测串口时间戳滞后调整融合新鲜度阈值：BMI088/导航 IMU 窗口放宽到 `0.30 s`，EKF sensor timeout 对齐到 `0.35 s`，raw visual 单帧容忍放宽到 `0.45 s`，避免健康 50Hz 数据被误判为超时。
- 新增独立虚拟 `/odometry/fused` 发布脚本，用于在不启动真实融合/下位机链路时隔离测试规划器前端和导航输入。
- 基于最新实车 rosbag 调整原地转弯参数：提高导航 pivot 命令和精转最低角速度，并让转弯监督使用实测底盘 yaw-rate 估计，避免把低速转弯误判为无进展。
- 融合层 turn-hold 阈值按实测低速 pivot 下调，并放宽原地转弯期间 local 位移容差，减少 ORB 原地旋转 XY 漂移进入 `/odometry/fused`。
- 轮式里程计按最新 bag 校正编码器计数极性：左右距离比例改为负号并允许 signed scale，使直行 `vx` 和转弯 yaw 与视觉/IMU 同向；同时降低实际慢速 pivot 的 wheel-vx 置零阈值并减少普通视觉帧间隙恢复告警。

## 2026-09-15

- 串口上位机新增 BMI088 `ATT` 解析链路，使用下位机已处理的 yaw、gyro-z 和 bias，发布 `/cup_car_serial/bmi088_attitude`，并加入 MCU 时间到 ROS 时间的单调映射。
- 融合层切到真实物理安装：`base_link` 为几何中心，D455 位于前向 `0.055 m`；默认融合 ORB 速度、轮式 `vx` 和 BMI088 yaw/yaw-rate，轮式 yaw 保持诊断-only。
- 融合层去除对导航速度命令的依赖，轮速转弯误差通过 IMU 观测转弯时降权/置零处理，降级融合状态不耦合控制层限速。
- 融合里程计输出改为发布时单调时间戳，避免 local odom 未更新的定时发布周期重复 stamp 触发导航层非单调里程计告警。
- 融合包测试构建局部压制 Humble vendored gtest 的外部 deprecation warning，保留包内 C++ 警告检查。
- 视觉导航启动脚本将 ORB-SLAM3 的 Pangolin/Sophus CMake 参数限定到 `orbslam3` 包，避免无关融合包产生 unused CMake 变量告警。
- 导航层去除融合降级限速和 transient localization bridge；允许的 `DEGRADED_*` 状态正常行驶，`FAULT`、状态陈旧、无效位姿立即停车等待或锁存。
- 规划器性能优化：起点投影改为局部窗口搜索，水平/垂直路线段增加车体扫掠矩形快速碰撞判定，heading-aware A* 支持按出发约束缓存，并用候选下界剪枝减少无效搜索。
- 规划器构建警告清理：更新算法包 CMake 最低版本，固定 ROS 生成链路的 Python 查找策略，并局部压制 Humble vendored gtest 的外部 deprecation warning。
- 规划器失败日志补充请求上下文，输出模式、激活标志、请求/投影起点、目标数、覆盖进度和延迟目标数，便于现场定位不可达请求。
- 规划器节点启动时预热静态三阶段路径缓存，避免第一次预览或实车开始导航时把冷启动 A* 代价暴露给前端。
- 仿真前端的手动朝向输入改为 90 度吸附，避免预览生成实车不会执行的非格点初始姿态。
- 串口上位机对不可用 BMI088 `ATT` 帧输出缺失状态位和 startup bias 进度，现场可直接区分未静止标定、饱和和采样超时。
- 清理视觉导航启动链路中剩余包的 CMake/gtest deprecation 告警，保持运行日志只暴露真正的算法和硬件状态。
- 更新相关配置、测试和文档，并清理调试遗留空文件。
