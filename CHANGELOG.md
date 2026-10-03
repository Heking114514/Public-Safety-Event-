# Changelog

## 2026-10-04

- 新增 `scripts/record_camera_samples.sh` 交互式定点相机采样工具：仅启动 D455，
  不启动串口或运动控制；每次放置小车后录制一段独立的 RGB、对齐深度和标定
  rosbag，并在外接录制盘的会话目录保存样本备注与时间索引。
- 为相机采样脚本增加模拟 ROS 进程测试，覆盖完整样本落盘、RGB-only 模式和非法
  时长拒绝，确保采集话题不包含 `cmd_vel` 或串口控制链。
- 复查并收紧正式 `visual_navigation` 过弯控制器与
  `corner_turn_shunt_animation.html` 的一致性：默认 8mm 安全边、0.05m/s approach、
  更严格的 HTML 出口姿态、统一 0.19s 微动作段，并补充 8mm/出口条件回归测试。
- 保留一个 disabled 的真实到点姿态严格仿真用例，记录当前 153.5mm 车宽加 8mm
  安全边仍存在局部死角；该场景尚不能声明已按 HTML 完整通过。

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
  已用真实 `arena_path_planner` 路线和正式 `waypoint_navigator` 跑完整 19 点路线，
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
