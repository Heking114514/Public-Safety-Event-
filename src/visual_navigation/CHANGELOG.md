# visual_navigation Changelog

## 2026-10-04

- 对 `CornerShuntController` 按 `corner_turn_shunt_animation.html` 收紧参数：
  默认 8mm 道路安全边、0.05m/s approach、严格出弯目标姿态和 0.19s 微动作段；
  倒退恢复改为优先反向微转，并补充 8mm 安全边与 HTML 出口回归测试。
- 新增从真实到点姿态出发的控制器级仿真用例，当前作为 disabled 保留：
  8mm 安全边和现有 153.5mm 车宽下，严格 HTML 出口仍会出现局部死角，需要后续
  以规划式动作序列或调整车体/安全边参数继续解决，未作为已完成能力声明。
- 收紧正式 `waypoint_navigator` 的 `CornerShuntController` 出弯完成条件，
  防止在残余航向误差过大时提前切回直线寻迹并把下一路口带到内角边缘。
- 新增 `corner_shunt` 回归测试，覆盖“大残余航向不能交棒”和“受约束出弯姿态可以交棒”。

## 2026-09-16

- 根据最新实车 rosbag 修正停车路口控制：转向进度恢复保持原地对齐状态，原地精调最低角速度提高到 `0.50 rad/s`，停车拐点进站段角速度按线速度收紧。
- 将导航接收 `/imu/control` 的新鲜度窗口放宽到 `0.30 s`，匹配满负载下 BMI088 ATT 测量时间戳约 `0.21 s` 的实际延迟。

## 2026-09-15

- 更新融合健康策略相关测试：允许的 `DEGRADED_*` 状态不再携带速度缩放，定位陈旧、融合状态超时和无效位姿立即停车等待，融合 `FAULT_*` 继续锁存任务。
- 清理 README、导航代码说明和默认配置中的旧 transient localization grace / fusion speed scale 表述，明确导航只消费 `/imu/control`，D455 前向偏移为 `0.055 m`。
