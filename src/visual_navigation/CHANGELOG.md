# visual_navigation Changelog

## 2026-09-16

- 根据最新实车 rosbag 修正停车路口控制：转向进度恢复保持原地对齐状态，原地精调最低角速度提高到 `0.50 rad/s`，停车拐点进站段角速度按线速度收紧。
- 将导航接收 `/imu/control` 的新鲜度窗口放宽到 `0.30 s`，匹配满负载下 BMI088 ATT 测量时间戳约 `0.21 s` 的实际延迟。

## 2026-09-15

- 更新融合健康策略相关测试：允许的 `DEGRADED_*` 状态不再携带速度缩放，定位陈旧、融合状态超时和无效位姿立即停车等待，融合 `FAULT_*` 继续锁存任务。
- 清理 README、导航代码说明和默认配置中的旧 transient localization grace / fusion speed scale 表述，明确导航只消费 `/imu/control`，D455 前向偏移为 `0.055 m`。
