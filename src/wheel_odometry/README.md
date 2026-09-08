# wheel_odometry

根据 STM32 上报的左右编码器累计计数计算差速轮式里程计。

## 接口

输入：

- `/cup_car_serial/encoder_ticks` (`std_msgs/msg/Int32MultiArray`)
- `data = [mcu_time_ms, sequence, left_total_ticks, right_total_ticks]`

输出：

- `/wheel/odom` (`nav_msgs/msg/Odometry`)
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)，包含采样新鲜度、丢帧、
  重建基线/计数跳变次数和当前动态方差
- 可选的 `odom -> base_link` TF；默认关闭，需要时设置 `publish_tf: true`

节点用相邻两帧累计计数求左右轮增量，并采用航向中点积分更新
`x/y/yaw`。速度使用 MCU 时间差计算，不受 ROS 接收调度抖动影响。丢失中间帧时，
累计计数仍能恢复完整路程。

发布的 `/wheel/odom` 时间戳也使用 MCU 测量时间：节点在首个有效区间建立一次
MCU 到 ROS 的时间锚点，之后按 MCU 毫秒计数（含 32 位回绕）推进测量时间；串口
接收时刻用于建立/校准该映射并诊断数据流新鲜度，不再逐帧冒充测量时刻。MCU
重启、乱序或异常导致编码器基线重建时，时间映射也重新锚定；映射器从五秒后开始
定期以初始锚点为长基线估计 MCU/ROS
频率比，并低通补偿晶振频偏，避免短窗口的毫秒量化误差累积成长期偏差。无法保证
单调的样本会丢弃并计入诊断，绝不发布未来时间。

节点会识别 32 位 MCU 时间、序号和编码器计数的正常回绕。MCU 重启、乱序、
非正时间差或超过 `max_wheel_speed_mps` 的计数跳变会触发重新建立编码器基线，
但不会清空已经累计的位姿。

当前比赛配置 `config/wheel_odometry.yaml` 使用实测物理轮距 `0.187 m`，并以
`yaw_slip_scale: 0.448` 修正原地转弯时轮胎侧向摩擦造成的差分角度偏大；它不改变
轮距和直线距离。这个比例来自最新 rosbag 中编码器约 `192.9 deg` 对视觉/IMU
约 `105 deg` 的动态转弯结果，后续仍应使用多次顺、逆时针转弯继续验证。代码头文件
里的默认值只是未加载 YAML 时的兜底值，现场以 YAML 为准。

## 运行

串口桥运行后启动：

```bash
ros2 launch wheel_odometry wheel_odometry.launch.py
```

查看输出：

```bash
ros2 topic echo /wheel/odom
```

默认配置不发布 TF，避免与 ORB-SLAM 或后续融合节点冲突。仅在轮式里程计作为
唯一里程计来源时，才在 `config/wheel_odometry.yaml` 中设置 `publish_tf: true`。

轮式里程计会随轮胎打滑、轮径误差和地面差异逐渐漂移。在当前比赛配置中，它只作为
短时辅助和健康监督输入，默认不进入主 EKF；不应被当作长期无漂移的绝对定位。只有
经过单独标定并确认误差边界后，才考虑把某个轮速分量用于有限的降级模式。

`/wheel/odom` 的协方差不是固定常数。位姿方差随累计轮上行程、累计转角、
序号缺口和异常重建基线持续增长；速度方差还会根据当前采样间隔和左右轮速度差
动态增加。因此融合节点可以在编码器丢包、急转弯或长期运行后自然降低轮速权重。
