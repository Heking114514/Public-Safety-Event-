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

节点会识别 32 位 MCU 时间、序号和编码器计数的正常回绕。MCU 重启、乱序、
非正时间差或超过 `max_wheel_speed_mps` 的计数跳变会触发重新建立编码器基线，
但不会清空已经累计的位姿。

`wheel_track_m` 使用实测物理轮距 `0.254 m`。原地转弯时轮胎侧向摩擦会让编码器
差分角度偏大，因此 `yaw_slip_scale` 单独修正 yaw；它不改变轮距和直线距离。
当前 `0.33` 来自最新 rosbag 中编码器约 `192.9 deg` 对视觉/IMU 约 `105 deg`
的动态转弯结果，后续应使用多次顺、逆时针转弯继续验证。

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

轮式里程计会随轮胎打滑、轮径误差和地面差异逐渐漂移，适合与视觉/IMU 里程计融合，
不应被当作长期无漂移的绝对定位。

`/wheel/odom` 的协方差不是固定常数。位姿方差随累计轮上行程、累计转角、
序号缺口和异常重建基线持续增长；速度方差还会根据当前采样间隔和左右轮速度差
动态增加。因此融合节点可以在编码器丢包、急转弯或长期运行后自然降低轮速权重。
