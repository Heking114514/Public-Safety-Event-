# cup_car_serial

该 ROS 2 节点将导航速度和 IMU 姿态转发给下位机，并解析下位机回传的轮编码器与控制遥测。

默认接口：

```text
输入话题：/cmd_vel_nav
输入话题：/imu/rpy
输出话题：/cup_car_serial/encoder_ticks
输出话题：/cup_car_serial/control_telemetry
输出话题：/cup_car_serial/actuator_healthy
串口设备：/dev/ttyUSB0
波特率：115200，8N1
发送频率：20 Hz
命令超时：0.4 s
```

串口帧格式：

```text
vx,az\r\n
RPY,roll,pitch,yaw\r\n
ENC,sample_time_ms,sample_sequence,left_total,right_total\r\n
CTL,time_ms,sample_sequence,mode,estop,rx_valid,rx_age_ms,rx_vx_mmps,rx_wz_mradps,target_left_mmps,target_right_mmps,measured_left_mmps,measured_right_mmps,pwm_left,pwm_right\r\n
```

- `vx`：`Twist.linear.x`，单位 m/s。
- `az`：`Twist.angular.z`，单位 rad/s。
- 数值保留三位小数，例如 `0.250,-1.500\r\n`。
- `roll`、`pitch`、`yaw` 来自 `/imu/rpy`（`geometry_msgs/msg/Vector3Stamped`），单位 rad。
- 编码器输出类型为 `std_msgs/msg/Int32MultiArray`，`data` 固定为
  `[sample_time_ms, sample_sequence, left_total_ticks, right_total_ticks]`。
  节点兼容旧固件的 `ENC,left_total,right_total`，其前两项补零。
- 控制遥测输出类型为
  `mission_control_interfaces/msg/ControlTelemetry`，速度整数毫单位会转换为
  `m/s` 和 `rad/s`。它保留下位机模式、急停、命令年龄、轮速目标/实测值和PWM。
- `/cup_car_serial/actuator_healthy` 只有在控制遥测新鲜、下位机处于导航模式、
  未急停且收到的速度命令未超时时才为 `true`。它和只表示串口文件已打开的
  `/cup_car_serial/connected` 含义不同。

单独启动：

```bash
ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=auto \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav \
  rpy_topic:=/imu/rpy
```

## 使用 Python 模拟导航速度

终端 1 持续发布测试速度：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/test_cmd_vel_nav.py \
  --linear 0.03 \
  --angular 0.10 \
  --rate 10 \
  --enable-output
```

终端 2 启动串口节点：

```bash
cd /home/j/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=auto \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav
```

按 `Ctrl+C` 关闭 Python 发布器时，它会连续发送 5 次零速度后退出。

检查连接和下位机回传：

```bash
ros2 topic echo /cup_car_serial/connected --once
ros2 topic echo /cup_car_serial/rx
ros2 topic echo /cup_car_serial/encoder_ticks
ros2 topic echo /cup_car_serial/control_telemetry
ros2 topic echo /cup_car_serial/actuator_healthy
```

连接正常时 `connected` 为 `true`。最新下位机固件每 50 ms 回传一次 `ENC`，
每 100 ms 回传一次 `CTL`；串口桥会将原始帧同时保留在 `rx` 中，并发布结构化话题。
固件默认处于遥控模式，切换到导航模式前 `actuator_healthy` 会保持 `false`。

如果超过 `0.4 s` 没有收到新速度，节点会向串口发送零速度。`/imu/rpy` 超过
`rpy_timeout_s`（默认 0.4 s）未更新时，节点暂停发送 RPY 帧。下位机仍应实现独立通信看门狗，并切换到允许接收导航速度的工作模式。
