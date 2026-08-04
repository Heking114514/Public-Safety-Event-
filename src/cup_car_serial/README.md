# cup_car_serial

该 ROS 2 节点将导航速度和 IMU 姿态转发给下位机，并解析下位机回传的轮编码器计数。

默认接口：

```text
输入话题：/cmd_vel_nav
输入话题：/imu/rpy
输出话题：/cup_car_serial/encoder_ticks
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
```

- `vx`：`Twist.linear.x`，单位 m/s。
- `az`：`Twist.angular.z`，单位 rad/s。
- 数值保留三位小数，例如 `0.250,-1.500\r\n`。
- `roll`、`pitch`、`yaw` 来自 `/imu/rpy`（`geometry_msgs/msg/Vector3Stamped`），单位 rad。
- 编码器输出类型为 `std_msgs/msg/Int32MultiArray`，`data` 固定为
  `[sample_time_ms, sample_sequence, left_total_ticks, right_total_ticks]`。
  节点兼容旧固件的 `ENC,left_total,right_total`，其前两项补零。

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
```

连接正常时 `connected` 为 `true`。最新下位机固件每 50 ms 回传一次
`ENC,sample_time_ms,sample_sequence,left_total,right_total`；串口桥会将其同时保留在
`rx` 中，并解析发布到 `encoder_ticks`。

如果超过 `0.4 s` 没有收到新速度，节点会向串口发送零速度。`/imu/rpy` 超过
`rpy_timeout_s`（默认 0.4 s）未更新时，节点暂停发送 RPY 帧。下位机仍应实现独立通信看门狗，并切换到允许接收导航速度的工作模式。
