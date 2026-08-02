# cup_car_serial

该 ROS 2 节点订阅 `geometry_msgs/msg/Twist`，并将速度转换成下位机串口协议。

默认接口：

```text
输入话题：/cmd_vel_nav
串口设备：/dev/ttyUSB0
波特率：115200，8N1
发送频率：20 Hz
命令超时：0.4 s
```

串口帧格式：

```text
vx,az\r\n
```

- `vx`：`Twist.linear.x`，单位 m/s。
- `az`：`Twist.angular.z`，单位 rad/s。
- 数值保留三位小数，例如 `0.250,-1.500\r\n`。

单独启动：

```bash
ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=auto \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav
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
```

连接正常时 `connected` 为 `true`，下位机应通过 `rx` 发布 `ENC,left_total,right_total`。

如果超过 `0.4 s` 没有收到新速度，节点会向串口发送零速度。下位机仍应实现独立通信看门狗，并切换到允许接收导航速度的工作模式。
