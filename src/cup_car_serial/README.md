# cup_car_serial

该 ROS 2 节点将导航速度和 IMU 姿态转发给下位机，并解析下位机回传的轮编码器与控制遥测。

默认接口：

```text
输入话题：/cmd_vel_nav
输入话题：/imu/rpy
输出话题：/cup_car_serial/encoder_ticks
输出话题：/cup_car_serial/control_telemetry
输出话题：/cup_car_serial/actuator_healthy
输出话题：/cup_car_serial/actuator_tracking_status
串口设备：`auto`；默认只接受唯一的 `/dev/serial/by-id/*` 设备
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
  未急停、速度命令未超时且左右轮跟踪诊断健康时才为 `true`。它和只表示串口
  文件已打开的 `/cup_car_serial/connected` 含义不同。
- `/cup_car_serial/actuator_tracking_status` 给出轮速跟踪状态和故障轮，例如
  `SUSPECT:LEFT_NO_RESPONSE+RIGHT_OK`、`RECOVERING` 或 `LATCHED:...`。

轮速跟踪只用于执行器健康诊断，不参与里程计或定位融合。默认在目标轮速绝对值
达到 `0.08 m/s` 后才检查：实测绝对值低于 `0.03 m/s` 为无响应，方向相反为
方向错误，误差超过 `max(0.15 m/s, 0.8 * |target|)` 为严重跟踪误差。异常必须
持续 `1.5 s` 才会令 `actuator_healthy=false`，因此启动延迟、单帧编码器漂移和
瞬时负载变化不会停车。

遥控模式、急停或下位机命令失效时不具备跟踪判定前提，状态显示
`INACTIVE_CONTROL_STATE` 并清除本轮诊断历史；解除后从新的启动宽限开始，不会
因急停期间残留目标轮速而锁存故障。重复或乱序 CTL 帧不累计异常时间，也不刷新
遥测健康时间；MCU 重启会清除未完成的诊断窗口。

故障停车后，目标轮速保持死区内 `0.5 s` 可进行一次自动重试；只有重新连续
正常跟踪 `3.0 s` 才会恢复重试额度。默认最多自动重试两次，之后状态锁存为
`LATCHED`，需重启或重连串口节点复位，避免故障与零速恢复无限振荡。参数集中在
`config/cmd_vel_serial.yaml`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `allow_generic_auto_device` | `false` | `device=auto` 时是否允许无身份的 ttyUSB/ttyACM 探测；默认只接受唯一 `/dev/serial/by-id` 设备 |
| `tracking_command_deadband_mps` | `0.08` | 启用单轮诊断的最低目标速度 |
| `tracking_response_floor_mps` | `0.03` | 判定轮子无响应的实测速度上限 |
| `tracking_severe_absolute_error_mps` | `0.15` | 严重误差绝对门槛 |
| `tracking_severe_relative_error` | `0.80` | 严重误差相对门槛 |
| `tracking_startup_grace_s` | `1.0` | 进入可诊断状态后的启动宽限 |
| `tracking_fault_persistence_s` | `1.5` | 异常持续到故障的时间 |
| `tracking_recovery_stop_s` | `0.5` | 停驶后允许重试的滞回时间 |
| `tracking_retry_rearm_s` | `3.0` | 清除历史重试次数所需的正常跟踪时间 |
| `tracking_maximum_recovery_attempts` | `2` | 锁存前允许的自动重试次数 |

单独启动：

```bash
ros2 launch cup_car_serial cmd_vel_serial.launch.py \
  device:=auto \
  baud_rate:=115200 \
  topic:=/cmd_vel_nav \
  rpy_topic:=/imu/rpy
```

实车建议将 `device` 显式设置为实际的 `/dev/serial/by-id/...` 路径；只有在确认
设备枚举环境受控时才启用 `allow_generic_auto_device:=true`，让 `auto` 回退探测
`/dev/ttyUSB*` 或 `/dev/ttyACM*`。

## 使用 Python 模拟导航速度

终端 1 持续发布测试速度：

```bash
cd /path/to/your/workspace
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
cd /path/to/your/workspace
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
ros2 topic echo /cup_car_serial/actuator_tracking_status
```

连接正常时 `connected` 为 `true`。最新下位机固件每 50 ms 回传一次 `ENC`，
每 100 ms 回传一次 `CTL`；串口桥会将原始帧同时保留在 `rx` 中，并发布结构化话题。
固件默认处于遥控模式，切换到导航模式前 `actuator_healthy` 会保持 `false`。

如果超过 `0.4 s` 没有收到新速度，节点会向串口发送零速度。`/imu/rpy` 超过
`rpy_timeout_s`（默认 0.4 s）未更新时，节点暂停发送 RPY 帧。下位机仍应实现独立通信看门狗，并切换到允许接收导航速度的工作模式。
