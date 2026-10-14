# cup_car_serial

该 ROS 2 包转发导航速度和 IMU 姿态，解析下位机遥测，并提供车载语音串口接口。

默认接口：

```text
输入话题：/cmd_vel_nav
输入话题：/imu/rpy
输出话题：/cup_car_serial/encoder_ticks
输出话题：/cup_car_serial/rfid
输出话题：/cup_car_serial/rfid_number
输出话题：/cup_car_serial/rfid_uid
输出话题：/cup_car_serial/bmi088_attitude
输出话题：/cup_car_serial/bmi088_raw_imu
输出话题：/cup_car_serial/control_telemetry
输出话题：/cup_car_serial/actuator_healthy
输出话题：/cup_car_serial/actuator_tracking_status
串口设备：`auto`；默认只接受唯一的 `/dev/serial/by-id/*` 设备
波特率：115200，8N1
发送频率：20 Hz
命令超时：0.4 s
语音服务：`/cup_car_serial/speak_text`，接收 UTF-8 文本并转为下位机 GBK TTS 帧
```

串口帧格式：

```text
vx,az\r\n
RPY,roll,pitch,yaw\r\n
ENC,sample_time_ms,sample_sequence,left_total,right_total\r\n
IMU,sample_time_ms,sample_sequence,status,gx,gy,gz,temp_cC\r\n
ATT,sample_time_ms,att_sequence,status,yaw_mdeg,gyro_z_mdps,bias_z_mdps,startup_samples\r\n
CTL,time_ms,sample_sequence,mode,estop,rx_valid,rx_age_ms,rx_vx_mmps,rx_wz_mradps,target_left_mmps,target_right_mmps,measured_left_mmps,measured_right_mmps,pwm_left,pwm_right\r\n
RFID,time_ms,card_sequence,status,protocol_status,tag_type,sak,number,uid_hex\r\n
```

- `vx`：`Twist.linear.x`，单位 m/s。
- `az`：`Twist.angular.z`，单位 rad/s。
- 数值保留三位小数，例如 `0.250,-1.500\r\n`。
- `roll`、`pitch`、`yaw` 来自 `/imu/rpy`（`geometry_msgs/msg/Vector3Stamped`），单位 rad。
- RFID 有效新读卡事件只发布到 `/cup_car_serial/rfid_number` 和 UID 话题。任务去重、
  驻停时间、播报文本和恢复行驶由 `navigation_mission_manager` 负责。串口节点的
  `/cup_car_serial/speak_text` service 将 UTF-8 转为 GBK，并异步等待 MCU 状态反馈。
  输入限制为 1..128 UTF-8 字节，转码后最多 128 GBK 字节；拒绝控制字节、NUL 和 `<`/`>`。
  命令为 `TTS2,<非零uint32编号>,<GBK文本>\r\n`，反馈为 `TTS,<编号>,<状态>\r\n`。
  `QUEUED`、`PLAYING` 不结束请求，仅匹配编号的 `ESTIMATED_DONE` 返回成功。
  MCU 按每字 800 ms 加 1500 ms 估算占用，服务成功不是扬声器真实播放完成确认。
  未获接收反馈时首秒内每 200 ms 重发同一编号，接收后停止重发；总超时 120 秒。
  `BUSY`、`INVALID`、`ERROR`、断连或超时返回失败，已有待完成请求时新请求立即失败。
  等待期间周期速度与传感器反馈继续工作；需要配套 TTS2 MCU 固件，旧固件会超时。
- 编码器输出类型为 `std_msgs/msg/Int32MultiArray`，`data` 固定为
  `[sample_time_ms, sample_sequence, left_total_ticks, right_total_ticks]`。
  节点兼容旧固件的 `ENC,left_total,right_total`，其前两项补零。
- `RFID` 来自下位机 UART10 读卡器。`time_ms` 为下位机 RFID 状态更新时间，
  `card_sequence` 为读到新卡事件序号，`number` 是 0 扇区 1 块 0 字节解析出的
  巡逻点编号，范围 `1..12`，无效时为 `0`。
- `/cup_car_serial/rfid` 发布完整 RFID 串口行；`/cup_car_serial/rfid_number`
  仅在新读到有效编号时发布 `std_msgs/msg/Int32`，避免同一张卡持续在场时重复触发；
  `/cup_car_serial/rfid_uid` 同步发布本次事件的 UID 字符串。
- 控制遥测输出类型为
  `mission_control_interfaces/msg/ControlTelemetry`，速度整数毫单位会转换为
  `m/s` 和 `rad/s`。它保留下位机模式、急停、命令年龄、轮速目标/实测值和PWM。
- `ATT` 是主融合建议使用的 BMI088 平面航向：`sample_time_ms` 为下位机采到
  BMI088 样本的时刻，`att_sequence` 每条 ATT 帧递增，`yaw_mdeg` 为连续累计 yaw，
  正方向遵循 ROS `base_link` 的 `+Z`（从车顶看逆时针为正）。`gyro_z_mdps`
  是低通后的 Z 轴角速度、尚未扣除 bias；节点发布 `/cup_car_serial/bmi088_attitude`
  时会计算 `gyro_z_mdps - bias_z_mdps`，因此不会双扣。
- `ATT status` 位：`0x01` gyro valid，`0x02` temperature valid，`0x04` yaw valid，
  `0x08` bias valid，`0x10` stationary，`0x20` gyro saturated，`0x40` sample timeout。
  `/cup_car_serial/bmi088_attitude` 只发布 yaw/bias 有效且未饱和、未超时的帧。
- `IMU` 保留 BMI088 原始 gyro counts，用于诊断；节点会按当前固件的 `±250 dps`
  量程换算后发布到 `/cup_car_serial/bmi088_raw_imu`。
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
| `speak_text_service` | `/cup_car_serial/speak_text` | 总控下发车载语音文本的 service |

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
ros2 topic echo /cup_car_serial/rfid
ros2 topic echo /cup_car_serial/rfid_number
ros2 topic echo /cup_car_serial/rfid_uid
ros2 topic echo /cup_car_serial/control_telemetry
ros2 topic echo /cup_car_serial/actuator_healthy
ros2 topic echo /cup_car_serial/actuator_tracking_status
```

连接正常时 `connected` 为 `true`。最新下位机固件每 20 ms 回传一次 `ENC`/`IMU`/`ATT`/`HLD`，
每 100 ms 回传一次 `CTL`；串口桥会将原始帧同时保留在 `rx` 中，并发布结构化话题。
固件默认处于遥控模式，切换到导航模式前 `actuator_healthy` 会保持 `false`。

查看 RFID 上报：

```bash
ros2 topic echo /cup_car_serial/rfid_number
ros2 topic echo /cup_car_serial/rx
```

测试总控使用的底层语音接口：

```bash
ros2 service call /cup_car_serial/speak_text \
  mission_control_interfaces/srv/SpeakText '{text: "到达一号巡逻点"}'
```

如果超过 `0.4 s` 没有收到新速度，节点会向串口发送零速度。`/imu/rpy` 超过
`rpy_timeout_s`（默认 0.4 s）未更新时，节点暂停发送 RPY 帧。下位机仍应实现独立通信看门狗，并切换到允许接收导航速度的工作模式。
