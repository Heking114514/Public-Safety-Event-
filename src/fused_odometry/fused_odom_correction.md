# 融合里程计修正策略（审核稿）

方案版本：2026-10-10  
状态：待审核，当前只记录方案，不修改生产代码。

## 1. 目标与边界

### 1.1 目标

定位链采用明确的传感器权威切换：

```text
视觉健康       -> 视觉主导全局位姿
视觉短暂丢失   -> 轮速 + IMU 连续推算
视觉恢复       -> 对齐并平滑切回视觉
```

视觉单独丢失时，导航不进入驻停状态，也不降低当前控制器给出的速度，继续沿当前路线运动。轮速只在视觉不可用时承担位置续航；视觉恢复后，轮速不再覆盖视觉结果。

### 1.2 本次任务范围

本方案只涉及：

- `fused_odometry` 内的视觉、轮速、IMU 健康判断和输出选择；
- `/odometry/fused`、`/odometry/local`、`/odometry/fusion_status` 的状态契约；
- 为允许 `DEGRADED_DEAD_RECKONING` 正常运行所需的导航允许状态配置。

本方案不涉及：

- 下位机固件、PWM、电机控制和串口协议；
- `visual_navigation` 的转弯控制、路口门控和规划算法；
- `new_map_planner`、仿真器和 RFID 任务逻辑；
- 通过放宽 footprint 或道路安全门来掩盖定位错误。

导航包的允许状态配置虽然不属于本功能包，但它是本方案能够实现“视觉丢失不停车”的必要接口变更，只允许做状态白名单配套修改。

## 2. 当前实现与问题

当前链路的基础方向是正确的：

- `/odometry/visual_raw` 经过融合门控后更新 `map -> odom`；
- `/odometry/local` 使用轮速线速度和 IMU 角速度保持连续；
- `/odometry/fused` 对外提供地图坐标下的连续位姿；
- 视觉恢复时已有连续样本收集和 bounded blend 机制。

当前需要修正的契约问题：

1. `fused_odometry` 会发布 `DEGRADED_DEAD_RECKONING`，但导航配置的 `allowed_fusion_states` 未包含该状态，视觉丢失后导航因此进入 `WAITING_FOR_ALLOWED_FUSION_STATUS`。
2. 视觉健康时，轮速与视觉残差主要是诊断，轮速仍进入本地 EKF；需要明确“视觉是全局权威，轮速只提供局部连续性”的语义。
3. 轮速与视觉的残差门控当前计算了 `ResidualGate`，但没有将拒绝结果用于轮速输出选择；需要区分“健康诊断”和“输入有效性”。
4. 轮速航向已经被 bag 证明不可靠，默认不应重新启用 wheel yaw 融合；短期航向继续由 IMU 提供。
5. 视觉恢复期间必须继续输出连续位姿，不能因为收集恢复样本或修正 `map -> odom` 而向导航发零速。

## 3. 状态机设计

状态名称沿用现有 `fusion_status`，不新增并行的布尔开关。

| 状态 | 进入条件 | 输出权威 | 导航行为 | 退出条件 |
| --- | --- | --- | --- | --- |
| `FULL` | 视觉样本新鲜、连续增量有效，IMU可用 | 视觉全局修正 + 局部轮速/IMU连续性 | 正常运动 | 视觉超时或连续增量无效 |
| `DEGRADED_DEAD_RECKONING` | 视觉超时，但轮速和IMU均新鲜 | 轮速线速度 + IMU角速度，冻结上一份 `map -> odom` 修正 | 按当前控制器命令继续运动，不停车、不降速 | 视觉恢复或轮速/IMU失效 |
| `DEGRADED_VISUAL_REALIGNED` | 已收集足够连续视觉样本，正在做视觉重对齐 | 轮速/IMU预测与视觉修正混合 | 继续运动 | blend 完成后进入 `FULL` |
| `DEGRADED_NO_IMU` | 视觉可用、IMU不可用 | 视觉位置，角速度能力受限 | 按现有降级策略继续 | IMU恢复或视觉失效 |
| `DEGRADED_NO_WHEEL` | 视觉可用、轮速不可用 | 视觉 + IMU | 按现有降级策略继续 | 轮速恢复或视觉失效 |
| `FAULT` | 视觉、轮速、IMU均不能提供有效连续位姿，或输入陈旧/非法 | 无可靠来源 | 这是唯一允许进入故障驻停的定位条件 | 重新初始化或人工恢复 |

### 3.1 状态切换门控

- 视觉丢失必须经过现有 `visual_timeout_s`，单帧异常不能切换状态。
- 视觉恢复必须满足现有 `visual_recovery_samples` 个连续有效增量；恢复样本不足时保持轮速/IMU续航。
- `DEGRADED_DEAD_RECKONING` 不是故障状态，不得触发导航驻停或任务锁存。
- `FAULT` 只表示当前没有可用的连续定位来源，不能由单纯视觉丢失触发。
- 状态切换必须有 dwell 和滞回，避免视觉帧在健康/不健康之间横跳。

### 3.2 与控制器、规划器的边界

融合定位只决定“当前位姿是否有连续来源”和“全局位姿由哪个传感器校正”，不决定转弯是否允许执行。

- 视觉健康时：视觉负责全局位置和长期航向；转弯短期姿态反馈使用下位机 IMU。
- 视觉丢失时：轮速提供平移增量，下位机 IMU 提供转弯姿态和角速度；只要两者新鲜且输入合法，继续输出原速度。
- 编码器累计 yaw 永不作为转弯姿态来源，也不因 wheel yaw 异常而丢弃可信的 wheel vx。
- `DEGRADED_DEAD_RECKONING` 必须被导航允许状态白名单接受，因此不会触发
  `WAITING_FOR_ALLOWED_FUSION_STATUS`、`TURN_BLOCKED` 或自动重规划。
- `TURN_ENTRY_CHECK`、`TURN_NEED_REPLAN`、`FAULT_TURN_BLOCKED` 仍只由
  `visual_navigation` 的转弯几何、扫掠安全和实际运动进度决定；融合包不得复制这些门控。
- 只有双源定位失效、障碍物真实触发或转弯几何确实不可行时，现有控制器/规划器门控才可以暂停、重规划或故障锁存。

## 4. 数据流策略

### 4.1 视觉健康时

1. 接受经过坐标系、时间戳、增量和 tracking 状态校验的视觉样本。
2. 视觉样本更新 `map -> odom`，通过现有平滑限制进入 `/odometry/fused`。
3. `/odometry/local` 继续由轮速线速度和 IMU 角速度维持高频连续性。
4. 轮速与视觉的速度/航向差异只记录诊断，不得覆盖视觉全局位姿。
5. 轮速航向不重新进入 EKF；`fuse_wheel_yaw` 继续保持关闭。

这里的“相信视觉”指 `/odometry/fused` 的全局位置和长期航向由视觉校正主导，不是把未经门控的 `/odometry/visual_raw` 直接接给导航。

### 4.2 视觉丢失时

1. 冻结最后有效的视觉 `map -> odom` 修正，不再使用过期视觉帧。
2. `/odometry/local` 继续使用轮速 `vx/vy` 和 IMU `wz` 推进。
3. `/odometry/fused` 继续输出连续地图位姿，不能因状态变为 `DEGRADED_DEAD_RECKONING` 而清零速度。
4. 位姿协方差随续航时间和轮速行程增长，诊断中明确标记当前为轮速/IMU续航；协方差增长只表达不确定性，不得映射成降速命令。
5. 不应用 `dead_reckoning_speed_scale` 或任何新的视觉丢失限速；输出线速度与导航控制器在同一周期给出的有效线速度一致。

轮速在该阶段是唯一的短时平移来源；没有视觉时，系统不能凭空判断轮胎是否打滑，因此只拒绝格式非法、时间过期、超物理上限的轮速样本，不用无法验证的残差强行切断全部轮速。

### 4.3 视觉恢复时

1. 保持轮速/IMU连续预测，不等待视觉恢复完成后再运动。
2. 收集连续有效视觉增量，计算视觉位姿与当前轮速/IMU预测之间的 SE(2) 偏差。
3. 通过现有融合门参数 `visual_recovery_initial_covariance_scale`、`visual_recovery_ramp_s`，以及地图修正参数 `visual_recovery_blend_duration_s` 限制修正速度。
4. 修正 `map -> odom`，不得直接跳变 `/odometry/fused` 的公开位姿。
5. blend 完成后再发布 `FULL`，恢复视觉主导。

## 5. 轮速异常处理（只使用里程计输入）

本方案不读取下位机状态，不依据 PWM 或电机反馈判断堵转。

### 5.1 视觉健康时

- 视觉是权威，轮速异常只能产生 `wheel_visual_residual`、`wheel_imu_yaw_residual` 等诊断信息。
- 视觉健康期间不能因为轮速漂移而切换到轮速主导。
- 如果轮速航向与 IMU 不一致，继续保持 `fuse_wheel_yaw=false`。

### 5.2 视觉丢失时

- 轮速只提供平移增量，IMU提供短期航向增量。
- 轮速航向不参与主 EKF，避免使用已经验证过的错误 wheel yaw。
- 轮速样本仍经过 frame、时间戳、有限值和速度上限检查。
- 视觉恢复后，用视觉与轮速/IMU的累计差异做诊断，但不把恢复瞬间的差异直接变成位姿跳变。

### 5.3 残差门控改造原则

`ResidualGate` 的结果必须有明确语义，不能“计算但不使用”：

- 视觉健康时：残差门控只用于诊断和协方差，不切换视觉权威；
- 视觉不健康时：只使用输入合法性门控，不能用没有独立真值的视觉残差判定轮速失效；
- wheel yaw 继续独立于 wheel vx 处理，不因为 wheel yaw 异常而丢弃可信的轮速平移；
- 所有被拒绝的样本必须增加诊断计数并说明原因。

## 6. 必要修改清单

以下是审核通过后才实施的修改范围：

### `src/fused_odometry`

- `src/fusion_health.cpp`、`include/fused_odometry/fusion_health.hpp`
  - 明确 `DEGRADED_DEAD_RECKONING` 为可运行状态；
  - 保持视觉丢失期间轮速/IMU连续推进；
  - 增加状态转换原因和 dwell 诊断。
- `src/wheel.cpp`
  - 明确 wheel yaw 永不进入默认 EKF；
  - 整理轮速输入门控和残差诊断；
  - 让门控结果与输出策略一致，避免“更新返回值未使用”。
- `src/map_odom_correction_node.cpp`
  - 视觉丢失时冻结视觉修正；
  - 恢复时继续使用轮速/IMU预测并做有界 blend；
  - 保留 RFID 修正和视觉恢复之间的重新锚定顺序。
- `config/fused_odometry.yaml`
  - 只调整融合状态和恢复参数；
  - 不在本轮修改轮距、轮径、编码器比例等物理参数。
- `test/`
  - 增加视觉主导、视觉丢失续航、恢复 blend、wheel yaw 不进入 EKF、状态滞回测试。

### 配套接口修改（功能包外，仅一处白名单）

`src/visual_navigation/config/waypoint_navigation.yaml` 的 `allowed_fusion_states` 必须加入：

```yaml
- "DEGRADED_DEAD_RECKONING"
```

这不是修改控制器逻辑，而是让导航正确理解“视觉丢失但轮速/IMU仍可用”不是故障。

## 7. 验收标准

### 7.1 视觉健康

- 人为制造轮式里程计航向漂移时，`/odometry/fused` 仍跟随视觉轨迹；
- 状态保持 `FULL`；
- 轮速异常只出现在诊断，不触发驻停和路线重规划。

### 7.2 视觉丢失

- `/odometry/fusion_status` 进入 `DEGRADED_DEAD_RECKONING`；
- 导航继续发布非零速度，不进入 `WAITING_FOR_ALLOWED_FUSION_STATUS`；
- 融合节点不修改控制器线速度，不因视觉丢失降速；
- `/odometry/fused` 位姿连续，无突然回零或跳变；
- 位置协方差和续航距离诊断持续增长。

### 7.3 视觉恢复

- 先保持轮速/IMU运动，再完成视觉恢复样本收集；
- `/odometry/fused` 不出现单帧大跳；
- 状态顺序必须是：

```text
DEGRADED_DEAD_RECKONING
-> DEGRADED_VISUAL_REALIGNED
-> FULL
```

### 7.4 双源失效边界

- 视觉、轮速、IMU同时无效时，必须明确发布 `FAULT`；
- 不能用无来源的位姿继续假装定位正常；
- 该边界不应被“视觉丢失不停车”要求覆盖。

## 8. 回放验证顺序

审核通过后按以下顺序验证，不直接上实车：

1. 使用现有融合单元测试验证状态迁移和恢复 blend。
2. 回放第一次 bag，检查视觉主导时轮速错误不污染 `/odometry/fused`。
3. 回放第二次 bag，检查视觉丢失时不再进入 `WAITING_FOR_ALLOWED_FUSION_STATUS`。
4. 检查视觉恢复时是否仍保持连续运动和有限修正。
5. 通过后再进行短距离实车采集，单独验证视觉丢失和恢复，不把规划器、避障和转弯改动混入同一次实验。

## 9. 明确不做的事情

- 不通过放宽规划器 footprint 门控解决里程计错误。
- 不把 wheel yaw 重新接入 EKF 来“补偿”视觉丢失。
- 不因单次轮速残差直接停车或锁存任务。
- 不修改下位机死区、电机响应和串口反馈逻辑。
- 不承诺在视觉、轮速、IMU全部失效时无限制盲行。

