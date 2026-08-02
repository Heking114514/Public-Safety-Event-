# ORB_SLAM3 ROS2 启动指南

## 前置条件

- Ubuntu 22.04 / ROS2 Humble
- Intel RealSense D455 摄像头（USB 3.x 连接，`lsusb -t` 显示 `5000M`）
- 已编译好 ORB_SLAM3 库
- 词汇文件 `vocabulary/ORBvoc.txt`（约 1M 行，已在仓库中）

---

## 1. 编译

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DSophus_DIR=$PWD/deps/share/sophus/cmake \
  -DPangolin_DIR=$PWD/Pangolin/build
source install/setup.bash
```

> 如果 Sophus 和 Pangolin 路径不同，请按实际情况修改 `-DSophus_DIR` 和 `-DPangolin_DIR`。

---

## 2. 启动程序

### 基础启动（纯里程计，无可视化窗口）

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py
```

### 带 Pangolin 可视化窗口

```bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py visualization:=true
```

### 常用启动参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_no` | `_038122250473` | D455 序列号，注意前缀下划线保持字符串类型 |
| `visualization` | `false` | 开启 Pangolin 可视化窗口 |
| `use_imu` | `true` | `false` 切换为纯双目模式（无 IMU） |
| `equalize` | `false` | 开启 CLAHE 直方图均衡化（红外图不建议） |
| `initial_reset` | `false` | 启动时重置摄像头（IMU 模式下不建议） |
| `body_frame_id` | `camera_link` | 机身坐标系名，机器人上建议设为 `base_link` |
| `map_frame_id` | `map` | 地图坐标系名 |

**示例：纯视觉双目模式**

```bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py use_imu:=false
```

**示例：机器人上使用，指定机身坐标系**

```bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  body_frame_id:=base_link visualization:=true
```

---

## 3. 初始化

启动后需要**移动相机**才能完成初始化：

1. 前 **2 秒**是相机预热期，帧被丢弃等自动曝光稳定
2. 预热完成后终端会打印：`Camera warmup complete after N stereo pairs`
3. **立刻手持相机做中等幅度的平移 + 多轴旋转**
4. 如果加速度不够，终端会反复打印：`not enough acceleration`
5. 当条件满足时，初始化自动完成

> ⚠️ **静止不动永远不会初始化。** 必须让 IMU 感受到足够的加速度变化。

### 确认初始化成功

打开另一个终端：

```bash
ros2 topic echo /diagnostics --once | grep tracking_state
```

看到 `tracking_state: OK`（值为 2）或 `OK_KLT`（值为 5）即表示成功。

---

## 4. 验证

初始化成功后，以下话题开始发布数据：

| 话题 | 类型 | 内容 |
|------|------|------|
| `/pose` | `geometry_msgs/PoseStamped` | 当前位姿 |
| `/odom` | `nav_msgs/Odometry` | 位姿 + 速度 + 协方差 |
| `/path` | `nav_msgs/Path` | 历史轨迹（最多 2000 帧） |
| `/tracking_state` | `std_msgs/Int32` | 跟踪状态码 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 诊断信息 |
| `/tf` | `tf2_msgs/TFMessage` | `map` → `camera_link` 变换 |

```bash
# 查看实时位姿
ros2 topic echo /pose

# 查看跟踪状态
ros2 topic echo /tracking_state

# 查看诊断信息
ros2 topic echo /diagnostics

# RViz2 可视化轨迹
rviz2
# 在 RViz 中添加 /path 和 /tf 显示
```

### 快速精度测试

1. 地上标记 1 米距离
2. 初始化完成后平推相机 1 米
3. 查看 `/pose` 的位移是否接近 1.0m

---

## 5. 话题与参数配置

### 可配置的话题名

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `odom_topic` | `odom` | 里程计话题名 |
| `pose_topic` | `pose` | 位姿话题名 |
| `path_topic` | `path` | 轨迹话题名 |
| `tracking_state_topic` | `tracking_state` | 跟踪状态话题名 |
| `diagnostics_topic` | `/diagnostics` | 诊断话题名 |

### 可配置的行为参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_tf` | `true` | 是否发布 TF |
| `publish_path` | `true` | 是否发布轨迹 |
| `save_trajectory` | `true` | 退出时是否保存关键帧轨迹到文件 |
| `trajectory_file` | `KeyFrameTrajectory.txt` | 轨迹保存文件名（TUM 格式） |
| `reset_path_on_tracking_loss` | `true` | 跟踪丢失时是否清空轨迹 |
| `camera_warmup_seconds` | `2.0` | 预热时长（秒） |
| `max_stereo_time_diff` | `0.01` | 左右帧最大时间差（秒） |
| `path_max_poses` | `2000` | 轨迹最大帧数 |
| `diagnostics_period` | `1.0` | 诊断发布周期（秒） |
| `imu_time_offset` | `0.0` | IMU 时间戳偏移（秒） |

---

## 6. 录制与回放

### 录制

```bash
ros2 bag record -o my_session /path /odom /pose /tf /diagnostics
```

### 用 rosbag 播放数据集

```bash
# 需要 ros1_bridge 桥接 ros1 bag
# 详见 README 中 "Run with rosbag" 章节
```

---

## 7. 跟踪状态码

| 值 | 状态 | 说明 |
|----|------|------|
| -1 | `SYSTEM_NOT_READY` | 系统未就绪（预热中） |
| 0 | `NO_IMAGES_YET` | 尚未收到图像 |
| 1 | `NOT_INITIALIZED` | 未初始化（正在尝试） |
| 2 | `OK` | 正常跟踪 |
| 3 | `RECENTLY_LOST` | 最近丢失（短期内可能恢复） |
| 4 | `LOST` | 跟踪丢失 |
| 5 | `OK_KLT` | 正常跟踪（KLT 模式） |

---

## 8. 常见问题

**Q: 启动后一直显示 `not enough acceleration`？**
移动相机。加速度变化需要 > 0.5 m/s²，静止不动无法初始化。

**Q: 轨迹漂移严重？**
- 确保 IMU 模式下初始化时有足够的运动变化
- 回到已访问过的区域触发回环检测
- 纯视觉模式下漂移会更大，建议使用 stereo-inertial

**Q: 退出后轨迹保存在哪？**
当前目录下的 `KeyFrameTrajectory.txt`（TUM 格式）。
