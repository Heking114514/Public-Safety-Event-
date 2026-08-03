# ORB_SLAM3_ROS2
本仓库提供 ORB_SLAM3 的 ROS2 封装。

---

## 演示视频
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## 环境要求
- 已在以下版本中测试：
  - Ubuntu 20.04
  - ROS2 Foxy
  - OpenCV 4.2.0

- 构建 ORB_SLAM3：
  - 前往此[仓库](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED)，按照其中的构建说明操作。

- 安装相关 ROS2 软件包：
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## 构建方法
1. 将仓库克隆到 ROS 工作空间：
```
$ mkdir -p colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

2. 将此处的[路径](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeLists.txt#L5)修改为你自己的 `python site-packages` 路径。

3. 将此处的[路径](https://github.com/zang09/ORB_SLAM3_ROS2/blob/ee82428ed627922058b93fea1d647725c813584e/CMakeModules/FindORB_SLAM3.cmake#L8)修改为你自己的 `ORB_SLAM3` 路径。

完成以上配置后即可开始构建：
```
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select orbslam3
```

## 故障排查
1. 如果找不到 `sophus/se3.hpp`：  
进入 `ORB_SLAM3_ROOT_DIR` 并安装 Sophus 库。
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

2. 请使用 `OpenCV 4.2.0` 进行编译。
详情请参考此 [Issue](https://github.com/zang09/ORB_SLAM3_ROS2/issues/2#issuecomment-1251850857)。

## 使用方法
1. 加载工作空间环境：  
```
$ source ~/colcon_ws/install/local_setup.bash
```

2. 运行所需的 ORB-SLAM 模式。  
本仓库目前仅支持 `MONO、STEREO、RGBD、STEREO-INERTIAL` 模式。  
词汇表文件和配置文件位于仓库内（例如，单目 SLAM 使用 `orbslam3_ros2/vocabulary/ORBvoc.txt` 和 `orbslam3_ros2/config/monocular/TUM1.yaml`）。
  - `MONO` 模式  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO` 模式  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` 模式  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO-INERTIAL` 模式  
```
$ ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

## Intel RealSense D455 配置

以下 D455 配置文件使用从序列号为 `038122250473`、固件版本为 `5.17.0.10` 的相机中读取的出厂标定数据。使用其他实体相机时，请重新读取其出厂标定数据。

| 传感器模式 | 配置文件 | 所需 RealSense 数据流配置 |
| --- | --- | --- |
| 单目 | `config/monocular/RealSense_D455.yaml` | 彩色 `640x480x30`，RGB8 |
| 单目-惯性 | `config/monocular-inertial/RealSense_D455.yaml` | 彩色 `640x480x30`，RGB8；陀螺仪 `200 Hz`；加速度计 `63 Hz` |
| RGB-D | `config/rgb-d/RealSense_D455.yaml` | 彩色 `640x480x30`、深度 `640x480x30`，启用深度对齐 |
| RGB-D-惯性 | `config/rgb-d-inertial/RealSense_D455.yaml` | RGB-D 数据流配置，加上陀螺仪和加速度计 |
| 双目 | `config/stereo/RealSense_D455.yaml` | 红外 1/2 `848x480x30`，Y8 |
| 双目-惯性 | `config/stereo-inertial/RealSense_D455.yaml` | 双目数据流配置，加上 `200 Hz` 陀螺仪和 `63 Hz` 加速度计 |

`config/monocular/RealSense_D455_424x240_5fps.yaml` 文件仅作为 USB 2.0 降级备用配置。正常的 D455 配置需要 USB 3.x 连接，并且 `lsusb -t` 报告的速率应为 `5000M` 或更高。

双目配置使用 D455 的 95 mm 基线。惯性配置使用 librealsense 报告的 IMU 到彩色相机或 IMU 到左红外相机的出厂变换参数。

### D455 双目-惯性里程计

构建并加载工作空间环境，然后同时启动相机和 ORB-SLAM3：

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select orbslam3 \
  --cmake-args \
  -DSophus_DIR=$PWD/src/deps/share/sophus/cmake \
  -DPangolin_DIR=$PWD/src/deps/lib/cmake/Pangolin
source install/setup.bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py
```

该 Launch 文件会选择序列号为 `038122250473` 的相机以及 `848x480x30` 的红外数据流。默认使用两个红外相机和融合后的 IMU 数据流执行双目-惯性尺度里程计。启动后的前两秒双目图像会被丢弃，以等待 D455 自动曝光稳定。红外发射器会被禁用，避免其投射的点阵图案干扰 ORB 特征提取；所需的话题重映射会自动应用。

发布的输出：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 具有真实尺度的机体位姿和帧间速度 |
| `/pose` | `geometry_msgs/msg/PoseStamped` | 配置的地图坐标系中的机体位姿 |
| `/path` | `nav_msgs/msg/Path` | 有长度限制的历史位姿轨迹 |
| `/tracking_state` | `std_msgs/msg/Int32` | ORB-SLAM3 跟踪状态枚举值 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 跟踪、IMU、同步和丢帧状态 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 从 `map_fram0e_id` 到 `body_frame_id` 的坐标变换 |

跟踪状态值分别为：`-1 SYSTEM_NOT_READY`、`0 NO_IMAGES_YET`、`1 NOT_INITIALIZED`、`2 OK`、`3 RECENTLY_LOST`、`4 LOST` 和 `5 OK_KLT`。仅当状态为 `OK` 或 `OK_KLT` 时才发布里程计数据；跟踪不可用时不会发布过期位姿。

默认机体坐标系为 `camera_link`。在机器人上使用时，请发布从 `base_link` 到 `camera_link` 的实测静态变换，然后运行：

```bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  body_frame_id:=base_link
```

启动时，以适中的幅度平移相机并绕多个轴转动，直到 `/diagnostics` 报告 `Tracking`。保持相机完全静止无法为惯性初始化提供足够的加速度变化。

如需禁用 IMU，请使用纯双目模式，并根据需要启用启动重置：

```bash
ros2 launch orbslam3 realsense_d455_stereo_inertial.launch.py \
  use_imu:=false initial_reset:=true
```

常用 Launch 参数：

```bash
visualization:=true       # 启用 Pangolin 可视化界面
equalize:=true            # 启用 CLAHE；D455 红外图像默认禁用
initial_reset:=true       # 启用启动重置；使用 IMU 时请勿启用
use_imu:=false            # 切换回纯双目里程计
map_frame_id:=map
body_frame_id:=camera_link
serial_no:=_038122250473  # 前导下划线可使 ROS 将参数识别为字符串
```

## 使用 rosbag 运行
如需播放 ROS1 bag 文件，应安装 `ROS1 Noetic` 和 `ros1_bridge`。  
此[链接](https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/)提供了 `ros1-ros2 bridge` 操作流程示例。  
如果已经安装 `ROS1 Noetic` 和 `ros1_bridge`，请打开终端并按照以下步骤操作：  
（Shell A、B、C、D 分别表示不同的终端，以下以 `stereo-inertial` 模式为例。）

1. 下载 EuRoC 数据集（`V1_02_medium.bag`）：
```
$ wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
```

2. 启动各终端：  
（例如，`ROS1_INSTALL_PATH`=`/opt/ros/noetic`，`ROS2_INSTALL_PATH`=`/opt/ros/foxy`。）
```
# Shell A：
source ${ROS1_INSTALL_PATH}/setup.bash
roscore

# Shell B：
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge

# Shell C：
source ${ROS1_INSTALL_PATH}/setup.bash
rosbag play ~/Downloads/V1_02_medium.bag --pause /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

# Shell D：
source ${ROS2_INSTALL_PATH}/setup.bash
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

3. 在 `Shell C` 中按空格键，继续播放 bag 文件。

## 致谢
本仓库基于[此仓库](https://github.com/curryc/ros2_orbslam3)修改，增加了 `stereo-inertial` 模式并改善了构建方面的问题。
