# contrast_navigation_core

这是 `contrast_controller` 使用的独立航点导航核心。

源码从当前 `visual_navigation::waypoint_navigator_core` 拆出一份，保留相同的 C++ 命名空间、消息接口、参数接口和控制话题接口，便于逐项对照。`visual_navigation` 继续构建并运行自己的核心；`contrast_controller` 链接本包的 `contrast_waypoint_navigator_core`，不再链接 `visual_navigation::waypoint_navigator_core`。

当前包只提供静态核心库，不直接启动 ROS 节点。可通过 `contrast_controller` 的对比或避障验证 launch 运行它。这样修改本包中的路径跟踪、障碍恢复或姿态恢复实现时，不会改动正式 `visual_navigation` 控制器。
