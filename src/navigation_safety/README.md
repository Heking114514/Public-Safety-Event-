# navigation_safety

`navigation_safety` 是规划器和导航控制器共用的 C++ 几何判定库，不依赖 ROS。它统一
车体 footprint、多边形相交、自由区域边界和离散位姿间的扫掠采样实现；调用方提供环境、
车体尺寸、轴偏移和安全余量。

当前规划器将地图自由区域和静态/动态障碍传入判定器；`visual_navigation` 将现有路口 L 形
区域传入同一判定器。两端算法已统一，但环境数据和坐标系尚未统一：规划器地图位于 `arena`
坐标系，导航路线/融合位姿使用 `map`，导航器目前没有订阅规划器地图。本包不把不同环境
伪装成同一张地图；接入公共地图及明确坐标变换需单独完成和验证。

达到配置余量后的精确相切按可行处理，正面积交叠按碰撞处理。规划器原有路径余量、转弯余量及导航器路口余量保持原值，本次不调整速度、
尺寸、余量、道路拓扑或转向策略。

构建与测试：

```bash
colcon build --packages-up-to navigation_safety new_map_planner visual_navigation
colcon test --packages-select navigation_safety new_map_planner visual_navigation
```
