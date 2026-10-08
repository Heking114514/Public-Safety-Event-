# Changelog

## 2026-10-08

- 将 `rectangle_route_node` 和矩形路线几何从 Python 迁移到 C++17，保留原有 launch
  参数、路线 topic、ACK 和闭环误差日志。
- 离散 primitive A* 实验控制器继续保留 Python，并通过兼容脚本安装，便于逐步迁移和
  对比验证。
