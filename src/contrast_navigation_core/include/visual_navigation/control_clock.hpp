#ifndef VISUAL_NAVIGATION__CONTROL_CLOCK_HPP_
#define VISUAL_NAVIGATION__CONTROL_CLOCK_HPP_

#include <chrono>
#include "rclcpp/clock.hpp"

namespace visual_navigation {

// 仅借用 steady_clock::time_point 作为既有算法接口的时间容器。
// 激活仿真时钟后，计时和控制周期都由 ROS /clock 推进；实车仍使用单调时钟。
inline std::chrono::steady_clock::time_point ControlTime(rclcpp::Clock &clock) {
  using Clock = std::chrono::steady_clock;
  if (!clock.ros_time_is_active()) return Clock::now();
  return Clock::time_point(std::chrono::duration_cast<Clock::duration>(
      std::chrono::nanoseconds(clock.now().nanoseconds())));
}

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__CONTROL_CLOCK_HPP_
