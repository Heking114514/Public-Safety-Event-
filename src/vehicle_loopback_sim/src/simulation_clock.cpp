#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vehicle_loopback_sim
{

// 场景只启动一个时钟源；车辆、控制器和任务计时统一订阅 /clock。
class SimulationClock : public rclcpp::Node
{
public:
  SimulationClock() : Node("simulation_clock")
  {
    scale_ = checked_scale(declare_parameter("simulation_time_scale", 1.0));
    const double rate = declare_parameter("clock_rate_hz", 100.0);
    if (!std::isfinite(rate) || rate <= 0.0) {
      throw std::invalid_argument("clock_rate_hz must be positive and finite");
    }
    step_s_ = 1.0 / rate;
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    scale_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/simulation/time_scale_state", rclcpp::QoS(1).reliable().transient_local());
    scale_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/simulation/time_scale", 10, [this](std_msgs::msg::Float64::SharedPtr message) {
        if (!std::isfinite(message->data) || message->data < 0.1 || message->data > 10.0) {
          RCLCPP_WARN(get_logger(), "simulation time scale must be between 0.1 and 10.0");
          return;
        }
        advance();  // 用旧倍率推进到变更时刻，保持仿真时间连续。
        scale_ = message->data;
        timer_->cancel();
        start_timer();
        publish_scale();
        RCLCPP_INFO(get_logger(), "simulation time scale applied: %.1fx", scale_);
      });
    last_tick_ = std::chrono::steady_clock::now();
    start_timer();
    publish_scale();
    RCLCPP_INFO(get_logger(), "shared /clock ready: %.1fx; vehicle speed parameters unchanged", scale_);
  }

private:
  static double checked_scale(double value)
  {
    if (!std::isfinite(value) || value < 0.1 || value > 10.0) {
      throw std::invalid_argument("simulation_time_scale must be between 0.1 and 10.0");
    }
    return value;
  }

  void start_timer()
  {
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(step_s_ / scale_)), [this]() {advance();});
  }

  void advance()
  {
    const auto current = std::chrono::steady_clock::now();
    elapsed_s_ += std::chrono::duration<double>(current - last_tick_).count() * scale_;
    last_tick_ = current;
    rosgraph_msgs::msg::Clock message;
    message.clock = rclcpp::Time(static_cast<int64_t>(elapsed_s_ * 1.0e9));
    clock_publisher_->publish(message);
  }

  void publish_scale()
  {
    std_msgs::msg::Float64 message;
    message.data = scale_;
    scale_publisher_->publish(message);
  }

  double scale_{1.0};
  double step_s_{0.01};
  double elapsed_s_{1.0};  // 0 表示 ROS 仿真时钟尚未收到首帧。
  std::chrono::steady_clock::time_point last_tick_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr scale_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr scale_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vehicle_loopback_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle_loopback_sim::SimulationClock>());
  rclcpp::shutdown();
  return 0;
}
