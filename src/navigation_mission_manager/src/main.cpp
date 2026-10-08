#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "navigation_mission_manager/manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<navigation_mission_manager::MissionManager>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
    executor.add_node(node);
    executor.spin();
    if (rclcpp::ok()) {
      node->request_shutdown_hold();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("navigation_mission_manager"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
