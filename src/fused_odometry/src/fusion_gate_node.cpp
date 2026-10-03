#include "fused_odometry/fusion_gate_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<fused_odometry::FusionGateNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fused_odometry_gate"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
