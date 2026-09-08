#include "fused_odometry/fusion_gate_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fused_odometry::FusionGateNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fused_odometry_gate"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
