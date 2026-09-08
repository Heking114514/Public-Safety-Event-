#include "imu_rpy_filter/imu_rpy_filter_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_rpy_filter::ImuRpyFilterNode>());
  rclcpp::shutdown();
  return 0;
}
