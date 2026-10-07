#include "visual_navigation/waypoint_navigator.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
