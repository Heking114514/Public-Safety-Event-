#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "vehicle_loopback_sim/simulator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle_loopback_sim::VehicleLoopbackSim>());
  rclcpp::shutdown();
  return 0;
}
