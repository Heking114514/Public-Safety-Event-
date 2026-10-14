#include "visual_navigation/waypoint_navigator.hpp"

#include <yaml-cpp/yaml.h>

void WaypointNavigator::InitLaunchDeparture() {
  if (!declare_parameter<bool>("launch_departure_enabled", false))
    return;
  const auto path = has_parameter("recovery_map_file")
      ? get_parameter("recovery_map_file").as_string()
      : declare_parameter<std::string>("recovery_map_file", "");
  const auto map = YAML::LoadFile(path);
  const auto home_id = map["home_node"].as<std::string>();
  const auto home = map["nodes"][home_id];
  std::string junction_id;
  for (const auto &lane : map["lanes"]) {
    const auto from = lane["from"].as<std::string>();
    const auto to = lane["to"].as<std::string>();
    if (from == home_id || to == home_id) {
      if (!junction_id.empty())
        throw std::invalid_argument("fixed launch departure requires one home lane");
      junction_id = from == home_id ? to : from;
    }
  }
  const auto junction = map["nodes"][junction_id];
  const double length = std::hypot(junction[0].as<double>() - home[0].as<double>(),
                                   junction[1].as<double>() - home[1].as<double>());
  if (length <= visual_navigation::LaunchDeparture::distance())
    throw std::invalid_argument("launch junction must be beyond the 11 cm stop");
  const double ox = map["frame"]["map_origin_m"][0].as<double>();
  const double oy = map["frame"]["map_origin_m"][1].as<double>();
  const double yaw = map["frame"]["map_x_heading_deg"].as<double>() * kPi / 180.0;
  const auto x = [&](const YAML::Node &p) {
    return std::cos(yaw) * (p[0].as<double>() - ox) +
           std::sin(yaw) * (p[1].as<double>() - oy);
  };
  const auto y = [&](const YAML::Node &p) {
    return -std::sin(yaw) * (p[0].as<double>() - ox) +
            std::cos(yaw) * (p[1].as<double>() - oy);
  };
  launchDeparture_.configure(x(home), y(home), x(junction), y(junction));
  RCLCPP_INFO(get_logger(), "Launch departure: fixed base_link stop 0.110 m from home");
}
