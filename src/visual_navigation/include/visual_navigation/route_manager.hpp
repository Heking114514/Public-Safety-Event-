#ifndef VISUAL_NAVIGATION__ROUTE_MANAGER_HPP_
#define VISUAL_NAVIGATION__ROUTE_MANAGER_HPP_

#include <cmath>
#include <cctype>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace visual_navigation
{

struct RouteWaypoint
{
  double x{0.0};
  double y{0.0};
  double yaw{std::numeric_limits<double>::quiet_NaN()};
  double speed{0.0};
  double tolerance{0.0};
  double stop_time{0.0};
};

// Owns route parsing and comparison policy. The ROS node remains responsible
// for route publication and mission-state transitions.
class RouteManager
{
public:
  static bool LoadCsv(
    const std::string & route_file, double default_speed, double default_tolerance,
    std::vector<RouteWaypoint> & output, std::string & error)
  {
    output.clear();
    error.clear();
    if (route_file.empty()) {
      error = "No startup route configured";
      return false;
    }

    std::ifstream input(route_file);
    if (!input.is_open()) {
      error = "Cannot open waypoint file: " + route_file;
      return false;
    }

    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
      ++line_number;
      const std::size_t comment = line.find('#');
      if (comment != std::string::npos)
        line.resize(comment);
      line = trim(line);
      if (line.empty())
        continue;
      const std::vector<std::string> fields = split_csv(line);
      if (looks_like_header(fields))
        continue;
      if (fields.size() < 2) {
        error = "Waypoint line " + std::to_string(line_number) + " requires at least x,y";
        output.clear();
        return false;
      }
      try {
        RouteWaypoint waypoint;
        waypoint.x = std::stod(fields[0]);
        waypoint.y = std::stod(fields[1]);
        if (fields.size() > 2 && !fields[2].empty())
          waypoint.yaw = std::stod(fields[2]);
        waypoint.speed = fields.size() > 3 && !fields[3].empty() ?
          std::stod(fields[3]) : default_speed;
        waypoint.tolerance = fields.size() > 4 && !fields[4].empty() ?
          std::stod(fields[4]) : default_tolerance;
        waypoint.stop_time = fields.size() > 5 && !fields[5].empty() ?
          std::stod(fields[5]) : 0.0;
        if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y) ||
          !std::isfinite(waypoint.speed) || !std::isfinite(waypoint.tolerance) ||
          !std::isfinite(waypoint.stop_time) || waypoint.speed < 0.0 ||
          waypoint.tolerance <= 0.0 || waypoint.stop_time < 0.0)
        {
          throw std::runtime_error("waypoint values are outside their valid range");
        }
        output.push_back(waypoint);
      } catch (const std::exception & exception) {
        error = "Invalid waypoint at line " + std::to_string(line_number) + ": " +
          exception.what();
        output.clear();
        return false;
      }
    }

    if (output.empty()) {
      error = "Waypoint file contains no valid waypoints";
      return false;
    }
    return true;
  }

  static bool Equivalent(
    const std::vector<RouteWaypoint> & left, const std::vector<RouteWaypoint> & right)
  {
    if (left.size() != right.size())
      return false;
    constexpr double k_position_tolerance = 1e-4;
    constexpr double k_yaw_tolerance = 1e-3;
    for (std::size_t index = 0; index < left.size(); ++index) {
      const bool left_yaw = std::isfinite(left[index].yaw);
      const bool right_yaw = std::isfinite(right[index].yaw);
      if (std::abs(left[index].x - right[index].x) > k_position_tolerance ||
        std::abs(left[index].y - right[index].y) > k_position_tolerance ||
        left_yaw != right_yaw ||
        (left_yaw && std::abs(normalize_angle(left[index].yaw - right[index].yaw)) >
        k_yaw_tolerance))
      {
        return false;
      }
    }
    return true;
  }

  static bool LoadPath(
    const nav_msgs::msg::Path & path, double default_speed, double default_tolerance,
    std::vector<RouteWaypoint> & output, std::size_t & invalid_index, std::string & error)
  {
    output.clear();
    error.clear();
    invalid_index = 0;
    if (path.poses.empty()) {
      error = "route is empty";
      return false;
    }
    output.reserve(path.poses.size());
    for (std::size_t index = 0; index < path.poses.size(); ++index) {
      const auto & pose = path.poses[index].pose;
      if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !valid_quaternion(pose.orientation))
      {
        invalid_index = index;
        error = "waypoint pose is invalid";
        output.clear();
        return false;
      }
      RouteWaypoint waypoint;
      waypoint.x = pose.position.x;
      waypoint.y = pose.position.y;
      waypoint.yaw = quaternion_yaw(pose.orientation);
      waypoint.speed = default_speed;
      waypoint.tolerance = default_tolerance;
      output.push_back(waypoint);
    }
    return true;
  }

private:
  static std::string trim(const std::string & text)
  {
    const std::string whitespace = " \t\r\n";
    const std::size_t first = text.find_first_not_of(whitespace);
    if (first == std::string::npos)
      return "";
    const std::size_t last = text.find_last_not_of(whitespace);
    return text.substr(first, last - first + 1);
  }

  static std::vector<std::string> split_csv(const std::string & line)
  {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
      fields.push_back(trim(field));
    return fields;
  }

  static bool looks_like_header(const std::vector<std::string> & fields)
  {
    if (fields.empty())
      return false;
    std::string first = fields.front();
    for (char & character : first)
      character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
    return first == "x" || first == "x[m]" || first == "x_m";
  }

  static double normalize_angle(double angle)
  {
    constexpr double k_pi = 3.14159265358979323846;
    while (angle > k_pi)
      angle -= 2.0 * k_pi;
    while (angle < -k_pi)
      angle += 2.0 * k_pi;
    return angle;
  }

  static bool valid_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
  {
    if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
      !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
    {
      return false;
    }
    const double norm_squared = quaternion.x * quaternion.x +
      quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w;
    return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
  }

  static double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
  {
    const double norm_squared =
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
    return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y) / norm_squared,
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z) /
      norm_squared);
  }
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__ROUTE_MANAGER_HPP_
