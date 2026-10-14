#ifndef VISUAL_NAVIGATION__LAUNCH_DEPARTURE_HPP_
#define VISUAL_NAVIGATION__LAUNCH_DEPARTURE_HPP_

#include "visual_navigation/route_manager.hpp"

namespace visual_navigation {

// Only the first outbound launch lane uses the user's fixed 11 cm stop.
// Positions are base_link in the route frame; map geometry is supplied by YAML.
class LaunchDeparture {
public:
  static constexpr double distance() { return 0.11; }
  static constexpr double stop_tolerance() { return 0.001; }

  void configure(double home_x, double home_y, double junction_x,
                 double junction_y) {
    home_x_ = home_x;
    home_y_ = home_y;
    junction_x_ = junction_x;
    junction_y_ = junction_y;
    length_ = std::hypot(junction_x - home_x, junction_y - home_y);
    ux_ = (junction_x - home_x) / length_;
    uy_ = (junction_y - home_y) / length_;
    configured_ = true;
  }

  bool applies(const std::vector<RouteWaypoint> &route, std::size_t index) const {
    if (!configured_ || index != 1 || route.size() < 3 ||
        !route[1].turn_junction || route[1].reverse_arrival_turn ||
        route[1].rfid_supernode)
      return false;
    const auto &start = route.front();
    const auto &junction = route[1];
    const double along = ux_ * (start.x - home_x_) + uy_ * (start.y - home_y_);
    const double lateral = -uy_ * (start.x - home_x_) + ux_ * (start.y - home_y_);
    const double next_x = route[2].x - junction.x;
    const double next_y = route[2].y - junction.y;
    const double next_length = std::hypot(next_x, next_y);
    return along >= -0.02 && along < distance() - stop_tolerance() &&
           std::abs(lateral) <= 0.02 &&
           std::hypot(junction.x - junction_x_, junction.y - junction_y_) <= 0.01 &&
           next_length > 0.02 &&
           std::abs(ux_ * next_x + uy_ * next_y) <= 0.1 * next_length;
  }

  double remaining(double x, double y) const {
    return distance() - ux_ * (x - home_x_) - uy_ * (y - home_y_);
  }

  double entry_target_x() const { return distance() - length_; }
  double heading() const { return std::atan2(uy_, ux_); }

private:
  bool configured_{false};
  double home_x_{0.0}, home_y_{0.0}, junction_x_{0.0}, junction_y_{0.0};
  double length_{0.0}, ux_{0.0}, uy_{0.0};
};

}  // namespace visual_navigation
#endif
