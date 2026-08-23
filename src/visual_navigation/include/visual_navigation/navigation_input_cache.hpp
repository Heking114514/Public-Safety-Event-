#ifndef VISUAL_NAVIGATION__NAVIGATION_INPUT_CACHE_HPP_
#define VISUAL_NAVIGATION__NAVIGATION_INPUT_CACHE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visual_navigation/navigation_supervisor.hpp"

namespace visual_navigation
{

// Owns receipt and freshness state for navigation prerequisites. It deliberately
// does not decide whether navigation may drive; NavigationSupervisor owns that policy.
class NavigationInputCache
{
public:
  void update_odometry(
    bool frame_valid, bool pose_valid, bool velocity_valid, double velocity,
    const rclcpp::Time & received_at)
  {
    odometry_received_ = true;
    odometry_frame_valid_ = frame_valid;
    odometry_pose_valid_ = pose_valid;
    odometry_velocity_valid_ = velocity_valid;
    if (velocity_valid)
      odometry_velocity_ = velocity;
    odometry_received_at_ = received_at;
  }

  void update_imu(const rclcpp::Time & received_at)
  {
    imu_received_ = true;
    imu_received_at_ = received_at;
  }

  void update_tracking(int state)
  {
    tracking_received_ = true;
    tracking_state_ = state;
  }

  void update_fusion_status(const std::string & status, const rclcpp::Time & received_at)
  {
    fusion_status_received_ = true;
    fusion_status_ = status;
    fusion_status_received_at_ = received_at;
  }

  void update_actuator(bool connected, const rclcpp::Time & received_at)
  {
    actuator_received_ = true;
    actuator_connected_ = connected;
    actuator_received_at_ = received_at;
  }

  bool imu_fresh(const rclcpp::Time & current, double timeout) const
  {
    return imu_received_ && (current - imu_received_at_).seconds() <= timeout;
  }

  NavigationInputStatus snapshot(
    const rclcpp::Time & current, double odometry_timeout, double fusion_timeout,
    double actuator_timeout, bool fusion_required, bool tracking_required,
    bool actuator_required) const
  {
    NavigationInputStatus input;
    input.odometry_received = odometry_received_;
    input.odometry_frame_valid = odometry_frame_valid_;
    input.odometry_pose_valid = odometry_pose_valid_;
    input.odometry_fresh = odometry_received_ &&
      (current - odometry_received_at_).seconds() <= odometry_timeout;
    input.fusion_required = fusion_required;
    input.fusion_status_received = fusion_status_received_;
    input.fusion_status_fresh = fusion_status_received_ &&
      (current - fusion_status_received_at_).seconds() <= fusion_timeout;
    input.fusion_status = fusion_status_;
    input.tracking_required = tracking_required;
    input.tracking_state_received = tracking_received_;
    input.tracking_state = tracking_state_;
    input.actuator_required = actuator_required;
    input.actuator_status_received = actuator_received_;
    input.actuator_status_fresh = actuator_received_ &&
      (current - actuator_received_at_).seconds() <= actuator_timeout;
    input.actuator_connected = actuator_connected_;
    return input;
  }

  bool odometry_velocity_valid() const {return odometry_velocity_valid_;}
  double odometry_velocity() const {return odometry_velocity_;}
  const std::string & fusion_status() const {return fusion_status_;}

private:
  bool odometry_received_{false};
  bool odometry_frame_valid_{false};
  bool odometry_pose_valid_{false};
  bool odometry_velocity_valid_{false};
  double odometry_velocity_{0.0};
  rclcpp::Time odometry_received_at_{0, 0, RCL_ROS_TIME};

  bool imu_received_{false};
  rclcpp::Time imu_received_at_{0, 0, RCL_ROS_TIME};

  bool tracking_received_{false};
  int tracking_state_{-1};

  bool fusion_status_received_{false};
  std::string fusion_status_;
  rclcpp::Time fusion_status_received_at_{0, 0, RCL_ROS_TIME};

  bool actuator_received_{false};
  bool actuator_connected_{false};
  rclcpp::Time actuator_received_at_{0, 0, RCL_ROS_TIME};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__NAVIGATION_INPUT_CACHE_HPP_
