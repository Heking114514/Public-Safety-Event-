#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mission_control_interfaces/msg/motion_hold_state.hpp"
#include "mission_control_interfaces/srv/set_motion_hold.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visual_navigation/fusion_health_policy.hpp"
#include "visual_navigation/navigation_supervisor.hpp"
#include "visual_navigation/navigation_input_cache.hpp"
#include "visual_navigation/path_control.hpp"
#include "visual_navigation/path_progress_supervisor.hpp"
#include "visual_navigation/rate_limiter.hpp"
#include "visual_navigation/route_manager.hpp"
#include "visual_navigation/turn_progress_supervisor.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double Clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

double NormalizeAngle(double angle)
{
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  const double sinYaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosYaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sinYaw, cosYaw);
}

double YawFromValidQuaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  geometry_msgs::msg::Quaternion normalized;
  normalized.x = quaternion.x / norm;
  normalized.y = quaternion.y / norm;
  normalized.z = quaternion.z / norm;
  normalized.w = quaternion.w / norm;
  return YawFromQuaternion(normalized);
}

bool QuaternionIsValid(const geometry_msgs::msg::Quaternion &quaternion)
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return false;
  }

  const double squaredNorm = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(squaredNorm) && squaredNorm > 1e-12;
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

std::string Trim(const std::string &text)
{
  const std::string whitespace = " \t\r\n";
  const std::size_t first = text.find_first_not_of(whitespace);
  if (first == std::string::npos)
    return "";
  const std::size_t last = text.find_last_not_of(whitespace);
  return text.substr(first, last - first + 1);
}

}

class WaypointNavigator : public rclcpp::Node
{
public:
  WaypointNavigator()
    : Node("waypoint_navigator")
  {
    routeFile_ = declare_parameter<std::string>("route_file", "");
    routeFrame_ = declare_parameter<std::string>("route_frame", "map");
    odomTopic_ = declare_parameter<std::string>("odom_topic", "/odometry/fused");
    imuTopic_ = declare_parameter<std::string>("imu_topic", "/imu/control");
    trackingPointOffsetX_ = declare_parameter<double>("tracking_point_offset_x", 0.0);
    trackingPointOffsetY_ = declare_parameter<double>("tracking_point_offset_y", 0.0);
    if (!std::isfinite(trackingPointOffsetX_) || !std::isfinite(trackingPointOffsetY_))
      throw std::invalid_argument("tracking point offsets must be finite");
    fusionStatusTopic_ = declare_parameter<std::string>(
      "fusion_status_topic", "/odometry/fusion_status");
    actuatorHealthTopic_ = declare_parameter<std::string>(
      "actuator_health_topic", "/cup_car_serial/actuator_healthy");
    trackingStateTopic_ = declare_parameter<std::string>("tracking_state_topic", "/tracking_state");
    cmdVelTopic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    pathTopic_ = declare_parameter<std::string>("path_topic", "/waypoint_path");
    routeInputTopic_ = declare_parameter<std::string>(
      "route_input_topic", "/waypoint_navigation/route_input");
    startTopic_ = declare_parameter<std::string>(
      "start_topic", "/waypoint_navigation/start");
    statusTopic_ = declare_parameter<std::string>("status_topic", "/waypoint_navigation/status");
    currentWaypointTopic_ = declare_parameter<std::string>(
      "current_waypoint_topic", "/waypoint_navigation/current_waypoint");
    motionHoldStateTopic_ = declare_parameter<std::string>(
      "motion_hold_state_topic", "/waypoint_navigation/motion_hold_state");
    routeAckTopic_ = declare_parameter<std::string>(
      "route_ack_topic", "/waypoint_navigation/route_ack");

    controlFrequency_ = std::max(1.0, declare_parameter<double>("control_frequency", 30.0));
    defaultSpeed_ = std::max(0.0, declare_parameter<double>("default_speed", 0.50));
    maxLinearSpeed_ = std::max(0.0, declare_parameter<double>("max_linear_speed", 0.50));
    maxAngularSpeed_ = std::max(0.0, declare_parameter<double>("max_angular_speed", 0.95));
    maxPathAngularSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("max_path_angular_speed", 0.45)));
    minPathAngularSpeed_ = std::min(
      maxPathAngularSpeed_, std::max(
        0.0, declare_parameter<double>("min_path_angular_speed", 0.14)));
    pathAngularActivationError_ = std::max(
      0.0, declare_parameter<double>("path_angular_activation_error", 0.025));
    pathYawResponseThreshold_ = std::max(
      0.0, declare_parameter<double>("path_yaw_response_threshold", 0.04));
    maxLinearAcceleration_ = std::max(
      0.01, declare_parameter<double>("max_linear_acceleration", 0.40));
    maxLinearDeceleration_ = std::max(
      0.01, declare_parameter<double>("max_linear_deceleration", 0.80));
    effectiveBrakingDeceleration_ = std::max(
      0.01, declare_parameter<double>("effective_braking_deceleration", 0.35));
    brakingControlDelay_ = std::max(
      0.0, declare_parameter<double>("braking_control_delay", 0.20));
    brakingSafetyMargin_ = std::max(
      0.0, declare_parameter<double>("braking_safety_margin", 0.015));
    brakingDistanceFeedbackGain_ = std::max(
      0.0, declare_parameter<double>("braking_distance_feedback_gain", 1.0));
    maxAngularAcceleration_ = std::max(
      0.01, declare_parameter<double>("max_angular_acceleration", 1.80));
    maxAngularDeceleration_ = std::max(
      0.01, declare_parameter<double>("max_angular_deceleration", 3.00));
    linearGain_ = std::max(0.0, declare_parameter<double>("linear_gain", 0.8));
    angularGain_ = std::max(0.0, declare_parameter<double>("angular_gain", 2.8));
    pathPidKp_ = std::max(0.0, declare_parameter<double>("path_pid_kp", 2.4));
    pathPidKi_ = std::max(0.0, declare_parameter<double>("path_pid_ki", 0.15));
    pathPidKd_ = std::max(0.0, declare_parameter<double>("path_pid_kd", 0.0));
    pathYawRateDamping_ = std::max(
      0.0, declare_parameter<double>("path_yaw_rate_damping", 0.45));
    turnYawRateDamping_ = std::max(
      0.0, declare_parameter<double>("turn_yaw_rate_damping", 0.55));
    turnCruiseSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("turn_cruise_speed", 0.90)));
    turnPositionHoldGain_ = std::max(
      0.0, declare_parameter<double>("turn_position_hold_gain", 0.80));
    turnPositionHoldDeadband_ = std::max(
      0.0, declare_parameter<double>("turn_position_hold_deadband", 0.015));
    turnPositionHoldMaxSpeed_ = std::min(
      maxLinearSpeed_, std::max(
        0.0, declare_parameter<double>("turn_position_hold_max_speed", 0.08)));
    crossTrackGain_ = std::max(
      0.0, declare_parameter<double>("cross_track_gain", 1.5));
    stanleySofteningSpeed_ = std::max(
      0.01, declare_parameter<double>("stanley_softening_speed", 0.25));
    maxCrossTrackCorrection_ = std::max(
      0.0, declare_parameter<double>("max_cross_track_correction", 0.70));
    crossTrackSlowdownStart_ = std::max(
      0.0, declare_parameter<double>("cross_track_slowdown_start", 0.015));
    crossTrackSlowdownFull_ = std::max(
      crossTrackSlowdownStart_ + 0.001,
      declare_parameter<double>("cross_track_slowdown_full", 0.06));
    crossTrackMinimumSpeed_ = std::max(
      0.0, declare_parameter<double>("cross_track_minimum_speed", 0.20));
    maxLateralAcceleration_ = std::max(
      0.01, declare_parameter<double>("max_lateral_acceleration", 0.22));
    pathCurvatureFeedforwardGain_ = std::max(
      0.0, declare_parameter<double>("path_curvature_feedforward_gain", 1.0));
    pathPidIntegralLimit_ = std::max(
      0.0, declare_parameter<double>("path_pid_integral_limit", 0.20));
    rotateInPlaceThreshold_ = std::max(
      0.0, declare_parameter<double>("rotate_in_place_threshold", 0.18));
    rotateInPlaceReentryThreshold_ = std::max(
      rotateInPlaceThreshold_,
      declare_parameter<double>("rotate_in_place_reentry_threshold", 0.44));
    rotateInPlaceExitThreshold_ = std::min(
      rotateInPlaceThreshold_, std::max(
        0.0, declare_parameter<double>("rotate_in_place_exit_threshold", 0.035)));
    precisionTurnThreshold_ = std::max(
      rotateInPlaceExitThreshold_,
      declare_parameter<double>("precision_turn_threshold", 0.45));
    turnSettleYawRate_ = std::max(
      0.01, declare_parameter<double>("turn_settle_yaw_rate", 0.12));
    turnSettleDwell_ = std::max(
      0.0, declare_parameter<double>("turn_settle_dwell", 0.30));
    minPrecisionTurnSpeed_ = std::min(
      turnCruiseSpeed_, std::max(
        0.0, declare_parameter<double>("min_precision_turn_speed", 0.25)));
    waypointTolerance_ = std::max(
      0.001, declare_parameter<double>("waypoint_tolerance", 0.04));
    waypointPassLongitudinalTolerance_ = std::max(
      0.0, declare_parameter<double>("waypoint_pass_longitudinal_tolerance", 0.01));
    waypointPassLateralTolerance_ = std::max(
      waypointTolerance_,
      declare_parameter<double>("waypoint_pass_lateral_tolerance", 0.06));
    waypointRecoverySpeed_ = std::min(
      maxLinearSpeed_, std::max(
        0.0, declare_parameter<double>("waypoint_recovery_speed", 0.10)));
    waypointRecoveryHeadingTolerance_ = std::max(
      0.01, declare_parameter<double>("waypoint_recovery_heading_tolerance", 0.12));
    waypointRecoveryMaxAngularSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("waypoint_recovery_max_angular_speed", 0.60)));
    const double pathProgressTimeout = std::max(
      0.0, declare_parameter<double>("path_progress_timeout", 10.0));
    const double pathProgressMinimumAdvance = std::max(
      0.0, declare_parameter<double>("path_progress_minimum_advance", 0.05));
    const double pathProgressMinimumActualDisplacement = std::max(
      0.0, declare_parameter<double>(
        "path_progress_minimum_actual_displacement", 0.25));
    const int pathProgressMaxRecoveryAttempts = std::max(
      0, static_cast<int>(declare_parameter<int64_t>(
        "path_progress_max_recovery_attempts", 2)));
    pathProgressSupervisor_ = visual_navigation::PathProgressSupervisor(
      {pathProgressTimeout, pathProgressMinimumAdvance,
        pathProgressMinimumActualDisplacement,
        pathProgressMaxRecoveryAttempts});
    const double turnProgressStallTimeout = std::max(
      0.0, declare_parameter<double>("turn_progress_stall_timeout", 6.0));
    const double turnProgressMinimumErrorReduction = std::max(
      0.0, declare_parameter<double>("turn_progress_minimum_error_reduction", 0.08));
    const double turnProgressBaseTotalTimeout = std::max(
      0.0, declare_parameter<double>("turn_progress_base_total_timeout", 12.0));
    const double turnProgressMinimumExpectedYawRate = std::max(
      0.001, declare_parameter<double>("turn_progress_minimum_expected_yaw_rate", 0.12));
    const double turnProgressMaxTotalTimeout = std::max(
      turnProgressBaseTotalTimeout,
      declare_parameter<double>("turn_progress_max_total_timeout", 40.0));
    const int turnProgressMaxRecoveryAttempts = std::max(
      0, static_cast<int>(declare_parameter<int64_t>(
        "turn_progress_max_recovery_attempts", 2)));
    turnProgressSupervisor_ = visual_navigation::TurnProgressSupervisor(
      {turnProgressStallTimeout, turnProgressMinimumErrorReduction,
        turnProgressBaseTotalTimeout, turnProgressMinimumExpectedYawRate,
        turnProgressMaxTotalTimeout, turnProgressMaxRecoveryAttempts});
    preTurnStopHeadingThreshold_ = std::max(
      0.0, declare_parameter<double>("pre_turn_stop_heading_threshold", 0.18));
    preTurnStopSpeed_ = std::max(
      0.0, declare_parameter<double>("pre_turn_stop_speed", 0.03));
    preTurnStopDwell_ = std::max(
      0.0, declare_parameter<double>("pre_turn_stop_dwell", 0.10));
    stopMotionWindow_ = std::max(
      0.05, declare_parameter<double>("stop_motion_window", 0.20));
    preTurnMinimumStopTime_ = std::max(
      0.0, declare_parameter<double>("pre_turn_minimum_stop_time", 0.20));
    preTurnBrakeTimeout_ = std::max(
      preTurnMinimumStopTime_, declare_parameter<double>("pre_turn_brake_timeout", 0.60));
    finalYawTolerance_ = std::max(
      0.001, declare_parameter<double>("final_yaw_tolerance", 0.060));
    finalYawMaxAngularSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("final_yaw_max_angular_speed", 0.90)));
    finalYawMinTurnSpeed_ = std::min(
      finalYawMaxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("final_yaw_min_turn_speed", 0.70)));
    finalYawMinPrecisionSpeed_ = std::min(
      finalYawMinTurnSpeed_, std::max(
        0.0, declare_parameter<double>("final_yaw_min_precision_speed", 0.22)));
    odomTimeout_ = std::max(0.01, declare_parameter<double>("odom_timeout", 0.40));
    imuTimeout_ = std::max(0.01, declare_parameter<double>("imu_timeout", 0.15));
    fusionStatusTimeout_ = std::max(
      0.01, declare_parameter<double>("fusion_status_timeout", 0.60));
    actuatorHealthTimeout_ = std::max(
      0.01, declare_parameter<double>("actuator_health_timeout", 0.80));
    stampFutureTolerance_ = std::max(
      0.0, declare_parameter<double>("stamp_future_tolerance", 0.20));
    const double transientLocalizationGrace = std::max(
      0.0, declare_parameter<double>("transient_localization_grace", 2.00));
    const double transientFaultSpeedScale = Clamp(
      declare_parameter<double>("transient_fault_speed_scale", 0.25), 0.0, 1.0);
    requireFusionStatus_ = declare_parameter<bool>("require_fusion_status", true);
    requireActuatorHealth_ = declare_parameter<bool>("require_actuator_health", false);
    allowedFusionStates_ = declare_parameter<std::vector<std::string>>(
      "allowed_fusion_states",
      {"FULL", "DEGRADED_NO_VISION", "DEGRADED_NO_IMU", "DEGRADED_NO_WHEEL",
        "DEGRADED_VISION_ONLY", "DEGRADED_WHEEL_ONLY", "DEGRADED_VISUAL_REALIGNED"});
    visual_navigation::FusionSpeedScales fusionSpeedScales;
    fusionSpeedScales.fallback = declare_parameter<double>("degraded_speed_scale", 0.50);
    fusionSpeedScales.no_vision = declare_parameter<double>("no_vision_speed_scale", 0.50);
    fusionSpeedScales.no_imu = declare_parameter<double>("no_imu_speed_scale", 0.65);
    fusionSpeedScales.no_wheel = declare_parameter<double>("no_wheel_speed_scale", 0.60);
    fusionSpeedScales.vision_only = declare_parameter<double>("vision_only_speed_scale", 0.40);
    fusionSpeedScales.wheel_only = declare_parameter<double>("wheel_only_speed_scale", 0.25);
    fusionSpeedScales.visual_realigned = declare_parameter<double>(
      "visual_realigned_speed_scale", 0.50);
    fusionHealthPolicy_ = visual_navigation::FusionHealthPolicy(
      allowedFusionStates_, fusionSpeedScales);
    requireTrackingState_ = declare_parameter<bool>("require_tracking_state", false);
    const bool abortOnTrackingLoss =
      declare_parameter<bool>("abort_on_tracking_loss", false);
    navigationSupervisor_ = visual_navigation::NavigationSupervisor(
      {transientLocalizationGrace, transientFaultSpeedScale, abortOnTrackingLoss});
    autostart_ = declare_parameter<bool>("autostart", false);

    cmdVelPublisher_ = create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic_, 10);
    pathPublisher_ = create_publisher<nav_msgs::msg::Path>(
      pathTopic_, rclcpp::QoS(1).transient_local().reliable());
    statusPublisher_ = create_publisher<std_msgs::msg::String>(
      statusTopic_, rclcpp::QoS(1).transient_local().reliable());
    currentWaypointPublisher_ = create_publisher<std_msgs::msg::Int32>(
      currentWaypointTopic_, rclcpp::QoS(1).transient_local().reliable());
    motionHoldStatePublisher_ =
      create_publisher<mission_control_interfaces::msg::MotionHoldState>(
      motionHoldStateTopic_, rclcpp::QoS(1).transient_local().reliable());
    routeAckPublisher_ = create_publisher<std_msgs::msg::UInt64>(
      routeAckTopic_, rclcpp::QoS(1).transient_local().reliable());

    odomSubscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odomTopic_, 10,
      std::bind(&WaypointNavigator::HandleOdometry, this, std::placeholders::_1));
    imuSubscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imuTopic_, rclcpp::SensorDataQoS(),
      std::bind(&WaypointNavigator::HandleImu, this, std::placeholders::_1));
    fusionStatusSubscription_ = create_subscription<std_msgs::msg::String>(
      fusionStatusTopic_, 10,
      std::bind(&WaypointNavigator::HandleFusionStatus, this, std::placeholders::_1));
    actuatorHealthSubscription_ = create_subscription<std_msgs::msg::Bool>(
      actuatorHealthTopic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WaypointNavigator::HandleActuatorHealth, this, std::placeholders::_1));
    trackingStateSubscription_ = create_subscription<std_msgs::msg::Int32>(
      trackingStateTopic_, 10,
      std::bind(&WaypointNavigator::HandleTrackingState, this, std::placeholders::_1));
    routeInputSubscription_ = create_subscription<nav_msgs::msg::Path>(
      routeInputTopic_, rclcpp::QoS(1).reliable(),
      std::bind(&WaypointNavigator::HandleRouteInput, this, std::placeholders::_1));
    startSubscription_ = create_subscription<std_msgs::msg::Empty>(
      startTopic_, rclcpp::QoS(1).reliable(),
      std::bind(&WaypointNavigator::HandleStartTopic, this, std::placeholders::_1));

    startService_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      std::bind(
        &WaypointNavigator::HandleStart, this,
        std::placeholders::_1, std::placeholders::_2));
    stopService_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(
        &WaypointNavigator::HandleStop, this,
        std::placeholders::_1, std::placeholders::_2));
    resetService_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(
        &WaypointNavigator::HandleReset, this,
        std::placeholders::_1, std::placeholders::_2));
    motionHoldService_ = create_service<mission_control_interfaces::srv::SetMotionHold>(
      "~/set_motion_hold",
      std::bind(
        &WaypointNavigator::HandleSetMotionHold, this,
        std::placeholders::_1, std::placeholders::_2));

    routeLoaded_ = LoadRoute(routeFile_);
    if (routeLoaded_)
    {
      PublishRoutePath();
      PublishCurrentWaypoint();
    }

    const auto period = std::chrono::duration<double>(1.0 / controlFrequency_);
    controlTimer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(period.count()),
      std::bind(&WaypointNavigator::RunControl, this));

    if (autostart_ && routeLoaded_)
    {
      navigationActive_ = true;
      SetState("WAITING_FOR_LOCALIZATION");
    }
    else
    {
      SetState(routeFile_.empty() ? "WAITING_FOR_ROUTE" : "FAULT_ROUTE_NOT_LOADED");
    }
    PublishMotionHoldState();

    RCLCPP_INFO(
      get_logger(),
      "Waypoint navigator ready: odom=%s fusion_status=%s actuator_health=%s "
      "route_input=%s output=%s",
      odomTopic_.c_str(), fusionStatusTopic_.c_str(), actuatorHealthTopic_.c_str(),
      routeInputTopic_.c_str(), cmdVelTopic_.c_str());
  }

  ~WaypointNavigator() override
  {
    PublishStop();
  }

private:
  using Waypoint = visual_navigation::RouteWaypoint;

  struct PositionSample
  {
    std::chrono::steady_clock::time_point time;
    double x{0.0};
    double y{0.0};
  };

  enum class WaitAction
  {
    NONE,
    ADVANCE,
    COMPLETE
  };

  bool LoadRoute(const std::string &routeFile)
  {
    std::vector<Waypoint> loadedWaypoints;
    std::string error;
    if (!visual_navigation::RouteManager::LoadCsv(
        routeFile, defaultSpeed_, waypointTolerance_, loadedWaypoints, error))
    {
      if (routeFile.empty())
        RCLCPP_INFO(get_logger(), "%s; waiting for a dynamic route", error.c_str());
      else
        RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return false;
    }

    waypoints_ = std::move(loadedWaypoints);
    RCLCPP_INFO(
      get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), routeFile.c_str());
    return true;
  }

  void PublishRoutePath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = routeFrame_;
    path.poses.reserve(waypoints_.size());

    for (std::size_t index = 0; index < waypoints_.size(); ++index)
    {
      const Waypoint &waypoint = waypoints_[index];
      double yaw = waypoint.yaw;
      if (!std::isfinite(yaw))
      {
        if (index + 1 < waypoints_.size())
        {
          yaw = std::atan2(
            waypoints_[index + 1].y - waypoint.y,
            waypoints_[index + 1].x - waypoint.x);
        }
        else if (index > 0)
        {
          yaw = std::atan2(
            waypoint.y - waypoints_[index - 1].y,
            waypoint.x - waypoints_[index - 1].x);
        }
        else
        {
          yaw = 0.0;
        }
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.orientation = QuaternionFromYaw(yaw);
      path.poses.push_back(pose);
    }

    pathPublisher_->publish(path);
  }

  void PublishRouteAck(uint64_t route_id)
  {
    std_msgs::msg::UInt64 message;
    message.data = route_id;
    routeAckPublisher_->publish(message);
  }

  void HandleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!AcceptMeasurementStamp(
        message->header.stamp, lastOdomStamp_, odomStampValid_, odomTimeout_, "odometry"))
      return;
    const bool odometryFrameValid = message->header.frame_id.empty() ||
      message->header.frame_id == routeFrame_;
    const bool odometryPoseValid =
      std::isfinite(message->pose.pose.position.x) &&
      std::isfinite(message->pose.pose.position.y) &&
      QuaternionIsValid(message->pose.pose.orientation);
    const auto odometryReceivedAt = now();
    inputCache_.update_odometry(
      odometryFrameValid, odometryPoseValid, false, 0.0, odometryReceivedAt);

    if (!odometryFrameValid)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejecting odometry frame '%s'; route frame is '%s' and TF conversion is disabled",
        message->header.frame_id.c_str(), routeFrame_.c_str());
      return;
    }
    if (!odometryPoseValid)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejecting invalid odometry pose");
      return;
    }

    currentYaw_ = YawFromValidQuaternion(message->pose.pose.orientation);
    if (!trackingReferenceInitialized_)
    {
      trackingReferenceYaw_ = currentYaw_;
      trackingReferenceInitialized_ = true;
    }
    const auto basePosition = visual_navigation::BasePositionFromTrackingPoint(
      message->pose.pose.position.x, message->pose.pose.position.y,
      currentYaw_, trackingReferenceYaw_, trackingPointOffsetX_, trackingPointOffsetY_);
    currentX_ = basePosition.x;
    currentY_ = basePosition.y;
    UpdateObservedLinearSpeed();
    const bool odometryVelocityValid = std::isfinite(message->twist.twist.linear.x);
    const double odometryVelocity = odometryVelocityValid ? message->twist.twist.linear.x : 0.0;
    inputCache_.update_odometry(
      odometryFrameValid, odometryPoseValid, odometryVelocityValid,
      odometryVelocity, odometryReceivedAt);
  }

  void HandleImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    if (!AcceptMeasurementStamp(
        message->header.stamp, lastImuStamp_, imuStampValid_, imuTimeout_, "IMU"))
      return;
    if (!std::isfinite(message->angular_velocity.z))
      return;
    imuYawRate_ = message->angular_velocity.z;
    inputCache_.update_imu(now());
  }

  void HandleTrackingState(const std_msgs::msg::Int32::SharedPtr message)
  {
    inputCache_.update_tracking(message->data);
  }

  void HandleFusionStatus(const std_msgs::msg::String::SharedPtr message)
  {
    inputCache_.update_fusion_status(Trim(message->data), now());
  }

  void HandleActuatorHealth(const std_msgs::msg::Bool::SharedPtr message)
  {
    inputCache_.update_actuator(message->data, now());
  }

  void HandleRouteInput(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (message->poses.empty())
    {
      RCLCPP_WARN(get_logger(), "Ignoring empty dynamic waypoint route");
      return;
    }

    if (!message->header.frame_id.empty() && message->header.frame_id != routeFrame_)
    {
      RCLCPP_ERROR(
        get_logger(), "Ignoring dynamic route in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), routeFrame_.c_str());
      return;
    }
    const uint64_t route_id = static_cast<uint64_t>(
      rclcpp::Time(message->header.stamp).nanoseconds());
    if (route_id == 0U) {
      RCLCPP_ERROR(get_logger(), "Ignoring dynamic route without a non-zero route id");
      return;
    }

    std::vector<Waypoint> loadedWaypoints;
    std::size_t invalidWaypointIndex = 0;
    std::string routeError;
    if (!visual_navigation::RouteManager::LoadPath(
        *message, defaultSpeed_, waypointTolerance_, loadedWaypoints,
        invalidWaypointIndex, routeError))
    {
      RCLCPP_ERROR(
        get_logger(), "Ignoring dynamic route: waypoint %zu %s",
        invalidWaypointIndex, routeError.c_str());
      return;
    }

    if (routeLoaded_ && visual_navigation::RouteManager::Equivalent(waypoints_, loadedWaypoints))
    {
      // Re-publish the accepted path so the editor receives its acknowledgement,
      // but never reset a route that is already running. After a latched fault,
      // accept a retry only once localization and the actuator are healthy again.
      if (navigationActive_ ||
        !visual_navigation::EvaluateNavigationStart(CurrentNavigationInputs()).ready)
      {
        PublishRoutePath();
        PublishRouteAck(route_id);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring duplicate route while navigation is active or unhealthy");
        return;
      }
    }

    // Stop the old route before atomically replacing it with the new one.
    navigationActive_ = false;
    currentWaypointIndex_ = 0;
    finalPositionCaptured_ = false;
    navigationSupervisor_.ResetLocalizationHistory();
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    waitAction_ = WaitAction::NONE;
    pathSegmentInitialized_ = false;
    resumePathFromCurrentPose_ = false;
    ResetPathPid();
    PublishStop();

    waypoints_ = std::move(loadedWaypoints);
    routeLoaded_ = true;
    navigationActive_ = true;
    PublishRoutePath();
    PublishCurrentWaypoint();
    PublishRouteAck(route_id);
    SetState(MotionHeld() ? "HELD_FOR_MISSION" : "WAITING_FOR_LOCALIZATION");
    RCLCPP_INFO(
      get_logger(), "Loaded and activated %zu dynamic waypoints",
      waypoints_.size());
  }

  void HandleStart(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->success = ActivateNavigation(response->message);
  }

  void HandleStartTopic(const std_msgs::msg::Empty::SharedPtr)
  {
    std::string result;
    if (ActivateNavigation(result))
      RCLCPP_INFO(get_logger(), "Navigation start topic accepted: %s", result.c_str());
    else
      RCLCPP_WARN(get_logger(), "Navigation start topic rejected: %s", result.c_str());
  }

  bool ActivateNavigation(std::string & result)
  {
    if (!routeLoaded_)
    {
      result = "Waypoint route is not loaded";
      return false;
    }

    const auto readiness =
      visual_navigation::EvaluateNavigationStart(CurrentNavigationInputs());
    if (!readiness.ready)
    {
      navigationActive_ = false;
      PublishStop();
      SetState(readiness.failure_state);
      result = "Navigation start rejected: " + readiness.failure_state;
      return false;
    }

    if (currentWaypointIndex_ >= waypoints_.size())
      currentWaypointIndex_ = 0;
    waitAction_ = WaitAction::NONE;
    finalPositionCaptured_ = false;
    navigationActive_ = true;
    navigationSupervisor_.ResetLocalizationHistory();
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    pathSegmentInitialized_ = false;
    ResetPathPid();
    SetState(MotionHeld() ? "HELD_FOR_MISSION" : "WAITING_FOR_LOCALIZATION");
    result = "Waypoint navigation started";
    return true;
  }

  void HandleStop(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    navigationActive_ = false;
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    waitAction_ = WaitAction::NONE;
    finalPositionCaptured_ = false;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    PublishStop();
    SetState("IDLE");
    response->success = true;
    response->message = "Waypoint navigation stopped";
  }

  void HandleSetMotionHold(
    const mission_control_interfaces::srv::SetMotionHold::Request::SharedPtr request,
    mission_control_interfaces::srv::SetMotionHold::Response::SharedPtr response)
  {
    const std::string source = Trim(request->source);
    const std::string reason = Trim(request->reason);
    if (source.empty() || source.size() > 128)
    {
      response->success = false;
      response->motion_held = MotionHeld();
      response->active_sources = ActiveHoldSources();
      response->message = "source must contain 1 to 128 non-whitespace characters";
      return;
    }
    if (reason.size() > 256)
    {
      response->success = false;
      response->motion_held = MotionHeld();
      response->active_sources = ActiveHoldSources();
      response->message = "reason must not exceed 256 characters";
      return;
    }

    const bool wasHeld = MotionHeld();
    if (request->hold)
    {
      motionHolds_[source] = reason;
      if (!wasHeld)
      {
        motionHoldStartedAt_ = now();
        motionHoldStartInitialized_ = true;
        pathProgressSupervisor_.Reset();
        turnProgressSupervisor_.Reset();
        pathSegmentInitialized_ = false;
        finalPositionCaptured_ = false;
        ResetPathPid();
      }
      PublishStop();
      SetState("HELD_FOR_MISSION");
      response->message = "Motion hold acquired for " + source;
    }
    else
    {
      const bool sourceWasActive = motionHolds_.erase(source) > 0;
      if (wasHeld && !MotionHeld())
      {
        if (waitAction_ != WaitAction::NONE && motionHoldStartInitialized_)
        {
          const rclcpp::Duration holdDuration = now() - motionHoldStartedAt_;
          if (holdDuration.nanoseconds() > 0)
            waitUntil_ = waitUntil_ + holdDuration;
        }
        motionHoldStartInitialized_ = false;
        navigationSupervisor_.ResetLocalizationHistory();
        pathSegmentInitialized_ = false;
        resumePathFromCurrentPose_ = true;
        finalPositionCaptured_ = false;
        ResetPathPid();
        SetState(navigationActive_ ? "RESUME_PENDING_LOCALIZATION" : "IDLE");
      }
      response->message = sourceWasActive ?
        "Motion hold released for " + source :
        "No motion hold was active for " + source;
    }

    PublishMotionHoldState();
    response->success = true;
    response->motion_held = MotionHeld();
    response->active_sources = ActiveHoldSources();
    RCLCPP_INFO(
      get_logger(), "Motion hold request: source=%s hold=%s active_sources=%zu",
      source.c_str(), request->hold ? "true" : "false", motionHolds_.size());
  }

  void HandleReset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    navigationActive_ = false;
    currentWaypointIndex_ = 0;
    finalPositionCaptured_ = false;
    navigationSupervisor_.ResetLocalizationHistory();
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    waitAction_ = WaitAction::NONE;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    PublishStop();
    PublishCurrentWaypoint();
    SetState("IDLE");
    response->success = true;
    response->message = "Waypoint navigation reset";
  }

  bool ImuYawRateIsFresh() const
  {
    return inputCache_.imu_fresh(now(), imuTimeout_);
  }

  bool AcceptMeasurementStamp(
    const builtin_interfaces::msg::Time & stamp_message,
    rclcpp::Time & last_stamp, bool & stamp_valid, double max_age,
    const char * source)
  {
    const rclcpp::Time stamp(stamp_message);
    if (stamp.nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting %s measurement with an unset timestamp", source);
      return false;
    }
    if (stamp_valid && stamp <= last_stamp) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting non-monotonic %s timestamp", source);
      return false;
    }
    const rclcpp::Time current = now();
    if (current.nanoseconds() > 0) {
      const double age = (current - stamp).seconds();
      if (age > max_age || age < -stampFutureTolerance_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting %s timestamp with age %.3fs", source, age);
        return false;
      }
    }
    last_stamp = stamp;
    stamp_valid = true;
    return true;
  }

  double ControlYawRate() const
  {
    return ImuYawRateIsFresh() ? imuYawRate_ : 0.0;
  }

  visual_navigation::NavigationInputStatus CurrentNavigationInputs() const
  {
    auto input = inputCache_.snapshot(
      now(), odomTimeout_, fusionStatusTimeout_, actuatorHealthTimeout_,
      requireFusionStatus_, requireTrackingState_, requireActuatorHealth_);
    input.fusion_health = fusionHealthPolicy_.Evaluate(inputCache_.fusion_status());
    return input;
  }

  void RunControl()
  {
    if (MotionHeld())
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      PublishStop();
      SetState("HELD_FOR_MISSION");
      return;
    }

    if (!navigationActive_)
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      PublishStop();
      return;
    }

    const auto supervision = navigationSupervisor_.Evaluate(
      CurrentNavigationInputs(), std::chrono::steady_clock::now());
    if (supervision.action != visual_navigation::NavigationRuntimeAction::DRIVE)
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      PublishStop();
      if (supervision.reset_path_control)
        ResetPathPid();
      if (supervision.action == visual_navigation::NavigationRuntimeAction::STOP_AND_LATCH)
        navigationActive_ = false;
      SetState(supervision.state);
      return;
    }
    auto fusionHealth = supervision.fusion_health;
    const bool progressSupervisionAllowed =
      supervision.state != "DEGRADED_TRANSIENT_LOCALIZATION";
    if (!progressSupervisionAllowed)
    {
      pathProgressSupervisor_.Pause();
      turnProgressSupervisor_.Pause();
    }
    if (!supervision.state.empty())
      SetState(supervision.state);

    if (waitAction_ != WaitAction::NONE)
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      PublishStop();
      if (now() < waitUntil_)
      {
        SetState("WAITING_AT_WAYPOINT");
        return;
      }

      const WaitAction completedAction = waitAction_;
      waitAction_ = WaitAction::NONE;
      if (completedAction == WaitAction::ADVANCE)
      {
        ++currentWaypointIndex_;
        pathSegmentInitialized_ = false;
        ResetPathPid();
        PublishCurrentWaypoint();
      }
      else
      {
        CompleteNavigation();
        return;
      }
    }

    if (currentWaypointIndex_ >= waypoints_.size())
    {
      CompleteNavigation();
      return;
    }

    const Waypoint &target = waypoints_[currentWaypointIndex_];
    if (!pathSegmentInitialized_)
      BeginPathSegment();

    const double deltaX = target.x - currentX_;
    const double deltaY = target.y - currentY_;
    const double distance = std::hypot(deltaX, deltaY);
    const double controlYawRate = ControlYawRate();
    const bool finalWaypoint = currentWaypointIndex_ + 1 == waypoints_.size();
    const double pathDeltaX = target.x - pathSegmentStartX_;
    const double pathDeltaY = target.y - pathSegmentStartY_;
    // Duplicate waypoints can occur at planner cell/curve joins.  A zero-length
    // segment has no defined heading; steer directly toward the target until it
    // is consumed instead of entering a spurious ROTATING_TO_PATH loop.
    const double pathLength = std::hypot(pathDeltaX, pathDeltaY);
    const double pathHeading = pathLength > 1.0e-6 ?
      std::atan2(pathDeltaY, pathDeltaX) : std::atan2(deltaY, deltaX);
    const auto pathProjection = visual_navigation::ProjectOntoPathSegment(
      pathSegmentStartX_, pathSegmentStartY_, target.x, target.y,
      currentX_, currentY_);
    const bool waypointRequiresStop = WaypointRequiresStop(pathHeading, finalWaypoint) ||
      target.stop_time > 0.0;
    const bool continuousWaypointPassed =
      !finalPositionRecoveryActive_ &&
      visual_navigation::ContinuousPathWaypointPassed(
        finalWaypoint, waypointRequiresStop, pathProjection,
        waypointPassLongitudinalTolerance_);
    const bool waypointReached = continuousWaypointPassed ||
      (finalPositionRecoveryActive_ ?
      distance <= target.tolerance :
      visual_navigation::WaypointReached(
        distance, target.tolerance, pathProjection,
        waypointPassLongitudinalTolerance_, waypointPassLateralTolerance_));
    finalPositionCaptured_ = finalPositionCaptured_ ||
      (finalWaypoint && waypointReached);

    if (visual_navigation::ShouldBeginWaypointBrake(
        waypointBraking_, waypointBrakeCompleted_,
        waypointReached, waypointRequiresStop))
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      BeginWaypointBraking();
      PublishStop();
      SetState("BRAKING_AT_WAYPOINT");
      return;
    }
    if (waypointBraking_)
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      PublishStop();
      if (!WaypointBrakeHasCompleted())
      {
        if (waypointBrakeTimedOut_)
        {
          navigationActive_ = false;
          ResetPathPid();
          SetState("FAULT_WAYPOINT_BRAKE_TIMEOUT");
        }
        else
        {
          SetState("BRAKING_AT_WAYPOINT");
        }
        return;
      }

      waypointBraking_ = false;
      waypointBrakeCompleted_ = true;
      if (finalWaypoint)
        finalPositionRecoveryActive_ = false;
      waypointStopTimerInitialized_ = false;
      waypointSpeedSettleTimerInitialized_ = false;
      if (!finalWaypoint)
      {
        if (target.stop_time > 0.0)
        {
          waitUntil_ = now() + rclcpp::Duration::from_seconds(target.stop_time);
          waitAction_ = WaitAction::ADVANCE;
          SetState("WAITING_AT_WAYPOINT");
          return;
        }
        ++currentWaypointIndex_;
        pathSegmentInitialized_ = false;
        ResetPathPid();
        PublishCurrentWaypoint();
        return;
      }
    }

    if (finalPositionCaptured_)
    {
      pathProgressSupervisor_.Reset();
      if (std::isfinite(target.yaw))
      {
        const double finalYawError = NormalizeAngle(target.yaw - currentYaw_);
        const bool finalYawNeedsControl =
          std::abs(finalYawError) > finalYawTolerance_ ||
          std::abs(controlYawRate) > turnSettleYawRate_ || rotatingInPlace_;
        if (finalYawNeedsControl)
        {
          if (SuperviseTurnProgress(
              2, std::abs(finalYawError),
              finalYawMaxAngularSpeed_ * fusionHealth.speed_scale,
              progressSupervisionAllowed))
          {
            return;
          }
          if (!rotatingInPlace_)
          {
            rotatingInPlace_ = true;
            turnAnchorX_ = currentX_;
            turnAnchorY_ = currentY_;
            turnAnchorValid_ = true;
            turnSettling_ = false;
            turnSettleTimerInitialized_ = false;
          }
          if (!turnAnchorValid_)
          {
            turnAnchorX_ = currentX_;
            turnAnchorY_ = currentY_;
            turnAnchorValid_ = true;
          }

          geometry_msgs::msg::Twist command;
          if (std::abs(finalYawError) <= finalYawTolerance_)
            turnSettling_ = true;
          if (turnSettling_)
          {
            PublishMotionCommand(command);
            const double turnActivity = std::max(
              std::abs(controlYawRate), std::abs(lastMotionCommand_.angular.z));
            if (turnActivity > turnSettleYawRate_)
            {
              turnSettleTimerInitialized_ = false;
              SetState("ALIGNING_FINAL_YAW");
              return;
            }

            const auto settleTime = std::chrono::steady_clock::now();
            if (!turnSettleTimerInitialized_)
            {
              turnSettleStarted_ = settleTime;
              turnSettleTimerInitialized_ = true;
            }
            const double settledSeconds =
              std::chrono::duration<double>(settleTime - turnSettleStarted_).count();
            if (!visual_navigation::TurnHasSettled(
                turnActivity, settledSeconds,
                turnSettleYawRate_, turnSettleDwell_))
            {
              SetState("ALIGNING_FINAL_YAW");
              return;
            }

            if (std::abs(finalYawError) <= finalYawTolerance_)
            {
              if (!visual_navigation::FinalPositionCanComplete(
                  finalPositionCaptured_, distance, target.tolerance))
              {
                BeginFinalPositionRecovery();
                return;
              }
              CompleteNavigation();
              return;
            }
            turnSettling_ = false;
            turnSettleTimerInitialized_ = false;
            SetState("ALIGNING_FINAL_YAW");
            return;
          }

          turnSettleTimerInitialized_ = false;
          const double scaledMaxAngularSpeed =
            finalYawMaxAngularSpeed_ * fusionHealth.speed_scale;
          const double requestedAngularSpeed = Clamp(
            angularGain_ * finalYawError - turnYawRateDamping_ * controlYawRate,
            -scaledMaxAngularSpeed, scaledMaxAngularSpeed);
          const double minimumTurnSpeed = visual_navigation::SelectMinimumTurnSpeed(
            std::abs(finalYawError), precisionTurnThreshold_,
            finalYawMinPrecisionSpeed_, finalYawMinTurnSpeed_);
          command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
            requestedAngularSpeed, finalYawError,
            minimumTurnSpeed * fusionHealth.speed_scale, scaledMaxAngularSpeed);
          command.linear.x = visual_navigation::TurnPositionHoldSpeed(
            currentX_, currentY_, currentYaw_, turnAnchorX_, turnAnchorY_,
            turnPositionHoldGain_, turnPositionHoldDeadband_,
            turnPositionHoldMaxSpeed_) * fusionHealth.speed_scale;
          PublishMotionCommand(command);
          SetState("ALIGNING_FINAL_YAW");
          return;
        }
      }

      turnProgressSupervisor_.Reset();

      if (!visual_navigation::FinalPositionCanComplete(
          finalPositionCaptured_, distance, target.tolerance))
      {
        BeginFinalPositionRecovery();
        return;
      }

      if (target.stop_time > 0.0)
      {
        waitUntil_ = now() + rclcpp::Duration::from_seconds(target.stop_time);
        waitAction_ = WaitAction::COMPLETE;
        PublishStop();
        SetState("WAITING_AT_WAYPOINT");
        return;
      }

      CompleteNavigation();
      return;
    }

    if (waypointReached)
    {
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      ++currentWaypointIndex_;
      pathSegmentInitialized_ = false;
      ResetPathPid();
      PublishCurrentWaypoint();
      return;
    }

    if (waypointRequiresStop && !waypointRecoveryActive_ &&
      visual_navigation::WaypointNeedsRecovery(
        pathProjection, waypointPassLongitudinalTolerance_,
        waypointPassLateralTolerance_))
    {
      ResetPathPid();
      waypointRecoveryActive_ = true;
    }

    if (waypointRecoveryActive_)
    {
      const double recoveryHeading = std::atan2(deltaY, deltaX);
      const double recoveryHeadingError = NormalizeAngle(recoveryHeading - currentYaw_);
      const double scaledMaxAngularSpeed =
        waypointRecoveryMaxAngularSpeed_ * fusionHealth.speed_scale;
      const double requestedAngularSpeed = Clamp(
        angularGain_ * recoveryHeadingError - turnYawRateDamping_ * controlYawRate,
        -scaledMaxAngularSpeed, scaledMaxAngularSpeed);
      geometry_msgs::msg::Twist command;
      if (std::abs(recoveryHeadingError) > waypointRecoveryHeadingTolerance_)
      {
        command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
          requestedAngularSpeed, recoveryHeadingError,
          minPrecisionTurnSpeed_ * fusionHealth.speed_scale,
          scaledMaxAngularSpeed);
      }
      else
      {
        command.angular.z = requestedAngularSpeed;
        command.linear.x = std::min(
          waypointRecoverySpeed_, linearGain_ * distance) *
          std::max(0.0, std::cos(recoveryHeadingError)) * fusionHealth.speed_scale;
      }
      const double recoveryProgress =
        CompletedRouteLengthBeforeCurrentSegment() +
        Clamp(pathLength - distance, 0.0, pathLength);
      if (SupervisePathProgress(recoveryProgress, progressSupervisionAllowed))
        return;
      if (std::abs(recoveryHeadingError) > waypointRecoveryHeadingTolerance_)
      {
        if (SuperviseTurnProgress(
            1, std::abs(recoveryHeadingError), scaledMaxAngularSpeed,
            progressSupervisionAllowed))
        {
          return;
        }
      }
      else
      {
        turnProgressSupervisor_.Reset();
      }
      PublishMotionCommand(command);
      SetState("RECOVERING_WAYPOINT");
      return;
    }

    const double headingError = NormalizeAngle(pathHeading - currentYaw_);
    // Positive cross-track error means the car is to the left of the path.
    const double crossTrackError =
      std::cos(pathHeading) * (currentY_ - pathSegmentStartY_) -
      std::sin(pathHeading) * (currentX_ - pathSegmentStartX_);
    const double requestedSpeed = std::min(target.speed, maxLinearSpeed_);
    double signedPathCurvature = 0.0;
    if (currentWaypointIndex_ + 1 < waypoints_.size()) {
      const Waypoint & next = waypoints_[currentWaypointIndex_ + 1];
      const double nextLength = std::hypot(next.x - target.x, next.y - target.y);
      if (nextLength > 1.0e-4) {
        const double nextHeading = std::atan2(next.y - target.y, next.x - target.x);
        signedPathCurvature = NormalizeAngle(nextHeading - pathHeading) /
          std::max(0.02, nextLength);
      }
    }
    const double pathCurvature = std::abs(signedPathCurvature);
    const double pathError = visual_navigation::StanleyPathError(
      headingError, crossTrackError, crossTrackGain_, requestedSpeed,
      stanleySofteningSpeed_, maxCrossTrackCorrection_);
    const double rotationEntryThreshold = visual_navigation::RotationEntryThreshold(
      pathAlignmentCompleted_, rotateInPlaceThreshold_, rotateInPlaceReentryThreshold_);
    const bool shouldRotateInPlace = visual_navigation::ShouldRotateInPlace(
      false, std::abs(headingError), std::abs(controlYawRate),
      rotationEntryThreshold, rotateInPlaceExitThreshold_, turnSettleYawRate_);

    if (!rotatingInPlace_ && shouldRotateInPlace)
    {
      ResetPathPid();
      rotatingInPlace_ = true;
      turnAnchorX_ = currentX_;
      turnAnchorY_ = currentY_;
      turnAnchorValid_ = true;
      turnSettling_ = false;
    }
    else if (!rotatingInPlace_ && !pathAlignmentCompleted_)
    {
      pathAlignmentCompleted_ = true;
    }

    const bool allowPathIntegral = !rotatingInPlace_ &&
      std::abs(headingError) < 0.30 && std::abs(crossTrackError) < 0.08;
    const double pathFeedbackAngularSpeed = UpdatePathPid(
      pathError, crossTrackError, allowPathIntegral, controlYawRate) *
      fusionHealth.speed_scale;
    geometry_msgs::msg::Twist command;
    bool brakingApproach = false;
    const double segmentProgress = pathProjection.valid ?
      CompletedRouteLengthBeforeCurrentSegment() +
      Clamp(pathLength - pathProjection.remaining, 0.0, pathLength) :
      std::numeric_limits<double>::quiet_NaN();
    if (rotatingInPlace_)
    {
      if (SupervisePathProgress(segmentProgress, progressSupervisionAllowed))
        return;
      if (SuperviseTurnProgress(
          0, std::abs(headingError),
          turnCruiseSpeed_ * fusionHealth.speed_scale,
          progressSupervisionAllowed))
      {
        return;
      }
      if (!turnAnchorValid_)
      {
        turnAnchorX_ = currentX_;
        turnAnchorY_ = currentY_;
        turnAnchorValid_ = true;
      }
      if (std::abs(headingError) <= rotateInPlaceExitThreshold_)
        turnSettling_ = true;
      if (turnSettling_)
      {
        PublishMotionCommand(command);
        const double turnActivity = std::max(
          std::abs(controlYawRate), std::abs(lastMotionCommand_.angular.z));
        if (turnActivity > turnSettleYawRate_)
        {
          turnSettleTimerInitialized_ = false;
          SetState("ROTATING_TO_PATH");
          return;
        }
        const auto settleTime = std::chrono::steady_clock::now();
        if (!turnSettleTimerInitialized_)
        {
          turnSettleStarted_ = settleTime;
          turnSettleTimerInitialized_ = true;
        }
        const double settledSeconds =
          std::chrono::duration<double>(settleTime - turnSettleStarted_).count();
        if (!visual_navigation::TurnHasSettled(
            turnActivity, settledSeconds,
            turnSettleYawRate_, turnSettleDwell_))
        {
          SetState("ROTATING_TO_PATH");
          return;
        }
        if (std::abs(headingError) <= rotateInPlaceExitThreshold_)
        {
          ResetPathPid();
          turnProgressSupervisor_.Reset();
          pathAlignmentCompleted_ = true;
          SetState("PATH_ALIGNED");
          return;
        }
        turnSettling_ = false;
        turnSettleTimerInitialized_ = false;
        SetState("ROTATING_TO_PATH");
        return;
      }

      turnSettleTimerInitialized_ = false;
      const double scaledMaxAngularSpeed = turnCruiseSpeed_ * fusionHealth.speed_scale;
      const double requestedAngularSpeed = Clamp(
        angularGain_ * headingError - turnYawRateDamping_ * controlYawRate,
        -scaledMaxAngularSpeed, scaledMaxAngularSpeed);
      command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
        requestedAngularSpeed, headingError,
        minPrecisionTurnSpeed_ * fusionHealth.speed_scale, scaledMaxAngularSpeed);
      command.linear.x = visual_navigation::TurnPositionHoldSpeed(
        currentX_, currentY_, currentYaw_, turnAnchorX_, turnAnchorY_,
        turnPositionHoldGain_, turnPositionHoldDeadband_,
        turnPositionHoldMaxSpeed_) * fusionHealth.speed_scale;
    }
    else
    {
      turnProgressSupervisor_.Reset();
      const double headingScale = std::max(0.0, std::cos(pathError));
      double linearSpeed = visual_navigation::CrossTrackSpeedLimit(
        requestedSpeed, std::abs(crossTrackError),
        crossTrackSlowdownStart_, crossTrackSlowdownFull_, crossTrackMinimumSpeed_);
      linearSpeed = visual_navigation::CurvatureSpeedLimit(
        linearSpeed, pathCurvature, maxLateralAcceleration_);
      const double approachDistance =
        visual_navigation::EndpointApproachDistance(distance, pathProjection);
      if (waypointRequiresStop)
      {
        linearSpeed = visual_navigation::BrakingSpeedLimit(
          linearSpeed, approachDistance, effectiveBrakingDeceleration_,
          brakingControlDelay_, brakingSafetyMargin_);
        const double feedbackSpeed = inputCache_.odometry_velocity_valid() ?
          std::abs(inputCache_.odometry_velocity()) : std::abs(lastMotionCommand_.linear.x);
        linearSpeed = visual_navigation::BrakingFeedbackSpeedLimit(
          linearSpeed, feedbackSpeed, approachDistance,
          effectiveBrakingDeceleration_, brakingControlDelay_,
          brakingSafetyMargin_, brakingDistanceFeedbackGain_);
      }
      command.linear.x = linearSpeed * headingScale * fusionHealth.speed_scale;
      // Feed the planned curvature forward before the heading error develops.
      // The feedback PID then only removes odometry, slip and timing error.
      const double feedforwardAngularSpeed =
        visual_navigation::CurvatureFeedforwardAngularSpeed(
        command.linear.x, signedPathCurvature, pathCurvatureFeedforwardGain_);
      const double scaledMaxPathAngularSpeed =
        maxPathAngularSpeed_ * fusionHealth.speed_scale;
      const double lateralAngularLimit =
        visual_navigation::LateralAccelerationAngularLimit(
        scaledMaxPathAngularSpeed, command.linear.x, maxLateralAcceleration_);
      command.angular.z = visual_navigation::CompensatePathTurnDeadband(
        feedforwardAngularSpeed + pathFeedbackAngularSpeed, pathError, controlYawRate,
        pathAngularActivationError_, pathYawResponseThreshold_,
        minPathAngularSpeed_ * fusionHealth.speed_scale,
        lateralAngularLimit);

      const double measuredSpeed = inputCache_.odometry_velocity_valid() ?
        std::abs(inputCache_.odometry_velocity()) : std::abs(lastMotionCommand_.linear.x);
      const double stoppingDistance = visual_navigation::BrakingDistance(
        measuredSpeed, effectiveBrakingDeceleration_,
        brakingControlDelay_, brakingSafetyMargin_);
      brakingApproach = waypointRequiresStop &&
        approachDistance <= stoppingDistance && command.linear.x > 0.0;
    }

    if (!rotatingInPlace_ && SupervisePathProgress(
        segmentProgress, !brakingApproach && progressSupervisionAllowed))
    {
      return;
    }
    PublishMotionCommand(command);
    if (rotatingInPlace_)
      SetState("ROTATING_TO_PATH");
    else
      SetState(brakingApproach ? "BRAKING_APPROACH" : "FOLLOWING");
  }

  void CompleteNavigation()
  {
    navigationActive_ = false;
    finalPositionCaptured_ = false;
    finalPositionRecoveryActive_ = false;
    pathSegmentInitialized_ = false;
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    ResetPathPid();
    PublishStop();
    SetState("GOAL_REACHED");
  }

  void BeginPathSegment()
  {
    if (resumePathFromCurrentPose_ || currentWaypointIndex_ == 0)
    {
      pathSegmentStartX_ = currentX_;
      pathSegmentStartY_ = currentY_;
    }
    else
    {
      pathSegmentStartX_ = waypoints_[currentWaypointIndex_ - 1].x;
      pathSegmentStartY_ = waypoints_[currentWaypointIndex_ - 1].y;
    }
    resumePathFromCurrentPose_ = false;
    pathSegmentInitialized_ = true;
    pathAlignmentCompleted_ = false;
    waypointBrakeCompleted_ = false;
    finalPositionRecoveryActive_ = false;
    ResetPathPid();
  }

  bool SupervisePathProgress(
    double measuredProgress, bool eligible)
  {
    const auto decision = pathProgressSupervisor_.Evaluate(
      currentWaypointIndex_, measuredProgress, currentX_, currentY_, eligible,
      std::chrono::steady_clock::now());
    if (decision.action == visual_navigation::PathProgressAction::RECOVER)
    {
      RCLCPP_WARN(
        get_logger(),
        "No path progress; resetting control for waypoint %zu "
        "(recovery %d)",
        currentWaypointIndex_, decision.recovery_attempt);
      PublishStop();
      ResetPathPid();
      SetState("RECOVERING_NO_PATH_PROGRESS");
      return true;
    }
    if (decision.action == visual_navigation::PathProgressAction::FAULT)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Navigation stopped: waypoint %zu made no path progress after %d recoveries",
        currentWaypointIndex_, decision.recovery_attempt);
      navigationActive_ = false;
      PublishStop();
      ResetPathPid();
      SetState("FAULT_NO_PATH_PROGRESS");
      return true;
    }
    return false;
  }

  bool SuperviseTurnProgress(
    std::size_t phaseOffset, double absoluteYawError,
    double expectedYawRate, bool eligible)
  {
    const std::size_t phaseId = currentWaypointIndex_ * 3 + phaseOffset;
    const auto decision = turnProgressSupervisor_.Evaluate(
      phaseId, absoluteYawError, expectedYawRate, eligible,
      std::chrono::steady_clock::now());
    if (decision.action == visual_navigation::TurnProgressAction::RECOVER)
    {
      RCLCPP_WARN(
        get_logger(),
        "No turn progress at waypoint %zu (recovery %d)",
        currentWaypointIndex_, decision.recovery_attempt);
      PublishStop();
      ResetPathPid();
      SetState("RECOVERING_NO_TURN_PROGRESS");
      return true;
    }
    if (decision.action == visual_navigation::TurnProgressAction::FAULT)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Navigation stopped: waypoint %zu made no turn progress after %d recoveries",
        currentWaypointIndex_, decision.recovery_attempt);
      navigationActive_ = false;
      PublishStop();
      ResetPathPid();
      SetState("FAULT_NO_TURN_PROGRESS");
      return true;
    }
    return false;
  }

  double CompletedRouteLengthBeforeCurrentSegment() const
  {
    double completedLength = 0.0;
    const std::size_t end = std::min(currentWaypointIndex_, waypoints_.size());
    for (std::size_t index = 1; index < end; ++index)
    {
      completedLength += std::hypot(
        waypoints_[index].x - waypoints_[index - 1].x,
        waypoints_[index].y - waypoints_[index - 1].y);
    }
    return completedLength;
  }

  void BeginFinalPositionRecovery()
  {
    finalPositionCaptured_ = false;
    finalPositionRecoveryActive_ = true;
    turnProgressSupervisor_.Reset();
    waypointBrakeCompleted_ = false;
    ResetPathPid();
    waypointRecoveryActive_ = true;
    PublishStop();
    SetState("RECOVERING_FINAL_POSITION");
  }

  bool WaypointRequiresStop(double currentPathHeading, bool finalWaypoint) const
  {
    if (finalWaypoint)
      return true;
    const Waypoint & currentTarget = waypoints_[currentWaypointIndex_];
    const Waypoint & nextTarget = waypoints_[currentWaypointIndex_ + 1];
    const double nextDeltaX = nextTarget.x - currentTarget.x;
    const double nextDeltaY = nextTarget.y - currentTarget.y;
    if (std::hypot(nextDeltaX, nextDeltaY) <= 1.0e-9)
      return true;
    const double nextHeading = std::atan2(nextDeltaY, nextDeltaX);
    return std::abs(NormalizeAngle(nextHeading - currentPathHeading)) >=
      preTurnStopHeadingThreshold_;
  }

  void BeginWaypointBraking()
  {
    ResetPathPid();
    waypointBraking_ = true;
    waypointBrakeTimedOut_ = false;
    waypointStopStarted_ = std::chrono::steady_clock::now();
    waypointStopTimerInitialized_ = true;
    waypointSpeedSettleTimerInitialized_ = false;
  }

  void UpdateObservedLinearSpeed()
  {
    const auto sampleTime = std::chrono::steady_clock::now();
    if (!positionSamples_.empty())
    {
      const double gap =
        std::chrono::duration<double>(sampleTime - positionSamples_.back().time).count();
      if (gap > std::max(0.5, 2.0 * stopMotionWindow_))
        positionSamples_.clear();
    }
    positionSamples_.push_back({sampleTime, currentX_, currentY_});
    while (positionSamples_.size() >= 2)
    {
      const double secondAge = std::chrono::duration<double>(
        sampleTime - positionSamples_[1].time).count();
      if (secondAge < stopMotionWindow_)
        break;
      positionSamples_.pop_front();
    }
    const double span = std::chrono::duration<double>(
      sampleTime - positionSamples_.front().time).count();
    observedLinearVelocityValid_ = span >= 0.5 * stopMotionWindow_;
    if (observedLinearVelocityValid_)
    {
      observedLinearVelocity_ = std::hypot(
        currentX_ - positionSamples_.front().x,
        currentY_ - positionSamples_.front().y) / span;
      observedLinearVelocityValid_ = std::isfinite(observedLinearVelocity_);
    }
  }

  bool WaypointBrakeHasCompleted()
  {
    const auto currentTime = std::chrono::steady_clock::now();
    if (!waypointStopTimerInitialized_)
    {
      waypointStopStarted_ = currentTime;
      waypointStopTimerInitialized_ = true;
    }
    const double totalStopSeconds =
      std::chrono::duration<double>(currentTime - waypointStopStarted_).count();
    const double absoluteSpeed = observedLinearVelocityValid_ ?
      observedLinearVelocity_ : std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(absoluteSpeed) && absoluteSpeed <= preTurnStopSpeed_)
    {
      if (!waypointSpeedSettleTimerInitialized_)
      {
        waypointSpeedSettleStarted_ = currentTime;
        waypointSpeedSettleTimerInitialized_ = true;
      }
    }
    else
    {
      waypointSpeedSettleTimerInitialized_ = false;
    }
    const double settledSeconds = waypointSpeedSettleTimerInitialized_ ?
      std::chrono::duration<double>(currentTime - waypointSpeedSettleStarted_).count() : 0.0;
    const bool complete = visual_navigation::WaypointStopSatisfied(
      absoluteSpeed, settledSeconds, totalStopSeconds, preTurnStopSpeed_,
      preTurnStopDwell_, preTurnMinimumStopTime_, preTurnBrakeTimeout_);
    if (!complete && totalStopSeconds >= preTurnBrakeTimeout_)
    {
      waypointBrakeTimedOut_ = true;
      if (std::isfinite(absoluteSpeed))
      {
        RCLCPP_ERROR(
          get_logger(), "Waypoint brake timed out after %.3fs (observed speed=%.3fm/s)",
          totalStopSeconds, absoluteSpeed);
      }
      else
      {
        RCLCPP_ERROR(
          get_logger(), "Waypoint brake timed out after %.3fs (observed speed invalid)",
          totalStopSeconds);
      }
    }
    return complete;
  }

  void ResetPathPid()
  {
    pathPidIntegral_ = 0.0;
    previousPathError_ = 0.0;
    pathPidInitialized_ = false;
    rotatingInPlace_ = false;
    waypointRecoveryActive_ = false;
    turnAnchorValid_ = false;
    turnSettling_ = false;
    turnSettleTimerInitialized_ = false;
    waypointBraking_ = false;
    waypointStopTimerInitialized_ = false;
    waypointSpeedSettleTimerInitialized_ = false;
  }

  double UpdatePathPid(
    double error, double crossTrackError, bool allowIntegral, double yawRate)
  {
    const auto currentTime = std::chrono::steady_clock::now();
    double derivative = 0.0;
    if (pathPidInitialized_)
    {
      const double dt = std::chrono::duration<double>(currentTime - lastPathPidTime_).count();
      if (dt > 0.0 && dt <= 0.2)
      {
        if (allowIntegral)
        {
          pathPidIntegral_ = Clamp(
            pathPidIntegral_ - crossTrackError * dt,
            -pathPidIntegralLimit_, pathPidIntegralLimit_);
        }
        else
        {
          pathPidIntegral_ *= std::max(0.0, 1.0 - 4.0 * dt);
        }
        derivative = (error - previousPathError_) / dt;
      }
    }

    previousPathError_ = error;
    lastPathPidTime_ = currentTime;
    pathPidInitialized_ = true;
    return Clamp(
      pathPidKp_ * error + pathPidKi_ * pathPidIntegral_ +
      pathPidKd_ * derivative - pathYawRateDamping_ * yawRate,
      -maxPathAngularSpeed_, maxPathAngularSpeed_);
  }

  void PublishStop()
  {
    geometry_msgs::msg::Twist command;
    cmdVelPublisher_->publish(command);
    lastMotionCommand_ = command;
    lastMotionCommandTime_ = std::chrono::steady_clock::now();
    motionCommandInitialized_ = true;
  }

  void PublishMotionCommand(const geometry_msgs::msg::Twist &desired)
  {
    const auto currentTime = std::chrono::steady_clock::now();
    double dt = 1.0 / controlFrequency_;
    if (motionCommandInitialized_)
    {
      const double measuredDt =
        std::chrono::duration<double>(currentTime - lastMotionCommandTime_).count();
      if (measuredDt > 0.0 && measuredDt <= 0.2)
        dt = measuredDt;
    }

    geometry_msgs::msg::Twist limited = desired;
    limited.linear.x = visual_navigation::LimitRate(
      desired.linear.x, lastMotionCommand_.linear.x,
      maxLinearAcceleration_, maxLinearDeceleration_, dt);
    limited.angular.z = visual_navigation::LimitRate(
      desired.angular.z, lastMotionCommand_.angular.z,
      maxAngularAcceleration_, maxAngularDeceleration_, dt);
    cmdVelPublisher_->publish(limited);
    lastMotionCommand_ = limited;
    lastMotionCommandTime_ = currentTime;
    motionCommandInitialized_ = true;
  }

  void PublishCurrentWaypoint()
  {
    std_msgs::msg::Int32 message;
    message.data = static_cast<int32_t>(currentWaypointIndex_);
    currentWaypointPublisher_->publish(message);
  }

  bool MotionHeld() const
  {
    return !motionHolds_.empty();
  }

  std::vector<std::string> ActiveHoldSources() const
  {
    std::vector<std::string> sources;
    sources.reserve(motionHolds_.size());
    for (const auto &entry : motionHolds_)
      sources.push_back(entry.first);
    return sources;
  }

  void PublishMotionHoldState()
  {
    mission_control_interfaces::msg::MotionHoldState message;
    message.held = MotionHeld();
    message.active_sources.reserve(motionHolds_.size());
    message.reasons.reserve(motionHolds_.size());
    for (const auto &entry : motionHolds_)
    {
      message.active_sources.push_back(entry.first);
      message.reasons.push_back(entry.second);
    }
    motionHoldStatePublisher_->publish(message);
  }

  void SetState(const std::string &state)
  {
    if (state == state_)
      return;
    state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    statusPublisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Navigation state: %s", state.c_str());
  }

  std::string routeFile_;
  std::string routeFrame_;
  std::string odomTopic_;
  std::string imuTopic_;
  std::string fusionStatusTopic_;
  std::string actuatorHealthTopic_;
  std::string trackingStateTopic_;
  std::string cmdVelTopic_;
  std::string pathTopic_;
  std::string routeInputTopic_;
  std::string startTopic_;
  std::string statusTopic_;
  std::string currentWaypointTopic_;
  std::string motionHoldStateTopic_;
  std::string routeAckTopic_;
  double controlFrequency_{30.0};
  double trackingPointOffsetX_{0.0};
  double trackingPointOffsetY_{0.0};
  double defaultSpeed_{0.50};
  double maxLinearSpeed_{0.50};
  double maxAngularSpeed_{0.95};
  double maxPathAngularSpeed_{0.45};
  double minPathAngularSpeed_{0.14};
  double pathAngularActivationError_{0.025};
  double pathYawResponseThreshold_{0.04};
  double maxLinearAcceleration_{0.40};
  double maxLinearDeceleration_{0.80};
  double effectiveBrakingDeceleration_{0.35};
  double brakingControlDelay_{0.20};
  double brakingSafetyMargin_{0.015};
  double brakingDistanceFeedbackGain_{1.0};
  double maxAngularAcceleration_{1.80};
  double maxAngularDeceleration_{3.00};
  double linearGain_{0.8};
  double angularGain_{2.8};
  double pathPidKp_{2.4};
  double pathPidKi_{0.15};
  double pathPidKd_{0.0};
  double pathYawRateDamping_{0.45};
  double turnYawRateDamping_{0.55};
  double turnCruiseSpeed_{0.90};
  double turnPositionHoldGain_{0.80};
  double turnPositionHoldDeadband_{0.015};
  double turnPositionHoldMaxSpeed_{0.08};
  double crossTrackGain_{1.5};
  double stanleySofteningSpeed_{0.25};
  double maxCrossTrackCorrection_{0.70};
  double crossTrackSlowdownStart_{0.015};
  double crossTrackSlowdownFull_{0.06};
  double crossTrackMinimumSpeed_{0.20};
  double maxLateralAcceleration_{0.22};
  double pathCurvatureFeedforwardGain_{1.0};
  double pathPidIntegralLimit_{0.20};
  double rotateInPlaceThreshold_{0.18};
  double rotateInPlaceReentryThreshold_{0.44};
  double rotateInPlaceExitThreshold_{0.035};
  double precisionTurnThreshold_{0.45};
  double turnSettleYawRate_{0.12};
  double turnSettleDwell_{0.30};
  double minPrecisionTurnSpeed_{0.25};
  double waypointTolerance_{0.04};
  double waypointPassLongitudinalTolerance_{0.01};
  double waypointPassLateralTolerance_{0.06};
  double waypointRecoverySpeed_{0.10};
  double waypointRecoveryHeadingTolerance_{0.12};
  double waypointRecoveryMaxAngularSpeed_{0.60};
  double preTurnStopHeadingThreshold_{0.18};
  double preTurnStopSpeed_{0.03};
  double preTurnStopDwell_{0.10};
  double stopMotionWindow_{0.20};
  double preTurnMinimumStopTime_{0.20};
  double preTurnBrakeTimeout_{0.60};
  double finalYawTolerance_{0.060};
  double finalYawMaxAngularSpeed_{0.90};
  double finalYawMinTurnSpeed_{0.70};
  double finalYawMinPrecisionSpeed_{0.22};
  double odomTimeout_{0.40};
  double imuTimeout_{0.15};
  double fusionStatusTimeout_{0.60};
  double actuatorHealthTimeout_{0.80};
  double stampFutureTolerance_{0.20};
  bool requireFusionStatus_{true};
  bool requireActuatorHealth_{false};
  bool requireTrackingState_{false};
  bool autostart_{false};
  std::vector<std::string> allowedFusionStates_;
  visual_navigation::NavigationInputCache inputCache_;
  visual_navigation::FusionHealthPolicy fusionHealthPolicy_;
  visual_navigation::NavigationSupervisor navigationSupervisor_;
  visual_navigation::PathProgressSupervisor pathProgressSupervisor_;
  visual_navigation::TurnProgressSupervisor turnProgressSupervisor_;

  std::vector<Waypoint> waypoints_;
  std::size_t currentWaypointIndex_{0};
  bool routeLoaded_{false};
  bool navigationActive_{false};
  bool resumePathFromCurrentPose_{false};
  bool motionHoldStartInitialized_{false};
  bool trackingReferenceInitialized_{false};
  double currentX_{0.0};
  double currentY_{0.0};
  double currentYaw_{0.0};
  double observedLinearVelocity_{0.0};
  double trackingReferenceYaw_{0.0};
  double imuYawRate_{0.0};
  double pathSegmentStartX_{0.0};
  double pathSegmentStartY_{0.0};
  double turnAnchorX_{0.0};
  double turnAnchorY_{0.0};
  double pathPidIntegral_{0.0};
  double previousPathError_{0.0};
  bool pathSegmentInitialized_{false};
  bool pathAlignmentCompleted_{false};
  bool pathPidInitialized_{false};
  bool rotatingInPlace_{false};
  bool waypointRecoveryActive_{false};
  bool turnAnchorValid_{false};
  bool turnSettling_{false};
  bool turnSettleTimerInitialized_{false};
  bool observedLinearVelocityValid_{false};
  bool waypointBraking_{false};
  bool waypointBrakeTimedOut_{false};
  bool waypointBrakeCompleted_{false};
  bool finalPositionRecoveryActive_{false};
  bool waypointStopTimerInitialized_{false};
  bool waypointSpeedSettleTimerInitialized_{false};
  bool finalPositionCaptured_{false};
  std::string state_;
  std::map<std::string, std::string> motionHolds_;
  rclcpp::Time lastOdomStamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastImuStamp_{0, 0, RCL_ROS_TIME};
  bool odomStampValid_{false};
  bool imuStampValid_{false};
  rclcpp::Time waitUntil_{0, 0, RCL_ROS_TIME};
  rclcpp::Time motionHoldStartedAt_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point lastPathPidTime_{};
  std::chrono::steady_clock::time_point lastMotionCommandTime_{};
  std::chrono::steady_clock::time_point turnSettleStarted_{};
  std::chrono::steady_clock::time_point waypointStopStarted_{};
  std::chrono::steady_clock::time_point waypointSpeedSettleStarted_{};
  std::deque<PositionSample> positionSamples_;
  geometry_msgs::msg::Twist lastMotionCommand_;
  bool motionCommandInitialized_{false};
  WaitAction waitAction_{WaitAction::NONE};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr currentWaypointPublisher_;
  rclcpp::Publisher<mission_control_interfaces::msg::MotionHoldState>::SharedPtr
    motionHoldStatePublisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr routeAckPublisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fusionStatusSubscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuatorHealthSubscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trackingStateSubscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr routeInputSubscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr startSubscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;
  rclcpp::Service<mission_control_interfaces::srv::SetMotionHold>::SharedPtr
    motionHoldService_;
  rclcpp::TimerBase::SharedPtr controlTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
