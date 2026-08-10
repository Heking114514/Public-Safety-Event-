#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visual_navigation/actuator_health_policy.hpp"
#include "visual_navigation/fusion_health_policy.hpp"
#include "visual_navigation/path_control.hpp"
#include "visual_navigation/rate_limiter.hpp"

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

std::vector<std::string> SplitCsv(const std::string &line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
    fields.push_back(Trim(field));
  return fields;
}

bool LooksLikeHeader(const std::vector<std::string> &fields)
{
  if (fields.empty())
    return false;
  std::string firstField = fields.front();
  std::transform(firstField.begin(), firstField.end(), firstField.begin(), ::tolower);
  return firstField == "x" || firstField == "x[m]" || firstField == "x_m";
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
    imuTopic_ = declare_parameter<std::string>("imu_topic", "/imu/filtered");
    trackingPointOffsetX_ = declare_parameter<double>("tracking_point_offset_x", 0.087);
    trackingPointOffsetY_ = declare_parameter<double>("tracking_point_offset_y", 0.040);
    if (!std::isfinite(trackingPointOffsetX_) || !std::isfinite(trackingPointOffsetY_))
      throw std::invalid_argument("tracking point offsets must be finite");
    fusionStatusTopic_ = declare_parameter<std::string>(
      "fusion_status_topic", "/odometry/fusion_status");
    actuatorHealthTopic_ = declare_parameter<std::string>(
      "actuator_health_topic", "/cup_car_serial/connected");
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

    controlFrequency_ = std::max(1.0, declare_parameter<double>("control_frequency", 30.0));
    defaultSpeed_ = std::max(0.0, declare_parameter<double>("default_speed", 0.20));
    maxLinearSpeed_ = std::max(0.0, declare_parameter<double>("max_linear_speed", 0.30));
    maxAngularSpeed_ = std::max(0.0, declare_parameter<double>("max_angular_speed", 0.80));
    maxPathAngularSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("max_path_angular_speed", 0.65)));
    maxLinearAcceleration_ = std::max(
      0.01, declare_parameter<double>("max_linear_acceleration", 0.40));
    maxLinearDeceleration_ = std::max(
      0.01, declare_parameter<double>("max_linear_deceleration", 0.80));
    maxAngularAcceleration_ = std::max(
      0.01, declare_parameter<double>("max_angular_acceleration", 2.50));
    maxAngularDeceleration_ = std::max(
      0.01, declare_parameter<double>("max_angular_deceleration", 4.00));
    linearGain_ = std::max(0.0, declare_parameter<double>("linear_gain", 0.8));
    angularGain_ = std::max(0.0, declare_parameter<double>("angular_gain", 2.0));
    pathPidKp_ = std::max(0.0, declare_parameter<double>("path_pid_kp", 2.4));
    pathPidKi_ = std::max(0.0, declare_parameter<double>("path_pid_ki", 0.35));
    pathPidKd_ = std::max(0.0, declare_parameter<double>("path_pid_kd", 0.0));
    pathYawRateDamping_ = std::max(
      0.0, declare_parameter<double>("path_yaw_rate_damping", 0.18));
    turnYawRateDamping_ = std::max(
      0.0, declare_parameter<double>("turn_yaw_rate_damping", 0.10));
    turnBrakeHorizon_ = std::max(
      0.0, declare_parameter<double>("turn_brake_horizon", 0.35));
    turnCruiseSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("turn_cruise_speed", 0.65)));
    crossTrackGain_ = std::max(
      0.0, declare_parameter<double>("cross_track_gain", 2.6));
    stanleySofteningSpeed_ = std::max(
      0.01, declare_parameter<double>("stanley_softening_speed", 0.25));
    maxCrossTrackCorrection_ = std::max(
      0.0, declare_parameter<double>("max_cross_track_correction", 0.70));
    crossTrackSlowdownStart_ = std::max(
      0.0, declare_parameter<double>("cross_track_slowdown_start", 0.03));
    crossTrackSlowdownFull_ = std::max(
      crossTrackSlowdownStart_ + 0.001,
      declare_parameter<double>("cross_track_slowdown_full", 0.06));
    crossTrackMinimumSpeed_ = std::max(
      0.0, declare_parameter<double>("cross_track_minimum_speed", 0.08));
    pathPidIntegralLimit_ = std::max(
      0.0, declare_parameter<double>("path_pid_integral_limit", 0.4));
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
      0.001, declare_parameter<double>("waypoint_tolerance", 0.08));
    finalYawTolerance_ = std::max(
      0.001, declare_parameter<double>("final_yaw_tolerance", 0.060));
    finalYawMaxAngularSpeed_ = std::min(
      maxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("final_yaw_max_angular_speed", 0.80)));
    finalYawMinTurnSpeed_ = std::min(
      finalYawMaxAngularSpeed_, std::max(
        0.0, declare_parameter<double>("final_yaw_min_turn_speed", 0.70)));
    finalYawMinPrecisionSpeed_ = std::min(
      finalYawMinTurnSpeed_, std::max(
        0.0, declare_parameter<double>("final_yaw_min_precision_speed", 0.32)));
    odomTimeout_ = std::max(0.01, declare_parameter<double>("odom_timeout", 0.40));
    imuTimeout_ = std::max(0.01, declare_parameter<double>("imu_timeout", 0.15));
    fusionStatusTimeout_ = std::max(
      0.01, declare_parameter<double>("fusion_status_timeout", 0.60));
    actuatorHealthTimeout_ = std::max(
      0.01, declare_parameter<double>("actuator_health_timeout", 0.80));
    transientLocalizationGrace_ = std::max(
      0.0, declare_parameter<double>("transient_localization_grace", 2.00));
    transientFaultSpeedScale_ = Clamp(
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
    abortOnTrackingLoss_ = declare_parameter<bool>("abort_on_tracking_loss", false);
    autostart_ = declare_parameter<bool>("autostart", false);

    cmdVelPublisher_ = create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic_, 10);
    pathPublisher_ = create_publisher<nav_msgs::msg::Path>(
      pathTopic_, rclcpp::QoS(1).transient_local().reliable());
    statusPublisher_ = create_publisher<std_msgs::msg::String>(
      statusTopic_, rclcpp::QoS(1).transient_local().reliable());
    currentWaypointPublisher_ = create_publisher<std_msgs::msg::Int32>(
      currentWaypointTopic_, rclcpp::QoS(1).transient_local().reliable());

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

    routeLoaded_ = LoadRoute(routeFile_);
    if (routeLoaded_)
    {
      PublishRoutePath();
      PublishCurrentWaypoint();
    }

    const auto period = std::chrono::duration<double>(1.0 / controlFrequency_);
    controlTimer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WaypointNavigator::RunControl, this));

    if (autostart_ && routeLoaded_)
    {
      navigationActive_ = true;
      SetState("WAITING_FOR_LOCALIZATION");
    }
    else
    {
      SetState(routeLoaded_ ? "IDLE" : "FAULT_ROUTE_NOT_LOADED");
    }

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
  struct Waypoint
  {
    double x{0.0};
    double y{0.0};
    double yaw{std::numeric_limits<double>::quiet_NaN()};
    double speed{0.0};
    double tolerance{0.0};
    double stopTime{0.0};
  };

  enum class WaitAction
  {
    NONE,
    ADVANCE,
    COMPLETE
  };

  bool LoadRoute(const std::string &routeFile)
  {
    if (routeFile.empty())
    {
      RCLCPP_ERROR(get_logger(), "Parameter 'route_file' is empty");
      return false;
    }

    std::ifstream input(routeFile);
    if (!input.is_open())
    {
      RCLCPP_ERROR(get_logger(), "Cannot open waypoint file: %s", routeFile.c_str());
      return false;
    }

    std::vector<Waypoint> loadedWaypoints;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line))
    {
      ++lineNumber;
      const std::size_t commentPosition = line.find('#');
      if (commentPosition != std::string::npos)
        line = line.substr(0, commentPosition);
      line = Trim(line);
      if (line.empty())
        continue;

      const std::vector<std::string> fields = SplitCsv(line);
      if (LooksLikeHeader(fields))
        continue;
      if (fields.size() < 2)
      {
        RCLCPP_ERROR(
          get_logger(), "Waypoint line %zu requires at least x,y", lineNumber);
        return false;
      }

      try
      {
        Waypoint waypoint;
        waypoint.x = std::stod(fields[0]);
        waypoint.y = std::stod(fields[1]);
        if (fields.size() > 2 && !fields[2].empty())
          waypoint.yaw = std::stod(fields[2]);
        waypoint.speed = fields.size() > 3 && !fields[3].empty()
          ? std::stod(fields[3]) : defaultSpeed_;
        waypoint.tolerance = fields.size() > 4 && !fields[4].empty()
          ? std::stod(fields[4]) : waypointTolerance_;
        waypoint.stopTime = fields.size() > 5 && !fields[5].empty()
          ? std::stod(fields[5]) : 0.0;

        if (waypoint.speed < 0.0 || waypoint.tolerance <= 0.0 || waypoint.stopTime < 0.0)
          throw std::runtime_error("speed, tolerance or stop_time is outside its valid range");
        loadedWaypoints.push_back(waypoint);
      }
      catch (const std::exception &exception)
      {
        RCLCPP_ERROR(
          get_logger(), "Invalid waypoint at line %zu: %s", lineNumber, exception.what());
        return false;
      }
    }

    if (loadedWaypoints.empty())
    {
      RCLCPP_ERROR(get_logger(), "Waypoint file contains no valid waypoints");
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

  void HandleOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    currentOdomFrameValid_ = message->header.frame_id.empty() ||
      message->header.frame_id == routeFrame_;
    currentOdomPoseValid_ =
      std::isfinite(message->pose.pose.position.x) &&
      std::isfinite(message->pose.pose.position.y) &&
      QuaternionIsValid(message->pose.pose.orientation);
    lastOdomArrival_ = now();
    hasOdometry_ = true;

    if (!currentOdomFrameValid_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejecting odometry frame '%s'; route frame is '%s' and TF conversion is disabled",
        message->header.frame_id.c_str(), routeFrame_.c_str());
      return;
    }
    if (!currentOdomPoseValid_)
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
  }

  void HandleImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    if (!std::isfinite(message->angular_velocity.z))
      return;
    imuYawRate_ = message->angular_velocity.z;
    lastImuArrival_ = now();
    hasImu_ = true;
  }

  void HandleTrackingState(const std_msgs::msg::Int32::SharedPtr message)
  {
    trackingState_ = message->data;
    hasTrackingState_ = true;
  }

  void HandleFusionStatus(const std_msgs::msg::String::SharedPtr message)
  {
    fusionStatus_ = Trim(message->data);
    lastFusionStatusArrival_ = now();
    hasFusionStatus_ = true;
  }

  void HandleActuatorHealth(const std_msgs::msg::Bool::SharedPtr message)
  {
    actuatorConnected_ = message->data;
    lastActuatorHealthArrival_ = now();
    hasActuatorHealth_ = true;
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

    std::vector<Waypoint> loadedWaypoints;
    loadedWaypoints.reserve(message->poses.size());
    for (std::size_t index = 0; index < message->poses.size(); ++index)
    {
      const auto &pose = message->poses[index].pose;
      if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !QuaternionIsValid(pose.orientation))
      {
        RCLCPP_ERROR(
          get_logger(), "Ignoring dynamic route: waypoint %zu has an invalid pose", index);
        return;
      }

      Waypoint waypoint;
      waypoint.x = pose.position.x;
      waypoint.y = pose.position.y;
      waypoint.yaw = YawFromValidQuaternion(pose.orientation);
      waypoint.speed = defaultSpeed_;
      waypoint.tolerance = waypointTolerance_;
      waypoint.stopTime = 0.0;
      loadedWaypoints.push_back(waypoint);
    }

    // Stop the old route before atomically replacing it with the new one.
    navigationActive_ = false;
    currentWaypointIndex_ = 0;
    finalPositionCaptured_ = false;
    localizationWasValid_ = false;
    waitAction_ = WaitAction::NONE;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    PublishStop();

    waypoints_ = std::move(loadedWaypoints);
    routeLoaded_ = true;
    navigationActive_ = true;
    PublishRoutePath();
    PublishCurrentWaypoint();
    SetState("WAITING_FOR_LOCALIZATION");
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

    const auto fusionHealth = FusionHealth();
    if (!visual_navigation::LocalizationCanStart(
        OdometryIsValid(), fusionHealth, TrackingStateIsValid()))
    {
      navigationActive_ = false;
      PublishStop();
      const std::string failureState = LocalizationFailureState(fusionHealth);
      SetState(failureState);
      result = "Navigation start rejected: " + failureState;
      return false;
    }
    if (!ActuatorHealthIsValid())
    {
      navigationActive_ = false;
      PublishStop();
      const std::string failureState = ActuatorFailureState();
      SetState(failureState);
      result = "Navigation start rejected: " + failureState;
      return false;
    }

    if (currentWaypointIndex_ >= waypoints_.size())
      currentWaypointIndex_ = 0;
    waitAction_ = WaitAction::NONE;
    finalPositionCaptured_ = false;
    navigationActive_ = true;
    localizationWasValid_ = false;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    SetState("WAITING_FOR_LOCALIZATION");
    result = "Waypoint navigation started";
    return true;
  }

  void HandleStop(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    navigationActive_ = false;
    waitAction_ = WaitAction::NONE;
    finalPositionCaptured_ = false;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    PublishStop();
    SetState("IDLE");
    response->success = true;
    response->message = "Waypoint navigation stopped";
  }

  void HandleReset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    navigationActive_ = false;
    currentWaypointIndex_ = 0;
    finalPositionCaptured_ = false;
    localizationWasValid_ = false;
    waitAction_ = WaitAction::NONE;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    PublishStop();
    PublishCurrentWaypoint();
    SetState("IDLE");
    response->success = true;
    response->message = "Waypoint navigation reset";
  }

  bool OdometryIsValid() const
  {
    if (!hasOdometry_)
      return false;
    return currentOdomFrameValid_ && currentOdomPoseValid_ &&
      (now() - lastOdomArrival_).seconds() <= odomTimeout_;
  }

  bool ImuYawRateIsFresh() const
  {
    return hasImu_ && (now() - lastImuArrival_).seconds() <= imuTimeout_;
  }

  double ControlYawRate() const
  {
    return ImuYawRateIsFresh() ? imuYawRate_ : 0.0;
  }

  bool TrackingStateIsValid() const
  {
    return !requireTrackingState_ ||
      (hasTrackingState_ && (trackingState_ == 2 || trackingState_ == 5));
  }

  bool FusionStatusIsFresh() const
  {
    return hasFusionStatus_ &&
      (now() - lastFusionStatusArrival_).seconds() <= fusionStatusTimeout_;
  }

  visual_navigation::FusionHealthDecision FusionHealth() const
  {
    if (!requireFusionStatus_)
      return visual_navigation::FusionHealthDecision{true, false, 1.0};
    if (!FusionStatusIsFresh())
      return visual_navigation::FusionHealthDecision{};
    return fusionHealthPolicy_.Evaluate(fusionStatus_);
  }

  bool ActuatorHealthIsFresh() const
  {
    return hasActuatorHealth_ &&
      (now() - lastActuatorHealthArrival_).seconds() <= actuatorHealthTimeout_;
  }

  bool ActuatorHealthIsValid() const
  {
    return visual_navigation::ActuatorHealthIsValid(
      requireActuatorHealth_, hasActuatorHealth_, ActuatorHealthIsFresh(),
      actuatorConnected_);
  }

  std::string ActuatorFailureState() const
  {
    if (!hasActuatorHealth_ || !ActuatorHealthIsFresh())
      return "FAULT_ACTUATOR_STALE";
    return "FAULT_ACTUATOR_DISCONNECTED";
  }

  std::string LocalizationFailureState(
    const visual_navigation::FusionHealthDecision &health) const
  {
    if (!OdometryIsValid())
    {
      if (hasOdometry_ && !currentOdomFrameValid_)
        return "FAULT_ODOMETRY_FRAME";
      if (hasOdometry_ && !currentOdomPoseValid_)
        return "FAULT_ODOMETRY_INVALID";
      return "WAITING_FOR_ODOMETRY";
    }
    if (requireFusionStatus_ && !FusionStatusIsFresh())
      return "FAULT_FUSION_STATUS_STALE";
    if (health.fault)
      return "FAULT_FUSION_STATUS";
    if (requireFusionStatus_ && !health.allowed)
      return "WAITING_FOR_ALLOWED_FUSION_STATUS";
    if (!TrackingStateIsValid())
      return "FAULT_TRACKING_LOST";
    return "WAITING_FOR_LOCALIZATION";
  }

  void RunControl()
  {
    if (!navigationActive_)
    {
      PublishStop();
      return;
    }

    if (visual_navigation::ShouldLatchActuatorLoss(
        navigationActive_, ActuatorHealthIsValid()))
    {
      navigationActive_ = false;
      PublishStop();
      ResetPathPid();
      SetState(ActuatorFailureState());
      return;
    }

    auto fusionHealth = FusionHealth();
    const bool localizationValid =
      OdometryIsValid() && fusionHealth.allowed && TrackingStateIsValid();
    const auto controlTime = std::chrono::steady_clock::now();
    if (localizationValid)
    {
      lastValidLocalizationTime_ = controlTime;
      validLocalizationTimeInitialized_ = true;
    }

    const double secondsSinceValid = validLocalizationTimeInitialized_ ?
      std::chrono::duration<double>(controlTime - lastValidLocalizationTime_).count() :
      std::numeric_limits<double>::infinity();
    const bool hardFault = fusionStatus_ == "FAULT_STALLED";
    const bool bridgeTransientLoss = !localizationValid &&
      visual_navigation::CanBridgeTransientLocalizationLoss(
        localizationWasValid_, hardFault, secondsSinceValid, transientLocalizationGrace_);

    if (bridgeTransientLoss)
    {
      fusionHealth.allowed = true;
      fusionHealth.fault = false;
      fusionHealth.speed_scale = transientFaultSpeedScale_;
      SetState("DEGRADED_TRANSIENT_LOCALIZATION");
    }
    else if (!localizationValid)
    {
      PublishStop();
      ResetPathPid();
      if (hardFault)
      {
        navigationActive_ = false;
        SetState(LocalizationFailureState(fusionHealth));
      }
      else if (!TrackingStateIsValid() && localizationWasValid_ && abortOnTrackingLoss_)
      {
        navigationActive_ = false;
        SetState("FAULT_TRACKING_LOST");
      }
      else
      {
        SetState(LocalizationFailureState(fusionHealth));
      }
      return;
    }

    localizationWasValid_ = true;

    if (waitAction_ != WaitAction::NONE)
    {
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
    finalPositionCaptured_ = visual_navigation::FinalPositionCaptured(
      finalPositionCaptured_, finalWaypoint, distance, target.tolerance);

    if (finalPositionCaptured_)
    {
      if (std::isfinite(target.yaw))
      {
        const double finalYawError = NormalizeAngle(target.yaw - currentYaw_);
        if (std::abs(finalYawError) > finalYawTolerance_ ||
          std::abs(controlYawRate) > turnSettleYawRate_)
        {
          geometry_msgs::msg::Twist command;
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
          PublishMotionCommand(command);
          SetState("ALIGNING_FINAL_YAW");
          return;
        }
      }

      if (target.stopTime > 0.0)
      {
        waitUntil_ = now() + rclcpp::Duration::from_seconds(target.stopTime);
        waitAction_ = WaitAction::COMPLETE;
        PublishStop();
        SetState("WAITING_AT_WAYPOINT");
        return;
      }

      CompleteNavigation();
      return;
    }

    if (distance <= target.tolerance)
    {

      ++currentWaypointIndex_;
      pathSegmentInitialized_ = false;
      ResetPathPid();
      PublishCurrentWaypoint();
      return;
    }

    const double pathDeltaX = target.x - pathSegmentStartX_;
    const double pathDeltaY = target.y - pathSegmentStartY_;
    const double pathHeading = std::atan2(pathDeltaY, pathDeltaX);
    const double headingError = NormalizeAngle(pathHeading - currentYaw_);
    // Positive cross-track error means the car is to the left of the path.
    const double crossTrackError =
      std::cos(pathHeading) * (currentY_ - pathSegmentStartY_) -
      std::sin(pathHeading) * (currentX_ - pathSegmentStartX_);
    const double requestedSpeed = std::min(target.speed, maxLinearSpeed_);
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
      turnDirection_ = std::copysign(1.0, headingError);
    }
    else if (!rotatingInPlace_ && !pathAlignmentCompleted_)
    {
      pathAlignmentCompleted_ = true;
    }

    const bool allowPathIntegral = !rotatingInPlace_ &&
      std::abs(headingError) < 0.30 && std::abs(crossTrackError) < 0.08;
    double angularSpeed = UpdatePathPid(
      pathError, crossTrackError, allowPathIntegral, controlYawRate) *
      fusionHealth.speed_scale;
    if (rotatingInPlace_)
    {
      if (turnBraking_)
      {
        PublishStop();
        if (std::abs(controlYawRate) > turnSettleYawRate_)
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
            std::abs(controlYawRate), settledSeconds,
            turnSettleYawRate_, turnSettleDwell_))
        {
          SetState("ROTATING_TO_PATH");
          return;
        }
        if (std::abs(headingError) <= rotateInPlaceExitThreshold_)
        {
          ResetPathPid();
          pathAlignmentCompleted_ = true;
          SetState("PATH_ALIGNED");
          return;
        }
        turnBraking_ = false;
        turnSettleTimerInitialized_ = false;
        turnDirection_ = std::copysign(1.0, headingError);
      }

      const double directedError = turnDirection_ * headingError;
      if (visual_navigation::ShouldBrakeTurn(
          directedError, std::abs(controlYawRate),
          rotateInPlaceExitThreshold_, turnBrakeHorizon_))
      {
        turnBraking_ = true;
        turnSettleTimerInitialized_ = false;
        PublishStop();
        SetState("ROTATING_TO_PATH");
        return;
      }

      const double turnSpeed = visual_navigation::TurnSpeedForError(
        directedError, rotateInPlaceExitThreshold_, precisionTurnThreshold_,
        minPrecisionTurnSpeed_, turnCruiseSpeed_);
      angularSpeed = turnDirection_ * turnSpeed * fusionHealth.speed_scale;
    }

    geometry_msgs::msg::Twist command;
    command.angular.z = angularSpeed;

    if (!rotatingInPlace_)
    {
      const double headingScale = std::max(0.0, std::cos(pathError));
      double linearSpeed = visual_navigation::CrossTrackSpeedLimit(
        requestedSpeed, std::abs(crossTrackError),
        crossTrackSlowdownStart_, crossTrackSlowdownFull_, crossTrackMinimumSpeed_);
      linearSpeed = visual_navigation::WaypointApproachSpeedLimit(
        linearSpeed, requestedSpeed, linearGain_, distance);
      command.linear.x = linearSpeed * headingScale * fusionHealth.speed_scale;
    }

    PublishMotionCommand(command);
    SetState(rotatingInPlace_ ? "ROTATING_TO_PATH" : "FOLLOWING");
  }

  void CompleteNavigation()
  {
    navigationActive_ = false;
    finalPositionCaptured_ = false;
    pathSegmentInitialized_ = false;
    ResetPathPid();
    PublishStop();
    SetState("GOAL_REACHED");
  }

  void BeginPathSegment()
  {
    if (currentWaypointIndex_ > 0)
    {
      pathSegmentStartX_ = waypoints_[currentWaypointIndex_ - 1].x;
      pathSegmentStartY_ = waypoints_[currentWaypointIndex_ - 1].y;
    }
    else
    {
      pathSegmentStartX_ = currentX_;
      pathSegmentStartY_ = currentY_;
    }
    pathSegmentInitialized_ = true;
    pathAlignmentCompleted_ = false;
    ResetPathPid();
  }

  void ResetPathPid()
  {
    pathPidIntegral_ = 0.0;
    previousPathError_ = 0.0;
    pathPidInitialized_ = false;
    rotatingInPlace_ = false;
    turnBraking_ = false;
    turnDirection_ = 0.0;
    turnSettleTimerInitialized_ = false;
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
  double controlFrequency_{30.0};
  double trackingPointOffsetX_{0.087};
  double trackingPointOffsetY_{0.040};
  double defaultSpeed_{0.20};
  double maxLinearSpeed_{0.30};
  double maxAngularSpeed_{0.80};
  double maxPathAngularSpeed_{0.65};
  double maxLinearAcceleration_{0.40};
  double maxLinearDeceleration_{0.80};
  double maxAngularAcceleration_{2.50};
  double maxAngularDeceleration_{4.00};
  double linearGain_{0.8};
  double angularGain_{2.0};
  double pathPidKp_{2.4};
  double pathPidKi_{0.35};
  double pathPidKd_{0.0};
  double pathYawRateDamping_{0.18};
  double turnYawRateDamping_{0.10};
  double turnBrakeHorizon_{0.35};
  double turnCruiseSpeed_{0.65};
  double crossTrackGain_{2.6};
  double stanleySofteningSpeed_{0.25};
  double maxCrossTrackCorrection_{0.70};
  double crossTrackSlowdownStart_{0.03};
  double crossTrackSlowdownFull_{0.06};
  double crossTrackMinimumSpeed_{0.08};
  double pathPidIntegralLimit_{0.4};
  double rotateInPlaceThreshold_{0.18};
  double rotateInPlaceReentryThreshold_{0.44};
  double rotateInPlaceExitThreshold_{0.035};
  double precisionTurnThreshold_{0.45};
  double turnSettleYawRate_{0.12};
  double turnSettleDwell_{0.30};
  double minPrecisionTurnSpeed_{0.25};
  double waypointTolerance_{0.08};
  double finalYawTolerance_{0.060};
  double finalYawMaxAngularSpeed_{0.80};
  double finalYawMinTurnSpeed_{0.70};
  double finalYawMinPrecisionSpeed_{0.32};
  double odomTimeout_{0.40};
  double imuTimeout_{0.15};
  double fusionStatusTimeout_{0.60};
  double actuatorHealthTimeout_{0.80};
  double transientLocalizationGrace_{2.00};
  double transientFaultSpeedScale_{0.25};
  bool requireFusionStatus_{true};
  bool requireActuatorHealth_{false};
  bool requireTrackingState_{false};
  bool abortOnTrackingLoss_{false};
  bool autostart_{false};
  std::vector<std::string> allowedFusionStates_;
  visual_navigation::FusionHealthPolicy fusionHealthPolicy_;

  std::vector<Waypoint> waypoints_;
  std::size_t currentWaypointIndex_{0};
  bool routeLoaded_{false};
  bool navigationActive_{false};
  bool hasOdometry_{false};
  bool hasImu_{false};
  bool currentOdomFrameValid_{false};
  bool currentOdomPoseValid_{false};
  bool hasFusionStatus_{false};
  bool hasActuatorHealth_{false};
  bool hasTrackingState_{false};
  bool trackingReferenceInitialized_{false};
  bool localizationWasValid_{false};
  bool validLocalizationTimeInitialized_{false};
  int trackingState_{-1};
  std::string fusionStatus_;
  bool actuatorConnected_{false};
  double currentX_{0.0};
  double currentY_{0.0};
  double currentYaw_{0.0};
  double trackingReferenceYaw_{0.0};
  double imuYawRate_{0.0};
  double pathSegmentStartX_{0.0};
  double pathSegmentStartY_{0.0};
  double pathPidIntegral_{0.0};
  double previousPathError_{0.0};
  bool pathSegmentInitialized_{false};
  bool pathAlignmentCompleted_{false};
  bool pathPidInitialized_{false};
  bool rotatingInPlace_{false};
  bool turnBraking_{false};
  double turnDirection_{0.0};
  bool turnSettleTimerInitialized_{false};
  bool finalPositionCaptured_{false};
  std::string state_;
  rclcpp::Time lastOdomArrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastImuArrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastFusionStatusArrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastActuatorHealthArrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time waitUntil_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point lastPathPidTime_{};
  std::chrono::steady_clock::time_point lastMotionCommandTime_{};
  std::chrono::steady_clock::time_point lastValidLocalizationTime_{};
  std::chrono::steady_clock::time_point turnSettleStarted_{};
  geometry_msgs::msg::Twist lastMotionCommand_;
  bool motionCommandInitialized_{false};
  WaitAction waitAction_{WaitAction::NONE};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr currentWaypointPublisher_;
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
  rclcpp::TimerBase::SharedPtr controlTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
