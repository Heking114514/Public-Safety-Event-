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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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
    odomTopic_ = declare_parameter<std::string>("odom_topic", "/odom");
    trackingStateTopic_ = declare_parameter<std::string>("tracking_state_topic", "/tracking_state");
    cmdVelTopic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
    pathTopic_ = declare_parameter<std::string>("path_topic", "/waypoint_path");
    statusTopic_ = declare_parameter<std::string>("status_topic", "/waypoint_navigation/status");
    currentWaypointTopic_ = declare_parameter<std::string>(
      "current_waypoint_topic", "/waypoint_navigation/current_waypoint");

    controlFrequency_ = std::max(1.0, declare_parameter<double>("control_frequency", 30.0));
    defaultSpeed_ = std::max(0.0, declare_parameter<double>("default_speed", 0.20));
    maxLinearSpeed_ = std::max(0.0, declare_parameter<double>("max_linear_speed", 0.30));
    maxAngularSpeed_ = std::max(0.0, declare_parameter<double>("max_angular_speed", 0.80));
    linearGain_ = std::max(0.0, declare_parameter<double>("linear_gain", 0.8));
    angularGain_ = std::max(0.0, declare_parameter<double>("angular_gain", 1.8));
    rotateInPlaceThreshold_ = std::max(
      0.0, declare_parameter<double>("rotate_in_place_threshold", 0.60));
    waypointTolerance_ = std::max(
      0.001, declare_parameter<double>("waypoint_tolerance", 0.15));
    finalYawTolerance_ = std::max(
      0.001, declare_parameter<double>("final_yaw_tolerance", 0.12));
    odomTimeout_ = std::max(0.01, declare_parameter<double>("odom_timeout", 0.40));
    requireTrackingState_ = declare_parameter<bool>("require_tracking_state", true);
    abortOnTrackingLoss_ = declare_parameter<bool>("abort_on_tracking_loss", true);
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
    trackingStateSubscription_ = create_subscription<std_msgs::msg::Int32>(
      trackingStateTopic_, 10,
      std::bind(&WaypointNavigator::HandleTrackingState, this, std::placeholders::_1));

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
      "Waypoint navigator ready: odom=%s tracking=%s output=%s",
      odomTopic_.c_str(), trackingStateTopic_.c_str(), cmdVelTopic_.c_str());
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
    currentX_ = message->pose.pose.position.x;
    currentY_ = message->pose.pose.position.y;
    currentYaw_ = YawFromQuaternion(message->pose.pose.orientation);
    lastOdomArrival_ = now();
    hasOdometry_ = true;

    if (!message->header.frame_id.empty() && message->header.frame_id != routeFrame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odometry frame '%s' differs from route frame '%s'; no TF conversion is performed",
        message->header.frame_id.c_str(), routeFrame_.c_str());
    }
  }

  void HandleTrackingState(const std_msgs::msg::Int32::SharedPtr message)
  {
    trackingState_ = message->data;
    hasTrackingState_ = true;
  }

  void HandleStart(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!routeLoaded_)
    {
      response->success = false;
      response->message = "Waypoint route is not loaded";
      return;
    }

    if (currentWaypointIndex_ >= waypoints_.size())
      currentWaypointIndex_ = 0;
    waitAction_ = WaitAction::NONE;
    navigationActive_ = true;
    localizationWasValid_ = false;
    SetState("WAITING_FOR_LOCALIZATION");
    response->success = true;
    response->message = "Waypoint navigation started";
  }

  void HandleStop(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    navigationActive_ = false;
    waitAction_ = WaitAction::NONE;
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
    localizationWasValid_ = false;
    waitAction_ = WaitAction::NONE;
    PublishStop();
    PublishCurrentWaypoint();
    SetState("IDLE");
    response->success = true;
    response->message = "Waypoint navigation reset";
  }

  bool LocalizationIsValid() const
  {
    if (!hasOdometry_)
      return false;
    if ((now() - lastOdomArrival_).seconds() > odomTimeout_)
      return false;
    if (!requireTrackingState_)
      return true;
    if (!hasTrackingState_)
      return false;
    return trackingState_ == 2 || trackingState_ == 5;
  }

  void RunControl()
  {
    if (!navigationActive_)
    {
      PublishStop();
      return;
    }

    if (!LocalizationIsValid())
    {
      PublishStop();
      if (localizationWasValid_ && abortOnTrackingLoss_)
      {
        navigationActive_ = false;
        SetState("FAULT_TRACKING_LOST");
      }
      else
      {
        SetState("WAITING_FOR_LOCALIZATION");
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
    const double deltaX = target.x - currentX_;
    const double deltaY = target.y - currentY_;
    const double distance = std::hypot(deltaX, deltaY);
    const bool finalWaypoint = currentWaypointIndex_ + 1 == waypoints_.size();

    if (distance <= target.tolerance)
    {
      if (finalWaypoint && std::isfinite(target.yaw))
      {
        const double finalYawError = NormalizeAngle(target.yaw - currentYaw_);
        if (std::abs(finalYawError) > finalYawTolerance_)
        {
          geometry_msgs::msg::Twist command;
          command.angular.z = Clamp(
            angularGain_ * finalYawError, -maxAngularSpeed_, maxAngularSpeed_);
          cmdVelPublisher_->publish(command);
          SetState("ALIGNING_FINAL_YAW");
          return;
        }
      }

      if (target.stopTime > 0.0)
      {
        waitUntil_ = now() + rclcpp::Duration::from_seconds(target.stopTime);
        waitAction_ = finalWaypoint ? WaitAction::COMPLETE : WaitAction::ADVANCE;
        PublishStop();
        SetState("WAITING_AT_WAYPOINT");
        return;
      }

      if (finalWaypoint)
      {
        CompleteNavigation();
        return;
      }

      ++currentWaypointIndex_;
      PublishCurrentWaypoint();
      return;
    }

    const double targetHeading = std::atan2(deltaY, deltaX);
    const double headingError = NormalizeAngle(targetHeading - currentYaw_);
    geometry_msgs::msg::Twist command;
    command.angular.z = Clamp(
      angularGain_ * headingError, -maxAngularSpeed_, maxAngularSpeed_);

    if (std::abs(headingError) < rotateInPlaceThreshold_)
    {
      const double requestedSpeed = std::min(target.speed, maxLinearSpeed_);
      const double headingScale = std::max(0.0, std::cos(headingError));
      double linearSpeed = requestedSpeed;
      if (finalWaypoint)
        linearSpeed = std::min(requestedSpeed, linearGain_ * distance);
      command.linear.x = linearSpeed * headingScale;
    }

    cmdVelPublisher_->publish(command);
    SetState("FOLLOWING");
  }

  void CompleteNavigation()
  {
    navigationActive_ = false;
    PublishStop();
    SetState("GOAL_REACHED");
  }

  void PublishStop()
  {
    cmdVelPublisher_->publish(geometry_msgs::msg::Twist());
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
  std::string trackingStateTopic_;
  std::string cmdVelTopic_;
  std::string pathTopic_;
  std::string statusTopic_;
  std::string currentWaypointTopic_;
  double controlFrequency_{30.0};
  double defaultSpeed_{0.20};
  double maxLinearSpeed_{0.30};
  double maxAngularSpeed_{0.80};
  double linearGain_{0.8};
  double angularGain_{1.8};
  double rotateInPlaceThreshold_{0.60};
  double waypointTolerance_{0.15};
  double finalYawTolerance_{0.12};
  double odomTimeout_{0.40};
  bool requireTrackingState_{true};
  bool abortOnTrackingLoss_{true};
  bool autostart_{false};

  std::vector<Waypoint> waypoints_;
  std::size_t currentWaypointIndex_{0};
  bool routeLoaded_{false};
  bool navigationActive_{false};
  bool hasOdometry_{false};
  bool hasTrackingState_{false};
  bool localizationWasValid_{false};
  int trackingState_{-1};
  double currentX_{0.0};
  double currentY_{0.0};
  double currentYaw_{0.0};
  std::string state_;
  rclcpp::Time lastOdomArrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Time waitUntil_{0, 0, RCL_ROS_TIME};
  WaitAction waitAction_{WaitAction::NONE};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr currentWaypointPublisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trackingStateSubscription_;
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
