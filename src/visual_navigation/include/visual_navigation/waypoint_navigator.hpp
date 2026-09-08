#ifndef VISUAL_NAVIGATION__WAYPOINTNAVIGATOR_HPP_
#define VISUAL_NAVIGATION__WAYPOINTNAVIGATOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
#include "visual_navigation/control_state.hpp"
#include "visual_navigation/fusion_health_policy.hpp"
#include "visual_navigation/navigation_input_cache.hpp"
#include "visual_navigation/navigation_supervisor.hpp"
#include "visual_navigation/odometry_input_validation.hpp"
#include "visual_navigation/path_control.hpp"
#include "visual_navigation/path_progress_supervisor.hpp"
#include "visual_navigation/path_tracking_controller.hpp"
#include "visual_navigation/rate_limiter.hpp"
#include "visual_navigation/route_manager.hpp"
#include "visual_navigation/turn_progress_supervisor.hpp"
#include "visual_navigation/turn_settle_controller.hpp"
#include "visual_navigation/waypoint_brake_controller.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

inline double Clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(maximum, value));
}

inline double NormalizeAngle(double angle) {
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

inline double
YawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion) {
  const double sinYaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosYaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sinYaw, cosYaw);
}

inline double
YawFromValidQuaternion(const geometry_msgs::msg::Quaternion &quaternion) {
  const double norm =
      std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  geometry_msgs::msg::Quaternion normalized;
  normalized.x = quaternion.x / norm;
  normalized.y = quaternion.y / norm;
  normalized.z = quaternion.z / norm;
  normalized.w = quaternion.w / norm;
  return YawFromQuaternion(normalized);
}

inline bool
QuaternionIsValid(const geometry_msgs::msg::Quaternion &quaternion) {
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
      !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w)) {
    return false;
  }

  const double squaredNorm =
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(squaredNorm) && squaredNorm > 1e-12;
}

inline geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

inline std::string Trim(const std::string &text) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = text.find_first_not_of(whitespace);
  if (first == std::string::npos)
    return "";
  const std::size_t last = text.find_last_not_of(whitespace);
  return text.substr(first, last - first + 1);
}

} // namespace

class WaypointNavigator : public rclcpp::Node {
public:
  WaypointNavigator();
  ~WaypointNavigator() override;

private:
  using Waypoint = visual_navigation::RouteWaypoint;

  enum class ControlStep { DONE, CONTINUE };

  struct ControlFrame {
    Waypoint target;
    visual_navigation::FusionHealthDecision fusion_health;
    visual_navigation::PathProjection path_projection;
    double delta_x{0.0};
    double delta_y{0.0};
    double distance{0.0};
    double control_yaw_rate{0.0};
    double path_length{0.0};
    double path_heading{0.0};
    bool final_waypoint{false};
    bool waypoint_requires_stop{false};
    bool waypoint_reached{false};
    bool progress_supervision_allowed{false};
  };

  bool LoadRoute(const std::string &routeFile);

  void PublishRoutePath();

  void PublishRouteAck(uint64_t route_id);

  void HandleOdometry(const nav_msgs::msg::Odometry::SharedPtr message);

  void HandleImu(const sensor_msgs::msg::Imu::SharedPtr message);

  void HandleTrackingState(const std_msgs::msg::Int32::SharedPtr message);

  void HandleFusionStatus(const std_msgs::msg::String::SharedPtr message);

  void HandleActuatorHealth(const std_msgs::msg::Bool::SharedPtr message);

  void HandleRouteInput(const nav_msgs::msg::Path::SharedPtr message);

  void HandleStart(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response);

  void HandleStartTopic(const std_msgs::msg::Empty::SharedPtr);

  bool ActivateNavigation(std::string &result);

  void HandleStop(const std_srvs::srv::Trigger::Request::SharedPtr,
                  std_srvs::srv::Trigger::Response::SharedPtr response);

  void HandleSetMotionHold(
      const mission_control_interfaces::srv::SetMotionHold::Request::SharedPtr
          request,
      mission_control_interfaces::srv::SetMotionHold::Response::SharedPtr
          response);

  void HandleReset(const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response);

  bool ImuYawRateIsFresh() const;

  bool
  AcceptMeasurementStamp(const builtin_interfaces::msg::Time &stamp_message,
                         rclcpp::Time &last_stamp, bool &stamp_valid,
                         double max_age, const char *source);

  double ControlYawRate() const;

  visual_navigation::NavigationInputStatus CurrentNavigationInputs() const;

  void RunControl();

  ControlFrame BuildControlFrame(
      const visual_navigation::FusionHealthDecision &fusion_health,
      bool progress_supervision_allowed) const;

  ControlStep RunWait();

  ControlStep RunBrake(const ControlFrame &frame);

  void RunGoal(const ControlFrame &frame);

  void RunRecovery(const ControlFrame &frame);

  void RunPath(const ControlFrame &frame);

  void AdvanceWaypoint();

  void CompleteNavigation();

  void BeginPathSegment();

  bool SupervisePathProgress(
      double measuredProgress, bool eligible,
      double commandedLinearVelocity = 0.0,
      double measuredLinearVelocity = 0.0,
      bool measuredLinearVelocityValid = false,
      bool linearMotionExpected = true);

  bool SuperviseTurnProgress(std::size_t phaseOffset, double absoluteYawError,
                             double expectedYawRate, bool eligible);

  double CompletedRouteLengthBeforeCurrentSegment() const;

  void BeginFinalPositionRecovery();

  bool WaypointRequiresStop(double currentPathHeading,
                            bool finalWaypoint) const;

  void BeginWaypointBraking();

  void UpdateObservedLinearSpeed();

  bool WaypointBrakeHasCompleted();

  void ResetPathFeedback();

  void ResetManeuver();

  void ResetRunControl();

  double UpdatePathPid(double error, double crossTrackError, bool allowIntegral,
                       double yawRate);

  void PublishStop();

  void PublishMotionCommand(const geometry_msgs::msg::Twist &desired);

  void PublishCurrentWaypoint();

  bool MotionHeld() const;

  std::vector<std::string> ActiveHoldSources() const;

  void PublishMotionHoldState();

  void PublishState();

  void SetState(const std::string &state);

  std::string routeFile_;
  std::string routeFrame_;
  // Odometry is expected to describe the configured route frame -> base frame
  // transform. Navigation has no TF lookup, so this pair is checked literally.
  std::string odomChildFrame_;
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
  double trackingReferenceYaw_{0.0};
  double imuYawRate_{0.0};
  double pathSegmentStartX_{0.0};
  double pathSegmentStartY_{0.0};
  bool pathSegmentInitialized_{false};
  std::string state_;
  std::map<std::string, std::string> motionHolds_;
  rclcpp::Time lastOdomStamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastImuStamp_{0, 0, RCL_ROS_TIME};
  bool odomStampValid_{false};
  bool imuStampValid_{false};
  rclcpp::Time waitUntil_{0, 0, RCL_ROS_TIME};
  rclcpp::Time motionHoldStartedAt_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point lastMotionCommandTime_{};
  visual_navigation::PathTrackingController pathTrackingController_;
  visual_navigation::WaypointBrakeController waypointBrakeController_;
  visual_navigation::TurnSettleController turnSettleController_;
  visual_navigation::ControlState controlState_;
  geometry_msgs::msg::Twist lastMotionCommand_;
  bool motionCommandInitialized_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr currentWaypointPublisher_;
  rclcpp::Publisher<mission_control_interfaces::msg::MotionHoldState>::SharedPtr
      motionHoldStatePublisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr routeAckPublisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      fusionStatusSubscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      actuatorHealthSubscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      trackingStateSubscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr routeInputSubscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr startSubscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stopService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService_;
  rclcpp::Service<mission_control_interfaces::srv::SetMotionHold>::SharedPtr
      motionHoldService_;
  rclcpp::TimerBase::SharedPtr controlTimer_;
};

#endif // VISUAL_NAVIGATION__WAYPOINTNAVIGATOR_HPP_
