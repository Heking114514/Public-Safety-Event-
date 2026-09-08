#include "visual_navigation/waypoint_navigator.hpp"

WaypointNavigator::WaypointNavigator() : Node("waypoint_navigator") {
  routeFile_ = declare_parameter<std::string>("route_file", "");
  routeFrame_ = declare_parameter<std::string>("route_frame", "map");
  odomChildFrame_ = declare_parameter<std::string>("odom_child_frame", "base_link");
  if (Trim(routeFrame_).empty() || Trim(odomChildFrame_).empty()) {
    throw std::invalid_argument(
      "route_frame and odom_child_frame must not be empty");
  }
  odomTopic_ = declare_parameter<std::string>("odom_topic", "/odometry/fused");
  imuTopic_ = declare_parameter<std::string>("imu_topic", "/imu/control");
  trackingPointOffsetX_ =
      declare_parameter<double>("tracking_point_offset_x", 0.0);
  trackingPointOffsetY_ =
      declare_parameter<double>("tracking_point_offset_y", 0.0);
  if (!std::isfinite(trackingPointOffsetX_) ||
      !std::isfinite(trackingPointOffsetY_))
    throw std::invalid_argument("tracking point offsets must be finite");
  fusionStatusTopic_ = declare_parameter<std::string>(
      "fusion_status_topic", "/odometry/fusion_status");
  actuatorHealthTopic_ = declare_parameter<std::string>(
      "actuator_health_topic", "/cup_car_serial/actuator_healthy");
  trackingStateTopic_ =
      declare_parameter<std::string>("tracking_state_topic", "/tracking_state");
  cmdVelTopic_ =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_nav");
  pathTopic_ = declare_parameter<std::string>("path_topic", "/waypoint_path");
  routeInputTopic_ = declare_parameter<std::string>(
      "route_input_topic", "/waypoint_navigation/route_input");
  startTopic_ = declare_parameter<std::string>("start_topic",
                                               "/waypoint_navigation/start");
  statusTopic_ = declare_parameter<std::string>("status_topic",
                                                "/waypoint_navigation/status");
  currentWaypointTopic_ = declare_parameter<std::string>(
      "current_waypoint_topic", "/waypoint_navigation/current_waypoint");
  motionHoldStateTopic_ = declare_parameter<std::string>(
      "motion_hold_state_topic", "/waypoint_navigation/motion_hold_state");
  routeAckTopic_ = declare_parameter<std::string>(
      "route_ack_topic", "/waypoint_navigation/route_ack");

  controlFrequency_ =
      std::max(1.0, declare_parameter<double>("control_frequency", 30.0));
  defaultSpeed_ =
      std::max(0.0, declare_parameter<double>("default_speed", 0.50));
  maxLinearSpeed_ =
      std::max(0.0, declare_parameter<double>("max_linear_speed", 0.50));
  maxAngularSpeed_ =
      std::max(0.0, declare_parameter<double>("max_angular_speed", 0.95));
  maxPathAngularSpeed_ = std::min(
      maxAngularSpeed_,
      std::max(0.0, declare_parameter<double>("max_path_angular_speed", 0.45)));
  minPathAngularSpeed_ = std::min(
      maxPathAngularSpeed_,
      std::max(0.0, declare_parameter<double>("min_path_angular_speed", 0.14)));
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
  brakingControlDelay_ =
      std::max(0.0, declare_parameter<double>("braking_control_delay", 0.20));
  brakingSafetyMargin_ =
      std::max(0.0, declare_parameter<double>("braking_safety_margin", 0.015));
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
  pathYawRateDamping_ =
      std::max(0.0, declare_parameter<double>("path_yaw_rate_damping", 0.45));
  turnYawRateDamping_ =
      std::max(0.0, declare_parameter<double>("turn_yaw_rate_damping", 0.55));
  turnCruiseSpeed_ = std::min(
      maxAngularSpeed_,
      std::max(0.0, declare_parameter<double>("turn_cruise_speed", 0.90)));
  turnPositionHoldGain_ =
      std::max(0.0, declare_parameter<double>("turn_position_hold_gain", 0.80));
  turnPositionHoldDeadband_ = std::max(
      0.0, declare_parameter<double>("turn_position_hold_deadband", 0.015));
  turnPositionHoldMaxSpeed_ =
      std::min(maxLinearSpeed_,
               std::max(0.0, declare_parameter<double>(
                                 "turn_position_hold_max_speed", 0.08)));
  crossTrackGain_ =
      std::max(0.0, declare_parameter<double>("cross_track_gain", 1.5));
  stanleySofteningSpeed_ = std::max(
      0.01, declare_parameter<double>("stanley_softening_speed", 0.25));
  maxCrossTrackCorrection_ = std::max(
      0.0, declare_parameter<double>("max_cross_track_correction", 0.70));
  crossTrackSlowdownStart_ = std::max(
      0.0, declare_parameter<double>("cross_track_slowdown_start", 0.015));
  crossTrackSlowdownFull_ =
      std::max(crossTrackSlowdownStart_ + 0.001,
               declare_parameter<double>("cross_track_slowdown_full", 0.06));
  crossTrackMinimumSpeed_ = std::max(
      0.0, declare_parameter<double>("cross_track_minimum_speed", 0.20));
  maxLateralAcceleration_ = std::max(
      0.01, declare_parameter<double>("max_lateral_acceleration", 0.22));
  pathCurvatureFeedforwardGain_ = std::max(
      0.0, declare_parameter<double>("path_curvature_feedforward_gain", 1.0));
  pathPidIntegralLimit_ =
      std::max(0.0, declare_parameter<double>("path_pid_integral_limit", 0.20));
  pathTrackingController_.configure(pathPidKp_, pathPidKi_, pathPidKd_,
                                    pathYawRateDamping_, pathPidIntegralLimit_,
                                    maxPathAngularSpeed_);
  rotateInPlaceThreshold_ = std::max(
      0.0, declare_parameter<double>("rotate_in_place_threshold", 0.18));
  rotateInPlaceReentryThreshold_ = std::max(
      rotateInPlaceThreshold_,
      declare_parameter<double>("rotate_in_place_reentry_threshold", 0.44));
  rotateInPlaceExitThreshold_ =
      std::min(rotateInPlaceThreshold_,
               std::max(0.0, declare_parameter<double>(
                                 "rotate_in_place_exit_threshold", 0.035)));
  precisionTurnThreshold_ =
      std::max(rotateInPlaceExitThreshold_,
               declare_parameter<double>("precision_turn_threshold", 0.45));
  turnSettleYawRate_ =
      std::max(0.01, declare_parameter<double>("turn_settle_yaw_rate", 0.12));
  turnSettleDwell_ =
      std::max(0.0, declare_parameter<double>("turn_settle_dwell", 0.30));
  minPrecisionTurnSpeed_ = std::min(
      turnCruiseSpeed_, std::max(0.0, declare_parameter<double>(
                                          "min_precision_turn_speed", 0.25)));
  waypointTolerance_ =
      std::max(0.001, declare_parameter<double>("waypoint_tolerance", 0.04));
  waypointPassLongitudinalTolerance_ = std::max(
      0.0,
      declare_parameter<double>("waypoint_pass_longitudinal_tolerance", 0.01));
  waypointPassLateralTolerance_ = std::max(
      waypointTolerance_,
      declare_parameter<double>("waypoint_pass_lateral_tolerance", 0.06));
  waypointRecoverySpeed_ = std::min(
      maxLinearSpeed_, std::max(0.0, declare_parameter<double>(
                                         "waypoint_recovery_speed", 0.10)));
  waypointRecoveryHeadingTolerance_ = std::max(
      0.01,
      declare_parameter<double>("waypoint_recovery_heading_tolerance", 0.12));
  waypointRecoveryMaxAngularSpeed_ =
      std::min(maxAngularSpeed_,
               std::max(0.0, declare_parameter<double>(
                                 "waypoint_recovery_max_angular_speed", 0.60)));
  const double pathProgressTimeout =
      std::max(0.0, declare_parameter<double>("path_progress_timeout", 10.0));
  const double pathProgressMinimumAdvance = std::max(
      0.0, declare_parameter<double>("path_progress_minimum_advance", 0.05));
  const double pathProgressMinimumActualDisplacement =
      std::max(0.0, declare_parameter<double>(
                        "path_progress_minimum_actual_displacement", 0.25));
  const int pathProgressMaxRecoveryAttempts =
      std::max(0, static_cast<int>(declare_parameter<int64_t>(
                      "path_progress_max_recovery_attempts", 2)));
  const double pathProgressCommandThreshold = std::max(
      0.0, declare_parameter<double>("path_progress_command_threshold", 0.08));
  const double pathProgressStationarySpeedThreshold = std::max(
      0.0, declare_parameter<double>(
                "path_progress_stationary_speed_threshold", 0.02));
  const double pathProgressNoMotionTimeout = std::max(
      0.0, declare_parameter<double>("path_progress_no_motion_timeout", 3.0));
  const double pathProgressNoMotionDisplacement = std::max(
      0.0, declare_parameter<double>(
                "path_progress_no_motion_displacement", 0.05));
  pathProgressSupervisor_ = visual_navigation::PathProgressSupervisor(
      {pathProgressTimeout, pathProgressMinimumAdvance,
       pathProgressMinimumActualDisplacement, pathProgressMaxRecoveryAttempts,
       pathProgressCommandThreshold, pathProgressStationarySpeedThreshold,
       pathProgressNoMotionTimeout, pathProgressNoMotionDisplacement});
  const double turnProgressStallTimeout = std::max(
      0.0, declare_parameter<double>("turn_progress_stall_timeout", 6.0));
  const double turnProgressMinimumErrorReduction = std::max(
      0.0,
      declare_parameter<double>("turn_progress_minimum_error_reduction", 0.08));
  const double turnProgressBaseTotalTimeout = std::max(
      0.0, declare_parameter<double>("turn_progress_base_total_timeout", 12.0));
  const double turnProgressMinimumExpectedYawRate =
      std::max(0.001, declare_parameter<double>(
                          "turn_progress_minimum_expected_yaw_rate", 0.12));
  const double turnProgressMaxTotalTimeout = std::max(
      turnProgressBaseTotalTimeout,
      declare_parameter<double>("turn_progress_max_total_timeout", 40.0));
  const int turnProgressMaxRecoveryAttempts =
      std::max(0, static_cast<int>(declare_parameter<int64_t>(
                      "turn_progress_max_recovery_attempts", 2)));
  turnProgressSupervisor_ = visual_navigation::TurnProgressSupervisor(
      {turnProgressStallTimeout, turnProgressMinimumErrorReduction,
       turnProgressBaseTotalTimeout, turnProgressMinimumExpectedYawRate,
       turnProgressMaxTotalTimeout, turnProgressMaxRecoveryAttempts});
  preTurnStopHeadingThreshold_ = std::max(
      0.0, declare_parameter<double>("pre_turn_stop_heading_threshold", 0.18));
  preTurnStopSpeed_ =
      std::max(0.0, declare_parameter<double>("pre_turn_stop_speed", 0.03));
  preTurnStopDwell_ =
      std::max(0.0, declare_parameter<double>("pre_turn_stop_dwell", 0.10));
  stopMotionWindow_ =
      std::max(0.05, declare_parameter<double>("stop_motion_window", 0.20));
  preTurnMinimumStopTime_ = std::max(
      0.0, declare_parameter<double>("pre_turn_minimum_stop_time", 0.20));
  preTurnBrakeTimeout_ =
      std::max(preTurnMinimumStopTime_,
               declare_parameter<double>("pre_turn_brake_timeout", 0.60));
  waypointBrakeController_.configure(preTurnStopSpeed_, preTurnStopDwell_,
                                     preTurnMinimumStopTime_,
                                     preTurnBrakeTimeout_);
  finalYawTolerance_ =
      std::max(0.001, declare_parameter<double>("final_yaw_tolerance", 0.060));
  finalYawMaxAngularSpeed_ = std::min(
      maxAngularSpeed_,
      std::max(0.0,
               declare_parameter<double>("final_yaw_max_angular_speed", 0.90)));
  finalYawMinTurnSpeed_ = std::min(
      finalYawMaxAngularSpeed_,
      std::max(0.0,
               declare_parameter<double>("final_yaw_min_turn_speed", 0.70)));
  finalYawMinPrecisionSpeed_ =
      std::min(finalYawMinTurnSpeed_,
               std::max(0.0, declare_parameter<double>(
                                 "final_yaw_min_precision_speed", 0.22)));
  odomTimeout_ =
      std::max(0.01, declare_parameter<double>("odom_timeout", 0.40));
  imuTimeout_ = std::max(0.01, declare_parameter<double>("imu_timeout", 0.15));
  fusionStatusTimeout_ =
      std::max(0.01, declare_parameter<double>("fusion_status_timeout", 0.60));
  actuatorHealthTimeout_ = std::max(
      0.01, declare_parameter<double>("actuator_health_timeout", 0.80));
  stampFutureTolerance_ =
      std::max(0.0, declare_parameter<double>("stamp_future_tolerance", 0.20));
  const double transientLocalizationGrace = std::max(
      0.0, declare_parameter<double>("transient_localization_grace", 2.00));
  const double transientFaultSpeedScale = Clamp(
      declare_parameter<double>("transient_fault_speed_scale", 0.25), 0.0, 1.0);
  requireFusionStatus_ = declare_parameter<bool>("require_fusion_status", true);
  requireActuatorHealth_ =
      declare_parameter<bool>("require_actuator_health", false);
  allowedFusionStates_ = declare_parameter<std::vector<std::string>>(
      "allowed_fusion_states",
      {"FULL", "DEGRADED_NO_VISION", "DEGRADED_NO_IMU", "DEGRADED_NO_WHEEL",
       "DEGRADED_VISION_ONLY", "DEGRADED_WHEEL_ONLY",
       "DEGRADED_VISUAL_REALIGNED"});
  visual_navigation::FusionSpeedScales fusionSpeedScales;
  fusionSpeedScales.fallback =
      declare_parameter<double>("degraded_speed_scale", 0.50);
  fusionSpeedScales.no_vision =
      declare_parameter<double>("no_vision_speed_scale", 0.50);
  fusionSpeedScales.no_imu =
      declare_parameter<double>("no_imu_speed_scale", 0.65);
  fusionSpeedScales.no_wheel =
      declare_parameter<double>("no_wheel_speed_scale", 0.60);
  fusionSpeedScales.vision_only =
      declare_parameter<double>("vision_only_speed_scale", 0.40);
  fusionSpeedScales.wheel_only =
      declare_parameter<double>("wheel_only_speed_scale", 0.25);
  fusionSpeedScales.visual_realigned =
      declare_parameter<double>("visual_realigned_speed_scale", 0.50);
  fusionHealthPolicy_ = visual_navigation::FusionHealthPolicy(
      allowedFusionStates_, fusionSpeedScales);
  requireTrackingState_ =
      declare_parameter<bool>("require_tracking_state", false);
  const bool abortOnTrackingLoss =
      declare_parameter<bool>("abort_on_tracking_loss", false);
  navigationSupervisor_ = visual_navigation::NavigationSupervisor(
      {transientLocalizationGrace, transientFaultSpeedScale,
       abortOnTrackingLoss});
  autostart_ = declare_parameter<bool>("autostart", false);

  cmdVelPublisher_ =
      create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic_, 10);
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
      std::bind(&WaypointNavigator::HandleOdometry, this,
                std::placeholders::_1));
  imuSubscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imuTopic_, rclcpp::SensorDataQoS(),
      std::bind(&WaypointNavigator::HandleImu, this, std::placeholders::_1));
  fusionStatusSubscription_ = create_subscription<std_msgs::msg::String>(
      fusionStatusTopic_, 10,
      std::bind(&WaypointNavigator::HandleFusionStatus, this,
                std::placeholders::_1));
  actuatorHealthSubscription_ = create_subscription<std_msgs::msg::Bool>(
      actuatorHealthTopic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&WaypointNavigator::HandleActuatorHealth, this,
                std::placeholders::_1));
  trackingStateSubscription_ = create_subscription<std_msgs::msg::Int32>(
      trackingStateTopic_, 10,
      std::bind(&WaypointNavigator::HandleTrackingState, this,
                std::placeholders::_1));
  routeInputSubscription_ = create_subscription<nav_msgs::msg::Path>(
      routeInputTopic_, rclcpp::QoS(1).reliable(),
      std::bind(&WaypointNavigator::HandleRouteInput, this,
                std::placeholders::_1));
  startSubscription_ = create_subscription<std_msgs::msg::Empty>(
      startTopic_, rclcpp::QoS(1).reliable(),
      std::bind(&WaypointNavigator::HandleStartTopic, this,
                std::placeholders::_1));

  startService_ = create_service<std_srvs::srv::Trigger>(
      "~/start", std::bind(&WaypointNavigator::HandleStart, this,
                           std::placeholders::_1, std::placeholders::_2));
  stopService_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&WaypointNavigator::HandleStop, this,
                          std::placeholders::_1, std::placeholders::_2));
  resetService_ = create_service<std_srvs::srv::Trigger>(
      "~/reset", std::bind(&WaypointNavigator::HandleReset, this,
                           std::placeholders::_1, std::placeholders::_2));
  motionHoldService_ =
      create_service<mission_control_interfaces::srv::SetMotionHold>(
          "~/set_motion_hold",
          std::bind(&WaypointNavigator::HandleSetMotionHold, this,
                    std::placeholders::_1, std::placeholders::_2));

  routeLoaded_ = LoadRoute(routeFile_);
  if (routeLoaded_) {
    PublishRoutePath();
    PublishCurrentWaypoint();
  }

  const auto period = std::chrono::duration<double>(1.0 / controlFrequency_);
  controlTimer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(period.count()),
      std::bind(&WaypointNavigator::RunControl, this));

  if (!routeLoaded_) {
    SetState(routeFile_.empty() ? "WAITING_FOR_ROUTE"
                                : "FAULT_ROUTE_NOT_LOADED");
  } else if (autostart_) {
    navigationActive_ = true;
    SetState("WAITING_FOR_LOCALIZATION");
  } else {
    SetState("IDLE");
  }
  PublishMotionHoldState();

  RCLCPP_INFO(
      get_logger(),
      "Waypoint navigator ready: odom=%s fusion_status=%s actuator_health=%s "
      "require_actuator_health=%s actuator_health_timeout=%.2f "
      "route_input=%s output=%s",
      odomTopic_.c_str(), fusionStatusTopic_.c_str(),
      actuatorHealthTopic_.c_str(), requireActuatorHealth_ ? "true" : "false",
      actuatorHealthTimeout_, routeInputTopic_.c_str(), cmdVelTopic_.c_str());
}

WaypointNavigator::~WaypointNavigator() { PublishStop(); }
