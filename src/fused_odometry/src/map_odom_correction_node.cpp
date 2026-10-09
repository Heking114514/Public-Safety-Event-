#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <stdexcept>
#include <vector>

#include "fused_odometry/fusion_logic.hpp"
#include "fused_odometry/fusion_status_authority.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mission_control_interfaces/msg/motion_hold_state.hpp"
#include "mission_control_interfaces/srv/correct_map_position.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

using fused_odometry::Pose2d;

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
}

bool valid_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & quaternion = pose.orientation;
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(norm_squared) && norm_squared > 0.25 && norm_squared < 2.25;
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

Pose2d message_pose(const nav_msgs::msg::Odometry & message)
{
  return {
    message.pose.pose.position.x,
    message.pose.pose.position.y,
    quaternion_yaw(message.pose.pose.orientation)};
}

Pose2d limit_pose_step(
  const Pose2d & from, const Pose2d & to, double max_translation, double max_yaw)
{
  Pose2d limited = to;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  if (distance > max_translation) {
    const double scale = max_translation / distance;
    limited.x = from.x + dx * scale;
    limited.y = from.y + dy * scale;
  }
  const double yaw_delta = fused_odometry::wrap_angle(to.yaw - from.yaw);
  const double limited_yaw_delta = std::clamp(yaw_delta, -max_yaw, max_yaw);
  limited.yaw = fused_odometry::wrap_angle(from.yaw + limited_yaw_delta);
  return limited;
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double finite_covariance(double value, double fallback)
{
  return std::isfinite(value) && value >= 0.0 && value < 1.0e6 ? value : fallback;
}

}  // namespace

class MapOdomCorrectionNode : public rclcpp::Node
{
public:
  MapOdomCorrectionNode()
  : Node("map_odom_correction")
  {
    visual_topic_ = declare_parameter<std::string>(
      "visual_topic", "/fusion/input/visual_odom");
    local_topic_ = declare_parameter<std::string>("local_topic", "/odometry/local");
    fusion_status_topic_ = declare_parameter<std::string>(
      "fusion_status_topic", "/odometry/fusion_status");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/fused");
    map_change_topic_ = declare_parameter<std::string>(
      "map_change_topic", "/orbslam3/map_change");
    correct_position_service_ = declare_parameter<std::string>(
      "correct_map_position_service", "/fused_odometry/correct_map_position");
    rfid_hold_state_topic_ = declare_parameter<std::string>(
      "rfid_hold_state_topic", "/waypoint_navigation/motion_hold_state");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    initial_map_x_ = declare_parameter<double>("initial_map_x", 0.0);
    initial_map_y_ = declare_parameter<double>("initial_map_y", 0.0);
    initial_map_yaw_ = declare_parameter<double>("initial_map_yaw", 0.0);
    use_absolute_visual_correction_ = declare_parameter<bool>(
      "use_absolute_visual_correction", true);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const bool deprecated_direct_visual_tracking =
      declare_parameter<bool>("direct_visual_tracking", false);
    if (deprecated_direct_visual_tracking) {
      RCLCPP_WARN(
        get_logger(), "direct_visual_tracking is deprecated; visual corrections remain smoothed");
    }
    visual_recovery_gap_ = positive("visual_recovery_gap_s", 0.60);
    visual_recovery_time_constant_ = positive(
      "visual_recovery_time_constant_s", 0.25);
    visual_recovery_blend_duration_ = positive(
      "visual_recovery_blend_duration_s", 0.80);
    hold_global_xy_during_turn_ = declare_parameter<bool>("hold_global_xy_during_turn", false);
    publish_frequency_ = positive("publish_frequency", 30.0);
    correction_time_constant_ = positive("correction_time_constant_s", 0.15);
    stationary_correction_time_constant_ = positive(
      "stationary_correction_time_constant_s", 0.08);
    loop_correction_time_constant_ = positive("loop_correction_time_constant_s", 1.25);
    max_correction_step_ = positive("max_correction_step_m", 0.03);
    max_correction_yaw_step_ = positive("max_correction_yaw_step_rad", 0.03);
    moving_max_correction_step_ = positive(
      "moving_max_correction_step_m", max_correction_step_);
    moving_max_correction_yaw_step_ = positive(
      "moving_max_correction_yaw_step_rad", max_correction_yaw_step_);
    visual_recovery_max_correction_step_ = positive(
      "visual_recovery_max_correction_step_m", moving_max_correction_step_);
    visual_recovery_max_correction_yaw_step_ = positive(
      "visual_recovery_max_correction_yaw_step_rad", moving_max_correction_yaw_step_);
    map_change_smoothing_window_ = positive("map_change_smoothing_window_s", 2.0);
    synchronization_tolerance_ = positive("synchronization_tolerance_s", 0.12);
    local_timeout_ = positive("local_timeout_s", 0.50);
    visual_timeout_ = positive("visual_timeout_s", 0.60);
    fusion_status_timeout_ = positive("fusion_status_timeout_s", 0.80);
    stationary_max_linear_speed_ = positive("stationary_max_linear_speed_mps", 0.03);
    stationary_max_angular_speed_ = positive("stationary_max_angular_speed_radps", 0.12);
    turn_hold_max_linear_speed_ = positive("turn_hold_max_linear_speed_mps", 0.05);
    turn_hold_max_translation_ = positive("turn_hold_max_translation_m", 0.04);
    turn_hold_entry_angular_speed_ = positive("turn_hold_entry_angular_speed_radps", 0.55);
    turn_hold_min_angular_speed_ = positive("turn_hold_min_angular_speed_radps", 0.12);
    turn_hold_release_delay_ = positive("turn_hold_release_delay_s", 0.28);
    rfid_correction_time_constant_ = positive(
      "rfid_correction_time_constant_s", 0.15);
    rfid_correction_max_step_ = positive("rfid_correction_max_step_m", 0.10);
    dead_reckoning_xy_variance_rate_ = positive(
      "dead_reckoning_xy_variance_rate", 0.0025);
    dead_reckoning_yaw_variance_rate_ = positive(
      "dead_reckoning_yaw_variance_rate", 0.0010);
    if (turn_hold_min_angular_speed_ >= turn_hold_entry_angular_speed_) {
      throw std::invalid_argument(
              "turn_hold_min_angular_speed_radps must be less than entry speed");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    local_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      local_topic_, sensor_qos,
      std::bind(&MapOdomCorrectionNode::local_callback, this, std::placeholders::_1));
    visual_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      visual_topic_, sensor_qos,
      std::bind(&MapOdomCorrectionNode::visual_callback, this, std::placeholders::_1));
    fusion_status_subscription_ = create_subscription<std_msgs::msg::String>(
      fusion_status_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&MapOdomCorrectionNode::fusion_status_callback, this, std::placeholders::_1));
    map_change_subscription_ = create_subscription<std_msgs::msg::UInt64>(
      map_change_topic_, 10,
      std::bind(&MapOdomCorrectionNode::map_change_callback, this, std::placeholders::_1));
    rfid_hold_subscription_ = create_subscription<mission_control_interfaces::msg::MotionHoldState>(
      rfid_hold_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const mission_control_interfaces::msg::MotionHoldState::SharedPtr message) {
        rfid_hold_active_ = message->held && std::find(
          message->active_sources.begin(), message->active_sources.end(),
          "navigation_mission_manager/rfid") !=
          message->active_sources.end();
      });
    correct_position_service_server_ = create_service<
      mission_control_interfaces::srv::CorrectMapPosition>(
      correct_position_service_,
      std::bind(
        &MapOdomCorrectionNode::correct_map_position, this,
        std::placeholders::_1, std::placeholders::_2));
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(10).reliable());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_frequency_),
      std::bind(&MapOdomCorrectionNode::publish, this));
    RCLCPP_INFO(
      get_logger(),
      "Global correction ready: %s -> %s -> %s, local=%s visual=%s output=%s",
      map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
      local_topic_.c_str(), visual_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Absolute visual map correction: %s",
      use_absolute_visual_correction_ ? "ENABLED" : "DISABLED (start-aligned local)");
  }

private:
  struct TimedLocalPose
  {
    rclcpp::Time stamp;
    Pose2d pose;
  };

  double positive(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  void local_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_ ||
      !valid_pose(message->pose.pose))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting local odometry frame/value (%s -> %s; expected %s -> %s)",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }
    const rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting local odometry with an unset measurement timestamp");
      return;
    }
    if (!local_history_.empty() && stamp <= local_history_.back().stamp) {
      return;
    }
    local_history_.push_back({stamp, message_pose(*message)});
    while (local_history_.size() > 120) {
      local_history_.pop_front();
    }
    latest_local_ = *message;
    local_received_ = true;
    local_received_at_ = steady_seconds();
    if (!use_absolute_visual_correction_ && !correction_valid_) {
      // Keep /odometry/fused usable without pretending that ORB's arbitrary
      // absolute XY is a map measurement. This is the same start alignment as
      // local_map_odometry, while preserving the public fused topic/TF pair.
      map_from_odom_ = fused_odometry::compose_pose(
        {initial_map_x_, initial_map_y_, initial_map_yaw_},
        fused_odometry::inverse_pose(message_pose(*message)));
      desired_map_from_odom_ = map_from_odom_;
      correction_valid_ = true;
      last_update_at_ = local_received_at_;
    }
    if (pending_visual_) {
      const auto pending = *pending_visual_;
      process_visual(pending);
    }
  }

  void visual_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    process_visual(*message);
  }

  bool synchronized_local_pose(
    const rclcpp::Time & stamp, Pose2d & pose, double & delta)
  {
    if (local_history_.empty()) {
      delta = std::numeric_limits<double>::infinity();
      return false;
    }
    const auto upper = std::lower_bound(
      local_history_.begin(), local_history_.end(), stamp,
      [](const TimedLocalPose & sample, const rclcpp::Time & value) {
        return sample.stamp < value;
      });
    if (upper != local_history_.end() && upper->stamp == stamp) {
      pose = upper->pose;
      delta = 0.0;
      return true;
    }
    if (upper != local_history_.begin() && upper != local_history_.end()) {
      const auto & before = *(upper - 1);
      const auto & after = *upper;
      const double span = (after.stamp - before.stamp).seconds();
      if (!(span > 0.0) || !std::isfinite(span)) {
        return false;
      }
      const double fraction = std::clamp(
        (stamp - before.stamp).seconds() / span, 0.0, 1.0);
      pose = fused_odometry::interpolate_pose(before.pose, after.pose, fraction);
      delta = std::max(
        (stamp - before.stamp).seconds(), (after.stamp - stamp).seconds());
      return delta <= synchronization_tolerance_;
    }

    // A visual sample newer than the local history may simply have arrived
    // first. Wait for a local sample at that timestamp rather than pairing
    // it with an older pose. Samples older than retained history are stale
    // and are rejected explicitly.
    if (upper == local_history_.end()) {
      delta = (stamp - local_history_.back().stamp).seconds();
      return false;
    }
    delta = (local_history_.front().stamp - stamp).seconds();
    return false;
  }

  void process_visual(const nav_msgs::msg::Odometry & message)
  {
    if (!use_absolute_visual_correction_) {
      return;
    }
    if (message.header.frame_id != map_frame_ || message.child_frame_id != base_frame_ ||
      !valid_pose(message.pose.pose))
    {
      return;
    }

    const rclcpp::Time visual_stamp(message.header.stamp);
    if (visual_stamp.nanoseconds() <= 0) {
      ++unsynchronized_visual_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting visual odometry with an unset measurement timestamp");
      return;
    }
    Pose2d local_pose;
    double local_delta = std::numeric_limits<double>::infinity();
    if (!synchronized_local_pose(visual_stamp, local_pose, local_delta)) {
      if (!local_history_.empty() && visual_stamp > local_history_.back().stamp &&
        local_delta <= synchronization_tolerance_)
      {
        pending_visual_ = message;
        return;
      }
      pending_visual_.reset();
      ++unsynchronized_visual_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting unsynchronized visual/local pose (delta %.3f s)", local_delta);
      return;
    }
    pending_visual_.reset();

    const double visual_arrival = steady_seconds();
    const double recovery_gap = std::max(visual_recovery_gap_, visual_timeout_);
    const bool visual_recovered = correction_valid_ && visual_stamp_received_ &&
      (visual_stamp - last_visual_stamp_).seconds() > recovery_gap;
    latest_raw_visual_ = message_pose(message);
    latest_visual_local_ = local_pose;
    visual_xy_variance_ = finite_covariance(message.pose.covariance[0], 0.05);
    visual_yaw_variance_ = finite_covariance(message.pose.covariance[35], 0.10);
    visual_pair_valid_ = true;
    visual_received_at_ = visual_arrival;
    last_visual_stamp_ = visual_stamp;
    visual_stamp_received_ = true;

    if (rfid_correction_active_) {
      // RFID owns map position while the vehicle is held. Keep the newest
      // synchronized visual pair for re-anchoring, but do not compete with it.
      desired_valid_ = false;
      return;
    }

    if (!visual_pose_aligner_.initialized()) {
      // ORB-SLAM's first map has an arbitrary planar yaw and origin. Align
      // that first visual pose to the start-relative map frame: the car starts
      // at the map origin and its forward axis is map +X. The local odom frame
      // remains zeroed at the same vehicle pose.
      visual_pose_aligner_.align_to(
        latest_raw_visual_, {initial_map_x_, initial_map_y_, initial_map_yaw_});
    } else if (rfid_visual_reanchor_pending_) {
      reanchor_visual(
        latest_raw_visual_, latest_visual_local_, visual_arrival,
        !visual_recovered);
    } else if (visual_recovered) {
      // ORB may restart its source origin after tracking loss. Keep the
      // vehicle at the locally propagated global pose on the first recovered
      // sample, then apply only subsequent visual increments. Treating the
      // recovered raw pose as an absolute correction can otherwise drag
      // map->odom back toward the ORB origin while the vehicle is moving.
      const Pose2d recovery_anchor = fused_odometry::compose_pose(
        map_from_odom_, latest_visual_local_);
      visual_pose_aligner_.align_to(latest_raw_visual_, recovery_anchor);
      visual_translation_aligner_.clear();
      RCLCPP_WARN(
        get_logger(),
        "Visual tracking recovered; rebasing raw visual pose without jumping map->odom");
      visual_recovery_blend_until_ = visual_arrival + visual_recovery_blend_duration_;
    }
    if (turn_hold_active_) {
      return;
    }
    const Pose2d aligned_visual = visual_translation_aligner_.apply(
      visual_pose_aligner_.apply(latest_raw_visual_));
    desired_map_from_odom_ = fused_odometry::compose_pose(
      aligned_visual,
      fused_odometry::inverse_pose(latest_visual_local_));
    desired_valid_ = true;
    if (!correction_valid_) {
      map_from_odom_ = desired_map_from_odom_;
      correction_valid_ = true;
      last_update_at_ = visual_received_at_;
    } else if (visual_recovered) {
      visual_recovery_blend_until_ = visual_arrival + visual_recovery_blend_duration_;
    }
  }

  void reanchor_visual(
    const Pose2d & raw_visual, const Pose2d & synchronized_local,
    double current_time, bool position_only)
  {
    const Pose2d corrected_global = fused_odometry::compose_pose(
      map_from_odom_, synchronized_local);
    if (position_only) {
      visual_translation_aligner_.align_to(
        visual_pose_aligner_.apply(raw_visual), corrected_global);
    } else {
      visual_pose_aligner_.align_to(raw_visual, corrected_global);
      visual_translation_aligner_.clear();
    }
    desired_map_from_odom_ = map_from_odom_;
    desired_valid_ = true;
    rfid_visual_reanchor_pending_ = false;
    visual_recovery_blend_until_ = current_time + visual_recovery_blend_duration_;
    RCLCPP_INFO(
      get_logger(),
      "RFID_VISUAL_REANCHORED at map position (%.3f, %.3f)",
      corrected_global.x, corrected_global.y);
  }

  void finish_rfid_correction(double current_time)
  {
    rfid_correction_active_ = false;
    rfid_visual_reanchor_pending_ = use_absolute_visual_correction_;
    if (rfid_visual_reanchor_pending_ && visual_pair_valid_ &&
      current_time - visual_received_at_ <= visual_timeout_)
    {
      reanchor_visual(latest_raw_visual_, latest_visual_local_, current_time, true);
    }
    RCLCPP_INFO(get_logger(), "RFID map correction completed");
  }

  bool local_motion_is_stationary() const
  {
    const auto & twist = latest_local_.twist.twist;
    const double linear_speed = std::hypot(twist.linear.x, twist.linear.y);
    return std::isfinite(linear_speed) && std::isfinite(twist.angular.z) &&
      linear_speed <= stationary_max_linear_speed_ &&
      std::abs(twist.angular.z) <= stationary_max_angular_speed_;
  }

  void update_turn_hold(double current_time)
  {
    if (!use_absolute_visual_correction_ || !hold_global_xy_during_turn_) {
      return;
    }
    const auto & twist = latest_local_.twist.twist;
    const bool low_linear_speed =
      std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
      std::hypot(twist.linear.x, twist.linear.y) <= turn_hold_max_linear_speed_;
    const double absolute_angular_speed =
      std::isfinite(twist.angular.z) ? std::abs(twist.angular.z) : 0.0;
    if (turn_hold_motion_rejected_) {
      if (!low_linear_speed || absolute_angular_speed < turn_hold_min_angular_speed_) {
        turn_hold_motion_rejected_ = false;
      } else {
        return;
      }
    }
    if (turn_hold_active_) {
      if (low_linear_speed && absolute_angular_speed >= turn_hold_min_angular_speed_) {
        last_turn_motion_at_ = current_time;
      }
      return;
    }
    if (!low_linear_speed ||
      absolute_angular_speed < turn_hold_entry_angular_speed_ ||
      !local_received_ || !correction_valid_)
    {
      return;
    }

    turn_anchor_global_ = fused_odometry::compose_pose(
      map_from_odom_, message_pose(latest_local_));
    turn_anchor_local_ = message_pose(latest_local_);
    turn_hold_active_ = true;
    last_turn_motion_at_ = current_time;
    desired_valid_ = false;
    RCLCPP_INFO(
      get_logger(), "Holding global XY during observed in-place turn at (%.3f, %.3f)",
      turn_anchor_global_.x, turn_anchor_global_.y);
  }

  void fusion_status_callback(const std_msgs::msg::String::SharedPtr message)
  {
    const bool was_dead_reckoning =
      fusion_status_authority_.status() == "DEGRADED_DEAD_RECKONING";
    fusion_status_authority_.update(message->data);
    const bool is_dead_reckoning = message->data == "DEGRADED_DEAD_RECKONING";
    if (is_dead_reckoning && !was_dead_reckoning) {
      dead_reckoning_started_at_ = steady_seconds();
    } else if (!is_dead_reckoning) {
      dead_reckoning_started_at_.reset();
    }
  }

  void finish_turn_hold()
  {
    if (visual_pair_valid_) {
      const Pose2d current_global = fused_odometry::compose_pose(
        map_from_odom_, latest_visual_local_);
      visual_translation_aligner_.align_to(
        visual_pose_aligner_.apply(latest_raw_visual_), current_global);
      const Pose2d aligned_visual = visual_translation_aligner_.apply(
        visual_pose_aligner_.apply(latest_raw_visual_));
      desired_map_from_odom_ = fused_odometry::compose_pose(
        aligned_visual,
        fused_odometry::inverse_pose(latest_visual_local_));
      desired_valid_ = true;
    }
    turn_hold_active_ = false;
    RCLCPP_INFO(
      get_logger(), "Released global XY turn hold at (%.3f, %.3f)",
      turn_anchor_global_.x, turn_anchor_global_.y);
  }

  void map_change_callback(const std_msgs::msg::UInt64::SharedPtr message)
  {
    // Keep the startup arena alignment through ORB loop corrections. The raw
    // pose jump is the global correction we want to smooth into map->odom;
    // resetting here would erase that correction.
    map_change_sequence_ = message->data;
    map_change_active_until_ = steady_seconds() + map_change_smoothing_window_;
    RCLCPP_INFO(
      get_logger(), "Smoothing ORB map correction sequence %lu into map->odom",
      static_cast<unsigned long>(map_change_sequence_));
  }

  void correct_map_position(
    const mission_control_interfaces::srv::CorrectMapPosition::Request::SharedPtr request,
    mission_control_interfaces::srv::CorrectMapPosition::Response::SharedPtr response)
  {
    if (request->source != "navigation_mission_manager/rfid") {
      response->accepted = false;
      response->message = "only the mission manager RFID workflow may request correction";
      return;
    }
    if (!rfid_hold_active_) {
      response->accepted = false;
      response->message = "RFID motion hold is not active";
      return;
    }
    if (!std::isfinite(request->x) || !std::isfinite(request->y) ||
      !local_received_ || !correction_valid_)
    {
      response->accepted = false;
      response->message = "map correction pose is unavailable or invalid";
      return;
    }
    const Pose2d local_pose = message_pose(latest_local_);
    const Pose2d current_global = fused_odometry::compose_pose(map_from_odom_, local_pose);
    rfid_target_map_from_odom_ = map_from_odom_;
    rfid_target_map_from_odom_.x += request->x - current_global.x;
    rfid_target_map_from_odom_.y += request->y - current_global.y;
    // RFID supplies position only. Preserve the current map/odom yaw.
    rfid_target_map_from_odom_.yaw = map_from_odom_.yaw;
    rfid_correction_active_ = true;
    rfid_visual_reanchor_pending_ = use_absolute_visual_correction_;
    desired_valid_ = false;
    turn_hold_active_ = false;
    turn_hold_motion_rejected_ = false;
    RCLCPP_INFO(
      get_logger(), "Mission manager position correction target: (%.3f, %.3f)",
      request->x, request->y);
    response->accepted = true;
    response->message = "position correction accepted";
  }

  void publish()
  {
    const double current_time = steady_seconds();
    if (!local_received_ || current_time - local_received_at_ > local_timeout_ ||
      !correction_valid_)
    {
      return;
    }

    if (use_absolute_visual_correction_) {
      update_turn_hold(current_time);
    }

    if (use_absolute_visual_correction_ && turn_hold_active_ &&
      (current_time - last_turn_motion_at_ >= turn_hold_release_delay_ ||
      std::hypot(
        message_pose(latest_local_).x - turn_anchor_local_.x,
        message_pose(latest_local_).y - turn_anchor_local_.y) > turn_hold_max_translation_))
    {
      const Pose2d current_local = message_pose(latest_local_);
      const double translation = std::hypot(
        current_local.x - turn_anchor_local_.x,
        current_local.y - turn_anchor_local_.y);
      if (translation > turn_hold_max_translation_) {
        turn_hold_motion_rejected_ = true;
        RCLCPP_WARN(
          get_logger(), "Releasing turn hold after %.3f m local translation", translation);
      }
      finish_turn_hold();
    }

    const auto authority_time = fused_odometry::FusionStatusAuthority::Clock::now();
    const bool visual_correction_active = use_absolute_visual_correction_ &&
      !rfid_correction_active_ && !turn_hold_active_ && desired_valid_ &&
      current_time - visual_received_at_ <= visual_timeout_ &&
      fusion_status_authority_.visual_correction_allowed(
        authority_time, fusion_status_timeout_);
    if (visual_correction_active)
    {
      const double elapsed = std::max(0.0, current_time - last_update_at_);
      const bool recovery_blend = current_time <= visual_recovery_blend_until_;
      const bool stationary = local_motion_is_stationary();
      // Never publish raw visual corrections directly. Even in the legacy
      // direct_visual_tracking mode, interpolate the map correction so
      // keyframe and feature-tracking jitter cannot pass into fused odometry.
      double time_constant = recovery_blend ?
        visual_recovery_time_constant_ : (stationary ?
        stationary_correction_time_constant_ : correction_time_constant_);
      double max_correction_step = stationary ?
        max_correction_step_ : moving_max_correction_step_;
      double max_correction_yaw_step = stationary ?
        max_correction_yaw_step_ : moving_max_correction_yaw_step_;
      if (recovery_blend) {
        max_correction_step = std::min(
          max_correction_step, visual_recovery_max_correction_step_);
        max_correction_yaw_step = std::min(
          max_correction_yaw_step, visual_recovery_max_correction_yaw_step_);
      }
      if (current_time <= map_change_active_until_) {
        time_constant = loop_correction_time_constant_;
      }
      const double fraction = 1.0 - std::exp(-elapsed / time_constant);
      const Pose2d proposed_map_from_odom = fused_odometry::interpolate_pose(
        map_from_odom_, desired_map_from_odom_, fraction);
      map_from_odom_ = limit_pose_step(
        map_from_odom_, proposed_map_from_odom,
        max_correction_step, max_correction_yaw_step);
    }
    if (rfid_correction_active_) {
      const double elapsed = std::max(0.0, current_time - last_update_at_);
      const double fraction = 1.0 - std::exp(
        -elapsed / rfid_correction_time_constant_);
      const Pose2d proposed = fused_odometry::interpolate_pose(
        map_from_odom_, rfid_target_map_from_odom_, fraction);
      map_from_odom_ = limit_pose_step(
        map_from_odom_, proposed, rfid_correction_max_step_, 0.0);
      if (std::hypot(
          map_from_odom_.x - rfid_target_map_from_odom_.x,
          map_from_odom_.y - rfid_target_map_from_odom_.y) < 0.005)
      {
        map_from_odom_.x = rfid_target_map_from_odom_.x;
        map_from_odom_.y = rfid_target_map_from_odom_.y;
        finish_rfid_correction(current_time);
      }
    }
    last_update_at_ = current_time;

    const Pose2d local_pose = message_pose(latest_local_);
    Pose2d global_pose = fused_odometry::compose_pose(map_from_odom_, local_pose);
    if (turn_hold_active_) {
      const double dx = global_pose.x - turn_anchor_global_.x;
      const double dy = global_pose.y - turn_anchor_global_.y;
      const double translation = std::hypot(dx, dy);
      if (translation > turn_hold_max_translation_) {
        const double scale = turn_hold_max_translation_ / translation;
        global_pose.x = turn_anchor_global_.x + dx * scale;
        global_pose.y = turn_anchor_global_.y + dy * scale;
      }
      map_from_odom_ = fused_odometry::compose_pose(
        global_pose, fused_odometry::inverse_pose(local_pose));
    }
    rclcpp::Time publish_stamp = now();
    if (last_output_stamp_.nanoseconds() > 0 && publish_stamp <= last_output_stamp_) {
      publish_stamp = rclcpp::Time(
        last_output_stamp_.nanoseconds() + 1, publish_stamp.get_clock_type());
    }
    last_output_stamp_ = publish_stamp;

    nav_msgs::msg::Odometry output = latest_local_;
    output.header.stamp = publish_stamp;
    output.header.frame_id = map_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = global_pose.x;
    output.pose.pose.position.y = global_pose.y;
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation = yaw_quaternion(global_pose.yaw);
    // The pose is in map while latest_local_ was estimated in odom. Include
    // both state uncertainties instead of relabeling local covariance as map
    // covariance after applying the visual correction.
    output.pose.covariance.fill(0.0);
    const double visual_xy_contribution =
      visual_correction_active && visual_pair_valid_ ? visual_xy_variance_ : 0.0;
    const double visual_yaw_contribution =
      visual_correction_active && visual_pair_valid_ ? visual_yaw_variance_ : 0.0;
    const double dead_reckoning_seconds = dead_reckoning_started_at_.has_value() ?
      std::max(0.0, current_time - *dead_reckoning_started_at_) : 0.0;
    const double dead_reckoning_xy_contribution =
      dead_reckoning_seconds * dead_reckoning_xy_variance_rate_;
    const double dead_reckoning_yaw_contribution =
      dead_reckoning_seconds * dead_reckoning_yaw_variance_rate_;
    output.pose.covariance[0] = finite_covariance(latest_local_.pose.covariance[0], 0.0) +
      visual_xy_contribution + dead_reckoning_xy_contribution;
    output.pose.covariance[7] = finite_covariance(latest_local_.pose.covariance[7], 0.0) +
      visual_xy_contribution + dead_reckoning_xy_contribution;
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = finite_covariance(latest_local_.pose.covariance[35], 0.0) +
      visual_yaw_contribution + dead_reckoning_yaw_contribution;
    publisher_->publish(output);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = odom_frame_;
      transform.transform.translation.x = map_from_odom_.x;
      transform.transform.translation.y = map_from_odom_.y;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = yaw_quaternion(map_from_odom_.yaw);
      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::string visual_topic_;
  std::string local_topic_;
  std::string fusion_status_topic_;
  std::string output_topic_;
  std::string map_change_topic_;
  std::string correct_position_service_;
  std::string rfid_hold_state_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool use_absolute_visual_correction_{true};
  bool publish_tf_{true};
  bool hold_global_xy_during_turn_{false};
  double publish_frequency_{30.0};
  double visual_recovery_gap_{0.60};
  double visual_recovery_time_constant_{0.25};
  double visual_recovery_blend_duration_{0.80};
  double correction_time_constant_{0.15};
  double stationary_correction_time_constant_{0.08};
  double loop_correction_time_constant_{1.25};
  double max_correction_step_{0.03};
  double max_correction_yaw_step_{0.03};
  double moving_max_correction_step_{0.03};
  double moving_max_correction_yaw_step_{0.03};
  double visual_recovery_max_correction_step_{0.03};
  double visual_recovery_max_correction_yaw_step_{0.03};
  double map_change_smoothing_window_{2.0};
  double synchronization_tolerance_{0.12};
  double local_timeout_{0.50};
  double visual_timeout_{0.60};
  double fusion_status_timeout_{0.80};
  double stationary_max_linear_speed_{0.03};
  double stationary_max_angular_speed_{0.12};
  double turn_hold_max_linear_speed_{0.05};
  double turn_hold_max_translation_{0.04};
  double turn_hold_entry_angular_speed_{0.55};
  double turn_hold_min_angular_speed_{0.12};
  double turn_hold_release_delay_{0.28};
  double rfid_correction_time_constant_{0.15};
  double rfid_correction_max_step_{0.10};
  double dead_reckoning_xy_variance_rate_{0.0025};
  double dead_reckoning_yaw_variance_rate_{0.0010};

  std::deque<TimedLocalPose> local_history_;
  std::optional<nav_msgs::msg::Odometry> pending_visual_;
  nav_msgs::msg::Odometry latest_local_;
  Pose2d desired_map_from_odom_;
  Pose2d map_from_odom_;
  Pose2d latest_raw_visual_;
  Pose2d latest_visual_local_;
  double visual_xy_variance_{0.05};
  double visual_yaw_variance_{0.10};
  Pose2d turn_anchor_global_;
  Pose2d turn_anchor_local_;
  fused_odometry::PoseAligner visual_pose_aligner_;
  fused_odometry::TranslationAligner visual_translation_aligner_;
  fused_odometry::FusionStatusAuthority fusion_status_authority_;
  bool local_received_{false};
  bool desired_valid_{false};
  bool correction_valid_{false};
  bool visual_pair_valid_{false};
  bool visual_stamp_received_{false};
  bool turn_hold_active_{false};
  bool turn_hold_motion_rejected_{false};
  double local_received_at_{0.0};
  double visual_received_at_{0.0};
  rclcpp::Time last_visual_stamp_{0, 0, RCL_ROS_TIME};
  double visual_recovery_blend_until_{0.0};
  double last_update_at_{0.0};
  double map_change_active_until_{0.0};
  double last_turn_motion_at_{0.0};
  uint64_t map_change_sequence_{0};
  uint64_t unsynchronized_visual_count_{0};
  bool rfid_hold_active_{false};
  bool rfid_correction_active_{false};
  bool rfid_visual_reanchor_pending_{false};
  std::optional<double> dead_reckoning_started_at_;
  Pose2d rfid_target_map_from_odom_;
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};
  double initial_map_x_{0.0};
  double initial_map_y_{0.0};
  double initial_map_yaw_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fusion_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr map_change_subscription_;
  rclcpp::Subscription<mission_control_interfaces::msg::MotionHoldState>::SharedPtr
    rfid_hold_subscription_;
  rclcpp::Service<mission_control_interfaces::srv::CorrectMapPosition>::SharedPtr
    correct_position_service_server_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomCorrectionNode>());
  rclcpp::shutdown();
  return 0;
}
