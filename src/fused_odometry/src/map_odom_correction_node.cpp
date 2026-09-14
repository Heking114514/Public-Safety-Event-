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

#include "fused_odometry/fusion_logic.hpp"
#include "fused_odometry/fusion_status_authority.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    command_topic_ = declare_parameter<std::string>("command_topic", "/cmd_vel_nav");
    fusion_status_topic_ = declare_parameter<std::string>(
      "fusion_status_topic", "/odometry/fusion_status");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/fused");
    map_change_topic_ = declare_parameter<std::string>(
      "map_change_topic", "/orbslam3/map_change");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    initial_map_x_ = declare_parameter<double>("initial_map_x", 0.0);
    initial_map_y_ = declare_parameter<double>("initial_map_y", 0.0);
    initial_map_yaw_ = declare_parameter<double>("initial_map_yaw", 0.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    const bool deprecated_direct_visual_tracking =
      declare_parameter<bool>("direct_visual_tracking", false);
    if (deprecated_direct_visual_tracking) {
      RCLCPP_WARN(
        get_logger(), "direct_visual_tracking is deprecated; visual corrections remain smoothed");
    }
    visual_recovery_gap_ = positive("visual_recovery_gap_s", 0.10);
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
    map_change_smoothing_window_ = positive("map_change_smoothing_window_s", 2.0);
    synchronization_tolerance_ = positive("synchronization_tolerance_s", 0.12);
    local_timeout_ = positive("local_timeout_s", 0.50);
    visual_timeout_ = positive("visual_timeout_s", 0.60);
    command_timeout_ = positive("command_timeout_s", 0.40);
    fusion_status_timeout_ = positive("fusion_status_timeout_s", 0.80);
    stationary_max_linear_speed_ = positive("stationary_max_linear_speed_mps", 0.03);
    stationary_max_angular_speed_ = positive("stationary_max_angular_speed_radps", 0.12);
    turn_hold_max_linear_speed_ = positive("turn_hold_max_linear_speed_mps", 0.03);
    turn_hold_max_translation_ = positive("turn_hold_max_translation_m", 0.04);
    turn_hold_entry_angular_speed_ = positive("turn_hold_entry_angular_speed_radps", 0.55);
    turn_hold_min_angular_speed_ = positive("turn_hold_min_angular_speed_radps", 0.12);
    turn_hold_release_delay_ = positive("turn_hold_release_delay_s", 0.28);
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
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, 10,
      std::bind(&MapOdomCorrectionNode::command_callback, this, std::placeholders::_1));
    fusion_status_subscription_ = create_subscription<std_msgs::msg::String>(
      fusion_status_topic_, rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&MapOdomCorrectionNode::fusion_status_callback, this, std::placeholders::_1));
    map_change_subscription_ = create_subscription<std_msgs::msg::UInt64>(
      map_change_topic_, 10,
      std::bind(&MapOdomCorrectionNode::map_change_callback, this, std::placeholders::_1));
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
    const bool visual_recovered = correction_valid_ && visual_stamp_received_ &&
      (visual_stamp - last_visual_stamp_).seconds() > visual_recovery_gap_;
    latest_raw_visual_ = message_pose(message);
    latest_visual_local_ = local_pose;
    visual_xy_variance_ = finite_covariance(message.pose.covariance[0], 0.05);
    visual_yaw_variance_ = finite_covariance(message.pose.covariance[35], 0.10);
    visual_pair_valid_ = true;
    visual_received_at_ = visual_arrival;
    last_visual_stamp_ = visual_stamp;
    visual_stamp_received_ = true;
    if (turn_hold_active_) {
      return;
    }

    if (!visual_pose_aligner_.initialized()) {
      // ORB-SLAM's first map has an arbitrary planar yaw and origin. Align
      // that first visual pose to the start-relative map frame: the car starts
      // at the map origin and its forward axis is map +X. The local odom frame
      // remains zeroed at the same vehicle pose.
      visual_pose_aligner_.align_to(
        latest_raw_visual_, {initial_map_x_, initial_map_y_, initial_map_yaw_});
    } else if (visual_recovered) {
      // ORB may restart its source origin after tracking loss. Keep the
      // vehicle at the locally propagated global pose on the first recovered
      // sample, then apply only subsequent visual increments. Treating the
      // recovered raw pose as an absolute correction can otherwise drag
      // map->odom back toward the ORB origin while the vehicle is moving.
      const Pose2d recovery_anchor = fused_odometry::compose_pose(
        map_from_odom_, latest_visual_local_);
      visual_pose_aligner_.align_to(latest_raw_visual_, recovery_anchor);
      RCLCPP_WARN(
        get_logger(),
        "Visual tracking recovered; rebasing raw visual pose without jumping map->odom");
    }
    const Pose2d aligned_visual = visual_pose_aligner_.apply(latest_raw_visual_);
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

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      return;
    }
    const double current_time = steady_seconds();
    latest_command_linear_ = message->linear.x;
    latest_command_angular_ = message->angular.z;
    command_received_at_ = current_time;
    command_received_ = true;
    if (!hold_global_xy_during_turn_) {
      return;
    }
    const bool low_linear_speed =
      std::abs(message->linear.x) <= turn_hold_max_linear_speed_;
    const double absolute_angular_speed = std::abs(message->angular.z);
    if (turn_hold_motion_rejected_) {
      if (!low_linear_speed || absolute_angular_speed < turn_hold_min_angular_speed_) {
        turn_hold_motion_rejected_ = false;
      } else {
        return;
      }
    }
    if (turn_hold_active_) {
      if (low_linear_speed && absolute_angular_speed >= turn_hold_min_angular_speed_) {
        last_turn_command_at_ = current_time;
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
    last_turn_command_at_ = current_time;
    desired_valid_ = false;
    RCLCPP_INFO(
      get_logger(), "Holding global XY during in-place turn at (%.3f, %.3f)",
      turn_anchor_global_.x, turn_anchor_global_.y);
  }

  void fusion_status_callback(const std_msgs::msg::String::SharedPtr message)
  {
    fusion_status_authority_.update(message->data);
  }

  void finish_turn_hold()
  {
    if (visual_pair_valid_) {
      // The hold only prevents a transient correction while the chassis is
      // rotating. Never move the visual origin here: doing so would turn every
      // encoder/command-classified turn into a permanent global position bias.
      const Pose2d aligned_visual = visual_pose_aligner_.apply(latest_raw_visual_);
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

  void publish()
  {
    const double current_time = steady_seconds();
    if (!local_received_ || current_time - local_received_at_ > local_timeout_ ||
      !correction_valid_)
    {
      return;
    }

    if (turn_hold_active_ &&
      (current_time - last_turn_command_at_ >= turn_hold_release_delay_ ||
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
    if (!turn_hold_active_ && desired_valid_ &&
      current_time - visual_received_at_ <= visual_timeout_ &&
      fusion_status_authority_.visual_correction_allowed(
        authority_time, fusion_status_timeout_))
    {
      const double elapsed = std::max(0.0, current_time - last_update_at_);
      const bool command_fresh = command_received_ &&
        current_time - command_received_at_ <= command_timeout_;
      const bool command_stationary = fused_odometry::motion_command_is_stationary(
        latest_command_linear_, latest_command_angular_, command_fresh,
        stationary_max_linear_speed_, stationary_max_angular_speed_);
      const bool recovery_blend = current_time <= visual_recovery_blend_until_;
      // Never publish raw visual corrections directly. Even in the legacy
      // direct_visual_tracking mode, interpolate the map correction so
      // keyframe and feature-tracking jitter cannot pass into fused odometry.
      double time_constant = recovery_blend ?
        visual_recovery_time_constant_ : (command_stationary ?
        stationary_correction_time_constant_ : correction_time_constant_);
      if (current_time <= map_change_active_until_) {
        time_constant = loop_correction_time_constant_;
      }
      const double fraction = 1.0 - std::exp(-elapsed / time_constant);
      map_from_odom_ = fused_odometry::interpolate_pose(
        map_from_odom_, desired_map_from_odom_, fraction);
    }
    last_update_at_ = current_time;

    Pose2d global_pose = fused_odometry::compose_pose(
      map_from_odom_, message_pose(latest_local_));
    if (turn_hold_active_) {
      global_pose.x = turn_anchor_global_.x;
      global_pose.y = turn_anchor_global_.y;
      map_from_odom_ = fused_odometry::compose_pose(
        global_pose, fused_odometry::inverse_pose(message_pose(latest_local_)));
    }
    nav_msgs::msg::Odometry output = latest_local_;
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
    output.pose.covariance[0] = finite_covariance(latest_local_.pose.covariance[0], 0.0) +
      visual_xy_variance_;
    output.pose.covariance[7] = finite_covariance(latest_local_.pose.covariance[7], 0.0) +
      visual_xy_variance_;
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = finite_covariance(latest_local_.pose.covariance[35], 0.0) +
      visual_yaw_variance_;
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
  std::string command_topic_;
  std::string fusion_status_topic_;
  std::string output_topic_;
  std::string map_change_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  bool hold_global_xy_during_turn_{false};
  double publish_frequency_{30.0};
  double visual_recovery_gap_{0.10};
  double visual_recovery_time_constant_{0.25};
  double visual_recovery_blend_duration_{0.80};
  double correction_time_constant_{0.15};
  double stationary_correction_time_constant_{0.08};
  double loop_correction_time_constant_{1.25};
  double map_change_smoothing_window_{2.0};
  double synchronization_tolerance_{0.12};
  double local_timeout_{0.50};
  double visual_timeout_{0.60};
  double command_timeout_{0.40};
  double fusion_status_timeout_{0.80};
  double stationary_max_linear_speed_{0.03};
  double stationary_max_angular_speed_{0.12};
  double turn_hold_max_linear_speed_{0.03};
  double turn_hold_max_translation_{0.04};
  double turn_hold_entry_angular_speed_{0.55};
  double turn_hold_min_angular_speed_{0.12};
  double turn_hold_release_delay_{0.28};

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
  fused_odometry::FusionStatusAuthority fusion_status_authority_;
  bool local_received_{false};
  bool desired_valid_{false};
  bool correction_valid_{false};
  bool visual_pair_valid_{false};
  bool visual_stamp_received_{false};
  bool turn_hold_active_{false};
  bool turn_hold_motion_rejected_{false};
  bool command_received_{false};
  double local_received_at_{0.0};
  double visual_received_at_{0.0};
  rclcpp::Time last_visual_stamp_{0, 0, RCL_ROS_TIME};
  double visual_recovery_blend_until_{0.0};
  double last_update_at_{0.0};
  double map_change_active_until_{0.0};
  double last_turn_command_at_{0.0};
  double command_received_at_{0.0};
  double latest_command_linear_{0.0};
  double latest_command_angular_{0.0};
  uint64_t map_change_sequence_{0};
  uint64_t unsynchronized_visual_count_{0};
  double initial_map_x_{0.0};
  double initial_map_y_{0.0};
  double initial_map_yaw_{0.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fusion_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr map_change_subscription_;
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
