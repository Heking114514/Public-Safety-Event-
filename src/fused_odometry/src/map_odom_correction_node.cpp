#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <stdexcept>

#include "fused_odometry/fusion_logic.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
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
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/fused");
    map_change_topic_ = declare_parameter<std::string>(
      "map_change_topic", "/orbslam3/map_change");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_frequency_ = positive("publish_frequency", 30.0);
    correction_time_constant_ = positive("correction_time_constant_s", 0.75);
    loop_correction_time_constant_ = positive("loop_correction_time_constant_s", 1.25);
    map_change_smoothing_window_ = positive("map_change_smoothing_window_s", 2.0);
    synchronization_tolerance_ = positive("synchronization_tolerance_s", 0.12);
    local_timeout_ = positive("local_timeout_s", 0.50);
    visual_timeout_ = positive("visual_timeout_s", 0.60);
    turn_hold_max_linear_speed_ = positive("turn_hold_max_linear_speed_mps", 0.03);
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
    map_change_subscription_ = create_subscription<std_msgs::msg::UInt64>(
      map_change_topic_, 10,
      std::bind(&MapOdomCorrectionNode::map_change_callback, this, std::placeholders::_1));
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(10).reliable());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_frequency_),
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
  }

  void visual_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_ || message->child_frame_id != base_frame_ ||
      !valid_pose(message->pose.pose) || local_history_.empty())
    {
      return;
    }

    const rclcpp::Time visual_stamp(message->header.stamp);
    const TimedLocalPose * closest = nullptr;
    double closest_delta = std::numeric_limits<double>::infinity();
    for (const auto & candidate : local_history_) {
      const double delta = std::abs((candidate.stamp - visual_stamp).seconds());
      if (delta < closest_delta) {
        closest = &candidate;
        closest_delta = delta;
      }
    }
    if (closest == nullptr || closest_delta > synchronization_tolerance_) {
      ++unsynchronized_visual_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for synchronized local odometry (visual/local delta %.3f s)", closest_delta);
      return;
    }

    latest_raw_visual_ = message_pose(*message);
    latest_visual_local_ = closest->pose;
    visual_pair_valid_ = true;
    visual_received_at_ = steady_seconds();
    if (turn_hold_active_) {
      return;
    }

    desired_map_from_odom_ = fused_odometry::compose_pose(
      visual_translation_aligner_.apply(latest_raw_visual_),
      fused_odometry::inverse_pose(latest_visual_local_));
    desired_valid_ = true;
    if (!correction_valid_) {
      map_from_odom_ = desired_map_from_odom_;
      correction_valid_ = true;
      last_update_at_ = visual_received_at_;
    }
  }

  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      return;
    }
    const double current_time = steady_seconds();
    const bool low_linear_speed =
      std::abs(message->linear.x) <= turn_hold_max_linear_speed_;
    const double absolute_angular_speed = std::abs(message->angular.z);
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
    turn_hold_active_ = true;
    last_turn_command_at_ = current_time;
    desired_valid_ = false;
    RCLCPP_INFO(
      get_logger(), "Holding global XY during in-place turn at (%.3f, %.3f)",
      turn_anchor_global_.x, turn_anchor_global_.y);
  }

  void finish_turn_hold()
  {
    if (visual_pair_valid_) {
      visual_translation_aligner_.align_to(latest_raw_visual_, turn_anchor_global_);
      desired_map_from_odom_ = fused_odometry::compose_pose(
        visual_translation_aligner_.apply(latest_raw_visual_),
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
    visual_translation_aligner_.clear();
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
      current_time - last_turn_command_at_ >= turn_hold_release_delay_)
    {
      finish_turn_hold();
    }

    if (!turn_hold_active_ && desired_valid_ &&
      current_time - visual_received_at_ <= visual_timeout_)
    {
      const double elapsed = std::max(0.0, current_time - last_update_at_);
      const double time_constant = current_time <= map_change_active_until_ ?
        loop_correction_time_constant_ : correction_time_constant_;
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
  std::string output_topic_;
  std::string map_change_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  double publish_frequency_{30.0};
  double correction_time_constant_{0.75};
  double loop_correction_time_constant_{1.25};
  double map_change_smoothing_window_{2.0};
  double synchronization_tolerance_{0.12};
  double local_timeout_{0.50};
  double visual_timeout_{0.60};
  double turn_hold_max_linear_speed_{0.03};
  double turn_hold_entry_angular_speed_{0.55};
  double turn_hold_min_angular_speed_{0.12};
  double turn_hold_release_delay_{0.28};

  std::deque<TimedLocalPose> local_history_;
  nav_msgs::msg::Odometry latest_local_;
  Pose2d desired_map_from_odom_;
  Pose2d map_from_odom_;
  Pose2d latest_raw_visual_;
  Pose2d latest_visual_local_;
  Pose2d turn_anchor_global_;
  fused_odometry::TranslationAligner visual_translation_aligner_;
  bool local_received_{false};
  bool desired_valid_{false};
  bool correction_valid_{false};
  bool visual_pair_valid_{false};
  bool turn_hold_active_{false};
  double local_received_at_{0.0};
  double visual_received_at_{0.0};
  double last_update_at_{0.0};
  double map_change_active_until_{0.0};
  double last_turn_command_at_{0.0};
  uint64_t map_change_sequence_{0};
  uint64_t unsynchronized_visual_count_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr visual_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
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
