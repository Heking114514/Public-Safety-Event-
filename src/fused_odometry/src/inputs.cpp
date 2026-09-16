#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

bool FusionGateNode::tracking_fresh_and_good() const {
  return tracking_.good && tracking_.input.fresh(tracking_timeout_);
}

bool FusionGateNode::vision_healthy() const {
  return visual_.accepted && tracking_fresh_and_good() &&
         visual_.input.fresh(visual_timeout_);
}

bool FusionGateNode::wheel_healthy() const {
  const MotionFault motion_fault = health_monitor_.motion_fault();
  const bool classified_bad = motion_fault == MotionFault::kSlip ||
                              motion_fault == MotionFault::kEncoderFailure;
  return wheel_.input.fresh(wheel_timeout_) && !wheel_gate_.rejected() &&
         !classified_bad;
}

bool FusionGateNode::imu_healthy() const {
  return imu_.input.fresh(imu_timeout_);
}

bool FusionGateNode::raw_visual_increment_healthy() const {
  return raw_visual_.velocity_valid && tracking_fresh_and_good() &&
         raw_visual_.input.fresh(raw_visual_timeout_) &&
         raw_visual_vx_window_.ready(robust_min_samples_);
}

void FusionGateNode::tracking_callback(
    const std_msgs::msg::Int32::SharedPtr message) {
  tracking_.input.update();
  tracking_.good = message->data == 2 || message->data == 5;
  tracking_.code = message->data;
  if (!tracking_.good) {
    mark_visual_interrupted();
  }
}

void FusionGateNode::map_change_callback(
    const std_msgs::msg::UInt64::SharedPtr message) {
  map_change_.sequence = message->data;
  map_change_.grace_until = steady_seconds() + map_change_grace_;
  map_change_.pending = true;
  RCLCPP_INFO(get_logger(), "Accepting gated ORB map correction sequence %lu",
              static_cast<unsigned long>(map_change_.sequence));
}

} // namespace fused_odometry
