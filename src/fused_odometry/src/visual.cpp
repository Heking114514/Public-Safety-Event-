#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

void FusionGateNode::reject_raw_visual_sample() {
  raw_visual_.input.received = false;
  raw_visual_.velocity_valid = false;
  raw_visual_.increment_pose_valid = false;
  // An invalid frame is an interruption just like a tracking-state loss.
  // This clears the recovery counter so isolated valid samples cannot
  // accumulate across an invalid or stale interval.
  if (visual_.ever_accepted)
    mark_visual_interrupted();
}

void FusionGateNode::mark_visual_interrupted() {
  const bool already_interrupted = visual_.interrupted;
  visual_.interrupted = true;
  visual_.accepted = false;
  visual_.velocity_valid = false;
  visual_.forward_velocity = 0.0;
  visual_.yaw_rate = 0.0;
  imu_.visual_residual = 0.0;
  imu_.robust_visual_residual = 0.0;
  visual_.yaw_disagreement_scale = 1.0;
  visual_vx_window_.clear();
  visual_wz_window_.clear();
  raw_visual_vx_window_.clear();
  raw_visual_wz_window_.clear();
  wheel_visual_residual_window_.clear();
  imu_visual_residual_window_.clear();
  raw_visual_.velocity_valid = false;
  raw_visual_.increment_pose_valid = false;
  visual_.recovery_samples = 0;
  if (!already_interrupted) {
    health_monitor_.start_vision_outage(steady_seconds());
  }
}

void FusionGateNode::raw_visual_callback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  const auto &position = message->pose.pose.position;
  const bool orientation_valid =
      ValidQuaternion(message->pose.pose.orientation);
  const double tilt = orientation_valid
                          ? QuaternionTilt(message->pose.pose.orientation)
                          : std::numeric_limits<double>::infinity();
  if (!FramePairMatches(message->header.frame_id, message->child_frame_id,
                        raw_visual_expected_frame_,
                        visual_expected_child_frame_) ||
      !finite(position.x) || !finite(position.y) || !orientation_valid ||
      tilt > raw_visual_max_tilt_) {
    reject_raw_visual_sample();
    ++raw_visual_.invalid_samples;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting raw visual odometry with invalid "
                         "frame/value or %.3f rad body tilt",
                         tilt);
    return;
  }
  const rclcpp::Time stamp(message->header.stamp);
  if (!accept_measurement_stamp(message->header.stamp, raw_visual_.stamp,
                                raw_visual_timeout_, "raw visual")) {
    reject_raw_visual_sample();
    ++raw_visual_.invalid_samples;
    return;
  }
  const Pose2d current_raw_pose{position.x, position.y,
                                quaternion_yaw(message->pose.pose.orientation)};
  if (raw_visual_.increment_pose_valid) {
    const double dt = (stamp - raw_visual_.last_increment_stamp).seconds();
    if (dt > 0.01 && dt < 0.5) {
      const double dx = current_raw_pose.x - raw_visual_.last_increment_pose.x;
      const double dy = current_raw_pose.y - raw_visual_.last_increment_pose.y;
      const double forward_velocity =
          (std::cos(raw_visual_.last_increment_pose.yaw) * dx +
           std::sin(raw_visual_.last_increment_pose.yaw) * dy) /
          dt;
      const double yaw_rate = wrap_angle(current_raw_pose.yaw -
                                         raw_visual_.last_increment_pose.yaw) /
                              dt;
      raw_visual_.velocity_valid =
          finite(forward_velocity) && finite(yaw_rate) &&
          std::abs(forward_velocity) <= max_wheel_speed_ &&
          std::abs(yaw_rate) <= max_imu_yaw_rate_;
      if (raw_visual_.velocity_valid) {
        const double sample_time = steady_seconds();
        raw_visual_vx_window_.add(sample_time, forward_velocity);
        raw_visual_wz_window_.add(sample_time, yaw_rate);
        raw_visual_.forward_velocity = forward_velocity;
        raw_visual_.yaw_rate = yaw_rate;
      } else {
        raw_visual_vx_window_.clear();
        raw_visual_wz_window_.clear();
      }
    } else {
      raw_visual_.velocity_valid = false;
      raw_visual_vx_window_.clear();
      raw_visual_wz_window_.clear();
    }
  }
  raw_visual_.last_increment_pose = current_raw_pose;
  raw_visual_.last_increment_stamp = stamp;
  raw_visual_.increment_pose_valid = true;
  raw_visual_.input.update();
  publish_raw_visual(*message, current_raw_pose);
}

void FusionGateNode::publish_raw_visual(const nav_msgs::msg::Odometry &message,
                                        const Pose2d &raw_pose) {
  if (!tracking_fresh_and_good()) {
    return;
  }

  const bool recovering = !visual_.ever_accepted || visual_.interrupted;
  const double current_steady = steady_seconds();
  const bool map_change_grace =
      map_change_.pending && current_steady <= map_change_.grace_until;
  if (map_change_.pending && !map_change_grace) {
    map_change_.pending = false;
  }
  // A normal frame must have a physically plausible raw increment. On ORB
  // loop closure MapChanged() is published before the corrected pose, so the
  // one discontinuous observation is allowed and smoothed by map->odom.
  if (!raw_visual_.velocity_valid && !map_change_grace) {
    if (!recovering) {
      ++visual_.rejected_recoveries;
      mark_visual_interrupted();
    }
    return;
  }
  if (map_change_grace) {
    // Consume authorization on the first post-event visual sample.
    map_change_.pending = false;
  }
  if (recovering) {
    if (!raw_visual_increment_healthy()) {
      return;
    }
    if (++visual_.recovery_samples < recovery_samples_) {
      return;
    }
  }

  if (recovering) {
    visual_.interrupted = false;
    health_monitor_.finish_vision_outage();
    visual_.ever_accepted = true;
    visual_.ramp_started_at = steady_seconds();
    visual_.ramp_active = true;
    visual_.realigned_until = visual_.ramp_started_at + visual_ramp_duration_;
    ++visual_.realign_count;
    RCLCPP_INFO(get_logger(),
                "Raw visual odometry accepted after %zu coherent samples; "
                "correcting global drift",
                visual_.recovery_samples);
  }

  double position_scale = 1.0;
  double yaw_scale = 1.0;
  visual_.yaw_disagreement_scale = 1.0;
  if (imu_healthy() && imu_visual_residual_window_.ready(robust_min_samples_)) {
    visual_.yaw_disagreement_scale = disagreement_covariance_scale(
        imu_.robust_visual_residual, imu_soft_residual_,
        imu_visual_covariance_cap_);
    yaw_scale = std::max(yaw_scale, visual_.yaw_disagreement_scale);
  }
  if (visual_.ramp_active) {
    const double progress = std::clamp(
        (steady_seconds() - visual_.ramp_started_at) / visual_ramp_duration_,
        0.0, 1.0);
    const double ramp_scale =
        1.0 + (visual_ramp_initial_scale_ - 1.0) * (1.0 - progress);
    position_scale = std::max(position_scale, ramp_scale);
    yaw_scale = std::max(yaw_scale, ramp_scale);
    if (progress >= 1.0) {
      visual_.ramp_active = false;
    }
  }

  nav_msgs::msg::Odometry output = message;
  output.header.frame_id = world_frame_;
  output.child_frame_id = base_frame_;
  output.pose.pose.position.x = raw_pose.x;
  output.pose.pose.position.y = raw_pose.y;
  output.pose.pose.position.z = 0.0;
  output.pose.pose.orientation = yaw_quaternion(raw_pose.yaw);
  output.pose.covariance.fill(0.0);
  output.pose.covariance[0] = visual_xy_variance_ * position_scale;
  output.pose.covariance[7] = visual_xy_variance_ * position_scale;
  output.pose.covariance[14] = 1.0e6;
  output.pose.covariance[21] = 1.0e6;
  output.pose.covariance[28] = 1.0e6;
  output.pose.covariance[35] = visual_yaw_variance_ * yaw_scale;
  output.twist.covariance.fill(0.0);
  for (std::size_t index = 0; index < 6; ++index) {
    output.twist.covariance[index * 6 + index] = 1.0e6;
  }
  visual_.forward_velocity = raw_visual_vx_window_.median();
  visual_.yaw_rate = raw_visual_wz_window_.median();
  visual_.velocity_valid = true;
  visual_vx_window_.add(steady_seconds(), visual_.forward_velocity);
  visual_wz_window_.add(steady_seconds(), visual_.yaw_rate);
  output.twist.twist.linear.x = visual_.forward_velocity;
  output.twist.twist.linear.y = 0.0;
  output.twist.twist.linear.z = 0.0;
  output.twist.covariance[0] = visual_vx_variance_ * position_scale;
  visual_publisher_->publish(output);

  visual_.accepted = true;
  visual_.input.update();
}

} // namespace fused_odometry
