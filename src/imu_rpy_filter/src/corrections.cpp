#include "imu_rpy_filter/imu_rpy_filter_node.hpp"

namespace imu_rpy_filter {

void ImuRpyFilterNode::apply_stationary_update(double stamp, bool stationary) {
  const bool candidate = stationary_detector_.has_candidate();
  if (stationary || candidate) {
    if (!have_stationary_anchor_) {
      stationary_anchor_ = yaw_filter_.yaw();
      last_stationary_update_ = stamp;
      have_stationary_anchor_ = true;
    } else if (stamp - last_stationary_update_ >= 0.05) {
      if (stationary_detector_.exit_pending()) {
        correction_source_ = "stationary_exit_pending";
        return;
      }
      yaw_filter_.hold_yaw(stationary_anchor_);
      if (stationary_bias_window_.ready(20)) {
        yaw_filter_.update_bias(stationary_bias_window_.median(),
                                stationary_bias_std_);
      }
      last_stationary_update_ = stamp;
      correction_source_ = stationary ? "stationary" : "stationary_candidate";
    }
  } else {
    have_stationary_anchor_ = false;
    correction_source_ = "gyro";
  }
}

void ImuRpyFilterNode::apply_visual_update() {
  if (!use_yaw_reference_ || !have_reference_ || !have_tracking_state_ ||
      reference_sequence_ == applied_reference_sequence_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const double reference_age =
      std::chrono::duration<double>(now - reference_received_at_).count();
  const double tracking_age =
      std::chrono::duration<double>(now - tracking_received_at_).count();
  if (tracking_state_ != 2 || reference_age > 0.2 || tracking_age > 0.5) {
    return;
  }

  if (!have_reference_offset_) {
    reference_offset_ = wrap_angle(latest_reference_yaw_ - yaw_filter_.yaw());
    have_reference_offset_ = true;
  }
  const double relative_reference =
      wrap_angle(latest_reference_yaw_ - reference_offset_);
  if (std::abs(yaw_filter_.innovation(relative_reference)) <=
      visual_yaw_gate_) {
    yaw_filter_.update(relative_reference, visual_yaw_std_);
    rejected_reference_count_ = 0;
    correction_source_ = "visual";
  } else {
    ++rejected_reference_count_;
    correction_source_ = "visual_reject";
    if (rejected_reference_count_ >= 10) {
      reference_offset_ = wrap_angle(latest_reference_yaw_ - yaw_filter_.yaw());
      rejected_reference_count_ = 0;
      correction_source_ = "visual_rebase";
    }
  }
  applied_reference_sequence_ = reference_sequence_;
}

} // namespace imu_rpy_filter
