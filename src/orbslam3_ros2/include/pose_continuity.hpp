#ifndef ORBSLAM3_POSE_CONTINUITY_HPP_
#define ORBSLAM3_POSE_CONTINUITY_HPP_

#include <sophus/se3.hpp>

#include <algorithm>
#include <cmath>

namespace orbslam3_ros2
{

inline bool PoseJumpExceedsThreshold(
    const Sophus::SE3f &previous_pose,
    const Sophus::SE3f &current_pose,
    float position_threshold,
    float rotation_threshold)
{
    if (!std::isfinite(position_threshold) || !std::isfinite(rotation_threshold))
        return true;

    const float position_limit = std::max(0.0f, position_threshold);
    const float rotation_limit = std::max(0.0f, rotation_threshold);
    const float position_delta =
        (current_pose.translation() - previous_pose.translation()).norm();
    const float rotation_delta =
        (previous_pose.so3().inverse() * current_pose.so3()).log().norm();
    return !std::isfinite(position_delta) || !std::isfinite(rotation_delta) ||
           position_delta > position_limit || rotation_delta > rotation_limit;
}

// Converts the source world into a fixed frame whose origin and axes are the
// first observed body pose. Unlike PoseContinuity, this reference never moves
// after tracking interruptions, so relocalization jumps remain observable.
class FixedPoseOrigin
{
public:
    Sophus::SE3f Align(const Sophus::SE3f &sourcePose)
    {
        if (!initialized_)
        {
            outputFromSource_ = sourcePose.inverse();
            initialized_ = true;
        }
        return outputFromSource_ * sourcePose;
    }

    bool Initialized() const
    {
        return initialized_;
    }

private:
    bool initialized_{false};
    Sophus::SE3f outputFromSource_;
};

class PoseContinuity
{
public:
    Sophus::SE3f Align(const Sophus::SE3f &sourcePose)
    {
        if (!initialized_)
        {
            outputFromSource_ = sourcePose.inverse();
            initialized_ = true;
        }
        else if (recoveryPending_)
        {
            outputFromSource_ = lastOutputPose_ * sourcePose.inverse();
            recoveryPending_ = false;
        }

        lastOutputPose_ = outputFromSource_ * sourcePose;
        return lastOutputPose_;
    }

    void MarkTrackingInterrupted()
    {
        if (initialized_)
            recoveryPending_ = true;
    }

    bool RecoveryPending() const
    {
        return recoveryPending_;
    }

private:
    bool initialized_{false};
    bool recoveryPending_{false};
    Sophus::SE3f outputFromSource_;
    Sophus::SE3f lastOutputPose_;
};

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_POSE_CONTINUITY_HPP_
