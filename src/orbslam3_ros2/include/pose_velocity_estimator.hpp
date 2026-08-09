#ifndef ORBSLAM3_POSE_VELOCITY_ESTIMATOR_HPP_
#define ORBSLAM3_POSE_VELOCITY_ESTIMATOR_HPP_

#include <sophus/se3.hpp>

#include <cmath>

namespace orbslam3_ros2
{

struct PoseVelocity
{
    bool valid{false};
    Eigen::Vector3f linear{Eigen::Vector3f::Zero()};
    Eigen::Vector3f angular{Eigen::Vector3f::Zero()};
};

// Estimates body-frame velocity only between poses in one uninterrupted
// tracking segment. Invalidate() deliberately discards the old anchor.
class PoseVelocityEstimator
{
public:
    PoseVelocity Observe(const Sophus::SE3f &pose, double timestamp)
    {
        PoseVelocity velocity;
        if (lastPoseValid_ && std::isfinite(timestamp) && timestamp > lastTimestamp_)
        {
            const float deltaTime = static_cast<float>(timestamp - lastTimestamp_);
            const Sophus::SE3f previousToCurrent = lastPose_.inverse() * pose;
            velocity.linear = previousToCurrent.translation() / deltaTime;
            velocity.angular = previousToCurrent.so3().log() / deltaTime;
            velocity.valid = velocity.linear.allFinite() && velocity.angular.allFinite();
            if (!velocity.valid)
            {
                velocity.linear.setZero();
                velocity.angular.setZero();
            }
        }

        lastPose_ = pose;
        lastTimestamp_ = timestamp;
        lastPoseValid_ = std::isfinite(timestamp);
        return velocity;
    }

    void Invalidate()
    {
        lastPoseValid_ = false;
    }

private:
    bool lastPoseValid_{false};
    Sophus::SE3f lastPose_;
    double lastTimestamp_{0.0};
};

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_POSE_VELOCITY_ESTIMATOR_HPP_
