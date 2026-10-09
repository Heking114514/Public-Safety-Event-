#ifndef ORBSLAM3_PLANAR_FRAME_HPP_
#define ORBSLAM3_PLANAR_FRAME_HPP_

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

namespace orbslam3_ros2
{

// ORB's normalized planar motion has +Y and +yaw opposite to ROS base_link.
// Changing basis by 180 degrees around X preserves a proper SE(3) transform
// while producing ROS +X forward, +Y left and +yaw counter-clockwise.
inline Sophus::SE3f ToRosPlanarFrame(const Sophus::SE3f & pose)
{
    constexpr float kPi = 3.14159265358979323846f;
    const Sophus::SE3f basis(
        Eigen::AngleAxisf(kPi, Eigen::Vector3f::UnitX()).toRotationMatrix(),
        Eigen::Vector3f::Zero());
    return basis * pose * basis.inverse();
}

}  // namespace orbslam3_ros2

#endif  // ORBSLAM3_PLANAR_FRAME_HPP_
