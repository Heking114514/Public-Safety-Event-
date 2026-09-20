#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

struct Pose2d
{
  double x;
  double y;
  double yaw;
};

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w);
}

bool valid_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & quaternion = pose.orientation;
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && finite_quaternion(quaternion) &&
         std::isfinite(norm_squared) && norm_squared > 0.25 && norm_squared < 2.25;
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
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

Pose2d relative_to_origin(const Pose2d & origin, const Pose2d & pose)
{
  const double dx = pose.x - origin.x;
  const double dy = pose.y - origin.y;
  const double cos_origin = std::cos(origin.yaw);
  const double sin_origin = std::sin(origin.yaw);
  return {
    cos_origin * dx + sin_origin * dy,
    -sin_origin * dx + cos_origin * dy,
    normalize_angle(pose.yaw - origin.yaw)};
}

std::array<double, 36> rotate_xy_covariance(
  const std::array<double, 36> & covariance, double origin_yaw)
{
  double jacobian[6][6] = {};
  for (int index = 0; index < 6; ++index) {
    jacobian[index][index] = 1.0;
  }
  const double cos_origin = std::cos(origin_yaw);
  const double sin_origin = std::sin(origin_yaw);
  jacobian[0][0] = cos_origin;
  jacobian[0][1] = sin_origin;
  jacobian[1][0] = -sin_origin;
  jacobian[1][1] = cos_origin;

  std::array<double, 36> rotated{};
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      double value = 0.0;
      for (int left = 0; left < 6; ++left) {
        for (int right = 0; right < 6; ++right) {
          value += jacobian[row][left] * covariance[left * 6 + right] *
            jacobian[column][right];
        }
      }
      rotated[row * 6 + column] = value;
    }
  }
  return rotated;
}

}  // namespace

class LocalMapOdometryNode : public rclcpp::Node
{
public:
  LocalMapOdometryNode()
  : Node("local_map_odometry")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/odometry/local");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/local_map");
    input_frame_ = declare_parameter<std::string>("input_frame", "odom");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const bool publish_tf = declare_parameter<bool>("publish_tf", false);

    require_nonempty(input_topic_, "input_topic");
    require_nonempty(output_topic_, "output_topic");
    require_nonempty(input_frame_, "input_frame");
    require_nonempty(output_frame_, "output_frame");
    require_nonempty(base_frame_, "base_frame");
    if (publish_tf) {
      RCLCPP_WARN(
        get_logger(),
        "publish_tf was requested, but local_map_odometry intentionally publishes "
        "only Odometry to avoid competing with the existing TF tree");
    }

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        odometry_callback(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Local map odometry ready: %s -> %s, frames %s -> %s, child %s",
      input_topic_.c_str(), output_topic_.c_str(), input_frame_.c_str(),
      output_frame_.c_str(), base_frame_.c_str());
  }

private:
  void require_nonempty(const std::string & value, const std::string & name)
  {
    if (value.empty()) {
      throw std::invalid_argument(name + " must not be empty");
    }
  }

  void odometry_callback(const nav_msgs::msg::Odometry & message)
  {
    if (message.header.frame_id != input_frame_ || message.child_frame_id != base_frame_ ||
      !valid_pose(message.pose.pose))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting local odometry frame/value (%s -> %s; expected %s -> %s)",
        message.header.frame_id.c_str(), message.child_frame_id.c_str(),
        input_frame_.c_str(), base_frame_.c_str());
      return;
    }

    const rclcpp::Time stamp(message.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting local odometry with an unset measurement timestamp");
      return;
    }

    const Pose2d local_pose = message_pose(message);
    if (!origin_) {
      origin_ = local_pose;
      RCLCPP_INFO(
        get_logger(), "Locked local odometry origin at x=%.3f y=%.3f yaw=%.3f",
        origin_->x, origin_->y, origin_->yaw);
    }

    const Pose2d output_pose = relative_to_origin(*origin_, local_pose);
    auto output = message;
    output.header.frame_id = output_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = output_pose.x;
    output.pose.pose.position.y = output_pose.y;
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation = yaw_quaternion(output_pose.yaw);
    output.pose.covariance = rotate_xy_covariance(
      message.pose.covariance, origin_->yaw);

    publisher_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string input_frame_;
  std::string output_frame_;
  std::string base_frame_;
  std::optional<Pose2d> origin_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMapOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
