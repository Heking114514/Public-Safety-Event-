#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include "pose_continuity.hpp"
#include "pose_velocity_estimator.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(
        ORB_SLAM3::System* pSLAM,
        const string &strSettingsFile,
        const string &strDoRectify,
        const string &strDoEqual,
        bool useImu);
    ~StereoInertialNode();

private:
    using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    struct StereoPair
    {
        ImageMsg::ConstSharedPtr left;
        ImageMsg::ConstSharedPtr right;
    };

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabStereo(const ImageMsg::ConstSharedPtr &msgLeft, const ImageMsg::ConstSharedPtr &msgRight);
    cv::Mat GetImage(const ImageMsg::ConstSharedPtr &msg) const;
    void SyncWithImu();
    void PublishPose(const Sophus::SE3f &Tcw, const ImageMsg::ConstSharedPtr &msgLeft, int trackingState);
    void PublishRawOdometry(const Sophus::SE3f &TrawMapBody, const builtin_interfaces::msg::Time &stamp);
    void PublishTrackingStatus(int trackingState, const builtin_interfaces::msg::Time &stamp, size_t imuCount, double stereoDelta);
    bool LookupCameraToBody(const std::string &cameraFrame, Sophus::SE3f &Tcb);
    void HandleTrackingInterruption();
    void StopProcessing();

    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> subImgLeft_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> subImgRight_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> stereoSync_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rawOdomPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr trackingStatePublisher_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr mapChangePublisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnosticsPublisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    ORB_SLAM3::System *SLAM_;
    std::thread syncThread_;
    std::atomic<bool> syncRunning_{true};
    std::atomic<bool> stopped_{false};

    std::deque<ImuMsg::SharedPtr> imuBuf_;
    std::deque<StereoPair> stereoBuf_;
    std::mutex dataMutex_;
    std::condition_variable dataCondition_;

    bool doRectify_;
    bool doEqual_;
    bool useImu_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));

    std::string mapFrameId_;
    std::string bodyFrameId_;
    bool publishTf_;
    bool publishPath_;
    bool saveTrajectory_;
    bool resetPathOnTrackingLoss_;
    std::string trajectoryFile_;
    double maxStereoTimeDiff_;
    double imuTimeOffset_;
    double diagnosticsPeriod_;
    double cameraWarmupSeconds_;
    size_t maxPendingStereoPairs_;
    size_t maxImuQueueSize_;
    size_t pathMaxPoses_;
    std::vector<double> poseCovarianceDiagonal_;
    std::vector<double> twistCovarianceDiagonal_;
    std::vector<double> unavailableTwistCovarianceDiagonal_;

    double lastImageTimestamp_{-1.0};
    double firstStereoTimestamp_{-1.0};
    bool cameraWarmupComplete_{false};
    bool lastPublishedPoseValid_{false};
    bool trackingInterruptionActive_{false};
    orbslam3_ros2::FixedPoseOrigin rawPoseOrigin_;
    orbslam3_ros2::PoseContinuity poseContinuity_;
    orbslam3_ros2::PoseVelocityEstimator rawVelocityEstimator_;
    Sophus::SE3f lastPublishedPose_;
    double lastPublishedTimestamp_{0.0};
    double lastDiagnosticsTimestamp_{-1.0};
    int lastDiagnosticsTrackingState_{ORB_SLAM3::Tracking::SYSTEM_NOT_READY};
    nav_msgs::msg::Path path_;
    std::atomic<size_t> droppedStereoPairs_{0};
    std::atomic<size_t> rejectedStereoPairs_{0};
    std::atomic<size_t> warmupStereoPairs_{0};
    std::atomic<uint64_t> mapChangeCount_{0};
};

#endif
