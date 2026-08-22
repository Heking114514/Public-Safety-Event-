#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
const char *TrackingStateName(int state)
{
    switch (state)
    {
        case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
            return "SYSTEM_NOT_READY";
        case ORB_SLAM3::Tracking::NO_IMAGES_YET:
            return "NO_IMAGES_YET";
        case ORB_SLAM3::Tracking::NOT_INITIALIZED:
            return "NOT_INITIALIZED";
        case ORB_SLAM3::Tracking::OK:
            return "OK";
        case ORB_SLAM3::Tracking::RECENTLY_LOST:
            return "RECENTLY_LOST";
        case ORB_SLAM3::Tracking::LOST:
            return "LOST";
        case ORB_SLAM3::Tracking::OK_KLT:
            return "OK_KLT";
        default:
            return "UNKNOWN";
    }
}

geometry_msgs::msg::Pose PoseFromSE3(const Sophus::SE3f &transform)
{
    geometry_msgs::msg::Pose pose;
    const Eigen::Vector3f translation = transform.translation();
    const Eigen::Quaternionf rotation = transform.unit_quaternion().normalized();

    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
}

geometry_msgs::msg::Pose PlanarPoseFromSE3(const Sophus::SE3f &transform)
{
    geometry_msgs::msg::Pose pose = PoseFromSE3(transform);
    const double yaw = std::atan2(
        2.0 * (pose.orientation.w * pose.orientation.z +
               pose.orientation.x * pose.orientation.y),
        1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                     pose.orientation.z * pose.orientation.z));
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(0.5 * yaw);
    pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
}

geometry_msgs::msg::Transform TransformFromSE3(const Sophus::SE3f &transform)
{
    geometry_msgs::msg::Transform message;
    const Eigen::Vector3f translation = transform.translation();
    const Eigen::Quaternionf rotation = transform.unit_quaternion().normalized();

    message.translation.x = translation.x();
    message.translation.y = translation.y();
    message.translation.z = translation.z();
    message.rotation.x = rotation.x();
    message.rotation.y = rotation.y();
    message.rotation.z = rotation.z();
    message.rotation.w = rotation.w();
    return message;
}

void SetCovarianceDiagonal(std::array<double, 36> &covariance, const std::vector<double> &diagonal)
{
    covariance.fill(0.0);
    for (size_t index = 0; index < std::min<size_t>(6, diagonal.size()); ++index)
        covariance[index * 6 + index] = diagonal[index];
}

diagnostic_msgs::msg::KeyValue DiagnosticValue(const std::string &key, const std::string &value)
{
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
}
}

StereoInertialNode::StereoInertialNode(
    ORB_SLAM3::System *SLAM,
    const string &strSettingsFile,
    const string &strDoRectify,
    const string &strDoEqual,
    bool useImu)
    : Node("ORB_SLAM3_ROS2"), SLAM_(SLAM), useImu_(useImu)
{
    std::stringstream rectifyStream(strDoRectify);
    rectifyStream >> std::boolalpha >> doRectify_;

    std::stringstream equalizeStream(strDoEqual);
    equalizeStream >> std::boolalpha >> doEqual_;

    mapFrameId_ = this->declare_parameter<std::string>("map_frame_id", "map");
    bodyFrameId_ = this->declare_parameter<std::string>("body_frame_id", "camera_link");
    publishTf_ = this->declare_parameter<bool>("publish_tf", true);
    publishPath_ = this->declare_parameter<bool>("publish_path", true);
    saveTrajectory_ = this->declare_parameter<bool>("save_trajectory", true);
    resetPathOnTrackingLoss_ = this->declare_parameter<bool>("reset_path_on_tracking_loss", true);
    trajectoryFile_ = this->declare_parameter<std::string>("trajectory_file", "KeyFrameTrajectory.txt");
    maxStereoTimeDiff_ = this->declare_parameter<double>("max_stereo_time_diff", 0.01);
    imuTimeOffset_ = this->declare_parameter<double>("imu_time_offset", 0.0);
    diagnosticsPeriod_ = std::max(0.0, this->declare_parameter<double>("diagnostics_period", 1.0));
    cameraWarmupSeconds_ = std::max(0.0, this->declare_parameter<double>("camera_warmup_seconds", 2.0));
    cameraWarmupComplete_ = cameraWarmupSeconds_ == 0.0;
    const int maxPendingStereoPairs = this->declare_parameter<int>("max_pending_stereo_pairs", 10);
    const int maxImuQueueSize = this->declare_parameter<int>("max_imu_queue_size", 4000);
    const int pathMaxPoses = this->declare_parameter<int>("path_max_poses", 2000);
    maxPendingStereoPairs_ = static_cast<size_t>(std::max(1, maxPendingStereoPairs));
    maxImuQueueSize_ = static_cast<size_t>(std::max(200, maxImuQueueSize));
    pathMaxPoses_ = static_cast<size_t>(std::max(1, pathMaxPoses));
    poseCovarianceDiagonal_ = this->declare_parameter<std::vector<double>>(
        "pose_covariance_diagonal", {0.01, 0.01, 0.01, 0.05, 0.05, 0.05});
    twistCovarianceDiagonal_ = this->declare_parameter<std::vector<double>>(
        "twist_covariance_diagonal", {0.04, 0.04, 0.04, 0.1, 0.1, 0.1});
    unavailableTwistCovarianceDiagonal_ = this->declare_parameter<std::vector<double>>(
        "unavailable_twist_covariance_diagonal", {1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6});

    const std::string odomTopic = this->declare_parameter<std::string>(
        "odom_topic", "/odometry/visual_continuous");
    const std::string rawOdomTopic = this->declare_parameter<std::string>(
        "raw_odom_topic", "/odometry/visual_raw");
    const std::string legacyOdomTopic = this->declare_parameter<std::string>(
        "legacy_odom_topic", "/odom");
    const std::string legacyRawOdomTopic = this->declare_parameter<std::string>(
        "legacy_raw_odom_topic", "/odom/orb_raw");
    const std::string poseTopic = this->declare_parameter<std::string>("pose_topic", "pose");
    const std::string pathTopic = this->declare_parameter<std::string>("path_topic", "path");
    const std::string stateTopic = this->declare_parameter<std::string>("tracking_state_topic", "tracking_state");
    const std::string mapChangeTopic = this->declare_parameter<std::string>(
        "map_change_topic", "/orbslam3/map_change");
    const std::string diagnosticsTopic = this->declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    RCLCPP_INFO(this->get_logger(), "Rectify: %s", doRectify_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Equalize: %s", doEqual_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Sensor mode: %s", useImu_ ? "stereo-inertial" : "stereo");
    RCLCPP_INFO(this->get_logger(), "Camera warmup: %.1f seconds", cameraWarmupSeconds_);
    RCLCPP_INFO(this->get_logger(), "Publishing %s -> %s", mapFrameId_.c_str(), bodyFrameId_.c_str());

    if (doRectify_)
    {
        cv::FileStorage settings(strSettingsFile, cv::FileStorage::READ);
        if (!settings.isOpened())
            throw std::runtime_error("Unable to open ORB-SLAM3 settings file: " + strSettingsFile);

        cv::Mat leftK, rightK, leftP, rightP, leftR, rightR, leftD, rightD;
        settings["LEFT.K"] >> leftK;
        settings["RIGHT.K"] >> rightK;
        settings["LEFT.P"] >> leftP;
        settings["RIGHT.P"] >> rightP;
        settings["LEFT.R"] >> leftR;
        settings["RIGHT.R"] >> rightR;
        settings["LEFT.D"] >> leftD;
        settings["RIGHT.D"] >> rightD;

        const int leftRows = settings["LEFT.height"];
        const int leftColumns = settings["LEFT.width"];
        const int rightRows = settings["RIGHT.height"];
        const int rightColumns = settings["RIGHT.width"];

        if (leftK.empty() || rightK.empty() || leftP.empty() || rightP.empty() || leftR.empty() ||
            rightR.empty() || leftD.empty() || rightD.empty() || leftRows == 0 || rightRows == 0 ||
            leftColumns == 0 || rightColumns == 0)
        {
            throw std::runtime_error("Stereo rectification parameters are missing from the settings file");
        }

        cv::initUndistortRectifyMap(
            leftK, leftD, leftR, leftP.rowRange(0, 3).colRange(0, 3),
            cv::Size(leftColumns, leftRows), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(
            rightK, rightD, rightR, rightP.rowRange(0, 3).colRange(0, 3),
            cv::Size(rightColumns, rightRows), CV_32F, M1r_, M2r_);
    }

    odomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic, 10);
    rawOdomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(rawOdomTopic, 10);
    if (!legacyOdomTopic.empty() && legacyOdomTopic != odomTopic)
        legacyOdomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(legacyOdomTopic, 10);
    if (!legacyRawOdomTopic.empty() && legacyRawOdomTopic != rawOdomTopic)
        legacyRawOdomPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(legacyRawOdomTopic, 10);
    posePublisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(poseTopic, 10);
    pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>(pathTopic, 10);
    trackingStatePublisher_ = this->create_publisher<std_msgs::msg::Int32>(stateTopic, 10);
    mapChangePublisher_ = this->create_publisher<std_msgs::msg::UInt64>(mapChangeTopic, 10);
    diagnosticsPublisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnosticsTopic, 10);

    RCLCPP_INFO(
        this->get_logger(), "Visual odometry topics: continuous=%s, raw=%s",
        odomTopic.c_str(), rawOdomTopic.c_str());
    if (legacyOdomPublisher_ || legacyRawOdomPublisher_)
        RCLCPP_INFO(
            this->get_logger(), "Legacy odometry aliases: continuous=%s, raw=%s",
            legacyOdomTopic.empty() ? "disabled" : legacyOdomTopic.c_str(),
            legacyRawOdomTopic.empty() ? "disabled" : legacyRawOdomTopic.c_str());

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this, true);
    tfBroadcaster_.reset(new tf2_ros::TransformBroadcaster(this));

    const auto sensorQos = rclcpp::SensorDataQoS();
    if (useImu_)
        subImu_ = this->create_subscription<ImuMsg>("imu", sensorQos, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/left", rmw_qos_profile_sensor_data);
    subImgRight_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/right", rmw_qos_profile_sensor_data);
    stereoSync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
        ApproximateSyncPolicy(20), *subImgLeft_, *subImgRight_);
    stereoSync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(maxStereoTimeDiff_));
    stereoSync_->registerCallback(std::bind(&StereoInertialNode::GrabStereo, this, _1, _2));

    path_.header.frame_id = mapFrameId_;
    syncThread_ = std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
    StopProcessing();
}

void StereoInertialNode::StopProcessing()
{
    if (stopped_.exchange(true))
        return;

    syncRunning_ = false;
    dataCondition_.notify_all();
    if (syncThread_.joinable())
        syncThread_.join();

    SLAM_->Shutdown();
    if (saveTrajectory_)
    {
        RCLCPP_INFO(this->get_logger(), "Saving keyframe trajectory to %s", trajectoryFile_.c_str());
        SLAM_->SaveKeyFrameTrajectoryTUM(trajectoryFile_);
    }
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    const double timestamp = Utility::StampToSec(msg->header.stamp) + imuTimeOffset_;
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        if (!imuBuf_.empty())
        {
            const double lastTimestamp = Utility::StampToSec(imuBuf_.back()->header.stamp) + imuTimeOffset_;
            if (timestamp <= lastTimestamp)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Discarding non-monotonic IMU sample: %.9f <= %.9f", timestamp, lastTimestamp);
                return;
            }
        }

        imuBuf_.push_back(msg);
        while (imuBuf_.size() > maxImuQueueSize_)
            imuBuf_.pop_front();
    }
    dataCondition_.notify_one();
}

void StereoInertialNode::GrabStereo(
    const ImageMsg::ConstSharedPtr &msgLeft,
    const ImageMsg::ConstSharedPtr &msgRight)
{
    const double leftTimestamp = Utility::StampToSec(msgLeft->header.stamp);
    const double rightTimestamp = Utility::StampToSec(msgRight->header.stamp);
    const double timestampDifference = std::abs(leftTimestamp - rightTimestamp);

    if (timestampDifference > maxStereoTimeDiff_)
    {
        ++rejectedStereoPairs_;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Rejecting stereo pair with %.3f ms timestamp difference",
            timestampDifference * 1000.0);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        if (stereoBuf_.size() >= maxPendingStereoPairs_)
        {
            stereoBuf_.pop_front();
            ++droppedStereoPairs_;
        }
        stereoBuf_.push_back({msgLeft, msgRight});
    }
    dataCondition_.notify_one();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::ConstSharedPtr &msg) const
{
    try
    {
        const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        return image->image.clone();
    }
    catch (const cv_bridge::Exception &exception)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", exception.what());
        return cv::Mat();
    }
}

void StereoInertialNode::SyncWithImu()
{
    while (syncRunning_)
    {
        StereoPair stereoPair;
        std::vector<ORB_SLAM3::IMU::Point> imuMeasurements;

        {
            std::unique_lock<std::mutex> lock(dataMutex_);
            dataCondition_.wait(lock, [this]() {
                if (!syncRunning_)
                    return true;
                if (stereoBuf_.empty())
                    return false;
                if (!useImu_)
                    return true;
                if (imuBuf_.empty())
                    return false;
                const double imageTimestamp = Utility::StampToSec(stereoBuf_.front().left->header.stamp);
                const double latestImuTimestamp = Utility::StampToSec(imuBuf_.back()->header.stamp) + imuTimeOffset_;
                return latestImuTimestamp >= imageTimestamp;
            });

            if (!syncRunning_)
                break;

            stereoPair = stereoBuf_.front();
            stereoBuf_.pop_front();
            const double imageTimestamp = Utility::StampToSec(stereoPair.left->header.stamp);

            while (useImu_ && !imuBuf_.empty())
            {
                const double imuTimestamp = Utility::StampToSec(imuBuf_.front()->header.stamp) + imuTimeOffset_;
                if (imuTimestamp > imageTimestamp)
                    break;

                const ImuMsg::SharedPtr imu = imuBuf_.front();
                imuBuf_.pop_front();
                if (lastImageTimestamp_ < 0.0 || imuTimestamp > lastImageTimestamp_)
                {
                    const cv::Point3f acceleration(
                        imu->linear_acceleration.x,
                        imu->linear_acceleration.y,
                        imu->linear_acceleration.z);
                    const cv::Point3f angularVelocity(
                        imu->angular_velocity.x,
                        imu->angular_velocity.y,
                        imu->angular_velocity.z);
                    imuMeasurements.emplace_back(acceleration, angularVelocity, imuTimestamp);
                }
            }
        }

        const double leftTimestamp = Utility::StampToSec(stereoPair.left->header.stamp);
        const double rightTimestamp = Utility::StampToSec(stereoPair.right->header.stamp);
        const double stereoDifference = std::abs(leftTimestamp - rightTimestamp);

        if (firstStereoTimestamp_ < 0.0)
        {
            firstStereoTimestamp_ = leftTimestamp;
        }

        if (!cameraWarmupComplete_ && leftTimestamp - firstStereoTimestamp_ < cameraWarmupSeconds_)
        {
            ++warmupStereoPairs_;
            lastImageTimestamp_ = leftTimestamp;
            PublishTrackingStatus(
                ORB_SLAM3::Tracking::SYSTEM_NOT_READY,
                stereoPair.left->header.stamp,
                imuMeasurements.size(),
                stereoDifference);
            continue;
        }

        if (!cameraWarmupComplete_)
        {
            cameraWarmupComplete_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Camera warmup complete after %zu stereo pairs",
                warmupStereoPairs_.load());
        }

        cv::Mat leftImage = GetImage(stereoPair.left);
        cv::Mat rightImage = GetImage(stereoPair.right);
        if (leftImage.empty() || rightImage.empty())
        {
            PublishTrackingStatus(ORB_SLAM3::Tracking::LOST, stereoPair.left->header.stamp, imuMeasurements.size(), stereoDifference);
            HandleTrackingInterruption();
            continue;
        }

        if (doEqual_)
        {
            clahe_->apply(leftImage, leftImage);
            clahe_->apply(rightImage, rightImage);
        }

        if (doRectify_)
        {
            cv::remap(leftImage, leftImage, M1l_, M2l_, cv::INTER_LINEAR);
            cv::remap(rightImage, rightImage, M1r_, M2r_, cv::INTER_LINEAR);
        }

        const Sophus::SE3f Tcw = SLAM_->TrackStereo(leftImage, rightImage, leftTimestamp, imuMeasurements);
        if (SLAM_->MapChanged())
        {
            std_msgs::msg::UInt64 event;
            event.data = ++mapChangeCount_;
            mapChangePublisher_->publish(event);
            RCLCPP_INFO(
                this->get_logger(), "ORB map correction detected (sequence=%lu)",
                static_cast<unsigned long>(event.data));
        }
        lastImageTimestamp_ = leftTimestamp;

        const int trackingState = SLAM_->GetTrackingState();
        PublishTrackingStatus(trackingState, stereoPair.left->header.stamp, imuMeasurements.size(), stereoDifference);

        if (trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT)
        {
            PublishPose(Tcw, stereoPair.left, trackingState);
        }
        else
        {
            HandleTrackingInterruption();
        }
    }
}

bool StereoInertialNode::LookupCameraToBody(const std::string &cameraFrame, Sophus::SE3f &Tcb)
{
    if (cameraFrame == bodyFrameId_)
    {
        Tcb = Sophus::SE3f();
        return true;
    }

    try
    {
        const geometry_msgs::msg::TransformStamped transform = tfBuffer_->lookupTransform(
            cameraFrame, bodyFrameId_, tf2::TimePointZero, tf2::durationFromSec(0.2));
        const auto &translation = transform.transform.translation;
        const auto &rotation = transform.transform.rotation;
        Eigen::Quaternionf quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
        quaternion.normalize();
        Tcb = Sophus::SE3f(
            quaternion.toRotationMatrix(),
            Eigen::Vector3f(translation.x, translation.y, translation.z));
        return true;
    }
    catch (const tf2::TransformException &exception)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Cannot transform body frame '%s' into camera frame '%s': %s",
            bodyFrameId_.c_str(), cameraFrame.c_str(), exception.what());
        return false;
    }
}

void StereoInertialNode::PublishPose(
    const Sophus::SE3f &Tcw,
    const ImageMsg::ConstSharedPtr &msgLeft,
    int trackingState)
{
    if (trackingState != ORB_SLAM3::Tracking::OK && trackingState != ORB_SLAM3::Tracking::OK_KLT)
        return;

    Sophus::SE3f TcameraBody;
    if (!LookupCameraToBody(msgLeft->header.frame_id, TcameraBody))
        return;

    const Sophus::SE3f TworldCamera = Tcw.inverse();
    const Sophus::SE3f TworldBody = TworldCamera * TcameraBody;
    // ORB's world axes follow the optical camera convention. Normalize them
    // once against the first body pose before labeling the result as ROS map.
    // The fixed origin intentionally does not move on tracking recovery.
    const Sophus::SE3f TrawMapBody = rawPoseOrigin_.Align(TworldBody);
    PublishRawOdometry(TrawMapBody, msgLeft->header.stamp);
    const bool recoveredFromInterruption = poseContinuity_.RecoveryPending();
    const Sophus::SE3f TmapBody = poseContinuity_.Align(TworldBody);
    if (recoveredFromInterruption)
    {
        RCLCPP_INFO(this->get_logger(), "Tracking recovered; preserving the published odometry frame");
    }
    trackingInterruptionActive_ = false;
    const double timestamp = Utility::StampToSec(msgLeft->header.stamp);

    geometry_msgs::msg::PoseStamped poseMessage;
    poseMessage.header.stamp = msgLeft->header.stamp;
    poseMessage.header.frame_id = mapFrameId_;
    poseMessage.pose = PoseFromSE3(TmapBody);
    posePublisher_->publish(poseMessage);

    nav_msgs::msg::Odometry odometry;
    odometry.header = poseMessage.header;
    odometry.child_frame_id = bodyFrameId_;
    odometry.pose.pose = poseMessage.pose;
    SetCovarianceDiagonal(odometry.pose.covariance, poseCovarianceDiagonal_);
    SetCovarianceDiagonal(odometry.twist.covariance, twistCovarianceDiagonal_);

    if (lastPublishedPoseValid_ && timestamp > lastPublishedTimestamp_)
    {
        const double deltaTime = timestamp - lastPublishedTimestamp_;
        const Sophus::SE3f previousToCurrent = lastPublishedPose_.inverse() * TmapBody;
        const Eigen::Vector3f linearVelocity = previousToCurrent.translation() / static_cast<float>(deltaTime);
        const Eigen::Vector3f angularVelocity = previousToCurrent.so3().log() / static_cast<float>(deltaTime);
        odometry.twist.twist.linear.x = linearVelocity.x();
        odometry.twist.twist.linear.y = linearVelocity.y();
        odometry.twist.twist.linear.z = linearVelocity.z();
        odometry.twist.twist.angular.x = angularVelocity.x();
        odometry.twist.twist.angular.y = angularVelocity.y();
        odometry.twist.twist.angular.z = angularVelocity.z();
    }
    odomPublisher_->publish(odometry);
    if (legacyOdomPublisher_)
        legacyOdomPublisher_->publish(odometry);

    if (publishTf_)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = poseMessage.header;
        transform.child_frame_id = bodyFrameId_;
        transform.transform = TransformFromSE3(TmapBody);
        tfBroadcaster_->sendTransform(transform);
    }

    if (publishPath_)
    {
        path_.header = poseMessage.header;
        path_.poses.push_back(poseMessage);
        while (path_.poses.size() > pathMaxPoses_)
            path_.poses.erase(path_.poses.begin());
        pathPublisher_->publish(path_);
    }

    lastPublishedPose_ = TmapBody;
    lastPublishedTimestamp_ = timestamp;
    lastPublishedPoseValid_ = true;
}

void StereoInertialNode::PublishRawOdometry(
    const Sophus::SE3f &TrawMapBody,
    const builtin_interfaces::msg::Time &stamp)
{
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = mapFrameId_;
    odometry.child_frame_id = bodyFrameId_;
    // This topic is the fixed-origin planar observation consumed by the
    // navigation fusion layer. ORB keeps its full SE(3) map internally.
    odometry.pose.pose = PlanarPoseFromSE3(TrawMapBody);
    SetCovarianceDiagonal(odometry.pose.covariance, poseCovarianceDiagonal_);

    const double timestamp = Utility::StampToSec(stamp);
    const orbslam3_ros2::PoseVelocity velocity = rawVelocityEstimator_.Observe(TrawMapBody, timestamp);
    if (velocity.valid)
    {
        odometry.twist.twist.linear.x = velocity.linear.x();
        odometry.twist.twist.linear.y = velocity.linear.y();
        odometry.twist.twist.linear.z = velocity.linear.z();
        odometry.twist.twist.angular.x = velocity.angular.x();
        odometry.twist.twist.angular.y = velocity.angular.y();
        odometry.twist.twist.angular.z = velocity.angular.z();
        SetCovarianceDiagonal(odometry.twist.covariance, twistCovarianceDiagonal_);
    }
    else
    {
        SetCovarianceDiagonal(odometry.twist.covariance, unavailableTwistCovarianceDiagonal_);
    }
    rawOdomPublisher_->publish(odometry);
    if (legacyRawOdomPublisher_)
        legacyRawOdomPublisher_->publish(odometry);
}

void StereoInertialNode::PublishTrackingStatus(
    int trackingState,
    const builtin_interfaces::msg::Time &stamp,
    size_t imuCount,
    double stereoDelta)
{
    std_msgs::msg::Int32 stateMessage;
    stateMessage.data = trackingState;
    trackingStatePublisher_->publish(stateMessage);

    const double diagnosticsTimestamp = Utility::StampToSec(stamp);
    const bool trackingStateChanged = trackingState != lastDiagnosticsTrackingState_;
    if (!trackingStateChanged && lastDiagnosticsTimestamp_ >= 0.0 &&
        diagnosticsTimestamp - lastDiagnosticsTimestamp_ < diagnosticsPeriod_)
    {
        return;
    }
    lastDiagnosticsTimestamp_ = diagnosticsTimestamp;
    lastDiagnosticsTrackingState_ = trackingState;

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "orbslam3/tracking";
    status.hardware_id = "Intel RealSense D455";

    if (trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT)
    {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Tracking";
    }
    else if (trackingState == ORB_SLAM3::Tracking::LOST)
    {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Tracking lost";
    }
    else
    {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = TrackingStateName(trackingState);
    }

    status.values.push_back(DiagnosticValue("tracking_state", TrackingStateName(trackingState)));
    status.values.push_back(DiagnosticValue(
        "sensor_mode", useImu_ ? "stereo-inertial" : "stereo"));
    status.values.push_back(DiagnosticValue(
        "imu_initialization",
        useImu_
            ? (trackingState == ORB_SLAM3::Tracking::OK || trackingState == ORB_SLAM3::Tracking::OK_KLT
                   ? "managed internally by ORB-SLAM3"
                   : "pending")
            : "disabled"));
    status.values.push_back(DiagnosticValue("imu_samples", std::to_string(imuCount)));
    status.values.push_back(DiagnosticValue(
        "camera_warmup", cameraWarmupComplete_ ? "complete" : "active"));
    status.values.push_back(DiagnosticValue(
        "warmup_stereo_pairs", std::to_string(warmupStereoPairs_)));
    status.values.push_back(DiagnosticValue("stereo_delta_ms", std::to_string(stereoDelta * 1000.0)));
    status.values.push_back(DiagnosticValue("dropped_stereo_pairs", std::to_string(droppedStereoPairs_)));
    status.values.push_back(DiagnosticValue("rejected_stereo_pairs", std::to_string(rejectedStereoPairs_)));
    diagnostics.status.push_back(status);
    diagnosticsPublisher_->publish(diagnostics);
}

void StereoInertialNode::HandleTrackingInterruption()
{
    poseContinuity_.MarkTrackingInterrupted();
    rawVelocityEstimator_.Invalidate();
    lastPublishedPoseValid_ = false;
    if (!trackingInterruptionActive_ && resetPathOnTrackingLoss_)
    {
        path_.poses.clear();
        path_.header.frame_id = mapFrameId_;
    }
    trackingInterruptionActive_ = true;
}
