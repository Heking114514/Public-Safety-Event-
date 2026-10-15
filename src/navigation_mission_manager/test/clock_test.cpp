#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rcl/time.h>

#include "navigation_mission_manager/manager.hpp"

namespace navigation_mission_manager
{
struct ManagerClockTestAccess
{
  static double seconds(const MissionManager & manager) {return manager.steady_seconds();}
  static rclcpp::TimerBase::SharedPtr timer(MissionManager & manager) {return manager.timer_;}
  static void odometry(MissionManager & manager)
  {
    auto message = std::make_shared<nav_msgs::msg::Odometry>();
    message->header.stamp = manager.now();
    message->header.frame_id = "map";
    message->child_frame_id = "base_link";
    message->pose.pose.orientation.w = 1.0;
    manager.on_odometry(message);
  }
  static bool fresh(const MissionManager & manager)
  {
    return manager.pose_.has_value() &&
           manager.pose_->fresh(manager.odom_timeout_s_, manager.steady_seconds());
  }
  static double timeout(const MissionManager & manager) {return manager.odom_timeout_s_;}
  static double received(const MissionManager & manager) {return manager.pose_->received_at;}
  static double retry(MissionManager & manager)
  {
    manager.plan_failed("测试规划重试");
    return *manager.plan_due_at_;
  }
  static double retry_delay(const MissionManager & manager) {return manager.plan_retry_delay_s_;}
  static double begin_rfid_stop(MissionManager & manager)
  {
    manager.rfid_active_ = 1;
    manager.rfid_active_card_ = 1;
    manager.begin_rfid_post_speech(manager.steady_seconds(), false);
    return *manager.rfid_release_at_;
  }
  static double rfid_duration(const MissionManager & manager) {return manager.rfid_stop_duration_s_;}
  static std::string service_rfid(MissionManager & manager)
  {
    manager.service_rfid(manager.steady_seconds());
    return manager.rfid_phase_;
  }
};
}  // namespace navigation_mission_manager

namespace nmm = navigation_mission_manager;
using ClockAccess = nmm::ManagerClockTestAccess;

class ManagerClockTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {manager = std::make_shared<nmm::MissionManager>();}
  void simulated(double seconds)
  {
    ASSERT_EQ(rcl_enable_ros_time_override(manager->get_clock()->get_clock_handle()), RCL_RET_OK);
    advance(seconds);
  }
  void advance(double seconds)
  {
    const auto clock = manager->get_clock();
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_set_ros_time_override(
        clock->get_clock_handle(), static_cast<int64_t>(seconds * 1.0e9)), RCL_RET_OK);
  }
  std::shared_ptr<nmm::MissionManager> manager;
};

TEST_F(ManagerClockTest, RealVehicleKeepsMonotonicTiming)
{
  const double before = nmm::steady_seconds();
  const double actual = ClockAccess::seconds(*manager);
  EXPECT_GE(actual, before);
  EXPECT_LE(actual, nmm::steady_seconds());
}

TEST_F(ManagerClockTest, PausedClockDoesNotAgeOdometryAndAdvanceExpiresIt)
{
  simulated(100.0);
  ClockAccess::odometry(*manager);
  EXPECT_DOUBLE_EQ(ClockAccess::received(*manager), 100.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_DOUBLE_EQ(ClockAccess::seconds(*manager), 100.0);
  EXPECT_TRUE(ClockAccess::fresh(*manager));
  advance(100.0 + ClockAccess::timeout(*manager) + 0.01);
  EXPECT_FALSE(ClockAccess::fresh(*manager));
}

TEST_F(ManagerClockTest, SchedulingTimerUsesSimulationTime)
{
  simulated(100.0);
  const auto timer = ClockAccess::timer(*manager);
  timer->reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(timer->time_until_trigger(), std::chrono::milliseconds(100));
  advance(100.09);
  EXPECT_FALSE(timer->is_ready());
  advance(100.10);
  EXPECT_TRUE(timer->is_ready());
}

TEST_F(ManagerClockTest, RfidStopAndPlanningRetryKeepConfiguredSimulationDurations)
{
  simulated(100.0);
  EXPECT_DOUBLE_EQ(ClockAccess::retry(*manager), 100.0 + ClockAccess::retry_delay(*manager));
  const double release = ClockAccess::begin_rfid_stop(*manager);
  EXPECT_DOUBLE_EQ(release, 100.0 + ClockAccess::rfid_duration(*manager));
  advance(release - 0.01);
  EXPECT_EQ(ClockAccess::service_rfid(*manager), "post_speech");
  advance(release);
  EXPECT_EQ(ClockAccess::service_rfid(*manager), "authorizing");
}
