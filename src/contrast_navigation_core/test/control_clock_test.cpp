#include <gtest/gtest.h>
#include <rcl/time.h>
#include "visual_navigation/control_clock.hpp"
#include "visual_navigation/recovery_stop.hpp"

namespace {
using Clock = std::chrono::steady_clock;

class ControlClockTest : public ::testing::Test {
protected:
  rclcpp::Clock clock_{RCL_ROS_TIME};

  void enable_simulation() {
    ASSERT_EQ(rcl_enable_ros_time_override(clock_.get_clock_handle()), RCL_RET_OK);
  }

  void set_time(int64_t nanoseconds) {
    ASSERT_EQ(rcl_set_ros_time_override(clock_.get_clock_handle(), nanoseconds),
              RCL_RET_OK);
  }

  double seconds() {
    return std::chrono::duration<double>(
        visual_navigation::ControlTime(clock_).time_since_epoch()).count();
  }
};

TEST_F(ControlClockTest, PhysicalVehicleKeepsSteadyClock) {
  const auto before = Clock::now();
  const auto control_time = visual_navigation::ControlTime(clock_);
  const auto after = Clock::now();
  EXPECT_GE(control_time, before);
  EXPECT_LE(control_time, after);
}

TEST_F(ControlClockTest, SimulationUsesClockTimeAndRemainsStillWithoutClockAdvance) {
  enable_simulation();
  set_time(0);
  EXPECT_EQ(visual_navigation::ControlTime(clock_), Clock::time_point{});
  set_time(1000000000LL);
  const auto first = visual_navigation::ControlTime(clock_);
  EXPECT_EQ(visual_navigation::ControlTime(clock_), first);
  // 倍率由 /clock 发布端控制；控制器只认已推进的仿真时间，不另乘倍率。
  set_time(3500000000LL);
  EXPECT_DOUBLE_EQ(std::chrono::duration<double>(
      visual_navigation::ControlTime(clock_) - first).count(), 2.5);
}

TEST_F(ControlClockTest, RecoveryStopDwellIsMeasuredInSimulationSeconds) {
  enable_simulation();
  set_time(10000000000LL);
  visual_navigation::RecoveryStopGuard guard;
  // 阈值来自 final_navigation.yaml，样本数为测试夹具显式指定。
  guard.configure(0.002, 0.03, 0.22, 2.0, 3);
  guard.begin(seconds(), 100);
  using Result = visual_navigation::RecoveryStopGuard::Result;
  set_time(10010000000LL);
  EXPECT_EQ(guard.update(seconds(), 101, true, 0, 0, 0, 0, 0), Result::WAITING);
  set_time(10100000000LL);
  EXPECT_EQ(guard.update(seconds(), 102, true, 0, 0, 0, 0, 0), Result::WAITING);
  // 仿真暂停时，即使新样本到达也不能凑够停车驻留时长。
  EXPECT_EQ(guard.update(seconds(), 103, true, 0, 0, 0, 0, 0), Result::WAITING);
  set_time(10240000000LL);
  EXPECT_EQ(guard.update(seconds(), 104, true, 0, 0, 0, 0, 0), Result::STOPPED);
}

} // namespace
