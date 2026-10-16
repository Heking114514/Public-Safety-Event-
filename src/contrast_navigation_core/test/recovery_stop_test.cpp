#include <gtest/gtest.h>
#include <limits>
#include "visual_navigation/recovery_stop.hpp"

namespace {
using Guard=visual_navigation::RecoveryStopGuard;
using Result=Guard::Result;
Guard guard() {
  Guard value;
  // 测试阈值来自 final_navigation.yaml；连续样本数由本测试显式指定。
  value.configure(0.002,0.03,0.22,2.0,3);
  value.begin(10.0,100);
  return value;
}
}

TEST(RecoveryStop, RequiresFreshSamplesAndMeasuredDwell) {
  auto value=guard();
  EXPECT_EQ(value.update(10.01,101,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.10,102,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.24,103,true,0,0,0,0,0),Result::STOPPED);
}

TEST(RecoveryStop, RepeatedCachedZeroNeverConfirmsStopAndHasIndependentTimeout) {
  auto value=guard();
  for (double now:{10.1,10.5,11.0,11.9})
    EXPECT_EQ(value.update(now,100,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(12.0,100,true,0,0,0,0,0),Result::TIMED_OUT);
}

TEST(RecoveryStop, StaleZerosResetTheConsecutiveEvidence) {
  auto value=guard();
  EXPECT_EQ(value.update(10.01,101,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.10,102,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.24,103,false,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.30,104,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.40,105,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.53,106,true,0,0,0,0,0),Result::STOPPED);
}

TEST(RecoveryStop, MovingWheelTargetOrYawCannotBeCalledStopped) {
  for (int moving=0;moving<5;++moving) {
    auto value=guard();
    double left=0,right=0,target_left=0,target_right=0,yaw=0;
    if(moving==0) left=0.004;
    if(moving==1) right=-0.004;
    if(moving==2) target_left=0.004;
    if(moving==3) target_right=-0.004;
    if(moving==4) yaw=0.05;
    for(unsigned sample=101;sample<110;++sample)
      EXPECT_EQ(value.update(10.0+(sample-100)*0.15,sample,true,
        left,right,target_left,target_right,yaw),Result::WAITING);
    EXPECT_EQ(value.update(12.0,110,true,left,right,target_left,target_right,yaw),Result::TIMED_OUT);
  }
}

TEST(RecoveryStop, NewMovementRestartsDwellRatherThanReusingEarlierStop) {
  auto value=guard();
  value.update(10.01,101,true,0,0,0,0,0);
  value.update(10.10,102,true,0,0,0,0,0);
  EXPECT_EQ(value.update(10.20,103,true,0.01,0.01,0,0,0),Result::WAITING);
  value.update(10.25,104,true,0,0,0,0,0);
  value.update(10.35,105,true,0,0,0,0,0);
  EXPECT_EQ(value.update(10.40,106,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(10.48,107,true,0,0,0,0,0),Result::STOPPED);
}

TEST(RecoveryStop, NonfiniteWheelTelemetryDoesNotConfirmStop) {
  const auto nan=std::numeric_limits<double>::quiet_NaN();
  for (int invalid=0;invalid<5;++invalid) {
    auto value=guard();
    double channels[]{0,0,0,0,0}; channels[invalid]=nan;
    for(unsigned sample=101;sample<105;++sample)
      EXPECT_EQ(value.update(10.0+(sample-100)*0.15,sample,true,
        channels[0],channels[1],channels[2],channels[3],channels[4]),Result::WAITING);
    EXPECT_EQ(value.update(12.0,105,true,channels[0],channels[1],channels[2],channels[3],channels[4]),
      Result::TIMED_OUT);
  }
}

TEST(RecoveryStop, EntryStopTransferRequiresNewFreshDwell) {
  auto value=guard();
  value.update(10.01,101,true,0,0,0,0,0);
  value.update(10.10,102,true,0,0,0,0,0);
  // 正常入口停车中遇障：切换恢复时钟，禁止复用切换前的停车样本。
  value.begin(0.0,102);
  EXPECT_EQ(value.update(.01,102,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(.02,103,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(.12,104,true,0,0,0,0,0),Result::WAITING);
  EXPECT_EQ(value.update(.25,105,true,0,0,0,0,0),Result::STOPPED);
}
