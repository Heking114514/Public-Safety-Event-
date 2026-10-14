#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "cup_car_serial/tts2_protocol.hpp"
#include "gtest/gtest.h"

TEST(Tts2Protocol, ParsesAllRecognizedStatuses)
{
  using cup_car_serial::Tts2Status;
  const struct
  {
    const char * line;
    Tts2Status status;
  } cases[] = {
    {"TTS,42,QUEUED", Tts2Status::QUEUED},
    {"TTS,42,PLAYING", Tts2Status::PLAYING},
    {"TTS,42,ESTIMATED_DONE", Tts2Status::ESTIMATED_DONE},
    {"TTS,42,ERROR", Tts2Status::ERROR},
    {"TTS,42,BUSY", Tts2Status::BUSY},
    {"TTS,42,INVALID", Tts2Status::INVALID}};

  for (const auto & item : cases) {
    cup_car_serial::Tts2TelemetryFrame frame;
    ASSERT_TRUE(cup_car_serial::parse_tts2_telemetry_frame(item.line, &frame));
    EXPECT_EQ(frame.id, 42U);
    EXPECT_EQ(frame.status, item.status);
  }
}

TEST(Tts2Protocol, RejectsMalformedIdsAndStatuses)
{
  cup_car_serial::Tts2TelemetryFrame frame;
  const char * malformed[] = {
    "TTS,,QUEUED", "TTS,0,QUEUED", "TTS,-1,QUEUED", "TTS,+1,QUEUED",
    "TTS, 1,QUEUED", "TTS,4294967296,QUEUED", "TTS,1x,QUEUED",
    "TTS,1,UNKNOWN", "TTS,1,queued", "TTS,1,QUEUED,extra", "TTS,1,"};
  for (const char * line : malformed) {
    EXPECT_FALSE(cup_car_serial::parse_tts2_telemetry_frame(line, &frame)) << line;
  }
  EXPECT_FALSE(cup_car_serial::parse_tts2_telemetry_frame("TTS,1,QUEUED", nullptr));
}

TEST(Tts2Protocol, BuildsBoundedRequestFramesAndSkipsZeroIds)
{
  EXPECT_EQ(
    cup_car_serial::make_tts2_request_frame(42U, "\xD2\xBB\xBA\xC5"),
    std::string("TTS2,42,\xD2\xBB\xBA\xC5\r\n", 14U));
  EXPECT_THROW(cup_car_serial::make_tts2_request_frame(0U, "x"), std::invalid_argument);
  EXPECT_THROW(cup_car_serial::make_tts2_request_frame(1U, ""), std::invalid_argument);
  EXPECT_THROW(cup_car_serial::make_tts2_request_frame(1U, std::string(129U, 'x')), std::invalid_argument);
  EXPECT_THROW(cup_car_serial::make_tts2_request_frame(1U, std::string("x\0y", 3U)), std::invalid_argument);
  EXPECT_THROW(cup_car_serial::make_tts2_request_frame(1U, "x\ny"), std::invalid_argument);
  EXPECT_THROW(cup_car_serial::make_tts2_request_frame(1U, "x<y"), std::invalid_argument);

  cup_car_serial::Tts2IdGenerator ids(std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(ids.Next(), std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(ids.Next(), 1U);
  cup_car_serial::Tts2IdGenerator zero_seed(0U);
  EXPECT_EQ(zero_seed.Next(), 1U);
}

TEST(Tts2Protocol, IgnoresStaleTelemetryAndCompletesOnlyAtEstimatedDone)
{
  using namespace std::chrono_literals;
  using cup_car_serial::Tts2RequestResult;
  using cup_car_serial::Tts2Status;
  const auto started = cup_car_serial::Tts2RequestTracker::TimePoint{};
  cup_car_serial::Tts2RequestTracker tracker(17U, started, 200ms, 1s, 120s);

  EXPECT_EQ(tracker.observe({16U, Tts2Status::ESTIMATED_DONE}, started + 2s),
    Tts2RequestResult::PENDING);
  EXPECT_EQ(tracker.observe({17U, Tts2Status::QUEUED}, started + 3s),
    Tts2RequestResult::PENDING);
  EXPECT_FALSE(tracker.should_transmit(started + 3s));
  EXPECT_EQ(tracker.observe({17U, Tts2Status::PLAYING}, started + 4s),
    Tts2RequestResult::PENDING);
  EXPECT_EQ(tracker.observe({17U, Tts2Status::ESTIMATED_DONE}, started + 5s),
    Tts2RequestResult::SUCCEEDED);
}

TEST(Tts2Protocol, RetriesSameIdOnlyUntilAcceptanceWindowAndEnforcesDeadline)
{
  using namespace std::chrono_literals;
  const auto started = cup_car_serial::Tts2RequestTracker::TimePoint{};
  cup_car_serial::Tts2RequestTracker tracker(900U, started, 200ms, 1s, 120s);

  ASSERT_TRUE(tracker.should_transmit(started));
  tracker.mark_transmitted(started);
  EXPECT_FALSE(tracker.should_transmit(started + 199ms));
  ASSERT_TRUE(tracker.should_transmit(started + 200ms));
  tracker.mark_transmitted(started + 200ms);
  EXPECT_EQ(tracker.id(), 900U);
  EXPECT_TRUE(tracker.should_transmit(started + 400ms));
  tracker.mark_transmitted(started + 400ms);
  EXPECT_TRUE(tracker.should_transmit(started + 600ms));
  tracker.mark_transmitted(started + 600ms);
  EXPECT_TRUE(tracker.should_transmit(started + 800ms));
  tracker.mark_transmitted(started + 800ms);
  EXPECT_FALSE(tracker.should_transmit(started + 1s));
  EXPECT_FALSE(tracker.should_transmit(started + 10s));
  EXPECT_EQ(tracker.check_deadline(started + 119s), cup_car_serial::Tts2RequestResult::PENDING);
  EXPECT_EQ(tracker.check_deadline(started + 120s), cup_car_serial::Tts2RequestResult::TIMED_OUT);
}

TEST(Tts2Protocol, BusyInvalidAndErrorStatusesFail)
{
  using namespace std::chrono_literals;
  using cup_car_serial::Tts2RequestResult;
  const auto started = cup_car_serial::Tts2RequestTracker::TimePoint{};
  for (const auto status : {cup_car_serial::Tts2Status::BUSY,
    cup_car_serial::Tts2Status::INVALID, cup_car_serial::Tts2Status::ERROR})
  {
    cup_car_serial::Tts2RequestTracker tracker(1U, started, 200ms, 1s, 120s);
    EXPECT_EQ(tracker.observe({1U, status}, started + 1ms), Tts2RequestResult::FAILED);
  }
}
