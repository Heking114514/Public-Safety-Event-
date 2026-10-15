#ifndef CUP_CAR_SERIAL__TTS2_PROTOCOL_HPP_
#define CUP_CAR_SERIAL__TTS2_PROTOCOL_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace cup_car_serial
{
enum class Tts2Status
{
  QUEUED,
  PLAYING,
  ESTIMATED_DONE,
  ERROR,
  BUSY,
  INVALID
};

struct Tts2TelemetryFrame
{
  uint32_t id{};
  Tts2Status status{Tts2Status::INVALID};
};

enum class Tts2RequestResult
{
  PENDING,
  SUCCEEDED,
  FAILED,
  TIMED_OUT
};

bool parse_tts2_telemetry_frame(const std::string & line, Tts2TelemetryFrame * frame);
const char * tts2_status_name(Tts2Status status);
std::string make_tts2_request_frame(uint32_t id, const std::string & gbk_payload);

class Tts2IdGenerator
{
public:
  explicit Tts2IdGenerator(uint32_t seed);
  uint32_t Next();

private:
  uint32_t next_id_;
};

class Tts2RequestTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  Tts2RequestTracker(
    uint32_t id, TimePoint started_at, Clock::duration retry_interval,
    Clock::duration retry_window, Clock::duration deadline);

  uint32_t id() const;
  bool should_transmit(TimePoint now) const;
  void mark_transmitted(TimePoint now);
  Tts2RequestResult observe(const Tts2TelemetryFrame & frame, TimePoint now);
  Tts2RequestResult check_deadline(TimePoint now);
  Tts2RequestResult result() const;

private:
  uint32_t id_;
  TimePoint next_transmit_at_;
  TimePoint retry_until_;
  TimePoint deadline_;
  Clock::duration retry_interval_;
  uint32_t transmission_count_{0};
  bool accepted_{false};
  Tts2RequestResult result_{Tts2RequestResult::PENDING};
};
}  // namespace cup_car_serial

#endif  // CUP_CAR_SERIAL__TTS2_PROTOCOL_HPP_
