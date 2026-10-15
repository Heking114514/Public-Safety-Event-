#include "cup_car_serial/tts2_protocol.hpp"

#include <limits>
#include <stdexcept>

namespace cup_car_serial
{
namespace
{
bool parse_nonzero_uint32(const std::string & text, uint32_t * value)
{
  if (value == nullptr || text.empty()) {
    return false;
  }
  uint64_t parsed = 0U;
  for (const unsigned char ch : text) {
    if (ch < static_cast<unsigned char>('0') || ch > static_cast<unsigned char>('9')) {
      return false;
    }
    parsed = parsed * 10U + static_cast<uint64_t>(ch - static_cast<unsigned char>('0'));
    if (parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
  }
  if (parsed == 0U) {
    return false;
  }
  *value = static_cast<uint32_t>(parsed);
  return true;
}

bool parse_status(const std::string & text, Tts2Status * status)
{
  if (status == nullptr) {
    return false;
  }
  if (text == "QUEUED") {
    *status = Tts2Status::QUEUED;
  } else if (text == "PLAYING") {
    *status = Tts2Status::PLAYING;
  } else if (text == "ESTIMATED_DONE") {
    *status = Tts2Status::ESTIMATED_DONE;
  } else if (text == "ERROR") {
    *status = Tts2Status::ERROR;
  } else if (text == "BUSY") {
    *status = Tts2Status::BUSY;
  } else if (text == "INVALID") {
    *status = Tts2Status::INVALID;
  } else {
    return false;
  }
  return true;
}
}  // namespace

bool parse_tts2_telemetry_frame(const std::string & line, Tts2TelemetryFrame * frame)
{
  if (frame == nullptr || line.rfind("TTS,", 0U) != 0U) {
    return false;
  }
  const size_t id_start = 4U;
  const size_t separator = line.find(',', id_start);
  if (separator == std::string::npos || line.find(',', separator + 1U) != std::string::npos) {
    return false;
  }

  Tts2TelemetryFrame parsed;
  if (!parse_nonzero_uint32(line.substr(id_start, separator - id_start), &parsed.id) ||
    !parse_status(line.substr(separator + 1U), &parsed.status))
  {
    return false;
  }
  *frame = parsed;
  return true;
}

const char * tts2_status_name(Tts2Status status)
{
  switch (status) {
    case Tts2Status::QUEUED: return "QUEUED";
    case Tts2Status::PLAYING: return "PLAYING";
    case Tts2Status::ESTIMATED_DONE: return "ESTIMATED_DONE";
    case Tts2Status::ERROR: return "ERROR";
    case Tts2Status::BUSY: return "BUSY";
    case Tts2Status::INVALID: return "INVALID";
  }
  return "INVALID";
}

std::string make_tts2_request_frame(uint32_t id, const std::string & gbk_payload)
{
  if (id == 0U || gbk_payload.empty() || gbk_payload.size() > 128U ||
    gbk_payload.find('\0') != std::string::npos ||
    gbk_payload.find_first_of("\r\n<>") != std::string::npos)
  {
    throw std::invalid_argument("TTS2 requires a nonzero id and 1 to 128 safe GBK bytes");
  }
  for (size_t i = 0U; i < gbk_payload.size(); ++i) {
    const auto ch = static_cast<unsigned char>(gbk_payload[i]);
    if (ch >= 0x81U && ch <= 0xFEU) {
      if (++i >= gbk_payload.size()) {
        throw std::invalid_argument("incomplete GBK character");
      }
      const auto trail = static_cast<unsigned char>(gbk_payload[i]);
      if (trail < 0x40U || trail > 0xFEU || trail == 0x7FU) {
        throw std::invalid_argument("invalid GBK character");
      }
    } else if (ch < 0x20U || ch > 0x7EU) {
      throw std::invalid_argument("speech payload contains a control or invalid GBK byte");
    }
  }
  return "TTS2," + std::to_string(id) + "," + gbk_payload + "\r\n";
}

Tts2IdGenerator::Tts2IdGenerator(uint32_t seed)
: next_id_(seed == 0U ? 1U : seed)
{
}

uint32_t Tts2IdGenerator::Next()
{
  const uint32_t id = next_id_;
  ++next_id_;
  if (next_id_ == 0U) {
    next_id_ = 1U;
  }
  return id;
}

Tts2RequestTracker::Tts2RequestTracker(
  uint32_t id, TimePoint started_at, Clock::duration retry_interval,
  Clock::duration retry_window, Clock::duration deadline)
: id_(id),
  next_transmit_at_(started_at),
  retry_until_(started_at + retry_window),
  deadline_(started_at + deadline),
  retry_interval_(retry_interval)
{
  if (id == 0U || retry_interval <= Clock::duration::zero() ||
    retry_window < Clock::duration::zero() || deadline <= Clock::duration::zero())
  {
    throw std::invalid_argument("invalid TTS2 request tracker configuration");
  }
}

uint32_t Tts2RequestTracker::id() const
{
  return id_;
}

bool Tts2RequestTracker::should_transmit(TimePoint now) const
{
  if (result_ != Tts2RequestResult::PENDING || accepted_ || now >= deadline_) {
    return false;
  }
  if (transmission_count_ == 0U) {
    return true;
  }
  return now < retry_until_ && now >= next_transmit_at_;
}

void Tts2RequestTracker::mark_transmitted(TimePoint now)
{
  if (!should_transmit(now)) {
    return;
  }
  ++transmission_count_;
  next_transmit_at_ = now + retry_interval_;
}

Tts2RequestResult Tts2RequestTracker::observe(
  const Tts2TelemetryFrame & frame, TimePoint now)
{
  if (frame.id != id_ || result_ != Tts2RequestResult::PENDING) {
    return result_;
  }
  if (check_deadline(now) != Tts2RequestResult::PENDING) {
    return result_;
  }

  switch (frame.status) {
    case Tts2Status::QUEUED:
    case Tts2Status::PLAYING:
      accepted_ = true;
      break;
    case Tts2Status::ESTIMATED_DONE:
      accepted_ = true;
      result_ = Tts2RequestResult::SUCCEEDED;
      break;
    case Tts2Status::ERROR:
    case Tts2Status::BUSY:
    case Tts2Status::INVALID:
      accepted_ = true;
      result_ = Tts2RequestResult::FAILED;
      break;
  }
  return result_;
}

Tts2RequestResult Tts2RequestTracker::check_deadline(TimePoint now)
{
  if (result_ == Tts2RequestResult::PENDING && now >= deadline_) {
    result_ = Tts2RequestResult::TIMED_OUT;
  }
  return result_;
}

Tts2RequestResult Tts2RequestTracker::result() const
{
  return result_;
}
}  // namespace cup_car_serial
