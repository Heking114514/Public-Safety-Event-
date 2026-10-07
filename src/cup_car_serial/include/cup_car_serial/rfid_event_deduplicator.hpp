#ifndef CUP_CAR_SERIAL__RFID_EVENT_DEDUPLICATOR_HPP_
#define CUP_CAR_SERIAL__RFID_EVENT_DEDUPLICATOR_HPP_

#include <cstdint>
#include <string>

namespace cup_car_serial
{

class RfidEventDeduplicator
{
public:
  bool MarkIfNew(uint32_t card_sequence, const std::string & uid)
  {
    if (initialized_ && card_sequence == card_sequence_ && uid == uid_) {
      return false;
    }
    initialized_ = true;
    card_sequence_ = card_sequence;
    uid_ = uid;
    return true;
  }

  void Reset()
  {
    initialized_ = false;
    card_sequence_ = 0;
    uid_.clear();
  }

private:
  bool initialized_{false};
  uint32_t card_sequence_{0};
  std::string uid_;
};

}  // namespace cup_car_serial

#endif  // CUP_CAR_SERIAL__RFID_EVENT_DEDUPLICATOR_HPP_
