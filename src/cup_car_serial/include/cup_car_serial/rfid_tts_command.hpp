#ifndef CUP_CAR_SERIAL__RFID_TTS_COMMAND_HPP_
#define CUP_CAR_SERIAL__RFID_TTS_COMMAND_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace cup_car_serial
{

inline std::string BuildRfidTtsCommand(uint8_t number)
{
  static const std::array<std::string, 13> number_gbk{
    "", "\xD2\xBB", "\xB6\xFE", "\xC8\xFD", "\xCB\xC4", "\xCE\xE5", "\xC1\xF9",
    "\xC6\xDF", "\xB0\xCB", "\xBE\xC5", "\xCA\xAE", "\xCA\xAE\xD2\xBB", "\xCA\xAE\xB6\xFE"};
  if (number < 1U || number > 12U) {
    return {};
  }

  std::string command = "TTS,";
  command.append("\xB5\xBD\xB4\xEF", 4);  // 到达
  command += number_gbk[number];
  command.append("\xBA\xC5\xD1\xB2\xC2\xDF\xB5\xE3", 8);  // 号巡逻点
  command += "\r\n";
  return command;
}

}  // namespace cup_car_serial

#endif  // CUP_CAR_SERIAL__RFID_TTS_COMMAND_HPP_
