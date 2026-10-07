#include "cup_car_serial/rfid_event_deduplicator.hpp"
#include "cup_car_serial/rfid_tts_command.hpp"

#include <gtest/gtest.h>

TEST(RfidEventDeduplicator, SuppressesRepeatedSequenceAndUid)
{
  cup_car_serial::RfidEventDeduplicator deduplicator;

  EXPECT_TRUE(deduplicator.MarkIfNew(7, "56C68E89"));
  EXPECT_FALSE(deduplicator.MarkIfNew(7, "56C68E89"));
  EXPECT_TRUE(deduplicator.MarkIfNew(8, "56C68E89"));
  EXPECT_TRUE(deduplicator.MarkIfNew(8, "01020304"));
  EXPECT_FALSE(deduplicator.MarkIfNew(8, "01020304"));

  deduplicator.Reset();
  EXPECT_TRUE(deduplicator.MarkIfNew(7, "56C68E89"));
}

TEST(RfidTtsCommand, BuildsMcuGbkAnnouncementFrame)
{
  const std::string expected = "TTS,\xB5\xBD\xB4\xEF\xD2\xBB\xBA\xC5\xD1\xB2\xC2\xDF\xB5\xE3\r\n";
  EXPECT_EQ(expected, cup_car_serial::BuildRfidTtsCommand(1));
  EXPECT_TRUE(cup_car_serial::BuildRfidTtsCommand(0).empty());
  EXPECT_TRUE(cup_car_serial::BuildRfidTtsCommand(13).empty());
}
