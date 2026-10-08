#include "cup_car_serial/rfid_event_deduplicator.hpp"

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
