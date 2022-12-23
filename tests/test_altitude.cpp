/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <gtest/gtest.h>
#include "../include/sen0251.h"
#include "common.h"

class AltitudeTest : public ::testing::Test {
  void SetUp() {
    setSilentLoggerInstance();
  }
};

TEST_F(AltitudeTest, altitude_zero) {
  Sen0251::Readings readings { 25.0f, 1013.25f, 0 };

  TestSen0251 obj;
  obj.update_altitude(readings);

  ASSERT_FLOAT_EQ(readings.altitude, 0);
}

TEST_F(AltitudeTest, altitude_negative_158) {
  Sen0251::Readings readings { 26.0f, 995.00f, 0 };

  TestSen0251 obj;
  obj.update_altitude(readings);

  ASSERT_NEAR(readings.altitude, 158.88f, 0.01f);
}

TEST_F(AltitudeTest, altitude_negative_58) {
  Sen0251::Readings readings { 26.0f, 1020.0f, 0 };

  TestSen0251 obj;
  obj.update_altitude(readings);

  ASSERT_NEAR(readings.altitude, -58.18f, 0.01f);
}

TEST_F(AltitudeTest, altitude_recalibrate_sea_level_0) {
  Sen0251::Readings readings { 27.0f, 1025.0f, 0 };

  TestSen0251 obj;
  obj.obj.sea_level_pressure_adjust(1025.0f);
  obj.update_altitude(readings);

  ASSERT_FLOAT_EQ(readings.altitude, 0);
}

TEST_F(AltitudeTest, altitude_recalibrate_sea_level_158) {
  Sen0251::Readings readings { 27.0f, 1013.0f, 0 };

  TestSen0251 obj;
  obj.obj.sea_level_pressure_adjust(1025.0f);
  obj.update_altitude(readings);

  ASSERT_NEAR(readings.altitude, 103.35f, 0.01f);
}