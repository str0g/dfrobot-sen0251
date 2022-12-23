/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <gtest/gtest.h>
#include "../include/sen0251.h"
#include "common.h"

class TemperatureTest : public ::testing::Test {
  void SetUp() {
    setSilentLoggerInstance();
  }
};

TEST_F(TemperatureTest, temperature_zero) {
  Sen0251::Readings readings {};
  float calibration[] = {6.98598e+06, 1.74791e-05, 8.73968e-13};
  float from_sensor = 6985980.0f; // == 6.98598e+06

  TestSen0251 obj;
  obj.update_temperature(readings, from_sensor, calibration);

  ASSERT_FLOAT_EQ(readings.temperature, 0);
}

TEST_F(TemperatureTest, temperature_25) {
  Sen0251::Readings readings {};
  float calibration[] = {6.98598e+06, 1.74791e-05, 8.73968e-13};
  float from_sensor = 8326600.0f;

  TestSen0251 obj;
  obj.update_temperature(readings, from_sensor, calibration);

  ASSERT_NEAR(readings.temperature, 25.0f, 0.01f);
}