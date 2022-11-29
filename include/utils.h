/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef DFROBOT_SEN0251_UTILS_H
#define DFROBOT_SEN0251_UTILS_H

#include <cstdint>
#include <vector>

/**
 * @return naive index list based on /dev/i2c-*
 */
std::vector<unsigned> list_i2c_devices();

float to_24bit(uint8_t b2, uint8_t b1, uint8_t b0);

#endif // DFROBOT_SEN0251_UTILS_H
