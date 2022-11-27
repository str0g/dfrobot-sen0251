//
// Created by lukasz on 25.11.2022.
//

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
