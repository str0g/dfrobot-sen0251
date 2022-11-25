//
// Created by lukasz on 25.11.2022.
//

#ifndef DFROBOT_SEN0251_UTILS_H
#define DFROBOT_SEN0251_UTILS_H

#include <vector>

/**
 * @return naive index list based on /dev/i2c-*
 */
std::vector<unsigned> list_i2c_devices();

#endif // DFROBOT_SEN0251_UTILS_H
