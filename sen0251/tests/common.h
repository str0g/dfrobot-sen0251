/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef DFROBOT_SEN0251_COMMON_H
#define DFROBOT_SEN0251_COMMON_H

#include <micro_logger/micro_logger.hpp>

inline void set_silent_logger_instance() {
  static micro_logger::SilentWriter writer;
  micro_logger::initialize(writer);
}

#endif // DFROBOT_SEN0251_COMMON_H
