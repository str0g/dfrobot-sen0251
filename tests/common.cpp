/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */


#include "common.h"
#include "micro_logger.hpp"

void setSilentLoggerInstance() {
  static micro_logger::SilentWriter writer;
  micro_logger::set_writer(writer);
}