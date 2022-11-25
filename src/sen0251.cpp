#include <cstring>
#include <cerrno>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>

#include "sen0251.h"

Sen0251::Sen0251(unsigned dev) {
  char filename[255];

  snprintf(filename, sizeof(filename), "/dev/i2c-%u", dev);
  file = open(filename, O_RDWR);
  if (not file) {
    throw std::runtime_error(strerror_l(errno, NULL));
  }
}

Sen0251::~Sen0251() { close(file); }
