#include <cstring>
#include <cerrno>
#include <iostream>

extern "C" {
#include <unistd.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

#include "sen0251.h"
#include "utils.h"
#include "sen0251_exception.h"

#include "../dependecies/micro-logger/includes/micro_logger.hpp"

namespace Register {
enum {
  id = 0x00,
  pressure = 0x04,
  temperature = 0x07,
  event = 0x10,
  fifo_ready = 0x12,
  fifo_size = 0x13,
  config = 0x1F,
  odr = 0x1D,
  oversampling = 0x1C,
  pwr_ctrl = 0x1B,
  int_ctrl = 0x19,
  calib00 = 0x31,
  cmd = 0x7E,
  status = 0x03,
  err = 0x02,
};
}

namespace Status {
enum {
  fatal_error = 0x01,
  command_error = 0x02,
  configuration_error = 0x04,
  command_ready = 0x10,
  pressure_ready = 0x20,
  temperature_ready = 0x40,
};
const char* to_string(int flag) {
  const char *p = nullptr;
  if (flag & fatal_error) p = "fatal_error";
  if (flag & command_error) p = "command_error";
  if (flag & configuration_error) p = "configuration_error";
  if (flag & command_ready) p = "command_ready";
  if (flag & pressure_ready) p = "pressure_ready";
  if (flag & temperature_ready) p = "temperature_ready";

  return p;
}
}

std::string PowerControl::to_string(int flag) {
  std::string rc;
  rc += "\n\tpressure:    " + std::to_string(bool(flag & PowerControl::pressure_on));
  rc += "\n\ttemperature: " + std::to_string(bool(flag & PowerControl::temperature_on));
  if (flag & PowerControl::normal)
    rc += "\n\tnormal:      " + std::to_string(bool(flag & PowerControl::normal));
  else
    rc += "\n\tforce:       " + std::to_string((flag & PowerControl::force_on_a) && (flag & PowerControl::force_on_b));

  return rc;
}

Sen0251::Sen0251(unsigned dev, unsigned address) {
  MSG_ENTER();

  char filename[255];

  snprintf(filename, sizeof(filename), "/dev/i2c-%u", dev);
  file = open(filename, O_RDWR);
  if (file <= 0) {
    MSG_CRITICAL("%s %s", strerror_l(errno, NULL), filename);
    throw std::runtime_error(strerror_l(errno, NULL));
  }
  if (ioctl(file, I2C_SLAVE, address) < 0) {
    MSG_CRITICAL("%s %s", strerror_l(errno, NULL), filename);
    throw std::runtime_error(strerror_l(errno, NULL));
  }
  //reset device
  command(Commands::softreset);
  if(not event()) {
    THROW(OperationError, "bit has not been set, is device correct?");
  }

  filter_coefficient = get_filter_coefficient();

  MSG_EXIT();
}

Sen0251::~Sen0251() { close(file); }

unsigned char Sen0251::get_chip_id() const {
  auto rc = i2c_smbus_read_word_data(file, Register::id);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  return (rc >> 2);
}

const char* Sen0251::get_error() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_word_data(file, Register::err);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  MSG_DEBUG("%d", rc);
  MSG_DEBUG("%s", Status::to_string(rc));

  auto ret = Status::to_string(rc);

  MSG_EXIT();

  return ret;
}

const char* Sen0251::get_status() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_word_data(file, Register::status);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  MSG_DEBUG("%d", rc);
  MSG_DEBUG("%s", Status::to_string(rc));

  auto ret = Status::to_string(rc);

  MSG_EXIT();
  return ret;
}

int Sen0251::get_temperature() {
  auto p1 = i2c_smbus_read_word_data(file, Register::temperature);//7-0
  std::cout << p1 << std::endl;
  auto p2 = i2c_smbus_read_word_data(file, Register::temperature+1);//15-8
  std::cout << p2 << std::endl;
  auto p3 = i2c_smbus_read_word_data(file, Register::temperature+2);//24-15
  std::cout << p3 << std::endl;
  return p3+p2+p1;
}

int Sen0251::get_pressure() {
  auto p1 = i2c_smbus_read_word_data(file, Register::pressure);//7-0
  std::cout << p1 << std::endl;
  auto p2 = i2c_smbus_read_word_data(file, Register::pressure+1);//15-8
  std::cout << p2 << std::endl;
  auto p3 = i2c_smbus_read_word_data(file, Register::pressure+2);//24-15
  std::cout << p3 << std::endl;
  return p3+p2+p1;
}

int Sen0251::get_osr() const {
  auto rc = i2c_smbus_read_word_data(file, Register::oversampling);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  return rc;
}

void Sen0251::power_control(unsigned char flag) {
  MSG_ENTER();

  MSG_DEBUG("flag: %d", static_cast<int>(flag));
  auto rc = i2c_smbus_write_byte_data(file, Register::pwr_ctrl, flag);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();
}

void Sen0251::command(Commands::cmd_t cmd) {
  MSG_ENTER();

  switch (cmd) {
  case Commands::nop:
  case Commands::extmode_en_middle:
  case Commands::fifo_flush:
  case Commands::softreset:
    break;
  default:
    MSG_ERROR("unsupported command %d", static_cast<int>(cmd));
    THROW(InvalidInputError, "unsupported command");
  }
  auto rc = i2c_smbus_write_byte_data(file, Register::cmd, cmd);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();
}

int Sen0251::get_fifo_size() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_word_data(file, Register::fifo_ready);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  } else if (rc > 0) {
    rc = i2c_smbus_read_word_data(file, Register::fifo_size);
    if (rc < 0) {
      MSG_WARN("fail to read: %d, %s", rc, get_error());
    }
  }

  MSG_EXIT();

  return rc;
}

bool Sen0251::event() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_word_data(file, Register::event);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();

  return rc;
}

unsigned char Sen0251::get_filter_coefficient() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_word_data(file, Register::config);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  switch (rc) {
  case 0x00:
    rc = 1;
    break;
  case 0x01:
    rc = 1;
    break;
  case 0x02:
    rc = 3;
    break;
  case 0x03:
    rc = 7;
    break;
  case 0x04:
    rc = 15;
    break;
  case 0x05:
    rc = 31;
    break;
  case 0x06:
    rc = 63;
    break;
  case 0x07:
    rc = 127;
    break;
  }

  MSG_EXIT();

  return rc;
}

void Sen0251::set_oversampling(Oversampling::oversampling_t pressure, Oversampling::oversampling_t temperature) {
  MSG_ENTER();

  auto rc = i2c_smbus_write_byte_data(file, Register::oversampling, (temperature << 3) | pressure);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();
}

unsigned char Sen0251::get_power() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_word_data(file, Register::pwr_ctrl);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();

  return rc;
}