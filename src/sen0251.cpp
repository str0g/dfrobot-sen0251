/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cstring>
#include <cerrno>
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>

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

using namespace std::chrono_literals;

std::vector<unsigned char> Supported_devices {0x50};

namespace Register {
enum {
  id = 0x00,
  pressure = 0x04,
  temperature = 0x07,
  event = 0x10,
  fifo_ready = 0x12,
  fifo_size = 0x13,
  iir_filter = 0x1F,
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

namespace Bit {
union Iir_Filter {
  struct {
    uint8_t reserved : 1;
    uint8_t filter : 3;
  } bit;
  uint8_t data;
};

union Int24 {
  struct {
    uint8_t int24[3];
  } bit;
  int data;
};
}

std::string PowerControl::to_string(int flag) {
  std::string rc;
  rc += "\n\tpressure:    " + std::to_string(bool(flag & PowerControl::pressure_on));
  rc += "\n\ttemperature: " + std::to_string(bool(flag & PowerControl::temperature_on));
  if (flag & PowerControl::normal) {
    rc += "\n\tforce:       0";
    rc += "\n\tnormal:      " + std::to_string(bool(flag & PowerControl::normal));
  } else {
    rc += "\n\tforce:       " + std::to_string((flag & PowerControl::force_on_a) && (flag & PowerControl::force_on_b));
    rc += "\n\tnormal:      0";
  }

  return rc;
}

Sen0251::Sen0251(unsigned dev, unsigned address) : sea_level_pressure(::sea_level_pressure) {
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
  auto id = get_chip_id();
  bool found = false;
  for(auto device : Supported_devices) {
    if (id == device) {
      found = true;
      break;
    }
  }
  if (not found) {
    THROW(DeviceNotSupported, "if device is BMP388, add it to support list");
  }

  soft_reset();

  set_iir_filter(IirFilter::coef_0);
  set_calibration_data();

  MSG_EXIT();
}

Sen0251::~Sen0251() { close(file); }

unsigned char Sen0251::get_chip_id() const {
  auto rc = i2c_smbus_read_byte_data(file, Register::id);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  return rc;
}

const char* Sen0251::get_error() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_byte_data(file, Register::err);
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

  auto rc = i2c_smbus_read_byte_data(file, Register::status);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  MSG_DEBUG("%d", rc);
  MSG_DEBUG("%s", Status::to_string(rc));

  auto ret = Status::to_string(rc);

  MSG_EXIT();

  return ret;
}

float Sen0251::_read_temperature_register() const {
  MSG_ENTER();

  Bit::Int24 reading {0};
  int cnt = 3;
  do {
    auto rc = i2c_smbus_read_i2c_block_data(file, Register::temperature,
                                            sizeof(reading.bit.int24),
                                            &reading.bit.int24[0]);
    if (rc < 0 or rc != sizeof(reading.bit.int24)) {
      MSG_WARN("fail to read: %d, %s", rc, get_error());
    }
    std::this_thread::sleep_for(10ms);
  } while(--cnt);
  reading.data = le32toh(reading.data);
  MSG_DEBUG("t1: %d t2: %d t3: %d, reading: %d", reading.bit.int24[0], reading.bit.int24[1], reading.bit.int24[2], reading.data);

  MSG_EXIT();
  return static_cast<float>(reading.data);
}

void Sen0251::set_temperature(Readings& obj, float temperature) const {
  MSG_ENTER();

  auto temp_part1 = temperature - temperature_calibration[0];
  auto temp_part2 = temp_part1 * temperature_calibration[1];

  obj.temperature = temp_part2 + (temp_part1 * temp_part1) * temperature_calibration[2];
  MSG_DEBUG("tp1: %f tp2: %f sensor: %f temp: %f", temp_part1, temp_part2, temperature, obj.temperature);

  MSG_EXIT();
}

float Sen0251::_read_pressure_register() const {
  MSG_ENTER();

  Bit::Int24 reading {0};
  auto rc = i2c_smbus_read_i2c_block_data(file, Register::pressure,
                                          sizeof(reading.bit.int24), &reading.bit.int24[0]);
  if (rc < 0 or rc != sizeof(reading.bit.int24)) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }
  reading.data = le32toh(reading.data);

  MSG_DEBUG("p1: %d p2: %d p3: %d, reading: %d", reading.bit.int24[0], reading.bit.int24[1], reading.bit.int24[2], reading.data);

  MSG_EXIT();
  return static_cast<float>(reading.data);
}

void Sen0251::set_pressure(Readings& obj, float pressure) const {
  MSG_ENTER();

  auto temperature_compensation = obj.temperature;

  float partial_out_1;
  {
    auto part1 = pressure_calibration[5] * temperature_compensation;
    auto part2 = pressure_calibration[6] * (temperature_compensation * temperature_compensation);
    auto part3 = pressure_calibration[7] * (temperature_compensation * temperature_compensation * temperature_compensation);
    partial_out_1 = pressure_calibration[4] + part1 + part2 + part3;
  }

  float partial_out_2;
  {
    auto part1 = pressure_calibration[1] * temperature_compensation;
    auto part2 = pressure_calibration[2] * (temperature_compensation * temperature_compensation);
    auto part3 = pressure_calibration[3] * (temperature_compensation * temperature_compensation * temperature_compensation);
    partial_out_2 = (pressure_calibration[0] + part1 + part2 + part3) * pressure;
  }

  float partial_out_3;
  {
    auto part1 = pressure * pressure;
    auto part2 = pressure_calibration[8] + pressure_calibration[9] * temperature_compensation;
    auto part3 = part1 * part2;
    partial_out_3 = part3 + (part1 * pressure) * pressure_calibration[10];
  }

  obj.pressure = (partial_out_1 + partial_out_2 + partial_out_3) * 0.01f; // to millibars

  MSG_DEBUG("pp1: %f pp2: %f pp3: %f, press: %f", partial_out_1, partial_out_2, partial_out_3, obj.pressure);

  MSG_EXIT();
}

void Sen0251::set_altitude(Readings& obj) const {
  MSG_ENTER();

  /// https://www.mide.com/air-pressure-at-altitude-calculator
  constexpr float molar_mass = 0.0289644f;
  constexpr float universal_gas_constant = 8.31432f;
  constexpr float std_temperature_lapse_rate = -0.0065f;
  constexpr float gravitation = 9.80665f;
  constexpr float earth_const = (universal_gas_constant * std_temperature_lapse_rate * -1.0f) / (gravitation * molar_mass);
  constexpr float kelvin = 273.15;

  obj.altitude = ((obj.temperature + kelvin) / std_temperature_lapse_rate) * (powf((obj.pressure / sea_level_pressure), earth_const) - 1.0f);

  MSG_EXIT();
}

Sen0251::Readings Sen0251::get_readings() const {
  MSG_ENTER();

  Readings obj;

  set_temperature(obj, _read_temperature_register());
  set_pressure(obj, _read_pressure_register());
  set_altitude(obj);

  MSG_EXIT();

  return obj;
}

void Sen0251::get_oversampling(unsigned char& temperature, unsigned char& pressure) const {
  auto rc = i2c_smbus_read_byte_data(file, Register::oversampling);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_DEBUG("%d", rc);
  temperature = rc >> 3;
  pressure =  rc ^ (temperature << 3);
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

  auto rc = i2c_smbus_read_byte_data(file, Register::fifo_ready);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  } else if (rc > 0) {
    rc = i2c_smbus_read_byte_data(file, Register::fifo_size);
    if (rc < 0) {
      MSG_WARN("fail to read: %d, %s", rc, get_error());
    }
  }

  MSG_EXIT();

  return rc;
}

bool Sen0251::event() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_byte_data(file, Register::event);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();

  return rc;
}

void Sen0251::set_calibration_data() {
  MSG_ENTER();

  set_temperature_calibration();
  set_pressure_calibration();

  MSG_EXIT();
}

void Sen0251::_read_calibration_register(float& out, double coefficient, unsigned char address1, unsigned char address2, bool sign, double limiter) {
  auto nvm = i2c_smbus_read_byte_data(file, address1);
  if (nvm < 0) {
    MSG_WARN("fail to read: %d, %s 0x%x", nvm, get_error(), address1);
  }
  if (address2) {
    auto nvm2 = i2c_smbus_read_byte_data(file, address2);
    if (nvm2 < 0) {
      MSG_WARN("fail to read: %d, %s 0x%x", nvm2, get_error(), address2);
    }
    auto tmp = le32toh((nvm2 << 8) | nvm);
    nvm = (sign ? static_cast<int16_t>(tmp) : static_cast<uint16_t>(tmp));
  }
  MSG_DEBUG("nvm: %d, 0x%x", nvm, address1);
  out = ((static_cast<double >(nvm) - limiter) / coefficient);
}

void Sen0251::set_temperature_calibration() {
  MSG_ENTER();
  {
    constexpr double res = 1L<<48;
    _read_calibration_register(temperature_calibration[2], res, 0x35);
  }
  {
    constexpr double res = 1<<30;
    _read_calibration_register(temperature_calibration[1], res, 0x33, 0x34);
  }
  {
    constexpr auto res{0.00390625}; // 2^-8
    _read_calibration_register(temperature_calibration[0], res, 0x31, 0x32);
  }

  MSG_EXIT();
}

void Sen0251::set_pressure_calibration() {
  MSG_ENTER();
  int i = sizeof(pressure_calibration)/sizeof(pressure_calibration[0])-1;
  {
    double res = powf(2, 65);
    _read_calibration_register(pressure_calibration[i], res, 0x45);
  }

  {
    constexpr double res = 1L<<48;
    _read_calibration_register(pressure_calibration[--i], res, 0x44);
  }

  {
    constexpr double res = 1L<<48;
    _read_calibration_register(pressure_calibration[--i], res, 0x42, 0x43);
  }

  {
    constexpr double res = 1L<<15;
    _read_calibration_register(pressure_calibration[--i], res, 0x41);
  }

  {
    constexpr double res = 1L<<8;
    _read_calibration_register(pressure_calibration[--i], res, 0x40);
  }

  {
    constexpr double res = 1<<6;
    _read_calibration_register(pressure_calibration[--i], res, 0x3E, 0x3F);
  }

  {
    constexpr double res = 0.125; //2^-3
    _read_calibration_register(pressure_calibration[--i], res, 0x3C, 0x3D);
  }

  {
    constexpr double res = 1L<<37;
    _read_calibration_register(pressure_calibration[--i], res, 0x3B);
  }

  {
    constexpr double res = 1L<<32;
    _read_calibration_register(pressure_calibration[--i], res, 0x3A);
  }

  {
    constexpr double res = 1<<29;
    constexpr double limiter = 1<<14;
    _read_calibration_register(pressure_calibration[--i], res, 0x38, 0x39, true,
                               limiter);
  }

  {
    constexpr double res = 1<<20;
    constexpr double limiter = 1<<14;
    _read_calibration_register(pressure_calibration[--i], res, 0x36, 0x37, true,
                               limiter);
  }

  if(i != 0) {
    throw std::out_of_range("incorrect calibration size");
  }

  MSG_EXIT();
}

void Sen0251::set_oversampling(Oversampling::oversampling_t pressure, Oversampling::oversampling_t temperature) {
  MSG_ENTER();

  auto sampling = (temperature << 3) | pressure;
  MSG_DEBUG("%d", sampling);
  auto rc = i2c_smbus_write_byte_data(file, Register::oversampling, sampling);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();
}

unsigned char Sen0251::get_power() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_byte_data(file, Register::pwr_ctrl);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();

  return rc;
}

void Sen0251::soft_reset() {
  MSG_ENTER();

  command(Commands::softreset);
  //documentation not less than 10ms
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  if(not event()) {
    THROW(OperationError, "bit has not been set, is device correct?");
  }

  MSG_EXIT();
}

void Sen0251::sea_level_pressure_adjust(float pressure) {
  sea_level_pressure = pressure;
}

void Sen0251::set_iir_filter(IirFilter::iir_filter_t filter) {
  MSG_ENTER();

  Bit::Iir_Filter bits {0};
  bits.bit.filter = filter;

  MSG_DEBUG("iir_filter %d", bits.data);

  auto rc = i2c_smbus_write_byte_data(file, Register::iir_filter, bits.data);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  MSG_EXIT();
}

unsigned char Sen0251::get_iir_filter() const {
  MSG_ENTER();

  auto rc = i2c_smbus_read_byte_data(file, Register::iir_filter);
  if (rc < 0) {
    MSG_WARN("fail to read: %d, %s", rc, get_error());
  }

  Bit::Iir_Filter bits {0};
  bits.bit.filter = rc;

  MSG_EXIT();

  return bits.data;
}

#ifdef WITH_SEN0251_TESTS
Sen0251::Sen0251() : file(0), temperature_calibration(), pressure_calibration(), sea_level_pressure(::sea_level_pressure) {};

void TestSen0251::update_altitude(Sen0251::Readings& readings) {
  obj.set_altitude(readings);
}

void TestSen0251::update_temperature(Sen0251::Readings& readings, float temperature, float *calibration) {
  std::memcpy(obj.temperature_calibration, calibration, sizeof(obj.temperature_calibration));
  obj.set_temperature(readings, temperature);
}

#endif