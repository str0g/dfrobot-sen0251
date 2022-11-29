#include <cstring>
#include <cerrno>
#include <limits>
#include <cmath>
#include <vector>
#include <thread>

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

void Sen0251::get_temperature(Readings& obj) const {
  MSG_ENTER();

  auto p1 = i2c_smbus_read_byte_data(file, Register::temperature);//7-0
  auto p2 = i2c_smbus_read_byte_data(file, Register::temperature+1);//15-8
  auto p3 = i2c_smbus_read_byte_data(file, Register::temperature+2);//24-15
  auto reading = to_24bit(p3, p2, p1);
  MSG_DEBUG("t1: %d t2: %d t3: %d, reading: %f", p1, p2, p3, reading);

  auto temp_part1 = reading - temperature_calibration[0];
  auto temp_part2 = temp_part1 * temperature_calibration[1];

  obj.temperature = temp_part2 + (temp_part1 * temp_part1) * temperature_calibration[2];
  MSG_DEBUG("tp1: %f tp2: %f temp: %f", temp_part1, temp_part2, obj.temperature);

  MSG_EXIT();
}

void Sen0251::get_pressure(Readings& obj) const {
  MSG_ENTER();

  auto p1 = i2c_smbus_read_byte_data(file, Register::pressure);//7-0
  auto p2 = i2c_smbus_read_byte_data(file, Register::pressure+1);//15-8
  auto p3 = i2c_smbus_read_byte_data(file, Register::pressure+2);//24-15

  auto reading = to_24bit(p1, p2, p3);
  MSG_DEBUG("p1: %d p2: %d p3: %d, reading: %f", p1, p2, p3, reading);

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
    partial_out_2 = (pressure_calibration[0] + part1 + part2 + part3) * reading;
  }

  float partial_out_3;
  {
    auto part1 = reading * reading;
    auto part2 = pressure_calibration[8] + pressure_calibration[9] * temperature_compensation;
    auto part3 = part1 * part2;
    partial_out_3 = part3 + (part1 * reading) * pressure_calibration[10];
  }

  obj.pressure = partial_out_1 + partial_out_2 + partial_out_3;

  MSG_DEBUG("pp1: %d pp2: %d pp3: %d, press: %f", p1, p2, p3, obj.pressure);

  MSG_EXIT();
}

void Sen0251::get_altitude(Readings& obj) const {
  constexpr double sea_level_pressure = 1013.23f;
  obj.altitude = ((float)powf(sea_level_pressure / obj.pressure, 0.190223f) - 1.0f) * (obj.temperature + 273.15f) / 0.0065f;
}

Sen0251::Readings Sen0251::get_readings() const {
  MSG_ENTER();

  Readings obj;

  get_temperature(obj);
  get_pressure(obj);
  get_altitude(obj);

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

void Sen0251::_set_data_to_calibration(float& out, double coefficient, unsigned char address1, unsigned char address2, bool sign, double limiter) {
  auto nvm = i2c_smbus_read_byte_data(file, address1);
  if (nvm < 0) {
    MSG_WARN("fail to read: %d, %s 0x%x", nvm, get_error(), address1);
  }
  if (address2) {
    auto nvm2 = i2c_smbus_read_byte_data(file, address2);
    if (nvm2 < 0) {
      MSG_WARN("fail to read: %d, %s 0x%x", nvm2, get_error(), address2);
    }
    nvm = (sign ? static_cast<int16_t>((nvm << 8) | nvm2) : static_cast<uint16_t>((nvm << 8) | nvm2));
  }
  out = (static_cast<double >(nvm) -limiter) / coefficient;
}

void Sen0251::set_temperature_calibration() {
  MSG_ENTER();
  {
    constexpr double res = 1L<<48;
    _set_data_to_calibration(temperature_calibration[2], res, 0x35);
  }
  {
    constexpr double res = 1<<30;
    _set_data_to_calibration(temperature_calibration[1], res, 0x33, 0x34, false);
  }
  {
    constexpr auto res{0.00390625}; // 2^-8
    _set_data_to_calibration(temperature_calibration[0], res, 0x31, 0x32, false);
  }

  MSG_EXIT();
}

void Sen0251::set_pressure_calibration() {
  MSG_ENTER();
  {
    constexpr double res = std::numeric_limits<double>::max();//1L<<65;//@TODO fix me
    _set_data_to_calibration(pressure_calibration[10], res, 0x45);
  }

  {
    constexpr double res = 1L<<48;
    _set_data_to_calibration(pressure_calibration[9], res, 0x44);
  }

  {
    constexpr double res = 1L<<48;
    _set_data_to_calibration(pressure_calibration[8], res, 0x42, 0x43);
  }

  {
    constexpr double res = 1L<<15;
    _set_data_to_calibration(pressure_calibration[7], res, 0x41);
  }

  {
    constexpr double res = 1L<<8;
    _set_data_to_calibration(pressure_calibration[6], res, 0x40);
  }

  {
    constexpr double res = 1<<6;
    _set_data_to_calibration(pressure_calibration[5], res, 0x3E, 0x3F, false);
  }

  {
    constexpr double res = 0.125; //2^-3
    _set_data_to_calibration(pressure_calibration[4], res, 0x3C, 0x3D, false);
  }

  {
    constexpr double res = 1L<<37;
    _set_data_to_calibration(pressure_calibration[3], res, 0x3B);
  }

  {
    constexpr double res = 1L<<32;
    _set_data_to_calibration(pressure_calibration[2], res, 0x3A);
  }

  {
    constexpr double res = 1<<29;
    constexpr double limiter = 1<<14;
    _set_data_to_calibration(pressure_calibration[1], res, 0x38, 0x39, true, limiter);
  }

  {
    constexpr double res = 1<<20;
    constexpr double limiter = 1<<14;
    _set_data_to_calibration(pressure_calibration[0], res, 0x36, 0x37, true, limiter);
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