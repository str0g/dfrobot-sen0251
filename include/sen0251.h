#ifndef SEN0251_H
#define SEN0251_H

/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <string>
#include <ostream>

namespace Commands {
enum cmd_t {
  nop = 0x00,
  extmode_en_middle = 0x34,
  fifo_flush = 0xb0,
  softreset = 0xb6
};
}

namespace PowerControl {
enum {
  off = 0x00, /// @off can be mixed with @pressure_on and @temperature_on
  pressure_on = 0x01,
  temperature_on = 0x02,
  force_on_a = 0x10,
  force_on_b = 0x20,
  normal = force_on_a | force_on_b
};
std::string to_string(int flag);
}

namespace Oversampling {
enum oversampling_t {
  no = 0x00,
  x2 = 0x01,
  x4 = 0x02,
  x8 = 0x03,
  x16 = 0x4,
  x32 = 0x5
};
}

namespace IirFilter {
enum iir_filter_t {
  coef_0 = 0x0,
  coef_1 = 0x1,
  coef_3 = 0x2,
  coef_7 = 0x3,
  coef_15 = 0x4,
  coef_31 = 0x5,
  coef_63 = 0x6,
  coef_127 = 0x7,
};
}

constexpr float sea_level_pressure = 1013.25f; // millibars

class Sen0251 {
public:
  /**
   * closing device may restart device settings!
   * @param dev
   * @param address
   */
  explicit Sen0251(unsigned dev, unsigned address=0x77);
  ~Sen0251();

  //read only
  /**
   * @return chip id
   */
  unsigned char get_chip_id() const;
  /**
   * @return error or nullptr if there was no error
   */
  const char* get_error() const;
  /**
   *
   * @return status on success or nullptr on error
   */
  const char* get_status() const;
  /**
   *
   * @param temperature get flag @oversampling_t
   * @param pressure get flag @oversampling_t
   */
  void get_oversampling(unsigned char& temperature, unsigned char& pressure) const;
  /**
   * @return fifo size
   */
  int get_fifo_size() const;
  unsigned char get_power() const;
  /**
   * Event like power on, soft reset etc, sets it to 1 as confirmation.
   * Reading sets it to 0
   * @return
   */
  bool event() const;
  //write
  struct Readings {
    float temperature;
    float pressure;
    float altitude;

    friend std::ostream& operator<<(std::ostream& os, const Readings& obj) {
      os <<"temperature: " << obj.temperature <<"C\n"
         <<"pressure: " << obj.pressure <<"hpa\n"
         <<"altitude: " << obj.altitude <<"m";
      return os;
    }
  };
  /**
   * Read register from devices and feeds set functions for Reading structure
   * @return @Readings
   */
  Readings get_readings() const;

  /**
   * Device needs to be activated and have sensor activated
   * low power mode should work in @force_on_a or @force_on_b mod or switch it self
   * off after reading session in @normal mode.
   * @param flag
   */
  void power_control(unsigned char flag=(PowerControl::pressure_on | PowerControl::temperature_on | PowerControl::normal));
  /**
   * Device flushing options @cmd_t
   */
  void command(Commands::cmd_t);
  /**
   * Set sampling resolution
   * @param pressure
   * @param temperature
   */
  void set_oversampling(Oversampling::oversampling_t pressure, Oversampling::oversampling_t temperature);

  void sea_level_pressure_adjust(float pressure);

  void soft_reset();

  void set_iir_filter(IirFilter::iir_filter_t);
  unsigned char get_iir_filter() const;

  friend std::ostream& operator<<(std::ostream& os, const Sen0251& obj) {
    os << "chip id: 0x" << std::hex << static_cast<int>(obj.get_chip_id()) << std::dec
       << "\nosr:  " << PowerControl::to_string(obj.get_power())
       << "\nfifo: " << obj.get_fifo_size()
       << "\nevent: " << obj.event()
       << "\niir_filter: 0x" << std::hex << static_cast<int>(obj.get_iir_filter()) << std::dec;
    auto status = obj.get_status();
    os << "\nstatus: [" << (status ? status : obj.get_error()) << "]";
    unsigned char temperature_flag,pressure_flag;
    obj.get_oversampling(temperature_flag, pressure_flag);
    os << "\nsampling:\n"
       << "\ttemperature: 0x" << std::hex << static_cast<int>(temperature_flag) << std::dec
       << "\tpressure: 0x" << std::hex << static_cast<int>(pressure_flag) << std::dec
       << "\ncalibration:\n"
       << "\ttemperature: [ ";
    for(int i=0; i<sizeof(temperature_calibration)/sizeof(temperature_calibration[0]); ++i) {
      os << obj.temperature_calibration[i] << " ";
    }
    os << "]\n\tpressure: [ ";
    for(int i=0; i<sizeof(pressure_calibration)/sizeof(pressure_calibration[0]); ++i) {
      os << obj.pressure_calibration[i] << " ";
    }
    os << "]\n"
       << obj.get_readings();

    return os;
  }

  Sen0251(const Sen0251 &) = delete;
  Sen0251(Sen0251 &&) = delete;
  Sen0251 &operator=(const Sen0251 &) = delete;
  Sen0251 &operator=(Sen0251 &&) = delete;

private:
  int file;
  float temperature_calibration[3];
  float pressure_calibration[11];
  float sea_level_pressure;

  void set_temperature(Readings&, float temperature) const;
  /**
   * requires @set_temperature to be called first
   * @param pressure
   */
  void set_pressure(Readings&, float pressure) const;
  /**
   * requires @set_pressure to be called first
   * @param pressure
   */
  void set_altitude(Readings&) const;
  /**
   * @return device coefficient filter
   */
  void set_calibration_data();
  void set_temperature_calibration();
  void set_pressure_calibration();
  void _read_calibration_register(float& , double, unsigned char, unsigned char =0, bool =false, double =0);
  float _read_temperature_register() const;
  float _read_pressure_register() const;

#ifdef WITH_SEN0251_TESTS
  Sen0251();

  friend class TestSen0251;
#endif
};
#ifdef WITH_SEN0251_TESTS
class TestSen0251 {
  public:
    void update_altitude(Sen0251::Readings&);
    void update_temperature(Sen0251::Readings&, float temperature, float *calibration);

    Sen0251 obj;
};
#endif

#endif
