#ifndef SEN0251_H
#define SEN0251_H

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

  void soft_reset();

  Sen0251(const Sen0251 &) = delete;
  Sen0251(Sen0251 &&) = delete;
  Sen0251 &operator=(const Sen0251 &) = delete;
  Sen0251 &operator=(Sen0251 &&) = delete;

private:
  int file;
  unsigned char filter_coefficient;
  float temperature_calibration[3];
  float pressure_calibration[11];

  /**
   * @return device coefficient filter
   */
  void set_filter_coefficient();
  void set_calibration_data();
  void set_temperature_calibration();
  void set_pressure_calibration();
  void _set_data_to_calibration(float&, double, unsigned char, unsigned char =0, bool=true, double =0);
  void get_temperature(Readings&) const;
  void get_pressure(Readings&) const;
  void get_altitude(Readings&) const;

  friend std::ostream& operator<<(std::ostream& os, const Sen0251& obj) {
    os << "chip id: 0x" << std::hex << static_cast<int>(obj.get_chip_id()) << std::dec
       << "\nosr:  " << PowerControl::to_string(obj.get_power())
       << "\nfifo: " << obj.get_fifo_size()
       << "\nevent: " << obj.event()
       << "\nfilter_coefficient: " << static_cast<int>(obj.filter_coefficient);
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
};

#endif
