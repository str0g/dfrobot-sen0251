#ifndef SEN0251_H
#define SEN0251_H

#include <string>

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
  no = 0x000,
  x2 = 0x001,
  x4 = 0x010,
  x8 = 0x011,
  x16 = 0x100,
  x32 = 0x101
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
  int get_osr() const;
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
  /**
   * @return device coefficient filter
   */
  unsigned char get_filter_coefficient() const;
  //write
  int get_temperature();
  int get_pressure();
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

  Sen0251(const Sen0251 &) = delete;
  Sen0251(Sen0251 &&) = delete;
  Sen0251 &operator=(const Sen0251 &) = delete;
  Sen0251 &operator=(Sen0251 &&) = delete;

private:
  int file;
  unsigned char filter_coefficient;

  friend std::ostream& operator<<(std::ostream& os, const Sen0251& obj) {
    os << "chip id: 0x" << std::hex << static_cast<int>(obj.get_chip_id()) << std::dec
       << "\nosr:  " << PowerControl::to_string(obj.get_power())
       << "\nfifo: " << obj.get_fifo_size()
       <<"\nevent: " << obj.event()
       <<"\nfilter_coefficient: " << static_cast<int>(obj.get_filter_coefficient());
    auto status = obj.get_status();
    os <<"\nstatus: [" << (status ? status : obj.get_error()) <<"]";

    return os;
  }
};

#endif
