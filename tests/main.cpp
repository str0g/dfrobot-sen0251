/*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <memory>
#include <chrono>
#include <thread>

#include "../dependecies/micro-logger/includes/micro_logger.hpp"

#include "sen0251.h"
#include "utils.h"

const char* ENV_DEVICE_INDEX = "ENV_DEVICE_INDEX";

std::shared_ptr<micro_logger::BaseWriter> writer;

void set_stdo_writer() {
  writer = std::make_shared<micro_logger::StandardOutWriter>();
  micro_logger::set_writer(*writer);
};

int get_device_index() {
  try {
    return std::stoi(std::getenv(ENV_DEVICE_INDEX));
  } catch (const std::logic_error& e) {
    std::cout << "please set ENV_DEVICE_INDEX" << std::endl;
    throw;
  }
}

struct Options {
  const char *description;
  void (*fun)(Sen0251&, char**, int&);
};

std::unordered_map<std::string, Options> actions;

void _help() {
  std::cout << "Options: " << std::endl;
  char buf[25];
  std::memset(buf, ' ', sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';
  for (auto &action : actions) {
    auto &str = action.first;
    std::memcpy(buf, str.c_str(), str.length());
    std::cout << "\t" << buf;
    std::cout << action.second.description << std::endl;
    std::memset(buf, ' ', str.length());
  }
  std::cout << std::endl;
}

void help(Sen0251& not_used, char **argv, int& argc) {
  _help();
}

void list_devices(Sen0251& not_used, char **argv, int& argc) {
  auto rc = list_i2c_devices();
  std::cout << "possible devices: [ ";
  if (rc.empty()) {
    std::cout << "none ";
  }
  for (auto dev : rc) {
    std::cout << dev << " ";
  }
  std::cout << "]" << std::endl;
}

void sensors(Sen0251& dev, char **argv, int& argc) {
  for(int i=3; i; --i) {
    std::cout << dev.get_readings() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void on(Sen0251& dev, char **argv, int& argc) {
  dev.power_control();
}

void off(Sen0251& dev, char **argv, int& argc) {
  dev.power_control(PowerControl::off);
}

void low_power(Sen0251& dev, char **argv, int& argc) {
  dev.power_control(PowerControl::pressure_on | PowerControl::temperature_on | PowerControl::force_on_a);
}

void status(Sen0251& dev, char **argv, int& argc) {
  dev.power_control();
  dev.set_oversampling(Oversampling::x8, Oversampling::x4);
  std::cout << dev << std::endl;
}

void misc(Sen0251& dev, char **argv, int& argc) {
  dev.get_status();
  dev.get_error();
}

void sea_level(Sen0251& dev, char **argv, int& argc) {
  dev.sea_level_pressure_adjust(std::stof(argv[++argc]));
  ++argc;
}

int main(int argc, char **argv) {
  actions["help"] = {"print help", help};
  actions["list"] = {"list device indexes", list_devices};
  actions["sensors"] = {"get readings", sensors};
  actions["on"] = {"get osr", on};
  actions["off"] = {"get osr", off};
  actions["low_power"] = {"low_power", low_power};
  actions["status"] = {"status", status};
  actions["misc"] = {"misc", misc};
  actions["sea"] = {"sea_level", sea_level};

  set_stdo_writer();

  if (argc == 1 or argv[1] == std::string("help")) {
    _help();
    return 0;
  }

  if (argv[1] == std::string("list")) {
    Sen0251 *empty = nullptr;
    list_devices(*empty, nullptr, argc);
    return 0;
  }

  Sen0251 dev(get_device_index());
  int i = 1;
  while (i < argc) {
    try {
      auto &option = actions.at(argv[i]);
      option.fun(dev, argv, i);
    } catch (const std::out_of_range &e) {
      std::cout << "unknown " << argv[i] << std::endl;
      _help();
      return 0;
    }
    ++i;
  }

  return 0;
}
