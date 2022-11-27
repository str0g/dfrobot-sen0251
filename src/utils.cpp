//
// Created by lukasz on 25.11.2022.
//
#include "utils.h"
#include <filesystem>
#include <iostream>

std::vector<unsigned> list_i2c_devices() {
  static std::filesystem::path dev("/dev");
  static std::string i2c{"i2c-"};

  std::vector<unsigned> rc;
  for (auto entry : std::filesystem::directory_iterator(dev)) {
    if (entry.is_character_file()) {
      auto tmp = entry.path().filename().string();
      auto pos = tmp.find(i2c);
      if (pos != std::string::npos) {
        try {
          rc.push_back(std::stoi(&tmp[pos + i2c.size()]));
        } catch (const std::invalid_argument &e) {
          std::cout << "failed to get index for: " << tmp << std::endl;
        }
      }
    }
  }

  return rc;
}

int to_24bit_int(uint8_t b2, uint8_t b1, uint8_t b0) {
  return (b2 << 16) + (b1 << 8) + b0;
}