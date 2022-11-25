#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>

#include "sen0251.h"

struct Options {
  const char *description;
  void (*fun)();
};

std::unordered_map<std::string, Options> actions;

void help() {
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

int main(int argc, char **argv) {
  actions["help"] = {"print help", help};

  for (int i=1; i < argc; ++i) {
    try {
      auto &option = actions.at(argv[i]);
      option.fun();
    } catch (const std::out_of_range &e) {
      std::cout << "unknown " << argv[i] << std::endl;
      help();
      return 0;
    }
  }

  return 0;
}
