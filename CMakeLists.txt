# This Source Code Form is subject to the terms of the Mozilla Public License,
# v. 2.0. If a copy of the MPL was not distributed with this file, You can
# obtain one at https://mozilla.org/MPL/2.0/

cmake_minimum_required(VERSION 3.25)
project(
  dfrobot-sen0251
  VERSION 0.2
  LANGUAGES CXX)

# ------------------------------------------------------------
# Detect top-level
# ------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(MICRO_LOGGER_TOP_LEVEL ON)
else()
  set(MICRO_LOGGER_TOP_LEVEL OFF)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if(MICRO_LOGGER_TOP_LEVEL)
  option(DFROBOT_SEN0251_TESTS "Enable unit tests" OFF)
  option(DFROBOT_USE_SUBMODULES "Use dependencies from submodule" OFF)
  option(DFROBOT_SEN0251_SANITIZER "Enable sanitizer" OFF)
else()
  set(DFROBOT_SEN0251_TESTS OFF)
  set(DFROBOT_USE_SUBMODULES OFF)
  set(DFROBOT_SEN0251_SANITIZER OFF)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DFROBOT_SEN0251_SANITIZER)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
endif()

if(DFROBOT_USE_SUBMODULES)
  add_subdirectory(dependencies/micro-logger)
else()  
  find_package(micro_logger REQUIRED)
  include_directories(${MICRO_LOGGER_INCLUDE_DIR})
endif()

if(DFROBOT_SEN0251_TESTS)
  enable_testing()
  include(cmake/custom_tests.cmake)
endif()

add_subdirectory(sen0251)
