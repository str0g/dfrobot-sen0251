# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/

cmake_minimum_required(VERSION 3.5)
project(dfrobot-sen0251 VERSION 0.1 LANGUAGES CXX)

option(WITH_SEN0251_TESTS "Enable unit tests" OFF)
option(USE_SUBMODULE "Use dependencies from submodule" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

if (${CMAKE_BUILD_TYPE} EQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
else ()
    add_definitions("-DNODEBUG")
endif()

if(WITH_SEN0251_TESTS)
    enable_testing()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_SEN0251_TESTS")
endif()

if(USE_SUBMODULE)
    set(WITH_MICRO_LOGGER_TESTS OFF)
    add_subdirectory(dependencies/micro-logger)
    include_directories(dependencies/micro-logger/includes)
else()
    include(FindPkgConfig)
    pkg_check_modules(MICRO_LOGGER REQUIRED micro_logger)
    include_directories(${MICRO_LOGGER_INCLUDE_DIRS})
endif()

include_directories(include)
aux_source_directory(src SRC)

add_library(${PROJECT_NAME} SHARED ${SRC})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC micro_logger++)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-li2c")

if(WITH_SEN0251_TESTS)
        add_executable(main tests/main.cpp)
        target_link_libraries(main LINK_PUBLIC ${PROJECT_NAME})
        include(cmake/custom_tests)
        #
        custom_test(test_altitude)
        custom_test(test_temperature)
endif()
