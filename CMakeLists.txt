# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/

cmake_minimum_required(VERSION 3.5)
project(dfrobot-sen0251 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_SEN0251_TESTS "Enable unit tests" ON)
if(WITH_SEN0251_TESTS)
    enable_testing()
endif()

set(WITH_TESTS OFF)
add_subdirectory(dependecies/micro-logger)

include_directories(include)
aux_source_directory(src SRC)

add_library(${PROJECT_NAME} SHARED ${SRC})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC micro_logger++)

if(WITH_SEN0251_TESTS)
        add_executable(main tests/main.cpp)
        set_target_properties(main PROPERTIES LINK_FLAGS "-li2c" LINK_PUBLIC ${PROJECT_NAME})
        target_link_libraries(main LINK_PUBLIC ${PROJECT_NAME})
        include(cmake/custom_tests)
        #        custom_tests(todo)
endif()
