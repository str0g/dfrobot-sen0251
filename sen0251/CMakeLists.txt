project(
  sen0251
  VERSION 0.20
  LANGUAGES CXX)

aux_source_directory(src SRC)

add_library(${PROJECT_NAME} SHARED ${SRC})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/sen0251>)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC micro_logger++)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-li2c")

if(DFROBOT_SEN0251_TESTS)
  target_compile_definitions(${PROJECT_NAME} PUBLIC DFROBOT_SEN0251_TESTS)
  #
  add_executable(main tests/main.cpp)
  target_link_libraries(main LINK_PUBLIC ${PROJECT_NAME})
  #
  custom_gtest(test_altitude)
  custom_gtest(test_temperature)
endif()
