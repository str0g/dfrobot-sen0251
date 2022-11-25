cmake_minimum_required(VERSION 3.5)
project(dfrobot-sen0251 VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_TESTS "Enable unit tests" ON)
if(WITH_TESTS)
    enable_testing()
endif()

include_directories(include)
aux_source_directory(src SRC)

add_library(${PROJECT_NAME} SHARED ${SRC})

if(WITH_TESTS)
        add_executable(main tests/main.cpp)
        set_target_properties(main PROPERTIES LINK_FLAGS "" LINK_PUBLIC ${PROJECT_NAME})
        target_link_libraries(main LINK_PUBLIC ${PROJECT_NAME})
        include(cmake/custom_tests)
        #        custom_tests(todo)
endif()
