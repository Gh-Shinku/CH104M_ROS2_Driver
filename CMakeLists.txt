cmake_minimum_required(VERSION 3.8)
project(imu_alone)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_compile_options(
  -Wall
  -Wpedantic
  -Wextra
  "$<$<CONFIG:Debug>:-O0;-g;-ggdb>"
  "$<$<CONFIG:Release>:-O2>"
)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

set(THIRD_DEPS
  Boost::system
  spdlog::spdlog
  fmt::fmt
)

set(BUILD_TEST ON)

add_library(lib_serial_port lib/serial_port/serial_port.cpp)
target_include_directories(lib_serial_port PUBLIC include)
target_link_libraries(lib_serial_port PUBLIC ${THIRD_DEPS})

if(BUILD_TEST)
  add_executable(test_serial test/test_serial_port.cpp)
  target_include_directories(test_serial PRIVATE include)
  target_link_libraries(test_serial PRIVATE lib_serial_port ${THIRD_DEPS})

  add_executable(test_integration test/test_integration.cpp)
  target_include_directories(test_integration PRIVATE include)
  target_link_libraries(test_integration PRIVATE lib_serial_port ${THIRD_DEPS})
endif()

ament_package()
