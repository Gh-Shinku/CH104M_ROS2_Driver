cmake_minimum_required(VERSION 3.8)
project(imu_publisher)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_DIR build)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(
  -Wall
  -Wpedantic
  -Wextra
  "$<$<CONFIG:Debug>:-O0;-g;-ggdb>"
  "$<$<CONFIG:Release>:-O2>"
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

set(ROS_DEPS
  rclcpp
  sensor_msgs
  tf2
)

set(THIRD_DEPS
  Boost::system
  spdlog::spdlog
  fmt::fmt
)

set(LOCAL_DEPS
  lib_serial_port
)

set(ALL_DEPS
  ${ROS_DEPS}
  ${THIRD_DEPS}
  ${LOCAL_DEPS}
)

set(BUILD_TEST ON)

add_library(lib_serial_port lib/serial_port/serial_port.cpp)
target_include_directories(lib_serial_port PUBLIC include)
target_link_libraries(lib_serial_port PUBLIC ${THIRD_DEPS})

add_executable(imu_publisher src/imu_publisher.cpp)
target_include_directories(imu_publisher PRIVATE include)
target_link_libraries(imu_publisher PRIVATE ${LOCAL_DEPS} ${THIRD_DEPS})
ament_target_dependencies(imu_publisher PUBLIC ${ROS_DEPS})

if(BUILD_TEST)
  add_executable(test_serial test/test_serial_port.cpp)
  target_include_directories(test_serial PRIVATE include)
  target_link_libraries(test_serial PRIVATE ${LOCAL_DEPS} ${THIRD_DEPS})

  add_executable(test_integration test/test_integration.cpp)
  target_include_directories(test_integration PRIVATE include)
  target_link_libraries(test_integration PRIVATE ${LOCAL_DEPS} ${THIRD_DEPS})
endif()

install(TARGETS imu_publisher
  DESTINATION lib/imu_publisher
)

ament_package()
