#include <spdlog/spdlog.h>

#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <serial_port/serial_port.hpp>

using namespace imu_receiver;
using namespace std::chrono_literals;

int main() {
  io_context ioc;

  auto serial_port = std::make_unique<SerialPort>(ioc, "/dev/CH104M-USB", [](const ImuData &imu_data) {
    spdlog::info("id: {}, acc: [{}, {}, {}], euler: [{}, {}, {}]", imu_data.id, imu_data.acceleration[0], imu_data.acceleration[1],
                 imu_data.acceleration[2], imu_data.euler_angles[0], imu_data.euler_angles[1], imu_data.euler_angles[2]);
  });

  ioc.run();
  return 0;
}
