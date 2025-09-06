#include <spdlog/spdlog.h>

#include <boost/asio.hpp>
#include <cmath>
#include <cstdint>
#include <memory>

#include "serial_port/serial_port.hpp"

using namespace boost::asio;
using namespace imu_receiver;

struct Pose2D {
  double x;
  double y;
  double theta;
};

class Odometry {
  constexpr static double ACC_G = 9.80665;

private:
  Pose2D pose_;
  io_context ioc;
  SerialPort serial_port_;
  uint32_t last_timestamp_;
  double acc_x_offset_, acc_y_offset_;
  double vx_odom_, vy_odom_;

  void ImuCallback(const ImuData &imu_data) {
    if (last_timestamp_ > 0) {
      double dt = (imu_data.timestamp - last_timestamp_) * 1e-3;
      double yaw_rad = pose_.theta * M_PI / 180.0;
      double sin_yaw = std::sin(yaw_rad);
      double cos_yaw = std::cos(yaw_rad);
      double acc_x = (imu_data.acceleration[0] - acc_x_offset_) * ACC_G;
      double acc_y = (imu_data.acceleration[1] - acc_y_offset_) * ACC_G;
      double acc_x_odom = acc_x * cos_yaw - acc_y * sin_yaw;
      double acc_y_odom = acc_x * sin_yaw + acc_y * cos_yaw;

      vx_odom_ += acc_x_odom * dt;
      vy_odom_ += acc_y_odom * dt;

      pose_.x += vx_odom_ * dt;
      pose_.y += vy_odom_ * dt;
      pose_.theta = imu_data.euler_angles[2];
      spdlog::info("x: {:.5f}(m), y: {:.5f}(m), yaw: {:.3f}(deg), vx: {:.3f}(m/s), vy: {:.3f}(m/s)", pose_.x, pose_.y, pose_.theta,
                   vx_odom_, vy_odom_);
    } else {
      acc_x_offset_ = imu_data.acceleration[0];
      acc_y_offset_ = imu_data.acceleration[1];
    }
    last_timestamp_ = imu_data.timestamp;
  }

public:
  Odometry()
      : pose_({.0, .0, .0}),
        ioc(),
        serial_port_(ioc, "/dev/CH104M-USB", std::bind(&Odometry::ImuCallback, this, std::placeholders::_1)),
        last_timestamp_(0),
        vx_odom_(.0),
        vy_odom_(.0) {}
  void start() {
    serial_port_.start();
    ioc.run();
  }
};

int main() {
  auto odometry = std::make_unique<Odometry>();
  odometry->start();
  return 0;
}