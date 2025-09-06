#include <tf2/LinearMath/Quaternion.h>

#include <boost/asio.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <serial_port/serial_port.hpp>

using Imu = sensor_msgs::msg::Imu;
using namespace std::chrono_literals;
using namespace boost::asio;
using namespace imu_receiver;

class ImuPublisher : public rclcpp::Node {
private:
  io_context ioc_;
  SerialPort serial_port_;
  rclcpp::Publisher<Imu>::SharedPtr imu_publisher_;

  void publish_imu_data(const ImuData &imu_data) {
    auto imu_msg = Imu();

    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.orientation.w = imu_data.quaternion[0];
    imu_msg.orientation.x = imu_data.quaternion[1];
    imu_msg.orientation.y = imu_data.quaternion[2];
    imu_msg.orientation.z = imu_data.quaternion[3];

    imu_msg.angular_velocity.x = imu_data.angular[0];
    imu_msg.angular_velocity.y = imu_data.angular[1];
    imu_msg.angular_velocity.z = imu_data.angular[2];

    imu_msg.linear_acceleration.x = imu_data.acceleration[0];
    imu_msg.linear_acceleration.y = imu_data.acceleration[1];
    imu_msg.linear_acceleration.z = imu_data.acceleration[2];

    imu_publisher_->publish(imu_msg);
  }

public:
  ImuPublisher()
      : rclcpp::Node("imu_publisher"),
        ioc_(),
        serial_port_(ioc_, "/dev/CH104M-USB", std::bind(&ImuPublisher::publish_imu_data, this, std::placeholders::_1)),
        imu_publisher_(this->create_publisher<Imu>("/imu", 10)) {
    RCLCPP_INFO(this->get_logger(), "IMU Publisher start!");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}