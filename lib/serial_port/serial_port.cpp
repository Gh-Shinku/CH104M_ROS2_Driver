#include "serial_port/serial_port.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>

using namespace imu_receiver;

SerialPort::SerialPort(io_context &ioc, const std::string &port, const std::function<void(const ImuData &)> &serial_rx_cplt_cb)
    : ioc_(ioc), serial_(ioc, port), serial_rx_cplt_cb_(serial_rx_cplt_cb), state_(1), timer_(ioc) {
  serial_.set_option(serial_port::baud_rate(115200));
  serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));
  serial_.set_option(serial_port::parity(serial_port::parity::none));
  serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  serial_.set_option(serial_port::character_size(8));
  fill_rx_buffer();
  start_profiling();
}

static void parse_frame_from_buffer(ImuFrame &imu_frame, const std::array<uint8_t, imu_frame_size> &buffer) {
  size_t offset = 0;
  imu_frame.header = parse_field<decltype(imu_frame.header)>(buffer, offset);
  imu_frame.type = parse_field<decltype(imu_frame.type)>(buffer, offset);
  imu_frame.length = parse_field<decltype(imu_frame.length)>(buffer, offset);
  imu_frame.crc16 = parse_field<decltype(imu_frame.crc16)>(buffer, offset);
  imu_frame.data.label = parse_field<decltype(imu_frame.data.label)>(buffer, offset);
  imu_frame.data.id = parse_field<decltype(imu_frame.data.id)>(buffer, offset);
  imu_frame.data.reserve = parse_field<decltype(imu_frame.data.reserve)>(buffer, offset);
  imu_frame.data.temperature = parse_field<decltype(imu_frame.data.temperature)>(buffer, offset);
  imu_frame.data.air_pressure = parse_field<decltype(imu_frame.data.air_pressure)>(buffer, offset);
  imu_frame.data.timestamp = parse_field<decltype(imu_frame.data.timestamp)>(buffer, offset);
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.acceleration[i] = parse_field<decltype(imu_frame.data.acceleration)::value_type>(buffer, offset);
  }
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.angular[i] = parse_field<decltype(imu_frame.data.angular)::value_type>(buffer, offset);
  }
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.magnetic_intensity[i] = parse_field<decltype(imu_frame.data.magnetic_intensity)::value_type>(buffer, offset);
  }
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.euler_angles[i] = parse_field<decltype(imu_frame.data.euler_angles)::value_type>(buffer, offset);
  }
  for (int i = 0; i < 4; ++i) {
    imu_frame.data.quaternion[i] = parse_field<decltype(imu_frame.data.quaternion)::value_type>(buffer, offset);
  }
}

static uint16_t checksum_from_buffer(const std::array<uint8_t, imu_frame_size> &buffer, crc_16_arc &crc16) {
  crc16.process_bytes(buffer.data(), 4);
  crc16.process_bytes(buffer.data() + 6, imu_frame_size - 6);
  // spdlog::info("CRC calculated: {:#X}", crc16.checksum());
  auto checksum = crc16.checksum();
  crc16.reset();
  return checksum;
}

void SerialPort::fill_rx_buffer() {
  boost::asio::async_read(serial_, boost::asio::buffer(rx_buffer_),
                          [this](const boost::system::error_code &ec, std::size_t bytes_transfered) {
                            if (!ec) {
                              cnt_ = 1;
                              while (cnt_ > 0) {
                                // spdlog::info("state: {}", state_);
                                switch (state_) {
                                  case 1: {
                                    have_cached_frame_ = false;
                                    rx_buf_iter_header_ = std::find(rx_buffer_.begin(), rx_buffer_.end(), frame_header);
                                    if (rx_buf_iter_header_ != rx_buffer_.end() && (*(rx_buf_iter_header_ + 1) == frame_type)
                                        && (static_cast<size_t>(std::distance(rx_buf_iter_header_, rx_buffer_.end())) >= imu_frame_size)) {
                                      ++cnt_;
                                      state_ = 2;
                                    }
                                    break;
                                  }
                                  case 2: {
                                    if (have_cached_frame_) {
                                      rx_buf_iter_header_ = rx_buffer_.begin() + (imu_frame_size - cached_frame_size_);
                                      std::copy(rx_buffer_.begin(), rx_buf_iter_header_, frame_buffer_.begin() + cached_frame_size_);
                                      imu_frame_.crc16 = frame_buffer_[4] | (frame_buffer_[5] << 8);
                                      // spdlog::info("CRC carried: {:#X}", imu_frame_.crc16);
                                      if (checksum_from_buffer(frame_buffer_, crc16_) == imu_frame_.crc16) {
                                        ++cnt_;
                                        state_ = 3;
                                      } else {
                                        state_ = 1;
                                        spdlog::warn("CRC check failed");
                                      }
                                    } else {
                                      ++cnt_;
                                      state_ = 4;
                                    }
                                    break;
                                  }
                                  case 3: {
                                    parse_frame_from_buffer(imu_frame_, frame_buffer_);
                                    serial_rx_cplt_cb_(imu_frame_.data);
                                    ++frame_count_;
                                    state_ = 4;
                                    ++cnt_;
                                    break;
                                  }
                                  case 4: {
                                    std::copy(rx_buf_iter_header_, rx_buf_iter_header_ + imu_frame_size, frame_buffer_.begin());
                                    imu_frame_.crc16 = frame_buffer_[4] | (frame_buffer_[5] << 8);
                                    // spdlog::info("CRC carried: {:#X}", imu_frame_.crc16);
                                    if (checksum_from_buffer(frame_buffer_, crc16_) == imu_frame_.crc16) {
                                      parse_frame_from_buffer(imu_frame_, frame_buffer_);  // op5
                                      serial_rx_cplt_cb_(imu_frame_.data);                 // op6
                                      ++frame_count_;
                                      std::copy(rx_buf_iter_header_ + imu_frame_size, rx_buffer_.end(), frame_buffer_.begin());  // op7
                                      cached_frame_size_ = std::distance(rx_buf_iter_header_ + imu_frame_size, rx_buffer_.end());
                                      state_ = 2;
                                      have_cached_frame_ = true;
                                    } else {
                                      state_ = 1;
                                      spdlog::warn("CRC check failed");
                                    }
                                    break;
                                  }
                                }
                                --cnt_;
                              }
                              fill_rx_buffer();
                            } else {
                              spdlog::error("async_read failed: {}", ec.message());
                              fill_rx_buffer();
                            }
                          });
}

void SerialPort::start_profiling() {
  timer_.expires_after(std::chrono::seconds(1));
  timer_.async_wait([this](const boost::system::error_code &ec) {
    if (!ec) {
      spdlog::info("Frames per second: {}", frame_count_.load());
      frame_count_ = 0;
      start_profiling();
    }
  });
}