#include "serial_port/serial_port.hpp"

#include <algorithm>

using namespace imu_receiver;

SerialPort::SerialPort(io_context &ioc, const std::string &port, const std::function<void(const ImuData &)> &serial_rx_cplt_cb)
    : ioc_(ioc),
      serial_(ioc, port),
      serial_rx_cplt_cb_(serial_rx_cplt_cb),
      ring_buffer_(RING_BUFFER_SIZE),
      logger_(spdlog::basic_logger_mt("imu_logger", "imu.log", true)),
      timer_(ioc) {
  serial_.set_option(serial_port::baud_rate(921600));
  serial_.set_option(serial_port::flow_control(serial_port::flow_control::none));
  serial_.set_option(serial_port::parity(serial_port::parity::none));
  serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  serial_.set_option(serial_port::character_size(8));
}

void SerialPort::start(bool do_profiling) {
  start_read();
  if (do_profiling) start_profiling();
}

static void parse_frame_from_buffer(ImuFrameBuffer::const_iterator frame_header_iter, ImuFrame &imu_frame) {
  size_t offset = 0;
  imu_frame.header = parse_field<decltype(imu_frame.header)>(frame_header_iter, offset);
  imu_frame.type = parse_field<decltype(imu_frame.type)>(frame_header_iter, offset);
  imu_frame.length = parse_field<decltype(imu_frame.length)>(frame_header_iter, offset);
  imu_frame.crc16 = parse_field<decltype(imu_frame.crc16)>(frame_header_iter, offset);
  imu_frame.data.label = parse_field<decltype(imu_frame.data.label)>(frame_header_iter, offset);
  imu_frame.data.id = parse_field<decltype(imu_frame.data.id)>(frame_header_iter, offset);
  imu_frame.data.reserve = parse_field<decltype(imu_frame.data.reserve)>(frame_header_iter, offset);
  imu_frame.data.temperature = parse_field<decltype(imu_frame.data.temperature)>(frame_header_iter, offset);
  imu_frame.data.air_pressure = parse_field<decltype(imu_frame.data.air_pressure)>(frame_header_iter, offset);
  imu_frame.data.timestamp = parse_field<decltype(imu_frame.data.timestamp)>(frame_header_iter, offset);
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.acceleration[i] = parse_field<decltype(imu_frame.data.acceleration)::value_type>(frame_header_iter, offset);
  }
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.angular[i] = parse_field<decltype(imu_frame.data.angular)::value_type>(frame_header_iter, offset);
  }
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.magnetic_intensity[i] = parse_field<decltype(imu_frame.data.magnetic_intensity)::value_type>(frame_header_iter, offset);
  }
  for (int i = 0; i < 3; ++i) {
    imu_frame.data.euler_angles[i] = parse_field<decltype(imu_frame.data.euler_angles)::value_type>(frame_header_iter, offset);
  }
  for (int i = 0; i < 4; ++i) {
    imu_frame.data.quaternion[i] = parse_field<decltype(imu_frame.data.quaternion)::value_type>(frame_header_iter, offset);
  }
}

static uint16_t check_crc(ImuFrameBuffer::const_iterator frame_header_iter, crc_16_arc &crc16) {
  crc16.process_bytes(frame_header_iter, 4);
  crc16.process_bytes(frame_header_iter + 6, IMU_FRAME_SIZE - 6);
  // spdlog::info("CRC calculated: {:#X}", crc16.checksum());
  auto checksum = crc16.checksum();
  crc16.reset();
  return checksum;
}

void SerialPort::start_read() {
  serial_.async_read_some(boost::asio::buffer(rx_buffer_), [this](const boost::system::error_code &ec, size_t bytes_transferred) {
    if (!ec) {
      handle_rx_data(rx_buffer_.begin(), bytes_transferred);
    } else {
      logger_->error("Serial read error: {}", ec.message());
    }
    start_read();
  });
}

void SerialPort::handle_rx_data(RxBuffer::const_iterator iter, const size_t len) {
  // logger_->info("read_some: {}(Bytes)", len);
  ring_buffer_.insert(ring_buffer_.end(), iter, iter + len);
  auto iter_header = std::find(ring_buffer_.begin(), ring_buffer_.end(), frame_header);
  if (iter_header != ring_buffer_.end() && (*(iter_header + 1)) == frame_type) {
    while (static_cast<size_t>(std::distance(iter_header, ring_buffer_.end())) >= IMU_FRAME_SIZE) {
      std::copy(iter_header, iter_header + IMU_FRAME_SIZE, frame_buffer_.begin());
      iter_header += IMU_FRAME_SIZE;
      imu_frame_.crc16 = (frame_buffer_[5] << 8) | frame_buffer_[4];
      if (check_crc(frame_buffer_.begin(), crc16_) == imu_frame_.crc16) {
        parse_frame_from_buffer(frame_buffer_.begin(), imu_frame_);
        const auto &imu_data = imu_frame_.data;
        serial_rx_cplt_cb_(imu_data);
        logger_->info("id: {}, acc: [{}, {}, {}], euler: [{}, {}, {}]", imu_data.id, imu_data.acceleration[0], imu_data.acceleration[1],
                      imu_data.acceleration[2], imu_data.euler_angles[0], imu_data.euler_angles[1], imu_data.euler_angles[2]);
        ++frame_count_;
      } else {
        logger_->warn("CRC check error");
      }
    }
    ring_buffer_.erase(ring_buffer_.begin(), iter_header);
  } else {
    ring_buffer_.clear();
  }
}

void SerialPort::start_profiling() {
  timer_.expires_after(std::chrono::seconds(1));
  timer_.async_wait([this](const boost::system::error_code &ec) {
    if (!ec) {
      logger_->info("Frames per second: {}", frame_count_.load());
      frame_count_ = 0;
      start_profiling();
    }
  });
}