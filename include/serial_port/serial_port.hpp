#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/crc.hpp>
#include <cstdint>
#include <functional>

namespace imu_receiver {

using boost::crc_16_type;
using boost::asio::io_context;
using boost::asio::serial_port;
using ring_buffer = boost::circular_buffer<uint8_t>;
using crc_16_arc = boost::crc_optimal<16, 0x1021, 0x0000, 0x0000, false, false>;

struct ImuData {
  uint8_t label; /* 0x91 */
  uint8_t id;
  uint8_t reserve;
  int8_t temperature;                      /* celsius */
  float air_pressure;                      /* Pa */
  uint32_t timestamp;                      /* ms */
  std::array<float, 3> acceleration;       /* g XYZ */
  std::array<float, 3> angular;            /* deg/s XYZ */
  std::array<float, 3> magnetic_intensity; /* uT XYZ */
  std::array<float, 3> euler_angles;       /* deg, RPY */
  std::array<float, 4> quaternion;         /* WXYZ */
};

struct ImuFrame {
  uint8_t header;  /* 0x5A */
  uint8_t type;    /* 0xA5 */
  uint16_t length; /* 76 */
  uint16_t crc16;
  ImuData data;
};

struct ImuFrameNoPadding {
  uint8_t header;
  uint8_t type;
  uint16_t length;
  uint16_t crc16;
  ImuData data;
} __attribute__((packed));

constexpr size_t IMU_FRAME_SIZE = sizeof(ImuFrameNoPadding);
constexpr size_t rx_buffer_size = 2 * IMU_FRAME_SIZE - 1;
constexpr size_t RX_BUFFER_SIZE = 512;
constexpr size_t RING_BUFFER_SIZE = 1024;
constexpr uint8_t frame_header = 0x5A;
constexpr uint8_t frame_type = 0xA5;
constexpr uint16_t frame_data_length = 76;

using ImuFrameBuffer = std::array<uint8_t, IMU_FRAME_SIZE>;
using RxBuffer = std::array<uint8_t, RX_BUFFER_SIZE>;

template <typename T>
inline T parse_field(const uint8_t *field_ptr, size_t &offset) {
  T value;
  std::memcpy(&value, field_ptr + offset, sizeof(T));
  offset += sizeof(T);
  return value;
}

class SerialPort {
private:
  io_context &ioc_;
  serial_port serial_;
  crc_16_arc crc16_;
  std::function<void(const ImuData &)> serial_rx_cplt_cb_;
  ImuFrame imu_frame_;
  ring_buffer ring_buffer_;
  RxBuffer rx_buffer_;
  ImuFrameBuffer frame_buffer_;
  std::shared_ptr<spdlog::logger> logger_;

  /* profiling */
  boost::asio::steady_timer timer_;
  std::atomic<size_t> frame_count_{0};

  void handle_rx_data(RxBuffer::const_iterator iter, const size_t len);
  void start_profiling();
  void start_read();

public:
  SerialPort(io_context &ioc, const std::string &port, const std::function<void(const ImuData &)> &serial_rx_cplt_cb);
};

}  // namespace imu_receiver

#endif /* SERIAL_PORT_H */