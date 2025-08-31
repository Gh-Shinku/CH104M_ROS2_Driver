#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <cstdint>
#include <functional>

namespace imu_receiver {

using boost::crc_16_type;
using boost::asio::io_context;
using boost::asio::serial_port;
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

constexpr size_t imu_frame_size = sizeof(ImuFrameNoPadding);
constexpr size_t rx_buffer_size = 2 * imu_frame_size - 1;
constexpr uint8_t frame_header = 0x5A;
constexpr uint8_t frame_type = 0xA5;
constexpr uint16_t frame_data_length = 76;

template <typename T, size_t N>
inline T parse_field(const std::array<uint8_t, N> &buffer, size_t &offset) {
  T value;
  std::memcpy(&value, buffer.data() + offset, sizeof(T));
  offset += sizeof(T);
  return value;
}

class SerialPort {
  using RxBuffer = std::array<uint8_t, rx_buffer_size>;
  using FrameBuffer = std::array<uint8_t, imu_frame_size>;

private:
  io_context &ioc_;
  serial_port serial_;
  crc_16_arc crc16_;
  std::function<void(const ImuData &)> serial_rx_cplt_cb_;
  ImuFrame imu_frame_;
  RxBuffer rx_buffer_;
  FrameBuffer frame_buffer_;
  size_t cached_frame_size_;
  RxBuffer::iterator rx_buf_iter_header_; /* 指向 rx_buffer_ 中完整一帧的起始位置 */
  int state_, cnt_;
  bool have_cached_frame_;

  /* profiling */
  boost::asio::steady_timer timer_;
  std::atomic<size_t> frame_count_{0};

  void fill_rx_buffer();
  void start_profiling();

public:
  SerialPort(io_context &ioc, const std::string &port, const std::function<void(const ImuData &)> &serial_rx_cplt_cb);
};

}  // namespace imu_receiver

#endif /* SERIAL_PORT_H */