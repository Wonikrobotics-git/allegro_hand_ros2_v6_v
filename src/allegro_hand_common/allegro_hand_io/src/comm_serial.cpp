/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file comm_serial.cpp
 * @brief Implements the Serial (2Mbps / Protobuf framing) communication interface for Allegro Hand V6.
 * @author ('c')void
 */

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <allegro_hand_io/comm.hpp>

namespace allegro_hand_io {

static uint16_t calculate_crc16_ccitt_false(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= (static_cast<uint16_t>(data[i]) << 8);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
      } else {
        crc = (crc << 1) & 0xFFFF;
      }
    }
  }
  return crc;
}

class CommIoSerial : public CommIo {
private:
  int serial_fd_{-1};
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  DecodeIoFrameFunc decode_io_frame_;
  EncodeIoFrameHeaderFunc encode_io_frame_header_;

  bool open_serial(const std::string& port_spec) {
    std::string device_path = "/dev/ttyUSB0";
    speed_t baud_rate = B2000000;

    size_t comma_pos = port_spec.find(',');
    if (comma_pos != std::string::npos) {
      device_path = port_spec.substr(0, comma_pos);
      std::string baud_str = port_spec.substr(comma_pos + 1);
      int baud_val = std::stoi(baud_str);
      if (baud_val == 2000000) {
        baud_rate = B2000000;
      } else if (baud_val == 115200) {
        baud_rate = B115200;
      } else if (baud_val == 921600) {
        baud_rate = B921600;
      }
    } else if (!port_spec.empty()) {
      device_path = port_spec;
    }

    SPDLOG_INFO("Serial: Opening port '{}' at baud rate {}", device_path, baud_rate);

    serial_fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
      SPDLOG_ERROR("Serial: Failed to open device '{}'", device_path);
      return false;
    }

    fcntl(serial_fd_, F_SETFL, 0);

    struct termios options;
    memset(&options, 0, sizeof(options));

    if (tcgetattr(serial_fd_, &options) != 0) {
      SPDLOG_ERROR("Serial: tcgetattr failed for '{}'", device_path);
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcflush(serial_fd_, TCIOFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      SPDLOG_ERROR("Serial: tcsetattr failed for '{}'", device_path);
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    running_ = true;
    return true;
  }

  void read_loop() {
    SPDLOG_INFO("Serial: Starting background read thread...");
    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(4096);

    uint8_t temp_buf[512];
    struct pollfd pfd;
    pfd.fd = serial_fd_;
    pfd.events = POLLIN;

    while (running_) {
      int poll_res = poll(&pfd, 1, 10); // 10ms timeout
      if (poll_res > 0 && (pfd.revents & POLLIN)) {
        ssize_t bytes_read = read(serial_fd_, temp_buf, sizeof(temp_buf));
        if (bytes_read > 0) {
          rx_buffer.insert(rx_buffer.end(), temp_buf, temp_buf + bytes_read);
        }
      }

      // Process framing: Header(2B: 'W','R') + Len(2B) + Payload(Len B) + CRC(2B)
      while (rx_buffer.size() >= 6) {
        // Find header 'W' 'R' (0x57, 0x52)
        if (rx_buffer[0] != 0x57 || rx_buffer[1] != 0x52) {
          rx_buffer.erase(rx_buffer.begin());
          continue;
        }

        uint16_t payload_len = (static_cast<uint16_t>(rx_buffer[2]) << 8) | static_cast<uint16_t>(rx_buffer[3]);
        size_t total_packet_size = 2 + 2 + payload_len + 2;

        if (rx_buffer.size() < total_packet_size) {
          // Incomplete packet, wait for more data
          break;
        }

        uint16_t packet_crc = (static_cast<uint16_t>(rx_buffer[total_packet_size - 2]) << 8) |
                              static_cast<uint16_t>(rx_buffer[total_packet_size - 1]);

        uint16_t calc_crc = calculate_crc16_ccitt_false(rx_buffer.data(), total_packet_size - 2);

        if (packet_crc == calc_crc) {
          if (decode_io_frame_) {
            decode_io_frame_(0, rx_buffer.data() + 4, payload_len);
          }
        } else {
          SPDLOG_WARN("Serial: CRC mismatch (recv: 0x{:04X}, calc: 0x{:04X})", packet_crc, calc_crc);
        }

        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + total_packet_size);
      }
    }
  }

public:
  CommIoSerial(const std::string& port_spec, DecodeIoFrameFunc decode_io_frame,
               EncodeIoFrameHeaderFunc encode_io_frame_header)
      : decode_io_frame_(decode_io_frame), encode_io_frame_header_(encode_io_frame_header) {
    if (!open_serial(port_spec)) {
      throw std::runtime_error("Failed to open serial port: " + port_spec);
    }
    read_thread_ = std::thread(&CommIoSerial::read_loop, this);
  }

  virtual ~CommIoSerial() {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  int send_message(uint32_t message_id, uint8_t* data, int data_size) override {
    if (serial_fd_ < 0) return -1;
    (void)message_id;

    std::vector<uint8_t> packet;
    packet.reserve(6 + data_size);

    // Header: 'W', 'R'
    packet.push_back(0x57);
    packet.push_back(0x52);

    // Length (Big-Endian)
    packet.push_back(static_cast<uint8_t>((data_size >> 8) & 0xFF));
    packet.push_back(static_cast<uint8_t>(data_size & 0xFF));

    // Data payload
    if (data != nullptr && data_size > 0) {
      packet.insert(packet.end(), data, data + data_size);
    }

    // Calculate CRC16 over Header + Length + Payload
    uint16_t crc = calculate_crc16_ccitt_false(packet.data(), packet.size());

    // Append CRC (Big-Endian)
    packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    packet.push_back(static_cast<uint8_t>(crc & 0xFF));

    ssize_t bytes_written = write(serial_fd_, packet.data(), packet.size());
    return static_cast<int>(bytes_written);
  }
};

CommIo::Ptr CreateCommIoSerial(std::string port_spec, CommIo::DecodeIoFrameFunc decode_io_frame,
                             CommIo::EncodeIoFrameHeaderFunc encode_io_frame_header, int snd_buf_size, int rcv_buf_size) {
  (void)snd_buf_size;
  (void)rcv_buf_size;
  return std::make_shared<CommIoSerial>(port_spec, decode_io_frame, encode_io_frame_header);
}

} // namespace allegro_hand_io
