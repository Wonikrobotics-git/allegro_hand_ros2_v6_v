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
 * @file comm_can.cpp
 * @brief Implements the CAN bus communication interface for the Allegro Hand.
 * @author ('c')void
 */

#include <atomic>
#include <fcntl.h>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <allegro_hand_io/comm.hpp>

#include <spdlog/spdlog.h>

namespace allegro_hand_io {

/**
 * @class CommIoCan
 * @brief A concrete implementation of the CommIo interface for CAN bus communication.
 *
 * This class handles the low-level details of opening a CAN socket, reading
 * incoming frames in a dedicated thread, and sending outgoing frames.
 */
class CommIoCan : public CommIo {
  /// @brief File descriptor for the CAN socket.
  int socket_;
  /// @brief Desired size of the socket's send buffer in bytes.
  int sndbuf_size_;
  /// @brief Desired size of the socket's receive buffer in bytes.
  int rcvbuf_size_;
  /// @brief Thread object for the background read loop.
  std::thread read_thread_;
  /// @brief Atomic flag to control the lifecycle of the read thread.
  std::atomic<bool> running_{false};

  /// @brief Callback function to decode received frames.
  DecodeIoFrameFunc decode_io_frame_;
  /// @brief Callback function to encode headers for outgoing frames.
  EncodeIoFrameHeaderFunc encode_io_frame_header_;

  /**
   * @brief Initializes the CAN socket.
   *
   * This method performs the following steps:
   * 1. Creates a raw CAN socket.
   * 2. Finds the network interface index for the given interface name (e.g., "can0").
   * 3. Binds the socket to the CAN interface.
   * 4. Sets the send and receive buffer sizes if specified.
   * 5. Sets the socket to non-blocking mode.
   * 6. Clears any pending messages from the receive buffer.
   *
   * @param interface_name The name of the CAN network interface.
   * @return True if the socket was opened and configured successfully, false otherwise.
   */
  bool open_socket(const std::string& interface_name) {
    SPDLOG_DEBUG("CAN: Initializing device on interface '{}'", interface_name);
    sockaddr_can addr;
    ifreq ifr;
    if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
      SPDLOG_ERROR("Failed to open CAN socket: {}", strerror(errno));
      return false;
    }
    strcpy(ifr.ifr_name, interface_name.c_str());
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) == -1) {
      SPDLOG_ERROR("Failed to find CAN bus '{}': {}", interface_name, strerror(errno));
      return false;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    SPDLOG_INFO("CAN interface '{}' found at index {}", interface_name, ifr.ifr_ifindex);
    if (bind(socket_, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
      SPDLOG_ERROR("Error binding CAN socket: {}", strerror(errno));
      return false;
    }

    if (rcvbuf_size_ > 0) {
      if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size_, sizeof(rcvbuf_size_)) == -1) {
        SPDLOG_WARN("Failed to set receive buffer size for CAN socket: {}", strerror(errno));
      }
    }

    if (sndbuf_size_ > 0) {
      if (setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, &sndbuf_size_, sizeof(sndbuf_size_)) == -1) {
        SPDLOG_WARN("Failed to set send buffer size for CAN socket: {}", strerror(errno));
      }
    }

    // Set socket to non-blocking mode and clear receive buffer
    int flags = fcntl(socket_, F_GETFL, 0);
    if (flags == -1) {
      SPDLOG_WARN("Could not get socket flags: {}", strerror(errno));
    } else if (fcntl(socket_, F_SETFL, flags | O_NONBLOCK) == -1) {
      SPDLOG_WARN("Could not set non-blocking mode for CAN socket: {}", strerror(errno));
    } else {
      SPDLOG_DEBUG("CAN: Clearing the CAN receive buffer");
      can_frame dummy_frame;
      while (read(socket_, &dummy_frame, sizeof(dummy_frame)) > 0) {
        // Keep reading until the buffer is empty
      }
    }

    return true;
  }

  /**
   * @brief The main loop for the read thread.
   *
   * This function continuously polls the CAN socket for incoming data. When data is
   * available, it reads all queued CAN frames in a non-blocking manner and passes
   * them to the `decode_io_frame_` callback for processing.
   */
  void read_loop() {
    can_frame msg;
    running_ = true;
    SPDLOG_TRACE("CAN read thread started.");
    while (running_) {
      struct pollfd pfd = {socket_, POLLIN, 0};
      int ret = poll(&pfd, 1, 200); // 200ms timeout

      if (ret < 0) {
        if (running_) { // Avoid error message on shutdown
          SPDLOG_ERROR("Error polling CAN socket: {}", strerror(errno));
        }
        break; // Exit loop on error
      } else if (ret == 0) {
        // Timeout, loop again to check running_ flag
        continue;
      }

      if (pfd.revents & POLLIN) {
        int frames_read = 0;
        while (running_) {
          int len = read(socket_, &msg, sizeof(can_frame));
          if (len < 0) {
            // EAGAIN or EWOULDBLOCK means no more messages in the queue
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
              SPDLOG_ERROR("Error reading from CAN socket: {}", strerror(errno));
            }
            break; // Exit the while loop
          }
          if (len == sizeof(can_frame) && !(msg.can_id & CAN_ERR_FLAG)) {
            decode_io_frame_(msg.can_id, msg.data, msg.can_dlc);
            frames_read++;
          }
        }
        // if (frames_read > 1) { SPDLOG_TRACE("Processed {} CAN frames in one go.", frames_read); }
      }
    }
    SPDLOG_TRACE("CAN read thread finished.");
  }

public:
  /**
   * @brief Constructs a CommIoCan object.
   *
   * @param interface_name The name of the CAN interface (e.g., "can0").
   * @param decode_io_frame Callback for processing received frames.
   * @param encode_io_frame_header Callback for creating CAN IDs for sent frames.
   * @param snd_buf_size Desired send buffer size in bytes. -1 for default.
   * @param rcv_buf_size Desired receive buffer size in bytes. -1 for default.
   * @throws std::runtime_error if the CAN socket fails to open.
   */
  explicit CommIoCan(const std::string& interface_name, DecodeIoFrameFunc decode_io_frame, EncodeIoFrameHeaderFunc encode_io_frame_header,
                     int snd_buf_size = -1, int rcv_buf_size = -1)
      : sndbuf_size_(snd_buf_size), rcvbuf_size_(rcv_buf_size), decode_io_frame_(decode_io_frame),
        encode_io_frame_header_(encode_io_frame_header) {
    SPDLOG_TRACE("CommIoCan[{}] entry", interface_name);
    if (!open_socket(interface_name)) {
      throw std::runtime_error("Failed to open CAN socket");
    }
    read_thread_ = std::thread(&CommIoCan::read_loop, this);
    SPDLOG_TRACE("CommIoCan[{}] ok", interface_name);
  }

  /**
   * @brief Destructor for the CommIoCan object.
   *
   * Signals the read thread to stop, waits for it to join, and closes the socket.
   */
  virtual ~CommIoCan() {
    if (running_) {
      running_ = false;
      if (read_thread_.joinable()) {
        read_thread_.join();
      }
      close(socket_);
    }
  }

  /**
   * @brief Sends a CAN message.
   *
   * @param message_id The application-level message ID.
   * @param data A pointer to the data payload.
   * @param data_size The size of the data payload (0-8 bytes).
   * @return The number of bytes written on success, or a negative value on error.
   */
  int send_message(uint32_t message_id, uint8_t* data, int data_size) override {
    can_frame msg;
    msg.can_id = encode_io_frame_header_(message_id);
    msg.can_dlc = data_size;
    memcpy(msg.data, data, data_size);
    return write(socket_, &msg, sizeof(can_frame));
  }
};

/**
 * @brief Factory function to create a CAN communication interface object.
 *
 * This function is called by `CommIo::Create` when the interface descriptor
 * specifies a CAN interface.
 *
 * @param can_interface_name The name of the CAN interface (e.g., "can0").
 * @param decode_io_frame Callback for processing received frames.
 * @param encode_io_frame_header Callback for creating CAN IDs for sent frames.
 * @param snd_buf_size Desired send buffer size in bytes.
 * @param rcv_buf_size Desired receive buffer size in bytes.
 * @return A shared pointer to the newly created CommIoCan instance.
 */
CommIo::Ptr CreateCommIoCan(std::string can_interface_name, CommIo::DecodeIoFrameFunc decode_io_frame,
                            CommIo::EncodeIoFrameHeaderFunc encode_io_frame_header, int snd_buf_size, int rcv_buf_size) {
  return std::make_shared<CommIoCan>(can_interface_name, decode_io_frame, encode_io_frame_header, snd_buf_size, rcv_buf_size);
}

} // namespace allegro_hand_io
