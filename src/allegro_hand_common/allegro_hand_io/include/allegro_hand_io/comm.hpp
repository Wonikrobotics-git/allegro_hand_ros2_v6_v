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
 * @file comm.hpp
 * @brief Defines the abstract communication interface for Allegro Hand.
 * @author ('c')void
 */

#pragma once

#include <atomic>
#include <cassert>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace allegro_hand_io {

/**
 * @class CommIo
 * @brief An abstract base class for communication interfaces (e.g., CAN, UDP).
 *
 * This class defines a generic interface for sending and receiving data frames,
 * allowing different communication protocols to be implemented and used interchangeably.
 */
class CommIo {
public:
  /** @brief A shared pointer to a CommIo instance. */
  using Ptr = std::shared_ptr<CommIo>;

  /**
   * @brief A function type for decoding an incoming data frame.
   * @param message_id The identifier of the received message.
   * @param data A pointer to the received data payload.
   * @param data_size The size of the data payload in bytes.
   */
  using DecodeIoFrameFunc = std::function<void(const uint32_t& message_id, uint8_t* data, int data_size)>;

  /**
   * @brief A function type for encoding the header of an outgoing data frame.
   * @param message_id The application-level message ID to be encoded.
   * @return The protocol-specific frame header (e.g., a CAN ID).
   */
  using EncodeIoFrameHeaderFunc = std::function<uint32_t(const uint32_t& message_id)>;

  CommIo() = default;
  virtual ~CommIo() = default;

  /**
   * @brief Sends a message over the communication channel.
   *
   * This is a pure virtual function that must be implemented by concrete derived classes.
   *
   * @param message_id The identifier for the message to be sent.
   * @param data A pointer to the data payload. Can be nullptr if there is no payload.
   * @param data_size The size of the data payload in bytes.
   * @return The number of bytes written on success, or a negative value on error.
   */
  virtual int send_message(uint32_t message_id, uint8_t* data = nullptr, int data_size = 0) = 0;

  /**
   * @brief A factory method to create a concrete communication interface instance.
   *
   * This method parses the `interface_descriptor` to determine which type of
   * communication object to create (e.g., CAN, UDP).
   *
   * @param interface_descriptor A string describing the interface (e.g., "can:can0", "udp:192.168.1.10:8080").
   * @param decode_io_frame A callback function for decoding received frames.
   * @param encode_io_frame_header A callback function for encoding outgoing frame headers.
   * @param snd_buf_size The desired size of the socket's send buffer in bytes. -1 for default.
   * @param rcv_buf_size The desired size of the socket's receive buffer in bytes. -1 for default.
   * @return A shared pointer to the created CommIo instance.
   */
  static Ptr Create(std::string interface_descriptor, DecodeIoFrameFunc decode_io_frame, EncodeIoFrameHeaderFunc encode_io_frame_header,
                    int snd_buf_size = -1, int rcv_buf_size = -1);
};

} // namespace allegro_hand_io