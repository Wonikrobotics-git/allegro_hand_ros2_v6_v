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
 * @file comm_udp.cpp
 * @brief Placeholder implementation for the UDP communication interface.
 * @author ('c')void
 */

#include <stdexcept>
#include <string>

#include <allegro_hand_io/comm.hpp>

namespace allegro_hand_io {

/**
 * @class CommIoUDP
 * @brief A placeholder class for UDP communication.
 *
 * This class is intended to provide a UDP communication backend for the
 * Allegro Hand, but it is not yet implemented. Attempting to create an
 * instance of this class will result in a `std::runtime_error`.
 */
class CommIoUDP : public CommIo {
public:
  /**
   * @brief Constructs a CommIoUDP object.
   * @throws std::runtime_error as this feature is not yet implemented.
   */
  explicit CommIoUDP(std::string ip, int port, DecodeIoFrameFunc decode_io_frame, EncodeIoFrameHeaderFunc encode_io_frame_header,
                     int snd_buf_size = -1, int rcv_buf_size = -1) {
    (void)ip;
    (void)port;
    (void)decode_io_frame;
    (void)encode_io_frame_header;
    (void)snd_buf_size;
    (void)rcv_buf_size;

    throw std::runtime_error("UDP communication not yet implemented");
  }

  /**
   * @brief Sends a message via UDP. Not implemented.
   * @return Always returns 0.
   */
  int send_message(uint32_t message_id, uint8_t* data, int data_size) override {
    (void)message_id;
    (void)data;
    (void)data_size;
    return 0;
  }
};

} // namespace allegro_hand_io
