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
 * @file comm.cpp
 * @brief Implements the communication interface classes and the factory method for creating them.
 * @author ('c')void
 */

#include <stdexcept>
#include <string>

#include <allegro_hand_io/comm.hpp>

namespace allegro_hand_io {

/**
 * @brief Forward declaration for the function that creates a CAN communication object.
 *
 * This function is defined in `comm_can.cpp` and is responsible for instantiating
 * a `CommIoCan` object.
 */
extern CommIo::Ptr CreateCommIoCan(std::string interface, CommIo::DecodeIoFrameFunc decode_io_frame,
                                   CommIo::EncodeIoFrameHeaderFunc encode_io_frame_header, int snd_buf_size, int rcv_buf_size);

extern CommIo::Ptr CreateCommIoSerial(std::string interface, CommIo::DecodeIoFrameFunc decode_io_frame,
                                      CommIo::EncodeIoFrameHeaderFunc encode_io_frame_header, int snd_buf_size, int rcv_buf_size);

/**
 * @brief Factory method to create a communication interface instance.
 * @see CommIo::Create in comm.hpp for parameter details.
 */
CommIo::Ptr CommIo::Create(std::string interface_descriptor, DecodeIoFrameFunc decode_io_frame,
                           EncodeIoFrameHeaderFunc encode_io_frame_header, int snd_buf_size, int rcv_buf_size) {

  /*
   * interface_descriptor :
   *    - CAN :=  "can:interface_name"
   *    - SERIAL := "serial:/dev/ttyUSB0,2000000"
   *    - UDP :=  "udp:ip,port"
   *    - MODBUS := "modbus:xxx"
   */

  // Check for "can:" prefix to create a CAN interface.
  if (interface_descriptor.rfind("can:", 0) == 0) {
    std::string can_interface_name = interface_descriptor.substr(4);
    return CreateCommIoCan(can_interface_name, decode_io_frame, encode_io_frame_header, snd_buf_size, rcv_buf_size);

  } else if (interface_descriptor.rfind("serial:", 0) == 0) {
    std::string serial_port_spec = interface_descriptor.substr(7);
    return CreateCommIoSerial(serial_port_spec, decode_io_frame, encode_io_frame_header, snd_buf_size, rcv_buf_size);

    // Check for "udp:" prefix. This is a placeholder for future implementation.
  } else if (interface_descriptor.rfind("udp:", 0) == 0) {
    // Parse UDP details (IP, port) from interface_descriptor
    // For now, just throw as it's not implemented
    throw std::runtime_error("UDP communication not yet implemented");

    // Check for "modbus:" prefix. This is a placeholder for future implementation.
  } else if (interface_descriptor.rfind("modbus:", 0) == 0) {
    // Parse Modbus details from interface_descriptor
    // For now, just throw as it's not implemented
    throw std::runtime_error("Modbus communication for V6 not yet defined");
  } else {
    // If no known prefix is found, throw an error.
    throw std::runtime_error("Unsupported communication interface: " + interface_descriptor);
  }
  return nullptr;
}

} //  namespace allegro_hand_io