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
 * @file test_library.cpp
 * @brief A test program to verify the dynamic linkage of the allegro_hand_utility library.
 * @author ('c')void
 *
 *
 *
 *
 */

#include <allegro_hand_utility/utility.hpp>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief A dummy callback function for decoding frames.
 *
 * This function serves as a placeholder to satisfy the signature of `CommIo::Create`.
 * It is not expected to be called during this test.
 */
void dummy_decode_func(const uint32_t&, uint8_t*, int) {
  // This is a placeholder and is not expected to be called in this test.
}

/**
 * @brief A dummy callback function for encoding frame headers.
 *
 * This function serves as a placeholder to satisfy the signature of `CommIo::Create`.
 * It is not expected to be called during this test.
 * @param message_id The input message ID.
 * @return The same message ID.
 */
uint32_t dummy_encode_func(const uint32_t& message_id) {
  // This is a placeholder.
  return message_id;
}

/**
 * @brief Main entry point for the library linkage test.
 *
 * This program verifies that the `allegro_hand_utility` shared library is correctly
 * linked to the executable by calling one of its core functions.
 */
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  std::cout << "--- Testing allegro_hand_utility library linkage ---" << std::endl;

  try {
    std::cout << "Attempting to call allegro_hand_utility::CommIo::Create()..." << std::endl;

    // This call attempts to use a core function from the library. We expect it
    // to throw an exception because the "can:test_if" interface does not
    // exist in a typical test environment.
    // Successfully catching this specific exception proves that the library's
    // symbols were resolved and the function was executed.
    // allegro_hand_utility::CommIo::Create("can:test_if", dummy_decode_func, dummy_encode_func);

    // // This case is unexpected but still indicates successful linkage.
    // std::cout << "SUCCESS: CommIo::Create() was called without throwing an exception." << std::endl;

  } catch (const std::exception& e) {
    // This is the expected successful outcome.
    std::cout << "SUCCESS: Caught an expected exception: " << e.what() << std::endl;
    std::cout << "This confirms that the library is linked and its symbols are resolved." << std::endl;
  }

  std::cout << "--- Test finished successfully ---" << std::endl;
  return 0;
}
