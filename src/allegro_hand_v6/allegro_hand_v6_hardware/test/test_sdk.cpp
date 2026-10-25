/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Wonik Robotics.
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
 * @file test_sdk.cpp
 * @brief GTest unit test for Allegro Hand V6 Modbus SDK (target device 192.168.1.100).
 * @author ('c')void
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>

#include "v6_sdk.hpp"
#include "v6_spec.hpp"

class HandV6SdkTest : public ::testing::Test {
protected:
  static std::unique_ptr<v6_sdk::HandV6ModBus> hand_;

  static void SetUpTestSuite() {
    v6_sdk::HandV6ModBus::Options options;
    options.comm_type = "tcp";
    options.ip_address = "192.168.1.100";
    options.tcp_port = 502;
    options.slave_id = 1;

    options.k_p_table = std::vector<double>(v6_sdk::JOINT_COUNT, 1.0);
    options.k_d_table = std::vector<double>(v6_sdk::JOINT_COUNT, 0.05);
    options.torque_limit_table = std::vector<double>(v6_sdk::JOINT_COUNT, 0.2);

    try {
      hand_ = std::make_unique<v6_sdk::HandV6ModBus>(options);
    } catch (const std::exception& e) {
      std::cerr << "Failed to connect to 192.168.1.100: " << e.what() << std::endl;
      hand_ = nullptr;
    }
  }

  static void TearDownTestSuite() {
    hand_.reset();
  }
};

std::unique_ptr<v6_sdk::HandV6ModBus> HandV6SdkTest::hand_ = nullptr;

/* =========================================================================
 * TEST 1: Connection & Device Information Query (device_info_)
 * ========================================================================= */
TEST_F(HandV6SdkTest, ConnectionAndDeviceInfo) {
  ASSERT_NE(hand_, nullptr) << "HandV6ModBus instance could not be created";
  EXPECT_TRUE(hand_->is_ready()) << "Modbus TCP connection failed to 192.168.1.100:502";

  const auto& dev_info = hand_->get_device_info();
  std::cout << "\n[Device Info]: FW=0x" << std::hex << dev_info.firmware_version
            << ", Type=" << (dev_info.hand_type == v6_sdk::HandType::RIGHT_HAND ? "RIGHT" : "LEFT")
            << ", SN=" << dev_info.serial_number << std::dec << std::endl;
  EXPECT_GT(dev_info.firmware_version, 0);
  EXPECT_FALSE(dev_info.serial_number.empty());
  EXPECT_EQ(dev_info.serial_number, hand_->device_info_.serial_number);
}

/* =========================================================================
 * TEST 2: 20-Joint State Feedback Reading (read_states)
 * ========================================================================= */
TEST_F(HandV6SdkTest, ReadJointStates) {
  ASSERT_NE(hand_, nullptr);
  ASSERT_TRUE(hand_->is_ready());

  double pos[v6_sdk::JOINT_COUNT] = {0};
  double vel[v6_sdk::JOINT_COUNT] = {0};
  double tau[v6_sdk::JOINT_COUNT] = {0};

  ASSERT_NO_THROW(hand_->read_states(pos, vel, tau));

  std::cout << "\n[Feedback State Table - 20 Joints]" << std::endl;
  for (int i = 0; i < v6_sdk::JOINT_COUNT; ++i) {
    std::cout << "  Joint " << i << ": Pos=" << pos[i] * 180.0 / M_PI << " deg ("
              << pos[i] << " rad), Vel=" << vel[i] << " rad/s, Torque=" << tau[i] << " Nm" << std::endl;

    // Verify positions are within reasonable physical limits [-2pi, 2pi]
    EXPECT_GE(pos[i], -2.0 * M_PI);
    EXPECT_LE(pos[i], 2.0 * M_PI);

    // Verify velocity is physically bounded
    EXPECT_GE(vel[i], -50.0);
    EXPECT_LE(vel[i], 50.0);
  }
}

/* =========================================================================
 * TEST 3: Sensor Telemetry Reading (unified read_states: IMU + 18 Pressures)
 * ========================================================================= */
TEST_F(HandV6SdkTest, ReadSensorStates) {
  ASSERT_NE(hand_, nullptr);
  ASSERT_TRUE(hand_->is_ready());

  // Unified telemetry read: joint positions + IMU quaternion + 18 tactile pressures
  double pos[v6_sdk::JOINT_COUNT] = {0};
  double quat[4] = {0};
  double pressures[v6_sdk::PRESSURE_COUNT] = {0};

  ASSERT_NO_THROW(hand_->read_states(pos, nullptr, nullptr, quat, pressures));

  // Verify quaternion normalization
  double norm_sq = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
  std::cout << "\n[IMU Quaternion]: (" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3]
            << ") -> Norm Sq: " << norm_sq << std::endl;
  EXPECT_NEAR(norm_sq, 1.0, 0.1);

  // Verify tactile pressure values
  std::cout << "[Tactile Pressures (18 Sensors - hPa)]:" << std::endl;
  for (int i = 0; i < v6_sdk::PRESSURE_COUNT; ++i) {
    std::cout << "  P[" << i << "]=" << pressures[i] << " hPa";
    if ((i + 1) % 6 == 0) std::cout << std::endl;
    EXPECT_GE(pressures[i], 0.0);
    EXPECT_LE(pressures[i], 50000.0);
  }
  std::cout << std::endl;
}

/* =========================================================================
 * TEST 4: System Diagnostics Reading (read_diagnostics)
 * ========================================================================= */
TEST_F(HandV6SdkTest, ReadDiagnostics) {
  ASSERT_NE(hand_, nullptr);
  ASSERT_TRUE(hand_->is_ready());

  v6_sdk::HandDiagnostics diag;
  EXPECT_TRUE(hand_->read_diagnostics(diag));

  std::cout << "\n[System Diagnostics]:" << std::endl;
  std::cout << "  Torque: " << (diag.torque_enabled ? "ON" : "OFF")
            << ", Calib Running: " << (diag.calibration_running ? "YES" : "NO")
            << ", Joint Status Raw: 0x" << std::hex << diag.joint_status.raw_value
            << ", IMU Calib Raw: 0x" << (int)diag.imu_calib_status.raw_value << std::dec << std::endl;

  std::cout << "  Motor Temperatures (degC): ";
  for (int i = 0; i < v6_sdk::JOINT_COUNT; ++i) {
    std::cout << diag.motor_temperatures[i] << " ";
    EXPECT_GE(diag.motor_temperatures[i], 5);
    EXPECT_LE(diag.motor_temperatures[i], 80);
  }
  std::cout << "\n  Sensor Temperatures (100 LSB/degC): ";
  for (int i = 0; i < v6_sdk::PRESSURE_COUNT; ++i) {
    std::cout << diag.sensor_temperatures[i] << " ";
  }
  std::cout << std::endl;
}

/* =========================================================================
 * TEST 4: PID Gain Read and Write
 * ========================================================================= */
TEST_F(HandV6SdkTest, ReadWriteGains) {
  ASSERT_NE(hand_, nullptr);
  ASSERT_TRUE(hand_->is_ready());

  // 1. Read existing gains
  auto original_p_gains = hand_->read_gains("p_gain");
  ASSERT_EQ(original_p_gains.size(), static_cast<size_t>(v6_sdk::JOINT_COUNT));

  std::cout << "\n[Original P Gains]: ";
  for (double g : original_p_gains) std::cout << g << " ";
  std::cout << std::endl;

  // 2. Write integer test gain values (uint16_t register)
  std::vector<double> test_gains(v6_sdk::JOINT_COUNT, 150.0);
  ASSERT_NO_THROW(hand_->write_gains("p_gain", test_gains));

  // 3. Read back and verify
  auto read_gains = hand_->read_gains("p_gain");
  ASSERT_EQ(read_gains.size(), static_cast<size_t>(v6_sdk::JOINT_COUNT));
  for (int i = 0; i < v6_sdk::JOINT_COUNT; ++i) {
    EXPECT_NEAR(read_gains[i], 150.0, 0.01);
  }

  // 4. Restore original gains
  hand_->write_gains("p_gain", original_p_gains);
}

/* =========================================================================
 * TEST 5: Safe Write Commands (Feed current position back to prevent motion)
 * ========================================================================= */
TEST_F(HandV6SdkTest, SafeWriteCommands) {
  ASSERT_NE(hand_, nullptr);
  ASSERT_TRUE(hand_->is_ready());

  double current_pos[v6_sdk::JOINT_COUNT] = {0};
  double vel[v6_sdk::JOINT_COUNT] = {0};
  double tau[v6_sdk::JOINT_COUNT] = {0};

  // Read current position
  hand_->read_states(current_pos, vel, tau);

  // Safely write current position back to target position
  ASSERT_NO_THROW(hand_->write_commands(current_pos, nullptr, nullptr));
  std::cout << "[Safe Write Command]: Current positions safely written back to 0x0100~0x0113" << std::endl;
}

/* =========================================================================
 * TEST 6: Continuous Communication Rate & Loop Stability (100 Iterations)
 * ========================================================================= */
TEST_F(HandV6SdkTest, LoopTimingAndRateMonitor) {
  ASSERT_NE(hand_, nullptr);
  ASSERT_TRUE(hand_->is_ready());

  double pos[v6_sdk::JOINT_COUNT] = {0};
  double vel[v6_sdk::JOINT_COUNT] = {0};
  double tau[v6_sdk::JOINT_COUNT] = {0};

  auto start_time = std::chrono::steady_clock::now();
  constexpr int ITERATIONS = 100;

  for (int i = 0; i < ITERATIONS; ++i) {
    hand_->read_states(pos, vel, tau);
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); // ~500 Hz target cycle
  }

  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();

  std::cout << "\n[Loop Performance]: " << ITERATIONS << " cycles completed in "
            << elapsed_ms << " ms (~" << (1000.0 * ITERATIONS / elapsed_ms) << " Hz)" << std::endl;
  EXPECT_GT(ITERATIONS, 0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
