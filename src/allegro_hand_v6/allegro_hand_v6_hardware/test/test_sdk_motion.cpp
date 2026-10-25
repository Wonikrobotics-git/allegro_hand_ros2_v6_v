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
 * @file test_sdk_motion.cpp
 * @brief Standalone motion test node executing pre-defined demo poses for Allegro Hand V6.
 * @author ('c')void
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "v6_sdk.hpp"
#include "v6_spec.hpp"

static std::atomic<bool> g_running{true};

void signal_handler(int sig) {
  (void)sig;
  std::cout << "\n[INFO] Interrupt signal received. Stopping motion safely..." << std::endl;
  g_running = false;
}

struct DemoPose {
  std::string name;
  std::vector<int16_t> pulses; // 20 joints in encoder pulses
};

// Pre-defined poses defined in test_comm/gui.py
const std::vector<DemoPose> DEMO_POSES = {
  {"Zero (Default)",
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},
  {"Demo 1 (Grasp)",
   {700, 512, 512, 512, 180, 300, 512, 512, 0, 300, 512, 512, -90, 300, 512, 512, -120, 300, 650, 550}},
  {"Demo 2 (Pinch)",
   {170, 0, 0, 0, 170, 0, 0, 0, 170, 0, 0, 0, 170, 0, 0, 0, 170, 0, 0, 0}},
  {"Demo 3 (Flex)",
   {0, 0, 0, 0, -170, 0, 0, 0, -170, 0, 0, 0, -170, 0, 0, 0, -170, 0, 0, 0}},
  {"Zero (Default)",
   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
};

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::string ip = "192.168.1.100";
  int port = 502;
  double hold_sec = 1.0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--ip" && i + 1 < argc) {
      ip = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else if (arg == "--hold" && i + 1 < argc) {
      hold_sec = std::stod(argv[++i]);
    }
  }

  std::cout << "=================================================" << std::endl;
  std::cout << "   Allegro Hand V6 Standalone Motion Test Node   " << std::endl;
  std::cout << "   Target: " << ip << ":" << port << " | Hold time: " << hold_sec << "s" << std::endl;
  std::cout << "=================================================" << std::endl;

  v6_sdk::HandV6ModBus::Options options;
  options.comm_type = "tcp";
  options.ip_address = ip;
  options.tcp_port = port;
  options.slave_id = 1;

  options.k_p_table = std::vector<double>(v6_sdk::JOINT_COUNT, 1.0);
  options.k_d_table = std::vector<double>(v6_sdk::JOINT_COUNT, 0.05);
  options.torque_limit_table = std::vector<double>(v6_sdk::JOINT_COUNT, 0.2);

  std::unique_ptr<v6_sdk::HandV6ModBus> hand;
  try {
    hand = std::make_unique<v6_sdk::HandV6ModBus>(options);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] Failed to connect to Allegro Hand V6: " << e.what() << std::endl;
    return 1;
  }

  if (!hand->is_ready()) {
    std::cerr << "[ERROR] HandV6 is not ready." << std::endl;
    return 1;
  }

  std::cout << "[INFO] Enabling Servo Torque and Position Control Mode..." << std::endl;
  hand->set_control_mode(true); // Position control mode & Torque ON

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  int cycle = 1;
  while (g_running) {
    std::cout << "\n=================================================" << std::endl;
    std::cout << ">>> Starting Motion Cycle #" << cycle++ << std::endl;
    std::cout << "=================================================" << std::endl;

    for (size_t p_idx = 0; p_idx < DEMO_POSES.size() && g_running; ++p_idx) {
      const auto& pose = DEMO_POSES[p_idx];
      std::cout << "\n-------------------------------------------------" << std::endl;
      std::cout << ">> Executing Pose [" << (p_idx + 1) << "/" << DEMO_POSES.size() << "]: "
                << pose.name << std::endl;
      std::cout << "-------------------------------------------------" << std::endl;

      double target_rad[v6_sdk::JOINT_COUNT] = {0};
      for (int i = 0; i < v6_sdk::JOINT_COUNT; ++i) {
        target_rad[i] = static_cast<double>(pose.pulses[i]) * v6_sdk::PUL2RAD;
      }

      // Send target position command
      hand->write_commands(target_rad, nullptr, nullptr);

      // Hold and monitor for hold_sec
      auto start_time = std::chrono::steady_clock::now();
      while (g_running) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= static_cast<int64_t>(hold_sec * 1000.0)) {
          break;
        }

        double cur_pos[v6_sdk::JOINT_COUNT] = {0};
        double vel[v6_sdk::JOINT_COUNT] = {0};
        double tau[v6_sdk::JOINT_COUNT] = {0};

        hand->read_states(cur_pos, vel, tau);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      // Print final feedback state for this pose
      double cur_pos[v6_sdk::JOINT_COUNT] = {0};
      double vel[v6_sdk::JOINT_COUNT] = {0};
      double tau[v6_sdk::JOINT_COUNT] = {0};
      hand->read_states(cur_pos, vel, tau);

      std::cout << "Pose Feedback:" << std::endl;
      for (int f = 0; f < 5; ++f) {
        std::string finger_names[] = {"Thumb ", "Index ", "Middle", "Ring  ", "Little"};
        std::cout << "  " << finger_names[f] << ": ";
        for (int j = 0; j < 4; ++j) {
          int j_idx = f * 4 + j;
          std::cout << "J" << j << "=" << std::fixed << std::setprecision(1)
                    << (cur_pos[j_idx] * 180.0 / M_PI) << "° ";
        }
        std::cout << std::endl;
      }
    }
  }

  std::cout << "\n[INFO] Motion sequence completed. Returning to home & disabling torque..." << std::endl;
  double zero_pos[v6_sdk::JOINT_COUNT] = {0};
  hand->write_commands(zero_pos, nullptr, nullptr);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  hand->set_control_mode(false); // Torque OFF
  hand.reset();

  std::cout << "[INFO] Safely finished." << std::endl;
  return 0;
}
