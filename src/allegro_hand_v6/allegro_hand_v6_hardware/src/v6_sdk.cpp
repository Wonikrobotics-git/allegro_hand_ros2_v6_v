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
 * @file v6_sdk.cpp
 * @brief Implements the HandV6ModBus SDK using libmodbus for Allegro Hand V6.
 * @author ('c')void
 */

#include "v6_sdk.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

#include <modbus/modbus.h>
#include <spdlog/spdlog.h>

#include "stopwatch.hpp" 

namespace v6_sdk {

const char* HandV6::TORQUE_LIMIT_GROUP = "torque_limit";
const char* HandV6ModBus::TORQUE_LIMIT_GROUP = "torque_limit";

/* =========================================================================
 * Constructor & Destructor
 * ========================================================================= */

HandV6ModBus::HandV6ModBus(const Options& options) : options_(options) {
  prv_state_.stamp_ns = 0;
  for (int i = 0; i < JOINT_COUNT; ++i) {
    prv_state_.positions[i] = 0.0;
  }
  prv_state_.velocity_filters.reserve(JOINT_COUNT);
  for (int i = 0; i < JOINT_COUNT; ++i) {
    prv_state_.velocity_filters.emplace_back(500.0, 40.0);
  }

  if (!_initialize()) {
    throw std::runtime_error("Failed to initialize Allegro HandV6ModBus");
  }
}

HandV6ModBus::~HandV6ModBus() {
  if (modbus_ctx_) {
    _set_torque_on_off(false);
    modbus_close(modbus_ctx_);
    modbus_free(modbus_ctx_);
    modbus_ctx_ = nullptr;
  }
  is_connected_ = false;
}

/* =========================================================================
 * Initialization & Connection Lifecycle
 * ========================================================================= */

bool HandV6ModBus::_initialize() {
  std::lock_guard<SpinLock> lock(lock_);

  if (options_.comm_type == "tcp") {
    modbus_ctx_ = modbus_new_tcp(options_.ip_address.c_str(), options_.tcp_port);
  } else {
    modbus_ctx_ = modbus_new_rtu(options_.serial_device.c_str(), options_.baudrate, 'N', 8, 1);
  }

  if (!modbus_ctx_) {
    SPDLOG_ERROR("Unable to create libmodbus context for type '{}'", options_.comm_type);
    return false;
  }

  // Set response timeout: 200 ms
  modbus_set_response_timeout(modbus_ctx_, 0, 200000);
  modbus_set_slave(modbus_ctx_, options_.slave_id);

  if (modbus_connect(modbus_ctx_) == -1) {
    SPDLOG_ERROR("Connection failed to Allegro Hand V6: {}", modbus_strerror(errno));
    modbus_free(modbus_ctx_);
    modbus_ctx_ = nullptr;
    is_connected_ = false;
    return false;
  }

  is_connected_ = true;
  SPDLOG_INFO("Successfully connected to Allegro Hand V6 ({}:{})",
              options_.comm_type == "tcp" ? options_.ip_address : options_.serial_device,
              options_.comm_type == "tcp" ? options_.tcp_port : options_.baudrate);

  // Safe startup: ensure torque is OFF initially
  modbus_write_bit(modbus_ctx_, static_cast<uint16_t>(ModbusCoil::TORQUE_ENABLE), 0);

  // Read firmware version & hand type (0x0070 ~ 0x0071)
  uint16_t dev_info[2] = {0};
  if (modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_FW_VER), 2, dev_info) == 2) {
    device_info_.firmware_version = dev_info[0];
    device_info_.hand_type = static_cast<HandType>(dev_info[1]);
    SPDLOG_INFO("Hand V6 FW Version: 0x{:04X}, Hand Type: {}",
                device_info_.firmware_version, device_info_.hand_type == HandType::RIGHT_HAND ? "RIGHT" : "LEFT");
  }

  // Read Serial Number (0x0318, 4 registers / 8 bytes)
  uint16_t sn_regs[4] = {0};
  if (modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CFG_SERIAL_NO), 4, sn_regs) == 4) {
    char sn_buf[9] = {0};
    memcpy(sn_buf, sn_regs, 8);
    device_info_.serial_number = std::string(sn_buf);
    SPDLOG_INFO("Hand V6 Serial Number: {}", device_info_.serial_number);
  }

  // Apply default gains if provided in Options
  if (!options_.k_p_table.empty() && options_.k_p_table.size() >= static_cast<size_t>(JOINT_COUNT)) {
    uint16_t kp_arr[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) kp_arr[i] = static_cast<uint16_t>(options_.k_p_table[i]);
    modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP), JOINT_COUNT, kp_arr);
  }
  if (!options_.k_i_table.empty() && options_.k_i_table.size() >= static_cast<size_t>(JOINT_COUNT)) {
    uint16_t ki_arr[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) ki_arr[i] = static_cast<uint16_t>(options_.k_i_table[i]);
    modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI), JOINT_COUNT, ki_arr);
  }
  if (!options_.k_d_table.empty() && options_.k_d_table.size() >= static_cast<size_t>(JOINT_COUNT)) {
    uint16_t kd_arr[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) kd_arr[i] = static_cast<uint16_t>(options_.k_d_table[i]);
    modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD), JOINT_COUNT, kd_arr);
  }
  if (!options_.torque_limit_table.empty() && options_.torque_limit_table.size() >= static_cast<size_t>(JOINT_COUNT)) {
    int16_t curr_limit[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) {
      curr_limit[i] = static_cast<int16_t>(options_.torque_limit_table[i] / TORQUE_CONSTANT);
    }
    modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_CUR), JOINT_COUNT, reinterpret_cast<const uint16_t*>(curr_limit));
  }

  return true;
}

/* =========================================================================
 * ros2_control Public Interface Overrides
 * ========================================================================= */

void HandV6ModBus::set_control_mode(bool position_control_mode) {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return;

  OpMode mode = position_control_mode ? OpMode::POSITION_CONTROL : OpMode::CURRENT_CONTROL;
  uint16_t mode_val = static_cast<uint16_t>(mode);
  modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_OP_MODE), mode_val);

  // Enable servo torque
  modbus_write_bit(modbus_ctx_, static_cast<uint16_t>(ModbusCoil::TORQUE_ENABLE), 1);
  signal_rate_monitor_.tick("set_control_mode");
}

bool HandV6ModBus::read_states(
    double* position_state,
    double* velocity_state,
    double* torque_state,
    double* imu_quat_state,
    double* pressure_state) {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return false;

  // Read fast contiguous feedback block: 0x0000 ~ 0x003D (62 registers = 124 bytes)
  constexpr int FAST_BLOCK_SIZE = 62;
  uint16_t fast_buf[FAST_BLOCK_SIZE] = {0};

  auto t_start = std::chrono::high_resolution_clock::now();
  StopWatch::instance().append("modbus_read_registers_entry", t_start); 

  int rc = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_POS), FAST_BLOCK_SIZE, fast_buf);

  auto t_end = std::chrono::high_resolution_clock::now();
  StopWatch::instance().append("modbus_read_registers_exit", t_end); 

  if (rc == FAST_BLOCK_SIZE) {
    // 1. Joint Positions (0x0000 ~ 0x0013, 20 regs)
    if (position_state) {
      const int16_t* f_pos = reinterpret_cast<const int16_t*>(&fast_buf[0]);
      for (int i = 0; i < JOINT_COUNT; ++i) {
        position_state[i] = static_cast<double>(f_pos[i]) * PUL2RAD;
      }
    }

    // 2. Joint Currents / Torques (0x0014 ~ 0x0027, 20 regs)
    if (torque_state) {
      const int16_t* f_curr = reinterpret_cast<const int16_t*>(&fast_buf[20]);
      for (int i = 0; i < JOINT_COUNT; ++i) {
        torque_state[i] = static_cast<double>(f_curr[i]) * TORQUE_CONSTANT;
      }
    }

    // 3. IMU Quaternion (0x0028 ~ 0x002B, 4 regs: [w, x, y, z])
    if (imu_quat_state) {
      const int16_t* imu_raw = reinterpret_cast<const int16_t*>(&fast_buf[40]);
      imu_quat_state[0] = static_cast<double>(imu_raw[0]) / 16384.0; // w
      imu_quat_state[1] = static_cast<double>(imu_raw[1]) / 16384.0; // x
      imu_quat_state[2] = static_cast<double>(imu_raw[2]) / 16384.0; // y
      imu_quat_state[3] = static_cast<double>(imu_raw[3]) / 16384.0; // z
    }

    // 4. Tactile Pressures (0x002C ~ 0x003D, 18 regs)
    if (pressure_state) {
      for (int i = 0; i < PRESSURE_COUNT; ++i) {
        pressure_state[i] = static_cast<double>(fast_buf[44 + i]);
      }
    }
  }

  StopWatch::instance().append("parse_fast_block", std::chrono::high_resolution_clock::now());

  // 5. Joint Velocities: estimated via numerical differentiation + LowpassFilter
  int64_t current_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  double dt = (prv_state_.stamp_ns > 0) ? static_cast<double>(current_ns - prv_state_.stamp_ns) * 1e-9 : 0.0;

  if (velocity_state) {
    if (dt > 1e-5 && dt < 0.1) { // valid cycle interval (10us ~ 100ms)
      for (int i = 0; i < JOINT_COUNT; ++i) {
        double current_pos = position_state ? position_state[i] : prv_state_.positions[i];
        double raw_vel = (current_pos - prv_state_.positions[i]) / dt;
        if (i < static_cast<int>(prv_state_.velocity_filters.size())) {
          velocity_state[i] = prv_state_.velocity_filters[i].filtering(raw_vel);
        } else {
          velocity_state[i] = raw_vel;
        }
      }
    } else {
      for (int i = 0; i < JOINT_COUNT; ++i) {
        velocity_state[i] = 0.0;
      }
    }
  }

  // Update previous positions & timestamp
  if (position_state) {
    for (int i = 0; i < JOINT_COUNT; ++i) {
      prv_state_.positions[i] = position_state[i];
    }
  }
  prv_state_.stamp_ns = current_ns;

  StopWatch::instance().append("calculate_velocities", std::chrono::high_resolution_clock::now());

  signal_rate_monitor_.tick("read_states", t_start, t_end - t_start);

  StopWatch::instance().append("update signal_rate_monitor", std::chrono::high_resolution_clock::now());
  return rc == FAST_BLOCK_SIZE;
}

void HandV6ModBus::write_commands(const double* position_command, const double* velocity_command, const double* torque_command) {
  (void)velocity_command;
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return;

  auto t_start = std::chrono::high_resolution_clock::now();
  StopWatch::instance().append("modbus_write_registers_entry", t_start); 

  if (position_command) {
    int16_t pos_pulse[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) {
      pos_pulse[i] = static_cast<int16_t>(std::round(position_command[i] * RAD2PUL));
    }
    modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_POS), JOINT_COUNT,
                           reinterpret_cast<const uint16_t*>(pos_pulse));
  }

  if (torque_command) {
    int16_t curr_ma[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) {
      curr_ma[i] = static_cast<int16_t>(std::round(torque_command[i] / TORQUE_CONSTANT));
    }
    modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_CUR), JOINT_COUNT,
                           reinterpret_cast<const uint16_t*>(curr_ma));
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  StopWatch::instance().append("modbus_write_registers_exit", t_end);  

  signal_rate_monitor_.tick("write_commands", t_start, t_end - t_start);
}

bool HandV6ModBus::read_diagnostics(HandDiagnostics& diag) {
  if (!modbus_ctx_ || !is_connected_) return false;

  // 1. Digital status & Operating mode (split lock scopes)
  {
    std::lock_guard<SpinLock> lock(lock_);
    _get_torque_status(diag.torque_enabled);
  }
  {
    std::lock_guard<SpinLock> lock(lock_);
    _get_calibration_status(diag.calibration_running);
  }
  {
    std::lock_guard<SpinLock> lock(lock_);
    _get_operation_mode(diag.operation_mode);
  }

  // 2. Read contiguous diagnostic registers: 0x0040 ~ 0x0067 (40 registers = 80 bytes)
  constexpr int DIAG_BLOCK_SIZE = 40;
  uint16_t diag_buf[DIAG_BLOCK_SIZE] = {0};
  bool block_ok = false;

  {
    std::lock_guard<SpinLock> lock(lock_);
    block_ok = (modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_FINGER_TEMP),
                                      DIAG_BLOCK_SIZE, diag_buf) == DIAG_BLOCK_SIZE);
  }

  if (block_ok) {
    // Motor temperatures
    for (int i = 0; i < JOINT_COUNT; ++i) {
      diag.motor_temperatures[i] = diag_buf[i];
    }
    // Sensor temperatures
    for (int i = 0; i < PRESSURE_COUNT; ++i) {
      diag.sensor_temperatures[i] = diag_buf[20 + i];
    }
    // Joint status
    diag.joint_status.raw_value = diag_buf[38];
    // IMU calibration
    diag.imu_calib_status.raw_value = static_cast<uint8_t>(diag_buf[39] & 0xFF);
  }

  signal_rate_monitor_.tick("read_diagnostics");
  return true;
}

std::vector<double> HandV6ModBus::read_gains(const char* group_name) {
  std::lock_guard<SpinLock> lock(lock_);
  std::vector<double> gains;
  if (!modbus_ctx_ || !is_connected_) return gains;

  uint16_t addr = 0;
  if (strcmp(group_name, "p_gain") == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP);
  } else if (strcmp(group_name, "i_gain") == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI);
  } else if (strcmp(group_name, "d_gain") == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD);
  } else if (strcmp(group_name, TORQUE_LIMIT_GROUP) == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_CUR);
  } else {
    return gains;
  }

  uint16_t raw_regs[JOINT_COUNT] = {0};
  if (modbus_read_registers(modbus_ctx_, addr, JOINT_COUNT, raw_regs) == JOINT_COUNT) {
    gains.resize(JOINT_COUNT);
    for (int i = 0; i < JOINT_COUNT; ++i) {
      if (strcmp(group_name, TORQUE_LIMIT_GROUP) == 0) {
        gains[i] = static_cast<int16_t>(raw_regs[i]) * TORQUE_CONSTANT;
      } else {
        gains[i] = static_cast<double>(raw_regs[i]);
      }
    }
  }
  return gains;
}

void HandV6ModBus::write_gains(const char* group_name, const std::vector<double>& gains) {
  if (gains.size() < static_cast<size_t>(JOINT_COUNT)) return;
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return;

  uint16_t addr = 0;
  uint16_t raw_regs[JOINT_COUNT] = {0};

  if (strcmp(group_name, "p_gain") == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP);
    for (int i = 0; i < JOINT_COUNT; ++i) raw_regs[i] = static_cast<uint16_t>(gains[i]);
  } else if (strcmp(group_name, "i_gain") == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI);
    for (int i = 0; i < JOINT_COUNT; ++i) raw_regs[i] = static_cast<uint16_t>(gains[i]);
  } else if (strcmp(group_name, "d_gain") == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD);
    for (int i = 0; i < JOINT_COUNT; ++i) raw_regs[i] = static_cast<uint16_t>(gains[i]);
  } else if (strcmp(group_name, TORQUE_LIMIT_GROUP) == 0) {
    addr = static_cast<uint16_t>(ModbusReg::CMD_CUR);
    for (int i = 0; i < JOINT_COUNT; ++i) {
      raw_regs[i] = static_cast<uint16_t>(static_cast<int16_t>(gains[i] / TORQUE_CONSTANT));
    }
  } else {
    return;
  }

  modbus_write_registers(modbus_ctx_, addr, JOINT_COUNT, raw_regs);
}

bool HandV6ModBus::set_torque_enabled(bool enabled) {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return false;
  return _set_torque_on_off(enabled);
}

bool HandV6ModBus::trigger_calibration() {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return false;
  return _trigger_joint_calibration(true);
}

bool HandV6ModBus::set_imu_profile(ImuProfile profile) {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return false;
  return _set_imu_config(profile);
}

bool HandV6ModBus::save_grasp_pose(uint16_t slot_id) {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return false;
  return _save_grasp_slot(slot_id);
}

bool HandV6ModBus::play_grasp_pose(uint16_t slot_id) {
  std::lock_guard<SpinLock> lock(lock_);
  if (!modbus_ctx_ || !is_connected_) return false;
  return _play_grasp_slot(slot_id);
}

/* =========================================================================
 * Private Helper Methods (Modbus Primitives)
 * ========================================================================= */

bool HandV6ModBus::_set_goal_positions(const int16_t* positions_pulse) {
  if (!modbus_ctx_) return false;
  return modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_POS), JOINT_COUNT,
                                reinterpret_cast<const uint16_t*>(positions_pulse)) == JOINT_COUNT;
}

bool HandV6ModBus::_set_goal_position(JointIndex joint, int16_t position_pulse) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_POS) + j_idx,
                               static_cast<uint16_t>(position_pulse)) == 1;
}

bool HandV6ModBus::_set_goal_currents(const int16_t* currents_ma) {
  if (!modbus_ctx_) return false;
  return modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_CUR), JOINT_COUNT,
                                reinterpret_cast<const uint16_t*>(currents_ma)) == JOINT_COUNT;
}

bool HandV6ModBus::_set_goal_current(JointIndex joint, int16_t current_ma) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_CUR) + j_idx,
                               static_cast<uint16_t>(current_ma)) == 1;
}

bool HandV6ModBus::_set_operation_mode(OpMode mode) {
  if (!modbus_ctx_) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_OP_MODE), static_cast<uint16_t>(mode)) == 1;
}

bool HandV6ModBus::_set_gains_kp(const uint16_t* kp) {
  if (!modbus_ctx_) return false;
  return modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP), JOINT_COUNT, kp) == JOINT_COUNT;
}

bool HandV6ModBus::_set_gain_kp(JointIndex joint, uint16_t kp) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP) + j_idx, kp) == 1;
}

bool HandV6ModBus::_set_gains_ki(const uint16_t* ki) {
  if (!modbus_ctx_) return false;
  return modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI), JOINT_COUNT, ki) == JOINT_COUNT;
}

bool HandV6ModBus::_set_gain_ki(JointIndex joint, uint16_t ki) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI) + j_idx, ki) == 1;
}

bool HandV6ModBus::_set_gains_kd(const uint16_t* kd) {
  if (!modbus_ctx_) return false;
  return modbus_write_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD), JOINT_COUNT, kd) == JOINT_COUNT;
}

bool HandV6ModBus::_set_gain_kd(JointIndex joint, uint16_t kd) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD) + j_idx, kd) == 1;
}

bool HandV6ModBus::_set_imu_config(ImuProfile profile) {
  if (!modbus_ctx_) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CFG_IMU_CONFIG), static_cast<uint16_t>(profile)) == 1;
}

bool HandV6ModBus::_set_torque_on_off(bool on_off) {
  if (!modbus_ctx_) return false;
  return modbus_write_bit(modbus_ctx_, static_cast<uint16_t>(ModbusCoil::TORQUE_ENABLE), on_off ? 1 : 0) == 1;
}

bool HandV6ModBus::_trigger_joint_calibration(bool start) {
  if (!modbus_ctx_) return false;
  return modbus_write_bit(modbus_ctx_, static_cast<uint16_t>(ModbusCoil::JOINT_CALIB), start ? 1 : 0) == 1;
}

bool HandV6ModBus::_save_grasp_slot(uint16_t slot_id) {
  if (!modbus_ctx_) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GRASP_SAVE), slot_id) == 1;
}

bool HandV6ModBus::_play_grasp_slot(uint16_t slot_id) {
  if (!modbus_ctx_) return false;
  return modbus_write_register(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GRASP_PLAY), slot_id) == 1;
}

bool HandV6ModBus::_get_feedback_positions(int16_t* positions_pulse) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_POS), JOINT_COUNT,
                               reinterpret_cast<uint16_t*>(positions_pulse)) == JOINT_COUNT;
}

bool HandV6ModBus::_get_feedback_position(JointIndex joint, int16_t& position_pulse) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  uint16_t reg = 0;
  bool ok = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_POS) + j_idx, 1, &reg) == 1;
  if (ok) position_pulse = static_cast<int16_t>(reg);
  return ok;
}

bool HandV6ModBus::_get_feedback_currents(int16_t* currents_ma) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_CUR), JOINT_COUNT,
                               reinterpret_cast<uint16_t*>(currents_ma)) == JOINT_COUNT;
}

bool HandV6ModBus::_get_feedback_current(JointIndex joint, int16_t& current_ma) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  uint16_t reg = 0;
  bool ok = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_CUR) + j_idx, 1, &reg) == 1;
  if (ok) current_ma = static_cast<int16_t>(reg);
  return ok;
}

bool HandV6ModBus::_get_motor_temperatures(uint16_t* temps_c) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_FINGER_TEMP), JOINT_COUNT, temps_c) == JOINT_COUNT;
}

bool HandV6ModBus::_get_motor_temperature(JointIndex joint, uint16_t& temp_c) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_FINGER_TEMP) + j_idx, 1, &temp_c) == 1;
}

bool HandV6ModBus::_get_operation_mode(OpMode& mode) {
  if (!modbus_ctx_) return false;
  uint16_t reg = 0;
  bool ok = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_OP_MODE), 1, &reg) == 1;
  if (ok) mode = static_cast<OpMode>(reg);
  return ok;
}

bool HandV6ModBus::_get_gains_kp(uint16_t* kp) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP), JOINT_COUNT, kp) == JOINT_COUNT;
}

bool HandV6ModBus::_get_gain_kp(JointIndex joint, uint16_t& kp) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KP) + j_idx, 1, &kp) == 1;
}

bool HandV6ModBus::_get_gains_ki(uint16_t* ki) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI), JOINT_COUNT, ki) == JOINT_COUNT;
}

bool HandV6ModBus::_get_gain_ki(JointIndex joint, uint16_t& ki) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KI) + j_idx, 1, &ki) == 1;
}

bool HandV6ModBus::_get_gains_kd(uint16_t* kd) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD), JOINT_COUNT, kd) == JOINT_COUNT;
}

bool HandV6ModBus::_get_gain_kd(JointIndex joint, uint16_t& kd) {
  uint16_t j_idx = static_cast<uint16_t>(joint);
  if (!modbus_ctx_ || j_idx >= JOINT_COUNT) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CMD_GAIN_KD) + j_idx, 1, &kd) == 1;
}

bool HandV6ModBus::_get_imu_data(int16_t* imu_data) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_IMU), 4,
                               reinterpret_cast<uint16_t*>(imu_data)) == 4;
}

bool HandV6ModBus::_get_pressures_hpa(uint16_t* pressures_hpa) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_PRESSURE_HPA), PRESSURE_COUNT,
                               pressures_hpa) == PRESSURE_COUNT;
}

bool HandV6ModBus::_get_pressure_hpa(PressureIndex sensor, uint16_t& pressure_hpa) {
  uint16_t s_idx = static_cast<uint16_t>(sensor);
  if (!modbus_ctx_ || s_idx >= PRESSURE_COUNT) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_PRESSURE_HPA) + s_idx, 1, &pressure_hpa) == 1;
}

bool HandV6ModBus::_get_sensor_temperatures(uint16_t* temps_raw) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_SENSOR_TEMP), PRESSURE_COUNT,
                               temps_raw) == PRESSURE_COUNT;
}

bool HandV6ModBus::_get_sensor_temperature(PressureIndex sensor, uint16_t& temp_raw) {
  uint16_t s_idx = static_cast<uint16_t>(sensor);
  if (!modbus_ctx_ || s_idx >= PRESSURE_COUNT) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_SENSOR_TEMP) + s_idx, 1, &temp_raw) == 1;
}

bool HandV6ModBus::_get_imu_calibration_status(ImuCalibStatus& calib_status) {
  if (!modbus_ctx_) return false;
  uint16_t reg = 0;
  bool ok = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_IMU_CALIB), 1, &reg) == 1;
  if (ok) calib_status.raw_value = static_cast<uint8_t>(reg & 0xFF);
  return ok;
}

bool HandV6ModBus::_get_torque_status(bool& on_off) {
  if (!modbus_ctx_) return false;
  uint8_t bit_val = 0;
  bool ok = modbus_read_bits(modbus_ctx_, static_cast<uint16_t>(ModbusCoil::TORQUE_ENABLE), 1, &bit_val) == 1;
  if (ok) on_off = (bit_val != 0);
  return ok;
}

bool HandV6ModBus::_get_calibration_status(bool& is_running) {
  if (!modbus_ctx_) return false;
  uint8_t bit_val = 0;
  bool ok = modbus_read_bits(modbus_ctx_, static_cast<uint16_t>(ModbusCoil::JOINT_CALIB), 1, &bit_val) == 1;
  if (ok) is_running = (bit_val != 0);
  return ok;
}

bool HandV6ModBus::_get_joint_status(JointStatusBitmask& status_bitmask) {
  if (!modbus_ctx_) return false;
  uint16_t reg = 0;
  bool ok = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_JOINT_STATUS), 1, &reg) == 1;
  if (ok) status_bitmask.raw_value = reg;
  return ok;
}

bool HandV6ModBus::_get_firmware_version(uint16_t& fw_ver) {
  if (!modbus_ctx_) return false;
  return modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_FW_VER), 1, &fw_ver) == 1;
}

bool HandV6ModBus::_get_hand_type(HandType& hand_type) {
  if (!modbus_ctx_) return false;
  uint16_t reg = 0;
  bool ok = modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::GET_HAND_TYPE), 1, &reg) == 1;
  if (ok) hand_type = static_cast<HandType>(reg);
  return ok;
}

bool HandV6ModBus::_get_serial_number(std::string& serial_no) {
  if (!modbus_ctx_) return false;
  uint16_t regs[4] = {0};
  if (modbus_read_registers(modbus_ctx_, static_cast<uint16_t>(ModbusReg::CFG_SERIAL_NO), 4, regs) == 4) {
    char buf[9] = {0};
    memcpy(buf, regs, 8);
    serial_no = std::string(buf);
    return true;
  }
  return false;
}

} // namespace v6_sdk
