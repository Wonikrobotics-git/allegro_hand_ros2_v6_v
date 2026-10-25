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
 * @file v6_sdk.hpp
 * @brief Defines the abstract HandV6 interface and the Modbus-based HandV6ModBus SDK.
 * @author ('c')void
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <allegro_hand_utility/utility.hpp>

#include "v6_spec.hpp"

// Forward declaration for libmodbus context
struct _modbus;
typedef struct _modbus modbus_t;

using allegro_hand_utility::SignalRateMonitor;
using allegro_hand_utility::SpinLock;

namespace v6_sdk {

/**
 * @class HandV6
 * @brief Abstract base class for Allegro Hand V6 SDK.
 *
 * This class provides a protocol-agnostic interface for `ros2_control`
 * to read states and write commands.
 */
class HandV6 {
public:
  static const char* TORQUE_LIMIT_GROUP;

  DeviceInfo device_info_;
  SignalRateMonitor signal_rate_monitor_;

  HandV6() = default;
  inline const DeviceInfo& get_device_info() const { return device_info_; }
  inline const SignalRateMonitor& get_rate_monitor() const { return signal_rate_monitor_; }
  virtual ~HandV6() = default;

  /** @brief Checks if the hand is initialized and ready. */
  virtual bool is_ready() const = 0;

  /**
   * @brief Activates the control loop and sets the control mode.
   * @param position_control_mode True for position control, false for torque control.
   */
  virtual void set_control_mode(bool position_control_mode) = 0;

  /**
   * @brief Reads a specific group of gains from the SDK.
   * @param group_name The name of the gain group to read ('p_gain', 'd_gain', or 'torque_max').
   * @return A vector of 20 gain values ordered by JointIndex:
   *         - [0..3]   : Thumb  (joint00, joint01, joint02, joint03)
   *         - [4..7]   : Index  (joint10, joint11, joint12, joint13)
   *         - [8..11]  : Middle (joint20, joint21, joint22, joint23)
   *         - [12..15] : Ring   (joint30, joint31, joint32, joint33)
   *         - [16..19] : Pinky  (joint40, joint41, joint42, joint43)
   */
  virtual std::vector<double> read_gains(const char* group_name) = 0;

  /**
   * @brief Writes a specific group of gains to the SDK.
   * @param group_name The name of the gain group to write ('p_gain', 'd_gain', or 'torque_max').
   * @param gains A vector of 20 gain values ordered by JointIndex:
   *         - [0..3]   : Thumb  (joint00, joint01, joint02, joint03)
   *         - [4..7]   : Index  (joint10, joint11, joint12, joint13)
   *         - [8..11]  : Middle (joint20, joint21, joint22, joint23)
   *         - [12..15] : Ring   (joint30, joint31, joint32, joint33)
   *         - [16..19] : Pinky  (joint40, joint41, joint42, joint43)
   */
  virtual void write_gains(const char* group_name, const std::vector<double>& gains) = 0;

  /**
   * @brief Reads all high-rate real-time states (joints, IMU, tactile pressures) in a single fast FC03 transaction.
   * @param position_state Pointer to double[20] for joint positions in radians.
   * @param velocity_state Pointer to double[20] for joint velocities in rad/s.
   * @param torque_state Pointer to double[20] for joint torques in Nm.
   * @param imu_quat_state Pointer to double[4] for IMU quaternion [w, x, y, z].
   * @param pressure_state Pointer to double[18] for tactile sensor pressures in hPa.
   * @return True only when the complete fast feedback block was received.
   */
  virtual bool read_states(
      double* position_state = nullptr,
      double* velocity_state = nullptr,
      double* torque_state = nullptr,
      double* imu_quat_state = nullptr,
      double* pressure_state = nullptr) = 0;

  /**
   * @brief Writes commands from ros2_control to the SDK.
   * @param position_command Pointer to double[20] target positions in radians.
   * @param velocity_command Pointer to double[20] target velocities in rad/s (reserved/unused).
   * @param torque_command Pointer to double[20] target efforts in Nm.
   */
  virtual void write_commands(const double* position_command, const double* velocity_command, const double* torque_command) = 0;

  /**
   * @brief Reads dynamic system diagnostics, thermal states, and fault statuses.
   * @param diag Output HandDiagnostics structure to be populated.
   * @return True on successful Modbus read, false otherwise.
   */
  virtual bool read_diagnostics(HandDiagnostics& diag) = 0;

  /** @brief Turns servo torque ON or OFF (Thread-safe). */
  virtual bool set_torque_enabled(bool enabled) = 0;

  /** @brief Triggers joint zero calibration (Thread-safe). */
  virtual bool trigger_calibration() = 0;

  /** @brief Sets BNO055 IMU operation profile (Thread-safe). */
  virtual bool set_imu_profile(ImuProfile profile) = 0;

  /** @brief Saves current joint pose to on-device flash slot (1~5) (Thread-safe). */
  virtual bool save_grasp_pose(uint16_t slot_id) = 0;

  /** @brief Plays stored grasp pose from on-device flash slot (1~5) (Thread-safe). */
  virtual bool play_grasp_pose(uint16_t slot_id) = 0;
};

/**
 * @class HandV6ModBus
 * @brief Official Modbus (TCP / RTU) implementation of the HandV6 SDK.
 */
class HandV6ModBus : public HandV6 {
public:
  /**
   * @struct Options
   * @brief Configuration options for initializing the HandV6ModBus SDK.
   */
  struct Options {
    /// @brief Communication type: "tcp" or "rtu"
    std::string comm_type{"tcp"};
    /// @brief Modbus TCP IP address (e.g., "192.168.1.100")
    std::string ip_address{"192.168.1.100"};
    /// @brief Modbus TCP port (default 502)
    int tcp_port{502};
    /// @brief Modbus RTU serial device (e.g., "/dev/ttyUSB0")
    std::string serial_device{"/dev/ttyUSB0"};
    /// @brief Modbus RTU baudrate (default 2000000)
    int baudrate{2000000};
    /// @brief Modbus Slave ID (default 1)
    uint8_t slave_id{1};

    /// @brief Proportional gains for all 20 joints.
    std::vector<double> k_p_table;
    /// @brief Integral gains for all 20 joints.
    std::vector<double> k_i_table;
    /// @brief Derivative gains for all 20 joints.
    std::vector<double> k_d_table;
    /// @brief Torque limits for all 20 joints.
    std::vector<double> torque_limit_table;
  };

  ///// PARAMETER GROUPS ///////
  static const char* TORQUE_LIMIT_GROUP;

private:
  /// @brief Spinlock for thread-safe access to shared data.
  mutable SpinLock lock_;

  /// @brief Configuration options passed during construction.
  Options options_;

  /// @brief libmodbus context handle.
  modbus_t* modbus_ctx_{nullptr};
  bool is_connected_{false};

  struct {
    int64_t stamp_ns{0}; ///< Nanoseconds from epoch
    double positions[JOINT_COUNT]{0.0};
    std::vector<allegro_hand_utility::LowpassFilter> velocity_filters;
  } prv_state_;

  /** @brief Initializes the communication interface and the hand. */
  bool _initialize();

  /* =========================================================================
   * 1. Joint Commands & Control (Holding Regs)
   * ========================================================================= */
  /// @brief Set target joint positions for all 20 joints in pulse (Holding Reg 0x0100, 20 regs).
  bool _set_goal_positions(const int16_t* positions_pulse);
  bool _set_goal_position(JointIndex joint, int16_t position_pulse);

  /// @brief Set target currents / current limits for all 20 joints in mA (Holding Reg 0x0114, 20 regs).
  bool _set_goal_currents(const int16_t* currents_ma);
  bool _set_goal_current(JointIndex joint, int16_t current_ma);

  /// @brief Set joint operation mode (Holding Reg 0x0128).
  bool _set_operation_mode(OpMode mode);

  /// @brief Set joint PID gains (Holding Reg 0x0200, 0x0214, 0x0228, 20 regs each or single joint).
  bool _set_gains_kp(const uint16_t* kp);
  bool _set_gain_kp(JointIndex joint, uint16_t kp);
  bool _set_gains_ki(const uint16_t* ki);
  bool _set_gain_ki(JointIndex joint, uint16_t ki);
  bool _set_gains_kd(const uint16_t* kd);
  bool _set_gain_kd(JointIndex joint, uint16_t kd);

  /* =========================================================================
   * 2. Sensor Configuration (Holding Regs)
   * ========================================================================= */
  /// @brief Set BNO055 IMU output profile (Holding Reg 0x0302).
  bool _set_imu_config(ImuProfile profile);

  /* =========================================================================
   * 3. Digital Controls & System Settings (Coils & Grasp Memory)
   * ========================================================================= */
  /// @brief Turn servo torque ON or OFF (Coil 0x0000).
  bool _set_torque_on_off(bool on_off);

  /// @brief Trigger joint calibration (Coil 0x0001).
  bool _trigger_joint_calibration(bool start);

  /// @brief Save current hand pose to on-device flash slot (Holding Reg 0x0129, slots 1~5).
  bool _save_grasp_slot(uint16_t slot_id);

  /// @brief Play stored grasp pose from on-device flash slot (Holding Reg 0x012A, slots 1~5).
  bool _play_grasp_slot(uint16_t slot_id);

  /* =========================================================================
   * 4. Joint Feedback & States (Holding Regs Read)
   * ========================================================================= */
  /// @brief Get feedback positions for all 20 joints in pulse (Holding Reg 0x0000, 20 regs).
  bool _get_feedback_positions(int16_t* positions_pulse);
  bool _get_feedback_position(JointIndex joint, int16_t& position_pulse);

  /// @brief Get feedback currents for all 20 joints in mA (Holding Reg 0x0014, 20 regs).
  bool _get_feedback_currents(int16_t* currents_ma);
  bool _get_feedback_current(JointIndex joint, int16_t& current_ma);

  /// @brief Get motor temperatures for all 20 joints in Celsius (Holding Reg 0x0040, 20 regs).
  bool _get_motor_temperatures(uint16_t* temps_c);
  bool _get_motor_temperature(JointIndex joint, uint16_t& temp_c);

  /// @brief Get joint operation mode (Holding Reg 0x0128).
  bool _get_operation_mode(OpMode& mode);

  /// @brief Get joint PID gains (Holding Reg 0x0200, 0x0214, 0x0228, 20 regs each or single joint).
  bool _get_gains_kp(uint16_t* kp);
  bool _get_gain_kp(JointIndex joint, uint16_t& kp);
  bool _get_gains_ki(uint16_t* ki);
  bool _get_gain_ki(JointIndex joint, uint16_t& ki);
  bool _get_gains_kd(uint16_t* kd);
  bool _get_gain_kd(JointIndex joint, uint16_t& kd);

  /* =========================================================================
   * 5. Sensor Telemetry (Holding Regs Read)
   * ========================================================================= */
  /// @brief Get BNO055 IMU data (Holding Reg 0x0028, 4 regs: raw or quaternion [w,x,y,z]).
  bool _get_imu_data(int16_t* imu_data);

  /// @brief Get tactile pressure sensor values in hPa (Holding Reg 0x002C, 18 regs: 15 fingers + 3 palm).
  bool _get_pressures_hpa(uint16_t* pressures_hpa);
  bool _get_pressure_hpa(PressureIndex sensor, uint16_t& pressure_hpa);

  /// @brief Get pressure sensor temperatures (Holding Reg 0x0054, 18 regs).
  bool _get_sensor_temperatures(uint16_t* temps_raw);
  bool _get_sensor_temperature(PressureIndex sensor, uint16_t& temp_raw);

  /// @brief Get BNO055 IMU calibration status (Holding Reg 0x0067).
  bool _get_imu_calibration_status(ImuCalibStatus& calib_status);

  /* =========================================================================
   * 6. System Diagnostics & Device Info (Coils & Holding Regs Read)
   * ========================================================================= */
  /// @brief Get servo torque ON/OFF state (Coil 0x0000).
  bool _get_torque_status(bool& on_off);

  /// @brief Get joint calibration running state (Coil 0x0001).
  bool _get_calibration_status(bool& is_running);

  /// @brief Get hardware fault / communication error bitmask (Holding Reg 0x0066).
  bool _get_joint_status(JointStatusBitmask& status_bitmask);

  /// @brief Get firmware version (Holding Reg 0x0070).
  bool _get_firmware_version(uint16_t& fw_ver);

  /// @brief Get hand type (Holding Reg 0x0071).
  bool _get_hand_type(HandType& hand_type);

  /// @brief Get hand serial number (Holding Reg 0x0318, 4 regs / 8 bytes).
  bool _get_serial_number(std::string& serial_no);

public:
  /**
   * @brief Constructs the HandV6ModBus object.
   * @param options Configuration options including Modbus connection parameters and PD gains.
   * @throws std::runtime_error if initialization fails.
   */
  HandV6ModBus(const Options& options);
  /** @brief Destructor, ensures the hand is powered off safely. */
  virtual ~HandV6ModBus();

  /** @brief Checks if the hand is initialized and ready. */
  inline bool is_ready() const override {
    return is_connected_ && (modbus_ctx_ != nullptr);
  }

  /**
   * @brief Activates the control loop and sets the control mode.
   * @param position_control_mode True for position control, false for torque control.
   */
  void set_control_mode(bool position_control_mode) override;

  /**
   * @brief Reads a specific group of gains from the SDK.
   * @param group_name The name of the gain group to read ('p_gain', 'd_gain', or 'torque_max').
   * @return A vector of 20 gain values ordered by JointIndex:
   *         - [0..3]   : Thumb  (joint00, joint01, joint02, joint03)
   *         - [4..7]   : Index  (joint10, joint11, joint12, joint13)
   *         - [8..11]  : Middle (joint20, joint21, joint22, joint23)
   *         - [12..15] : Ring   (joint30, joint31, joint32, joint33)
   *         - [16..19] : Pinky  (joint40, joint41, joint42, joint43)
   */
  std::vector<double> read_gains(const char* group_name) override;

  /**
   * @brief Writes a specific group of gains to the SDK.
   * @param group_name The name of the gain group to write ('p_gain', 'd_gain', or 'torque_max').
   * @param gains A vector of 20 gain values ordered by JointIndex:
   *         - [0..3]   : Thumb  (joint00, joint01, joint02, joint03)
   *         - [4..7]   : Index  (joint10, joint11, joint12, joint13)
   *         - [8..11]  : Middle (joint20, joint21, joint22, joint23)
   *         - [12..15] : Ring   (joint30, joint31, joint32, joint33)
   *         - [16..19] : Pinky  (joint40, joint41, joint42, joint43)
   */
  void write_gains(const char* group_name, const std::vector<double>& gains) override;

  /** @brief Reads all high-rate real-time states (joints, IMU, tactile pressures) for ros2_control. */
  bool read_states(
      double* position_state = nullptr,
      double* velocity_state = nullptr,
      double* torque_state = nullptr,
      double* imu_quat_state = nullptr,
      double* pressure_state = nullptr) override;

  /** @brief Writes commands from ros2_control to the SDK. */
  void write_commands(const double* position_command, const double* velocity_command, const double* torque_command) override;

  /** @brief Reads dynamic system diagnostics, thermal states, and fault statuses. */
  bool read_diagnostics(HandDiagnostics& diag) override;

  /** @brief Turns servo torque ON or OFF (Thread-safe). */
  bool set_torque_enabled(bool enabled) override;

  /** @brief Triggers joint zero calibration (Thread-safe). */
  bool trigger_calibration() override;

  /** @brief Sets BNO055 IMU operation profile (Thread-safe). */
  bool set_imu_profile(ImuProfile profile) override;

  /** @brief Saves current joint pose to on-device flash slot (1~5) (Thread-safe). */
  bool save_grasp_pose(uint16_t slot_id) override;

  /** @brief Plays stored grasp pose from on-device flash slot (1~5) (Thread-safe). */
  bool play_grasp_pose(uint16_t slot_id) override;
};

/**
 * @class HandV6Serial
 * @brief Placeholder implementation for Protocol Buffers / high-speed serial interface.
 */
class HandV6Serial : public HandV6 {
public:
  struct Options {
    std::string serial_device{"/dev/ttyUSB0"};
    int baudrate{2000000};
    uint8_t slave_id{1};
    std::vector<double> k_p_table;
    std::vector<double> k_i_table;
    std::vector<double> k_d_table;
    std::vector<double> torque_limit_table;
  };

  HandV6Serial() = default;
  explicit HandV6Serial(const Options& /*options*/) {}
  virtual ~HandV6Serial() override = default;

  bool is_ready() const override { return false; }
  void set_control_mode(bool /*position_control_mode*/) override {}
  std::vector<double> read_gains(const char* /*group_name*/) override { return {}; }
  void write_gains(const char* /*group_name*/, const std::vector<double>& /*gains*/) override {}
  bool read_states(
      double* /*position_state*/ = nullptr,
      double* /*velocity_state*/ = nullptr,
      double* /*torque_state*/ = nullptr,
      double* /*imu_quat_state*/ = nullptr,
      double* /*pressure_state*/ = nullptr) override { return false; }
  void write_commands(
      const double* /*position_command*/,
      const double* /*velocity_command*/,
      const double* /*torque_command*/) override {}
  bool read_diagnostics(HandDiagnostics& /*diag*/) override { return false; }
  bool set_torque_enabled(bool /*enabled*/) override { return false; }
  bool trigger_calibration() override { return false; }
  bool set_imu_profile(ImuProfile /*profile*/) override { return false; }
  bool save_grasp_pose(uint16_t /*slot_id*/) override { return false; }
  bool play_grasp_pose(uint16_t /*slot_id*/) override { return false; }
};

} // namespace v6_sdk
