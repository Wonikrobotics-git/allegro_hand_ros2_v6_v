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
 * @file v6_spec.hpp
 * @brief Defines constants, Modbus register/coil types, and data structures for Allegro Hand V6.
 * @author ('c')void
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace v6_sdk {

/* =========================================================================
 * 1. Topology & Hardware Constants
 * ========================================================================= */
constexpr int FINGER_COUNT = 5;
constexpr int JOINTS_PER_FINGER = 4;

constexpr double ENCODER_LSB_PER_REV = 4096.0;
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double RAD2PUL = ENCODER_LSB_PER_REV / (2.0 * M_PI);
constexpr double PUL2RAD = (2.0 * M_PI) / ENCODER_LSB_PER_REV;

constexpr double TORQUE_CONSTANT = 1.15 / 1000.0; // Stall Torque 1.150 Nm/A -> 0.00115 Nm/mA
constexpr int16_t DEFAULT_CURRENT_LIMIT_MA = 200;

/* =========================================================================
 * 2. Finger, Joint & Sensor Index Enumerations
 * ========================================================================= */
enum class FingerIndex : uint8_t {
  FINGER_THUMB  = 0,
  FINGER_INDEX  = 1,
  FINGER_MIDDLE = 2,
  FINGER_RING   = 3,
  FINGER_PINKY  = 4
};

enum class JointIndex : uint8_t {
  // Thumb (Joints 0~3)
  JOINT_THUMB_0  = 0,
  JOINT_THUMB_1  = 1,
  JOINT_THUMB_2  = 2,
  JOINT_THUMB_3  = 3,

  // Index (Joints 4~7)
  JOINT_INDEX_0  = 4,
  JOINT_INDEX_1  = 5,
  JOINT_INDEX_2  = 6,
  JOINT_INDEX_3  = 7,

  // Middle (Joints 8~11)
  JOINT_MIDDLE_0 = 8,
  JOINT_MIDDLE_1 = 9,
  JOINT_MIDDLE_2 = 10,
  JOINT_MIDDLE_3 = 11,

  // Ring (Joints 12~15)
  JOINT_RING_0   = 12,
  JOINT_RING_1   = 13,
  JOINT_RING_2   = 14,
  JOINT_RING_3   = 15,

  // Pinky (Joints 16~19)
  JOINT_PINKY_0  = 16,
  JOINT_PINKY_1  = 17,
  JOINT_PINKY_2  = 18,
  JOINT_PINKY_3  = 19,

  JOINT_COUNT    = 20
};

constexpr int JOINT_COUNT = static_cast<int>(JointIndex::JOINT_COUNT);

enum class PressureIndex : uint8_t {
  // Thumb Tactile Sensors (Sensors 0~2)
  PRESSURE_THUMB_0  = 0,
  PRESSURE_THUMB_1  = 1,
  PRESSURE_THUMB_2  = 2,

  // Index Tactile Sensors (Sensors 3~5)
  PRESSURE_INDEX_0  = 3,
  PRESSURE_INDEX_1  = 4,
  PRESSURE_INDEX_2  = 5,

  // Middle Tactile Sensors (Sensors 6~8)
  PRESSURE_MIDDLE_0 = 6,
  PRESSURE_MIDDLE_1 = 7,
  PRESSURE_MIDDLE_2 = 8,

  // Ring Tactile Sensors (Sensors 9~11)
  PRESSURE_RING_0   = 9,
  PRESSURE_RING_1   = 10,
  PRESSURE_RING_2   = 11,

  // Pinky Tactile Sensors (Sensors 12~14)
  PRESSURE_PINKY_0  = 12,
  PRESSURE_PINKY_1  = 13,
  PRESSURE_PINKY_2  = 14,

  // Palm Tactile Sensors (Sensors 15~17)
  PRESSURE_PALM_0   = 15,
  PRESSURE_PALM_1   = 16,
  PRESSURE_PALM_2   = 17,

  PRESSURE_COUNT    = 18
};

constexpr int PRESSURE_COUNT = static_cast<int>(PressureIndex::PRESSURE_COUNT); 



/* =========================================================================
 * 3. Modbus Holding Register Memory Map (0x0000 ~ 0x0320)
 * ========================================================================= */
enum class ModbusReg : uint16_t {
  // --- Telemetry Block (Read-Only) ---
  GET_POS             = 0x0000, ///< [20] int16_t Feedback joint positions (pulse)
  GET_CUR             = 0x0014, ///< [20] int16_t Feedback joint currents (mA)
  GET_IMU             = 0x0028, ///< [4]  int16_t BNO055 IMU data (raw or quat [w,x,y,z])
  GET_PRESSURE_HPA    = 0x002C, ///< [18] uint16_t Pressure sensor values (hPa)
  GET_FINGER_TEMP     = 0x0040, ///< [20] uint16_t Joint motor temperatures (degC)
  GET_SENSOR_TEMP     = 0x0054, ///< [18] uint16_t Pressure sensor temperatures (100 LSB/degC)
  GET_JOINT_STATUS    = 0x0066, ///< [1]  uint16_t Hardware fault / Comm error bitmask
  GET_IMU_CALIB       = 0x0067, ///< [1]  uint8_t  IMU Calibration status bitmask
  GET_FW_VER          = 0x0070, ///< [1]  uint16_t Firmware version (e.g. 0x0100 = v1.0)
  GET_HAND_TYPE       = 0x0071, ///< [1]  uint16_t Hand Type (0: Left, 1: Right)

  // --- Real-Time Control Block (Read-Write) ---
  CMD_POS             = 0x0100, ///< [20] int16_t Target joint positions (pulse)
  CMD_CUR             = 0x0114, ///< [20] int16_t Target currents / current limit (mA)
  CMD_OP_MODE         = 0x0128, ///< [1]  uint16_t Operating mode (0:Current, 5:Position)
  CMD_GRASP_SAVE      = 0x0129, ///< [1]  uint16_t Grasp position save trigger (slots 1~5)
  CMD_GRASP_PLAY      = 0x012A, ///< [1]  uint16_t Grasp position play trigger (slots 1~5)

  // --- PID Gain & Motion Profile Block ---
  CMD_GAIN_KP         = 0x0200, ///< [20] uint16_t Joint P gains
  CMD_GAIN_KI         = 0x0214, ///< [20] uint16_t Joint I gains
  CMD_GAIN_KD         = 0x0228, ///< [20] uint16_t Joint D gains
  CMD_PROF_VEL        = 0x023C, ///< [40] uint32_t Velocity profile (2 regs/joint)
  CMD_PROF_ACC        = 0x0264, ///< [40] uint32_t Acceleration profile (2 regs/joint)

  // --- System & Network Configuration Block ---
  CFG_COMM_SEL        = 0x0300, ///< [1]  uint16_t RS485 Comm type (0:Modbus RTU, 1:Custom)
  CFG_SLAVE_ID        = 0x0301, ///< [1]  uint16_t Modbus Slave ID (1~247)
  CFG_IMU_CONFIG      = 0x0302, ///< [1]  uint16_t BNO055 IMU operation mode
  CFG_SYS_REV0        = 0x0303, ///< [1]  uint16_t RS485 Baudrate CFG (reserved)
  CFG_SYS_REV1        = 0x0304, ///< [1]  uint16_t Dev Option (reserved)
  CFG_NET_IP          = 0x0310, ///< [2]  uint32_t IPv4 Address
  CFG_NET_MASK        = 0x0312, ///< [2]  uint32_t Subnet Mask
  CFG_NET_GW          = 0x0314, ///< [2]  uint32_t Gateway Address
  CFG_NET_DNS         = 0x0316, ///< [2]  uint32_t DNS Address
  CFG_SERIAL_NO       = 0x0318, ///< [4]  uint64_t Hand Serial No (requires factory auth)
  CFG_FACTORY_PWD     = 0x0320  ///< [1]  uint16_t Factory Auth Password (0xA5A5)
};

/* =========================================================================
 * 4. Modbus Coil Address Map (Digital 1-bit Flags)
 * ========================================================================= */
enum class ModbusCoil : uint16_t {
  TORQUE_ENABLE       = 0x0000, ///< [1] bool R/W Servo torque (0: OFF, 1: ON)
  JOINT_CALIB         = 0x0001, ///< [1] bool R/W Joint calibration (0: Idle, 1: Start)
  FACTORY_AUTH_STATUS = 0x0010  ///< [1] bool RO  Factory auth status (0: Locked, 1: Unlocked)
};

/* =========================================================================
 * 5. Operating Mode Enumeration
 * ========================================================================= */
enum class OpMode : uint16_t {
  CURRENT_CONTROL     = 0x0000, ///< Current / Effort torque control mode
  POSITION_CONTROL    = 0x0005  ///< Current-based position control mode (Default)
};

/* =========================================================================
 * 6. BNO055 IMU Profile Enumeration
 * ========================================================================= */
enum class ImuProfile : uint16_t {
  RAW_ACCEL           = 0x0001, ///< Accelerometer raw [x, y, z, 0] (1 LSB = 1 mg)
  RAW_MAG             = 0x0002, ///< Magnetometer raw [x, y, z, 0] (1 LSB = 1/16 uT)
  RAW_GYRO            = 0x0003, ///< Gyroscope raw [x, y, z, 0] (1 LSB = 1/16 dps)
  IMU_FUSION          = 0x0008, ///< Accel + Gyro relative quaternion [w, x, y, z] (Default)
  COMPASS             = 0x0009, ///< Direction fusion
  M4G                 = 0x000A, ///< Magnetic based fusion
  NDOF_FMC_OFF        = 0x000B, ///< 9-DOF fusion without fast mag calib
  NDOF                = 0x000C  ///< 9-DOF full fusion absolute orientation
};

/* =========================================================================
 * 7. Hand Type Enumeration
 * ========================================================================= */
enum class HandType : uint16_t {
  LEFT_HAND           = 0x0000, ///< Left Hand
  RIGHT_HAND          = 0x0001  ///< Right Hand
};

/* =========================================================================
 * 8. Joint Status Bitmask Structure (0x0066)
 * ========================================================================= */
struct JointStatusBitmask {
  uint16_t raw_value{0};

  bool overload() const { return (raw_value & (1 << 0)) != 0; }
  bool electrical_shock() const { return (raw_value & (1 << 1)) != 0; }
  bool overheat() const { return (raw_value & (1 << 2)) != 0; }
  bool input_voltage() const { return (raw_value & (1 << 3)) != 0; }

  bool finger_comm_error(FingerIndex finger) const {
    return (raw_value & (1 << (4 + static_cast<int>(finger)))) != 0;
  }

  bool finger_fault(FingerIndex finger) const {
    return (raw_value & (1 << (9 + static_cast<int>(finger)))) != 0;
  }

  bool has_error() const { return (raw_value & 0x3FFF) != 0; }
};

/* =========================================================================
 * 9. IMU Calibration Status (0x0067)
 * ========================================================================= */
struct ImuCalibStatus {
  uint8_t raw_value{0};

  uint8_t mag_calib() const { return raw_value & 0x03; }
  uint8_t acc_calib() const { return (raw_value >> 2) & 0x03; }
  uint8_t gyr_calib() const { return (raw_value >> 4) & 0x03; }
  uint8_t sys_calib() const { return (raw_value >> 6) & 0x03; }

  bool is_fully_calibrated() const { return raw_value == 0xFF; }
};

/* =========================================================================
 * 10. Device Information & Diagnostics Data Structures
 * ========================================================================= */
/**
 * @struct DeviceInfo
 * @brief Static hardware and firmware metadata.
 */
struct DeviceInfo {
  uint16_t firmware_version{0};
  HandType hand_type{HandType::RIGHT_HAND};
  std::string serial_number;
};

/**
 * @struct HandDiagnostics
 * @brief Dynamic system status, fault bitmasks, and thermal states.
 */
struct HandDiagnostics {
  // System & Servo States
  bool torque_enabled{false};
  bool calibration_running{false};
  OpMode operation_mode{OpMode::POSITION_CONTROL};

  // Hardware Fault & IMU Calibration Bitmasks
  JointStatusBitmask joint_status;
  ImuCalibStatus imu_calib_status;

  // Thermal Diagnostics (20 joint motors + 18 tactile sensors)
  uint16_t motor_temperatures[JOINT_COUNT]{0};     ///< Motor temperatures in Celsius
  uint16_t sensor_temperatures[PRESSURE_COUNT]{0}; ///< Sensor temperatures in 100 LSB/degC
};

} // namespace v6_sdk