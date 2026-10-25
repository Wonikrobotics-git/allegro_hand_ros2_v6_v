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
 * @file v6_hardware_interface.cpp
 * @brief Implements the ros2_control hardware interface for Allegro Hand V6.
 * @author ('c')void
 */

#include "allegro_hand_v6_hardware/hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <spdlog/spdlog.h>

#include "stopwatch.hpp"
#include "csv_logger.hpp"

using allegro_hand_utility::HardwareInfoParser;

namespace allegro_hand_v6_hardware {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("AllegroHandV6HardwareInterface");

/**
 * @class Diagnostics
 * @brief Manages and publishes diagnostic telemetry for Allegro Hand V6.
 */
class Diagnostics {
  rclcpp::Node* node_;
  std::shared_ptr<v6_sdk::HandV6> device_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  void hardware_status_task(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    if (!device_ || !device_->is_ready()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device not connected or not ready.");
      stat.add("Status", "Not Initialized");
      return;
    }

    const auto& dev_info = device_->get_device_info();
    v6_sdk::HandDiagnostics diag;
    bool ok = device_->read_diagnostics(diag);

    if (!ok) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Failed to read diagnostics via Modbus.");
      return;
    }

    if (diag.joint_status.has_error()) {
      std::string err_msg = "Device has errors: ";
      if (diag.joint_status.overload()) err_msg += "Overload ";
      if (diag.joint_status.electrical_shock()) err_msg += "ElectricalShock ";
      if (diag.joint_status.overheat()) err_msg += "Overheat ";
      if (diag.joint_status.input_voltage()) err_msg += "InputVoltage ";
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, err_msg);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Device is OK.");
    }

    stat.add("Serial Number", dev_info.serial_number);
    stat.add("Firmware Version", fmt::format("0x{:04X}", dev_info.firmware_version));
    stat.add("Hand Type", dev_info.hand_type == v6_sdk::HandType::RIGHT_HAND ? "Right" : "Left");
    stat.add("Torque Enabled", diag.torque_enabled ? "ON" : "OFF");
    stat.add("Calibration Running", diag.calibration_running ? "YES" : "NO");
    stat.add("Operation Mode", diag.operation_mode == v6_sdk::OpMode::POSITION_CONTROL ? "Position" : "Torque");

    // Motor Temperatures (20 joints grouped by 5 fingers)
    stat.addf("Thumb Motor Temp (C)", "%u, %u, %u, %u",
              diag.motor_temperatures[0], diag.motor_temperatures[1], diag.motor_temperatures[2], diag.motor_temperatures[3]);
    stat.addf("Index Motor Temp (C)", "%u, %u, %u, %u",
              diag.motor_temperatures[4], diag.motor_temperatures[5], diag.motor_temperatures[6], diag.motor_temperatures[7]);
    stat.addf("Middle Motor Temp (C)", "%u, %u, %u, %u",
              diag.motor_temperatures[8], diag.motor_temperatures[9], diag.motor_temperatures[10], diag.motor_temperatures[11]);
    stat.addf("Ring Motor Temp (C)", "%u, %u, %u, %u",
              diag.motor_temperatures[12], diag.motor_temperatures[13], diag.motor_temperatures[14], diag.motor_temperatures[15]);
    stat.addf("Pinky Motor Temp (C)", "%u, %u, %u, %u",
              diag.motor_temperatures[16], diag.motor_temperatures[17], diag.motor_temperatures[18], diag.motor_temperatures[19]);

    // Tactile Sensor Temperatures (18 sensors, raw / 100 degC)
    stat.addf("Thumb Tactile Temp (C)", "%.2f, %.2f, %.2f",
              diag.sensor_temperatures[0] / 100.0, diag.sensor_temperatures[1] / 100.0, diag.sensor_temperatures[2] / 100.0);
    stat.addf("Index Tactile Temp (C)", "%.2f, %.2f, %.2f",
              diag.sensor_temperatures[3] / 100.0, diag.sensor_temperatures[4] / 100.0, diag.sensor_temperatures[5] / 100.0);
    stat.addf("Middle Tactile Temp (C)", "%.2f, %.2f, %.2f",
              diag.sensor_temperatures[6] / 100.0, diag.sensor_temperatures[7] / 100.0, diag.sensor_temperatures[8] / 100.0);
    stat.addf("Ring Tactile Temp (C)", "%.2f, %.2f, %.2f",
              diag.sensor_temperatures[9] / 100.0, diag.sensor_temperatures[10] / 100.0, diag.sensor_temperatures[11] / 100.0);
    stat.addf("Pinky Tactile Temp (C)", "%.2f, %.2f, %.2f",
              diag.sensor_temperatures[12] / 100.0, diag.sensor_temperatures[13] / 100.0, diag.sensor_temperatures[14] / 100.0);
    stat.addf("Palm Tactile Temp (C)", "%.2f, %.2f, %.2f",
              diag.sensor_temperatures[15] / 100.0, diag.sensor_temperatures[16] / 100.0, diag.sensor_temperatures[17] / 100.0);

    // IMU Calibration Status
    stat.addf("IMU Calibration", "Mag:%u Acc:%u Gyr:%u Sys:%u (Fully Calib: %s)",
              diag.imu_calib_status.mag_calib(), diag.imu_calib_status.acc_calib(),
              diag.imu_calib_status.gyr_calib(), diag.imu_calib_status.sys_calib(),
              diag.imu_calib_status.is_fully_calibrated() ? "YES" : "NO");

    // Signal rate monitoring
    const auto& rate_info = device_->get_rate_monitor().get_info();
    for (const auto& pair : rate_info) {
      if (pair.second.rtt_count > 0) {
        stat.addf("Signal: " + pair.first,
                  "Freq: %.2f Hz, Jitter: %.3f ms, RTT: %.3f ms (Jitter: %.3f ms), Cnt: %d",
                  pair.second.frequency, pair.second.jitter * 1000.0,
                  pair.second.rtt_mean * 1000.0, pair.second.rtt_jitter * 1000.0,
                  pair.second.count);
      } else {
        stat.addf("Signal: " + pair.first, "Freq: %.2f Hz, Jitter: %.3f ms, Cnt: %d",
                  pair.second.frequency, pair.second.jitter * 1000.0, pair.second.count);
      }
    }
  }

public:
  explicit Diagnostics(rclcpp::Node* node) : node_(node), device_(nullptr) {
    updater_ = std::make_unique<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID("allegro_hand_v6");
    updater_->add("Hardware Status", this, &Diagnostics::hardware_status_task);
  }

  inline void update() { updater_->force_update(); }
  inline void bind_device(const std::shared_ptr<v6_sdk::HandV6>& device) { device_ = device; }

  bool set_parameter(const rclcpp::Parameter& parameter) {
    if (parameter.get_name() == "diagnostic_updater.period") {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        updater_->setPeriod(parameter.as_double());
        return true;
      }
    }
    return false;
  }
};

/**
 * @class ParameterHandler
 * @brief Manages runtime parameters for torque limits and joint gains.
 */
class ParameterHandler {
  rclcpp::Node* node_;
  std::shared_ptr<HardwareInfoParser> info_parser_;
  std::shared_ptr<v6_sdk::HandV6> device_;
  std::shared_ptr<Diagnostics> diagnostics_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_set_callback_;

  std::map<std::string, double> torque_limit_table_;
  std::map<std::string, double> initial_position_table_;

  std::string encode_finger_parameter_name(const std::string& param_group, const std::string& joint_name) {
    auto finger_name = info_parser_->get_finger_name(joint_name);
    return fmt::format("{}/{}/{}", param_group, finger_name, joint_name);
  }

  std::pair<std::string, std::string> decode_finger_parameter_name(const std::string& parameter_name) {
    std::vector<std::string> parts;
    boost::split(parts, parameter_name, boost::is_any_of("/"));
    if (parts.size() != 3) {
      return {};
    }
    return {parts[0], parts[2]};
  }

  rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
    for (const auto& param : parameters) {
      if (param.get_name().find("diagnostic_updater") != std::string::npos) {
        if (diagnostics_) diagnostics_->set_parameter(param);
        continue;
      }

      auto [param_group_name, joint_name] = decode_finger_parameter_name(param.get_name());
      if (param_group_name.empty() || joint_name.empty()) {
        auto res = rcl_interfaces::msg::SetParametersResult();
        res.successful = false;
        res.reason = "Invalid parameter name format";
        return res;
      }

      if (param_group_name == v6_sdk::HandV6::TORQUE_LIMIT_GROUP) {
        torque_limit_table_[joint_name] = param.as_double();
        if (device_) {
          device_->write_gains(param_group_name.c_str(), serialize_table(torque_limit_table_));
        }
      }
    }

    auto res = rcl_interfaces::msg::SetParametersResult();
    res.successful = true;
    return res;
  }

  std::map<std::string, double> deserialize_table(const std::vector<double>& values) const {
    const auto& names = info_parser_->sdk_ordered_joint_base_names();
    std::map<std::string, double> map_out;
    for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
      map_out[names[i]] = values[i];
    }
    return map_out;
  }

  std::vector<double> serialize_table(const std::map<std::string, double>& table) const {
    std::vector<double> vec;
    const auto& names = info_parser_->sdk_ordered_joint_base_names();
    vec.reserve(names.size());
    for (const auto& name : names) {
      auto it = table.find(name);
      vec.push_back(it != table.end() ? it->second : 0.0);
    }
    return vec;
  }

  void init_parameters() {
    torque_limit_table_ = deserialize_table(info_parser_->torque_limit_table());
    initial_position_table_ = deserialize_table(info_parser_->initial_position_table());

    auto torque_limit_desc = rcl_interfaces::msg::ParameterDescriptor();
    torque_limit_desc.description = "Maximum torque limit (Nm) for joint.";
    torque_limit_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;

    auto initial_pos_desc = rcl_interfaces::msg::ParameterDescriptor();
    initial_pos_desc.description = "Initial joint position (rad).";
    initial_pos_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    initial_pos_desc.read_only = true;

    for (const auto& joint_name : info_parser_->sdk_ordered_joint_base_names()) {
      node_->declare_parameter(
          encode_finger_parameter_name(v6_sdk::HandV6::TORQUE_LIMIT_GROUP, joint_name),
          torque_limit_table_[joint_name], torque_limit_desc);
      node_->declare_parameter(
          encode_finger_parameter_name("initial_position", joint_name),
          initial_position_table_[joint_name], initial_pos_desc);
    }
  }

public:
  ParameterHandler(rclcpp::Node* node, const std::shared_ptr<HardwareInfoParser>& info_parser,
                   const std::shared_ptr<Diagnostics>& diagnostics)
      : node_(node), info_parser_(info_parser), device_(nullptr), diagnostics_(diagnostics) {
    init_parameters();
    param_set_callback_ = node_->get_node_parameters_interface()->add_on_set_parameters_callback(
        std::bind(&ParameterHandler::set_parameters_callback, this, std::placeholders::_1));
  }

  inline void bind_device(const std::shared_ptr<v6_sdk::HandV6>& device) { device_ = device; }
  inline std::vector<double> torque_limit_table() const { return serialize_table(torque_limit_table_); }
  inline std::vector<double> initial_position_table() const { return serialize_table(initial_position_table_); }
};

/* =========================================================================
 * AllegroHandV6HardwareInterface Implementation
 * ========================================================================= */

hardware_interface::CallbackReturn AllegroHandV6HardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = params.hardware_info;

  const auto csv_enabled = info_.hardware_parameters.find("csv_logging");
  if (csv_enabled != info_.hardware_parameters.end()) {
    const auto& value = csv_enabled->second;
    if (value != "true" && value != "True" && value != "1" &&
        value != "false" && value != "False" && value != "0") {
      RCLCPP_ERROR(LOGGER, "csv_logging must be true or false");
      return hardware_interface::CallbackReturn::ERROR;
    }
    csv_logging_enabled_ = value == "true" || value == "True" || value == "1";
  }
  const auto csv_directory = info_.hardware_parameters.find("csv_directory");
  if (csv_directory != info_.hardware_parameters.end()) csv_directory_ = csv_directory->second;

  // Configure 20 joint base names in official V6 SDK order
  HardwareInfoParser::Parameters parser_param;
  parser_param.sdk_ordered_joint_base_names = {
      "joint00", "joint01", "joint02", "joint03", // Thumb (0~3)
      "joint10", "joint11", "joint12", "joint13", // Index (4~7)
      "joint20", "joint21", "joint22", "joint23", // Middle (8~11)
      "joint30", "joint31", "joint32", "joint33", // Ring (12~15)
      "joint40", "joint41", "joint42", "joint43", // Pinky (16~19)
  };

  parser_param.default_p_gain_table.assign(JOINT_NUM, 1.0);
  parser_param.default_d_gain_table.assign(JOINT_NUM, 0.1);
  parser_param.default_torque_limit_table.assign(JOINT_NUM, 0.23); // Default 0.23 Nm (~200mA)

  // Ensure safe fallback parameters for HardwareInfoParser
  if (info_.hardware_parameters.find("hand_id") == info_.hardware_parameters.end()) {
    info_.hardware_parameters["hand_id"] = "1";
  }
  if (info_.hardware_parameters.find("io_interface_descriptor") == info_.hardware_parameters.end()) {
    info_.hardware_parameters["io_interface_descriptor"] = "modbus_tcp:192.168.1.100:502";
  }

  hwinfo_parser_ = std::make_shared<HardwareInfoParser>(parser_param, info_);

  if (info_.joints.size() != static_cast<size_t>(JOINT_NUM)) {
    RCLCPP_FATAL(LOGGER, "Invalid joint count: %zu (expected %d for Allegro Hand V6).", info_.joints.size(), JOINT_NUM);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create dedicated auxiliary ROS 2 node for parameters, diagnostics, and topic publishers
  node_ = std::make_shared<rclcpp::Node>(hwinfo_parser_->dev_node_name());
  diagnostics_handler_ = std::make_shared<Diagnostics>(node_.get());
  param_handler_ = std::make_shared<ParameterHandler>(node_.get(), hwinfo_parser_, diagnostics_handler_);

  // Create real-time topic publishers
  auto imu_pub = node_->create_publisher<sensor_msgs::msg::Imu>("~/imu/data", rclcpp::SystemDefaultsQoS());
  imu_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(imu_pub);

  auto pressure_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>("~/tactile_pressures", rclcpp::SystemDefaultsQoS());
  pressure_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pressure_pub);
  pressure_publisher_->msg_.data.resize(PRESSURE_NUM);

  // Setup periodic diagnostic timer (1000ms)
  using namespace std::chrono_literals;
  node_->create_wall_timer(1000ms, [this]() {
    if (diagnostics_handler_) {
      diagnostics_handler_->update();
    }
  });

  csv_status_timer_ = node_->create_wall_timer(1000ms, [this]() {
    if (!csv_logger_) return;
    if (csv_logger_->write_failed()) {
      RCLCPP_ERROR_THROTTLE(LOGGER, *node_->get_clock(), 5000,
          "CSV write failed: %s (control continues)", csv_logger_->path().c_str());
    }
    if (csv_logger_->dropped() > 0) {
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 5000,
          "CSV samples dropped: %llu", static_cast<unsigned long long>(csv_logger_->dropped()));
    }
  });

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AllegroHandV6HardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AllegroHandV6HardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(LOGGER, "Activating Allegro Hand V6 Hardware Interface...");

  if (pressure_publisher_) {
    pressure_publisher_->msg_.data.resize(PRESSURE_NUM);
  }

  memset(joint_position_command_, 0, sizeof(joint_position_command_));
  memset(joint_velocity_command_, 0, sizeof(joint_velocity_command_));
  memset(joint_torque_command_, 0, sizeof(joint_torque_command_));

  memset(joint_position_state_, 0, sizeof(joint_position_state_));
  memset(joint_velocity_state_, 0, sizeof(joint_velocity_state_));
  memset(joint_torque_state_, 0, sizeof(joint_torque_state_));
  memset(joint_temperature_state_, 0, sizeof(joint_temperature_state_));

  memset(imu_quat_state_, 0, sizeof(imu_quat_state_));
  imu_quat_state_[0] = 1.0; // w=1.0 identity
  memset(pressure_state_, 0, sizeof(pressure_state_));

  // Initialize commands from initial position & torque limit tables
  auto init_positions = param_handler_->initial_position_table();
  auto init_torques = param_handler_->torque_limit_table();
  for (int i = 0; i < JOINT_NUM; ++i) {
    if (i < static_cast<int>(init_positions.size())) joint_position_command_[i] = init_positions[i];
    if (i < static_cast<int>(init_torques.size())) joint_torque_command_[i] = init_torques[i];
  }

  // Parse communication options from descriptor or parameters
  std::string desc = hwinfo_parser_->io_interface_descriptor();

  if (desc.rfind("modbus_rtu:", 0) == 0) {
    v6_sdk::HandV6ModBus::Options options;
    options.comm_type = "rtu";
    std::string s = desc.substr(11);
    size_t comma = s.find(',');
    if (comma != std::string::npos) {
      options.serial_device = s.substr(0, comma);
      options.baudrate = std::stoi(s.substr(comma + 1));
    } else {
      options.serial_device = s;
    }
    options.slave_id = hwinfo_parser_->hand_id();
    options.torque_limit_table = param_handler_->torque_limit_table();

    try {
      device_ = std::make_shared<v6_sdk::HandV6ModBus>(options);
      diagnostics_handler_->bind_device(device_);
      param_handler_->bind_device(device_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(LOGGER, "Failed to instantiate HandV6 SDK: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (desc.rfind("modbus_tcp:", 0) == 0) {
    v6_sdk::HandV6ModBus::Options options;
    options.comm_type = "tcp";
    std::string s = desc.substr(11);
    size_t colon = s.find(':');
    if (colon != std::string::npos) {
      options.ip_address = s.substr(0, colon);
      options.tcp_port = std::stoi(s.substr(colon + 1));
    } else {
      options.ip_address = s;
    }
    options.slave_id = hwinfo_parser_->hand_id();
    options.torque_limit_table = param_handler_->torque_limit_table();

    try {
      device_ = std::make_shared<v6_sdk::HandV6ModBus>(options);
      diagnostics_handler_->bind_device(device_);
      param_handler_->bind_device(device_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(LOGGER, "Failed to instantiate HandV6 SDK: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (desc.rfind("serial:", 0) == 0 || desc.rfind("protobuf:", 0) == 0) {
    v6_sdk::HandV6Serial::Options options;
    std::string s = (desc.rfind("serial:", 0) == 0) ? desc.substr(7) : desc.substr(9);
    size_t comma = s.find(',');
    if (comma != std::string::npos) {
      options.serial_device = s.substr(0, comma);
      options.baudrate = std::stoi(s.substr(comma + 1));
    } else {
      options.serial_device = s;
    }
    options.slave_id = hwinfo_parser_->hand_id();
    options.torque_limit_table = param_handler_->torque_limit_table();

    try {
      device_ = std::make_shared<v6_sdk::HandV6Serial>(options);
      diagnostics_handler_->bind_device(device_);
      param_handler_->bind_device(device_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(LOGGER, "Failed to instantiate HandV6Serial SDK: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Unsupported communication descriptor: %s", desc.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (device_->is_ready() && csv_logging_enabled_) {
    try {
      const auto hand = device_->get_device_info().hand_type == v6_sdk::HandType::LEFT_HAND ? "left" : "right";
      csv_logger_ = std::make_shared<CsvLogger>(csv_directory_, hand, hwinfo_parser_->sdk_ordered_joint_names());
      RCLCPP_INFO(LOGGER, "CSV recording: %s (timestamp: KST, host read completion)", csv_logger_->path().c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "Cannot start CSV recording: %s", e.what());
      device_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  executor_thread_ = std::thread([this]() { executor_->spin(); });

  if (device_->is_ready()) {
    RCLCPP_INFO(LOGGER, "Allegro Hand V6 Hardware Interface activated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  RCLCPP_FATAL(LOGGER, "Allegro Hand V6 device is not ready.");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn AllegroHandV6HardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(LOGGER, "Deactivating Allegro Hand V6 Hardware Interface...");

  if (executor_) executor_->cancel();
  if (executor_thread_.joinable()) executor_thread_.join();

  if (device_) {
    if (!device_->set_torque_enabled(false)) {
      RCLCPP_WARN(LOGGER, "Failed to disable motor torque during deactivation");
    }
    device_.reset();
  }

  // Release motor torque before waiting for the disk writer to drain.
  if (csv_logger_) {
    csv_logger_->stop();
    RCLCPP_INFO(LOGGER, "CSV stopped: rows=%llu, dropped=%llu, failed_reads=%llu, write_failed=%s, file=%s",
        static_cast<unsigned long long>(csv_logger_->written()),
        static_cast<unsigned long long>(csv_logger_->dropped()),
        static_cast<unsigned long long>(csv_logger_->failed_reads()),
        csv_logger_->write_failed() ? "true" : "false", csv_logger_->path().c_str());
    csv_logger_.reset();
  }

  RCLCPP_INFO(LOGGER, "Allegro Hand V6 Hardware Interface stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AllegroHandV6HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();

  // 1. Joint State Interfaces (20 joints)
  for (size_t i = 0; i < joint_names.size() && i < static_cast<size_t>(JOINT_NUM); ++i) {
    const auto& name = joint_names[i];
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, hardware_interface::HW_IF_POSITION, &joint_position_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, hardware_interface::HW_IF_EFFORT, &joint_torque_state_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(name, "temperature", &joint_temperature_state_[i]));
  }

  // 2. IMU Sensor State Interfaces (if configured)
  if (!info_.sensors.empty()) {
    const auto& sensor = info_.sensors[0];
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.w", &imu_quat_state_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.x", &imu_quat_state_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.y", &imu_quat_state_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, "orientation.z", &imu_quat_state_[3]));
  }

  // 3. Tactile Pressure Sensor State Interfaces (18 sensors)
  for (int i = 0; i < PRESSURE_NUM; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface("tactile_pressure", fmt::format("pressure_{}", i), &pressure_state_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AllegroHandV6HardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();

  for (size_t i = 0; i < joint_names.size() && i < static_cast<size_t>(JOINT_NUM); ++i) {
    const auto& name = joint_names[i];
    command_interfaces.emplace_back(hardware_interface::CommandInterface(name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(name, hardware_interface::HW_IF_EFFORT, &joint_torque_command_[i]));
  }

  return command_interfaces;
}

static std::vector<std::string> filter_local_interfaces(const std::set<std::string>& joint_name_set,
                                                        const std::vector<std::string>& interfaces) {
  std::vector<std::string> local_interfaces;
  local_interfaces.reserve(interfaces.size());

  for (const auto& full_name : interfaces) {
    size_t pos = full_name.find_last_of('/');
    if (pos != std::string::npos) {
      if (joint_name_set.count(full_name.substr(0, pos)) > 0) {
        local_interfaces.push_back(full_name);
      }
    }
  }
  return local_interfaces;
}

hardware_interface::return_type AllegroHandV6HardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  const std::set<std::string> expected_joints(joint_names.begin(), joint_names.end());

  std::vector<std::string> local_start = filter_local_interfaces(expected_joints, start_interfaces);
  std::vector<std::string> local_stop = filter_local_interfaces(expected_joints, stop_interfaces);

  if (local_start.empty() && local_stop.empty()) {
    return hardware_interface::return_type::OK;
  }

  if (!local_start.empty()) {
    bool has_position = false;
    bool has_effort = false;

    for (const auto& iface : local_start) {
      if (iface.find("position") != std::string::npos) has_position = true;
      if (iface.find("effort") != std::string::npos) has_effort = true;
    }

    if (!has_position && !has_effort) {
      RCLCPP_FATAL(LOGGER, "Unsupported command mode switch request.");
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AllegroHandV6HardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  const auto& joint_names = hwinfo_parser_->sdk_ordered_joint_names();
  const std::set<std::string> expected_joints(joint_names.begin(), joint_names.end());

  std::vector<std::string> local_start = filter_local_interfaces(expected_joints, start_interfaces);
  std::vector<std::string> local_stop = filter_local_interfaces(expected_joints, stop_interfaces);

  if (local_start.empty() && local_stop.empty()) {
    return hardware_interface::return_type::OK;
  }

  if (device_ && !local_start.empty()) {
    bool is_position_control = true;
    for (const auto& iface : local_start) {
      if (iface.find("effort") != std::string::npos && iface.find("position") == std::string::npos) {
        is_position_control = false;
        break;
      }
    }
    device_->set_control_mode(is_position_control);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AllegroHandV6HardwareInterface::read(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (!device_) return hardware_interface::return_type::OK;

  StopWatch::instance().clear();
  StopWatch::instance().append("read_start", std::chrono::high_resolution_clock::now()); 

  // Single FC03 transaction for all fast real-time states
  const bool read_ok = device_->read_states(
      joint_position_state_,
      joint_velocity_state_,
      joint_torque_state_,
      imu_quat_state_,
      pressure_state_);

  if (csv_logger_) {
    CsvSample sample;
    sample.unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    sample.read_ok = read_ok;
    if (read_ok) {
      std::copy_n(joint_position_state_, JOINT_NUM, sample.positions.begin());
      for (int i = 0; i < JOINT_NUM; ++i) {
        sample.currents_ma[i] = static_cast<int16_t>(std::lround(joint_torque_state_[i] / v6_sdk::TORQUE_CONSTANT));
      }
      std::copy_n(pressure_state_, PRESSURE_NUM, sample.pressures.begin());
    }
    csv_logger_->push(sample);
  }

  StopWatch::instance().append("read_states", std::chrono::high_resolution_clock::now());

  // Publish real-time IMU message
  if (imu_publisher_ && imu_publisher_->trylock()) {
    imu_publisher_->msg_.header.stamp = time;
    imu_publisher_->msg_.header.frame_id = "allegro_hand_imu";
    imu_publisher_->msg_.orientation.w = imu_quat_state_[0];
    imu_publisher_->msg_.orientation.x = imu_quat_state_[1];
    imu_publisher_->msg_.orientation.y = imu_quat_state_[2];
    imu_publisher_->msg_.orientation.z = imu_quat_state_[3];
    imu_publisher_->unlockAndPublish();
  }

  StopWatch::instance().append("pub_imu", std::chrono::high_resolution_clock::now());

  // Publish real-time tactile pressures message (18 sensors)
  if (pressure_publisher_ && pressure_publisher_->trylock()) {
    for (int i = 0; i < PRESSURE_NUM; ++i) {
      pressure_publisher_->msg_.data[i] = pressure_state_[i];
    }
    pressure_publisher_->unlockAndPublish();
  }

  StopWatch::instance().append("pub_pressure", std::chrono::high_resolution_clock::now());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AllegroHandV6HardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!device_) return hardware_interface::return_type::OK;

  StopWatch::instance().append("write_start", std::chrono::high_resolution_clock::now()); 

  device_->write_commands(joint_position_command_, joint_velocity_command_, joint_torque_command_);

  StopWatch::instance().append("complete", std::chrono::high_resolution_clock::now()); 

  // StopWatch::instance().print(); 

  return hardware_interface::return_type::OK;
}

} // namespace allegro_hand_v6_hardware

PLUGINLIB_EXPORT_CLASS(
    allegro_hand_v6_hardware::AllegroHandV6HardwareInterface,
    hardware_interface::SystemInterface)
