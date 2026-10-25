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
 * @file hardware_interface.hpp
 * @brief Defines the ros2_control hardware interface for Allegro Hand V6.
 * @author Wonik Robotics
 */

#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <allegro_hand_utility/ros_utility.hpp>
#include <allegro_hand_v6_hardware/visibility_control.h>

#include "v6_sdk.hpp"

namespace allegro_hand_v6_hardware {

class Diagnostics;
class ParameterHandler;
class CsvLogger;

/**
 * @class AllegroHandV6HardwareInterface
 * @brief ros2_control SystemInterface implementation for Allegro Hand V6.
 */
class AllegroHandV6HardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AllegroHandV6HardwareInterface)

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ALLEGRO_HAND_V6_HW_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  static constexpr int JOINT_NUM = v6_sdk::JOINT_COUNT;
  static constexpr int PRESSURE_NUM = v6_sdk::PRESSURE_COUNT;

  std::shared_ptr<allegro_hand_utility::HardwareInfoParser> hwinfo_parser_;

  // Dedicated auxiliary ROS 2 node for parameters, diagnostics, and sensor topic publishing
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<v6_sdk::HandV6> device_;
  std::shared_ptr<Diagnostics> diagnostics_handler_;
  std::shared_ptr<ParameterHandler> param_handler_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  bool csv_logging_enabled_{false};
  std::string csv_directory_{"./data_logs"};
  std::shared_ptr<CsvLogger> csv_logger_;
  rclcpp::TimerBase::SharedPtr csv_status_timer_;

  // Real-time sensor topic publishers
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> imu_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pressure_publisher_;

  // Joint Command Buffers
  double joint_position_command_[JOINT_NUM];
  double joint_velocity_command_[JOINT_NUM];
  double joint_torque_command_[JOINT_NUM];

  // Joint State Buffers
  double joint_position_state_[JOINT_NUM];
  double joint_velocity_state_[JOINT_NUM];
  double joint_torque_state_[JOINT_NUM];
  double joint_temperature_state_[JOINT_NUM];

  // Sensor State Buffers
  double imu_quat_state_[4];
  double pressure_state_[PRESSURE_NUM];
};

// Legacy compatibility alias
using AllegroHandPlexusHardwareInterface = AllegroHandV6HardwareInterface;

} // namespace allegro_hand_v6_hardware
