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

#include "allegro_hand_position_effort_controller/position_effort_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace allegro_hand_position_effort_controller {
PositionEffortController::PositionEffortController()
    : controller_interface::ControllerInterface(), rt_command_ptr_(nullptr), joints_command_subscriber_(nullptr), rt_effort_ptr_(nullptr),
      joints_effort_subscriber_(nullptr) {}

controller_interface::CallbackReturn PositionEffortController::on_init() {
  spdlog::set_level(spdlog::level::trace);

  try {
    declare_parameters();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionEffortController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Command interfaces
  command_interface_types_.clear();
  for (const auto& joint : params_.joints) {
    command_interface_types_.push_back(joint + "/position");
    command_interface_types_.push_back(joint + "/effort");
  }

  // State interfaces
  state_interface_types_.clear();
  for (const auto& joint : params_.joints) {
    state_interface_types_.push_back(joint + "/position");
    state_interface_types_.push_back(joint + "/velocity");
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(), [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  joints_effort_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/efforts", rclcpp::SystemDefaultsQoS(), [this](const CmdType::SharedPtr msg) { rt_effort_ptr_.writeFromNonRT(msg); });

  {
    auto initial_effort_msg = std::make_shared<CmdType>();
    initial_effort_msg->data.resize(params_.initial_efforts.size());
    for (size_t i = 0; i < params_.initial_efforts.size(); ++i) {
      initial_effort_msg->data[i] = params_.initial_efforts[i];
    }
    rt_effort_ptr_.writeFromNonRT(initial_effort_msg);
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PositionEffortController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration PositionEffortController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  return state_interfaces_config;
}

controller_interface::CallbackReturn PositionEffortController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
      command_interface_types_.size() != ordered_interfaces.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu", command_interface_types_.size(),
                 ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Store references to state interfaces
  for (auto& interface : state_interfaces_) {
    for (const auto& joint : params_.joints) {
      if (interface.get_name() == joint + "/position") {
        joint_position_states_.emplace_back(interface);
      }
      if (interface.get_name() == joint + "/velocity") {
        joint_velocity_states_.emplace_back(interface);
      }
    }
  }

  if (joint_position_states_.size() != params_.joints.size() || joint_velocity_states_.size() != params_.joints.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Not all state interfaces were found! Position interfaces: %zu, Velocity interfaces: %zu, Expected: %zu",
                 joint_position_states_.size(), joint_velocity_states_.size(), params_.joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  rt_effort_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionEffortController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  rt_effort_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PositionEffortController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // position command
  auto joint_commands = rt_command_ptr_.readFromRT();

  // torque command
  auto effort_commands = rt_effort_ptr_.readFromRT();

  // no position command received yet
  if (!joint_commands || !(*joint_commands)) {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != params_.joints.size()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                          "command size (%zu) does not match number of joints (%zu)", (*joint_commands)->data.size(),
                          params_.joints.size());
    return controller_interface::return_type::ERROR;
  }

  // command_interfaces_ : [pos_joint0, eff_joint0, pos_joint1, eff_joint1, ...]
  for (size_t index = 0; index < params_.joints.size(); index++) {
    double position_command = 0.0;
    if (joint_commands && *joint_commands) {
      position_command = (*joint_commands)->data[index];
    }

    double effort_command = 0.0;
    if (effort_commands && *effort_commands) {
      effort_command = (*effort_commands)->data[index];
    } else {
      constexpr double _100_mA = 0.115;
      effort_command = _100_mA * 3;
    }

    (void)command_interfaces_[index * 2].set_value(position_command);   // Position
    (void)command_interfaces_[index * 2 + 1].set_value(effort_command); // Effort
  }

  return controller_interface::return_type::OK;
}

void PositionEffortController::declare_parameters() { param_listener_ = std::make_shared<ParamListener>(get_node()); }

controller_interface::CallbackReturn PositionEffortController::read_parameters() {
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.initial_efforts.empty()) {
    constexpr double initial_effort = 0.1;
    for (size_t i = 0; i < params_.joints.size(); i++) {
      params_.initial_efforts.push_back(initial_effort);
    }
  } else {
    if (params_.joints.size() != params_.initial_efforts.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'initial_efforts' parameter size (%zu) does not match 'joints' parameter size (%zu)",
                   params_.initial_efforts.size(), params_.joints.size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace allegro_hand_position_effort_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(allegro_hand_position_effort_controller::PositionEffortController, controller_interface::ControllerInterface)
