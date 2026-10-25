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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "allegro_hand_position_effort_controller/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "allegro_hand_position_effort_controller/position_effort_controller_parameters.hpp"

namespace allegro_hand_position_effort_controller {
using CmdType = std_msgs::msg::Float64MultiArray;

class PositionEffortController : public controller_interface::ControllerInterface {
public:
  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  PositionEffortController();

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  ~PositionEffortController() = default;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  AH_POSITION_EFFORT_CONTROLLER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();

  using Params = aph_grasping_controller::Params;
  using ParamListener = aph_grasping_controller::ParamListener;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_states_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_states_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_effort_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_effort_subscriber_;
};

} // namespace allegro_hand_position_effort_controller
