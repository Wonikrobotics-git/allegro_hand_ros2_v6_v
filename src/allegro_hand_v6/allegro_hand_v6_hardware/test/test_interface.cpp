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
 * @file test_interface.cpp
 * @brief GTest unit test for AllegroHandV6HardwareInterface ros2_control plugin.
 * @author ('c')void
 */

#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

class AllegroHandV6InterfaceTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override {
    // Keep rclcpp active across tests
  }

  hardware_interface::HardwareInfo create_mock_hardware_info() {
    hardware_interface::HardwareInfo info;
    info.name = "allegro_hand";
    info.type = "system";

    info.hardware_parameters["io_interface_descriptor"] = "modbus_tcp:192.168.1.100:502";
    info.hardware_parameters["hand_id"] = "1";
    info.hardware_parameters["device_prefix"] = "";

    const std::vector<std::string> joint_names = {
        "joint00", "joint01", "joint02", "joint03", // Thumb
        "joint10", "joint11", "joint12", "joint13", // Index
        "joint20", "joint21", "joint22", "joint23", // Middle
        "joint30", "joint31", "joint32", "joint33", // Ring
        "joint40", "joint41", "joint42", "joint43", // Pinky
    };

    for (const auto& joint_name : joint_names) {
      hardware_interface::ComponentInfo joint;
      joint.name = joint_name;

      hardware_interface::InterfaceInfo cmd_pos;
      cmd_pos.name = hardware_interface::HW_IF_POSITION;
      cmd_pos.initial_value = "0.0";
      joint.command_interfaces.push_back(cmd_pos);

      hardware_interface::InterfaceInfo cmd_eff;
      cmd_eff.name = hardware_interface::HW_IF_EFFORT;
      cmd_eff.initial_value = "0.0";
      joint.command_interfaces.push_back(cmd_eff);

      hardware_interface::InterfaceInfo st_pos;
      st_pos.name = hardware_interface::HW_IF_POSITION;
      st_pos.initial_value = "0.0";
      joint.state_interfaces.push_back(st_pos);

      hardware_interface::InterfaceInfo st_vel;
      st_vel.name = hardware_interface::HW_IF_VELOCITY;
      joint.state_interfaces.push_back(st_vel);

      hardware_interface::InterfaceInfo st_eff;
      st_eff.name = hardware_interface::HW_IF_EFFORT;
      joint.state_interfaces.push_back(st_eff);

      hardware_interface::InterfaceInfo st_temp;
      st_temp.name = "temperature";
      joint.state_interfaces.push_back(st_temp);

      info.joints.push_back(joint);
    }

    // Add IMU Sensor Info
    hardware_interface::ComponentInfo imu_sensor;
    imu_sensor.name = "allegro_hand_imu";
    imu_sensor.type = "sensor";

    hardware_interface::InterfaceInfo q_w;
    q_w.name = "orientation.w";
    hardware_interface::InterfaceInfo q_x;
    q_x.name = "orientation.x";
    hardware_interface::InterfaceInfo q_y;
    q_y.name = "orientation.y";
    hardware_interface::InterfaceInfo q_z;
    q_z.name = "orientation.z";
    imu_sensor.state_interfaces.push_back(q_w);
    imu_sensor.state_interfaces.push_back(q_x);
    imu_sensor.state_interfaces.push_back(q_y);
    imu_sensor.state_interfaces.push_back(q_z);
    info.sensors.push_back(imu_sensor);

    return info;
  }
};

TEST_F(AllegroHandV6InterfaceTest, PluginLoadAndLifecycle) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");

  std::shared_ptr<hardware_interface::SystemInterface> hardware;
  ASSERT_NO_THROW({
    hardware = loader.createSharedInstance("allegro_hand_v6_hardware/AllegroHandV6HardwareInterface");
  });
  ASSERT_NE(hardware, nullptr);

  auto info = create_mock_hardware_info();
  hardware_interface::HardwareComponentInterfaceParams params{};
  params.hardware_info = info;

  // 1. on_init
  auto ret_init = hardware->on_init(params);
  EXPECT_EQ(ret_init, hardware_interface::CallbackReturn::SUCCESS);

  // 2. on_configure
  auto ret_cfg = hardware->on_configure(rclcpp_lifecycle::State());
  EXPECT_EQ(ret_cfg, hardware_interface::CallbackReturn::SUCCESS);

  // 3. on_activate
  auto ret_act = hardware->on_activate(rclcpp_lifecycle::State());
  EXPECT_EQ(ret_act, hardware_interface::CallbackReturn::SUCCESS);

  // 4. Export State & Command Interfaces
  auto state_interfaces = hardware->export_state_interfaces();
  auto command_interfaces = hardware->export_command_interfaces();

  // 20 joints * 4 states (80) + 4 IMU + 18 tactile = 102 state interfaces
  EXPECT_GE(state_interfaces.size(), 80u);
  EXPECT_EQ(command_interfaces.size(), 40u); // 20 joints * 2 commands (pos, effort)

  // 5. Mode Switch
  std::vector<std::string> start_interfaces;
  for (const auto& joint : info.joints) {
    start_interfaces.push_back(joint.name + "/position");
  }
  EXPECT_EQ(hardware->prepare_command_mode_switch(start_interfaces, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware->perform_command_mode_switch(start_interfaces, {}), hardware_interface::return_type::OK);

  // 6. Read / Write Cycles
  rclcpp::Time now = rclcpp::Clock().now();
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);

  for (int iter = 0; iter < 10; ++iter) {
    EXPECT_EQ(hardware->read(now, period), hardware_interface::return_type::OK);
    EXPECT_EQ(hardware->write(now, period), hardware_interface::return_type::OK);
  }

  // 7. on_deactivate
  auto ret_deact = hardware->on_deactivate(rclcpp_lifecycle::State());
  EXPECT_EQ(ret_deact, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(AllegroHandV6InterfaceTest, PluginAliasAllegroHandV6System) {
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");

  std::shared_ptr<hardware_interface::SystemInterface> hardware;
  ASSERT_NO_THROW({
    hardware = loader.createSharedInstance("allegro_hand_v6_hardware/AllegroHandV6System");
  });
  ASSERT_NE(hardware, nullptr);

  auto info = create_mock_hardware_info();
  hardware_interface::HardwareComponentInterfaceParams params{};
  params.hardware_info = info;
  ASSERT_EQ(hardware->on_init(params), hardware_interface::CallbackReturn::SUCCESS);

  // Verify plugin initialization and interface export without connecting to a device.
  auto state_interfaces = hardware->export_state_interfaces();
  auto command_interfaces = hardware->export_command_interfaces();
  ASSERT_EQ(state_interfaces.size(), 102u);
  ASSERT_EQ(command_interfaces.size(), 40u);
  for (size_t i = 0; i < info.joints.size(); ++i) {
    EXPECT_EQ(state_interfaces[i * 4].get_name(), info.joints[i].name + "/position");
    EXPECT_EQ(command_interfaces[i * 2].get_name(), info.joints[i].name + "/position");
    EXPECT_EQ(command_interfaces[i * 2 + 1].get_name(), info.joints[i].name + "/effort");
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
