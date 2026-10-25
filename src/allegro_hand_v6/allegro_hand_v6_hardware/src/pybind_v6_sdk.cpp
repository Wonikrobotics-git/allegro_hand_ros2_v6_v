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
 * @file pybind_v6_sdk.cpp
 * @brief pybind11 Python bindings for Allegro Hand V6 SDK.
 * @author ('c')void
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/chrono.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

#include "v6_sdk.hpp"
#include "v6_spec.hpp"

namespace py = pybind11;
using namespace v6_sdk;
using allegro_hand_utility::SignalRateMonitor;

PYBIND11_MODULE(allegro_hand_v6_sdk, m) {
  m.doc() = "Python bindings for Allegro Hand V6 SDK (Wonik Robotics)";

  // Constants
  m.attr("JOINT_COUNT") = JOINT_COUNT;
  m.attr("PRESSURE_COUNT") = PRESSURE_COUNT;
  m.attr("FINGER_COUNT") = FINGER_COUNT;
  m.attr("JOINTS_PER_FINGER") = JOINTS_PER_FINGER;
  m.attr("TORQUE_CONSTANT") = TORQUE_CONSTANT;

  // Enums
  py::enum_<HandType>(m, "HandType")
      .value("LEFT_HAND", HandType::LEFT_HAND)
      .value("RIGHT_HAND", HandType::RIGHT_HAND)
      .export_values();

  py::enum_<OpMode>(m, "OpMode")
      .value("CURRENT_CONTROL", OpMode::CURRENT_CONTROL)
      .value("POSITION_CONTROL", OpMode::POSITION_CONTROL)
      .export_values();

  py::enum_<FingerIndex>(m, "FingerIndex")
      .value("FINGER_THUMB", FingerIndex::FINGER_THUMB)
      .value("FINGER_INDEX", FingerIndex::FINGER_INDEX)
      .value("FINGER_MIDDLE", FingerIndex::FINGER_MIDDLE)
      .value("FINGER_RING", FingerIndex::FINGER_RING)
      .value("FINGER_PINKY", FingerIndex::FINGER_PINKY)
      .export_values();

  py::enum_<JointIndex>(m, "JointIndex")
      .value("JOINT_THUMB_0", JointIndex::JOINT_THUMB_0)
      .value("JOINT_THUMB_1", JointIndex::JOINT_THUMB_1)
      .value("JOINT_THUMB_2", JointIndex::JOINT_THUMB_2)
      .value("JOINT_THUMB_3", JointIndex::JOINT_THUMB_3)
      .value("JOINT_INDEX_0", JointIndex::JOINT_INDEX_0)
      .value("JOINT_INDEX_1", JointIndex::JOINT_INDEX_1)
      .value("JOINT_INDEX_2", JointIndex::JOINT_INDEX_2)
      .value("JOINT_INDEX_3", JointIndex::JOINT_INDEX_3)
      .value("JOINT_MIDDLE_0", JointIndex::JOINT_MIDDLE_0)
      .value("JOINT_MIDDLE_1", JointIndex::JOINT_MIDDLE_1)
      .value("JOINT_MIDDLE_2", JointIndex::JOINT_MIDDLE_2)
      .value("JOINT_MIDDLE_3", JointIndex::JOINT_MIDDLE_3)
      .value("JOINT_RING_0", JointIndex::JOINT_RING_0)
      .value("JOINT_RING_1", JointIndex::JOINT_RING_1)
      .value("JOINT_RING_2", JointIndex::JOINT_RING_2)
      .value("JOINT_RING_3", JointIndex::JOINT_RING_3)
      .value("JOINT_PINKY_0", JointIndex::JOINT_PINKY_0)
      .value("JOINT_PINKY_1", JointIndex::JOINT_PINKY_1)
      .value("JOINT_PINKY_2", JointIndex::JOINT_PINKY_2)
      .value("JOINT_PINKY_3", JointIndex::JOINT_PINKY_3)
      .value("JOINT_COUNT", JointIndex::JOINT_COUNT)
      .export_values();

  py::enum_<PressureIndex>(m, "PressureIndex")
      .value("PRESSURE_THUMB_0", PressureIndex::PRESSURE_THUMB_0)
      .value("PRESSURE_THUMB_1", PressureIndex::PRESSURE_THUMB_1)
      .value("PRESSURE_THUMB_2", PressureIndex::PRESSURE_THUMB_2)
      .value("PRESSURE_INDEX_0", PressureIndex::PRESSURE_INDEX_0)
      .value("PRESSURE_INDEX_1", PressureIndex::PRESSURE_INDEX_1)
      .value("PRESSURE_INDEX_2", PressureIndex::PRESSURE_INDEX_2)
      .value("PRESSURE_MIDDLE_0", PressureIndex::PRESSURE_MIDDLE_0)
      .value("PRESSURE_MIDDLE_1", PressureIndex::PRESSURE_MIDDLE_1)
      .value("PRESSURE_MIDDLE_2", PressureIndex::PRESSURE_MIDDLE_2)
      .value("PRESSURE_RING_0", PressureIndex::PRESSURE_RING_0)
      .value("PRESSURE_RING_1", PressureIndex::PRESSURE_RING_1)
      .value("PRESSURE_RING_2", PressureIndex::PRESSURE_RING_2)
      .value("PRESSURE_PINKY_0", PressureIndex::PRESSURE_PINKY_0)
      .value("PRESSURE_PINKY_1", PressureIndex::PRESSURE_PINKY_1)
      .value("PRESSURE_PINKY_2", PressureIndex::PRESSURE_PINKY_2)
      .value("PRESSURE_PALM_0", PressureIndex::PRESSURE_PALM_0)
      .value("PRESSURE_PALM_1", PressureIndex::PRESSURE_PALM_1)
      .value("PRESSURE_PALM_2", PressureIndex::PRESSURE_PALM_2)
      .value("PRESSURE_COUNT", PressureIndex::PRESSURE_COUNT)
      .export_values();

  py::enum_<ImuProfile>(m, "ImuProfile")
      .value("RAW_ACCEL", ImuProfile::RAW_ACCEL)
      .value("RAW_MAG", ImuProfile::RAW_MAG)
      .value("RAW_GYRO", ImuProfile::RAW_GYRO)
      .value("IMU_FUSION", ImuProfile::IMU_FUSION)
      .value("COMPASS", ImuProfile::COMPASS)
      .value("M4G", ImuProfile::M4G)
      .value("NDOF_FMC_OFF", ImuProfile::NDOF_FMC_OFF)
      .value("NDOF", ImuProfile::NDOF)
      .export_values();

  // Structures
  py::class_<DeviceInfo>(m, "DeviceInfo")
      .def(py::init<>())
      .def_readwrite("firmware_version", &DeviceInfo::firmware_version)
      .def_readwrite("hand_type", &DeviceInfo::hand_type)
      .def_readwrite("serial_number", &DeviceInfo::serial_number)
      .def("__repr__", [](const DeviceInfo& d) -> std::string {
        return fmt::format("<DeviceInfo FW:0x{:04X}, Type:{}, SN:{}>",
                           d.firmware_version,
                           (d.hand_type == HandType::RIGHT_HAND ? "RIGHT" : "LEFT"),
                           d.serial_number);
      });

  py::class_<JointStatusBitmask>(m, "JointStatusBitmask")
      .def(py::init<>())
      .def_readwrite("raw_value", &JointStatusBitmask::raw_value)
      .def("overload", &JointStatusBitmask::overload)
      .def("electrical_shock", &JointStatusBitmask::electrical_shock)
      .def("overheat", &JointStatusBitmask::overheat)
      .def("input_voltage", &JointStatusBitmask::input_voltage)
      .def("finger_comm_error", &JointStatusBitmask::finger_comm_error)
      .def("finger_fault", &JointStatusBitmask::finger_fault)
      .def("has_error", &JointStatusBitmask::has_error)
      .def("__repr__", [](const JointStatusBitmask& b) -> std::string {
        return fmt::format("<JointStatusBitmask 0x{:04X}, has_error={}>", b.raw_value, b.has_error());
      });

  py::class_<ImuCalibStatus>(m, "ImuCalibStatus")
      .def(py::init<>())
      .def_readwrite("raw_value", &ImuCalibStatus::raw_value)
      .def("mag_calib", &ImuCalibStatus::mag_calib)
      .def("acc_calib", &ImuCalibStatus::acc_calib)
      .def("gyr_calib", &ImuCalibStatus::gyr_calib)
      .def("sys_calib", &ImuCalibStatus::sys_calib)
      .def("is_fully_calibrated", &ImuCalibStatus::is_fully_calibrated)
      .def("__repr__", [](const ImuCalibStatus& c) -> std::string {
        return fmt::format("<ImuCalibStatus SYS:{} GYR:{} ACC:{} MAG:{}>",
                           c.sys_calib(), c.gyr_calib(), c.acc_calib(), c.mag_calib());
      });

  py::class_<HandDiagnostics>(m, "HandDiagnostics")
      .def(py::init<>())
      .def_readwrite("torque_enabled", &HandDiagnostics::torque_enabled)
      .def_readwrite("calibration_running", &HandDiagnostics::calibration_running)
      .def_readwrite("operation_mode", &HandDiagnostics::operation_mode)
      .def_readwrite("joint_status", &HandDiagnostics::joint_status)
      .def_readwrite("imu_calib_status", &HandDiagnostics::imu_calib_status)
      .def_property_readonly("motor_temperatures", [](const HandDiagnostics& d) -> py::array_t<uint16_t> {
        py::array_t<uint16_t> arr(JOINT_COUNT);
        auto r = arr.mutable_unchecked<1>();
        for (int i = 0; i < JOINT_COUNT; ++i) r(i) = d.motor_temperatures[i];
        return arr;
      })
      .def_property_readonly("sensor_temperatures", [](const HandDiagnostics& d) -> py::array_t<uint16_t> {
        py::array_t<uint16_t> arr(PRESSURE_COUNT);
        auto r = arr.mutable_unchecked<1>();
        for (int i = 0; i < PRESSURE_COUNT; ++i) r(i) = d.sensor_temperatures[i];
        return arr;
      })
      .def("__repr__", [](const HandDiagnostics& d) -> std::string {
        return fmt::format("<HandDiagnostics Torque:{}, Calib:{}, Mode:{}>",
                           d.torque_enabled ? "ON" : "OFF",
                           d.calibration_running ? "RUNNING" : "IDLE",
                           d.operation_mode == OpMode::POSITION_CONTROL ? "POSITION" : "CURRENT");
      });

  // SignalRateMonitor & RateInfo
  py::class_<SignalRateMonitor::RateInfo>(m, "RateInfo")
      .def_readonly("count", &SignalRateMonitor::RateInfo::count)
      .def_readonly("frequency", &SignalRateMonitor::RateInfo::frequency)
      .def_readonly("jitter", &SignalRateMonitor::RateInfo::jitter)
      .def_readonly("rtt_count", &SignalRateMonitor::RateInfo::rtt_count)
      .def_readonly("rtt_mean", &SignalRateMonitor::RateInfo::rtt_mean)
      .def_readonly("rtt_jitter", &SignalRateMonitor::RateInfo::rtt_jitter)
      .def("__repr__", [](const SignalRateMonitor::RateInfo& r) -> std::string {
        return fmt::format("<RateInfo freq={:.2f}Hz, jitter={:.3f}ms, rtt={:.3f}ms>",
                           r.frequency, r.jitter * 1000.0, r.rtt_mean * 1000.0);
      });

  py::class_<SignalRateMonitor>(m, "SignalRateMonitor")
      .def("get_info", &SignalRateMonitor::get_info)
      .def("to_string", &SignalRateMonitor::to_string)
      .def("__repr__", [](const SignalRateMonitor& s) -> std::string {
        return s.to_string();
      });

  // HandV6ModBus::Options
  py::class_<HandV6ModBus::Options>(m, "Options")
      .def(py::init<>())
      .def_readwrite("comm_type", &HandV6ModBus::Options::comm_type)
      .def_readwrite("ip_address", &HandV6ModBus::Options::ip_address)
      .def_readwrite("tcp_port", &HandV6ModBus::Options::tcp_port)
      .def_readwrite("serial_device", &HandV6ModBus::Options::serial_device)
      .def_readwrite("baudrate", &HandV6ModBus::Options::baudrate)
      .def_readwrite("slave_id", &HandV6ModBus::Options::slave_id)
      .def_readwrite("k_p_table", &HandV6ModBus::Options::k_p_table)
      .def_readwrite("k_i_table", &HandV6ModBus::Options::k_i_table)
      .def_readwrite("k_d_table", &HandV6ModBus::Options::k_d_table)
      .def_readwrite("torque_limit_table", &HandV6ModBus::Options::torque_limit_table)
      .def("__repr__", [](const HandV6ModBus::Options& opt) -> std::string {
        if (opt.comm_type == "rtu" || opt.comm_type == "serial") {
          return fmt::format("<Options RTU dev:{}, baud:{}, slave:{}>",
                             opt.serial_device, opt.baudrate, opt.slave_id);
        }
        return fmt::format("<Options TCP {}:{}, slave:{}>",
                           opt.ip_address, opt.tcp_port, opt.slave_id);
      });

  // HandV6ModBus Class
  py::class_<HandV6ModBus>(m, "HandV6ModBus")
      .def(py::init<const HandV6ModBus::Options&>(), py::arg("options"))
      .def("is_ready", &HandV6ModBus::is_ready)
      .def("get_device_info", &HandV6ModBus::get_device_info, py::return_value_policy::reference_internal)
      .def("get_rate_monitor", &HandV6ModBus::get_rate_monitor, py::return_value_policy::reference_internal)
      .def("set_control_mode", &HandV6ModBus::set_control_mode, py::arg("position_control_mode"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_gains", &HandV6ModBus::read_gains, py::arg("group_name"),
           py::call_guard<py::gil_scoped_release>())
      .def("write_gains", &HandV6ModBus::write_gains, py::arg("group_name"), py::arg("gains"),
           py::call_guard<py::gil_scoped_release>())
      .def("read_diagnostics", [](HandV6ModBus& self) -> HandDiagnostics {
        HandDiagnostics diag;
        bool ok = false;
        {
          py::gil_scoped_release release;
          ok = self.read_diagnostics(diag);
        }
        if (!ok) {
          throw std::runtime_error("Failed to read diagnostics from Allegro Hand V6");
        }
        return diag;
      })
      .def("read_states", [](HandV6ModBus& self) -> py::tuple {
        py::array_t<double> py_pos(JOINT_COUNT);
        py::array_t<double> py_vel(JOINT_COUNT);
        py::array_t<double> py_tau(JOINT_COUNT);
        py::array_t<double> py_imu(4);
        py::array_t<double> py_pressure(PRESSURE_COUNT);

        {
          py::gil_scoped_release release;
          self.read_states(
              py_pos.mutable_data(),
              py_vel.mutable_data(),
              py_tau.mutable_data(),
              py_imu.mutable_data(),
              py_pressure.mutable_data());
        }

        return py::make_tuple(py_pos, py_vel, py_tau, py_imu, py_pressure);
      }, "Reads all real-time states. Returns tuple: (positions, velocities, torques, imu_quaternion, tactile_pressures)")
      .def("write_commands", [](HandV6ModBus& self,
                                std::optional<py::array_t<double, py::array::c_style | py::array::forcecast>> position = std::nullopt,
                                std::optional<py::array_t<double, py::array::c_style | py::array::forcecast>> velocity = std::nullopt,
                                std::optional<py::array_t<double, py::array::c_style | py::array::forcecast>> torque = std::nullopt) -> void {
        const double* p_pos = nullptr;
        const double* p_vel = nullptr;
        const double* p_tau = nullptr;

        if (position.has_value()) {
          if (position->size() != JOINT_COUNT) {
            throw std::invalid_argument(fmt::format("Position array must have size {}, got {}", JOINT_COUNT, position->size()));
          }
          p_pos = position->data();
        }

        if (velocity.has_value()) {
          if (velocity->size() != JOINT_COUNT) {
            throw std::invalid_argument(fmt::format("Velocity array must have size {}, got {}", JOINT_COUNT, velocity->size()));
          }
          p_vel = velocity->data();
        }

        if (torque.has_value()) {
          if (torque->size() != JOINT_COUNT) {
            throw std::invalid_argument(fmt::format("Torque array must have size {}, got {}", JOINT_COUNT, torque->size()));
          }
          p_tau = torque->data();
        }

        {
          py::gil_scoped_release release;
          self.write_commands(p_pos, p_vel, p_tau);
        }
      }, py::arg("position") = py::none(), py::arg("velocity") = py::none(), py::arg("torque") = py::none(),
      "Writes target commands to the hand. position, velocity, and torque are optional arrays of size 20.")
      .def("set_torque_enabled", &HandV6ModBus::set_torque_enabled, py::arg("enabled"),
           py::call_guard<py::gil_scoped_release>(), "Turns servo torque ON or OFF.")
      .def("trigger_calibration", &HandV6ModBus::trigger_calibration,
           py::call_guard<py::gil_scoped_release>(), "Triggers joint zero calibration.")
      .def("set_imu_profile", &HandV6ModBus::set_imu_profile, py::arg("profile"),
           py::call_guard<py::gil_scoped_release>(), "Sets BNO055 IMU operation profile.")
      .def("save_grasp_pose", &HandV6ModBus::save_grasp_pose, py::arg("slot_id"),
           py::call_guard<py::gil_scoped_release>(), "Saves current joint pose to on-device flash slot (1~5).")
      .def("play_grasp_pose", &HandV6ModBus::play_grasp_pose, py::arg("slot_id"),
           py::call_guard<py::gil_scoped_release>(), "Plays stored grasp pose from on-device flash slot (1~5).");
}
