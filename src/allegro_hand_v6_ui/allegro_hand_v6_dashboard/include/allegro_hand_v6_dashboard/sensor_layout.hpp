// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__SENSOR_LAYOUT_HPP_
#define ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__SENSOR_LAYOUT_HPP_

#include <array>
#include <cstddef>
#include <set>
#include <string>
#include <vector>

namespace allegro_hand_v6_dashboard
{

struct SensorDefinition
{
  std::size_t index{};
  std::size_t source_index{};
  std::string name;
  std::string label;
  std::string group;
  std::string frame_id;
  std::array<double, 3> position{};
  std::array<double, 3> normal{};
  std::array<double, 2> svg{};
  double text_rotation{0.0};
};

enum class FingerSensorMode {Unknown, Vision, Pressure};
FingerSensorMode classify_finger(const std::array<double, 3> & values);

struct VisionSensorPlacement
{
  std::string finger;
  std::size_t pressure_start{256};
  std::array<std::array<double, 2>, 3> pressure_positions{};
  double pressure_rotation{0.0};
  std::vector<SensorDefinition> pressure_markers;
  std::array<double, 4> card{};  // x, y, width, height in canvas coordinates
  std::array<double, 2> tip{};   // anchor in source PNG coordinates
};

struct SensorLayout
{
  std::size_t input_channel_count{18};
  double canvas_width{900.0};
  double canvas_height{900.0};
  std::array<double, 4> hand_rect{100.0, 240.0, 700.0, 650.0};
  std::vector<VisionSensorPlacement> vision_sensors;
  std::string unit{"hPa"};
  double pressure_min{1000.0};
  double pressure_max{1200.0};
  double svg_width{600.0};
  double svg_height{900.0};
  std::array<double, 4> image_crop{0.0, 0.0, 600.0, 900.0};
  double image_scale{1.0};
  std::array<double, 2> image_offset{0.0, 0.0};
  std::vector<SensorDefinition> sensors;

  std::size_t channel_count() const;
  void validate_input() const;
  void resolve_robot_frames(const std::set<std::string> & links, const std::string & hand);
  std::vector<float> select_pressures(const std::vector<double> & values) const;
  static SensorLayout load(const std::string & path);
};

}  // namespace allegro_hand_v6_dashboard

#endif  // ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__SENSOR_LAYOUT_HPP_
