// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause

#include "allegro_hand_v6_dashboard/sensor_layout.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace allegro_hand_v6_dashboard
{
namespace
{

template<std::size_t N>
std::array<double, N> read_array(const YAML::Node & node, const char * field)
{
  const auto value = node[field];
  if (!value || !value.IsSequence() || value.size() != N) {
    throw std::runtime_error(std::string("sensor field '") + field + "' must contain " +
      std::to_string(N) + " numbers");
  }
  std::array<double, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = value[i].as<double>();
    if (!std::isfinite(result[i])) {
      throw std::runtime_error(std::string("sensor field '") + field + "' contains a non-finite value");
    }
  }
  return result;
}

}  // namespace

FingerSensorMode classify_finger(const std::array<double, 3> & values)
{
  if (!std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v) && v >= 0;})) {
    return FingerSensorMode::Unknown;
  }
  return std::any_of(values.begin(), values.end(), [](double v) {return v > 0;}) ?
    FingerSensorMode::Pressure : FingerSensorMode::Vision;
}

std::size_t SensorLayout::channel_count() const
{
  std::size_t result = 0;
  for (const auto & sensor : sensors) {
    result = std::max(result, sensor.index + 1);
  }
  return result;
}

void SensorLayout::resolve_robot_frames(
  const std::set<std::string> & links, const std::string & hand)
{
  // Current CAD sensor frames follow the contact surfaces; legacy base_link
  // offsets and R*_Link axes belong to a different robot coordinate system.
  for (auto & sensor : sensors) {
    if (sensor.frame_id != "base_link") {continue;}
    const std::string frame = sensor.name == "palm_center" ? "palm_s1" :
      sensor.name == "palm_left" ? "palm_s2" : sensor.name == "palm_bottom" ? "palm_s3" : "";
    if (links.count(frame)) {
      sensor.frame_id = frame;
      sensor.position = {0.0, -0.003, 0.0};
      sensor.normal = {0.0, -1.0, 0.0};
    }
  }
  const std::array<std::string, 5> fingers{"thumb", "index", "middle", "ring", "pinky"};
  const double mirror = hand == "left" ? 1.0 : -1.0;
  for (auto & placement : vision_sensors) {
    const auto finger = std::find(fingers.begin(), fingers.end(), placement.finger);
    if (finger == fingers.end()) {continue;}
    const auto legacy_prefix = "R" + std::to_string(std::distance(fingers.begin(), finger) + 1);
    for (std::size_t i = 0; i < placement.pressure_markers.size(); ++i) {
      auto & marker = placement.pressure_markers[i];
      if (links.count(marker.frame_id)) {continue;}
      if (marker.frame_id.rfind(legacy_prefix, 0) != 0) {continue;}
      const bool force_model = links.count(placement.finger + "_s3") != 0;
      const bool vision_model = links.count(placement.finger + "_4") != 0 &&
        links.count(placement.finger + "_s1") != 0;
      if ((force_model || vision_model) && i < 3) {
        std::string frame;
        std::array<double, 3> position;
        std::array<double, 3> normal{0.0, -1.0, 0.0};
        if (force_model) {
          frame = placement.finger + "_s" + std::to_string(3 - i);
          position = i == 0 ? std::array<double, 3>{0.0, 0.0, 0.007} :
            std::array<double, 3>{0.0, -0.002, 0.0};
          if (i == 0) {normal = {0.0, 0.0, 1.0};}
        } else if (i == 0) {
          frame = placement.finger + "_s1";
          position = {0.0, -0.0035, 0.0};
        } else {
          // V has only a fingertip sensor frame. Place the remaining pressure
          // channels on the front faces of the corresponding moving phalanges.
          const bool thumb = placement.finger == "thumb";
          frame = placement.finger + "_" + std::to_string((thumb ? 4 : 3) - i);
          position = thumb ? std::array<double, 3>{mirror * 0.0145, -0.016, -0.012} :
            std::array<double, 3>{mirror * 0.0205, -0.0135, 0.012};
        }
        if (links.count(frame)) {
          marker.frame_id = frame;
          marker.position = position;
          marker.normal = normal;
          continue;
        }
      }
      for (const auto segment : {1, 2, 3, 4}) {
        if (marker.frame_id != legacy_prefix + std::to_string(segment) + "_Link") {continue;}
        auto target = placement.finger + "_" + std::to_string(segment);
        // V6 V has a fixed fingertip link _4; V6 F ends at _3.
        if (segment == 4 && !links.count(target)) {target = placement.finger + "_3";}
        if (links.count(target)) {marker.frame_id = target;}
        break;
      }
    }
  }
}

SensorLayout SensorLayout::load(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  SensorLayout result;
  result.input_channel_count = root["input_channel_count"].as<std::size_t>(18);
  result.canvas_width = root["canvas_width"].as<double>(900.0);
  result.canvas_height = root["canvas_height"].as<double>(900.0);
  result.hand_rect = read_array<4>(root, "hand_rect");
  if (!std::isfinite(result.canvas_width) || !std::isfinite(result.canvas_height) ||
    result.canvas_width <= 0 || result.canvas_height <= 0)
  {
    throw std::runtime_error("canvas dimensions must be finite and positive");
  }
  const auto check_rect = [&result](const std::array<double, 4> & rect) {
      if (rect[0] < 0 || rect[1] < 0 || rect[2] <= 0 || rect[3] <= 0 ||
        rect[0] + rect[2] > result.canvas_width ||
        rect[1] + rect[3] > result.canvas_height)
      {
        throw std::runtime_error("hand/card rectangle must be inside canvas");
      }
    };
  check_rect(result.hand_rect);
  result.unit = root["unit"].as<std::string>("hPa");
  result.pressure_min = root["pressure_min"].as<double>(1000.0);
  result.pressure_max = root["pressure_max"].as<double>(1200.0);
  result.svg_width = root["svg_width"].as<double>(600.0);
  result.svg_height = root["svg_height"].as<double>(900.0);
  result.image_crop = root["image_crop"] ?
    read_array<4>(root, "image_crop") :
    std::array<double, 4>{0.0, 0.0, result.svg_width, result.svg_height};
  result.image_scale = root["image_scale"].as<double>(1.0);
  result.image_offset = root["image_offset"] ?
    read_array<2>(root, "image_offset") : std::array<double, 2>{0.0, 0.0};

  if (!std::isfinite(result.pressure_min) || !std::isfinite(result.pressure_max) ||
    result.pressure_max <= result.pressure_min)
  {
    throw std::runtime_error("pressure_max must be greater than pressure_min");
  }
  if (!std::isfinite(result.svg_width) || !std::isfinite(result.svg_height) ||
    result.svg_width <= 0.0 || result.svg_height <= 0.0) {
    throw std::runtime_error("source image dimensions must be positive");
  }
  if (result.image_crop[0] < 0.0 || result.image_crop[1] < 0.0 ||
    result.image_crop[2] <= 0.0 || result.image_crop[3] <= 0.0 ||
    result.image_crop[0] + result.image_crop[2] > result.svg_width ||
    result.image_crop[1] + result.image_crop[3] > result.svg_height)
  {
    throw std::runtime_error("image_crop must be inside the source image");
  }
  if (!std::isfinite(result.image_scale) || result.image_scale <= 0.0) {
    throw std::runtime_error("image_scale must be positive");
  }

  const auto sensors = root["sensors"];
  if (!sensors || !sensors.IsSequence() || sensors.size() == 0) {
    throw std::runtime_error("sensor layout must contain at least one sensor");
  }

  std::set<std::size_t> indices;
  std::set<std::string> names;
  for (const auto & node : sensors) {
    SensorDefinition sensor;
    sensor.index = node["index"].as<std::size_t>();
    sensor.source_index = node["source_index"].as<std::size_t>();
    sensor.name = node["name"].as<std::string>();
    sensor.label = node["label"].as<std::string>(sensor.name);
    sensor.group = node["group"].as<std::string>();
    sensor.frame_id = node["frame_id"].as<std::string>();
    sensor.position = read_array<3>(node, "position");
    sensor.normal = read_array<3>(node, "normal");
    sensor.svg = read_array<2>(node, "svg");
    sensor.text_rotation = node["text_rotation"].as<double>(0.0);

    if (sensor.index >= 256) {
      throw std::runtime_error("sensor index must be less than 256");
    }
    if (sensor.name.empty() || sensor.group.empty() || sensor.frame_id.empty()) {
      throw std::runtime_error("sensor name, group and frame_id must not be empty");
    }
    if (!std::isfinite(sensor.text_rotation)) {
      throw std::runtime_error("sensor text_rotation must be finite: " + sensor.name);
    }
    if (sensor.svg[0] < result.image_crop[0] ||
      sensor.svg[0] > result.image_crop[0] + result.image_crop[2] ||
      sensor.svg[1] < result.image_crop[1] ||
      sensor.svg[1] > result.image_crop[1] + result.image_crop[3])
    {
      throw std::runtime_error("sensor overlay is outside image_crop: " + sensor.name);
    }
    if (!indices.insert(sensor.index).second) {
      throw std::runtime_error("duplicate sensor index: " + std::to_string(sensor.index));
    }
    if (!names.insert(sensor.name).second) {
      throw std::runtime_error("duplicate sensor name: " + sensor.name);
    }
    const double length = std::sqrt(
      sensor.normal[0] * sensor.normal[0] + sensor.normal[1] * sensor.normal[1] +
      sensor.normal[2] * sensor.normal[2]);
    if (length < 1e-9) {
      throw std::runtime_error("sensor normal must be non-zero: " + sensor.name);
    }
    for (double & component : sensor.normal) {
      component /= length;
    }
    result.sensors.push_back(std::move(sensor));
  }

  std::sort(result.sensors.begin(), result.sensors.end(),
    [](const auto & left, const auto & right) {return left.index < right.index;});
  if (result.channel_count() != result.sensors.size()) {
    throw std::runtime_error("sensor indices must be contiguous and start at zero");
  }
  if (result.sensors.size() != 3) {
    throw std::runtime_error("V6 V dashboard expects exactly three palm pressure displays");
  }
  result.validate_input();
  std::set<std::string> fingers;
  const std::set<std::string> expected{"thumb", "index", "middle", "ring", "pinky"};
  if (!root["vision_sensors"] || !root["vision_sensors"].IsSequence()) {
    throw std::runtime_error("vision_sensors must define five finger cards");
  }
  for (const auto & node : root["vision_sensors"]) {
    VisionSensorPlacement vision_sensor;
    vision_sensor.finger = node["finger"].as<std::string>();
    vision_sensor.card = read_array<4>(node, "card");
    vision_sensor.tip = read_array<2>(node, "tip");
    if (node["pressure_start"]) {
      vision_sensor.pressure_start = node["pressure_start"].as<std::size_t>();
      if (vision_sensor.pressure_start > 253) {throw std::runtime_error("invalid finger pressure_start");}
      const auto positions = node["pressure_positions"];
      if (!positions || positions.size() != 3) {throw std::runtime_error("finger needs three pressure positions");}
      for (std::size_t i = 0; i < 3; ++i) {
        YAML::Node point;
        point["svg"] = positions[i];
        vision_sensor.pressure_positions[i] = read_array<2>(point, "svg");
      }
      vision_sensor.pressure_rotation = node["pressure_rotation"].as<double>(0.0);
      if (!std::isfinite(vision_sensor.pressure_rotation)) {throw std::runtime_error("invalid pressure rotation");}
    }
    if (const auto markers = node["pressure_markers"]) {
      if (!markers.IsSequence() || markers.size() != 3 || vision_sensor.pressure_start > 253) {
        throw std::runtime_error("finger requires three pressure markers and pressure_start");
      }
      for (std::size_t i = 0; i < 3; ++i) {
        SensorDefinition sensor;
        sensor.name = vision_sensor.finger + (i == 0 ? "_tip" : (i == 1 ? "_mid" : "_base"));
        sensor.index = 3 + vision_sensor.pressure_start + i;
        sensor.source_index = vision_sensor.pressure_start + i;
        sensor.frame_id = markers[i]["frame_id"].as<std::string>();
        sensor.position = read_array<3>(markers[i], "position");
        sensor.normal = read_array<3>(markers[i], "normal");
        double length = 0;
        for (double component : sensor.normal) {length += component * component;}
        length = std::sqrt(length);
        if (sensor.frame_id.empty() || length < 1e-9) {
          throw std::runtime_error("finger marker requires frame_id and nonzero normal");
        }
        for (double & component : sensor.normal) {component /= length;}
        vision_sensor.pressure_markers.push_back(sensor);
      }
    }
    check_rect(vision_sensor.card);
    if (!expected.count(vision_sensor.finger) || !fingers.insert(vision_sensor.finger).second) {
      throw std::runtime_error("unknown or duplicate VISION SENSOR finger");
    }
    if (vision_sensor.tip[0] < result.image_crop[0] ||
      vision_sensor.tip[0] > result.image_crop[0] + result.image_crop[2] ||
      vision_sensor.tip[1] < result.image_crop[1] ||
      vision_sensor.tip[1] > result.image_crop[1] + result.image_crop[3])
    {
      throw std::runtime_error("VISION SENSOR tip anchor must be inside image crop");
    }
    result.vision_sensors.push_back(vision_sensor);
  }
  if (fingers != expected) {
    throw std::runtime_error("vision_sensors must contain all five fingers");
  }
  return result;
}

void SensorLayout::validate_input() const
{
  if (input_channel_count == 0 || input_channel_count > 256) {
    throw std::runtime_error("pressure_channel_count must be 1..256");
  }
  std::set<std::size_t> indices;
  for (const auto & sensor : sensors) {
    if (sensor.source_index >= input_channel_count || !indices.insert(sensor.source_index).second) {
      throw std::runtime_error("pressure source indices must be unique and inside input array");
    }
  }
}

std::vector<float> SensorLayout::select_pressures(const std::vector<double> & values) const
{
  std::vector<float> result(sensors.size(), std::numeric_limits<float>::quiet_NaN());
  if (values.size() == input_channel_count) {
    for (const auto & sensor : sensors) {
      result.at(sensor.index) = static_cast<float>(values.at(sensor.source_index));
    }
  }
  return result;
}

}  // namespace allegro_hand_v6_dashboard
