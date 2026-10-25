// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <QApplication>
#include <QDir>
#include <QMessageBox>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "allegro_hand_v6_dashboard/dashboard_window.hpp"
#include "allegro_hand_v6_dashboard/sensor_layout.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto non_ros_arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<std::vector<char>> argument_storage;
  std::vector<char *> qt_arguments;
  argument_storage.reserve(non_ros_arguments.size());
  qt_arguments.reserve(non_ros_arguments.size() + 1);
  for (const auto & argument : non_ros_arguments) {
    argument_storage.emplace_back(argument.begin(), argument.end());
    argument_storage.back().push_back('\0');
  }
  for (auto & argument : argument_storage) {
    qt_arguments.push_back(argument.data());
  }
  int qt_argc = static_cast<int>(qt_arguments.size());
  qt_arguments.push_back(nullptr);
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, true);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps, true);
  QApplication app(qt_argc, qt_arguments.data());
  QApplication::setOrganizationName("Wonik Robotics");
  QApplication::setApplicationName("V6 Dashboard");
  QApplication::setStyle("Fusion");

  int result = 1;
  try {
    const std::string share = ament_index_cpp::get_package_share_directory(
      "allegro_hand_v6_dashboard");
    auto node_abstraction =
      std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("allegro_hand_v6_dashboard");
    auto node = node_abstraction->get_raw_node();
    const std::string hand = node->declare_parameter<std::string>("hand", "right");
    if (hand != "right" && hand != "left") {
      throw std::runtime_error("hand must be either 'right' or 'left'");
    }
    std::string config_path = node->declare_parameter<std::string>("sensor_config", "");
    if (config_path.empty()) {
      config_path = share + "/config/tactile_layout" +
        (hand == "left" ? "_left" : "") + ".yaml";
    } else if (QDir::isRelativePath(QString::fromStdString(config_path))) {
      config_path = share + "/" + config_path;
    }
    std::string svg_path = node->declare_parameter<std::string>("image_path", "");
    if (svg_path.empty()) {
      svg_path = share + "/assets/V6_" + hand + "_new.png";
    } else if (QDir::isRelativePath(QString::fromStdString(svg_path))) {
      svg_path = share + "/" + svg_path;
    }
    auto layout = allegro_hand_v6_dashboard::SensorLayout::load(config_path);
    allegro_hand_v6_dashboard::DashboardWindow window(
      std::move(node_abstraction), std::move(layout), QString::fromStdString(svg_path));
    window.show();
    result = app.exec();
  } catch (const std::exception & error) {
    QMessageBox::critical(nullptr, "V6 Dashboard", error.what());
  }

  rclcpp::shutdown();
  return result;
}
