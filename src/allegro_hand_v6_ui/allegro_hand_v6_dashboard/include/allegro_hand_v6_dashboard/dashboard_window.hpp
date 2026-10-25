// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__DASHBOARD_WINDOW_HPP_
#define ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__DASHBOARD_WINDOW_HPP_

#include <memory>
#include <string>
#include <vector>

#include <QMainWindow>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "allegro_hand_v6_dashboard/sensor_layout.hpp"
#include "allegro_hand_v6_dashboard/vision_sensor_feed.hpp"

class QTimer;
class QLabel;
class QResizeEvent;
class QSplitter;
class QTabBar;

namespace rviz_common
{
class RenderPanel;
class VisualizationManager;
}

namespace allegro_hand_v6_dashboard
{

class SensorPanel;

class DashboardWindow : public QMainWindow
{
public:
  DashboardWindow(
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_abstraction,
    SensorLayout sensor_layout, const QString & svg_path, QWidget * parent = nullptr);
  ~DashboardWindow() override;

protected:
  void resizeEvent(QResizeEvent * event) override;

private:
  void update_panel_layout();
  void setup_rviz();
  void on_tactile(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  void publish_markers(const std::vector<double> & values);
  void refresh_vision_sensors();

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_abstraction_;
  rclcpp::Node::SharedPtr node_;
  SensorLayout sensor_layout_;
  SensorPanel * sensor_panel_{nullptr};
  std::unique_ptr<VisionSensorFeed> vision_sensor_feed_;
  FingerMapping mapping_;
  QString mapping_path_;
  QString mapping_warning_;
  std::string mapping_content_;
  std::string hand_;
  SteadyTime mapping_checked_{};
  bool mapping_loaded_{false};
  double vision_sensor_timeout_seconds_{1.0};
  rviz_common::RenderPanel * render_panel_{nullptr};
  std::unique_ptr<rviz_common::VisualizationManager> visualization_manager_;
  QTimer * ros_timer_{nullptr};
  QLabel * robot_status_{nullptr};
  QSplitter * panel_splitter_{nullptr};
  QTabBar * panel_tabs_{nullptr};
  QWidget * visualization_panel_{nullptr};
  bool compact_layout_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tactile_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  bool show_visualization_{true};
  bool show_pressure_markers_{true};
  bool show_sensor_panel_{true};
  std::string fixed_frame_{"world"};
  std::string robot_description_topic_{"/robot_description"};
  std::string pressure_topic_{"/allegro_hand/tactile_pressures"};
  std::string marker_topic_{"/allegro_hand_v6/tactile_markers"};
  double stale_timeout_seconds_{1.0};
  double view_yaw_{3.403392};
  double view_pitch_{0.15};
  double view_distance_{0.70};
  double view_focal_x_{0.07};
  double view_focal_z_{0.11};
};

}  // namespace allegro_hand_v6_dashboard

#endif  // ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__DASHBOARD_WINDOW_HPP_
