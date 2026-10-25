// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause

#include "allegro_hand_v6_dashboard/dashboard_window.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <filesystem>
#include <stdexcept>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QLabel>
#include <QGuiApplication>
#include <QResizeEvent>
#include <QScreen>
#include <QSplitter>
#include <QTabBar>
#include <QTimer>
#include <QVBoxLayout>
#include <QXmlStreamReader>

#include "geometry_msgs/msg/point.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/render_system.hpp"

#include "allegro_hand_v6_dashboard/sensor_panel.hpp"

namespace allegro_hand_v6_dashboard
{
namespace
{

std::string default_mapping_path()
{
  namespace fs = std::filesystem;
  const char * config_home = std::getenv("XDG_CONFIG_HOME");
  const fs::path configured = config_home && *config_home ? fs::path(config_home) : fs::path();
  const fs::path root = configured.is_absolute() ?
    configured : fs::path(QDir::homePath().toStdString()) / ".config";
  return (root / "allegro_hand_vision_sensor" / "finger_serials.yaml").string();
}

QWidget * titled_panel(const QString & title, QWidget * content)
{
  auto * panel = new QWidget;
  auto * layout = new QVBoxLayout(panel);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  auto * label = new QLabel(title);
  label->setAlignment(Qt::AlignCenter);
  label->setMinimumHeight(38);
  label->setStyleSheet(
    "QLabel { background: #16357d; color: white; font-size: 15px; font-weight: 700; }");
  layout->addWidget(label);
  layout->addWidget(content, 1);
  return panel;
}

std_msgs::msg::ColorRGBA marker_color(double normalized, bool valid)
{
  const QColor color = pressure_color(normalized, valid);
  std_msgs::msg::ColorRGBA result;
  result.r = static_cast<float>(color.redF());
  result.g = static_cast<float>(color.greenF());
  result.b = static_cast<float>(color.blueF());
  result.a = valid ? 0.92F : 0.35F;
  return result;
}

}  // namespace

DashboardWindow::DashboardWindow(
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_abstraction,
  SensorLayout sensor_layout, const QString & svg_path, QWidget * parent)
: QMainWindow(parent),
  node_abstraction_(std::move(node_abstraction)),
  node_(node_abstraction_->get_raw_node()),
  sensor_layout_(std::move(sensor_layout))
{
  show_visualization_ = node_->declare_parameter<bool>("show_visualization", true);
  show_pressure_markers_ = node_->declare_parameter<bool>("show_pressure_markers", true);
  const auto visualization_title = node_->declare_parameter<std::string>(
    "visualization_title", "Live 3D Hand & Pressure");
  show_sensor_panel_ = node_->declare_parameter<bool>("show_sensor_panel", true);
  fixed_frame_ = node_->declare_parameter<std::string>("fixed_frame", "world");
  robot_description_topic_ = node_->declare_parameter<std::string>(
    "robot_description_topic", "/robot_description");
  pressure_topic_ = node_->declare_parameter<std::string>(
    "pressure_topic", "/allegro_hand/tactile_pressures");
  marker_topic_ = node_->declare_parameter<std::string>(
    "marker_topic", "/allegro_hand_v6_v/tactile_markers");
  stale_timeout_seconds_ = node_->declare_parameter<double>("stale_timeout", 1.0);
  vision_sensor_timeout_seconds_ = node_->declare_parameter<double>("vision_sensor_timeout", 1.0);
  if (!std::isfinite(stale_timeout_seconds_) || stale_timeout_seconds_ <= 0 ||
    !std::isfinite(vision_sensor_timeout_seconds_) || vision_sensor_timeout_seconds_ <= 0)
  {
    throw std::runtime_error("pressure and VISION SENSOR timeouts must be finite and positive");
  }
  hand_ = node_->get_parameter("hand").as_string();
  mapping_path_ = QString::fromStdString(node_->declare_parameter<std::string>(
    "mapping_file", default_mapping_path()));
  if (mapping_path_.startsWith("~/")) {mapping_path_.replace(0, 1, QDir::homePath());}
  const auto count = node_->declare_parameter<int64_t>(
    "pressure_channel_count", static_cast<int64_t>(sensor_layout_.input_channel_count));
  if (count <= 0) {throw std::runtime_error("pressure_channel_count must be positive");}
  sensor_layout_.input_channel_count = static_cast<std::size_t>(count);
  std::vector<int64_t> default_indices;
  for (const auto & sensor : sensor_layout_.sensors) {
    default_indices.push_back(static_cast<int64_t>(sensor.source_index));
  }
  const auto indices = node_->declare_parameter<std::vector<int64_t>>("pressure_indices", default_indices);
  if (indices.size() != 3) {throw std::runtime_error("pressure_indices requires display-order indices: bottom, upper (palm_center), middle (palm_left)");}
  for (std::size_t i = 0; i < indices.size(); ++i) {
    if (indices[i] < 0) {throw std::runtime_error("pressure_indices must be nonnegative");}
    sensor_layout_.sensors[i].source_index = static_cast<std::size_t>(indices[i]);
  }
  sensor_layout_.validate_input();
  const auto vision_sensor_prefix = node_->declare_parameter<std::string>("vision_sensor_topic_prefix", "/vision_sensor");
  view_yaw_ = node_->declare_parameter<double>("view_yaw", 3.403392);
  view_pitch_ = node_->declare_parameter<double>("view_pitch", 0.15);
  view_distance_ = node_->declare_parameter<double>("view_distance", 0.70);
  view_focal_x_ = node_->declare_parameter<double>("view_focal_x", 0.07);
  view_focal_z_ = node_->declare_parameter<double>("view_focal_z", 0.11);
  if (!show_visualization_ && !show_sensor_panel_) {
    throw std::runtime_error("at least one dashboard panel must be enabled");
  }

  setWindowTitle("V6 Dashboard");
  setMinimumSize(480, 400);

  auto * central = new QWidget;
  auto * central_layout = new QVBoxLayout(central);
  central_layout->setContentsMargins(0, 0, 0, 0);
  panel_tabs_ = new QTabBar(central);
  panel_tabs_->setObjectName("dashboardPanelTabs");
  panel_tabs_->addTab("3D Hand");
  panel_tabs_->addTab("Sensors");
  panel_tabs_->setExpanding(true);
  central_layout->addWidget(panel_tabs_);
  auto * splitter = new QSplitter(Qt::Horizontal, central);
  panel_splitter_ = splitter;
  splitter->setObjectName("dashboardPanels");
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(5);
  splitter->setStyleSheet("QSplitter::handle { background: #cbd5e5; }");

  if (show_visualization_) {
    auto * rviz_host = new QWidget;
    auto * rviz_layout = new QVBoxLayout(rviz_host);
    rviz_layout->setContentsMargins(0, 0, 0, 0);
    robot_status_ = new QLabel("Waiting for robot model · start bringup", rviz_host);
    robot_status_->setWordWrap(true);
    robot_status_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    robot_status_->setMargin(6);
    rviz_layout->addWidget(robot_status_);
    render_panel_ = new rviz_common::RenderPanel(rviz_host);
    rviz_layout->addWidget(render_panel_, 1);
    visualization_panel_ = titled_panel(QString::fromStdString(visualization_title), rviz_host);
    visualization_panel_->setObjectName("dashboard3DPanel");
    splitter->addWidget(visualization_panel_);
  }

  if (show_sensor_panel_) {
    sensor_panel_ = new SensorPanel(
      sensor_layout_, svg_path, stale_timeout_seconds_);
    sensor_panel_->setObjectName("dashboardSensorPanel");
    vision_sensor_feed_ = std::make_unique<VisionSensorFeed>(node_, vision_sensor_prefix);
    splitter->addWidget(sensor_panel_);
  }
  if (splitter->count() == 2) {
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 2);
    splitter->setSizes({650, 1150});
  }
  central_layout->addWidget(splitter, 1);
  setCentralWidget(central);
  connect(panel_tabs_, &QTabBar::currentChanged, this, [this]() {update_panel_layout();});
  QSize initial(show_visualization_ && show_sensor_panel_ ? 1850 : 1400, 1000);
  if (const auto * screen = QGuiApplication::primaryScreen()) {
    initial = initial.boundedTo(screen->availableGeometry().size() * 0.92);
  }
  resize(initial.expandedTo(minimumSize()));
  update_panel_layout();

  if (show_visualization_) {
    // Read the retained description too, including when bringup started before the dashboard.
    const auto configured_layout = sensor_layout_;
    robot_description_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      robot_description_topic_, rclcpp::QoS(1).transient_local(),
      [this, configured_layout](std_msgs::msg::String::SharedPtr message) {
        QMetaObject::invokeMethod(this, [this, configured_layout, message]() {
          QXmlStreamReader xml(QString::fromStdString(message->data));
          std::set<std::string> links;
          QString side;
          QString model;
          while (!xml.atEnd()) {
            xml.readNext();
            if (!xml.isStartElement()) {continue;}
            if (xml.name() == "link") {
              links.insert(xml.attributes().value("name").toString().toStdString());
            } else if (xml.name() == "mesh") {
              const auto path = xml.attributes().value("filename").toString();
              if (path.contains("/left/")) {side = "left";}
              if (path.contains("/right/")) {side = "right";}
              if (path.contains("/v6_v/")) {model = "v6_v";}
              if (path.contains("/v6_f/")) {model = "v6_f";}
            }
          }
          if (xml.hasError() || links.empty()) {
            robot_status_->setText("Invalid robot model: check the bringup URDF");
            return;
          }
          sensor_layout_ = configured_layout;
          sensor_layout_.resolve_robot_frames(links, side.isEmpty() ? hand_ : side.toStdString());
          QString status;
          if (side.isEmpty()) {
            status = "Legacy or custom URDF received · check bringup model and hand";
          } else if (side.toStdString() != hand_) {
            status = QString("Bringup publishes %1; dashboard hand=%2. Restart bringup with hand:=%2")
              .arg(side, QString::fromStdString(hand_));
          } else {
            status = QString("Robot model: %1 %2").arg(model, side);
          }
          robot_status_->setText(status);
          RCLCPP_INFO(node_->get_logger(), "%s", status.toStdString().c_str());
        }, Qt::QueuedConnection);
      });
    // OGRE needs the RenderPanel to have its final Qt/X11 hierarchy and native
    // window before it initializes. The zero-delay callback runs after
    // DashboardWindow::show() has delivered those events.
    QTimer::singleShot(0, this, [this]() {setup_rviz();});
  }

  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic_, rclcpp::QoS(5));
  tactile_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    pressure_topic_, rclcpp::SensorDataQoS(),
    [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
      // RViz may spin the node outside the Qt thread.
      QMetaObject::invokeMethod(this, [this, message]() {on_tactile(message);}, Qt::QueuedConnection);
    });

  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this,
    [this]() {
      if (!rclcpp::ok()) {
        QCoreApplication::quit();
        return;
      }
      // VisualizationManager owns an executor for this same node. In
      // sensor-only mode there is no manager, so the Qt timer spins it.
      if (!visualization_manager_) {
        rclcpp::spin_some(node_);
      }
      if (sensor_panel_) {
        refresh_vision_sensors();
        sensor_panel_->refresh_status();
      }
    });
  ros_timer_->start(33);

  RCLCPP_INFO(
    node_->get_logger(), "Dashboard ready: visualization=%s sensor_panel=%s, showing %zu palm sensors from %zu channels on %s",
    show_visualization_ ? "true" : "false", show_sensor_panel_ ? "true" : "false",
    sensor_layout_.channel_count(), sensor_layout_.input_channel_count, pressure_topic_.c_str());
}

DashboardWindow::~DashboardWindow()
{
  if (ros_timer_) {
    ros_timer_->stop();
  }
  tactile_subscription_.reset();
  robot_description_subscription_.reset();
  vision_sensor_feed_.reset();
  marker_publisher_.reset();
  if (visualization_manager_) {
    visualization_manager_->stopUpdate();
    visualization_manager_->removeAllDisplays();
    visualization_manager_.reset();
  }
}

void DashboardWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
  update_panel_layout();
}

void DashboardWindow::update_panel_layout()
{
  if (!panel_tabs_ || !panel_splitter_) {return;}
  const bool both = show_visualization_ && show_sensor_panel_;
  const bool compact = both && (width() < 1100 || height() < 650 || width() < height() * 1.2);
  panel_tabs_->setVisible(compact);
  if (visualization_panel_) {
    visualization_panel_->setVisible(!compact || panel_tabs_->currentIndex() == 0);
  }
  if (sensor_panel_) {
    sensor_panel_->setVisible(!compact || panel_tabs_->currentIndex() == 1);
  }
  if (both && !compact && compact_layout_) {panel_splitter_->setSizes({650, 1150});}
  compact_layout_ = compact;
}

void DashboardWindow::setup_rviz()
{
  rviz_rendering::RenderSystem::get();
  // VisualizationManager initializes its root display in the
  // constructor and therefore requires a live scene manager first. This is
  // the same ordering used by rviz_common::VisualizationFrame.
  render_panel_->getRenderWindow()->initialize();
  visualization_manager_ = std::make_unique<rviz_common::VisualizationManager>(
    render_panel_, node_abstraction_, nullptr, node_->get_clock());
  render_panel_->initialize(visualization_manager_.get());
  visualization_manager_->initialize();
  visualization_manager_->setFixedFrame(QString::fromStdString(fixed_frame_));

  auto * grid = visualization_manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  if (grid) {
    grid->subProp("Cell Size")->setValue(0.05);
    grid->subProp("Plane Cell Count")->setValue(10);
    grid->subProp("Alpha")->setValue(0.25);
  }

  auto * robot = visualization_manager_->createDisplay(
    "rviz_default_plugins/RobotModel", "Allegro Hand V6", true);
  if (robot) {
    if (auto * property = robot->subProp("Description Source")) {
      property->setValue("Topic");
    }
    if (auto * property = robot->subProp("Description Topic")) {
      property->setValue(QString::fromStdString(robot_description_topic_));
    }
  }

  auto * markers = visualization_manager_->createDisplay(
    "rviz_default_plugins/MarkerArray", "Tactile Pressure", true);
  if (markers) {
    markers->setEnabled(show_pressure_markers_);
    markers->setTopic(
      QString::fromStdString(marker_topic_), "visualization_msgs/msg/MarkerArray");
  }

  auto * views = visualization_manager_->getViewManager();
  views->setCurrentViewControllerType("rviz_default_plugins/Orbit");
  if (auto * view = views->getCurrent()) {
    if (auto * property = view->subProp("Distance")) {
      property->setValue(view_distance_);
    }
    if (auto * property = view->subProp("Pitch")) {
      property->setValue(view_pitch_);
    }
    if (auto * property = view->subProp("Yaw")) {
      property->setValue(view_yaw_);
    }
    if (auto * focal = view->subProp("Focal Point")) {
      focal->subProp("X")->setValue(view_focal_x_);
      focal->subProp("Y")->setValue(0.0);
      focal->subProp("Z")->setValue(view_focal_z_);
    }
  }
  visualization_manager_->startUpdate();
}

void DashboardWindow::on_tactile(const std_msgs::msg::Float64MultiArray::SharedPtr message)
{
  const auto values = sensor_layout_.select_pressures(message->data);
  if (sensor_panel_) {
    sensor_panel_->set_readings(values, message->data.size(), message->data);
  }
  if (show_visualization_ && show_pressure_markers_) {
    publish_markers(message->data);
  }
}

void DashboardWindow::publish_markers(const std::vector<double> & values)
{
  // A standalone sensor-panel launch has no robot_state_publisher. Avoid
  // flooding RViz's message filter with untransformable link-frame markers;
  // publication begins automatically once the V6 TF tree is present.
  const bool markers_in_fixed_frame = std::all_of(
    sensor_layout_.sensors.begin(), sensor_layout_.sensors.end(),
    [this](const auto & sensor) {return sensor.frame_id == fixed_frame_;});
  if (!markers_in_fixed_frame && node_->count_publishers("/tf") == 0U &&
    node_->count_publishers("/tf_static") == 0U) {
    return;
  }
  visualization_msgs::msg::MarkerArray array;
  auto sensors = sensor_layout_.sensors;
  for (const auto & vision_sensor : sensor_layout_.vision_sensors) {
    sensors.insert(sensors.end(), vision_sensor.pressure_markers.begin(), vision_sensor.pressure_markers.end());
  }
  array.markers.reserve(sensors.size());
  const auto stamp = node_->now();
  for (const auto & sensor : sensors) {
    const bool valid = values.size() == sensor_layout_.input_channel_count &&
      (sensor.index < 3 || sensor_layout_.input_channel_count >= 15) &&
      sensor.source_index < values.size() && std::isfinite(values[sensor.source_index]);
    const double value = valid ? values[sensor.source_index] : 0.0;
    const double normalized = std::clamp(
      (value - sensor_layout_.pressure_min) /
      (sensor_layout_.pressure_max - sensor_layout_.pressure_min), 0.0, 1.0);
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = sensor.frame_id;
    arrow.header.stamp = stamp;
    arrow.ns = "tactile_arrows";
    arrow.id = static_cast<int>(sensor.index);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = valid && value > 0.0 ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    arrow.frame_locked = true;
    arrow.lifetime = rclcpp::Duration::from_seconds(stale_timeout_seconds_);
    geometry_msgs::msg::Point start;
    start.x = sensor.position[0];
    start.y = sensor.position[1];
    start.z = sensor.position[2];
    geometry_msgs::msg::Point end = start;
    const double length = 0.012 + 0.075 * normalized;
    end.x += sensor.normal[0] * length;
    end.y += sensor.normal[1] * length;
    end.z += sensor.normal[2] * length;
    arrow.points = {start, end};
    arrow.scale.x = 0.0025 + 0.0035 * normalized;
    arrow.scale.y = 0.006 + 0.007 * normalized;
    arrow.scale.z = 0.008 + 0.010 * normalized;
    arrow.color = marker_color(normalized, valid);
    const bool is_tip = sensor.name.size() >= 4 &&
      sensor.name.compare(sensor.name.size() - 4, 4, "_tip") == 0;
    if (is_tip) {
      // Keep the marker identity so an existing arrow is replaced in RViz.
      arrow.type = visualization_msgs::msg::Marker::SPHERE;
      arrow.points.clear();
      arrow.pose.position = start;
      arrow.pose.orientation.w = 1.0;
      const double diameter = 0.014 + 0.020 * normalized;
      arrow.scale.x = diameter;
      arrow.scale.y = diameter;
      arrow.scale.z = diameter;
      arrow.color.a = 0.4F;
    }
    array.markers.push_back(arrow);
  }
  marker_publisher_->publish(array);
}

void DashboardWindow::refresh_vision_sensors()
{
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - mapping_checked_).count() >= 1.0) {
    mapping_checked_ = now;
    QFile file(mapping_path_);
    if (!file.open(QIODevice::ReadOnly)) {
      mapping_ = {};
      mapping_loaded_ = false;
      mapping_warning_ = "Mapping file unavailable · camera placement is temporary";
    } else {
      const auto text = file.readAll().toStdString();
      if (!mapping_loaded_ || text != mapping_content_) {
        mapping_content_ = text;
        mapping_loaded_ = true;
        try {
          auto candidate = parse_finger_mapping(text);
          if (candidate.hand != hand_) {throw std::runtime_error("YAML hand does not match dashboard hand");}
          mapping_ = std::move(candidate);
          mapping_warning_.clear();
        } catch (const std::exception & error) {
          mapping_ = {};
          mapping_warning_ = QString("Mapping error: ") + error.what();
        }
      }
    }
  }
  sensor_panel_->set_vision_sensors(vision_sensor_feed_->snapshot(), mapping_, mapping_warning_, vision_sensor_timeout_seconds_);
}

}  // namespace allegro_hand_v6_dashboard
