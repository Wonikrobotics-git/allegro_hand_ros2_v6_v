// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause

#ifndef ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__SENSOR_PANEL_HPP_
#define ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__SENSOR_PANEL_HPP_

#include <chrono>
#include <string>
#include <vector>

#include <QColor>
#include <QPixmap>
#include <QWidget>

#include "allegro_hand_v6_dashboard/sensor_layout.hpp"
#include "allegro_hand_v6_dashboard/vision_sensor_feed.hpp"

class QScrollArea;

namespace allegro_hand_v6_dashboard
{

QColor pressure_color(double normalized, bool valid = true);

class SensorPanel : public QWidget
{
public:
  SensorPanel(
    SensorLayout layout, const QString & svg_path,
    double stale_timeout_seconds, QWidget * parent = nullptr);

  void set_readings(const std::vector<float> & values, std::size_t received_channels,
    const std::vector<double> & raw_values = {});
  void set_vision_sensors(
    const VisionSensorSnapshot & snapshot, const FingerMapping & mapping,
    const QString & warning, double timeout);
  void refresh_status();

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  QRectF image_rect() const;
  QRectF canvas_rect() const;

  SensorLayout layout_;
  QPixmap hand_pixmap_;
  QRectF visible_hand_rect_;
  std::vector<float> values_;
  std::vector<double> raw_values_;
  std::chrono::steady_clock::time_point last_update_{};
  bool has_data_{false};
  bool size_matches_{false};
  std::size_t received_channels_{0};
  double stale_timeout_seconds_{1.0};
  double vision_sensor_timeout_seconds_{1.0};
  VisionSensorSnapshot vision_sensor_snapshot_;
  FingerMapping finger_mapping_;
  FingerMapping display_mapping_;
  QString mapping_warning_;
  QScrollArea * unknown_scroll_{nullptr};
  QWidget * unknown_strip_{nullptr};
  std::vector<std::string> unknown_serials_;
};

}  // namespace allegro_hand_v6_dashboard

#endif  // ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__SENSOR_PANEL_HPP_
