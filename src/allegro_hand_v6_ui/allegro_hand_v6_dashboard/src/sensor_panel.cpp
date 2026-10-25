// Copyright (c) 2026 Wonik Robotics
// SPDX-License-Identifier: BSD-3-Clause
#include "allegro_hand_v6_dashboard/sensor_panel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <QPainter>
#include <QPaintEvent>
#include <QScrollArea>

namespace allegro_hand_v6_dashboard
{
namespace
{
void draw_card(QPainter & painter, const QRectF & rect, const QString & title,
  const std::string & serial, const VisionSensorSnapshot & snapshot, double timeout)
{
  const bool live = !serial.empty() && vision_sensor_is_live(snapshot, serial, timeout);
  painter.save();
  painter.setPen(QPen(live ? QColor("#4e779b") : QColor("#aeb9c8"), 1.2));
  painter.setBrush(QColor("#172333"));
  painter.drawRoundedRect(rect, 8, 8);
  QFont font = painter.font();
  font.setPixelSize(13);
  font.setBold(true);
  painter.setFont(font);
  painter.setPen(Qt::white);
  painter.drawText(rect.adjusted(4, 5, -4, 0), Qt::AlignHCenter | Qt::AlignTop, title);
  font.setBold(false);
  font.setPixelSize(10);
  painter.setFont(font);
  painter.setPen(QColor("#b8cbdc"));
  const QString id = QString::fromStdString(serial);
  painter.drawText(rect.adjusted(5, 25, -5, 0), Qt::AlignHCenter | Qt::AlignTop,
    painter.fontMetrics().elidedText(id, Qt::ElideMiddle, static_cast<int>(rect.width() - 10)));
  const QRectF available = rect.adjusted(7, 43, -7, -7);
  if (live) {
    const auto & image = snapshot.frames.at(serial).image;
    const QSizeF size = QSizeF(image.size()).scaled(available.size(), Qt::KeepAspectRatio);
    painter.drawImage(QRectF(available.center() - QPointF(size.width()/2, size.height()/2), size), image);
  } else {
    painter.setPen(QColor("#d2dbe6"));
    painter.drawText(available, Qt::AlignCenter | Qt::TextWordWrap,
      serial.empty() ? "NOT MAPPED" : "NO SIGNAL");
  }
  painter.restore();
}

class UnmappedStrip : public QWidget
{
public:
  VisionSensorSnapshot snapshot;
  std::vector<std::string> serials;
  double timeout{1.0};
  using QWidget::QWidget;
protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), QColor("#edf1f7"));
    for (std::size_t i = 0; i < serials.size(); ++i) {
      draw_card(painter, QRectF(i * 142 + 5, 3, 132, 156), "UNMAPPED", serials[i],
        snapshot, timeout);
    }
  }
};
}  // namespace

QColor pressure_color(double normalized, bool valid)
{
  if (!valid) {return QColor(125, 135, 150);}
  const double t = std::clamp(normalized, 0.0, 1.0);
  if (t <= 0.5) {
    return QColor(0, static_cast<int>(std::round(t * 510)),
      static_cast<int>(std::round((1 - t * 2) * 255)));
  }
  return QColor(static_cast<int>(std::round((t * 2 - 1) * 255)),
    static_cast<int>(std::round((2 - t * 2) * 255)), 0);
}

SensorPanel::SensorPanel(SensorLayout layout, const QString & image_path,
  double stale_timeout_seconds, QWidget * parent)
: QWidget(parent), layout_(std::move(layout)), hand_pixmap_(image_path),
  values_(layout_.channel_count(), std::numeric_limits<float>::quiet_NaN()),
  stale_timeout_seconds_(stale_timeout_seconds)
{
  if (hand_pixmap_.isNull()) {throw std::runtime_error("Cannot load hand PNG: " + image_path.toStdString());}
  // Fit the opaque hand rather than the asymmetric transparent PNG margins.
  const QImage image = hand_pixmap_.toImage();
  const QRect crop = QRectF(layout_.image_crop[0], layout_.image_crop[1],
    layout_.image_crop[2], layout_.image_crop[3]).toAlignedRect().intersected(image.rect());
  QRect bounds;
  for (int y = crop.top(); y <= crop.bottom(); ++y) {
    for (int x = crop.left(); x <= crop.right(); ++x) {
      if (image.pixelColor(x, y).alpha() > 128) {
        bounds = bounds.united(QRect(x, y, 1, 1));
      }
    }
  }
  visible_hand_rect_ = bounds.isEmpty() ? QRectF(crop) : QRectF(bounds);
  setMinimumSize(320, 320);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  unknown_scroll_ = new QScrollArea(this);
  unknown_strip_ = new UnmappedStrip;
  unknown_scroll_->setWidget(unknown_strip_);
  unknown_scroll_->setFrameShape(QFrame::NoFrame);
  unknown_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  unknown_scroll_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  unknown_scroll_->hide();
}

void SensorPanel::set_readings(const std::vector<float> & values, std::size_t received_channels,
  const std::vector<double> & raw_values)
{
  raw_values_ = raw_values;
  values_ = values;
  received_channels_ = received_channels;
  size_matches_ = received_channels == layout_.input_channel_count && values.size() == layout_.channel_count();
  last_update_ = std::chrono::steady_clock::now();
  has_data_ = true;
  update();
}

void SensorPanel::set_vision_sensors(const VisionSensorSnapshot & snapshot, const FingerMapping & mapping,
  const QString & warning, double timeout)
{
  vision_sensor_snapshot_ = snapshot;
  finger_mapping_ = mapping;
  display_mapping_ = display_mapping(mapping, snapshot);
  mapping_warning_ = warning;
  vision_sensor_timeout_seconds_ = timeout;
  std::set<std::string> assigned;
  for (const auto & vision_sensor : layout_.vision_sensors) {
    const auto serial = assigned_serial(display_mapping_, snapshot, vision_sensor.finger, timeout);
    if (!serial.empty()) {assigned.insert(serial);}
  }
  unknown_serials_.clear();
  for (const auto & serial : snapshot.serials) {
    if (!assigned.count(serial)) {unknown_serials_.push_back(serial);}
  }
  auto * strip = static_cast<UnmappedStrip *>(unknown_strip_);
  strip->snapshot = snapshot;
  strip->serials = unknown_serials_;
  strip->timeout = timeout;
  strip->setFixedSize(std::max(1, static_cast<int>(unknown_serials_.size()) * 142), 162);
  strip->update();
  unknown_scroll_->setVisible(!unknown_serials_.empty());
  update();
}

void SensorPanel::refresh_status() {update();}

QRectF SensorPanel::canvas_rect() const
{
  const double header = 52;
  const double tray = unknown_serials_.empty() ? 0 : 195;
  const QRectF available(14, header, std::max(1, width() - 28),
    std::max(1.0, height() - header - 55 - tray));
  const double scale = std::min(available.width() / layout_.canvas_width,
    available.height() / layout_.canvas_height);
  const QSizeF size(layout_.canvas_width * scale, layout_.canvas_height * scale);
  return QRectF(available.center() - QPointF(size.width()/2, size.height()/2), size);
}

QRectF SensorPanel::image_rect() const
{
  const auto & box = layout_.hand_rect;
  const double scale = layout_.image_scale * std::min(
    box[2] / visible_hand_rect_.width(), box[3] / visible_hand_rect_.height());
  const QSizeF size(layout_.image_crop[2] * scale, layout_.image_crop[3] * scale);
  const QPointF center(box[0] + box[2]/2 + layout_.image_offset[0] * scale,
    box[1] + box[3]/2 + layout_.image_offset[1] * scale);
  const QPointF crop_origin(layout_.image_crop[0], layout_.image_crop[1]);
  return QRectF(center + (crop_origin - visible_hand_rect_.center()) * scale, size);
}

void SensorPanel::paintEvent(QPaintEvent * event)
{
  QWidget::paintEvent(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::SmoothPixmapTransform);
  painter.fillRect(rect(), QColor("#f7f9fc"));
  QFont font = painter.font();
  const QString heading = "V6 · Pressure & Vision Sensors";
  font.setPixelSize(28);
  font.setBold(true);
  while (font.pixelSize() > 12 && QFontMetrics(font).horizontalAdvance(heading) > width() - 24) {
    font.setPixelSize(font.pixelSize() - 1);
  }
  painter.setFont(font);
  painter.setPen(QColor("#16357d"));
  painter.drawText(QRectF(12, 12, width()-24, 32), Qt::AlignCenter,
    heading);
  const bool stale = has_data_ && std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_update_).count() > stale_timeout_seconds_;
  QString pressure_status = "Palm pressure: NO DATA";
  bool good = false;
  if (has_data_ && !size_matches_) {
    pressure_status = QString("Palm pressure: CHANNEL MISMATCH (%1 / %2)").arg(received_channels_).arg(layout_.input_channel_count);
  } else if (stale) {pressure_status = "Palm pressure: STALE";}
  else if (has_data_) {
    good = std::all_of(values_.begin(), values_.end(), [](float v) {return std::isfinite(v);});
    pressure_status = QString("Palm pressure: %1 · 3 displayed / %2 input · %3")
      .arg(good ? "LIVE" : "INVALID DATA").arg(layout_.input_channel_count).arg(QString::fromStdString(layout_.unit));
  }
  int live_count = 0, mapped_count = 0;
  for (const auto & serial : vision_sensor_snapshot_.serials) {
    live_count += vision_sensor_is_live(vision_sensor_snapshot_, serial, vision_sensor_timeout_seconds_) ? 1 : 0;
  }
  for (const auto & vision_sensor : layout_.vision_sensors) {
    mapped_count += assigned_serial(finger_mapping_, vision_sensor_snapshot_, vision_sensor.finger, vision_sensor_timeout_seconds_).empty() ? 0 : 1;
  }
  const QString vision_sensor_status = QString("Vision cameras: %1 live · %2 mapped; finger UI follows pressure triplets")
    .arg(live_count).arg(mapped_count);
  const auto warning = !vision_sensor_snapshot_.error.empty() ? QString::fromStdString(vision_sensor_snapshot_.error) : mapping_warning_;
  // Keep diagnostics available on hover without reserving space above the images.
  const QString diagnostics = pressure_status + "\n" + vision_sensor_status +
    (warning.isEmpty() ? QString() : "\n" + warning);
  if (toolTip() != diagnostics) {setToolTip(diagnostics);}

  painter.save();
  const auto canvas = canvas_rect();
  painter.translate(canvas.topLeft());
  painter.scale(canvas.width()/layout_.canvas_width, canvas.height()/layout_.canvas_height);
  const auto image = image_rect();
  painter.drawPixmap(image, hand_pixmap_, QRectF(layout_.image_crop[0], layout_.image_crop[1], layout_.image_crop[2], layout_.image_crop[3]));
  const auto anchor = [&](const std::array<double, 2> & source) {
      return QPointF(image.left() + (source[0]-layout_.image_crop[0])/layout_.image_crop[2]*image.width(),
        image.top() + (source[1]-layout_.image_crop[1])/layout_.image_crop[3]*image.height());
    };
  for (const auto & vision_sensor : layout_.vision_sensors) {
    std::array<double, 3> pressures{};
    FingerSensorMode mode = FingerSensorMode::Unknown;
    if (has_data_ && size_matches_ && !stale &&
      vision_sensor.pressure_start < raw_values_.size() && raw_values_.size() - vision_sensor.pressure_start >= 3)
    {
      std::copy_n(raw_values_.begin() + vision_sensor.pressure_start, 3, pressures.begin());
      mode = classify_finger(pressures);
    }
    const auto serial = assigned_serial(
      display_mapping_, vision_sensor_snapshot_, vision_sensor.finger,
      vision_sensor_timeout_seconds_);
    // This package is the direct-camera display experiment. A live camera must
    // remain visible even when no tactile-pressure message has arrived or the
    // finger pressure happens to be nonzero.
    if (vision_sensor_has_frame(vision_sensor_snapshot_, serial))
    {
      mode = FingerSensorMode::Vision;
    }
    if (mode == FingerSensorMode::Pressure) {
      painter.save();
      QFont pressure_font = painter.font();
      pressure_font.setPixelSize(24);
      pressure_font.setBold(true);
      painter.setFont(pressure_font);
      for (std::size_t i = 0; i < 3; ++i) {
        painter.save();
        painter.translate(anchor(vision_sensor.pressure_positions[i]));
        painter.rotate(vision_sensor.pressure_rotation);
        const QRectF box(-42, -16, 84, 32);
        const auto text = QString::number(pressures[i], 'f', 1);
        painter.setPen(QColor(255, 255, 255, 220));
        for (const QPointF offset : {QPointF(-1,0), QPointF(1,0), QPointF(0,-1), QPointF(0,1)}) {
          painter.drawText(box.translated(offset), Qt::AlignCenter, text);
        }
        painter.setPen(pressure_color((pressures[i] - layout_.pressure_min) /
          (layout_.pressure_max - layout_.pressure_min)));
        painter.drawText(box, Qt::AlignCenter, text);
        painter.restore();
      }
      painter.restore();
      continue;
    }
    const auto & r = vision_sensor.card;
    const QRectF card(r[0], r[1], r[2], r[3]);
    const auto tip = anchor(vision_sensor.tip);
    painter.setPen(QPen(QColor("#8ea5bc"), 1.5));
    painter.drawLine(QPointF(card.center().x(), card.bottom()), tip);
    painter.setBrush(QColor("#f7f9fc"));
    painter.drawEllipse(tip, 3, 3);
    if (mode == FingerSensorMode::Unknown) {
      QFont card_font = painter.font();
      card_font.setPixelSize(20);
      painter.setFont(card_font);
      painter.setBrush(QColor("#edf1f7"));
      painter.setPen(QColor("#65758b"));
      painter.drawRoundedRect(card, 8, 8);
      painter.drawText(card.adjusted(6, 6, -6, -6), Qt::AlignCenter | Qt::TextWordWrap,
        QString::fromStdString(vision_sensor.finger).toUpper() + "\n" +
        (stale ? "PRESSURE DATA STALE" : "SENSOR TYPE UNKNOWN"));
      continue;
    }
    draw_card(painter, card, QString::fromStdString(vision_sensor.finger).toUpper() +
      (finger_mapping_.fingers.empty() ? " (TEMP)" : ""),
      serial, vision_sensor_snapshot_, vision_sensor_timeout_seconds_);
  }
  font.setPixelSize(24);
  font.setBold(true);
  painter.setFont(font);
  for (const auto & sensor : layout_.sensors) {
    const bool valid = size_matches_ && !stale && sensor.index < values_.size() && std::isfinite(values_[sensor.index]);
    const double value = valid ? values_[sensor.index] : 0;
    const auto color = pressure_color((value-layout_.pressure_min)/(layout_.pressure_max-layout_.pressure_min), valid);
    const auto pos = anchor(sensor.svg);
    const auto text = valid ? QString::number(value, 'f', 1) : "--";
    const QRectF box(pos.x()-42, pos.y()-16, 84, 32);
    painter.setPen(QColor(255,255,255,220));
    for (const QPointF offset : {QPointF(-1,0), QPointF(1,0), QPointF(0,-1), QPointF(0,1)}) {
      painter.drawText(box.translated(offset), Qt::AlignCenter, text);
    }
    painter.setPen(color);
    painter.drawText(box, Qt::AlignCenter, text);
  }
  painter.restore();

  if (!unknown_serials_.empty()) {
    const int y = height()-55-195;
    font.setPixelSize(12);
    painter.setFont(font);
    painter.setPen(QColor("#a66316"));
    painter.drawText(QRectF(20,y,width()-40,22), "Unmapped cameras · assign finger_serials.yaml to place above fingertips");
    unknown_scroll_->setGeometry(20, y+23, width()-40, 174);
  }
  const QRectF legend(30, height()-40, width()-60, 12);
  QLinearGradient gradient(legend.topLeft(), legend.topRight());
  gradient.setColorAt(0, pressure_color(0));
  gradient.setColorAt(.5, pressure_color(.5));
  gradient.setColorAt(1, pressure_color(1));
  painter.setPen(Qt::NoPen);
  painter.setBrush(gradient);
  painter.drawRoundedRect(legend, 4, 4);
  font.setPixelSize(10);
  painter.setFont(font);
  painter.setPen(QColor("#26364f"));
  painter.drawText(legend.adjusted(0,14,0,20), Qt::AlignLeft, QString::number(layout_.pressure_min));
  painter.drawText(legend.adjusted(0,14,0,20), Qt::AlignRight,
    QString::number(layout_.pressure_max) + " " + QString::fromStdString(layout_.unit));
}
}  // namespace allegro_hand_v6_dashboard
