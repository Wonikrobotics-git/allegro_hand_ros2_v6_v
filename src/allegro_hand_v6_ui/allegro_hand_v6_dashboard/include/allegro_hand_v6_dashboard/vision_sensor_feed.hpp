#ifndef ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__VISION_SENSOR_FEED_HPP_
#define ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__VISION_SENSOR_FEED_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QImage>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace allegro_hand_v6_dashboard
{
using SteadyTime = std::chrono::steady_clock::time_point;
struct VisionSensorFrame
{
  QImage image;
  SteadyTime received;
};
struct VisionSensorSnapshot
{
  std::vector<std::string> serials;
  std::map<std::string, VisionSensorFrame> frames;
  std::string error;
  SteadyTime snapshot_at{};
};
struct FingerMapping
{
  std::string hand{"right"};
  std::map<std::string, std::vector<std::string>> fingers;
};

std::string vision_sensor_image_topic(const std::string & prefix, const std::string & serial);
QImage decode_vision_sensor_image(const sensor_msgs::msg::Image & message);
FingerMapping parse_finger_mapping(const std::string & text);
FingerMapping display_mapping(const FingerMapping & mapping, const VisionSensorSnapshot & snapshot);
bool vision_sensor_is_live(const VisionSensorSnapshot & snapshot, const std::string & serial, double timeout);
bool vision_sensor_has_frame(const VisionSensorSnapshot & snapshot, const std::string & serial);
std::string assigned_serial(
  const FingerMapping & mapping, const VisionSensorSnapshot & snapshot,
  const std::string & finger, double timeout);

class VisionSensorFeed
{
public:
  VisionSensorFeed(const rclcpp::Node::SharedPtr & node, const std::string & prefix);
  ~VisionSensorFeed();
  VisionSensorSnapshot snapshot() const;
private:
  struct State;
  std::shared_ptr<State> state_;
};
}  // namespace allegro_hand_v6_dashboard
#endif
