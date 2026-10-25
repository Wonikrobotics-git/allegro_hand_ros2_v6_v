#include "allegro_hand_v6_dashboard/vision_sensor_feed.hpp"
#include "allegro_hand_v6_dashboard/frame_roll_corrector.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <QByteArray>
#include <libudev.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

namespace allegro_hand_v6_dashboard
{
constexpr int kCameraStartupStaggerMs = 150;

std::string fourcc_text(int value)
{
  std::string result(4, ' ');
  for (int i = 0; i < 4; ++i) {
    result[static_cast<std::size_t>(i)] = static_cast<char>((value >> (8 * i)) & 0xff);
  }
  return result;
}

std::string vision_sensor_image_topic(const std::string & prefix, const std::string & serial)
{
  std::string root = prefix;
  while (!root.empty() && root.back() == '/') {root.pop_back();}
  return root + "/s_" + QByteArray::fromStdString(serial).toHex().toStdString() + "/image_raw";
}

QImage decode_vision_sensor_image(const sensor_msgs::msg::Image & message)
{
  const auto limit = static_cast<uint32_t>(std::numeric_limits<int>::max());
  if ((message.encoding != "bgr8" && message.encoding != "rgb8") ||
    message.width == 0 || message.height == 0 ||
    message.width > limit / 3 || message.height > limit || message.step > limit ||
    message.step < message.width * 3 ||
    message.data.size() != static_cast<uint64_t>(message.height) * message.step)
  {
    throw std::runtime_error("VISION SENSOR image must be a valid bgr8/rgb8 buffer");
  }
  QImage image(message.data.data(), static_cast<int>(message.width),
    static_cast<int>(message.height), static_cast<int>(message.step), QImage::Format_RGB888);
  return message.encoding == "bgr8" ? image.rgbSwapped() : image.copy();
}

FingerMapping parse_finger_mapping(const std::string & text)
{
  const auto root = YAML::Load(text);
  if (!root.IsMap()) {throw std::runtime_error("mapping YAML must be a map");}
  FingerMapping result;
  result.hand = root["hand"].as<std::string>("right");
  if (result.hand != "right" && result.hand != "left") {
    throw std::runtime_error("mapping hand must be right or left");
  }
  const auto fingers = root["fingers"];
  if (!fingers || !fingers.IsMap()) {throw std::runtime_error("mapping requires fingers map");}
  const std::set<std::string> allowed{"thumb", "index", "middle", "ring", "pinky"};
  std::set<std::string> seen, seen_fingers;
  for (const auto & item : fingers) {
    const auto finger = item.first.as<std::string>();
    if (!allowed.count(finger) || !seen_fingers.insert(finger).second) {
      throw std::runtime_error("unknown or duplicate finger in mapping");
    }
    if (item.second.IsNull()) {continue;}
    const auto add = [&](const YAML::Node & value) {
        if (!value.IsScalar()) {throw std::runtime_error("serial must be a string");}
        const auto serial = value.as<std::string>();
        if (serial.empty()) {return;}
        if (!seen.insert(serial).second) {throw std::runtime_error("duplicate serial in mapping");}
        result.fingers[finger].push_back(serial);
      };
    if (item.second.IsSequence()) {
      for (const auto & value : item.second) {add(value);}
    } else {add(item.second);}
  }
  return result;
}

bool vision_sensor_is_live(const VisionSensorSnapshot & snapshot, const std::string & serial, double timeout)
{
  const auto item = snapshot.frames.find(serial);
  return std::find(snapshot.serials.begin(), snapshot.serials.end(), serial) != snapshot.serials.end() &&
         item != snapshot.frames.end() && !item->second.image.isNull() &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() - item->second.received).count() < timeout;
}

bool vision_sensor_has_frame(const VisionSensorSnapshot & snapshot, const std::string & serial)
{
  const auto item = snapshot.frames.find(serial);
  return item != snapshot.frames.end() && !item->second.image.isNull();
}

std::string assigned_serial(
  const FingerMapping & mapping, const VisionSensorSnapshot & snapshot,
  const std::string & finger, double timeout)
{
  const auto found = mapping.fingers.find(finger);
  if (found == mapping.fingers.end() || found->second.empty()) {return "";}
  for (const auto & serial : found->second) {
    if (vision_sensor_is_live(snapshot, serial, timeout)) {return serial;}
  }
  for (const auto & serial : found->second) {
    if (vision_sensor_has_frame(snapshot, serial)) {return serial;}
  }
  return found->second.front();
}

FingerMapping display_mapping(const FingerMapping & mapping, const VisionSensorSnapshot & snapshot)
{
  if (!mapping.fingers.empty()) {return mapping;}
  auto result = mapping;
  const std::vector<std::string> order{"thumb", "index", "middle", "ring", "pinky"};
  std::vector<std::string> known_serials = snapshot.serials;
  for (const auto & item : snapshot.frames) {known_serials.push_back(item.first);}
  std::sort(known_serials.begin(), known_serials.end());
  known_serials.erase(std::unique(known_serials.begin(), known_serials.end()), known_serials.end());
  for (std::size_t i = 0; i < std::min(order.size(), known_serials.size()); ++i) {
    result.fingers[order[i]] = {known_serials[i]};
  }
  return result;
}

struct VisionSensorFeed::State
{
  mutable std::mutex mutex;
  rclcpp::Node::SharedPtr node;
  VisionSensorSnapshot data;
  std::atomic<bool> stop{false};
  std::thread manager;
  std::map<std::string, std::thread> workers;
  std::string pixel_format;
  double requested_fps{30.0};
  double tear_absolute_threshold{15.0};
  double tear_ratio_threshold{8.0};
  int tear_reopen_count{3};

  static std::map<std::string, std::string> discover()
  {
    std::map<std::string, std::string> result;
    udev * context = udev_new();
    if (!context) {throw std::runtime_error("cannot create udev context");}
    udev_enumerate * enumeration = udev_enumerate_new(context);
    if (!enumeration) {udev_unref(context); throw std::runtime_error("cannot enumerate video devices");}
    udev_enumerate_add_match_subsystem(enumeration, "video4linux");
    udev_enumerate_scan_devices(enumeration);
    udev_list_entry * entry = nullptr;
    udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(enumeration)) {
      udev_device * device = udev_device_new_from_syspath(context, udev_list_entry_get_name(entry));
      if (!device) {continue;}
      const char * model = udev_device_get_property_value(device, "ID_MODEL");
      const char * serial = udev_device_get_property_value(device, "ID_SERIAL_SHORT");
      const char * capabilities = udev_device_get_property_value(device, "ID_V4L_CAPABILITIES");
      const char * path = udev_device_get_devnode(device);
      if (model && serial && capabilities && path && std::strcmp(model, "DIGIT") == 0 &&
        std::strstr(capabilities, ":capture:") != nullptr)
      {
        const auto inserted = result.emplace(serial, path);
        if (!inserted.second && inserted.first->second != path) {
          udev_device_unref(device);
          udev_enumerate_unref(enumeration);
          udev_unref(context);
          throw std::runtime_error("multiple capture nodes use serial " + std::string(serial));
        }
      }
      udev_device_unref(device);
    }
    udev_enumerate_unref(enumeration);
    udev_unref(context);
    return result;
  }

  void wait_retry(int milliseconds)
  {
    for (int elapsed = 0; elapsed < milliseconds && !stop; elapsed += 100) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void capture(const std::string & serial, int startup_delay_ms)
  {
    // Spread only the initial device opens. Successful capture remains paced
    // solely by the camera; no per-frame sleep or artificial data latency.
    wait_retry(startup_delay_ms);
    if (stop) {return;}
    std::uint64_t sequence = 0;
    int consecutive_rejected = 0;
    while (!stop) {
      cv::VideoCapture camera;
      try {
        const auto devices = discover();
        const auto found = devices.find(serial);
        if (found == devices.end()) {wait_retry(1000); continue;}
        if (!camera.open(found->second, cv::CAP_V4L2)) {
          throw std::runtime_error("cannot open " + found->second);
        }
        const int requested_fourcc = pixel_format == "mjpeg" ?
          cv::VideoWriter::fourcc('M', 'J', 'P', 'G') :
          cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
        camera.set(cv::CAP_PROP_FOURCC, requested_fourcc);
        camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
        camera.set(cv::CAP_PROP_FPS, requested_fps);
        camera.set(cv::CAP_PROP_CONVERT_RGB, 1);
        camera.set(cv::CAP_PROP_BUFFERSIZE, 1);
        const int actual_fourcc = static_cast<int>(camera.get(cv::CAP_PROP_FOURCC));
        const double actual_fps = camera.get(cv::CAP_PROP_FPS);
        if (actual_fourcc != requested_fourcc) {
          throw std::runtime_error(
                  found->second + ": requested " + fourcc_text(requested_fourcc) +
                  " but camera selected " + fourcc_text(actual_fourcc));
        }
        if (!std::isfinite(actual_fps) || std::abs(actual_fps - requested_fps) > 0.5) {
          throw std::runtime_error(
                  found->second + ": requested " + std::to_string(requested_fps) +
                  " FPS but camera selected " + std::to_string(actual_fps) + " FPS");
        }
        RCLCPP_INFO(node->get_logger(), "%s %s streaming as %s %.0fx%.0f @ %.1f FPS",
          serial.c_str(), found->second.c_str(), fourcc_text(actual_fourcc).c_str(),
          camera.get(cv::CAP_PROP_FRAME_WIDTH), camera.get(cv::CAP_PROP_FRAME_HEIGHT),
          actual_fps);
        consecutive_rejected = 0;
        while (!stop) {
          cv::Mat raw;
          if (!camera.read(raw) || raw.empty() || raw.type() != CV_8UC3) {
            throw std::runtime_error("invalid frame from " + found->second);
          }
          const auto captured_steady = std::chrono::steady_clock::now();
          const auto current_sequence = ++sequence;
          const auto roll = correct_frame_roll(raw, tear_absolute_threshold, tear_ratio_threshold);
          if (roll.decision == FrameDecision::Rejected)
          {
            ++consecutive_rejected;
            RCLCPP_WARN(node->get_logger(),
              "%s rejected torn frame seq=%lu splits=%zu edge=%.2f median=%.2f ratio=%.2f "
              "(%d consecutive)", serial.c_str(),
              static_cast<unsigned long>(current_sequence), roll.split_rows.size(), roll.maximum,
              roll.median, roll.maximum / std::max(roll.median, 0.001), consecutive_rejected);
            if (consecutive_rejected >= tear_reopen_count) {
              throw std::runtime_error(
                      serial + ": repeated torn frames; reopening camera");
            }
            continue;
          }
          consecutive_rejected = 0;
          if (roll.decision == FrameDecision::Corrected) {
            RCLCPP_INFO(node->get_logger(), "%s corrected rolled frame seq=%lu row=%d",
              serial.c_str(), static_cast<unsigned long>(current_sequence), roll.split_rows.front());
          }
          const auto & approved = roll.frame;
          cv::Mat portrait;
          cv::transpose(approved, portrait);
          cv::flip(portrait, portrait, -1);
          QImage wrapped(portrait.data, portrait.cols, portrait.rows,
            static_cast<int>(portrait.step), QImage::Format_RGB888);
          VisionSensorFrame frame{wrapped.rgbSwapped(), captured_steady};
          {
            std::lock_guard<std::mutex> lock(mutex);
            data.frames[serial] = std::move(frame);
            data.error.clear();
          }
        }
      } catch (const std::exception & error) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          data.error = serial + ": " + error.what();
        }
        RCLCPP_WARN(node->get_logger(), "%s", error.what());
      }
      camera.release();
      wait_retry(1000);
    }
  }

  void manage()
  {
    while (!stop) {
      try {
        const auto devices = discover();
        std::vector<std::string> serials;
        for (const auto & item : devices) {serials.push_back(item.first);}
        {
          std::lock_guard<std::mutex> lock(mutex);
          data.serials = serials;
          // Retain the last approved frame when the device temporarily disappears.
        }
        for (const auto & item : devices) {
          if (!workers.count(item.first)) {
            const int startup_delay_ms =
              static_cast<int>(workers.size()) * kCameraStartupStaggerMs;
            RCLCPP_INFO(node->get_logger(), "%s camera open scheduled after %d ms",
              item.first.c_str(), startup_delay_ms);
            workers.emplace(item.first, std::thread(
              [this, serial = item.first, startup_delay_ms]() {
                capture(serial, startup_delay_ms);
              }));
          }
        }
      } catch (const std::exception & error) {
        std::lock_guard<std::mutex> lock(mutex);
        data.error = error.what();
      }
      wait_retry(1000);
    }
  }

  ~State()
  {
    stop = true;
    if (manager.joinable()) {manager.join();}
    for (auto & item : workers) {
      if (item.second.joinable()) {item.second.join();}
    }
  }
};

VisionSensorFeed::VisionSensorFeed(const rclcpp::Node::SharedPtr & node, const std::string & prefix)
: state_(std::make_shared<State>())
{
  (void)prefix;
  state_->node = node;
  state_->pixel_format = node->declare_parameter<std::string>(
    "vision_sensor_pixel_format", "yuyv");
  std::transform(state_->pixel_format.begin(), state_->pixel_format.end(),
    state_->pixel_format.begin(), [](unsigned char value) {
      return static_cast<char>(std::tolower(value));
    });
  if (state_->pixel_format != "yuyv" && state_->pixel_format != "mjpeg") {
    throw std::runtime_error("vision_sensor_pixel_format must be yuyv or mjpeg");
  }
  state_->requested_fps = node->declare_parameter<double>("vision_sensor_fps", 30.0);
  if (!std::isfinite(state_->requested_fps) || state_->requested_fps <= 0.0 ||
    state_->requested_fps > 120.0)
  {
    throw std::runtime_error("vision_sensor_fps must be finite and in (0, 120]");
  }
  state_->tear_absolute_threshold = node->declare_parameter<double>(
    "vision_sensor_tear_absolute_threshold", 15.0);
  state_->tear_ratio_threshold = node->declare_parameter<double>(
    "vision_sensor_tear_ratio_threshold", 8.0);
  state_->tear_reopen_count = node->declare_parameter<int>(
    "vision_sensor_tear_reopen_count", 3);
  if (!std::isfinite(state_->tear_absolute_threshold) ||
    state_->tear_absolute_threshold <= 0.0 ||
    !std::isfinite(state_->tear_ratio_threshold) || state_->tear_ratio_threshold <= 1.0 ||
    state_->tear_reopen_count <= 0)
  {
    throw std::runtime_error("vision sensor tear thresholds must be positive");
  }
  state_->manager = std::thread([state = state_]() {state->manage();});
}

VisionSensorFeed::~VisionSensorFeed()
{
  if (!state_) {return;}
  state_->stop = true;
  if (state_->manager.joinable()) {state_->manager.join();}
  for (auto & item : state_->workers) {
    if (item.second.joinable()) {item.second.join();}
  }
}

VisionSensorSnapshot VisionSensorFeed::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  auto result = state_->data;
  result.snapshot_at = std::chrono::steady_clock::now();
  return result;
}
}  // namespace allegro_hand_v6_dashboard
