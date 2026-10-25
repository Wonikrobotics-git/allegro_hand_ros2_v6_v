#include <algorithm>
#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "allegro_hand_v6_dashboard/sensor_layout.hpp"
#include "allegro_hand_v6_dashboard/sensor_panel.hpp"
#include "allegro_hand_v6_dashboard/vision_sensor_feed.hpp"

using namespace allegro_hand_v6_dashboard;

TEST(Layout, SelectsOnlyPalmFromLegacyEighteenChannels)
{
  const auto layout = SensorLayout::load(TEST_CONFIG_PATH);
  ASSERT_EQ(layout.sensors.size(), 3U);
  EXPECT_EQ(layout.channel_count(), 3U);
  EXPECT_EQ(layout.input_channel_count, 18U);
  ASSERT_EQ(layout.vision_sensors.size(), 5U);
  std::vector<double> input(18);
  for (std::size_t i = 0; i < input.size(); ++i) {input[i] = 1000 + i;}
  EXPECT_EQ(layout.select_pressures(input), (std::vector<float>{1017, 1015, 1016}));
  EXPECT_EQ(layout.sensors[0].name, "palm_bottom");
  EXPECT_EQ(layout.sensors[1].name, "palm_center");
  EXPECT_EQ(layout.sensors[2].name, "palm_left");
}

TEST(Layout, ThreeChannelOrderIsExplicitlyConfigurable)
{
  auto layout = SensorLayout::load(TEST_CONFIG_PATH);
  layout.input_channel_count = 3;
  layout.sensors[0].source_index = 2;
  layout.sensors[1].source_index = 0;
  layout.sensors[2].source_index = 1;
  ASSERT_NO_THROW(layout.validate_input());
  EXPECT_EQ(layout.select_pressures({1100, 1150, 1050}), (std::vector<float>{1050, 1100, 1150}));
}

TEST(Layout, RejectsDuplicateAndOutOfRangeChannels)
{
  auto layout = SensorLayout::load(TEST_CONFIG_PATH);
  layout.sensors[1].source_index = layout.sensors[0].source_index;
  EXPECT_THROW(layout.validate_input(), std::runtime_error);
  layout.sensors[1].source_index = 18;
  EXPECT_THROW(layout.validate_input(), std::runtime_error);
}

TEST(Layout, WrongLengthDoesNotAccidentallyDisplayFingerPressures)
{
  const auto layout = SensorLayout::load(TEST_CONFIG_PATH);
  for (const auto value : layout.select_pressures({1100, 1150, 1050})) {EXPECT_TRUE(std::isnan(value));}
  for (const auto value : layout.select_pressures(std::vector<double>(19, 1100))) {EXPECT_TRUE(std::isnan(value));}
}

TEST(Layout, LeftHandHasThreePalmSensorsAndFiveMirroredCards)
{
  const auto right = SensorLayout::load(TEST_CONFIG_PATH);
  const auto left = SensorLayout::load(LEFT_TEST_CONFIG_PATH);
  EXPECT_EQ(left.sensors.size(), 3U);
  ASSERT_EQ(left.vision_sensors.size(), 5U);
  for (std::size_t i = 0; i < left.vision_sensors.size(); ++i) {
    EXPECT_EQ(left.vision_sensors[i].finger, right.vision_sensors[i].finger);
    EXPECT_DOUBLE_EQ(left.vision_sensors[i].card[0], 900 - right.vision_sensors[i].card[0] - right.vision_sensors[i].card[2]);
  }
}

TEST(Layout, ResolvesLegacyMarkersAgainstBothCurrentUrdfVariants)
{
  for (const auto path : {TEST_CONFIG_PATH, LEFT_TEST_CONFIG_PATH}) {
    for (const bool vision_model : {false, true}) {
      auto layout = SensorLayout::load(path);
      std::set<std::string> links{"base_link"};
      for (const auto * palm : {"palm_s1", "palm_s2", "palm_s3"}) {links.insert(palm);}
      for (const auto & finger : {"thumb", "index", "middle", "ring", "pinky"}) {
        for (int segment = 0; segment <= (vision_model ? 4 : 3); ++segment) {
          links.insert(std::string(finger) + "_" + std::to_string(segment));
        }
        for (int sensor = 1; sensor <= (vision_model ? 1 : 3); ++sensor) {
          links.insert(std::string(finger) + "_s" + std::to_string(sensor));
        }
      }
      const std::string hand = path == std::string(LEFT_TEST_CONFIG_PATH) ? "left" : "right";
      layout.resolve_robot_frames(links, hand);
      EXPECT_EQ(layout.sensors[0].frame_id, "palm_s3");
      EXPECT_EQ(layout.sensors[1].frame_id, "palm_s1");
      EXPECT_EQ(layout.sensors[2].frame_id, "palm_s2");
      for (const auto & palm : layout.sensors) {
        EXPECT_EQ(palm.normal, (std::array<double, 3>{0.0, -1.0, 0.0}));
        EXPECT_DOUBLE_EQ(palm.position[0], 0.0);
        EXPECT_LT(palm.position[1], 0.0);
      }
      for (const auto & placement : layout.vision_sensors) {
        ASSERT_EQ(placement.pressure_markers.size(), 3U);
        EXPECT_EQ(placement.pressure_markers[0].frame_id,
          placement.finger + (vision_model ? "_s1" : "_s3"));
        const auto & tip = placement.pressure_markers[0];
        EXPECT_DOUBLE_EQ(tip.position[0], 0.0);
        EXPECT_LT(std::abs(tip.position[1]), 0.01);
        EXPECT_LT(std::abs(tip.position[2]), 0.01);
        for (std::size_t i = 1; i < 3; ++i) {
          const auto & marker = placement.pressure_markers[i];
          EXPECT_EQ(marker.normal, (std::array<double, 3>{0.0, -1.0, 0.0}));
          if (vision_model) {
            EXPECT_LT(marker.position[1], 0.0);
            EXPECT_GT(marker.position[0] * (hand == "left" ? 1 : -1), 0.0);
          } else {
            EXPECT_EQ(marker.frame_id, placement.finger + (i == 1 ? "_s2" : "_s1"));
          }
        }
        for (const auto & marker : placement.pressure_markers) {
          EXPECT_EQ(links.count(marker.frame_id), 1U) << marker.frame_id;
        }
      }
    }
  }
}

TEST(Layout, KeepsFramesAlreadyPresentInPublishedUrdf)
{
  auto layout = SensorLayout::load(LEFT_TEST_CONFIG_PATH);
  auto & marker = layout.vision_sensors[0].pressure_markers[0];
  const auto original = marker.frame_id;
  layout.resolve_robot_frames({original, "pinky_4"}, "left");
  EXPECT_EQ(marker.frame_id, original);
}

TEST(PressureColor, InvalidGreyAndValidBlueGreenRed)
{
  EXPECT_EQ(pressure_color(0, false), QColor(125,135,150));
  EXPECT_EQ(pressure_color(-1), QColor(0,0,255));
  EXPECT_EQ(pressure_color(.5), QColor(0,255,0));
  EXPECT_EQ(pressure_color(2), QColor(255,0,0));
}

TEST(VisionSensorImage, MatchesPythonHexTopicNaming)
{
  EXPECT_EQ(vision_sensor_image_topic("/vision_sensor/", "D21462"), "/vision_sensor/s_443231343632/image_raw");
  EXPECT_NE(vision_sensor_image_topic("/vision_sensor", "D-1"), vision_sensor_image_topic("/vision_sensor", "D_1"));
}

TEST(VisionSensorImage, ConvertsBgrAndHonorsRowPaddingAndOwnsPixels)
{
  sensor_msgs::msg::Image message;
  message.width = 1; message.height = 2; message.step = 4; message.encoding = "bgr8";
  message.data = {1,2,3,99,4,5,6,99};
  const auto image = decode_vision_sensor_image(message);
  message.data.clear();
  EXPECT_EQ(image.pixelColor(0,0), QColor(3,2,1));
  EXPECT_EQ(image.pixelColor(0,1), QColor(6,5,4));
}

TEST(VisionSensorImage, RejectsInvalidBuffer)
{
  sensor_msgs::msg::Image message;
  message.width=2; message.height=1; message.step=6; message.encoding="bgr8";
  message.data={1,2,3};
  EXPECT_THROW(decode_vision_sensor_image(message), std::runtime_error);
  message.data={1,2,3,4,5,6}; message.encoding="mono8";
  EXPECT_THROW(decode_vision_sensor_image(message), std::runtime_error);
}

TEST(Mapping, SerialOrderDoesNotDetermineFingerAndUnmappedStaysEmpty)
{
  const auto mapping = parse_finger_mapping("hand: right\nfingers: {index: D2, thumb: D1, ring: null}\n");
  VisionSensorSnapshot snapshot;
  snapshot.serials = {"D2", "D1"};
  EXPECT_EQ(assigned_serial(mapping, snapshot, "index", 1), "D2");
  EXPECT_EQ(assigned_serial(mapping, snapshot, "thumb", 1), "D1");
  EXPECT_EQ(assigned_serial(mapping, snapshot, "ring", 1), "");
}

TEST(Mapping, RejectsDuplicateSerialAndUnknownFingerAndBadHand)
{
  EXPECT_THROW(parse_finger_mapping("fingers: {index: D1, ring: D1}"), std::runtime_error);
  EXPECT_THROW(parse_finger_mapping("fingers: {other: D1}"), std::runtime_error);
  EXPECT_THROW(parse_finger_mapping("hand: wrong\nfingers: {}"), std::runtime_error);
}

TEST(Mapping, SelectsLiveCandidateAndRetainsLastApprovedImage)
{
  const auto mapping = parse_finger_mapping("hand: left\nfingers: {index: [D1, D2]}");
  VisionSensorSnapshot snapshot;
  snapshot.serials = {"D2"};
  snapshot.frames["D2"] = {QImage(1,1,QImage::Format_RGB888), std::chrono::steady_clock::now()};
  EXPECT_EQ(assigned_serial(mapping, snapshot, "index", 1), "D2");
  EXPECT_TRUE(vision_sensor_is_live(snapshot, "D2", 1));
  snapshot.frames["D2"].received -= std::chrono::seconds(2);
  EXPECT_FALSE(vision_sensor_is_live(snapshot, "D2", 1));
  EXPECT_TRUE(vision_sensor_has_frame(snapshot, "D2"));
  EXPECT_EQ(assigned_serial(mapping, snapshot, "index", 1), "D2");
}

TEST(Mapping, KeepsApprovedImageAvailableAfterDeviceDisappears)
{
  VisionSensorSnapshot snapshot;
  snapshot.frames["D2"] = {QImage(1, 1, QImage::Format_RGB888),
    std::chrono::steady_clock::now() - std::chrono::seconds(10)};
  EXPECT_FALSE(vision_sensor_is_live(snapshot, "D2", 1));
  EXPECT_TRUE(vision_sensor_has_frame(snapshot, "D2"));
  const auto mapping = display_mapping({}, snapshot);
  EXPECT_EQ(assigned_serial(mapping, snapshot, "thumb", 1), "D2");
}

TEST(Mapping, PrefersCachedCandidateWhenNoCandidateIsLive)
{
  const auto mapping = parse_finger_mapping("hand: left\nfingers: {index: [D1, D2]}");
  VisionSensorSnapshot snapshot;
  snapshot.frames["D2"] = {QImage(1, 1, QImage::Format_RGB888),
    std::chrono::steady_clock::now() - std::chrono::seconds(10)};
  EXPECT_EQ(assigned_serial(mapping, snapshot, "index", 1), "D2");
}

TEST(FingerSensors, ClassifiesEachTripletWithoutThresholdOrLatching)
{
  EXPECT_EQ(classify_finger({0, 0, 0}), FingerSensorMode::Vision);
  for (std::size_t i = 0; i < 3; ++i) {
    std::array<double, 3> values{};
    values[i] = 0.001;
    EXPECT_EQ(classify_finger(values), FingerSensorMode::Pressure);
    values[i] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(classify_finger(values), FingerSensorMode::Unknown);
    values[i] = std::numeric_limits<double>::infinity();
    EXPECT_EQ(classify_finger(values), FingerSensorMode::Unknown);
    values[i] = -1;
    EXPECT_EQ(classify_finger(values), FingerSensorMode::Unknown);
  }
}

TEST(FingerSensors, BothHandsUseWireOrderAndPositionsInsideImage)
{
  const std::vector<std::string> order{"thumb", "index", "middle", "ring", "pinky"};
  for (const auto * path : {TEST_CONFIG_PATH, LEFT_TEST_CONFIG_PATH}) {
    const auto layout = SensorLayout::load(path);
    for (const auto & vision_sensor : layout.vision_sensors) {
      const auto it = std::find(order.begin(), order.end(), vision_sensor.finger);
      EXPECT_EQ(vision_sensor.pressure_start, static_cast<std::size_t>(it - order.begin()) * 3);
      EXPECT_EQ(vision_sensor.pressure_positions[0], vision_sensor.tip);
      for (const auto & pos : vision_sensor.pressure_positions) {
        EXPECT_GE(pos[0], layout.image_crop[0]);
        EXPECT_LE(pos[0], layout.image_crop[0] + layout.image_crop[2]);
        EXPECT_GE(pos[1], layout.image_crop[1]);
        EXPECT_LE(pos[1], layout.image_crop[1] + layout.image_crop[3]);
      }
    }
  }
}

TEST(FingerSensors, MarkersCoverFifteenChannelsWithUniqueIdsAndUnitNormals)
{
  for (const auto * path : {TEST_CONFIG_PATH, LEFT_TEST_CONFIG_PATH}) {
    const auto layout = SensorLayout::load(path);
    std::vector<std::size_t> sources, ids;
    for (const auto & palm : layout.sensors) {ids.push_back(palm.index);}
    for (const auto & vision_sensor : layout.vision_sensors) {
      ASSERT_EQ(vision_sensor.pressure_markers.size(), 3U);
      for (std::size_t i = 0; i < 3; ++i) {
        const auto & marker = vision_sensor.pressure_markers[i];
        EXPECT_EQ(marker.source_index, vision_sensor.pressure_start + i);
        EXPECT_EQ(marker.name, vision_sensor.finger + (i == 0 ? "_tip" : (i == 1 ? "_mid" : "_base")));
        EXPECT_FALSE(marker.frame_id.empty());
        double squared_length = 0;
        for (double v : marker.normal) {squared_length += v * v;}
        EXPECT_NEAR(squared_length, 1.0, 1e-9);
        sources.push_back(marker.source_index);
        ids.push_back(marker.index);
      }
    }
    std::sort(sources.begin(), sources.end());
    std::sort(ids.begin(), ids.end());
    ASSERT_EQ(sources.size(), 15U);
    ASSERT_EQ(ids.size(), 18U);
    for (std::size_t i = 0; i < sources.size(); ++i) {EXPECT_EQ(sources[i], i);}
    for (std::size_t i = 0; i < ids.size(); ++i) {EXPECT_EQ(ids[i], i);}
  }
}

TEST(Layout, BothPalmsMapWireOrderTopToBottomIn2DAnd3D)
{
  const auto right = SensorLayout::load(TEST_CONFIG_PATH);
  const auto left = SensorLayout::load(LEFT_TEST_CONFIG_PATH);
  for (const auto & layout : {right, left}) {
    auto palm = layout.sensors;
    std::sort(palm.begin(), palm.end(), [](const auto & a, const auto & b) {
      return a.svg[1] < b.svg[1];
    });
    std::vector<double> raw(18, 0);
    raw[15] = 1050; raw[16] = 1100; raw[17] = 1150;
    const auto displayed = layout.select_pressures(raw);
    for (std::size_t i = 0; i < 3; ++i) {
      EXPECT_EQ(palm[i].source_index, 15 + i);
      EXPECT_EQ(displayed[palm[i].index], raw[15 + i]);
      EXPECT_EQ(palm[i].frame_id, "base_link");
      if (i > 0) {EXPECT_GT(palm[i - 1].position[0], palm[i].position[0]);}
    }
  }
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(right.sensors[i].source_index, left.sensors[i].source_index);
    EXPECT_DOUBLE_EQ(right.sensors[i].position[0], left.sensors[i].position[0]);
    EXPECT_DOUBLE_EQ(right.sensors[i].position[1], -left.sensors[i].position[1]);
    EXPECT_DOUBLE_EQ(right.sensors[i].position[2], left.sensors[i].position[2]);
  }
}
