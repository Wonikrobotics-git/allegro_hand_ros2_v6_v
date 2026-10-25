#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "allegro_hand_v6_dashboard/frame_roll_corrector.hpp"

using namespace allegro_hand_v6_dashboard;

TEST(FrameRoll, AcceptsNormalFrame)
{
  cv::Mat frame(240, 320, CV_8UC3);
  for (int row = 0; row < frame.rows; ++row) {
    frame.row(row).setTo(cv::Scalar(row, row, row));
  }
  const auto result = correct_frame_roll(frame, 15.0, 8.0);
  EXPECT_EQ(result.decision, FrameDecision::Normal);
  EXPECT_TRUE(result.split_rows.empty());
  EXPECT_EQ(result.frame.data, frame.data);
}

TEST(FrameRoll, CorrectsOneWrappedBoundary)
{
  cv::Mat expected(240, 320, CV_8UC3);
  for (int row = 0; row < expected.rows; ++row) {
    expected.row(row).setTo(cv::Scalar(row, row, row));
  }
  cv::Mat rolled(expected.size(), expected.type());
  expected.rowRange(50, 240).copyTo(rolled.rowRange(0, 190));
  expected.rowRange(0, 50).copyTo(rolled.rowRange(190, 240));

  const auto result = correct_frame_roll(rolled, 15.0, 8.0);
  EXPECT_EQ(result.decision, FrameDecision::Corrected);
  EXPECT_EQ(result.split_rows, (std::vector<int>{190}));
  ASSERT_FALSE(result.frame.empty());
  EXPECT_EQ(cv::norm(result.frame, expected, cv::NORM_INF), 0.0);
}

TEST(FrameRoll, RejectsTwoIndependentBoundaries)
{
  cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  frame.rowRange(80, 160).setTo(cv::Scalar(200, 200, 200));
  const auto result = correct_frame_roll(frame, 15.0, 8.0);
  EXPECT_EQ(result.decision, FrameDecision::Rejected);
  EXPECT_EQ(result.split_rows, (std::vector<int>{80, 160}));
  EXPECT_TRUE(result.frame.empty());
}

TEST(FrameRoll, RejectsSingleSceneEdgeThatCannotBeRepaired)
{
  cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
  frame.rowRange(120, 240).setTo(cv::Scalar(100, 100, 100));
  const auto result = correct_frame_roll(frame, 15.0, 8.0);
  EXPECT_EQ(result.decision, FrameDecision::Rejected);
  EXPECT_EQ(result.split_rows, (std::vector<int>{120}));
  EXPECT_TRUE(result.frame.empty());
}
