#include "allegro_hand_v6_dashboard/frame_roll_corrector.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace allegro_hand_v6_dashboard
{
namespace
{
constexpr int kClusterRowGap = 5;

struct RowScores
{
  std::vector<double> values;
  double maximum{0.0};
  double median{0.0};
};

RowScores score_rows(const cv::Mat & frame)
{
  RowScores result;
  if (frame.rows < 2) {return result;}
  result.values.reserve(static_cast<std::size_t>(frame.rows - 1));
  cv::Mat difference;
  for (int row = 1; row < frame.rows; ++row) {
    cv::absdiff(frame.row(row - 1), frame.row(row), difference);
    const auto mean = cv::mean(difference);
    const double score = (mean[0] + mean[1] + mean[2]) / 3.0;
    result.values.push_back(score);
    result.maximum = std::max(result.maximum, score);
  }
  auto ordered = result.values;
  auto middle = ordered.begin() + static_cast<std::ptrdiff_t>(ordered.size() / 2);
  std::nth_element(ordered.begin(), middle, ordered.end());
  result.median = *middle;
  return result;
}

std::vector<int> split_rows(const RowScores & scores, double absolute_threshold, double ratio_threshold)
{
  std::vector<int> result;
  int last_candidate = -kClusterRowGap - 1;
  double peak = 0.0;
  const double threshold = std::max(absolute_threshold,
    ratio_threshold * std::max(scores.median, 0.001));
  for (std::size_t i = 0; i < scores.values.size(); ++i) {
    const double score = scores.values[i];
    if (score < threshold) {continue;}
    const int row = static_cast<int>(i) + 1;
    if (row - last_candidate > kClusterRowGap) {
      result.push_back(row);
      peak = score;
    } else if (score > peak) {
      result.back() = row;
      peak = score;
    }
    last_candidate = row;
  }
  return result;
}
}  // namespace

FrameRollResult correct_frame_roll(
  const cv::Mat & frame, double absolute_threshold, double ratio_threshold)
{
  if (frame.empty() || frame.type() != CV_8UC3) {
    throw std::invalid_argument("frame roll correction requires a nonempty BGR8 frame");
  }
  const auto scores = score_rows(frame);
  FrameRollResult result;
  result.maximum = scores.maximum;
  result.median = scores.median;
  result.split_rows = split_rows(scores, absolute_threshold, ratio_threshold);
  if (result.split_rows.empty()) {
    result.frame = frame;
    return result;
  }
  result.decision = FrameDecision::Rejected;
  if (result.split_rows.size() != 1) {return result;}

  const int split = result.split_rows.front();
  cv::Mat rolled(frame.size(), frame.type());
  const int bottom_height = frame.rows - split;
  frame.rowRange(split, frame.rows).copyTo(rolled.rowRange(0, bottom_height));
  frame.rowRange(0, split).copyTo(rolled.rowRange(bottom_height, frame.rows));

  // A real wrap-around has one seam in the input and none after rotating.
  // Keep the last accepted frame if a new seam remains after the repair.
  const auto repaired_scores = score_rows(rolled);
  if (!split_rows(repaired_scores, absolute_threshold, ratio_threshold).empty()) {
    return result;
  }
  result.decision = FrameDecision::Corrected;
  result.frame = std::move(rolled);
  return result;
}
}  // namespace allegro_hand_v6_dashboard
