#ifndef ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__FRAME_ROLL_CORRECTOR_HPP_
#define ALLEGRO_HAND_V6_V_DASHBOARD_EXPERIMENTAL__FRAME_ROLL_CORRECTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace allegro_hand_v6_dashboard
{
enum class FrameDecision {Normal, Corrected, Rejected};

struct FrameRollResult
{
  FrameDecision decision{FrameDecision::Normal};
  cv::Mat frame;
  std::vector<int> split_rows;
  double maximum{0.0};
  double median{0.0};
};

FrameRollResult correct_frame_roll(
  const cv::Mat & frame, double absolute_threshold, double ratio_threshold);
}  // namespace allegro_hand_v6_dashboard

#endif
