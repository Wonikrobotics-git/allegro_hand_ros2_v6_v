/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file stopwatch.hpp
 * @brief Stopwatch for profiling Allegro Hand V6.
 * @author ('c')void
 */

#pragma once

#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>


/**
 * @class StopWatch
 * @brief Lightweight real-time cycle profiling tool to record and print lap times in milliseconds.
 */
class StopWatch {
  int sequence_ = 0;
  std::vector<std::pair<std::string, std::chrono::high_resolution_clock::time_point>> measures_;

  StopWatch() = default;

public:
  static StopWatch& instance() {
    static StopWatch inst;
    return inst;
  }

  StopWatch(const StopWatch&) = delete;
  StopWatch& operator=(const StopWatch&) = delete;
  StopWatch(StopWatch&&) = delete;
  StopWatch& operator=(StopWatch&&) = delete;

  inline void reset() {
    measures_.clear();
    ++sequence_; 
  }

  inline void clear() {
    measures_.clear();
    ++sequence_;
  }

  inline void append(const std::string& tag, std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now()) {
    measures_.emplace_back(tag, tp);
  }

  inline void print(double threshold_ms = 3.0) {
    if (measures_.size() < 2) return;

    double total_ms = std::chrono::duration<double, std::milli>(
        measures_.back().second - measures_.front().second).count();

    if (total_ms > threshold_ms) {
      std::string report = "";
      for (size_t i = 1; i < measures_.size(); ++i) {
        double lap_ms = std::chrono::duration<double, std::milli>(
            measures_[i].second - measures_[i - 1].second).count();
        report += fmt::format("\n  {} -> {}: {:.3f} ms", measures_[i - 1].first, measures_[i].first, lap_ms);
      }
      report += fmt::format("\n  TOTAL: {:.3f} ms", total_ms);
      spdlog::info("[Lap Times]: seq {}, thres {} ms, {}", sequence_, threshold_ms, report);
    }
  }
};

