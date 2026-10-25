/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Wonik Robotics.
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
 * @file utility.hpp
 * @brief Defines various utility classes for the Allegro Hand, such as filters and controllers.
 * @author ('c')void
 */

#pragma once

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace allegro_hand_utility {

/**
 * @class SpinLock
 * @brief A simple, non-recursive spinlock for high-contention, short-duration critical sections.
 *
 * This implementation uses `std::atomic_flag` to provide a lightweight locking mechanism.
 * It is suitable for scenarios where the lock is held for a very short time, as it avoids
 * the overhead of context switching associated with mutexes by busy-waiting.
 */
class SpinLock {
  std::atomic_flag flag_;

public:
  /**
   * @brief Constructs a new SpinLock, initializing the atomic flag to a clear (unlocked) state.
   */
  SpinLock() : flag_(ATOMIC_FLAG_INIT) {}

  SpinLock(const SpinLock&) = delete; // Prohibit copy and move to ensure single ownership.
  SpinLock& operator=(const SpinLock&) = delete;

  /**
   * @brief Acquires the lock.
   *
   * This function busy-waits (spins) until the lock is acquired.
   * It uses `std::atomic_flag::test_and_set` with acquire memory ordering to ensure
   * that all memory operations before this point are visible to the current thread after the lock is acquired.
   */
  void lock();

  /**
   * @brief Releases the lock.
   *
   * It uses `std::atomic_flag::clear` with release memory ordering to ensure that all
   * memory operations before the unlock become visible to other threads that subsequently
   * acquire this lock.
   */
  void unlock();
};

/**
 * @class LowpassFilter
 * @brief A first-order low-pass filter implementation.
 * This filter is implemented using the Bilinear Transform method to discretize an analog filter.
 */
class LowpassFilter {
private:
  // Previous input value, x[n-1]
  double v_prv_;
  // Previous filtered output, y[n-1]
  double v_filtered_;

  // Filter coefficients
  double a1_, b0_, b1_;

public:
  /**
   * @brief Constructs a LowpassFilter object.
   * @param sampling_frequency The sampling frequency of the input signal in Hz.
   * @param cutoff_frequency The cutoff frequency of the filter in Hz.
   */
  explicit LowpassFilter(double sampling_frequency, double cutoff_frequency);

  /**
   * @brief Applies the low-pass filter to an input value.
   * @param v The raw input value to be filtered.
   * @return The filtered output value.
   */
  double filtering(double v);
};

/**
 * @class DerivativeFilter
 * @brief A simple derivative filter using the backward difference method.
 * It calculates the rate of change of a signal based on a fixed time step (dt).
 */
class DerivativeFilter {
  double v_prv;
  double v_dot;
  double dt_;

public:
  /**
   * @brief Constructs a DerivativeFilter object.
   * @param sampling_frequency The sampling frequency of the input signal in Hz, used to calculate the time step `dt`.
   */
  explicit DerivativeFilter(double sampling_frequency);

  /**
   * @brief Applies the derivative filter to an input value.
   * @param v The current input value.
   * @return The calculated derivative (rate of change).
   */
  double filtering(double v);
};

/**
 * @brief A thread-safe class to calculate and manage frequency and jitter statistics for multiple signals.
 *
 * This class tracks event occurrences for named signals. Call `tick()` to record an event
 * and `get_info()` to retrieve the current statistics. Frequency is the average event rate,
 * and jitter is the standard deviation of the event period.
 */
class SignalRateMonitor {
  /// @brief Internal state for tracking statistics of a single signal.
  struct SignalContext {
    std::chrono::high_resolution_clock::time_point last_tick_time;
    long long count = 0;
    double mean_period = 0.0;   // Average period in seconds.
    double m2 = 0.0;            // Sum of squares of differences from the mean, for variance calculation (Welford's algorithm).
    std::deque<double> periods; ///< Stores recent periods for moving window calculation.

    long long rtt_count = 0;
    double mean_rtt = 0.0;      // Average RTT in seconds.
    double rtt_m2 = 0.0;        // Sum of squares of differences from mean RTT.
    std::deque<double> rtts;    ///< Stores recent RTT values for moving window calculation.
  };

  /// @brief A hash map that maps signal names to their corresponding contexts.
  std::unordered_map<std::string, SignalContext> signals_;

  /// @brief A mutex to control concurrent access to the signals_ map.
  mutable SpinLock lock_;

  /// @brief The number of recent samples to use for statistics calculation.
  size_t window_size_;

public:
  /**
   * @brief A struct to hold the statistical information for a signal.
   */
  struct RateInfo {
    int count;
    double frequency; ///< Average frequency of the event in Hz.
    double jitter;    ///< Jitter (standard deviation of the period) in seconds.
    int rtt_count;    ///< Number of RTT samples recorded.
    double rtt_mean;  ///< Average RTT (round-trip time / latency) in seconds.
    double rtt_jitter;///< Jitter (standard deviation of RTT) in seconds.
    RateInfo()
        : count(0), frequency(0.0), jitter(0.0), rtt_count(0), rtt_mean(0.0), rtt_jitter(0.0) {}
  };

  /**
   * @brief Constructs a SignalRateMonitor.
   * @param window_size The number of recent samples to use for statistics. Defaults to 1000.
   */
  explicit SignalRateMonitor(size_t window_size = 1000);
  virtual ~SignalRateMonitor();

  // Prohibit copy and move to prevent unintended behavior.
  SignalRateMonitor(const SignalRateMonitor&) = delete;
  SignalRateMonitor& operator=(const SignalRateMonitor&) = delete;

  /**
   * @brief Records the occurrence of a named event (signal) using the current time.
   * @param signal_name The unique name of the signal.
   */
  void tick(std::string signal_name);

  /**
   * @brief Records the occurrence of a named event with explicit timestamp and RTT duration.
   * @param signal_name The unique name of the signal.
   * @param now Explicit time point of the event.
   * @param rtt Round-trip / elapsed duration of the transaction.
   */
  void tick(std::string signal_name,
            std::chrono::high_resolution_clock::time_point now,
            std::chrono::high_resolution_clock::duration rtt);

  /**
   * @brief Returns the current statistical information for all tracked signals.
   * @return A map where the key is the signal name and the value is the RateInfo.
   */
  std::map<std::string, RateInfo> get_info() const;

  std::string to_string() const;
};

} // namespace allegro_hand_utility