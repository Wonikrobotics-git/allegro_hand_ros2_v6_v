#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "v6_spec.hpp"

namespace allegro_hand_v6_hardware {

// One host-side read completion, including unsuccessful reads. No ROS clock is used.
struct CsvSample {
  int64_t unix_ns{0};
  uint64_t sequence{0};
  uint64_t dropped_total{0};
  bool read_ok{false};
  std::array<double, v6_sdk::JOINT_COUNT> positions{};
  std::array<int16_t, v6_sdk::JOINT_COUNT> currents_ma{};
  std::array<double, v6_sdk::PRESSURE_COUNT> pressures{};
};

// Single producer (hardware read loop), single consumer (file writer).
// push() performs no allocation, file I/O, mutex locking, or waiting.
class CsvLogger {
public:
  static_assert(std::atomic<uint64_t>::is_always_lock_free,
                "CSV queue requires lock-free 64-bit atomics");
  CsvLogger(const std::string& directory, const std::string& hand,
            const std::vector<std::string>& joint_names, std::size_t capacity = 8192);
  ~CsvLogger();
  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;

  bool push(CsvSample sample) noexcept;
  void stop(); // Call after stopping the producer; drains queued rows.
  const std::string& path() const { return path_; }
  uint64_t attempted() const { return attempted_.load(); }
  uint64_t written() const { return written_.load(); }
  uint64_t dropped() const { return dropped_.load(); }
  uint64_t failed_reads() const { return failed_reads_.load(); }
  bool write_failed() const { return write_failed_.load(); }

private:
  void run() noexcept;
  bool write_row(const CsvSample& sample);
  void fail(int error) noexcept;
  void save_summary() const;

  std::string path_;
  std::FILE* file_{nullptr};
  std::array<char, 65536> file_buffer_{};
  std::vector<CsvSample> queue_;
  std::atomic<uint64_t> head_{0}, tail_{0};
  std::atomic<uint64_t> attempted_{0}, written_{0}, dropped_{0}, failed_reads_{0};
  std::atomic<bool> stopping_{false}, write_failed_{false};
  std::atomic<int> io_errno_{0};
  std::thread writer_;
};

} // namespace allegro_hand_v6_hardware
