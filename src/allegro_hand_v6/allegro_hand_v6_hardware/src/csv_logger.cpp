#include "csv_logger.hpp"

#include <cerrno>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <system_error>

namespace allegro_hand_v6_hardware {
namespace {
int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

// KST is UTC+09:00, independent of the process's TZ and ROS /clock.
std::string timestamp(int64_t unix_ns, bool filename = false) {
  const auto seconds = static_cast<std::time_t>(unix_ns / 1000000000 + 9 * 3600);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::put_time(&tm, filename ? "%Y%m%d_%H%M%S" : "%Y-%m-%d %H:%M:%S")
      << (filename ? '_' : '.') << std::setfill('0') << std::setw(filename ? 9 : 3)
      << (filename ? unix_ns % 1000000000 : unix_ns % 1000000000 / 1000000);
  return out.str();
}

std::string csv_field(const std::string& value) {
  std::string result = "\"";
  for (const char ch : value) {
    if (ch == '"') result += '"';
    result += ch;
  }
  return result + '"';
}
} // namespace

CsvLogger::CsvLogger(const std::string& directory, const std::string& hand,
                     const std::vector<std::string>& joint_names, std::size_t capacity)
    : queue_(capacity) {
  if (capacity == 0 || joint_names.size() != v6_sdk::JOINT_COUNT) {
    throw std::invalid_argument("CSV logger needs a nonempty queue and 20 joint names");
  }
  if (directory.empty()) throw std::invalid_argument("csv_directory must not be empty");
  std::filesystem::create_directories(directory);
  path_ = (std::filesystem::absolute(directory) /
      ("allegro_hand_" + hand + "_" + timestamp(now_ns(), true) + ".csv")).string();
  // Exclusive creation avoids truncating an existing recording.
  file_ = std::fopen(path_.c_str(), "wx");
  if (!file_) throw std::system_error(errno, std::generic_category(), "Cannot create CSV " + path_);
  try {
    if (std::setvbuf(file_, file_buffer_.data(), _IOFBF, file_buffer_.size()) != 0) {
      throw std::runtime_error("Cannot configure CSV output buffer");
    }
    std::ostringstream header;
    header << "timestamp,sequence,read_ok,dropped_total";
    for (const auto& name : joint_names) header << ',' << csv_field(name + "_position_rad");
    for (const auto& name : joint_names) header << ',' << csv_field(name + "_current_ma");
    for (int i = 0; i < v6_sdk::PRESSURE_COUNT; ++i) header << ",pressure_" << i << "_hpa";
    header << '\n';
    const auto text = header.str();
    if (std::fwrite(text.data(), 1, text.size(), file_) != text.size() || std::fflush(file_) != 0) {
      throw std::system_error(errno ? errno : EIO, std::generic_category(), "Cannot write CSV header");
    }
    writer_ = std::thread(&CsvLogger::run, this);
  } catch (...) {
    std::fclose(file_);
    file_ = nullptr;
    throw;
  }
}

CsvLogger::~CsvLogger() { stop(); }

bool CsvLogger::push(CsvSample sample) noexcept {
  sample.sequence = attempted_.fetch_add(1, std::memory_order_relaxed) + 1;
  if (!sample.read_ok) failed_reads_.fetch_add(1, std::memory_order_relaxed);
  const auto head = head_.load(std::memory_order_relaxed);
  if (stopping_.load(std::memory_order_relaxed) || write_failed_.load(std::memory_order_relaxed) ||
      head - tail_.load(std::memory_order_acquire) >= queue_.size()) {
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  sample.dropped_total = dropped_.load(std::memory_order_relaxed);
  queue_[head % queue_.size()] = sample;
  head_.store(head + 1, std::memory_order_release);
  return true;
}

void CsvLogger::fail(int error) noexcept {
  io_errno_.store(error ? error : EIO);
  write_failed_.store(true);
}

bool CsvLogger::write_row(const CsvSample& sample) {
  std::ostringstream row;
  row.imbue(std::locale::classic());
  row << timestamp(sample.unix_ns) << ',' << sample.sequence
      << ',' << (sample.read_ok ? 1 : 0) << ',' << sample.dropped_total;
  row << std::setprecision(17);
  if (sample.read_ok) {
    for (const double value : sample.positions) row << ',' << value;
    for (const auto value : sample.currents_ma) row << ',' << value;
    for (const double value : sample.pressures) row << ',' << value;
  } else {
    // Do not present unchanged state buffers as fresh measurements after a read failure.
    row << std::string(v6_sdk::JOINT_COUNT * 2 + v6_sdk::PRESSURE_COUNT, ',');
  }
  row << '\n';
  const auto text = row.str();
  if (std::fwrite(text.data(), 1, text.size(), file_) != text.size()) {
    fail(errno);
    return false;
  }
  written_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void CsvLogger::run() noexcept {
  auto next_flush = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  try {
    while (!write_failed_.load()) {
      auto tail = tail_.load(std::memory_order_relaxed);
      const auto head = head_.load(std::memory_order_acquire);
      if (tail != head) {
        const auto sample = queue_[tail % queue_.size()];
        tail_.store(tail + 1, std::memory_order_release);
        if (!write_row(sample)) break;
      } else {
        // Recheck head after observing stop: the producer may have published
        // its last row between the first head load and the stop flag load.
        if (stopping_.load() && tail == head_.load(std::memory_order_acquire)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_flush) {
        if (std::fflush(file_) != 0) { fail(errno); break; }
        next_flush = now + std::chrono::seconds(1);
      }
    }
  } catch (...) {
    fail(EIO);
  }
}

void CsvLogger::save_summary() const {
  std::ofstream summary(path_ + ".summary.json");
  summary << "{\n  \"samples_attempted\": " << attempted()
          << ",\n  \"rows_written\": " << written()
          << ",\n  \"dropped_samples\": " << dropped()
          << ",\n  \"failed_reads\": " << failed_reads()
          << ",\n  \"write_failed\": " << (write_failed() ? "true" : "false")
          << ",\n  \"io_errno\": " << io_errno_.load() << "\n}\n";
}

void CsvLogger::stop() {
  stopping_.store(true);
  if (writer_.joinable()) writer_.join();
  if (file_) {
    if (std::fclose(file_) != 0) fail(errno);
    file_ = nullptr;
    save_summary();
  }
}

} // namespace allegro_hand_v6_hardware
