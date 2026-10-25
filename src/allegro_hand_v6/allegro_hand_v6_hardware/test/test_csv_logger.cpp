#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <sstream>
#include <thread>
#include <unistd.h>

#include "csv_logger.hpp"

using allegro_hand_v6_hardware::CsvLogger;
using allegro_hand_v6_hardware::CsvSample;

class CsvLoggerTest : public ::testing::Test {
protected:
  std::filesystem::path directory;
  std::vector<std::string> names;
  void SetUp() override {
    directory = std::filesystem::temp_directory_path() /
        ("allegro_csv_test_" + std::to_string(getpid()) + "_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    for (int i = 0; i < 20; ++i) names.push_back("joint" + std::to_string(i / 4) + std::to_string(i % 4));
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  static std::vector<std::string> fields(const std::string& row) {
    std::vector<std::string> result;
    std::stringstream input(row);
    std::string field;
    while (std::getline(input, field, ',')) result.push_back(field);
    if (!row.empty() && row.back() == ',') result.push_back("");
    return result;
  }
};

TEST_F(CsvLoggerTest, WallClockUnitsFailedReadAndDrain) {
  CsvLogger logger(directory.string(), "left", names);
  CsvSample sample;
  sample.unix_ns = 123456789; // Unix epoch + 0.123456789 seconds, rendered as KST.
  sample.read_ok = true;
  sample.positions[0] = 0.125;
  sample.currents_ma[0] = -123;
  sample.currents_ma[19] = 32767;
  sample.pressures[17] = 1013;
  ASSERT_TRUE(logger.push(sample));
  sample.unix_ns += 2000000;
  sample.read_ok = false;
  ASSERT_TRUE(logger.push(sample));
  logger.stop();
  logger.stop(); // Repeated lifecycle cleanup is harmless.
  EXPECT_EQ(logger.written(), 2u);
  EXPECT_EQ(logger.failed_reads(), 1u);
  EXPECT_EQ(logger.dropped(), 0u);
  EXPECT_FALSE(logger.write_failed());
  std::ifstream file(logger.path());
  std::string header, good, failed, extra;
  ASSERT_TRUE(static_cast<bool>(std::getline(file, header)));
  ASSERT_TRUE(static_cast<bool>(std::getline(file, good)));
  ASSERT_TRUE(static_cast<bool>(std::getline(file, failed)));
  EXPECT_FALSE(static_cast<bool>(std::getline(file, extra)));
  EXPECT_EQ(fields(header).size(), 62u);
  EXPECT_EQ(header.find("imu_"), std::string::npos);
  EXPECT_EQ(fields(header).back(), "pressure_17_hpa");
  EXPECT_EQ(header.find("timestamp_unix_ns"), std::string::npos);
  EXPECT_EQ(header.find("monotonic_ns"), std::string::npos);
  const auto values = fields(good);
  ASSERT_EQ(values.size(), 62u);
  EXPECT_EQ(values[0], "1970-01-01 09:00:00.123");
  EXPECT_EQ(values[1], "1");
  EXPECT_EQ(values[2], "1");
  EXPECT_EQ(values[4], "0.125");
  EXPECT_EQ(values[24], "-123");
  EXPECT_EQ(values[43], "32767");
  EXPECT_EQ(values[61], "1013");
  const auto blanks = fields(failed);
  ASSERT_EQ(blanks.size(), 62u);
  EXPECT_EQ(blanks[0], "1970-01-01 09:00:00.125");
  EXPECT_EQ(blanks[2], "0");
  for (std::size_t i = 4; i < blanks.size(); ++i) EXPECT_TRUE(blanks[i].empty());
  EXPECT_TRUE(std::filesystem::exists(logger.path() + ".summary.json"));
}

TEST_F(CsvLoggerTest, Sustained500HzAndImmediateShutdownPreserveAllSamples) {
  CsvLogger logger(directory.string(), "left", names);
  const auto start = std::chrono::steady_clock::now();
  CsvSample sample;
  sample.read_ok = true;
  for (int i = 0; i < 500; ++i) {
    sample.unix_ns = 1700000000000000000LL + i * 2000000LL;
    sample.positions[0] = i;
    EXPECT_TRUE(logger.push(sample));
    std::this_thread::sleep_until(start + std::chrono::microseconds((i + 1) * 2000));
  }
  // Drain a burst too, including the final row queued immediately before shutdown.
  for (int i = 0; i < 2000; ++i) EXPECT_TRUE(logger.push(sample));
  logger.stop();
  EXPECT_EQ(logger.attempted(), 2500u);
  EXPECT_EQ(logger.written(), 2500u);
  EXPECT_EQ(logger.dropped(), 0u);
  std::ifstream file(logger.path());
  std::string row;
  std::size_t count = 0;
  while (std::getline(file, row)) ++count;
  EXPECT_EQ(count, 2501u);
}

TEST_F(CsvLoggerTest, OverflowIsCountedAndExistingRecordingsArePreserved) {
  CsvLogger logger(directory.string(), "left", names, 1);
  CsvSample sample;
  for (int i = 0; i < 10000; ++i) logger.push(sample);
  logger.stop();
  EXPECT_GT(logger.dropped(), 0u);
  EXPECT_EQ(logger.written() + logger.dropped(), logger.attempted());
  CsvLogger another(directory.string(), "left", names);
  EXPECT_NE(logger.path(), another.path());
  EXPECT_TRUE(std::filesystem::exists(logger.path()));
}

TEST_F(CsvLoggerTest, InvalidOutputDirectoryFailsBeforeRecording) {
  std::filesystem::create_directories(directory);
  std::ofstream(directory / "not_a_directory") << "preserve";
  EXPECT_THROW(CsvLogger((directory / "not_a_directory" / "child").string(), "left", names),
               std::filesystem::filesystem_error);
}

TEST_F(CsvLoggerTest, DiskWriteFailureIsReported) {
  CsvLogger logger(directory.string(), "left", names);
  // Replace only this test logger's descriptor with Linux's ENOSPC device.
  int output_fd = -1;
  for (int fd = 3; fd < 1024; ++fd) {
    char target[4096];
    const auto count = readlink(("/proc/self/fd/" + std::to_string(fd)).c_str(), target, sizeof(target));
    if (count > 0 && std::string(target, count) == logger.path()) { output_fd = fd; break; }
  }
  ASSERT_GE(output_fd, 0);
  const int full = open("/dev/full", O_WRONLY);
  ASSERT_GE(full, 0);
  ASSERT_EQ(dup2(full, output_fd), output_fd);
  close(full);
  CsvSample sample;
  sample.read_ok = true;
  ASSERT_TRUE(logger.push(sample));
  logger.stop();
  EXPECT_TRUE(logger.write_failed());
  std::ifstream summary(logger.path() + ".summary.json");
  const std::string text((std::istreambuf_iterator<char>(summary)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("\"write_failed\": true"), std::string::npos);
}
