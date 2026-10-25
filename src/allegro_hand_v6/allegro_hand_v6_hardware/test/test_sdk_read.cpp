#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <modbus/modbus.h>

#include "v6_sdk.hpp"

namespace {
constexpr int fast_block_size = 62;
std::array<uint16_t, fast_block_size> feedback{};
int fast_result = fast_block_size;
int fast_requests = 0;
} // namespace

// Linker wrappers: exercise the real SDK decoder without opening a connection
// or sending any command to hardware.
extern "C" int __wrap_modbus_connect(modbus_t*) { return 0; }
extern "C" void __wrap_modbus_close(modbus_t*) {}
extern "C" int __wrap_modbus_write_bit(modbus_t*, int, int) { return 1; }
extern "C" int __wrap_modbus_read_registers(modbus_t*, int address, int count, uint16_t* dest) {
  if (address == static_cast<int>(v6_sdk::ModbusReg::GET_POS) && count == fast_block_size) {
    ++fast_requests;
    if (fast_result < 0) {
      errno = EIO;
      return -1;
    }
    std::copy_n(feedback.begin(), fast_result, dest);
    return fast_result;
  }
  // Firmware/hand information and serial number queried at startup.
  std::fill_n(dest, count, 0);
  return count;
}

class SdkReadTest : public ::testing::Test {
protected:
  void SetUp() override {
    feedback.fill(0);
    fast_result = fast_block_size;
    fast_requests = 0;
  }
};

TEST_F(SdkReadTest, OneFastBlockContainsAllCsvMeasurements) {
  for (int i = 0; i < v6_sdk::JOINT_COUNT; ++i) {
    feedback[i] = static_cast<uint16_t>(static_cast<int16_t>((i - 10) * 100));
    feedback[20 + i] = static_cast<uint16_t>(static_cast<int16_t>((i - 10) * 200));
  }
  feedback[20] = static_cast<uint16_t>(static_cast<int16_t>(-32768));
  feedback[39] = 32767;
  feedback[40] = 16384;
  feedback[41] = static_cast<uint16_t>(static_cast<int16_t>(-8192));
  feedback[42] = 4096;
  for (int i = 0; i < v6_sdk::PRESSURE_COUNT; ++i) feedback[44 + i] = 1000 + i;

  v6_sdk::HandV6ModBus device(v6_sdk::HandV6ModBus::Options{});
  std::array<double, 20> positions{}, torques{};
  std::array<double, 18> pressures{};
  std::array<double, 4> quaternion{};
  ASSERT_TRUE(device.read_states(positions.data(), nullptr, torques.data(),
                                 quaternion.data(), pressures.data()));
  EXPECT_EQ(fast_requests, 1);
  for (int i = 0; i < v6_sdk::JOINT_COUNT; ++i) {
    EXPECT_DOUBLE_EQ(positions[i], (i - 10) * 100 * v6_sdk::PUL2RAD);
    EXPECT_EQ(std::lround(torques[i] / v6_sdk::TORQUE_CONSTANT),
              static_cast<int16_t>(feedback[20 + i]));
  }
  for (int i = 0; i < v6_sdk::PRESSURE_COUNT; ++i) EXPECT_EQ(pressures[i], 1000 + i);
  EXPECT_EQ(quaternion, (std::array<double, 4>{1.0, -0.5, 0.25, 0.0}));
}

TEST_F(SdkReadTest, FailedAndShortReadsAreNotReportedAsFreshMeasurements) {
  v6_sdk::HandV6ModBus device(v6_sdk::HandV6ModBus::Options{});
  std::array<double, 20> positions, torques;
  std::array<double, 18> pressures;
  std::array<double, 4> quaternion;
  positions.fill(1.0);
  torques.fill(2.0);
  pressures.fill(3.0);
  quaternion.fill(4.0);
  for (const int result : {-1, fast_block_size - 1}) {
    fast_result = result;
    EXPECT_FALSE(device.read_states(positions.data(), nullptr, torques.data(),
                                    quaternion.data(), pressures.data()));
    for (double value : positions) EXPECT_EQ(value, 1.0);
    for (double value : torques) EXPECT_EQ(value, 2.0);
    for (double value : pressures) EXPECT_EQ(value, 3.0);
    for (double value : quaternion) EXPECT_EQ(value, 4.0);
  }
  EXPECT_EQ(fast_requests, 2);
  v6_sdk::HandV6Serial serial_placeholder;
  EXPECT_FALSE(serial_placeholder.read_states());
}
