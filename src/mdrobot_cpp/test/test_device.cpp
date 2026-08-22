// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file test_device.cpp
/// SingleMotorDriver / DualMotorDriver unit tests — mirrors Python test_device.py.
///
/// FakeDevice auto-generates responses per function code so both sent frames
/// and read decoding are verified.

#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "mdrobot_cpp/crc.hpp"
#include "mdrobot_cpp/device.hpp"
#include "mdrobot_cpp/exceptions.hpp"
#include "mdrobot_cpp/frame.hpp"
#include "mdrobot_cpp/registers.hpp"

using namespace mdrobot;

static std::vector<uint8_t> with_crc(const std::vector<uint8_t>& body) {
  return append_crc(body.data(), body.size());
}

/// Fake transport that auto-generates responses like a real controller.
class FakeDevice : public Transport {
 public:
  explicit FakeDevice(
      const std::map<uint16_t, std::vector<uint16_t>>& registers = {})
      : registers_(registers) {}

  std::size_t write(const uint8_t* data, std::size_t len) override {
    auto req = std::vector<uint8_t>(data, data + len);
    frames.push_back(req);
    auto resp = respond(req);
    rx_.insert(rx_.end(), resp.begin(), resp.end());
    return len;
  }

  std::vector<uint8_t> read(std::size_t size) override {
    std::size_t n = std::min(size, rx_.size());
    std::vector<uint8_t> chunk(rx_.begin(), rx_.begin() + n);
    rx_.erase(rx_.begin(), rx_.begin() + n);
    return chunk;
  }

  void flush_input() override {}

  std::vector<std::vector<uint8_t>> frames;
  std::map<uint16_t, std::vector<uint16_t>> registers_;

 private:
  std::vector<uint8_t> rx_;

  std::vector<uint8_t> respond(const std::vector<uint8_t>& req) {
    uint8_t slave = req[0];
    uint8_t func = req[1];
    uint16_t pid = static_cast<uint16_t>((req[2] << 8) | req[3]);
    if (func == 0x03) {
      uint16_t count = static_cast<uint16_t>((req[4] << 8) | req[5]);
      auto it = registers_.find(pid);
      std::vector<uint16_t> words;
      if (it != registers_.end()) {
        words = it->second;
      }
      words.resize(count, 0);
      std::vector<uint8_t> body;
      body.push_back(slave);
      body.push_back(0x03);
      body.push_back(static_cast<uint8_t>(2 * count));
      for (auto w : words) {
        body.push_back(static_cast<uint8_t>((w >> 8) & 0xFF));
        body.push_back(static_cast<uint8_t>(w & 0xFF));
      }
      return with_crc(body);
    }
    if (func == 0x06) {
      return req;  // echo
    }
    if (func == 0x10) {
      auto body = std::vector<uint8_t>(req.begin(), req.begin() + 6);
      return with_crc(body);
    }
    return {};
  }
};

static std::vector<uint8_t> w1(uint16_t pid, uint16_t word) {
  return build_write_single_request(1, pid, word);
}

static std::vector<uint8_t> wN(uint16_t pid, const std::vector<uint16_t>& words) {
  return build_write_multiple_request(1, pid, words);
}

// --- shared / enable ---

TEST(Device, EnableWritesUicomComtarThenStart) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.enable();
  ASSERT_EQ(dev.frames.size(), 3u);
  EXPECT_EQ(dev.frames[0], w1(PID_UI_COM, 1));
  EXPECT_EQ(dev.frames[1], w1(PID_COM_TAR_SPEED, 0));
  EXPECT_EQ(dev.frames[2], w1(PID_START_STOP, 1));
}

TEST(Device, DisableClearsStartStop) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.disable();
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_START_STOP, 0));
}

TEST(Device, ResetAlarmUsesCommandGateway) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.reset_alarm();
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_COMMAND, CMD_ALARM_RESET));
}

TEST(Device, PingTrueFalse) {
  FakeDevice dev({{PID_VERSION, {0x2D}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_TRUE(drv.ping());
}

// --- SingleMotorDriver ---

TEST(Device, SingleSetVelocityPositive) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_velocity(100);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_VEL_CMD, 100));
}

TEST(Device, SingleSetVelocityNegativeTwosComplement) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_velocity(-100);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_VEL_CMD, 0xFF9C));
}

TEST(Device, SingleStopBrakeTorqueOff) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.stop();
  drv.brake();
  drv.torque_off();
  ASSERT_EQ(dev.frames.size(), 3u);
  EXPECT_EQ(dev.frames[0], w1(PID_VEL_CMD, 0));
  EXPECT_EQ(dev.frames[1], w1(PID_BRAKE, 0));
  EXPECT_EQ(dev.frames[2], w1(PID_TQ_OFF, 0));
}

TEST(Device, SingleGetSpeedSigned) {
  FakeDevice dev({{PID_INT_RPM_DATA, {0xFF9C}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_EQ(drv.get_speed(), -100);
}

TEST(Device, SingleGetVoltageAndCurrent) {
  FakeDevice dev({{PID_VOLT_IN, {239}}, {PID_TQ_DATA, {12}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_NEAR(drv.get_voltage(), 23.9, 0.01);
  EXPECT_NEAR(drv.get_current(), 1.2, 0.01);
}

TEST(Device, SingleGetPositionLong) {
  FakeDevice dev({{PID_POSI_DATA, {0x5678, 0x1234}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_EQ(drv.get_position(), 0x12345678);
}

TEST(Device, SingleReadMonitor) {
  FakeDevice dev({{PID_MONITOR, {0x0064, 0x000C, 0x0000, 0x0005, 0x0000, 0x0000}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  auto mon = drv.read_monitor();
  EXPECT_EQ(mon.speed_rpm, 100);
  EXPECT_EQ(mon.position, 5);
}

TEST(Device, SingleMoveToEncodesPositionLowWordFirstPlusSpeed) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.move_to(0x12345678, 50);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], wN(PID_POSI_VEL_CMD, {0x5678, 0x1234, 50}));
}

TEST(Device, SingleMoveToSmallTarget) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.move_to(80, 50);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], wN(PID_POSI_VEL_CMD, {80, 0, 50}));
}

TEST(Device, SingleMoveByNegativeDelta) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.move_by(-2, 40);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], wN(PID_INC_POSI_VEL_CMD, {0xFFFE, 0xFFFF, 40}));
}

TEST(Device, SingleGetInPosition) {
  FakeDevice dev1({{PID_IN_POSITION_OK, {1}}});
  ModbusClient client1(dev1, 1);
  SingleMotorDriver drv1(client1);
  EXPECT_TRUE(drv1.get_in_position());

  FakeDevice dev2({{PID_IN_POSITION_OK, {0}}});
  ModbusClient client2(dev2, 1);
  SingleMotorDriver drv2(client2);
  EXPECT_FALSE(drv2.get_in_position());
}

TEST(Device, SingleWaitInPositionTrueImmediate) {
  FakeDevice dev({{PID_IN_POSITION_OK, {1}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_TRUE(drv.wait_in_position(1.0, 0.01));
}

TEST(Device, SingleWaitInPositionTimeout) {
  FakeDevice dev({{PID_IN_POSITION_OK, {0}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_FALSE(drv.wait_in_position(0.05, 0.01));
}

// --- encoder velocity feedback ---
// settle = 0 keeps the tests fast; on hardware the default covers the controller
// reinitialising after the write.

TEST(Device, SingleGetEncoderPpr) {
  FakeDevice dev({{PID_ENC_PPR, {1000}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_EQ(drv.get_encoder_ppr(), 1000);
}

TEST(Device, SingleSetEncoderPprWritesThenVerifies) {
  // the fake keeps registers static, so seed the value the write is expected to land on
  FakeDevice dev({{PID_ENC_PPR, {1000}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_encoder_ppr(1000, 0.0);
  ASSERT_FALSE(dev.frames.empty());
  EXPECT_EQ(dev.frames[0], w1(PID_ENC_PPR, 1000));
}

TEST(Device, SingleDisableEncoderWritesZero) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.disable_encoder(0.0);
  ASSERT_FALSE(dev.frames.empty());
  EXPECT_EQ(dev.frames[0], w1(PID_ENC_PPR, 0));
}

TEST(Device, SingleSetEncoderPprThrowsWhenNotApplied) {
  FakeDevice dev;  // reads back 0, not the 1000 that was written
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_THROW(drv.set_encoder_ppr(1000, 0.0), ProtocolError);
}

TEST(Device, SingleSetEncoderPprCanSkipVerification) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_encoder_ppr(1000, 0.0, false);
  EXPECT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_ENC_PPR, 1000));
}

TEST(Device, SingleSetEncoderPprRejectsOutOfRange) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_THROW(drv.set_encoder_ppr(70000, 0.0), std::invalid_argument);
  EXPECT_TRUE(dev.frames.empty());
}

// --- encoder position source (USE_EPOSI) ---
// Enabling first reads ENC_PPR (the ENC_PPR=0 guard), so the 46 write is frames[1].

TEST(Device, SingleGetUseEncoderPosition) {
  FakeDevice dev({{PID_USE_EPOSI, {1}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_TRUE(drv.get_use_encoder_position());
  FakeDevice dev0;
  ModbusClient client0(dev0, 1);
  SingleMotorDriver drv0(client0);
  EXPECT_FALSE(drv0.get_use_encoder_position());
}

TEST(Device, SingleSetUseEncoderPositionWritesThenVerifies) {
  // the fake keeps registers static, so seed the value the write is expected to land on
  FakeDevice dev({{PID_USE_EPOSI, {1}}, {PID_ENC_PPR, {1000}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_use_encoder_position(true, 0.0);
  ASSERT_GE(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[1], w1(PID_USE_EPOSI, 1));
}

TEST(Device, SingleSetUseEncoderPositionFalseWritesZero) {
  FakeDevice dev;  // reads back 0 = the written value; no ENC_PPR guard read
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_use_encoder_position(false, 0.0);
  ASSERT_FALSE(dev.frames.empty());
  EXPECT_EQ(dev.frames[0], w1(PID_USE_EPOSI, 0));
}

TEST(Device, SingleSetUseEncoderPositionThrowsWhenNotApplied) {
  // read-back stays 0 although 1 was written (e.g. firmware ignoring the register)
  FakeDevice dev({{PID_ENC_PPR, {1000}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_THROW(drv.set_use_encoder_position(true, 0.0), ProtocolError);
}

TEST(Device, SingleSetUseEncoderPositionCanSkipVerification) {
  FakeDevice dev({{PID_ENC_PPR, {1000}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_use_encoder_position(true, 0.0, false);
  ASSERT_EQ(dev.frames.size(), 2u);  // guard read + write, no read-back
  EXPECT_EQ(dev.frames[1], w1(PID_USE_EPOSI, 1));
}

TEST(Device, SingleSetUseEncoderPositionRequiresEncoderPpr) {
  FakeDevice dev;  // ENC_PPR reads 0 -> refused before any write
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_THROW(drv.set_use_encoder_position(true, 0.0), ProtocolError);
  ASSERT_EQ(dev.frames.size(), 1u);  // only the ENC_PPR guard read went out
}

// --- DualMotorDriver ---

TEST(Device, DualSetVelocitiesUsesTwoLegacyPids) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.set_velocities(40, -40);
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_VEL_CMD, 40));
  EXPECT_EQ(dev.frames[1], w1(PID_VEL_CMD2, 0xFFD8));
}

TEST(Device, DualSetVelocityChannel) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.set_velocity(1, 30);
  drv.set_velocity(2, 30);
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_VEL_CMD, 30));
  EXPECT_EQ(dev.frames[1], w1(PID_VEL_CMD2, 30));
}

TEST(Device, DualSetVelocityBadChannel) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  EXPECT_THROW(drv.set_velocity(3, 10), std::invalid_argument);
}

TEST(Device, DualBrakeAndTorqueOffFlags) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.brake_both();
  drv.brake(1);
  drv.brake(2);
  drv.torque_off_both();
  drv.torque_off(1);
  drv.torque_off(2);
  ASSERT_EQ(dev.frames.size(), 6u);
  EXPECT_EQ(dev.frames[0], w1(PID_PNT_BRAKE, 0x0101));
  EXPECT_EQ(dev.frames[1], w1(PID_PNT_BRAKE, 0x0001));
  EXPECT_EQ(dev.frames[2], w1(PID_PNT_BRAKE, 0x0100));
  EXPECT_EQ(dev.frames[3], w1(PID_PNT_TQ_OFF, 0x0101));
  EXPECT_EQ(dev.frames[4], w1(PID_PNT_TQ_OFF, 0x0001));
  EXPECT_EQ(dev.frames[5], w1(PID_PNT_TQ_OFF, 0x0100));
}

TEST(Device, DualStopSendsBothZero) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.stop();
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_VEL_CMD, 0));
  EXPECT_EQ(dev.frames[1], w1(PID_VEL_CMD2, 0));
}

TEST(Device, DualReadMonitorAndGetSpeed) {
  FakeDevice dev({{PID_PNT_MONITOR,
                   {0x0064, 0x0005, 0x0000, 0xFFCE, 0xFFFF, 0xFFFF, 0x0000}}});
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  auto dm = drv.read_monitor();
  EXPECT_EQ(dm.motor1.speed_rpm, 100);
  EXPECT_EQ(dm.motor2.speed_rpm, -50);
  // get_speed reads monitor again from the same register values
  // but FakeDevice always serves the same data
  EXPECT_EQ(drv.get_speed(1), 100);
  EXPECT_EQ(drv.get_speed(2), -50);
}

TEST(Device, DualReadMainDataHasCurrent) {
  FakeDevice dev({{PID_PNT_MAIN_DATA,
                   {0x0064, 0x000C, 0x0000, 0x0000,
                    0x0000, 0x0006, 0x0000, 0x0000, 0x0000}}});
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  auto dm = drv.read_main_data();
  EXPECT_NEAR(dm.motor1.current_a.value(), 1.2, 0.01);
  EXPECT_NEAR(dm.motor2.current_a.value(), 0.6, 0.01);
}

TEST(Device, DualMoveToBothMatchesGoldenVector) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.move_to_both(0x12345678, 0x34567890, 0x9ABC, 0x0123);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0],
            wN(PID_PNT_POSI_VEL_CMD,
               {0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123}));
}

TEST(Device, DualMoveToBothDefaultSpeed2FollowsSpeed1) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.move_to_both(50, 80, 60);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], wN(PID_PNT_POSI_VEL_CMD, {50, 0, 60, 80, 0, 60}));
}

TEST(Device, DualMoveByBothNegativeDelta) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.move_by_both(20, -20, 50);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0],
            wN(PID_PNT_INC_POSI_VEL_CMD, {20, 0, 50, 0xFFEC, 0xFFFF, 50}));
}

TEST(Device, DualGetPositionsAndReset) {
  FakeDevice dev({{PID_PNT_MONITOR,
                   {0x0000, 0x0032, 0x0000, 0x0000, 0x0050, 0x0000, 0x0000}}});
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  auto [p1, p2] = drv.get_positions();
  EXPECT_EQ(p1, 50);
  EXPECT_EQ(p2, 80);
  EXPECT_EQ(drv.get_position(2), 80);
  drv.reset_position();
  EXPECT_EQ(dev.frames.back(), w1(PID_POSI_RESET, 0));
}

// --- slow-start / slow-down (NOT hardware-verified) ---

TEST(Device, SingleSetSlowStartFullScale) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_slow_start(15);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_SLOW_START, 1023));
}

TEST(Device, SingleSetSlowDownZero) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_slow_down(0);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0], w1(PID_SLOW_DOWN, 0));
}

TEST(Device, SingleGetSlowStartSeconds) {
  FakeDevice dev({{PID_SLOW_START, {1023}}});
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  EXPECT_NEAR(drv.get_slow_start(), 15.0, 0.01);
}

TEST(Device, SinglePositionSlowStartDown) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.set_position_slow_start(15);
  drv.set_position_slow_down(15);
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_POSI_SS, 1023));
  EXPECT_EQ(dev.frames[1], w1(PID_POSI_SD, 1023));
}

TEST(Device, SingleClearSlowUsesCommandGateway) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  SingleMotorDriver drv(client);
  drv.clear_slow_start();
  drv.clear_slow_down();
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_COMMAND, CMD_SLOW_START_OFF));
  EXPECT_EQ(dev.frames[1], w1(PID_COMMAND, CMD_SLOW_DOWN_OFF));
}

TEST(Device, DualSetSlowStartPerChannel) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.set_slow_start(1, 15);
  drv.set_slow_start(2, 0);
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_SLOW_START1, 1023));
  EXPECT_EQ(dev.frames[1], w1(PID_SLOW_START2, 0));
}

TEST(Device, DualSetSlowDownPerChannel) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.set_slow_down(1, 7.5);
  ASSERT_EQ(dev.frames.size(), 1u);
  EXPECT_EQ(dev.frames[0],
            w1(PID_SLOW_DOWN1, static_cast<uint16_t>(std::round(7.5 / 15 * 1023))));
}

TEST(Device, DualSlowBadChannel) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  EXPECT_THROW(drv.set_slow_start(3, 10), std::invalid_argument);
}

TEST(Device, DualGetSlowDownSeconds) {
  FakeDevice dev({{PID_SLOW_DOWN2, {1023}}});
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  EXPECT_NEAR(drv.get_slow_down(2), 15.0, 0.01);
}

TEST(Device, DualPositionSlowPerChannel) {
  FakeDevice dev;
  ModbusClient client(dev, 1);
  DualMotorDriver drv(client);
  drv.set_position_slow_start(1, 15);
  drv.set_position_slow_down(2, 15);
  ASSERT_EQ(dev.frames.size(), 2u);
  EXPECT_EQ(dev.frames[0], w1(PID_POSI_SS1, 1023));
  EXPECT_EQ(dev.frames[1], w1(PID_POSI_SD2, 1023));
}

// --- twin: N single drivers on ONE bus at DISTINCT slave IDs ----------------
// Underpins device_type=twin in mdrobot_ros2_control: one transport backs N
// ModbusClient with different slave_ids, each feeding a SingleMotorDriver. The
// frames must carry the per-client slave id so the two controllers don't collide.

TEST(Device, TwinTwoClientsOneTransportDistinctSlaveIds) {
  FakeDevice dev({{PID_VERSION, {0x2D}}});
  ModbusClient c1(dev, 1);
  ModbusClient c2(dev, 2);
  SingleMotorDriver d1(c1);
  SingleMotorDriver d2(c2);

  // Both controllers answer independently on the shared bus.
  EXPECT_TRUE(d1.ping());
  EXPECT_TRUE(d2.ping());

  // Commands are addressed to each client's own slave id.
  d1.set_velocity(40);
  d2.set_velocity(-40);

  ASSERT_EQ(dev.frames.size(), 4u);
  // ping = read PID_VERSION; verify the read requests carry the right slave id.
  EXPECT_EQ(dev.frames[0], build_read_request(1, PID_VERSION, 1));
  EXPECT_EQ(dev.frames[1], build_read_request(2, PID_VERSION, 1));
  // set_velocity writes carry the right slave id (40 / -40 two's complement).
  EXPECT_EQ(dev.frames[2], build_write_single_request(1, PID_VEL_CMD, 40));
  EXPECT_EQ(dev.frames[3], build_write_single_request(2, PID_VEL_CMD, 0xFFD8));
}

TEST(Device, TwinIndependentPositionAndUseLimitSwPerSlave) {
  FakeDevice dev;
  ModbusClient c1(dev, 1);
  ModbusClient c2(dev, 2);
  SingleMotorDriver d1(c1);
  SingleMotorDriver d2(c2);

  // USE_LIMIT_SW is written to each single controller's own slave id (no SW2).
  c1.write_register(PID_USE_LIMIT_SW, 0);
  c2.write_register(PID_USE_LIMIT_SW, 0);
  // Each wheel takes an independent absolute move on its own slave id.
  d1.move_to(80, 50);
  d2.move_to(-80, 50);

  ASSERT_EQ(dev.frames.size(), 4u);
  EXPECT_EQ(dev.frames[0], build_write_single_request(1, PID_USE_LIMIT_SW, 0));
  EXPECT_EQ(dev.frames[1], build_write_single_request(2, PID_USE_LIMIT_SW, 0));
  EXPECT_EQ(dev.frames[2],
            build_write_multiple_request(1, PID_POSI_VEL_CMD, {80, 0, 50}));
  EXPECT_EQ(dev.frames[3],
            build_write_multiple_request(2, PID_POSI_VEL_CMD, {0xFFB0, 0xFFFF, 50}));
}

// --- resolve_port / MDROBOT_PORT ------------------------------------------------------

TEST(ResolvePort, ExplicitWinsOverEnv) {
  ::setenv("MDROBOT_PORT", "/dev/from_env", 1);
  EXPECT_EQ(resolve_port("/dev/explicit"), "/dev/explicit");
  ::unsetenv("MDROBOT_PORT");
}

TEST(ResolvePort, FallsBackToEnv) {
  ::setenv("MDROBOT_PORT", "/dev/from_env", 1);
  EXPECT_EQ(resolve_port(), "/dev/from_env");
  EXPECT_EQ(resolve_port(""), "/dev/from_env");
  ::unsetenv("MDROBOT_PORT");
}

TEST(ResolvePort, MissingThrowsWithHint) {
  ::unsetenv("MDROBOT_PORT");
  try {
    resolve_port();
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("MDROBOT_PORT"), std::string::npos);
  }
}

TEST(ResolvePort, EnvIsTrimmedAndWhitespaceOnlyThrows) {
  ::setenv("MDROBOT_PORT", "  /dev/padded \n", 1);
  EXPECT_EQ(resolve_port(), "/dev/padded");
  ::setenv("MDROBOT_PORT", "   ", 1);
  EXPECT_THROW(resolve_port(), std::invalid_argument);
  ::unsetenv("MDROBOT_PORT");
}
