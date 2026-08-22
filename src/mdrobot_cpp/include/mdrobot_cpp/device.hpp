// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file device.hpp
/// High-level device drivers: SingleMotorDriver / DualMotorDriver.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "mdrobot_cpp/protocol.hpp"
#include "mdrobot_cpp/status.hpp"
#include "mdrobot_cpp/transport.hpp"

namespace mdrobot {

/// Writing PID_ENC_PPR(156) can reinitialise the controller; wait this long before reading back.
constexpr double kEncoderSettleS = 2.5;

/// PID_USE_EPOSI(46) applies immediately, but the first read-back can still return the OLD
/// value; wait this long before each read-back attempt in set_use_encoder_position.
constexpr double kEposiSettleS = 0.2;

/// Shared base for single/dual: connection, version/voltage/status, enable/disable, alarm reset, slow.
class DriverBase {
 public:
  explicit DriverBase(ModbusClient& client);
  virtual ~DriverBase() = default;

  // --- shared reads ---
  int get_version();
  double get_voltage();
  StatusBits get_status();
  bool ping();

  // --- enable / safety ---
  void enable();
  void disable();
  void reset_alarm();

  // --- slow clear (global CMDs) ---
  void clear_slow_start();
  void clear_slow_down();
  void clear_position_slow_start();
  void clear_position_slow_down();

  ModbusClient& client() { return client_; }

 protected:
  void set_slow(uint16_t pid, double seconds, double full_scale_s);
  double get_slow(uint16_t pid, double full_scale_s);

  ModbusClient& client_;
};

/// Single-channel motor driver.
class SingleMotorDriver : public DriverBase {
 public:
  using DriverBase::DriverBase;

  void set_velocity(int rpm);
  void stop();
  void brake();
  void torque_off();
  void reset_position();

  int get_speed();
  double get_current();
  int32_t get_position();
  Monitor read_monitor();

  // --- position control ---
  void move_to(int32_t position, int speed = 100);
  void move_by(int32_t delta, int speed = 100);
  bool get_in_position();
  bool wait_in_position(double timeout = 10.0, double poll = 0.1);

  // --- encoder velocity feedback (hardware-verified 2026-08-04, 2x MD400 v8.6) ---
  /// Encoder pulses-per-rev setting; 0 = encoder off (hall closed-loop).
  int get_encoder_ppr();
  /// Use an attached encoder for velocity feedback; @p ppr is its rated pulses-per-rev.
  ///
  /// WARNING: the controller trusts this number as one revolution, so a mismatched value
  /// scales the real speed by (configured / actual). A too-large value makes the motor
  /// turn faster than commanded while the reported rpm still looks correct. The encoder
  /// must be wired: a nonzero PPR with no signal trips ENC_FAIL once the motor starts.
  /// Reported position stays on the hall counter regardless, so odometry counts/rev is
  /// unaffected. Stored in EEPROM. The write can reinitialise the controller, hence the
  /// settle delay plus read-back (@p verify).
  void set_encoder_ppr(int ppr, double settle = kEncoderSettleS, bool verify = true);
  /// Turn the encoder off (PPR = 0) and run hall closed-loop instead.
  void disable_encoder(double settle = kEncoderSettleS, bool verify = true);

  // --- encoder position source (USE_EPOSI, hardware-verified 2026-08-22, 2x MD400 v8.6) ---
  /// Whether reported position and position control use the encoder (PID_USE_EPOSI).
  bool get_use_encoder_position();
  /// Switch the position source between the hall counter (false, factory default) and an
  /// attached encoder (true). With the encoder source, position and position-control
  /// targets are in encoder counts: counts/rev = 4 x ENC_PPR (1000 PPR -> 4000/rev).
  ///
  /// WARNING (verified on MD400 v8.6): the physical +/- direction convention FLIPS versus
  /// hall mode (+ turned the verified motors CW where hall + is CCW) - remap signs in any
  /// layer that assumes the hall convention. The IN_POSITION_OK window is in counts, so
  /// arrival can take many seconds or never latch (always time-bound wait_in_position).
  /// Position is a 32-bit count and overflows ~133x sooner than hall mode. Switch only at
  /// standstill and call reset_position() afterwards. Takes effect immediately; EEPROM.
  /// Updates PID_POS_SEN_TYPE(26) automatically in both directions.
  ///
  /// Enabling requires a nonzero ENC_PPR (throws ProtocolError otherwise). The first
  /// read-back can return the old value, so @p verify retries (waiting @p settle before
  /// each attempt) and throws ProtocolError if the value never sticks.
  void set_use_encoder_position(bool enabled, double settle = kEposiSettleS, bool verify = true);

  // --- slow-start / slow-down (speed slow hardware-verified Phase 12; position slow doc-based) ---
  void set_slow_start(double seconds, double full_scale_s = 15.0);
  double get_slow_start(double full_scale_s = 15.0);
  void set_slow_down(double seconds, double full_scale_s = 15.0);
  double get_slow_down(double full_scale_s = 15.0);
  void set_position_slow_start(double seconds, double full_scale_s = 15.0);
  double get_position_slow_start(double full_scale_s = 15.0);
  void set_position_slow_down(double seconds, double full_scale_s = 15.0);
  double get_position_slow_down(double full_scale_s = 15.0);

 private:
  void write_posi_vel(uint16_t pid, int32_t position, int speed);
  int read_encoder_ppr_retrying();
};

/// Dual-channel motor driver. channel is 1 or 2.
class DualMotorDriver : public DriverBase {
 public:
  using DriverBase::DriverBase;

  void set_velocities(int rpm1, int rpm2);
  void set_velocity(int channel, int rpm);
  void stop();
  void stop_channel(int channel);

  void brake_both();
  void brake(int channel);
  void torque_off_both();
  void torque_off(int channel);

  DualMonitor read_monitor();
  DualMonitor read_main_data();
  int get_speed(int channel);
  double get_current(int channel);  ///< motor current (A) for channel (1 or 2).
  std::pair<int32_t, int32_t> get_positions();
  int32_t get_position(int channel);
  void reset_position();

  // --- position control ---
  void move_to_both(int32_t pos1, int32_t pos2, int speed1 = 100, int speed2 = -1);
  void move_by_both(int32_t delta1, int32_t delta2, int speed1 = 100, int speed2 = -1);

  // --- slow per channel (speed slow hardware-verified Phase 12; position slow doc-based) ---
  void set_slow_start(int channel, double seconds, double full_scale_s = 15.0);
  double get_slow_start(int channel, double full_scale_s = 15.0);
  void set_slow_down(int channel, double seconds, double full_scale_s = 15.0);
  double get_slow_down(int channel, double full_scale_s = 15.0);
  void set_position_slow_start(int channel, double seconds, double full_scale_s = 15.0);
  double get_position_slow_start(int channel, double full_scale_s = 15.0);
  void set_position_slow_down(int channel, double seconds, double full_scale_s = 15.0);
  double get_position_slow_down(int channel, double full_scale_s = 15.0);

 private:
  static uint16_t vel_pid(int channel);
  static uint16_t ch_flag_word(int channel);
  static uint16_t ch_pid(int channel, uint16_t pid1, uint16_t pid2);
  void write_pnt_posi_vel(uint16_t pid, int32_t pos1, int spd1, int32_t pos2, int spd2);
};

/// One-call owner of transport + client + driver (mirrors Python's
/// `SingleMotorDriver.open(...)`). Owns the serial port; closes it on
/// destruction (RAII). Access the driver via `->` or `driver()`.
///
///   auto conn = mdrobot::SingleMotorConnection::open("/dev/ttyUSB0");
///   conn->enable();
///   conn->set_velocity(40);
///
/// Non-copyable / non-movable (the driver references the owned client);
/// returned by value through C++17 guaranteed copy elision.
class SingleMotorConnection {
 public:
  /// With @p port empty/omitted, the MDROBOT_PORT environment variable is used
  /// (see resolve_port); throws std::invalid_argument when neither is set.
  static SingleMotorConnection open(const std::string& port = "", int baudrate = 19200,
                                    uint8_t slave_id = 1) {
    return SingleMotorConnection(resolve_port(port), baudrate, slave_id);
  }

  SingleMotorConnection(const SingleMotorConnection&) = delete;
  SingleMotorConnection& operator=(const SingleMotorConnection&) = delete;
  SingleMotorConnection(SingleMotorConnection&&) = delete;
  SingleMotorConnection& operator=(SingleMotorConnection&&) = delete;

  SingleMotorDriver& driver() { return driver_; }
  SingleMotorDriver* operator->() { return &driver_; }
  ModbusClient& client() { return client_; }

 private:
  SingleMotorConnection(const std::string& port, int baudrate, uint8_t slave_id)
      : transport_(std::make_unique<SerialTransport>(port, baudrate)),
        client_(*transport_, slave_id),
        driver_(client_) {}

  std::unique_ptr<SerialTransport> transport_;
  ModbusClient client_;
  SingleMotorDriver driver_;
};

/// One-call owner for a dual-channel controller. See SingleMotorConnection.
///
///   auto conn = mdrobot::DualMotorConnection::open("/dev/ttyUSB0");
///   conn->enable();
///   conn->set_velocities(40, 40);
class DualMotorConnection {
 public:
  /// With @p port empty/omitted, the MDROBOT_PORT environment variable is used
  /// (see resolve_port); throws std::invalid_argument when neither is set.
  static DualMotorConnection open(const std::string& port = "", int baudrate = 19200,
                                  uint8_t slave_id = 1) {
    return DualMotorConnection(resolve_port(port), baudrate, slave_id);
  }

  DualMotorConnection(const DualMotorConnection&) = delete;
  DualMotorConnection& operator=(const DualMotorConnection&) = delete;
  DualMotorConnection(DualMotorConnection&&) = delete;
  DualMotorConnection& operator=(DualMotorConnection&&) = delete;

  DualMotorDriver& driver() { return driver_; }
  DualMotorDriver* operator->() { return &driver_; }
  ModbusClient& client() { return client_; }

 private:
  DualMotorConnection(const std::string& port, int baudrate, uint8_t slave_id)
      : transport_(std::make_unique<SerialTransport>(port, baudrate)),
        client_(*transport_, slave_id),
        driver_(client_) {}

  std::unique_ptr<SerialTransport> transport_;
  ModbusClient client_;
  DualMotorDriver driver_;
};

}  // namespace mdrobot
