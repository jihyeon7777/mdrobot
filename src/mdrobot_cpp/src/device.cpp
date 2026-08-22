// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/device.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include "mdrobot_cpp/codec.hpp"
#include "mdrobot_cpp/exceptions.hpp"
#include "mdrobot_cpp/registers.hpp"
#include "mdrobot_cpp/units.hpp"

namespace mdrobot {

// PID_UI_COM(78) value: serial-only control.
static constexpr uint16_t UI_COM_SERIAL = 1;
// PID_START_STOP values.
static constexpr uint16_t START = 1;
static constexpr uint16_t STOP_VAL = 0;

// ==========================================================================
// DriverBase
// ==========================================================================

DriverBase::DriverBase(ModbusClient& client) : client_(client) {}

int DriverBase::get_version() {
  return client_.read_register(PID_VERSION) & 0xFF;
}

double DriverBase::get_voltage() {
  return client_.read_register(PID_VOLT_IN) / 10.0;
}

StatusBits DriverBase::get_status() {
  return StatusBits::from_byte(
      static_cast<uint8_t>(client_.read_register(PID_CTRL_STATUS) & 0xFF));
}

bool DriverBase::ping() {
  try {
    get_version();
    return true;
  } catch (const MdrobotError&) {
    return false;
  }
}

void DriverBase::enable() {
  client_.write_register(PID_UI_COM, UI_COM_SERIAL);
  client_.write_register(PID_COM_TAR_SPEED, 0);
  client_.write_register(PID_START_STOP, START);
}

void DriverBase::disable() {
  client_.write_register(PID_START_STOP, STOP_VAL);
}

void DriverBase::reset_alarm() {
  client_.command(CMD_ALARM_RESET);
}

void DriverBase::set_slow(uint16_t pid, double seconds, double full_scale_s) {
  client_.write_register(pid, static_cast<uint16_t>(slow_seconds_to_raw(seconds, full_scale_s)));
}

double DriverBase::get_slow(uint16_t pid, double full_scale_s) {
  return slow_raw_to_seconds(client_.read_register(pid), full_scale_s);
}

void DriverBase::clear_slow_start() { client_.command(CMD_SLOW_START_OFF); }
void DriverBase::clear_slow_down() { client_.command(CMD_SLOW_DOWN_OFF); }
void DriverBase::clear_position_slow_start() { client_.command(CMD_POSI_SS_OFF); }
void DriverBase::clear_position_slow_down() { client_.command(CMD_POSI_SD_OFF); }

// ==========================================================================
// SingleMotorDriver
// ==========================================================================

void SingleMotorDriver::set_velocity(int rpm) {
  client_.write_register(PID_VEL_CMD, word_from_int16(static_cast<int16_t>(rpm)));
}

void SingleMotorDriver::stop() {
  client_.write_register(PID_VEL_CMD, 0);
}

void SingleMotorDriver::brake() {
  client_.write_register(PID_BRAKE, 0);
}

void SingleMotorDriver::torque_off() {
  client_.write_register(PID_TQ_OFF, 0);
}

void SingleMotorDriver::reset_position() {
  client_.write_register(PID_POSI_RESET, 0);
}

int SingleMotorDriver::get_speed() {
  return to_int16(client_.read_register(PID_INT_RPM_DATA));
}

double SingleMotorDriver::get_current() {
  return client_.read_register(PID_TQ_DATA) / 10.0;
}

int32_t SingleMotorDriver::get_position() {
  return client_.read_long(PID_POSI_DATA);
}

Monitor SingleMotorDriver::read_monitor() {
  return decode_monitor(client_.read_registers(PID_MONITOR, 6));
}

void SingleMotorDriver::write_posi_vel(uint16_t pid, int32_t position, int speed) {
  auto [low, high] = split_i32_low_word_first(position);
  client_.write_registers(pid, {low, high, static_cast<uint16_t>(speed & 0xFFFF)});
}

void SingleMotorDriver::move_to(int32_t position, int speed) {
  write_posi_vel(PID_POSI_VEL_CMD, position, speed);
}

void SingleMotorDriver::move_by(int32_t delta, int speed) {
  write_posi_vel(PID_INC_POSI_VEL_CMD, delta, speed);
}

bool SingleMotorDriver::get_in_position() {
  return client_.read_register(PID_IN_POSITION_OK) != 0;
}

bool SingleMotorDriver::wait_in_position(double timeout, double poll) {
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    if (get_in_position()) return true;
    std::this_thread::sleep_for(std::chrono::duration<double>(poll));
  }
  return false;
}

// --- encoder velocity feedback ---
// Hardware-verified 2026-08-04 (2x MD400 v8.6, 1000 PPR encoders wired, one bus): an
// attached encoder feeds the VELOCITY closed loop only. Reported position stays on the
// hall counter (counts/rev = 3 x poles) whether the encoder is on or off.
namespace {
constexpr int kEncoderReadTries = 4;
constexpr double kEncoderReadRetryS = 0.3;
}  // namespace

int SingleMotorDriver::get_encoder_ppr() {
  return client_.read_register(PID_ENC_PPR);
}

void SingleMotorDriver::set_encoder_ppr(int ppr, double settle, bool verify) {
  if (ppr < 0 || ppr > 0xFFFF) {
    throw std::invalid_argument("encoder PPR must be 0..65535, got " + std::to_string(ppr));
  }
  try {
    client_.write_register(PID_ENC_PPR, static_cast<uint16_t>(ppr));
  } catch (const MdrobotError&) {
    if (!verify) throw;
    // Reinitialisation can swallow the response; the read-back below decides.
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(settle));
  if (!verify) return;
  const int readback = read_encoder_ppr_retrying();
  if (readback != ppr) {
    throw ProtocolError("PID_ENC_PPR not applied: wrote " + std::to_string(ppr) +
                        ", read back " + std::to_string(readback));
  }
}

void SingleMotorDriver::disable_encoder(double settle, bool verify) {
  set_encoder_ppr(0, settle, verify);
}

int SingleMotorDriver::read_encoder_ppr_retrying() {
  for (int attempt = 0; attempt < kEncoderReadTries; ++attempt) {
    try {
      return get_encoder_ppr();
    } catch (const MdrobotError&) {
      if (attempt == kEncoderReadTries - 1) throw;
      std::this_thread::sleep_for(std::chrono::duration<double>(kEncoderReadRetryS));
    }
  }
  throw ProtocolError("unreachable");  // loop above always returns or throws
}

// --- encoder position source (USE_EPOSI) ---
// Hardware-verified 2026-08-22 (2x MD400 v8.6, 1000 PPR encoders): PID_USE_EPOSI(46)
// switches reported position AND position control onto the encoder counter
// (counts/rev = 4 x ENC_PPR) and flips the physical +/- direction vs hall mode.
namespace {
constexpr int kEposiVerifyTries = 5;
}  // namespace

bool SingleMotorDriver::get_use_encoder_position() {
  return client_.read_register(PID_USE_EPOSI) != 0;
}

void SingleMotorDriver::set_use_encoder_position(bool enabled, double settle, bool verify) {
  if (enabled && get_encoder_ppr() == 0) {
    throw ProtocolError(
        "cannot enable encoder position with ENC_PPR = 0 - "
        "call set_encoder_ppr() with the encoder's rated PPR first");
  }
  const uint16_t value = enabled ? 1 : 0;
  client_.write_register(PID_USE_EPOSI, value);
  if (!verify) return;
  uint16_t readback = 0;
  for (int attempt = 0; attempt < kEposiVerifyTries; ++attempt) {
    std::this_thread::sleep_for(std::chrono::duration<double>(settle));
    readback = client_.read_register(PID_USE_EPOSI);
    if (readback == value) return;
  }
  throw ProtocolError("PID_USE_EPOSI not applied: wrote " + std::to_string(value) +
                      ", read back " + std::to_string(readback));
}

void SingleMotorDriver::set_slow_start(double s, double fs) { set_slow(PID_SLOW_START, s, fs); }
double SingleMotorDriver::get_slow_start(double fs) { return get_slow(PID_SLOW_START, fs); }
void SingleMotorDriver::set_slow_down(double s, double fs) { set_slow(PID_SLOW_DOWN, s, fs); }
double SingleMotorDriver::get_slow_down(double fs) { return get_slow(PID_SLOW_DOWN, fs); }
void SingleMotorDriver::set_position_slow_start(double s, double fs) { set_slow(PID_POSI_SS, s, fs); }
double SingleMotorDriver::get_position_slow_start(double fs) { return get_slow(PID_POSI_SS, fs); }
void SingleMotorDriver::set_position_slow_down(double s, double fs) { set_slow(PID_POSI_SD, s, fs); }
double SingleMotorDriver::get_position_slow_down(double fs) { return get_slow(PID_POSI_SD, fs); }

// ==========================================================================
// DualMotorDriver
// ==========================================================================

uint16_t DualMotorDriver::vel_pid(int channel) {
  if (channel == 1) return PID_VEL_CMD;
  if (channel == 2) return PID_VEL_CMD2;
  throw std::invalid_argument("channel must be 1 or 2, got " + std::to_string(channel));
}

uint16_t DualMotorDriver::ch_flag_word(int channel) {
  if (channel == 1) return 0x0001;
  if (channel == 2) return 0x0100;
  throw std::invalid_argument("channel must be 1 or 2, got " + std::to_string(channel));
}

uint16_t DualMotorDriver::ch_pid(int channel, uint16_t pid1, uint16_t pid2) {
  if (channel == 1) return pid1;
  if (channel == 2) return pid2;
  throw std::invalid_argument("channel must be 1 or 2, got " + std::to_string(channel));
}

void DualMotorDriver::set_velocities(int rpm1, int rpm2) {
  client_.write_register(PID_VEL_CMD, word_from_int16(static_cast<int16_t>(rpm1)));
  client_.write_register(PID_VEL_CMD2, word_from_int16(static_cast<int16_t>(rpm2)));
}

void DualMotorDriver::set_velocity(int channel, int rpm) {
  client_.write_register(vel_pid(channel), word_from_int16(static_cast<int16_t>(rpm)));
}

void DualMotorDriver::stop() {
  set_velocities(0, 0);
}

void DualMotorDriver::stop_channel(int channel) {
  client_.write_register(vel_pid(channel), 0);
}

void DualMotorDriver::brake_both() {
  client_.write_register(PID_PNT_BRAKE, 0x0101);
}

void DualMotorDriver::brake(int channel) {
  client_.write_register(PID_PNT_BRAKE, ch_flag_word(channel));
}

void DualMotorDriver::torque_off_both() {
  client_.write_register(PID_PNT_TQ_OFF, 0x0101);
}

void DualMotorDriver::torque_off(int channel) {
  client_.write_register(PID_PNT_TQ_OFF, ch_flag_word(channel));
}

DualMonitor DualMotorDriver::read_monitor() {
  return decode_pnt_monitor(client_.read_registers(PID_PNT_MONITOR, 7));
}

DualMonitor DualMotorDriver::read_main_data() {
  return decode_pnt_main_data(client_.read_registers(PID_PNT_MAIN_DATA, 9));
}

int DualMotorDriver::get_speed(int channel) {
  auto mon = read_monitor();
  if (channel == 1) return mon.motor1.speed_rpm;
  if (channel == 2) return mon.motor2.speed_rpm;
  throw std::invalid_argument("channel must be 1 or 2, got " + std::to_string(channel));
}

double DualMotorDriver::get_current(int channel) {
  // current lives in PNT_MAIN_DATA (PNT_MONITOR has none).
  auto mon = read_main_data();
  if (channel == 1) return mon.motor1.current_a.value_or(0.0);
  if (channel == 2) return mon.motor2.current_a.value_or(0.0);
  throw std::invalid_argument("channel must be 1 or 2, got " + std::to_string(channel));
}

std::pair<int32_t, int32_t> DualMotorDriver::get_positions() {
  auto mon = read_monitor();
  return {mon.motor1.position, mon.motor2.position};
}

int32_t DualMotorDriver::get_position(int channel) {
  auto [p1, p2] = get_positions();
  if (channel == 1) return p1;
  if (channel == 2) return p2;
  throw std::invalid_argument("channel must be 1 or 2, got " + std::to_string(channel));
}

void DualMotorDriver::reset_position() {
  client_.write_register(PID_POSI_RESET, 0);
}

void DualMotorDriver::write_pnt_posi_vel(uint16_t pid, int32_t pos1, int spd1,
                                          int32_t pos2, int spd2) {
  auto [l1, h1] = split_i32_low_word_first(pos1);
  auto [l2, h2] = split_i32_low_word_first(pos2);
  client_.write_registers(pid, {l1, h1, static_cast<uint16_t>(spd1 & 0xFFFF),
                                l2, h2, static_cast<uint16_t>(spd2 & 0xFFFF)});
}

void DualMotorDriver::move_to_both(int32_t pos1, int32_t pos2, int speed1, int speed2) {
  write_pnt_posi_vel(PID_PNT_POSI_VEL_CMD, pos1, speed1, pos2,
                     speed2 < 0 ? speed1 : speed2);
}

void DualMotorDriver::move_by_both(int32_t delta1, int32_t delta2, int speed1, int speed2) {
  write_pnt_posi_vel(PID_PNT_INC_POSI_VEL_CMD, delta1, speed1, delta2,
                     speed2 < 0 ? speed1 : speed2);
}

void DualMotorDriver::set_slow_start(int ch, double s, double fs) {
  set_slow(ch_pid(ch, PID_SLOW_START1, PID_SLOW_START2), s, fs);
}
double DualMotorDriver::get_slow_start(int ch, double fs) {
  return get_slow(ch_pid(ch, PID_SLOW_START1, PID_SLOW_START2), fs);
}
void DualMotorDriver::set_slow_down(int ch, double s, double fs) {
  set_slow(ch_pid(ch, PID_SLOW_DOWN1, PID_SLOW_DOWN2), s, fs);
}
double DualMotorDriver::get_slow_down(int ch, double fs) {
  return get_slow(ch_pid(ch, PID_SLOW_DOWN1, PID_SLOW_DOWN2), fs);
}
void DualMotorDriver::set_position_slow_start(int ch, double s, double fs) {
  set_slow(ch_pid(ch, PID_POSI_SS1, PID_POSI_SS2), s, fs);
}
double DualMotorDriver::get_position_slow_start(int ch, double fs) {
  return get_slow(ch_pid(ch, PID_POSI_SS1, PID_POSI_SS2), fs);
}
void DualMotorDriver::set_position_slow_down(int ch, double s, double fs) {
  set_slow(ch_pid(ch, PID_POSI_SD1, PID_POSI_SD2), s, fs);
}
double DualMotorDriver::get_position_slow_down(int ch, double fs) {
  return get_slow(ch_pid(ch, PID_POSI_SD1, PID_POSI_SD2), fs);
}

}  // namespace mdrobot
