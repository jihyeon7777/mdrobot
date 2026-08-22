# C++ Library Usage (`mdrobot_cpp`)

`mdrobot_cpp` is a C++17 RS485 / Modbus RTU driver for MDROBOT MD-series motor
controllers — a 1:1 port of the Python [`mdrobot`](python.md) library (POSIX
`termios` transport, CRC, Modbus RTU, registers, status decoding, unit
conversion, single/dual drivers). `ament_cmake`, no ROS dependency. The
[`ros2_control` plugin](ros2_control.md) is built on top of it.

## Build & link

```bash
colcon build --packages-select mdrobot_cpp
```

In your package's `CMakeLists.txt`:

```cmake
find_package(mdrobot_cpp REQUIRED)
ament_target_dependencies(your_target mdrobot_cpp)
```

`#include "mdrobot_cpp/device.hpp"`. Everything lives in namespace `mdrobot`.
Unit tests (golden Modbus vectors, decoders, units):
`colcon test --packages-select mdrobot_cpp`.

## Quick start

The `*Connection::open()` factories build transport + client + driver in one call
and close the port on destruction (RAII) — the easiest entry point. **Pass your
serial port**, e.g. `open("/dev/ttyUSB0")` (the examples below do). *Optional:* with
the port argument omitted it falls back to the `MDROBOT_PORT` environment variable
(an explicit port always wins; neither set → `std::invalid_argument`) — see
[Port setup](setup/port-setup.md):

```cpp
#include "mdrobot_cpp/device.hpp"
#include <iostream>          // std::cout
#include <thread>            // std::this_thread::sleep_for
#include <chrono>            // std::chrono::seconds (for the dwell below)

// single-channel — read-only first (no motion): confirm comms
auto s = mdrobot::SingleMotorConnection::open("/dev/ttyUSB0");  // your serial port; 19200, id 1
std::cout << s->get_version() << " " << s->get_voltage() << " V\n";

// drive (the motor turns)
s->enable();                 // REQUIRED before motion
s->set_velocity(40);         // signed rpm; + = CCW
std::this_thread::sleep_for(std::chrono::seconds(2));  // hold so it actually turns
auto m = s->read_monitor();  // m.speed_rpm, m.position, m.current_a
s->stop();
s->torque_off();             // NOTE: destruction closes the port but does NOT stop a moving motor

// dual-channel
auto d = mdrobot::DualMotorConnection::open("/dev/ttyUSB0");
d->enable();
d->set_velocities(40, 40);
std::this_thread::sleep_for(std::chrono::seconds(2));  // hold so they actually turn
d->stop();
d->torque_off_both();        // NOTE: destruction does NOT stop a moving motor
```

`->` forwards to the driver; `s.driver()` / `s.client()` give direct references.

> **`enable()` is required before motion** (`UI_COM = 1` + arm `START/STOP`).
> `+` = CCW = increasing position. Some dual controllers start ~1 s after the
> command.

### Manual construction (shared bus / custom transport)

```cpp
mdrobot::SerialTransport transport(mdrobot::resolve_port(), 19200);  // or an explicit port string
mdrobot::ModbusClient client(transport, /*slave_id=*/1);
mdrobot::SingleMotorDriver drv(client);   // drv references client; keep both alive
```

> **Object lifetime:** a driver holds a `ModbusClient&` and the client holds a
> `Transport&`. When you build them by hand, the transport must outlive the
> client and the client must outlive the driver. The `*Connection` wrappers own
> all three for you (and are non-copyable / non-movable for that reason).

---

# API reference

`int rpm` is signed mechanical rpm. `int32_t position` is an encoder/hall count
(`+` = CCW). `speed` for position moves is the max rpm magnitude.

## Construction

| Symbol | Description |
|---|---|
| `SerialTransport(port, baudrate=19200, timeout=0.3, settle=0.2, write_timeout=1.0)` | Open a POSIX serial port (8N1). Throws `std::runtime_error` on failure; closes on destruction. |
| `ModbusClient(Transport& t, uint8_t slave_id=1)` | Modbus RTU client over a transport. |
| `SingleMotorConnection::open(port="", baudrate=19200, slave_id=1)` | → `SingleMotorConnection` owning transport+client+driver. Port empty/omitted → `$MDROBOT_PORT` ([Port setup](setup/port-setup.md)). |
| `DualMotorConnection::open(port="", baudrate=19200, slave_id=1)` | → `DualMotorConnection` (dual). |
| `resolve_port(port="")` | The port-defaulting rule itself: explicit arg, else `$MDROBOT_PORT`, else throws. |
| `conn->`, `conn.driver()`, `conn.client()` | Access the driver / client held by a connection. |

## Shared driver methods (`DriverBase`)

| Method | Returns | Description |
|---|---|---|
| `ping()` | `bool` | `True` if the controller answers. |
| `get_version()` | `int` | Firmware/DL version. |
| `get_voltage()` | `double` | Input voltage (V). |
| `get_status()` | `StatusBits` | Status-1 bits. |
| `enable()` | `void` | Allow motion: `UI_COM = 1` + arm `START/STOP`. |
| `disable()` | `void` | Release the run-latch (`START_STOP = 0`); a continuously-driven motor stops. |
| `reset_alarm()` | `void` | Clear a latched alarm. |
| `clear_slow_start/down()`, `clear_position_slow_start/down()` | `void` | Erase ramp settings. |

## `SingleMotorDriver`

| Method | Returns | Description |
|---|---|---|
| `set_velocity(int rpm)` | `void` | Drive at signed rpm (`0` stops). |
| `stop()` / `brake()` / `torque_off()` | `void` | Controlled stop / short-brake / release torque. |
| `get_speed()` | `int` | Measured speed (rpm). |
| `get_current()` | `double` | Motor current (A). |
| `get_position()` | `int32_t` | Position count. |
| `read_monitor()` | `Monitor` | Speed + current + output + position. |
| `reset_position()` | `void` | Zero the position counter. |
| `move_to(int32_t position, int speed=100)` | `void` | Absolute move (counts) at ≤ speed rpm. |
| `move_by(int32_t delta, int speed=100)` | `void` | Relative move (counts). |
| `get_in_position()` | `bool` | Last move arrived? |
| `wait_in_position(double timeout=10.0, double poll=0.1)` | `bool` | Block until in-position; `true` if arrived. |
| `set_slow_start(double s, double full_scale_s=15.0)` … | `void` / `double` | Speed/position ramp setters & getters (see Python manual). |
| `get_encoder_ppr()` | `int` | Encoder PPR setting; `0` = encoder off (hall closed-loop). |
| `set_encoder_ppr(int ppr, double settle=2.5, bool verify=true)` | `void` | Use the encoder; `ppr` is its **rated** pulses-per-rev. |
| `disable_encoder(double settle=2.5, bool verify=true)` | `void` | Turn the encoder off (`ppr = 0`). |
| `get_use_encoder_position()` | `bool` | Position source: `true` = encoder counts, `false` = hall counts (default). |
| `set_use_encoder_position(bool enabled, double settle=0.2, bool verify=true)` | `void` | Switch the position source (needs a nonzero `ENC_PPR`). |

By default an encoder improves the **velocity** loop only; `set_use_encoder_position(true)`
additionally switches position onto the encoder (counts/rev = 4 × PPR). Before using
either, read **[Using an encoder](encoder.md)**: a too-large `ppr` runs faster than
commanded with no software symptom, and the encoder position source flips the physical
sign convention.

## `DualMotorDriver`

`channel` is `1` or `2`. `set_velocities` writes each motor on its own register.

> **Bare `brake(int)` / `torque_off(int)` act on one channel; `stop()` acts on both.**
> `d->brake(1)` brakes motor 1 while **motor 2 keeps running**. Use `stop()`,
> `brake_both()` or `torque_off_both()` for both.

| Method | Returns | Description |
|---|---|---|
| `set_velocities(int rpm1, int rpm2)` | `void` | Set both motors. |
| `set_velocity(int channel, int rpm)` | `void` | Set one motor. |
| `stop()` / `stop_channel(int)` | `void` | Stop both / one. |
| `brake_both()` / `brake(int)` | `void` | Short-brake both / one. |
| `torque_off_both()` / `torque_off(int)` | `void` | Release both / one. |
| `get_speed(int channel)` | `int` | Channel speed (rpm). |
| `get_current(int channel)` | `double` | Channel current (A), from `PNT_MAIN_DATA`. |
| `get_position(int channel)` | `int32_t` | Channel position count. |
| `get_positions()` | `std::pair<int32_t,int32_t>` | Both counts. |
| `read_monitor()` | `DualMonitor` | Speed + position both (no current). |
| `read_main_data()` | `DualMonitor` | Speed + **current** + position both. |
| `reset_position()` | `void` | Zero both counters. |
| `move_to_both(int32_t p1, int32_t p2, int speed1=100, int speed2=-1)` | `void` | Absolute move both (`speed2<0` reuses `speed1`). |
| `move_by_both(int32_t d1, int32_t d2, int speed1=100, int speed2=-1)` | `void` | Relative move both. |
| `set_slow_start(int channel, double s, double full_scale_s=15.0)` … | `void` / `double` | Per-channel ramp setters & getters. |

```cpp
auto d = mdrobot::DualMotorConnection::open("/dev/ttyUSB0");
d->enable();
d->set_velocities(30, -30);
std::cout << d->get_current(1) << " " << d->get_current(2) << " A\n";
d->stop();
d->torque_off_both();
```

## Raw register access (`ModbusClient`)

| Method | Returns | Description |
|---|---|---|
| `read_register(uint16_t pid)` | `uint16_t` | Read one register. |
| `read_registers(uint16_t pid, uint16_t count)` | `std::vector<uint16_t>` | Read `count` registers. |
| `write_register(uint16_t pid, uint16_t word)` | `void` | Write one register. |
| `write_registers(uint16_t pid, const std::vector<uint16_t>&)` | `void` | Write consecutive registers. |
| `read_long(uint16_t pid, bool is_signed=true)` | `int32_t` | Read an INT32 (low word first). |
| `write_long(uint16_t pid, int32_t value)` | `void` | Write an INT32. |
| `command(uint16_t cmd)` | `void` | Issue a command code. |

PIDs/commands are constants in `mdrobot_cpp/registers.hpp`, e.g.
`client.write_register(mdrobot::PID_USE_LIMIT_SW, 0);`. A full table of register
numbers, command codes and status bits is in the
**[Register reference](reference/registers.md)**. Here **PID** means *parameter ID* (a register
address), not a control gain.

## Data types

`StatusBits`: `bool alarm, ctrl_fail, over_voltage, over_temperature, overload,
hall_or_encoder_fail, inverse_velocity, stall;` + `uint8_t raw`.

`Monitor` (and `DualMonitor::motor1` / `::motor2`):

| Field | Type | Units / note |
|---|---|---|
| `speed_rpm` | `int` | signed rpm |
| `current_a` | `std::optional<double>` | A (empty for dual `read_monitor()` — use `read_main_data()`) |
| `output_raw` | `std::optional<int>` | controller output (−1023..1023) |
| `position` | `int32_t` | count |

## Unit conversion (`mdrobot_cpp/units.hpp`)

`counts_to_rad`, `rad_to_counts`, `rpm_to_rad_s`, `rad_s_to_rpm`,
`slow_seconds_to_raw`, `slow_raw_to_seconds` — same semantics as the
[Python `mdrobot.units`](python.md#unit-conversion-mdrobotunits). `counts_per_rev`
is counts per **one revolution of the motor shaft**; it scales position only (speed
is `rpm → rad/s` regardless), so keep it at the motor and handle any gearbox in the
layer above.

## Error handling

Protocol errors derive from `mdrobot::MdrobotError` (which is a
`std::runtime_error`); the serial layer throws plain `std::runtime_error`. Catch
`std::exception` to cover both:

```text
std::runtime_error
└── mdrobot::MdrobotError
    ├── mdrobot::CrcError
    └── mdrobot::ProtocolError
        └── mdrobot::IncompleteResponseError   # short read (timeout / wiring / baud)
```

```cpp
try {
  auto s = mdrobot::SingleMotorConnection::open("/dev/ttyUSB0");
  s->enable();
  s->set_velocity(40);
} catch (const mdrobot::IncompleteResponseError&) {
  std::cerr << "no/short reply — check baud, ID, wiring\n";
} catch (const std::exception& e) {
  std::cerr << "error: " << e.what() << "\n";   // SerialTransport open/IO too
}
```

The serial layer also throws on a missing/wrong **port** at `open()` (caught by the
`std::exception` arm above) — re-check the port path if `open()` fails. One
`IncompleteResponseError` covers both a transient hiccup (retry may help) and a fatal
mis-setup (wrong baud/ID, dead link); choose retry-vs-abort from context. `stop()` /
`torque_off()` are register writes, so on a dead link they throw again — the real
fallback is to **cut power / e-stop**.

## Safety

- Start at **low speed**, unloaded, with an emergency stop / power cut in reach.
- **No watchdog in the library.** It does not auto-stop; the controller holds the last
  commanded speed, and destroying the connection (closing the port) does **not** stop a
  moving motor. Always `stop()` explicitly and keep a power cut reachable.
- **Stopping under load:** `torque_off()` frees the shaft (coasts / back-drives — a
  gravity/spring load drops). Use `brake()` to hold a load; `torque_off()` only when
  free-spin is safe.
- This is a generic driver — soft limits, odometry and kinematics belong in the
  layer above it.
- **Firmware & DIP:** recent firmware ships in encoder mode — driving without an
  encoder needs `ENC_PPR (156) = 0`. See
  [README → Hardware setup](README.md#hardware-setup).
- **CTRL stop switch?** `USE_LIMIT_SW (17) = 1` turns the CTRL inputs into
  **per-direction** gates — pin 6 permits CW (negative rpm), pin 8 permits CCW
  (positive rpm) — so wiring only pin 8 leaves reverse permanently blocked. Wire both
  through one normally-closed contact: [Stop input](README.md#stop-input-ctrl-connector).
- **First drive, in order:** read-only comms check → (no encoder) `ENC_PPR (156) = 0`
  → (serial-only) `USE_LIMIT_SW (17) = 0` → `enable()` → low rpm + a dwell + a stop in
  reach. See the [Python manual checklist](python.md#first-drive-checklist).
