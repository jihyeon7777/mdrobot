# mdrobot_motor_driver — User Manual

Guide to connecting, reading, and driving MDROBOT MD-series motor controllers —
from the plain Python or C++ library, the ROS 2 node, or `ros2_control`.

## Read in this order

Pick the interface you actually use — the four manuals are independent of each
other, ordered low-level → high-level. Pass your serial port directly
(`open("/dev/ttyUSB0")`, or `port:` in the ROS yaml); **[Port setup](setup/port-setup.md)
is optional** — a convenience for not retyping the port (a udev fixed name and/or the
`MDROBOT_PORT` default).

| # | Page | Use it for |
|---|---|---|
| 1 | **[Python library](python.md)** | `mdrobot` — connect, read, drive, position control, slow ramps, raw registers, unit conversion, **full API reference (tables)**, error handling. Worth skimming even if you only use ROS 2 — the [first-drive checklist](python.md#first-drive-checklist) lives here. |
| 2 | **[C++ library](cpp.md)** | `mdrobot_cpp` — same API in C++ (`*Connection::open` factory, object lifetime, **API reference tables**, error handling). |
| 3 | **[ROS 2 node](ros2.md)** | `mdrobot_ros2_driver` — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting. |
| 4 | **[ros2_control (C++)](ros2_control.md)** | `mdrobot_ros2_control` — the `SystemInterface` plugin: URDF parameters, state/command interfaces, units, controllers, and **twin mode** (two single-channel controllers on one bus). |

Topic guide: **[Using an encoder](encoder.md)** — velocity feedback (`ENC_PPR`),
the encoder position source (`USE_EPOSI`), and their safety notes.

## Reference

Not a reading step — a lookup companion to every page above:

| Page | Use it for |
|---|---|
| **[Register reference](reference/registers.md)** | Full table of register numbers, command codes and status-1 bits (derived from `registers.py` / `status.py`) for raw access. |

> Both single-channel (one motor) and dual-channel (two motors) controllers are
> supported. The driver is **generic** — it exposes per-motor commands and state
> and contains no robot kinematics; differential drive, odometry and limits
> belong in the robot layer above it.

## Hardware setup

If you are **not using an encoder**, send `ENC_PPR (156) = 0` once to set the encoder
PPR to 0 (`driver.disable_encoder()`). (Recent firmware ships in encoder mode; older
firmware such as v8.1 needs nothing here.) **Until you do, the first command makes the
motor lurch ~0.6 s and then alarm** — keep clear on the first power-up.

If you **are** using an encoder, wire it and set `ENC_PPR` to the encoder's rated
pulses-per-rev (`driver.set_encoder_ppr(1000)`); on supported firmware, position can
also be switched onto the encoder. Read **[Using an encoder](encoder.md)** first —
a PPR larger than the real one makes the motor turn faster than commanded, and the
encoder position source flips the physical sign convention.

For the full ordered first-drive sequence (comms check → `ENC_PPR` → `USE_LIMIT_SW` →
`enable()` → low rpm + dwell + a stop in reach), see the
[Python manual first-drive checklist](python.md#first-drive-checklist).

**USB-RS485 adapter:** any of them works, but the adapter's buffering — not the baud
rate — sets how fast the control loop can run, and swapping adapters changes the port
name. If you drive two controllers on one bus (`twin`) or want to raise `update_rate`,
read [Port setup → Adapter latency](setup/port-setup.md#adapter-latency--it-sets-your-maximum-update-rate)
first; an FTDI at its factory setting costs about 30 ms per twin cycle.

### Stop input (CTRL connector)

To stop the motor with a switch on the CTRL connector: set `USE_LIMIT_SW (17) = 1`, then
wire CTRL **pin 6 and pin 8 together** through one **normally-closed** contact to that
controller's own GND (pin 1 or 9).

```
CTRL  pin 6 (DIR)        ─┬── NC stop contact ── pin 1/9 (GND)
      pin 8 (START/STOP) ─┘
```

Closed = the motor runs. Open = it stops.

Both pins are needed because each gates **one direction**: pin 6 permits CW (negative
rpm), pin 8 permits CCW (positive rpm). The inputs are internally pulled up, so shorted
to GND = ON and left open = OFF. With `USE_LIMIT_SW = 0` the CTRL inputs are ignored
altogether and the switch does nothing.

For **twin**, use a **2-pole** NC switch — one pole per controller, each returning to
that controller's own GND pin.

Worth knowing:

- The stop is a **coast**, not a brake, and closing the contact again restarts the motor
  as soon as the next command arrives — `ros2_control` sends one every cycle.
- `USE_LIMIT_SW` has been seen back at `0` after a power cycle, so set it at every
  startup: `use_limit_sw: 1` in the `ros2_control` yaml, or register 17 from the library.
- On **older MD400 (v8.1)** the encoder A/B lines share these inputs, so with an encoder
  wired `USE_LIMIT_SW = 1` blocks all motion. On **v8.6 this is fixed** — an encoder and
  the CTRL stop work together (verified with an encoder wired and `USE_LIMIT_SW = 1`).
- Pin 7 (`RUN/BRAKE`) is overridden by the periodic velocity command, so it will not stop
  a continuously driven motor.

**Two controllers on one bus (twin):** to drive a skid-steer base from two
single-channel controllers (e.g. two MD400) over one RS485 bus, give each a
distinct Modbus slave id first — with only that unit on the bus, write `PID_ID (133)`
with the wire word `(new_id << 8) | 0xAA` (high byte = new id, low byte = the `0xAA`
write-check; e.g. id 2 → `0x02AA`), power-cycle, then use `device_type=twin`. Full
steps: [ros2_control → Twin mode](ros2_control.md#twin-mode--two-single-channel-controllers-on-one-bus).
