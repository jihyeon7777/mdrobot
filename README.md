# mdrobot_motor_driver

[![CI](https://github.com/TaesuYim/mdrobot_motor_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/TaesuYim/mdrobot_motor_driver/actions/workflows/ci.yml)

**Unofficial** driver for **MDROBOT MD-series BLDC/DC motor controllers** (RS485 /
Modbus RTU) — control a motor directly from Python, or through the included ROS 2
node / `ros2_control` plugin. Not affiliated with MDROBOT; use it within the scope
that has actually been tested — see
[Tested drivers & firmware](#tested-drivers--firmware) first.

> **Before you drive a motor**
>
> - This software turns real motors. Bench-test with the shaft free, start at low
>   rpm, and keep a power cut / e-stop within reach. A spinning motor does **not**
>   stop just because your program exits or the port closes — command a stop
>   (the ROS 2 layers do this on shutdown; in plain Python use `try/finally`).
> - **Units differ by layer**: the ROS 2 node's **command** topics (`cmd_velocity`,
>   `cmd_position`) are always **raw controller units** (rpm / encoder counts) —
>   even when `counts_per_rev` is set, which switches its `joint_states` to SI
>   (rad, rad/s). The `ros2_control` plugin uses **SI for both commands and state**
>   once `counts_per_rev` is set. Details in the [manual](manual/README.md).
> - New here? Start with the [Python quick start](#python-library) — it needs
>   nothing but `pip` and an RS485 adapter, no ROS 2.

All packages are versioned and released together — current release: **1.3.0**.

The project is a colcon workspace of complementary packages — use only what you need:

| Package | What it is |
|---|---|
| [`mdrobot`](src/mdrobot) | Pure-Python communication library — framing, CRC, Modbus RTU protocol, registers, status, unit conversion — with **single-channel** and **dual-channel** motor driver classes. Usable on its own (plain Python / `pip`). |
| [`mdrobot_cpp`](src/mdrobot_cpp) | **C++ communication library** — the same layers as `mdrobot` (POSIX `termios` transport, CRC, Modbus RTU, registers, status, units, single/dual drivers). `ament_cmake`. |
| [`mdrobot_ros2_driver`](src/mdrobot_ros2_driver) | A generic **ROS 2 node** (Python) that wraps the library and exposes per-motor velocity/position commands and motor state. |
| [`mdrobot_ros2_control`](src/mdrobot_ros2_control) | A C++ [`ros2_control`](https://control.ros.org) **`SystemInterface` plugin** wrapping `mdrobot_cpp`. One plugin for every shape via `device_type` (single → 1 joint; dual → 2 joints on one two-channel controller; **twin → 2 joints on two single-channel controllers** at distinct slave ids on one bus, for a skid-steer base); exports position/velocity/effort state and velocity/position command interfaces. |
| [`mdrobot_mecanum`](src/mdrobot_mecanum) | An example **robot** layer, not a driver: mecanum kinematics for **4 wheels on two dual-channel controllers** sharing one bus, with bring-up/identification tooling and a keyboard teleop. Pure Python, no ROS 2. |

- **Single-channel** controllers (one motor) → `SingleMotorDriver`
- **Dual-channel** controllers (two motors) → `DualMotorDriver`

This is a *generic* motor driver: it does **not** include robot kinematics (differential drive, odometry, …). It exposes per-motor commands and state only; kinematics belong in a higher-level robot package that consumes this driver — [`mdrobot_mecanum`](src/mdrobot_mecanum) and [`mdrobot_diffbot_example`](src/mdrobot_diffbot_example) are what that looks like.

> **Python and C++.** The Python library/node and the C++ library/`ros2_control` plugin live side by side in one colcon workspace. Build only what you need with `colcon build --packages-select <pkg>`.

## Repository layout

```text
mdrobot_motor_driver/            # this repo == a colcon workspace
└── src/
    ├── mdrobot/                 # Python communication library (ament_python)
    ├── mdrobot_cpp/             # C++ communication library (ament_cmake)
    ├── mdrobot_ros2_driver/     # Python ROS 2 node (ament_python), depends on mdrobot
    ├── mdrobot_ros2_control/    # C++ ros2_control SystemInterface (ament_cmake), depends on mdrobot_cpp
    ├── mdrobot_mecanum/         # 4-wheel mecanum robot layer (ament_python), depends on mdrobot
    └── mdrobot_diffbot_example/ # optional example diff-drive robot (see its own README)
manual/                          # detailed user manual
examples/                        # minimal standalone examples
docs/                            # development working notes
```

## Requirements

- Python ≥ 3.10
- [`pyserial`](https://pypi.org/project/pyserial/) ≥ 3.5 (for real serial I/O)
- ROS 2 (tested on **Jazzy**) — for the ROS 2 node
- An RS485 (USB-serial) adapter. Default link settings: **19200 8N1**, controller ID **1**

## Install & build (ROS 2)

This repository **is** a colcon workspace — the packages live under `src/`.

```bash
git clone https://github.com/TaesuYim/mdrobot_motor_driver.git
cd mdrobot_motor_driver
rosdep install --from-paths src --ignore-src -r -y   # pulls rclpy, pyserial, ...
colcon build
source install/setup.bash
```

## Install (Python library only, no ROS 2)

From a clone of this repository (run at the repository root):

```bash
git clone https://github.com/TaesuYim/mdrobot_motor_driver.git
cd mdrobot_motor_driver
pip install -e 'src/mdrobot[serial]'    # editable install; [serial] pulls in pyserial
```

> On Ubuntu 24.04+ system Python, `pip install` may fail with
> *externally-managed-environment* (PEP 668) — use a virtualenv
> (`python3 -m venv .venv && source .venv/bin/activate`) or add
> `--break-system-packages`.

Or install directly from GitHub, without cloning:

```bash
pip install 'mdrobot[serial] @ git+https://github.com/TaesuYim/mdrobot_motor_driver.git#subdirectory=src/mdrobot'
```

## Quick start

### Python library

```python
from mdrobot import SingleMotorDriver, DualMotorDriver

# read-only first — confirm comms without moving the motor
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    print(d.get_version(), d.get_voltage(), "V", d.get_status().active)

# single-channel drive (motor turns)
import time
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()             # required before motion (UI_COM=1 + START/STOP arm)
    try:
        d.set_velocity(40) # signed rpm; + = CCW
        time.sleep(2.0)    # hold so it actually turns; no dwell = a twitch
    finally:
        d.stop(); d.torque_off()

# dual-channel
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    try:
        d.set_velocities(40, 40)
        time.sleep(2.0)    # some controllers start ~1 s late — hold, don't send 0 early
    finally:
        d.stop(); d.torque_off_both()
```

> First time? Recent firmware ships in encoder mode — see the [first-drive checklist](manual/python.md#first-drive-checklist).

> Tired of typing the port? `export MDROBOT_PORT=/dev/ttyUSB0` once and call
> `SingleMotorDriver.open()` with no arguments — or give the adapter a permanent
> name (and permissions) with a udev rule. Both in [Port setup](manual/setup/port-setup.md).

Low-level register/command access is always available via `d.client` for anything the high-level API doesn't cover.

### ROS 2 node

```bash
# set options in config/single.yaml or config/dual.yaml (port, counts_per_rev, ...).
# launch reads the INSTALLED copy: re-run `colcon build` after editing the src yaml
# (or build once with `--symlink-install`), then launch:
ros2 launch mdrobot_ros2_driver single.launch.py   # single-channel
ros2 launch mdrobot_ros2_driver dual.launch.py     # dual-channel

# send a velocity command — single: [rpm], dual: [rpm1, rpm2] (length must match the channel count)
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40]}"      # single
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40, 40]}"  # dual
# stop
ros2 service call /mdrobot_motor_driver/stop std_srvs/srv/Trigger
```

### Mecanum base (4 wheels, no ROS 2)

```bash
# wheels OFF THE GROUND for all of this
python3 examples/mecanum_scan.py                       # read-only: both controllers there?
python3 examples/mecanum_identify.py --preflight       # fix ENC_PPR / limit switches
python3 examples/mecanum_identify.py --only 1:1 --rpm 300 --spin 8   # which wheel is this?
#   ... repeat for 1:2, 2:1, 2:2, then write mecanum.yaml

python3 examples/mecanum_identify.py --config mecanum.yaml --verify  # check every wheel
python3 examples/mecanum_teleop.py --scale 20                        # drive it
```

`--rpm` is a **motor-shaft** number: behind a 20:1 reduction, 30 rpm is 1.5 wheel rpm and
invisible. Full walkthrough in the [mecanum manual](manual/mecanum.md).

### ros2_control (C++)

```bash
colcon build --packages-select mdrobot_cpp mdrobot_ros2_control
source install/setup.bash

# set port / motor id(s) / counts_per_rev in config/<type>_controllers.yaml, then:
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=single  # MD400
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=dual    # PNT50/MD400T diff base
ros2 launch mdrobot_ros2_control bringup.launch.py device_type:=twin    # two single controllers, one bus
```

The hardware plugin (`mdrobot_ros2_control/MdrobotSystemHardware`) is declared in the
robot's URDF `<ros2_control>` block. Connection settings — serial `port`, per-motor Modbus
`motor_id`, `counts_per_rev` (positive → SI rad/rad·s, otherwise raw count/rpm), gating —
live in `config/<device_type>_controllers.yaml` (the `mdrobot_hardware` section the launch
reads). **Twin** mode needs the two controllers re-IDed to distinct Modbus slave ids first
(hardware-verified on 2× MD400 — see [Tested drivers & firmware](#tested-drivers--firmware)).
See the manual for the full parameter list and twin mode.

## Documentation

Full usage, parameters, safety and troubleshooting are in the manual:

- **[Port setup](manual/setup/port-setup.md)** — set the serial port **once**: udev fixed name + permissions, the `MDROBOT_PORT` default port, multiple adapters
- **[Python library usage](manual/python.md)** — connect, read, drive, position control, API reference tables, error handling, raw access
- **[C++ library usage](manual/cpp.md)** — `mdrobot_cpp` API reference tables, `open()` factory, object lifetime, error handling
- **[ROS 2 usage](manual/ros2.md)** — build, launch, parameters, topics/services, `joint_states` units, shutdown, troubleshooting
- **[ros2_control (C++)](manual/ros2_control.md)** — `mdrobot_cpp` library + the `SystemInterface` plugin, URDF parameters, controllers, twin mode
- **[Mecanum drive](manual/mecanum.md)** — `mdrobot_mecanum`: 4 wheels on two dual-channel controllers, bring-up, kinematics, keyboard teleop

Minimal runnable examples are in [`examples/`](examples/).

## Tested drivers & firmware

Verified on real hardware (raw `PID_VERSION` DL byte is authoritative; the vX.Y is
the doc convention `DL/10 . DL%10`):

| Model | Type | Firmware (raw DL / approx.) | Verified |
|---|---|---|---|
| MD400 | single | DL=81 / v8.1 | identify, read, velocity (both directions), position (absolute/relative), ROS 2 node |
| MD400 | single | DL=86 / v8.6 | ships in encoder mode → set `ENC_PPR (156) = 0` for hall closed-loop drive (counts/rev = 30); velocity, position, ROS 2 node; `PID_ID (133)` slave-id change; twin diff-drive (2 units, one bus) via ros2_control; encoder mode with a 1000 PPR encoder wired (velocity loop only — position stays on the hall counter) |
| PNT50 | dual | DL=45 / v4.5 | identify, read, velocity (both motors), position (simultaneous), ROS 2 node |
| PNT50 | dual | DL=19 / v1.9 | identify, read, velocity (both motors); **2 units on one bus at ids 1 & 2 driving a 4-wheel mecanum base** via `mdrobot_mecanum` — wheel map, per-wheel direction, proportional clamp and keyboard teleop, all confirmed against the position counters. See the note on the command watchdog below. |
| MD400T | dual | DL=72 / v7.2 | identify, read, velocity (both motors), position (simultaneous), ROS 2 node |

> **Twin mode** (two single-channel controllers on one bus) is **hardware-verified
> end-to-end** on 2× MD400 v8.6 (2026-06): one RS485 bus at slave ids 1 & 2,
> `diff_drive_controller` velocity + differential steering with correct odometry,
> and stop + torque-off on shutdown. Re-ID one unit first by writing `PID_ID (133)`
> with the wire word `(new_id << 8) | 0xAA` (e.g. id 2 → `0x02AA`). Still **not**
> hardware-verified: the both-stop policy when one of the two controllers drops out
> mid-drive (unit-tested only). Full steps:
> [ros2_control → Twin mode](manual/ros2_control.md#twin-mode--two-single-channel-controllers-on-one-bus).

> **A velocity command is not a latch — on PNT50 v1.9 at least.** Measured 2026-08-12:
> the controller **cuts motor drive after roughly 2 seconds without bus traffic**
> (gaps of 0.5 / 1.0 / 1.5 s were survived; 2.0 and 3.0 s were not). It is a *traffic*
> watchdog rather than a command watchdog — a plain register read refreshes it just as
> well as a velocity write.
>
> So sustained motion needs a control loop that keeps talking; `set_velocity` followed
> by a `sleep` gives a twitch, not motion. The ROS 2 node and the `ros2_control` plugin
> both write every cycle and are unaffected. Read positively, it is a safety feature: a
> program that crashes stops sending, and the robot stops by itself within ~2 s.
>
> Two smaller measurements from the same session: `enable()` needs about **1.2 s to
> settle** before the first velocity command takes effect promptly (skipping the pause
> looks exactly like a dead channel), and the instantaneous speed register is far too
> quantised to trust at low rpm — a steady 30 rpm command reads back anywhere from 0 to
> 89. Use the position counter when the answer has to be right.

## License

[Apache License 2.0](LICENSE).
