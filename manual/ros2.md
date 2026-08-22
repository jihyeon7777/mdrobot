# ROS 2 Usage (`mdrobot_ros2_driver`)

A generic ROS 2 node wrapping the `mdrobot` library. It supports single- and
dual-channel controllers via the `device_type` parameter and exposes per-motor
velocity/position commands and motor state. No robot kinematics.

> The node is an intentional, simplified subset of the library: stop / brake /
> torque_off act on the **whole device** (no per-channel service), position moves use a
> single `position_max_rpm`, and a wrong-length command is dropped with a warning. For
> per-channel control or variable position speed, use the [Python](python.md) /
> [C++](cpp.md) library directly.

## Build

This repository is a colcon workspace (packages under `src/`).

```bash
git clone https://github.com/TaesuYim/mdrobot_motor_driver.git
cd mdrobot_motor_driver
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Configuration & run

All options are set in a parameter YAML file, **not** on the command line. Each
launch file loads its defaults from the **installed** copy under `install/`:
- `src/mdrobot_ros2_driver/config/single.yaml` (single-channel) ·
  `src/mdrobot_ros2_driver/config/dual.yaml` (dual-channel)

Edit that file (port, counts_per_rev, use_limit_sw, ...), then launch:

```bash
ros2 launch mdrobot_ros2_driver single.launch.py
ros2 launch mdrobot_ros2_driver dual.launch.py
```

> **The launch reads the installed copy, not your `src/` edit.** After editing the
> src yaml, **re-run `colcon build`** (it re-copies the config into `install/`) before
> launching — or build once with `colcon build --symlink-install` so later src edits
> apply live, or point straight at your file with `config:=/absolute/path.yaml`
> (below). Otherwise the launch silently keeps the old settings.

Use your own parameter file, or set a namespace:

```bash
ros2 launch mdrobot_ros2_driver single.launch.py config:=/path/to/my.yaml namespace:=robot1
```

To run the node directly (no launch file), pass the same YAML with `--params-file`
(use a full path — a bare `config/single.yaml` does not exist at the workspace root):

```bash
ros2 run mdrobot_ros2_driver motor_driver_node --ros-args \
  --params-file src/mdrobot_ros2_driver/config/single.yaml
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `port` | `/dev/ttyUSB0` | serial device — the node takes it from this yaml, not from `MDROBOT_PORT` (always set it here); for a re-plug-proof name see [Port setup](setup/port-setup.md) |
| `baudrate` | `19200` | serial baud rate |
| `motor_id` | `1` | Modbus slave ID |
| `device_type` | `single` | `single` or `dual` |
| `command_timeout` | `0.5` | seconds; auto-stop if no new velocity command arrives (**`0` disables**) |
| `publish_rate` | `20.0` | Hz; `joint_states` publish rate. A dual cycle is ~50 ms on a 19200 bus, so keep dual at **≤ 15 Hz** (single can go higher); higher rates overrun the serial link. |
| `diag_rate` | `2.0` | Hz; diagnostics publish rate |
| `position_max_rpm` | `100` | speed cap for position moves |
| `joint_names` | `[]` | names for `joint_states` (auto-generated if empty) |
| `auto_enable` | `true` | call `enable()` on startup |
| `counts_per_rev` | `[0.0]` | per-channel; enables SI `joint_states` (see below) |
| `use_limit_sw` | `-1` | `-1` = leave as-is, `0`/`1` = force; some controllers need `0` for serial drive. With `1`, CTRL pin 6 and pin 8 gate the two rotation directions separately — see [Stop input](README.md#stop-input-ctrl-connector) |

## Topics & services

| Kind | Name | Type |
|---|---|---|
| sub | `~/cmd_velocity` | `std_msgs/Float64MultiArray` — `[rpm]` or `[rpm1, rpm2]` |
| sub | `~/cmd_position` | `std_msgs/Float64MultiArray` — `[count]` or `[count1, count2]` |
| pub | `~/joint_states` | `sensor_msgs/JointState` |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` — voltage / status / alarm |
| srv | `~/enable` `~/disable` `~/stop` `~/brake` `~/torque_off` `~/reset_alarm` `~/reset_position` | `std_srvs/Trigger` |

```bash
# velocity command — the array length MUST match the channel count
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40]}"      # single
ros2 topic pub -1 /mdrobot_motor_driver/cmd_velocity std_msgs/msg/Float64MultiArray "{data: [40, 40]}"  # dual
ros2 service call /mdrobot_motor_driver/stop std_srvs/srv/Trigger
```

> A wrong-length array is ignored with a warning (single needs `[rpm]`, dual needs
> `[rpm1, rpm2]`).

> **`pub -1` sends the command once**, so after `command_timeout` (default 0.5 s) the
> watchdog auto-stops the motor — it turns for ~0.5 s, then a "command_timeout
> exceeded" warning prints. For continuous motion publish at a rate
> (`ros2 topic pub -r 10 ...`), raise `command_timeout`, or set it to `0` to disable.

Topic/service names sit under the node name (and namespace if you set one);
prefix accordingly.

## `joint_states` units

Without `counts_per_rev`, `~/joint_states` is published in raw units (position in
counts, velocity in rpm) and the node logs a warning. Set `counts_per_rev` (per
channel) to publish SI units (position in rad, velocity in rad/s):

```bash
ros2 run mdrobot_ros2_driver motor_driver_node --ros-args \
  -p device_type:=dual -p counts_per_rev:='[24.0, 24.0]'
```

> **Command topics are always raw.** Setting `counts_per_rev` makes `~/joint_states`
> SI, but `~/cmd_velocity` and `~/cmd_position` stay **raw** (rpm / count) regardless.
> The node applies no velocity cap, so never send rad/s to `~/cmd_velocity`.
> *(The C++ `ros2_control` plugin is different: there a positive `counts_per_rev`
> makes the **command** SI too — see [ros2_control.md](ros2_control.md#units). Node
> command = raw; ros2_control command = SI when `counts_per_rev > 0`.)*

`counts_per_rev` is **per channel**: length 1 for single, length 2 (`[L, R]`) for dual
— the same length rule as `cmd_velocity`. A wrong length, or any non-positive entry,
falls back to raw with a warning. Starting points (but **measure** — it is per motor):
3 × pole count (8-pole ≈ 24, 10-pole ≈ 30, 4-pole ≈ 12); measure with
[`examples/calibrate_counts_per_rev.py`](../examples/calibrate_counts_per_rev.py).
**Attaching an encoder does not change this by default** — the reported position stays
on the hall counter, and the encoder only feeds the controller's velocity loop (verified
on MD400 v8.6). The exception: if you switched the position source with
[`set_use_encoder_position(True)`](python.md#encoder-position-source), position becomes
encoder counts — use `counts_per_rev = 4 × PPR` (e.g. 4000 for a 1000 PPR encoder) and
note the **sign convention flips physically** in that mode, so re-check direction and
odometry signs.

It is counts per **one revolution of the motor shaft** and scales the position state
only (velocity is `rpm → rad/s` regardless); keep it at the motor and handle any
gearbox in the robot layer above. Full explanation:
[Python manual → Unit conversion](python.md#unit-conversion-mdrobotunits).

## Shutting down the node

On **Ctrl-C (SIGINT)** or `kill <pid>` (SIGTERM) the node sends stop +
torque-off and then exits. Notes:

- `ros2 run` is **two processes** — the launcher and the node executable. If you
  background the node (`&`) and kill only the launcher, the node is orphaned.
  Prefer `ros2 launch` (a single Ctrl-C is clean) or foreground + Ctrl-C.
- To stop a backgrounded node gracefully:
  ```bash
  pkill -INT -f 'lib/mdrobot_ros2_driver/motor_driver_node'
  ```
- Last resort (skips the safe stop): `pkill -9 -f 'lib/mdrobot_ros2_driver/motor_driver_node'`.
  After a SIGKILL the motor may not have been stopped — cut power / e-stop, or
  reconnect and `stop` it.

## Troubleshooting — motor won't move (or only turns one way)

1. Does the node's `port` match the device? It can become `ttyUSB1` after a re-plug or
   reboot — check `ls /dev/ttyUSB*` (or use a `/dev/serial/by-id/...` path). A missing
   port fails at startup with a serial-open error.
2. Is the node enabled? (`auto_enable` true, or call `~/enable`.) `enable()` sets
   `UI_COM=1` and arms `START/STOP`.
3. **Single-channel**: some controllers need `USE_LIMIT_SW=0` for serial drive —
   set `use_limit_sw: 0` in the config file. `0` makes the controller ignore the CTRL
   inputs entirely; `1` turns CTRL pin 6 and pin 8 into per-direction gates (next item).
4. **Turns one direction only?** With `use_limit_sw: 1`, pin 6 (`DIR`) permits CW
   (negative rpm) and pin 8 (`START/STOP`) permits CCW (positive rpm), so an open pin 6
   kills reverse. Read register 48 (`PID_DI`) to see the pin state — bit 2 = pin 6,
   bit 4 = pin 8, **1 = shorted to GND**. Wire pin 6 as well, or set `use_limit_sw: 0`.
   See [Stop input](README.md#stop-input-ctrl-connector).
5. **Recent firmware, no encoder**: motor turns briefly then stops with an alarm
   (~0.6 s) → **encoder mode**. Write `ENC_PPR (156) = 0` once with the Python/C++
   library (the node has no parameter — it's a one-time controller setting). See
   [README → Hardware setup](README.md#hardware-setup).
6. **Dual-channel, motor 2 not turning**: handled by the driver (motor 2 uses its
   own command register).
7. Some dual-channel controllers turn **~1 s after** the command.
8. Alarm bit set? Call `~/reset_alarm`.
9. Unstable readbacks may indicate a serial session desync (some adapters); the
   node cross-checks the version register on connect.
10. **Turns ~0.5 s then stops (no alarm)?** That is the `command_timeout` watchdog —
   `pub -1` sent a single command. Publish at a rate (`-r 10`), raise `command_timeout`,
   or set it to `0`. A "command_timeout exceeded" warning is logged when it fires.

