# mdrobot_imu

Publishes a **WITMOTION HWT901B** attitude sensor as `sensor_msgs/Imu`.

## Why it is on this robot

The machine drives under a vehicle and drills upward through the underbody.
Once it is under there, nothing can see it — and two things have no feedback
path at all:

- **Yaw.** The autonomous sequence commands `wz = 0` from start to finish, and
  the mecanum inverse kinematics assumes the wheels do not slip. On a smooth or
  oily floor they do, unevenly, so a commanded pure translation comes out as a
  translation *plus a rotation*. Nothing measures that today, so nothing can
  correct it or even report it afterwards.
- **Drill reaction torque.** During `DRILL` the wheels are commanded to zero
  while a bit cuts into steel, and the reaction torque acts on a machine
  standing on rollers. If it turns the robot while the bit is in the hole, the
  bit is what gives.

## What it is not

**Not a position sensor.** Integrating this accelerometer twice over the ~15 s
blind entry gives roughly a metre of error even after a perfect calibration —
worse than the wheel odometry it would be replacing. Measured on the fitted
unit, uncalibrated, the Y axis sits 0.03 g off zero; that alone integrates to
tens of metres. Distance stays on the encoders. This publishes attitude.

## Quick start

```bash
colcon build --packages-select mdrobot_imu
source install/setup.bash
ros2 launch mdrobot_imu imu.launch.py

ros2 topic echo /mdrobot_imu/attitude_deg   # roll/pitch/yaw in degrees
ros2 topic echo /mdrobot_imu/data           # the full sensor_msgs/Imu
```

Options are in [`config/imu.yaml`](config/imu.yaml) — edit that rather than
passing them on the command line. The launch reads the **installed** copy, so
re-run `colcon build` after editing (or build once with `--symlink-install`).

| Topic | Type | What it is |
|---|---|---|
| `~/data` | `sensor_msgs/Imu` | Orientation, angular velocity (rad/s), linear acceleration (m/s²), REP-103 body frame |
| `~/attitude_deg` | `geometry_msgs/Vector3Stamped` | The same attitude as plain roll/pitch/yaw degrees. Redundant on purpose — this is the one you record, plot, and read off an `echo` over SSH |

## Before it steers anything

### 1. Verify the signs by hand

The node maps the sensor's frame to REP-103 with three sign parameters. They
cannot be reasoned out — they are a property of how the sensor is bolted to a
particular machine. On this one, **two of the three defaults were wrong**.

Re-do these checks after any remount, and on any other machine.

```bash
ros2 launch mdrobot_imu imu.launch.py     # one terminal
ros2 run mdrobot_imu imu_check            # another
```

`imu_check` prints one line that updates in place, showing each axis and how far
it has moved from where you started. Move the machine; every axis must go **UP**:

| Move the robot | The reading must |
|---|---|
| turn **left** | yaw **increase** |
| nose **up** | pitch **increase** |
| roll **right** | roll **increase** |

An axis that goes down has its sign backwards: flip `<axis>_sign` in
`config/imu.yaml`, rebuild, check again.

> **Use `imu_check`, not `imu_survey`, for this.** The survey reads the port
> directly and prints the sensor's own raw degrees with no mounting applied —
> right for measuring drift, exactly wrong for asking whether the node's output
> moves the right way.

A sign that is wrong here does not produce a wobble. It produces a controller
that drives the error the wrong way, under a car, with a drill.

**Measured 2026-09-03**, with `imu_check`:

| Sign | Value | How it was settled |
|---|---|---|
| `yaw_sign` | **+1** | turning left made yaw rise. Defaulted to -1 on the reasoning that WITMOTION yaw grows clockwise like a compass; it does not |
| `roll_sign` | **−1** | rolling right made roll fall, so the sensor's roll runs opposite to REP-103 |
| `pitch_sign` | **+1** | tipping the nose up made pitch rise — the one default that was right |

Two of the three were wrong, and neither could have been settled by reading the
manual. Only by moving the machine.

### 2. Put the sensor in 6-axis mode

**Done on this machine, 2026-09-05** — `ros2 run mdrobot_imu imu_configure
--axis 6`, verified by reading the register back. What it cost to find out is
worth recording:

In 9-axis it fuses the magnetometer, and this robot's job is to park inside a
steel box, beside its own BLDC motors, with a drill running. The fitted unit
already reads a badly skewed field standing still — `mx +1399, my +1844,
mz -5012` (2026-09-03) — and that is *before* the car.

An autonomous run aborted on a runaway: the heading swung -8 deg, +7, -15,
each swing bigger, with the correction saturated. It read as an unstable
control loop and it was not one. Driving forward for six seconds moved the
reported heading 57 deg while the GYRO integrated 0.81 deg over the whole
run — and when the motors stopped the heading crept back, which a rotation
does not do. The machine had barely turned. The heading hold was faithfully
chasing a magnetic field, and the wz it commanded to correct it was the only
thing actually turning the robot.

    9-axis, driving:  gyro integral +0.81 deg,  reported yaw -57.66 deg
    6-axis, at rest:  gyro integral  0.00 deg,  reported yaw   0.00 deg

6-axis yaw is gyro-integrated, so it drifts. That is the right trade here: the
sequence only needs a reference held for tens of seconds, and 6-axis also
unlocks the sensor's "Reset Z-axis angle", which is exactly the
zero-at-the-start-of-a-phase behaviour wanted.

### 3. Settle the gyro deadband — this one is open

Standing still, **all three gyro axes read exactly `0.0000` for every sample**
(119/119 in a 12 s run, 900/900 in an earlier 30 s one). At the ±2000 °/s range
one count is 0.061 °/s and a real MEMS gyro dithers by at least a count, so the
sensor's automatic zero-bias calibration is clamping small rates to zero.

Excellent for standing still. Potentially fatal here, because **this robot
twists slowly**: `auto_hole_max_speed` is 0.05 m/s. If the twist rate sits
inside the clamp, the sensor reports no rotation while the machine quietly
turns — precisely the failure it was fitted to catch.

This has **not** been measured yet. To measure it, run the survey below and
turn the sensor by hand as slowly as you can, watching whether yaw moves while
the gyro insists it is zero. If it does, the sensor's "Gyro Auto Calibrate" has
to come off — at the price of bias drift, which the same tool then measures
standing still.

## Surveying what the robot actually does

`imu_survey` prints a summary and optionally writes a CSV. It has two sample
sources, and **which one you need depends on what else is running**:

```bash
# bringup is up (driving, or turning the drill) -> read the node's output
ros2 run mdrobot_imu imu_survey --seconds 30 --csv drill.csv --topic

# nothing else running -> read the port directly. Needs no ROS at all
python3 -m mdrobot_imu.survey --seconds 60 --csv shuffle.csv
```

`bringup.launch.py` starts the IMU node, so the default serial mode and bringup
are **mutually exclusive**: two readers on one tty each get an arbitrary half of
the stream and neither can frame it, so both quietly run at half rate with gaps
— and a peak excursion read off a run with gaps is a lower bound, not a
measurement. The tool checks its own sample rate against `--expect-hz` and says
so when this happens, but `--topic` avoids it entirely and works alongside
anything.

`--topic` reports the node's **mapped** attitude (signs and mount offset
applied) where the serial path reports the sensor's raw degrees. For the
questions this tool asks it makes no difference: an excursion is an excursion
and zero is zero either way.

It answers the question that decides how much of this is worth building:
**drive the robot the way the hole search does — 0.05 m/s, on the floor it will
really work on, for as long as the phase is allowed to run — and read the yaw
excursion off the summary.** Under a couple of degrees and yaw hold is not
worth building; watch it and abort on it instead. Ten or fifteen and it is the
main event.

Run it through a real `DRILL` phase too — with `--topic`, since that needs
bringup up to turn the drill. Whether the reaction torque turns the machine is
currently unknown, and it is the failure with the highest price: a machine that
turns with the bit in the hole breaks the bit.

## Wiring and the serial port

TTL: red VCC, yellow TX, green RX, black GND — sensor TX to adapter RX.

> **Check which supply variant you have.** The two WITMOTION documents
> disagree: the older TTL manual warns that more than 5 V destroys the sensor
> and prints `VCC 5V`, while the current datasheet prints `VCC 5-36V` with a
> 9 V minimum and notes that 5 V units are a build option. They are different
> variants. 12 V into a 5 V unit kills it; 5 V into a 9 V unit will not run
> properly.

The sensor enumerates through a **CH340** adapter, which lands on a
`/dev/ttyUSB` path — the same numbering space as the **FTDI** adapter carrying
the motor bus. Which one is `USB0` depends on plug order, so both the config
and the survey tool default to a `by-id` path:

```bash
ls -l /dev/serial/by-id/
# usb-1a86_USB_Serial-if00-port0            -> the CH340, this sensor
# usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 -> the FTDI, the motor bus
```

## Output rate and the baud rate

Default is 9600 baud, 10 Hz. With acceleration, angular velocity, angle,
magnetic field and barometer all enabled, one sample is 5 packets × 11 bytes =
55 bytes, so 10 Hz already uses **57 %** of a 9600-baud line.

Raising the sensor's output rate *without* raising its baud rate does not give
more samples, it gives torn packets. Raise both, and switch off the content
types you do not use. The node counts unframeable bytes and warns when the
count climbs.

## Layout

| File | What it is |
|---|---|
| `protocol.py` | Framing and decoding. Pure — bytes in, numbers out |
| `frames.py` | Angle wrapping and the sensor→REP-103 sign mapping. Pure |
| `reader.py` | Serial transport and sample assembly |
| `imu_node.py` | The ROS 2 node |
| `survey.py` | The drift/excursion measurement tool. Serial (no ROS) or `--topic` |
| `check.py` | Live mapped attitude, for settling the mounting signs by hand |

`protocol.py` and `frames.py` carry no I/O and no ROS so the two things that
actually go wrong — resynchronising a noisy stream, and getting a sign
backwards — are unit-testable without a sensor.

```bash
python3 -m pytest src/mdrobot_imu/test
```
