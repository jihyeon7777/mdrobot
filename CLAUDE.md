# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this repository is

An **unofficial, generic** driver for MDROBOT MD-series BLDC/DC motor controllers
(RS485 / Modbus RTU). A colcon workspace of complementary packages: a pure-Python
library (`mdrobot`), a C++ port (`mdrobot_cpp`), a ROS 2 node
(`mdrobot_ros2_driver`), and a `ros2_control` plugin (`mdrobot_ros2_control`).

Repository files are written in **English** — including comments and docs. Talk to
the user in **Korean**.

## Current work: mecanum drive

**Read [`docs/mecanum-plan.md`](docs/mecanum-plan.md) first, then work the first
unchecked step, in order.** That document is the source of truth for what is done,
what is next, and how each step is verified. After finishing a step, record the
result in its "Result" block and tick its checkbox before moving on.

The target robot: **4 mecanum wheels driven by 2× PNT50 dual-channel controllers**
on one RS485 bus at Modbus slave ids 1 and 2. New code lives in
`src/mdrobot_mecanum/`. **`src/mdrobot` is not modified** — the library is
deliberately kinematics-free (README.md:39) and 1.3.0 is a released,
hardware-verified artifact.

## Hardware safety protocol — the motors are connected and live

A spinning motor does **not** stop because your program exited or the port closed.

1. **Say what will move before it moves.** Ask for confirmation before any *new
   kind* of motion: a wheel that has not been spun before, the first time all four
   run together, a higher rpm, or anything on the floor. Repeating an already
   confirmed motion only needs an announcement, not a new confirmation.
2. **30 rpm, 2 s** is the default. Dual-channel controllers can take ~1 s to start,
   so anything shorter than 2 s misses the motion entirely.
3. **Every script that turns a motor uses `try/finally` → `stop()` then
   `torque_off*()`.** No exceptions. On shutdown, retry each independently (a SIGINT
   landing mid-transaction leaves a stale response tail that the next
   `flush_input()` cannot catch) — see `motor_driver_node.py:336`.
4. **If the result differs from the prediction, stop.** Do not proceed to the next
   step before the cause is understood. Incremental hardware verification is the
   whole point of the plan; skipping ahead destroys it.
5. **Record every step's outcome** in `docs/mecanum-plan.md`.

### Test environment

Wheels are **off the ground** by default. Most things verify in the air: which
wheel turns and in which direction, the four-wheel rotation pattern for `+vx` /
`+vy` / `+wz`, key mappings, and comms-failure behaviour — all checkable by reading
back `PNT_MONITOR`.

**Rollers only do their work on the ground.** `roller_layout` (which way `+vy`
strafes), geometry calibration, and the non-rotating-build diagnosis are physically
impossible to check in the air. When a step needs the floor, **ask the user to
lower the robot** and wait.

## Hardware facts that bite

Measured on **this robot** (2x PNT50, firmware DL=19, 12 V) on 2026-08-12. The first
three are not in the manual and were found by bench measurement — see
`docs/mecanum-plan.md` Step 3.

- **A velocity command is not a latch.** The controller cuts motor drive after about
  **2 s of bus silence** (0.5 / 1.0 / 1.5 s gaps survived; 2.0 and 3.0 s cut it). It is
  a *traffic* watchdog, not a command watchdog — a plain register read refreshes it as
  well as a write. Sustained motion needs a loop that keeps talking; command-then-sleep
  gives a twitch. Treat it as a safety feature: a crashed program stops the robot in
  ~2 s by itself.
- **`enable()` needs ~1.2 s to settle.** Commanding immediately after arming took
  1.93 s to produce motion; commanding 2 s later took 0.75 s. `MecanumBase.enable()`
  sleeps `ENABLE_SETTLE_S` for this reason. Skipping it looks exactly like a dead
  channel.
- **The instantaneous speed register is useless at low rpm.** A 4-pole hall gives only
  a few edges per second at 30 rpm, so single samples swing from 0 to 89 rpm for a
  steady 30 rpm command. **Measure with the position counter instead** — it
  accumulates and cannot lie about direction.
- **`enable()` is mandatory before motion** — `PID_UI_COM(78)=1`,
  `PID_COM_TAR_SPEED(180)=0`, `PID_START_STOP(100)=1`. Without it, commands echo
  correctly and the motor does not turn.
- **`ENC_PPR(156)` must be 0** for hall closed-loop when no encoder is wired.
  Recent firmware ships in encoder mode: the first command lurches ~0.6 s, then
  alarms. Writing it can reinitialise the controller, so the response may be lost —
  settle 2.5 s and read back.
- **`USE_LIMIT_SW(17)` and `USE_LIMIT_SW2(29)` usually need to be 0** for serial
  drive, and have been observed to reset after a power cycle. Re-assert at startup.
- **Sign convention**: `+` rpm = increasing position (CCW), `-` = CW.
- **No rpm clamping exists anywhere in the stack.** `word_from_int16()` is a bare
  `& 0xFFFF`, so a command past ±32767 silently flips sign. The mecanum layer
  clamps.
- **Shared bus**: one `SerialTransport` + one `ModbusClient` per slave id. The
  transport owns the Modbus t3.5 inter-frame gap bus-wide, which is what makes this
  safe (`transport.py:66-71`). Never use `DualMotorDriver.open()` for a shared bus —
  it builds its own transport. Close the transport exactly once.
- **FTDI adapters need `latency_timer=1`** or every round-trip roughly doubles.
  It is lost on reboot and on replug:
  `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`
- **Bus budget** at 19200 with `latency_timer=1`: `read_monitor` ≈ 17 ms, one
  velocity write ≈ 12 ms. Four wheels = 4 writes ≈ 48 ms per tick → 10 Hz is the
  shipped rate.

## Commands

```bash
pytest                                    # unit tests (config in pytest.ini)
pip install -e 'src/mdrobot[serial]'      # library only, no ROS 2
colcon build && source install/setup.bash # full workspace
export MDROBOT_PORT=/dev/ttyUSB0          # default port for the Python library
```

Scripts under `examples/` run straight from a bare clone — they insert the package
paths into `sys.path` themselves (see `examples/quickstart.py`).
