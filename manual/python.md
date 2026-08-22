# Python Library Usage (`mdrobot`)

`mdrobot` is a pure-Python RS485 / Modbus RTU driver for MDROBOT MD-series motor
controllers. It works without ROS 2.

- **Single-channel** controllers (one motor) → `SingleMotorDriver`
- **Dual-channel** controllers (two motors) → `DualMotorDriver`
- Low-level register/command access is always available via `driver.client`.

## Install

```bash
pip install -e 'src/mdrobot[serial]'    # [serial] installs pyserial
python -c "import mdrobot; print(mdrobot.__file__)"   # verify
```

> On Ubuntu 24.04+ system Python, `pip install` may fail with
> *externally-managed-environment* (PEP 668). Install into a virtualenv
> (`python3 -m venv .venv && source .venv/bin/activate`, then `pip install`) — or,
> to use the system Python anyway, add `--break-system-packages`.

## Connect

1. **Wire the RS485 link.** Complete the motor, power and (if fitted) hall/encoder
   wiring per your controller's own manual first — this page covers only the serial
   link. Connect the RS485 (USB-serial) adapter to the controller's RS485 **A/B**, and
   tie the adapter and controller **GND** together. Default link settings:
   **19200 8N1**, controller ID **1**.
2. **Find the port.** The adapter usually enumerates as `/dev/ttyUSB0`; list the
   candidates and watch which one appears when you plug it in:
   ```bash
   ls /dev/serial/by-id/        # stable per-adapter names (survive re-enumeration)
   ls /dev/ttyUSB* /dev/ttyACM*
   sudo dmesg | grep -i tty | tail   # which device attached just now (sudo: dmesg is restricted on stock Ubuntu)
   ```
   Prefer the `/dev/serial/by-id/...` path when you run more than one adapter.
3. **Fix permissions, then pass the port to `open()`.** `/dev/ttyUSB*` belongs to the
   `dialout` group, so add yourself once (log out / back in to apply):
   ```bash
   sudo usermod -aG dialout $USER   # then log out / back in
   sudo chmod a+rw /dev/ttyUSB0     # or, temporarily
   ```
   Then pass the port string to `open()` — **this manual writes `open("/dev/ttyUSB0")`**;
   substitute your own path. On **Windows** the port is `COMx` (e.g. `COM3`); on
   **macOS** it is `/dev/cu.usbserial-*`.

   *Optional — so you stop retyping the port:* a udev rule can give the adapter a
   permanent name *and* permissions (no more `ttyUSB0`→`ttyUSB1`, no more `chmod`), and
   `export MDROBOT_PORT=...` lets `open()` take the port with **no** argument. Both are
   in **[Port setup](setup/port-setup.md)** — entirely optional.
4. A controller is **dual-channel** if it answers the dual-only monitor register
   `PID_PNT_MONITOR (216)`, otherwise it is **single-channel**:
   ```python
   from mdrobot import DualMotorDriver, registers as reg
   from mdrobot.exceptions import MdrobotError
   with DualMotorDriver.open("/dev/ttyUSB0") as d:
       try:
           d.client.read_registers(reg.PID_PNT_MONITOR, 7)
           print("dual")
       except MdrobotError:
           print("single")
   ```

> **No reply / `IncompleteResponseError`?** Check, in order: the **port** is right
> (it can become `ttyUSB1` after re-plugging or a reboot — re-check `ls /dev/ttyUSB*`,
> or set up a fixed name: [Port setup](setup/port-setup.md)); swapped **A/B** lines (try swapping
> them); a missing common **GND**; the wrong **baud rate / ID**. Termination/bias resistors are rarely needed on
> a short, low-speed (19200) bus. Per-model verification status is in
> [Tested drivers & firmware](../README.md#tested-drivers--firmware).

## First-drive checklist

Run these once, **in order, before the first drive**. Skip a step only if it clearly
does not apply to your controller (`reg` is `from mdrobot import registers as reg`):

1. **Comms, read-only** — `print(d.get_version(), d.get_voltage())` (no motion).
2. **No encoder?** `d.client.write_register(reg.PID_ENC_PPR, 0)` (`ENC_PPR (156) = 0`).
   Recent firmware (e.g. MD400 v8.6) ships in encoder mode and will **lurch ~0.6 s then
   alarm** on the first command until this is set once. *(The motor may move briefly
   here — keep clear.)*
3. **CTRL limit inputs** — for serial-only drive,
   `d.client.write_register(reg.PID_USE_LIMIT_SW, 0)` (`USE_LIMIT_SW (17) = 0`); some
   controllers require it, and an attached encoder makes it mandatory. Keeping a
   hardware stop switch instead? Set it to `1` and wire **both** CTRL pin 6 and pin 8 —
   pin 8 alone leaves negative rpm blocked. See
   [Stop input](README.md#stop-input-ctrl-connector).
4. **Arm motion** — `d.enable()` (required; otherwise velocity is echoed but the motor
   does not turn).
5. **Drive low + dwell + keep a stop in reach** — low rpm, hold ≥ ~1.5 s to see real
   motion, then `stop()`. Have an e-stop / power cut ready.

## Quick start

```python
from mdrobot import SingleMotorDriver, DualMotorDriver

# --- read only (never moves the motor) — always start here ---
with SingleMotorDriver.open("/dev/ttyUSB0") as d:   # your serial port (see Connect)
    print(d.get_version(), d.get_voltage(), "V")
    print(d.get_status().active)     # active alarm/status bit names
    print(d.read_monitor())          # speed / current / position

# --- single-channel drive (low speed; do the first-drive checklist above first) ---
import time
with SingleMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()                       # REQUIRED before motion
    try:
        d.set_velocity(40)           # signed rpm; + = CCW
        time.sleep(2.0)              # hold ~2 s so you actually SEE it turn (no dwell = a twitch)
    finally:
        d.stop(); d.torque_off()     # always stop — even on Ctrl-C / an exception

# --- dual-channel drive ---
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    try:
        d.set_velocities(40, 40)     # motor 1, motor 2
        time.sleep(2.0)              # some controllers start ~1 s late — hold, don't send 0 early
    finally:
        d.stop(); d.torque_off_both()
```

> **`enable()` is required before any motion.** It sets `UI_COM = 1` (serial
> control) and arms `START/STOP`. Without it, velocity commands are echoed but
> the motor does not turn.
>
> **Sign / direction:** `+` = CCW = increasing position; `-` = CW = decreasing
> (single and dual alike). Some dual controllers start turning **~1 s after** the
> command — don't send `0` immediately or you'll miss the motion.

**Position control** (absolute / relative moves) is a separate step — see
[`move_to` / `move_by` in the API reference](#singlemotordriver) below.

---

# API reference

`rpm` is signed mechanical rpm. `position` / `count` is an INT32 encoder/hall
count (`+` = CCW). `speed` for position moves is the max rpm magnitude.

## Connection & shared (`SingleMotorDriver` / `DualMotorDriver`)

| Method | Returns | Description |
|---|---|---|
| `SingleMotorDriver.open(port=None, baudrate=19200, *, slave_id=1, timeout=0.3)` | driver | Open a port and build the driver. Use as a context manager or `close()` it. `port` omitted → `$MDROBOT_PORT` ([Port setup](setup/port-setup.md)); neither set → `ValueError`. |
| `DualMotorDriver.open(port=None, baudrate=19200, *, slave_id=1, timeout=0.3)` | driver | Same, for a dual-channel controller. |
| `close()` | `None` | Close the serial port. (Context-manager `with` does this automatically.) |
| `ping()` | `bool` | `True` if the controller answers a version read. |
| `get_version()` | `int` | Firmware/DL version register. |
| `get_voltage()` | `float` | Input voltage (V). |
| `get_status()` | `StatusBits` | Status-1 bits (see [Data types](#data-types)). |
| `enable()` | `None` | Allow motion: `UI_COM = 1` + arm `START/STOP`. Call before driving. |
| `disable()` | `None` | Release the run-latch (`START_STOP = 0`); cuts the velocity reference, so a continuously-driven motor stops. |
| `reset_alarm()` | `None` | Clear a latched alarm. |

```python
with SingleMotorDriver.open("/dev/ttyUSB0", slave_id=1) as d:
    if d.ping():
        d.enable()
```

## `SingleMotorDriver`

| Method | Returns | Description |
|---|---|---|
| `set_velocity(rpm)` | `None` | Drive at signed `rpm` (`+` = CCW). `0` decelerates to stop. |
| `stop()` | `None` | Command speed `0` (controlled stop). |
| `brake()` | `None` | Short-brake stop. |
| `torque_off()` | `None` | Release torque (free spin). |
| `get_speed()` | `int` | Measured speed (signed rpm). |
| `get_current()` | `float` | Motor current (A). |
| `get_position()` | `int` | Position count (INT32). |
| `read_monitor()` | `Monitor` | Speed + current + output + position in one read. |
| `reset_position()` | `None` | Set the position counter to 0. |
| `move_to(position, speed=100)` | `None` | Absolute position move to `position` counts at ≤ `speed` rpm; stops on arrival. Needs `UI_COM=1` only (no `START/STOP` arm). |
| `move_by(delta, speed=100)` | `None` | Relative move by `delta` counts at ≤ `speed` rpm. |
| `get_in_position()` | `bool` | `True` when the last position move has arrived. |
| `wait_in_position(timeout=10.0, poll=0.1)` | `bool` | Block until in-position or `timeout` s; `True` if arrived. |

```python
d.enable()
d.move_to(-120, speed=50)     # absolute, counts
if d.wait_in_position(timeout=8.0):
    print("arrived at", d.get_position())
```

> **In a control loop, read state with one call.** `get_speed()`, `get_current()`
> and `get_position()` are each a separate Modbus round-trip; `read_monitor()`
> (single) / `read_main_data()` (dual) return speed + current + position in **one**
> round-trip. Prefer the batched read per cycle.

`wait_in_position()` **blocks** and holds the (single) serial port for up to its
timeout — nothing else, including a `stop()`, can use the port meanwhile. In a loop or
an agent, poll non-blocking instead:
```python
d.move_to(-120, speed=50)
while not d.get_in_position():
    ...           # do other work, watch for an abort; the port is free between polls
    time.sleep(0.1)
```

## `DualMotorDriver`

`channel` is `1` or `2`. `set_velocities` writes each motor on its own register
(motor 2 will not move from a single combined register on tested hardware).

> **Bare `brake()` / `torque_off()` act on ONE channel, but `stop()` acts on BOTH.**
> On a dual driver `brake(channel)`, `torque_off(channel)` and `set_velocity(channel,
> rpm)` take a channel argument and affect only that motor — `d.brake(1)` brakes
> motor 1 while **motor 2 keeps running**. To act on both, use `stop()`,
> `brake_both()` or `torque_off_both()`. (`d.brake()` with no argument raises
> `TypeError`.)

| Method | Returns | Description |
|---|---|---|
| `set_velocities(rpm1, rpm2)` | `None` | Set both motors (signed rpm). |
| `set_velocity(channel, rpm)` | `None` | Set one motor. |
| `stop()` | `None` | Stop both motors. |
| `stop_channel(channel)` | `None` | Stop one motor. |
| `brake_both()` / `brake(channel)` | `None` | Short-brake both / one. |
| `torque_off_both()` / `torque_off(channel)` | `None` | Release torque on both / one. |
| `get_speed(channel)` | `int` | Measured speed of a channel (signed rpm). |
| `get_current(channel)` | `float` | Motor current of a channel (A), from `PNT_MAIN_DATA`. |
| `get_position(channel)` | `int` | Position count of a channel. |
| `get_positions()` | `tuple[int, int]` | Both position counts. |
| `read_monitor()` | `DualMonitor` | Speed + position for both (no current — lighter read). |
| `read_main_data()` | `DualMonitor` | Speed + **current** + position for both. |
| `reset_position()` | `None` | Zero both position counters. |
| `move_to_both(pos1, pos2, speed1=100, speed2=None)` | `None` | Absolute move both (counts). `speed2=None` reuses `speed1`. |
| `move_by_both(delta1, delta2, speed1=100, speed2=None)` | `None` | Relative move both (counts). |

```python
with DualMotorDriver.open("/dev/ttyUSB0") as d:
    d.enable()
    d.set_velocities(30, -30)            # spin opposite
    print(d.get_current(1), d.get_current(2))   # A
    d.stop(); d.torque_off_both()
```

## Encoder

`SingleMotorDriver` only. Full guide — what an encoder changes, and the safety
notes: **[Using an encoder](encoder.md)**.

| Method | Returns | Description |
|---|---|---|
| `get_encoder_ppr()` | `int` | Current setting; `0` = encoder off (hall closed-loop). |
| `set_encoder_ppr(ppr)` | `None` | Use the encoder; `ppr` is its **rated pulses-per-rev**. |
| `disable_encoder()` | `None` | Turn the encoder off (`ppr = 0`). |
| `get_use_encoder_position()` | `bool` | `True` = position is encoder counts; `False` = hall counts (default). |
| `set_use_encoder_position(enabled)` | `None` | Switch the position source (needs a nonzero `ENC_PPR`). |

```python
d.set_encoder_ppr(1000)            # encoder wired, rated 1000 PPR
d.set_use_encoder_position(True)   # optional: position = encoder counts (4 x PPR per rev)
d.reset_position()
d.set_use_encoder_position(False)  # back to hall counts
d.disable_encoder()                # hall closed-loop
```

Two things to know before using these — details in [Using an encoder](encoder.md):
a too-large `ppr` makes the motor turn **faster than commanded** with no software
symptom, and the encoder position source **flips the physical sign convention**
(`+` turned CW where hall `+` is CCW). Both setters read the value back and raise
`MdrobotError` if it did not stick; pass `verify=False` / `settle=` to tune that.

## Acceleration / deceleration (slow-start / slow-down)

Ramp time maps to a 0–`PID_MAX_SS_TIME` second scale (default full scale 15 s).
**Speed** slow is hardware-verified; **position** slow is protocol-doc-based.

| Method (single / dual) | Returns | Description |
|---|---|---|
| `set_slow_start(seconds)` / `set_slow_start(channel, seconds)` | `None` | Speed acceleration ramp time (s). |
| `get_slow_start()` / `get_slow_start(channel)` | `float` | Read it back (s). |
| `set_slow_down(...)` / `get_slow_down(...)` | `None` / `float` | Speed deceleration ramp. |
| `set_position_slow_start/down(...)` / `get_position_slow_start/down(...)` | `None` / `float` | Position-mode ramps (doc-based). |
| `clear_slow_start()` / `clear_slow_down()` | `None` | Erase the speed ramps (shared). |
| `clear_position_slow_start()` / `clear_position_slow_down()` | `None` | Erase the position ramps (shared). |

All setters/getters take a keyword `full_scale_s=15.0` if your controller's
`PID_MAX_SS_TIME` differs.

```python
d.set_slow_start(2.0)     # 2 s to ramp up (single)
d.set_slow_down(1.5)
# dual: d.set_slow_start(1, 2.0)
```

## Data types

`StatusBits` (from `get_status()`):

| Field | Type | Meaning |
|---|---|---|
| `raw` | `int` | Raw status-1 byte. |
| `alarm` | `bool` | Any alarm latched. |
| `over_voltage`, `over_temperature`, `overload`, `stall`, `ctrl_fail`, `hall_or_encoder_fail`, `inverse_velocity` | `bool` | Individual fault bits. |
| `active` (property) | `list[str]` | Names of the set bits, from `STATUS1_BIT_NAMES`: `ALARM`, `CTRL_FAIL`, `OVER_VOLT`, `OVER_TEMP`, `OVER_LOAD`, `HALL_OR_ENC_FAIL`, `INV_VEL`, `STALL`. For branching, the boolean fields above (`.stall`, `.over_voltage`, …) are more robust than string matching. |

`Monitor` (from single `read_monitor()`); `DualMonitor` has `.motor1` / `.motor2`
each a `Monitor`:

| Field | Type | Units / note |
|---|---|---|
| `speed_rpm` | `int` | signed rpm |
| `current_a` | `float \| None` | A (`None` for `read_monitor()` on dual — use `read_main_data()`) |
| `output_raw` | `int \| None` | controller output (−1023..1023) |
| `position` | `int` | INT32 count |

## Low-level register access (`driver.client`)

Anything the high-level API doesn't cover is reachable through the Modbus client.
PIDs/commands are named constants in `mdrobot.registers` — a full table of register
numbers, command codes and status bits is in the **[Register reference](reference/registers.md)**.
Here **PID** means *parameter ID* (a register address), **not** a control gain.

| Method | Returns | Description |
|---|---|---|
| `read_register(pid)` | `int` | Read one 16-bit register. |
| `read_registers(pid, count)` | `list[int]` | Read `count` consecutive registers. |
| `write_register(pid, word)` | `None` | Write one 16-bit register. |
| `write_registers(pid, words)` | `None` | Write consecutive registers. |
| `read_long(pid, *, signed=True)` | `int` | Read an INT32 (low word first). |
| `write_long(pid, value)` | `None` | Write an INT32. |
| `command(cmd)` | `None` | Issue a command code (`PID_COMMAND`). |

```python
from mdrobot import registers as reg
d.client.write_register(reg.PID_USE_LIMIT_SW, 0)   # = write_register(17, 0)
```

## Unit conversion (`mdrobot.units`)

| Function | Returns | Description |
|---|---|---|
| `counts_to_rad(count, counts_per_rev)` | `float` | Count → radians. |
| `rad_to_counts(rad, counts_per_rev)` | `int` | Radians → nearest count. |
| `rpm_to_rad_s(rpm)` | `float` | rpm → rad/s. |
| `rad_s_to_rpm(rad_s)` | `float` | rad/s → rpm. |
| `slow_seconds_to_raw(seconds, full_scale_s=15.0)` | `int` | Ramp seconds → raw (0–1023). |
| `slow_raw_to_seconds(raw, full_scale_s=15.0)` | `float` | Raw → seconds. |

`counts_per_rev` is counts per **one revolution of the motor shaft** — the
controller reports both position (count) and speed (rpm) at the motor, so measure
it there: turn the motor shaft exactly N turns and divide
([`examples/calibrate_counts_per_rev.py`](../examples/calibrate_counts_per_rev.py)).
Gear ratio is **not** applied here: `counts_to_rad` scales position only, while
`rpm_to_rad_s` needs no `counts_per_rev`. Measuring at a geared output shaft would
make position the output angle while velocity stayed the motor rate — the two would
disagree by the gear ratio. Keep `counts_per_rev` at the motor and handle the
gearbox / wheel in the layer above.

## Error handling

All library errors derive from `MdrobotError`, so one `except` catches everything:

```text
MdrobotError
├── CrcError                 # CRC mismatch in a response
└── ProtocolError           # malformed / unexpected response
    └── IncompleteResponseError   # short read (timeout / wiring / wrong baud)
```

A missing or wrong **port** fails earlier and differently: `open()` raises pyserial's
`SerialException` / `FileNotFoundError` — or `ValueError` when the port is omitted and
`MDROBOT_PORT` is not set ([Port setup](setup/port-setup.md)) — none of them an
`MdrobotError`, so handle those around `open()` itself.

```python
from mdrobot import SingleMotorDriver
from mdrobot.exceptions import MdrobotError, IncompleteResponseError

try:
    with SingleMotorDriver.open("/dev/ttyUSB0") as d:
        d.enable()
        d.set_velocity(40)
except IncompleteResponseError:
    print("no/short reply — check baud rate, ID, wiring")
except MdrobotError as e:
    print("driver error:", type(e).__name__, e)
```

A timeout surfaces as `IncompleteResponseError`. The **same** exception covers a
transient bus hiccup (a retry may help) and a fatal mis-setup (wrong baud/ID or a dead
link, where retrying loops forever) — decide retry-vs-abort from context, not from the
type alone.

**Recovery needs a live link.** `stop()` / `torque_off()` are themselves register
writes, so if the link is down they raise again — the safe response is to **cut power /
e-stop**, not to call them. And closing the port does **not** stop a moving motor (see
Safety). Wrap a drive so it always *tries* to stop, and catch `KeyboardInterrupt`
(Ctrl-C is not an `MdrobotError`):

```python
import time
from mdrobot import SingleMotorDriver
from mdrobot.exceptions import MdrobotError

d = SingleMotorDriver.open("/dev/ttyUSB0")
try:
    d.enable(); d.set_velocity(40); time.sleep(2.0)
except (MdrobotError, KeyboardInterrupt):
    pass
finally:
    try:
        d.stop(); d.torque_off()
    except MdrobotError:
        pass            # link is down — cut power / e-stop instead
    d.close()
```

## Safety

- Start at **low speed**, unloaded, with an emergency stop / power cut in reach.
- **No watchdog in the pure library.** Unlike the ROS node (`command_timeout`), the
  Python/C++ library does not auto-stop. The controller holds the **last commanded
  speed** indefinitely, and **closing the serial port does not stop a moving motor** —
  always stop explicitly (the `try/finally` above) and keep a power cut reachable.
- **Stopping under load:** `torque_off()` frees the shaft (it coasts / back-drives, so
  a gravity or spring load drops). To hold a load use `brake()` (short-brake); use
  `torque_off()` only when free-spin is safe.
- Position moves stop on arrival, but a wrong large target over-rotates — test with
  small values first.

## Troubleshooting — motor won't move (or only turns one way)

1. Did you call `enable()`? (`UI_COM=1` + `START/STOP` arm)
2. **Single-channel**: some controllers require `USE_LIMIT_SW = 0` (register 17)
   for serial drive. Connecting an encoder can make this mandatory (A/B share
   pins with limit inputs).
3. **Turns one direction only?** With `USE_LIMIT_SW = 1` the CTRL inputs gate the
   two directions separately — pin 6 permits CW (negative rpm), pin 8 permits CCW
   (positive rpm) — so an open pin 6 kills reverse. Read register 48 to see the
   actual pin state (bit 2 = `DIR`/pin 6, bit 4 = `START_STOP`/pin 8; **1 = shorted
   to GND**):

   ```python
   di = d.client.read_register(reg.PID_DI)
   print(f"DI=0x{di:02X}  pin6={'GND' if di & 0b100 else 'open'}  pin8={'GND' if di & 0b10000 else 'open'}")
   ```

   Fix by wiring pin 6 as well, or by setting `USE_LIMIT_SW = 0`. See
   [Stop input](README.md#stop-input-ctrl-connector).
4. **Recent firmware, no encoder**: if the motor turns briefly then stops with an
   alarm (~0.6 s), it is in **encoder mode** — write `ENC_PPR (156) = 0` once. See
   [README → Hardware setup](README.md#hardware-setup).
5. **Dual-channel, motor 2 not turning**: `set_velocities()` commands motor 2 on
   its own register — already handled.
6. Some dual controllers turn **~1 s after** the command — don't judge "stopped"
   too early.
7. `get_status().alarm` set? Call `reset_alarm()`.
8. Unstable readbacks → cross-check `get_version()` to confirm the transaction is
   aligned (some adapters show session desync).
