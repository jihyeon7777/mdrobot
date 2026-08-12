# Mecanum drive — 4 wheels on two dual-channel controllers

How to bring up and drive a mecanum base with [`mdrobot_mecanum`](../src/mdrobot_mecanum):
four wheels, two dual-channel controllers (e.g. two PNT50), one RS485 bus, two distinct
Modbus slave ids. Pure Python — no ROS 2, no colcon build.

This is a *robot* layer, not a driver. The [`mdrobot`](../src/mdrobot) library stays
kinematics-free; everything here sits on top of its public API and changes nothing in it.

> **Before anything turns.** A spinning motor does not stop because your program exited
> or the port closed. Bring the robot up with the **wheels off the ground**, start slow,
> and keep a power cut within reach. Only the last step needs the floor.

## Contents

- [What you need first](#what-you-need-first)
- [Bring-up, in order](#bring-up-in-order)
- [The configuration file](#the-configuration-file)
- [Kinematics, and the two things that surprise people](#kinematics-and-the-two-things-that-surprise-people)
- [Driving](#driving)
- [Safety properties](#safety-properties)
- [Troubleshooting](#troubleshooting)

## What you need first

**Two controllers at distinct slave ids on one bus.** Factory default is 1 on both, so
re-ID one unit before wiring them together: write `PID_ID (133)` with the wire word
`(new_id << 8) | 0xAA` (id 2 → `0x02AA`), with **only that unit on the bus**, then power
cycle. Full steps in
[ros2_control → Twin mode](ros2_control.md#twin-mode--two-single-channel-controllers-on-one-bus).

**A serial port and permissions.** See [Port setup](setup/port-setup.md). Setting
`MDROBOT_PORT` once saves passing `--port` everywhere.

**Four measurements**, or reasonable guesses you refine later:

| Value | What it is | What it affects |
|---|---|---|
| `wheel_radius` | wheel outer radius, m | overall speed scale |
| `track` | left↔right wheel centre distance, m | rotation gain |
| `wheelbase` | front↔rear wheel centre distance, m | rotation gain |
| `gear_ratio` | motor revolutions per wheel revolution | speed scale, and how fast you must spin to see anything |

Being 20 % out makes the numbers on screen wrong but the robot perfectly drivable. Only
`gear_ratio` really bites during bring-up, and only because it decides what counts as a
visible speed.

## Bring-up, in order

Run everything from the repository root, **wheels off the ground**.

### 1. Look before you touch

```bash
python3 examples/mecanum_scan.py
```

Read-only — function 0x03 and nothing else. Both controllers must answer with a sane
voltage. It prints the four settings that decide whether serial drive works at all:

| Register | What you want | Why |
|---|---|---|
| `ENC_PPR (156)` | **0** if no encoder is wired | Non-zero without an encoder makes the first command lurch ~0.6 s and then alarm |
| `USE_LIMIT_SW (17)` | usually **0** | The CTRL pins gate motion per direction; the manual records this resetting after a power cycle |
| `USE_LIMIT_SW2 (29)` | usually **0** | Same, for motor 2 |
| `MAX_RPM (221)` | — | The controller's own cap; sanity-check your limits against it |

It also reports the adapter's `latency_timer`. On FTDI parts the default of 16 roughly
doubles every round-trip, which halves the achievable control rate:

```bash
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

**This is lost on every reboot and every replug.** Check it each session, or make it
permanent with a udev rule ([Port setup](setup/port-setup.md)).

### 2. Preflight

```bash
python3 examples/mecanum_identify.py --preflight
```

Fixes the settings above and sets short controller acceleration ramps. **Every write is
gated on a read-back showing the value is wrong**, so a correctly configured controller
is left completely untouched — which matters for `ENC_PPR`, because writing it can
reinitialise the controller.

Encoder mode is reported but never changed automatically. If you have no encoder wired
and it reads non-zero, clear it deliberately:

```python
from mdrobot_mecanum.base import borrow_single
borrow_single(base.driver(1)).disable_encoder()
```

### 3. Find out which wheel is which

```bash
python3 examples/mecanum_identify.py --only 1:1 --rpm 300 --spin 8
```

Spins **one** motor output at a time so you can watch which wheel it drives. Repeat for
`1:2`, `2:1`, `2:2`.

Two things trip people up here.

**`--rpm` is a motor-shaft number.** Behind a 20:1 reduction, 30 rpm is 1.5 wheel rpm —
invisible. A few hundred is normal for identification. The tool refuses a `--rpm` above
its clamp rather than silently reducing it, because a silently clamped command looks
exactly like a motor that will not spin up.

**Judge direction by the wheel hub, not the rollers.** Mecanum rollers sit at 45° and
reading them is the single most common way to get this backwards. Pick a mark on the hub
or the rim and watch only that.

Record, for each wheel: which `(slave_id, channel)` drives it, and whether a **positive**
rpm command makes the top of the wheel move **toward the front** of the robot. That
answer is the `sign`: `+1` for forward, `-1` otherwise.

Decide which way is "front" and mark it physically before you start. Everything
downstream is relative to that choice.

### 4. Write the config

Start from [`config/mecanum.example.yaml`](../src/mdrobot_mecanum/config/mecanum.example.yaml)
and fill in what you measured. Leave `roller_layout: unknown` — see
[below](#kinematics-and-the-two-things-that-surprise-people).

### 5. Prove the dispatch path, still on blocks

```bash
python3 examples/mecanum_identify.py --config mecanum.yaml --verify
```

Drives a pure `+vx`, then `+vy`, then `+wz`, and after each compares every wheel's travel
against what the kinematics dispatched. No human judgement, no floor motion.

Expect these patterns, and this is what the tool checks:

| Motion | FL | FR | RL | RR |
|---|---|---|---|---|
| forward `+vx` | + | + | + | + |
| strafe `+vy` | − | + | + | − |
| rotate `+wz` | − | + | − | + |

It proves the wheel map, the per-wheel signs, the gear ratio and the clamp, and it
catches a dead or un-armed channel instantly. What it **cannot** prove is which way the
robot would actually travel — with the wheels in the air no roller is doing work.

### 6. Drive

```bash
python3 examples/mecanum_teleop.py --scale 20
```

Still on blocks the first time. Check that each key produces the pattern above.

### 7. On the floor

Clear 3 m × 3 m, power cut in hand. Drive a pure strafe:

- goes the way you asked → `roller_layout` is correct as configured
- goes the opposite way → flip it between `x` and `o`

Then calibrate: drive forward for 5 s at a known scale, tape-measure the travel, and
compare with `vx × t`. A constant factor error means `wheel_radius` or `gear_ratio` is
off. A 360° spin checks `track` + `wheelbase`.

## The configuration file

Two rules shape validation, and the asymmetry is deliberate:

- **A missing key is safe.** Everything except `wheels` and `geometry` has a default, so
  a hand-edited file that drops a line keeps working.
- **An unknown key is fatal**, with a "did you mean" hint. Silently ignoring
  `max_moter_rpm` would leave you believing a limit is in force when it is not.

Every problem is reported at once rather than one per run.

### Limits

`max_motor_rpm` is the **only hard cap** and the single source of truth for safety. It is
enforced at the wheel by the proportional clamp. The `max_linear_*` and `max_angular_z`
values are teleop *targets* — what the operator can ask for, not what the hardware will
accept.

If the status line shows `LIMITED` constantly, your linear targets exceed what
`max_motor_rpm` can deliver. The clamp keeps it safe either way, but the m/s on screen
becomes fiction: raise the cap or lower the targets.

To work out what a cap gives you:

```
wheel rad/s = max_motor_rpm / gear_ratio × 2π / 60
straight    = wheel rad/s × wheel_radius            m/s
pure spin   = straight / ((wheelbase + track) / 2)  rad/s
```

### Loop rate

`loop_hz` defaults to 10, and the config **rejects anything below 2 Hz**. That is not
about smoothness: the controller cuts motor drive after about 2 s of bus silence, so a
slower loop means the motors stop between ticks.

The ceiling is serial bandwidth, not CPU. At 19200 baud with `latency_timer=1` one
velocity write costs ~12 ms, so four wheels is ~48 ms per tick — 48 % duty at 10 Hz. The
config prints this on load:

```
bus budget: 4 writes x 12 ms = 48 ms/tick vs 100 ms period (48%)
```

## Kinematics, and the two things that surprise people

Convention is ROS REP-103: **+x forward, +y left, +wz counter-clockwise**. With
`lxy = (wheelbase + track) / 2`:

```
omega_FL = (vx − vy − lxy·wz) / r
omega_FR = (vx + vy + lxy·wz) / r
omega_RL = (vx + vy − lxy·wz) / r
omega_RR = (vx − vy + lxy·wz) / r
```

### `roller_layout` cannot be read off the robot

The roller you see from above projects to the **mirror** of the ground-contact roller
that actually does the work: rotating a roller 180° about the wheel axle maps its
projected direction `(a, b) → (−a, b)`. Half the mecanum diagrams in circulation are
wrong for this reason.

So `unknown` is a first-class value. It computes as `"x"`, so the robot is drivable and
everything in the air still verifies, while the tools keep telling you the strafe
*direction* is unconfirmed. Flipping it between `x` and `o` changes the `vy` column of
the kinematics and **nothing else** — forward and rotation are unaffected, and there is
no scrubbing either way. One short strafe on the floor settles it.

### The build error no software can fix

Mecanum rollers have a handedness, and only one arrangement works:

```
d = (FL, FR, RL, RR) = (−1, +1, +1, −1)
```

That is what makes the yaw terms **add** to `±(lx + ly)`. Build it the other way round
and they cancel to `±(lx − ly)` — **exactly zero for a square footprint**. Such a robot
cannot rotate at all: no combination of wheel speeds produces yaw without scrubbing.

This is a build error, not a setting. The fix is to physically swap the left and right
wheels. On the floor it shows up as `+wz` barely turning while the tyres scrub. The usual
"X or O, pick one" framing hides this diagnosis completely, which is why it is spelled
out here.

## Driving

Keys are printed on screen at startup:

```
  translate (vx, vy) — rotation is left alone
      u i o      u=(+x,+y)  i=(+x,0)  o=(+x,-y)
      j k l      j=(0,+y)   k=stop    l=(0,-y)
      m , .      m=(-x,+y)  ,=(-x,0)  .=(-x,-y)
      w/s = forward/back    a/d = STRAFE left/right
  rotate (wz) — translation is left alone
      q = left (CCW)    e = right (CW)    r = stop rotating
  arrows: up/down = +/-vx    left/right = +/-wz
  speed:  z/c linear -/+10%   v/b angular -/+10%   1..5 = 20/40/60/80/100%
  SPACE = HARD STOP    any other key = soft stop    Ctrl-C / ESC = quit
```

`a` and `d` **strafe**; they do not turn. That is the mecanum-native choice, and the
on-screen legend is there to unlearn the diff-drive habit.

### Why the keys latch

A terminal has no key-release event, only bytes. Hold-to-drive would have to be emulated
as "any byte within T ms keeps going", and T is trapped between two OS constants:
auto-repeat delivers one byte, waits ~500 ms, then repeats at ~25-30 Hz. Below 500 ms a
held key stutters; above it the robot keeps driving ~600 ms *after* release — worse,
because it betrays the operator's mental model. Latching at least tells the truth.

Safety comes from four properties instead of a short timeout: `SPACE` is a hard stop that
does not wait out the rest of the control period; any unmapped key is a soft stop, so a
startled operator mashing the keyboard stops the robot; `Ctrl-C` and `ESC` stop and cut
torque; and an idle watchdog (10 s by default) catches a dead ssh session or an absent
operator. The watchdog is deliberately long — latched teleop legitimately drives for a
long time on one keypress, and its job is not to substitute for a released key.

### Why acceleration is shaped in software

The controllers have their own slow-start ramps, but the four channels ramp
**independently**, so during any acceleration they break the wheel-speed ratio: the robot
curves while speeding up and straightens at the top. That is worst on a strafe, which
needs the ratio exactly.

Because the kinematics is linear, ramping the *twist* keeps the ratio exact at every
instant. So the controller ramps are set short and fixed (`controller_ramp_s`, 0.2 s, to
protect the drive stage from a step) and the real shaping happens in software via
`accel_*` / `decel_*`. `decel` must be ≥ `accel`: the robot has to stop at least as fast
as it starts.

## Safety properties

- **Nothing reaches a motor unclamped.** The library encodes rpm with a bare `& 0xFFFF`,
  so a command past ±32767 silently flips sign. Every rpm here passes the proportional
  clamp and an int16 guard.
- **The clamp scales all four wheels together**, never per wheel. The kinematics is
  linear, so scaling the whole vector is identical to having asked for a slower twist —
  same path, less speed. Clipping wheels individually changes the ratios between them,
  bending a straight line into an arc and putting yaw into a strafe, exactly when the
  operator has asked for the most speed and has the least margin.
- **A partial failure stops everything.** Two mecanum wheels driving while two are dead
  slews the robot unpredictably, so any write error triggers a best-effort stop of both
  controllers before the exception propagates. This mirrors the twin both-stop policy in
  the `ros2_control` plugin.
- **Shutdown stops twice, then cuts torque**, retrying each step. A SIGINT landing
  mid-transaction leaves a stale response tail on the wire that the next `flush_input()`
  cannot catch; the retry clears it. If both exhaust their retries you get a loud
  `COULD NOT STOP THE MOTORS — CUT POWER`.
- **The bus itself is a watchdog** (see below).

## Troubleshooting

### Nothing turns

In this order:

1. **Supply voltage.** Logic power alone will not turn a motor. `mecanum_scan.py` prints it.
2. **`ENC_PPR`** must be 0 unless an encoder is physically wired.
3. **`USE_LIMIT_SW` / `USE_LIMIT_SW2`** usually must be 0 for serial drive.
4. **`enable()`** is mandatory, and needs ~1.2 s to settle before the first command takes
   effect promptly. Skipping the pause looks exactly like a dead channel.
5. **Gear ratio.** 30 rpm behind a 20:1 reduction is 1.5 wheel rpm. Try a few hundred.

### It moves for a moment and then stops

**A velocity command is not a latch.** Measured on PNT50 v1.9: the controller cuts motor
drive after roughly **2 s of bus silence** (0.5 / 1.0 / 1.5 s gaps survived; 2.0 and 3.0 s
did not). It is a *traffic* watchdog, not a command watchdog — a plain register read
refreshes it as well as a write.

So `set_velocity` followed by a `sleep` gives a twitch, not motion. Any sustained motion
needs a loop that keeps talking; `MecanumBase` and the teleop write every tick for
exactly this reason.

Read positively, this is a safety feature: a control program that crashes stops sending,
and the robot stops by itself within ~2 s.

### The reported speed jumps around

At low rpm the instantaneous speed register is far too quantised to trust — a 4-pole hall
produces only a handful of edges per second, so a steady 30 rpm command reads back
anywhere from 0 to 89. **Use the position counter** when the answer has to be right; it
accumulates and cannot lie about direction. Every tool in this package measures that way.

### Forward is not forward

Symptom table for the floor test. Everything except the `+vy` and scrub rows is a `sign`
or wheel-map problem, and every one of them would already have been caught in the air by
`--verify`:

| Command | Symptom | Cause | Fix |
|---|---|---|---|
| `+vx` | goes backward | all four `sign` inverted | flip all four |
| `+vx` | rotates in place | one side's pair inverted | flip both `sign` on that side |
| `+vx` | pure sideways | one diagonal pair inverted | flip that diagonal's two |
| `+vx` | curves and slides | a single wheel inverted | flip that one |
| `+vx` | one wheel dead | un-armed or miswired channel | recheck `ENC_PPR`, `USE_LIMIT_SW`, wiring |
| `+vy` | strafes the wrong way | `roller_layout` | flip `x` ↔ `o` |
| `+vy` | rotates instead of strafing | front/rear swapped in the map | re-identify |
| `+wz` | barely turns, tyres scrub | rollers in the non-rotating pattern | **swap left and right wheels physically** |
| `+wz` | turns the wrong way | left/right swapped in the map | re-identify |

### `LIMITED` is on all the time

`max_linear_*` exceeds what `max_motor_rpm` can deliver. Safe, but the m/s readout is
fiction. Raise the cap or lower the targets — see [Limits](#limits).
