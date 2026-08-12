# mdrobot_mecanum

Mecanum drive for a **4-wheel base built from two dual-channel MDROBOT controllers**
(e.g. two PNT50) on one RS485 bus at distinct Modbus slave ids.

Pure Python — no ROS 2, no colcon build, nothing but `pyserial` and `PyYAML`. It runs
straight from a clone. The generic [`mdrobot`](../mdrobot) library stays
kinematics-free; this package is the robot layer on top of it.

> **The motors are real.** A spinning motor does not stop because your program exited.
> Bring the robot up with the **wheels off the ground**, start slow, and keep a power
> cut within reach.

## What is in it

| Module | What it does |
|---|---|
| `kinematics.py` | Inverse/forward mecanum kinematics and the proportional clamp. Pure math, zero I/O. |
| `config.py` | `mecanum.yaml` — dataclasses, validation, and a commented renderer. |
| `base.py` | `MecanumBase`: one serial bus, two controllers, four wheels. The only module that touches hardware. |
| `identify.py` | Bring-up: scan, preflight, and spin one motor output at a time so you can build the wheel map. |
| `verify.py` | Drive `+vx` / `+vy` / `+wz` on blocks and check every wheel against the kinematics. |
| `teleop_keyboard.py` | Keyboard driving with latched keys, software ramping and hard stops. |

## Bring-up, in order

Everything below runs from the repository root, wheels off the ground.

### 1. Look before you touch

```bash
python3 examples/mecanum_scan.py
```

Read-only. Both controllers must answer. It also checks the adapter's `latency_timer`,
which caps the achievable control rate and is lost on every reboot and replug.

### 2. Preflight

```bash
python3 examples/mecanum_identify.py --preflight
```

Reports `ENC_PPR`, `USE_LIMIT_SW`, `MAX_RPM`, and fixes the ones that would stop serial
drive from working. **Every write is gated on a read-back showing the value is wrong**,
so a correctly configured controller is left untouched.

### 3. Find out which wheel is which

```bash
python3 examples/mecanum_identify.py --only 1:1 --rpm 300 --spin 8
```

Spins **one** motor output so you can see which wheel it drives and which way. Repeat
for `1:2`, `2:1`, `2:2`.

Two things surprise people here. `--rpm` is a **motor-shaft** number: behind a 20:1
reduction, 30 rpm is 1.5 wheel rpm and invisible, which is why a few hundred is normal.
And when you judge the direction, watch the wheel **hub**, not the rollers — the
rollers sit at 45° and reading them is how people get this wrong.

Write the results into `mecanum.yaml` (start from
[`config/mecanum.example.yaml`](config/mecanum.example.yaml)):

```yaml
wheels:
  front_left:  {slave_id: 1, channel: 1, sign: -1}   # sign = +1 if a POSITIVE rpm
  front_right: {slave_id: 1, channel: 2, sign:  1}   # command drives the robot FORWARD
  rear_left:   {slave_id: 2, channel: 2, sign: -1}
  rear_right:  {slave_id: 2, channel: 1, sign:  1}
```

Leave `roller_layout: unknown`. It cannot be decided by eye — see below.

### 4. Prove the dispatch path, still on blocks

```bash
python3 examples/mecanum_identify.py --config mecanum.yaml --verify
```

Drives a pure `+vx`, then `+vy`, then `+wz`, and after each one compares every wheel's
travel against what the kinematics asked for. No human judgement. It proves the wheel
map, the per-wheel signs, the gear ratio and the clamp, and it catches a dead or
un-armed channel immediately.

### 5. Drive it

```bash
python3 examples/mecanum_teleop.py --scale 20
```

Keys are printed on screen. In short: `wasd` translates — **`a`/`d` strafe, they do not
turn** — `q`/`e` rotate, `SPACE` is a hard stop, any unmapped key is a soft stop, and
`Ctrl-C` stops the motors and cuts torque before exiting.

Keys are **latched**: press once and it keeps going. There is no key-release event on a
terminal, so hold-to-drive would have to be faked with a timeout, and the OS
auto-repeat delay (~500 ms) makes every choice of timeout wrong in one direction or the
other. Latched at least tells the truth about what it is doing.

### 6. Settle the roller layout, on the floor

Everything above works with the wheels in the air. `roller_layout` does not, because
rollers only do their work on the ground. Drive a pure strafe on the floor:

- goes the way you asked → keep the current setting
- goes the opposite way → flip `roller_layout` between `x` and `o`

That is the *only* thing a wrong layout changes; forward and rotation are unaffected.

## Why `roller_layout` cannot be read off the robot

The roller you see from above projects to the **mirror** of the ground-contact roller
that actually does the work — rotating a roller 180° about the wheel axle maps its
projected direction `(a, b) → (−a, b)`. Half the mecanum diagrams in circulation are
wrong for exactly this reason. So the config carries `unknown` as a first-class value:
it computes as `"x"` so the robot is drivable, everything in the air still verifies,
and the tools keep saying the strafe *direction* is unconfirmed until a floor test
settles it.

## The build error no software can fix

Mecanum rollers have a handedness, and only one arrangement works:

```
d = (front_left, front_right, rear_left, rear_right) = (-1, +1, +1, -1)
```

Build it the other way round and the yaw term of the kinematics collapses from
`(lx + ly)` to `(lx − ly)` — **zero for a square footprint**. Such a robot cannot rotate
at all, no matter what any software does, and the fix is to physically swap the left and
right wheels.

On the floor it shows up as: `+wz` barely turns and the tyres scrub. Worth knowing
before you go looking for it in the config.

## Safety properties

- **Nothing reaches a motor unclamped.** The `mdrobot` library encodes rpm with a bare
  `& 0xFFFF`, so a command past ±32767 would silently flip sign. Every rpm here passes
  the proportional clamp and an int16 guard.
- **The clamp scales all four wheels together**, never per wheel. The kinematics is
  linear, so scaling the whole vector is identical to having asked for a slower twist —
  the robot follows the same path. Clipping wheels individually changes the ratios
  between them, which bends a straight line into an arc and puts yaw into a strafe,
  exactly when the operator has asked for the most speed.
- **A partial failure stops everything.** Two mecanum wheels driving while two are dead
  slews the robot unpredictably, so any write error triggers a best-effort stop of both
  controllers before the exception propagates.
- **The bus itself is a watchdog.** The controller cuts motor drive after about two
  seconds without traffic, so a control program that crashes stops the robot by itself.
  The same fact means a velocity command is **not a latch** — sustained motion needs a
  loop that keeps talking.

## Configuration

Start from [`config/mecanum.example.yaml`](config/mecanum.example.yaml); it is generated
by the renderer, so it cannot drift from the schema. Two rules shape validation:

- **A missing key is safe, an unknown key is not.** Everything except `wheels` and
  `geometry` has a default, so a hand-edited file that drops a line keeps working. An
  unknown key is a hard error with a "did you mean" hint — silently ignoring
  `max_moter_rpm` would leave you believing a limit is in force when it is not.
- **Every problem is reported at once**, not one per run.

`max_motor_rpm` is the only hard cap and the single source of truth for safety. The
`max_linear_*` and `max_angular_z` values are teleop *targets*: what the operator can
ask for, not what the hardware will accept.

## Other ways to run it

```bash
pip install -e 'src/mdrobot[serial]' -e 'src/mdrobot_mecanum'
python3 -m mdrobot_mecanum.teleop_keyboard

colcon build --packages-select mdrobot mdrobot_mecanum
ros2 run mdrobot_mecanum teleop_keyboard
```

## Tests

```bash
pytest                                  # whole workspace
pytest src/mdrobot_mecanum/test         # this package only
```

No hardware needed: a fake two-controller bus records every request frame, so the exact
bytes on the wire are asserted, not just the outcome.

## Not here yet

A ROS 2 node. `kinematics.py`, `config.py` and `base.py` are deliberately rclpy-free,
so a separate package can import them unchanged and subscribe to `/cmd_vel` — that gets
`teleop_twist_keyboard` now and `teleop_twist_joy` for a gamepad later. The orthodox
`mecanum_drive_controller` / `ros2_control` route additionally needs a 4-joint device
type in the C++ plugin, which currently accepts only 1 or 2.
