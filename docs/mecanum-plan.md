# Mecanum drive + keyboard teleop — working plan

Status board and build order for `src/mdrobot_mecanum`. **Work the first unchecked
step, verify it on the real motors, record the result, tick the box, then move on.**
Safety rules and hardware gotchas live in [`../CLAUDE.md`](../CLAUDE.md).

---

## Target

4 mecanum wheels driven by **2× PNT50 dual-channel controllers** on **one RS485 bus**
at Modbus slave ids **1 and 2**. Phase 1 is keyboard teleop, pure Python, no ROS 2
and no colcon build. A ROS 2 node comes later as a separate package.

## Why a new package

`src/mdrobot` is deliberately kinematics-free (README.md:39) and 1.3.0 is a released,
hardware-verified artifact. Everything the mecanum layer needs is already reachable
through public API, so **`src/mdrobot` is not modified**:

| Need | Resolution |
|---|---|
| Two slave ids on one bus | `SerialTransport` ×1 + `ModbusClient(transport, slave_id=N)` ×2. The transport owns the t3.5 gap bus-wide (`transport.py:66-71`) |
| Batched write `PID_PNT_VEL_CMD(207)` | The library's own escape hatch, `drv.client.write_registers(207, [w1, w2])`. Adding an *unverified* register as a method to a *verified* library would be a regression in its trust model |
| rpm clamping | In `base.py`. Changing `word_from_int16()` would silently alter released semantics |
| Gear ratio | `units.py:15-16` excludes it on purpose → `config.py` / `base.py` |
| `disable_encoder()` on a dual controller | `PID_ENC_PPR(156)` is channel-independent, so borrow the method: `SingleMotorDriver(dual.client).disable_encoder()` |

## Test environment

Wheels **off the ground** by default. **Step 8 is the only step that needs the
floor** — rollers only do their work on the ground, so `roller_layout`, geometry
calibration, and the non-rotating-build diagnosis cannot be checked in the air. Ask
the user to lower the robot when Step 8 is reached; a second floor session may
follow if fixes are needed.

---

## Progress

- [x] **Step 0** — this document + `CLAUDE.md` + read-only bus scan · *HW: no writes, no motion*
- [x] **Step 1** — `kinematics.py` + unit tests · *SW only*
- [x] **Step 2** — package plumbing + `config.py` + tests · *SW only*
- [x] **Step 3** — `base.py` skeleton + preflight + **first motor spin** · *HW, in the air*
- [x] **Step 4** — wheel map identified → `mecanum.yaml` · *HW, in the air*
- [x] **Step 5** — `drive()`: IK + proportional clamp + both-stop · *HW, in the air*
- [ ] **Step 6** — fault injection: unplug one controller mid-drive · *HW — deferred by the operator*
- [x] **Step 7** — `teleop_keyboard.py` · *HW, in the air*
- [x] **Step 8** — **floor verification** · *HW, on the floor*
- [x] **Step 9** — documentation
- [ ] **Step 10** — optional: `PID_PNT_VEL_CMD(207)` batched write · *HW, in the air*

---

## Step 0 · Documents + bus check

Write this document and `CLAUDE.md`. Add a read-only bus scan (it later becomes
`identify.py`'s phase 0).

**Verify (HW) — no writes, nothing turns.**
1. `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` → set to 1 if FTDI.
2. Scan: both ids answer, printing version / voltage / status bits / `ENC_PPR(156)` /
   `USE_LIMIT_SW(17,29)` / `MAX_RPM(221)`.

**Pass**: both controllers respond, voltage sane. If not, fix wiring / ids / baud
here and go no further.

### Result — PASS, 2026-08-12

`examples/mecanum_scan.py` on `/dev/ttyUSB0` @ 19200 8N1. Adapter is an **FTDI
FT232R** (`ID_SERIAL_SHORT=A50285BI`) with `latency_timer` **already 1** — no change
needed, but re-check every session (lost on reboot/replug).

Both ids answered, identical settings:

| | id 1 | id 2 |
|---|---|---|
| version | **DL=19 (~v1.9)** | **DL=19 (~v1.9)** |
| voltage | **11.7 V** | **11.6 V** |
| status bits | none | none |
| `ENC_PPR(156)` | **0** — hall closed loop | **0** |
| `USE_LIMIT_SW(17)` | **0** — disabled | **0** |
| `USE_LIMIT_SW2(29)` | **0** — disabled | **0** |
| `MAX_RPM(221)` | 1800 rpm | 1800 rpm |
| `HALL_TYPE(21)` | code 0 = 4-pole | code 0 = 4-pole |
| `PNT_MONITOR(216)` | M1/M2 both 0 rpm, pos 0 | same |

**Good news that changes the plan.** Step 3's preflight was expected to have to write
`ENC_PPR = 0` and `USE_LIMIT_SW = 0`. All four registers are **already correct on both
controllers**, so the documented encoder-mode lurch will not happen and no preflight
write is strictly required. Keep the preflight code — the manual records
`USE_LIMIT_SW` resetting after a power cycle — but make every write conditional on a
read-back showing the value is wrong. Nothing to change if nothing is wrong.

`HALL_TYPE` code 0 = 4-pole → counts/rev = 3 × 4 = **12**, which matches the repo's
measured PNT50 value (`dual_controllers.yaml:19`). Not needed for velocity commands,
but it confirms these are the expected motors.

`PNT_MONITOR(216)` returning 7 words for both ids confirms both really are
**dual-channel** controllers — the 4-motor topology this plan assumes is real.

**Two observations flagged to the user, unresolved:**

1. **Firmware DL=19 (~v1.9) is not the tested PNT50 revision.** The root README's
   tested table lists PNT50 at DL=45 / v4.5. DL=19 is an untested revision, so
   register behaviour verified on v4.5 is not automatically guaranteed here. This is
   a reason to keep the incremental hardware verification strict, not a blocker.
2. **Supply reads 11.6-11.7 V.** If this is a 24 V system, main power is off and the
   controllers are running on logic power only — motors would not turn (or would
   behave oddly) in Step 3. If it is a 12 V system, this is normal.

---

## Step 1 · `kinematics.py`

Pure math, zero I/O. See [Kinematics](#kinematics-reference) below.

**Verify (SW)** — `pytest`:
- pure `+vx` → all four equal
- pure `+vy` → `(-,+,+,-)` for layout `x`, mirrored for `o`
- pure `+wz` → `(-,+,-,+)` and the coefficient magnitude is exactly `(track+wheelbase)/2`
- `forward(inverse(V)) == V` over a twist grid, both layouts
- linearity `inverse(a·V) == a·inverse(V)`
- `slip_residual(inverse(V)) == 0`
- zero / negative / NaN geometry → `ValueError`; bad layout string → `ValueError`

### Result — PASS, 2026-08-12

`src/mdrobot_mecanum/mdrobot_mecanum/kinematics.py` + `test/test_kinematics.py`.
**235 tests pass** (128 pre-existing `mdrobot` + 107 new). `pytest.ini` updated to
cover both packages.

Written for a deliberately **non-square** test base (track 0.30, wheelbase 0.28), so
a test cannot pass by accident on `lx == ly`.

Two tests pin down the module's central claim rather than restating it:
`test_wz_coefficients_match_the_first_principles_formula` re-derives `d_i·x_i - y_i`
from the wheel positions and checks `inverse` agrees; and
`test_the_other_handedness_would_make_a_square_base_unable_to_rotate` computes the
`(lx - ly)` coefficient for the flipped pattern and shows it is exactly 0 on a square
footprint — the build-error diagnosis, executable.

**A real bug was caught by the tests and fixed.** `scale_to_limit` originally checked
only `math.isfinite(peak)`. But `max()` compares with `>`, and every comparison
against NaN is False, so a NaN sitting in the middle of the wheel vector is simply
stepped over and the peak comes back finite — a NaN wheel speed would have passed the
clamp and reached a motor command. Now every element is checked.

`roller_layout: "unknown"` is a first-class value: it computes as `"x"` so the robot
is drivable and every in-the-air check still passes (those compare measured wheel
speeds against what the IK dispatched, which is layout-independent), while
`layout_is_provisional` lets callers warn that the strafe *direction* is unverified.
`mirrored()` resolves it to `"o"` — exactly the fix when Step 8 shows `+vy` going the
wrong way.

---

## Step 2 · Package plumbing + `config.py`

`package.xml` / `setup.py` / `setup.cfg` / `resource/`, the YAML schema, and updates
to `pytest.ini` and `.github/workflows/ci.yml`. See [Config schema](#config-schema)
and [Packaging](#packaging).

**Verify (SW)** — round-trip (load → render → load) identity, each validation rule
failing individually plus multi-error aggregation, duplicate `(slave_id, channel)`,
a third slave id, `channel: 3`, `sign: 0`, unknown key rejected with a "did you
mean" hint.

### Result — PASS, 2026-08-12

`config.py` + `test/test_config.py`, plus `package.xml` / `setup.py` / `setup.cfg` /
`resource/mdrobot_mecanum` / `config/mecanum.example.yaml`. CI updated (both jobs).
**358 tests pass** from the repo root; `pytest src/mdrobot_mecanum/test` alone also
works (230), which is what the duplicated `setup.cfg` settings are for.

`config/mecanum.example.yaml` is **generated by `MecanumConfig.render()`**, not
hand-written, so the shipped example cannot drift from the schema.

**Three real bugs caught by the tests:**

1. `channel: 1.0` and `sign: 1.0` passed validation. `1.0 == 1` and `True == 1` in
   Python, so `value in (1, 2)` accepts a float or a boolean — which would then be
   written into a wire address. Fixed with an explicit `_is_int` helper that excludes
   both.
2. **Error aggregation was broken by construction.** `from_dict` raised as soon as
   the parse phase found anything, so the cross-field checks in `validate()` never
   ran and the operator got a second, different error list after fixing the first.
   `validate()` was split into `_validation_problems()` (returns a list) plus a thin
   raising wrapper, and `from_dict` now builds the config unconditionally — every
   parse failure substitutes a safe fallback — then merges both lists into one
   `ConfigError`.
3. The provisional-layout comment rendered with a stray two-space indent.

Design decisions worth remembering:

- **Missing key = safe, unknown key = error.** Every field except `wheels` and
  `geometry` has a default, so a hand-edited file that drops a line keeps working.
  An unknown key is fatal with a `difflib` hint, because silently ignoring
  `max_moter_rpm` would leave the operator believing a limit is in force when it is
  not.
- `decel_*` must be `>= accel_*` — the robot has to stop at least as fast as it
  starts, and that failure is silent until an emergency.
- `max_motor_rpm > 32767` is rejected at load time rather than wrapping sign on the
  wire later.
- `bus_budget_note()` prints `4 writes x 12 ms = 48 ms/tick vs 100 ms period (48%)`
  and escalates to `tight` above 80 % and `OVERRUN` above 100 %.

---

## Step 3 · `base.py` skeleton + preflight + first motor spin

`MecanumBase`: connection, `preflight()`, `spin_one()`, `stop()`, `torque_off()`,
`close()`, context manager. **No `drive()` yet.**

**Verify (HW, in the air) — first real motion. Confirm with the user first.**
1. Preflight writes, each confirmed and read back: `ENC_PPR = 0` (expect the
   documented ~0.6 s lurch on encoder-mode firmware), `USE_LIMIT_SW = 0` **and
   `USE_LIMIT_SW2 = 0`**, controller ramps 0.2 s.
2. `(1,1) → (1,2) → (2,1) → (2,2)` **one at a time**, 30 rpm for 2 s. Record which
   wheel turned and in which direction each time.
3. After Ctrl-C, **spin all four wheels by hand** — the only proof `torque_off`
   actually landed.

**Pass**: all four channels drive individually; stop and torque-off are certain.

**Open question to settle here**: is `PID_ENC_PPR(156)` per-channel or global on
PNT50? The repo's verification history is single-channel MD400 only. Read back after
writing and warn loudly if the value does not stick.

### Result — PASS, 2026-08-12 (after fixing what it exposed)

`base.py` + `identify.py` + `examples/mecanum_identify.py`, and `test_base.py`.
**417 tests pass.**

**Preflight wrote nothing** — all four registers were already correct on both
controllers, which is exactly the intended behaviour (every write is gated on a
read-back). `ENC_PPR` was already 0 on both, so the per-channel-or-global question
never had to be answered; it stays open.

**Final spin result** — `+30 rpm`, 5 s each, wheels in the air:

| motor output | position delta | direction for +rpm |
|---|---|---|
| id 1 ch 1 | +12 counts | forward |
| id 1 ch 2 | +12 counts | forward |
| id 2 ch 1 | +12 counts | forward |
| id 2 ch 2 | +13 counts | forward |

All four channels drive, and all four turn the same way for a positive command.

#### What the first run exposed, and the three fixes

The first attempt "passed" but the numbers were incoherent: a commanded +30 rpm read
back as +57 / +25 / +25 / +46, and motion only began 1.25-1.75 s in. Rather than
accept it, the cause was chased down. Ruled out in order: controller slow-start ramps
(all read 0), a latched stall alarm (status clean on both controllers). Then a
sequence of bench probes isolated three separate effects.

**1. A velocity command is not a latch — the controller cuts drive after ~2 s of bus
silence.** The decisive experiment held everything constant except whether frames kept
arriving:

| | counts in 10 s |
|---|---|
| commanded once | **0** |
| refreshed at 10 Hz | **35, continuous** |

Follow-up gap testing put the threshold between 1.5 s and 2 s: gaps of 0.5 / 1.0 /
1.5 s were survived, 2.0 s and 3.0 s cut the motor. It is a **traffic** watchdog, not
a command watchdog — a plain register read refreshes it just as well as a write, which
is why earlier probes that polled continuously appeared to work and ones that slept
did not.

This is not in the repo's documentation and it changes the design in three places:
`spin_and_measure` now re-sends the command every sample; `COMMAND_WATCHDOG_S = 2.0`
is documented in `base.py`; and `config.py` now **rejects** a `loop_hz` below 2 Hz,
because under that the motors stop between ticks — a correctness bug, not a comfort
one. It also retroactively promotes the teleop's "send every tick unconditionally"
from good practice to a hard requirement. Read positively, it is a safety feature: a
control program that crashes stops sending, and the robot stops by itself in ~2 s.

**2. `enable()` needs ~1.2 s to settle.** Commanding immediately after arming took
1.93 s to produce motion; commanding 2 s later took 0.75 s; re-commanding while
already armed took 0.73 s. So the extra second is a one-off cost of the run-latch arm,
not something every command pays. `MecanumBase.enable()` now sleeps `ENABLE_SETTLE_S`
(1.5 s). Without it the first command looks exactly like a dead channel.

**3. The instantaneous speed register is unusable at identification speeds.** A 4-pole
hall produces only a few edges per second at 30 rpm, so single samples swing from 0 to
89 rpm for a genuinely steady command — that was the entire "+57 vs +25" mystery. The
**position counter** accumulates and cannot lie about direction, so identification now
reports position delta and treats the speed samples as indicative only. The default
spin also went from 2 s to 5 s: at ~3 counts/s, and with ~0.8 s of every spin lost to
the start delay, a 2 s spin can end on a delta of 1 — too thin to call a direction
from. Deltas under `WEAK_SIGNAL_COUNTS` (3) are now reported as untrustworthy with a
suggestion to re-run longer, rather than silently believed.

#### Deferred

The true rpm scale is still unknown, because counts-per-rev is unknown: 12 counts in
~4.2 s is 17 rpm at 12 counts/rev but 26 rpm at 8. It does not matter here — velocity
commands are raw rpm and never touch the hall counter — and Step 8's tape-measure
check settles it. Recorded so the number is not mistaken for a measured value later.

---

## Step 4 · `identify.py` wizard → `mecanum.yaml`

Turn Step 3's observations into a config file, interactively.

**Verify (HW, in the air)** — complete the wizard:
1. Tape the front of the robot first; everything downstream is relative to it.
2. Per channel: *"which wheel turned?"* and *"watching the wheel **hub** (ignore the
   rollers) — did the top move toward the **front**?"* → `sign` = +1/−1.
3. Enter geometry. `track` and `wheelbase` only set the rotation gain and
   `wheel_radius` only sets the overall speed scale — being 20 % off makes the
   numbers wrong but the robot perfectly drivable, so refine later.
4. Leave `roller_layout: unknown`. It cannot be decided by eye: the top roller you
   see projects to the **mirror** of the ground-contact roller that does the work.
   Step 8 settles it.

**Output**: `./mecanum.yaml`

### Result — PASS, 2026-08-12

**Wheel map, measured one channel at a time at 300 rpm:**

| wheel | slave id | channel | sign |
|---|---|---|---|
| front_left | 1 | 1 | −1 |
| front_right | 1 | 2 | +1 |
| rear_left | 2 | **2** | −1 |
| rear_right | 2 | **1** | +1 |

Controller 1 drives the front axle, controller 2 the rear — but with the channels in
the opposite order, which the config simply records. Left wheels are −1 and right
wheels +1: the mirrored mounting that a left/right motor pair always produces, and a
useful coherence check on the data.

**Geometry**, supplied by the operator: wheel diameter 125 mm → `wheel_radius`
0.0625 m; `wheelbase` 0.500 m; `track` 0.575 m; **`gear_ratio` 20.0**. So
`lxy = 0.5375 m`, and at the chosen `max_motor_rpm` of 600 the robot does
0.196 m/s straight or 0.365 rad/s spinning.

`roller_layout` left at `unknown` — Step 8 settles it.

#### The gearbox changed how identification has to work

The first attempt at 30 rpm was invisible: behind a 20:1 reduction that is 1.5 wheel
rpm. Two things came out of it.

**`--rpm` is a MOTOR-shaft number**, and the docs and help text now say so, because
the wheel speed is that divided by the gear ratio. Identification generally needs a
few hundred rpm before anything is visible.

**The clamp was about to hide the problem.** The bootstrap config capped
`max_motor_rpm` at 60, so `--rpm 300` would have been silently reduced to 60 — which
looks exactly like a motor that will not spin up, with nothing on screen to say
otherwise. The bootstrap now derives its clamp from the requested speed and prints it,
a `--rpm` above the config's cap is a hard error rather than a silent reduction, and
`RPM_CEILING` (600) guards a slipped digit at the CLI where that belongs.

At 300 rpm the position counter also settled the **counts-per-rev** question left open
in Step 3: ~64 counts/s works out to ~309 rpm at 12 counts/rev, matching the command,
so 12 is right. And that in turn confirms the earlier anomaly was real — at 30 rpm the
motor managed only ~17 rpm, i.e. **there is a low-speed region where the motor cannot
hold the commanded speed**. Worth remembering when choosing teleop limits.

#### Deferred: the interactive wizard

`identify.py` grew `--only ID:CH` (spin one output and stop) rather than a full TTY
wizard, because the operator here answers through the assistant rather than at the
keyboard, and `--only` supports that flow exactly. The wizard remains worth building
for other users of the repository — deferred to Step 9.

---

## Step 5 · `drive()` — IK + proportional clamp + both-stop

**Verify (SW)** — `FakeDevice` frame-level assertions, ratio-preserving clamp,
partial failure sending zero velocity to **both** slaves.

**Verify (HW, in the air)** — `--verify --on-blocks`: command `+vx`, then `+vy`,
then `+wz` at low speed; after each, read `PNT_MONITOR` from both controllers and
**automatically** compare every channel's measured sign and rough magnitude against
what the IK dispatched. No human judgement, no floor motion. This proves the whole
dispatch path — map, sign, gear ratio, clamp — and instantly catches a dead or
un-armed channel (commanded nonzero, measured zero).

### Result — PASS, 2026-08-12

`verify.py` + `test/test_verify.py`. **436 tests pass.** Hardware run at 50 % and
again at 60 % effort, wheels in the air, position deltas over 6 s per motion:

| motion | FL | FR | RL | RR | expected pattern |
|---|---|---|---|---|---|
| forward `+vx` | +436 | +436 | +437 | +437 | all equal |
| strafe `+vy` | −433 | +442 | +445 | −431 | (−,+,+,−) |
| rotate `+wz` | −441 | +440 | −427 | +426 | (−,+,−,+) |

Every wheel matched its commanded direction, and the magnitudes agree within about
3 %. The wheel map, the per-wheel signs, the gear ratio and the clamp are all proven
end to end.

The module is deliberate about what it does **not** claim: with the wheels in the air
no roller is doing work, so which way `+vy` actually strafes is physically undecidable
here. A test asserts this rather than leaving it to a comment — the same fake robot
must pass under both `roller_layout` settings, because the checker compares wheels
against the kinematics it dispatched, not against the ground.

One display bug was caught by reading the real output: wheel labels were built with
`name[:2].upper()`, and `"front_left"[:2]` and `"front_right"[:2]` are **both `"fr"`**,
so all four columns read `FR FR RE RE`. Fixed with an explicit `WHEEL_ABBREV` constant
next to `WHEEL_NAMES`, with a comment saying why it is not derived by truncation.

---

## Step 6 · Fault injection

**Verify (HW, in the air)** — with all four wheels driving, **unplug the RS485 line
to controller 2**. Both controllers must stop, and the program must abort after
`max_comm_errors`.

This is exactly the scenario the root README lists as **still unverified** for twin
mode ("the both-stop policy when one of the two controllers drops out mid-drive —
unit-tested only"). If it passes, it is worth recording in that table.

### Result — DEFERRED, 2026-08-12

Skipped at the operator's request; the cable was not accessible at the time. The
tooling is in place and repeatable whenever it suits:

    python3 examples/mecanum_identify.py --config mecanum.yaml --fault-test

It drives all four wheels forward and opens a 30 s window; pulling the RS485 line to
either controller during it should raise a `DriveError`, and the property to confirm
by eye is that **all four wheels stop, including the two on the controller that is
still connected**. The both-stop path itself is unit-tested (`test_base.py`), so what
is missing is only the real-bus confirmation.

---

## Step 7 · `teleop_keyboard.py`

See [Teleop design](#teleop-design).

**Verify (HW, in the air)** at `--scale 20`: `i` → all four turn "forward"; `j` →
the `(-,+,+,-)` strafe pattern; `q` → the `(-,+,-,+)` rotation pattern; Space → all
stop; Ctrl-C → stop + torque-off. Each pattern must match Step 5's expectation.

### Result — PASS, 2026-08-12

`teleop_keyboard.py` + `test/test_teleop.py`. **492 tests pass.** The operator ran the
key check on blocks (teleop needs a real terminal) and reported every key behaving as
expected.

Startup and shutdown **are** verified on hardware, driven through a pseudo-terminal
that sends ESC immediately so nothing but zero is ever commanded: the bus opens, both
controllers arm, the status block redraws in place, and it exits 0 with a clean stop
and torque-off.

**That smoke test caught a timing bug no unit test would have.** The status line read
`loop 119.8 ms` against a configured 100 ms. The loop was polling for a whole period
*and then* doing ~48 ms of bus writes, so the real period was `period + work`: a
`loop_hz` of 10 ran at 8, and raising it to 15 would have quietly given 10. The loop
now does its work first and spends only what is left of the period polling for keys.
Measured again on hardware afterwards: **100.0 ms**, exactly as configured.

Two follow-on fixes came out of that restructure, both about not losing control of a
moving robot:

- A **hard stop breaks out of the polling window immediately** rather than waiting out
  the rest of the period. At the shipped 10 Hz that is 100 ms, but the docstring
  claimed "written immediately" and now it is true. Tested by measuring the gap
  between SPACE arriving and the zero reaching the wheels.
- **An overrunning tick still gets a minimum polling window** (`MIN_POLL_S`).
  Otherwise a struggling loop would poll for zero seconds and stop responding to keys
  exactly when the operator most needs them.

---

## Step 8 · Floor verification — **ask the user to lower the robot**

Clear 3 m × 3 m, power cut in hand, announce the travel distance
(`0.15 m/s × 2 s = 0.3 m`).

1. `--verify` floor mode: `+vx`, `+vy`, `+wz` for 2 s each → diagnose with the
   [symptom table](#floor-symptom-table) → offer to rewrite `mecanum.yaml`.
2. **`roller_layout` is decided here** — does `+vy` strafe left or right.
3. 20 % → 50 % → 100 %: forward/back, strafe, rotate, then the four diagonals.
4. Calibrate: drive `i` for 5 s at a known scale, tape-measure the travel, compare
   with `vx·t`. A constant factor error means `wheel_radius` or `gear_ratio` is
   wrong. A 360° spin checks `track` + `wheelbase`.

If `LIMITED k%` is on constantly, `max_linear_*` exceeds what `max_motor_rpm` can
deliver — the clamp keeps it safe but the m/s on screen becomes fiction. Raise
`max_motor_rpm` or lower the linear caps.

Any fix → back in the air to re-verify → a second floor session.

### Result — PASS, 2026-08-12

Run one motion at a time (`--motion forward|strafe|rotate`, added for this) so the
operator could watch and report each.

| motion | effort / hold | wheel deltas | robot did |
|---|---|---|---|
| forward `+vx` | 40 %, 4 s | +196 +192 +190 +191 | **went straight forward** |
| strafe `+vy` | 50 %, 4 s | −238 +237 +238 −239 | **went RIGHT** — wrong way |
| strafe `+vy` after the fix | 50 %, 4 s | +239 −236 −237 +235 | **went LEFT** — correct |
| rotate `+wz` | 70 %, 6 s | −501 +494 −486 +499 | **turned counter-clockwise, no scrub** |

**`roller_layout` is `o` on this robot.** The strafe went the opposite way on the first
attempt, which is exactly one of the two expected outcomes and exactly why the value
starts as `unknown` rather than being guessed from a diagram. Flipping it inverted the
wheel pattern — `(−,+,+,−)` became `(+,−,−,+)` — and the robot then strafed left as
commanded. Forward and rotation were unaffected either side of the change, as the
kinematics says they must be.

The rotation was clean: no scrubbing, no reluctance. So the rollers are in the
**working** handedness pattern and the build-error case from Step 1 does not apply
here. Worth stating positively, since that failure has no software remedy.

`mecanum.yaml` now has no provisional values left.

#### Deferred: dimension calibration

The tape-measure check (drive a known distance, compare with `vx × t`) was not done.
`gear_ratio` 20:1 and `wheel_radius` 0.0625 m came from the operator rather than from
measurement, so the m/s readouts are trusted, not verified. Nothing depends on them for
correctness — they set the speed scale, and the safety cap is `max_motor_rpm` — but
until it is done the numbers on screen should be read as nominal. Predicted travel for
the runs above was 0.32 m and 0.40 m if anyone wants to check retrospectively.

#### Wording fixed as a result

`--verify` was written assuming wheels in the air and said so in three places. It is
now used on the floor too, so the messages say what happens in each case, and the
failure text no longer claims "because these wheels are in the air" — the real reason a
failure there is never a layout question is that the check compares wheels against the
kinematics that dispatched them, which is identical under either layout.

---

## Step 9 · Documentation

`src/mdrobot_mecanum/README.md`, the root `README.md` package table and layout tree,
`manual/mecanum.md`. Summarise final results here.

### Result — DONE, 2026-08-12

- `src/mdrobot_mecanum/README.md` — what the package is, bring-up in order, and the two
  things that surprise people (the mirror-projection problem, and the handedness build
  error).
- `manual/mecanum.md` — the full user page, added to the manual index as entry 5 and to
  the root README's documentation list.
- Root `README.md` — package table row, repository layout, a "Mecanum base" quick start
  next to the ROS 2 ones, and two additions to **Tested drivers & firmware**: a PNT50
  DL=19 row covering the 4-wheel base, and a call-out for the command watchdog.

The watchdog note belongs in the root README rather than only here, because it applies
to **anyone** using the library on this firmware, not just to mecanum: a velocity
command is not a latch, so `set_velocity` followed by `sleep` gives a twitch. The ROS 2
node and the `ros2_control` plugin both write every cycle and are unaffected, and the
note says so.

All relative links in the repository were checked and resolve.

`/mecanum.yaml` is now gitignored. It describes one specific robot's wheel map and
dimensions, so it is local by default; `git add -f mecanum.yaml` tracks it deliberately.

#### The interactive wizard was dropped, not just deferred

Step 4 deferred it here, and on reflection it should not be built. The identification
flow that actually got used — `--only ID:CH` to spin one output, look, decide — is
supported, documented and hardware-proven. A TTY wizard would be a second, parallel
interactive path that nothing in this project exercises and that cannot be tested on
this bench, which is a poor thing to add to a repository. The `--only` flow is written
up in both READMEs and the manual instead.

---

## Step 10 · Optional — `PID_PNT_VEL_CMD(207)` batched write

The register table says "4 bytes: speed1, speed2", but it has **never been on a
wire**. Get the word order backwards and FL receives FR's command — and the symptom
(goes sideways instead of forward) looks *identical* to a wheel-map error, so it
gets misdiagnosed. The payoff is 4 writes (48 ms) → 2 writes (28 ms), lifting the
ceiling from ~15 Hz to ~20 Hz — but keyboard input carries about 2 Hz of
information, so 10 Hz already oversamples 5×.

**Verify (HW, in the air)** — `--probe-batched`, all three required:
1. `[+30, 0]` → dwell 2 s → `PNT_MONITOR` shows ch1 ≈ +30, ch2 ≈ 0
2. `[0, +30]` → the mirror image
3. `[+30, -30]` → both signs

Only all three passing proves the word order **and** the sign encoding. On success,
offer to set `use_batched_velocity: true`; a follow-up PR could then promote it to
`DualMotorDriver.set_velocities_batched()`.

### Result
_not run yet_

---
---

# Reference

## Kinematics reference

ROS REP-103, right-handed, z up: **+x forward, +y LEFT, +wz counter-clockwise**.
Wheel order is fixed everywhere: `0=FL, 1=FR, 2=RL, 3=RR`. `omega_i` is rad/s **at
the wheel**, positive when that wheel would drive the robot forward — motor mounting
direction and gear ratio are applied later.

### Derivation

Wheel *i* at body position `(x_i, y_i)` has ground-contact velocity
`v_ix = vx - wz·y_i`, `v_iy = vy + wz·x_i`. A mecanum wheel's passive rollers slide
freely perpendicular to their own axle, so the only no-slip constraint is *along*
the roller axle:

```
r·omega_i = v_ix + d_i·v_iy = vx + d_i·vy + wz·(d_i·x_i - y_i)     (d_i = ±1, 45° rollers)
```

With `lx = wheelbase/2` and `ly = track/2`, the `wz` coefficient `(d_i·x_i - y_i)`
comes out as `±(lx + ly)` — i.e. the terms **add** rather than cancel — only for

```
d = (FL, FR, RL, RR) = (-1, +1, +1, -1)
```

> ### ⚠ The most important fact in this document
>
> The other handedness pattern `(+1, -1, -1, +1)` yields a `wz` coefficient of
> `(lx - ly)`. **A square-footprint base built that way cannot rotate at all** — the
> `wz` column of the IK matrix is identically zero, so no combination of wheel
> speeds produces yaw without scrubbing.
>
> This is a **build error**, not a config option, and no amount of sign-flipping in
> software fixes it — the left and right wheels have to be physically swapped. The
> usual "X or O, pick one" framing hides this diagnosis forever. It appears in the
> [symptom table](#floor-symptom-table) as: `+wz` barely turns and the tyres scrub.

### The `roller_layout` bit

`roller_layout` is **not** "which of two valid robots do I have". It flips the `vy`
column only (`sy = +1` for `"x"`, `-1` for `"o"`); the `vx` and `wz` columns are
unchanged. The result is fully consistent — no scrubbing — and its real meaning is
"does `+vy` strafe left or right".

Its job is to absorb the fact that **the user cannot tell which pattern they have by
looking**: rotating a roller 180° about the wheel axle maps its projected direction
`(a, b) → (-a, b)`, so the top roller you see is the mirror of the ground-contact
roller that actually does the work. Half of the internet's mecanum diagrams are
wrong for exactly this reason. Default to `unknown` and let Step 8 decide.

### Implementation

```python
def inverse(vx, vy, wz, geom):
    """(vx m/s, vy m/s, wz rad/s) -> wheel angular velocity rad/s, in WHEEL_NAMES order."""
    sy = _layout_sign(geom.roller_layout)   # "x" -> +1, "o" -> -1
    r, k, y = geom.wheel_radius, geom.lxy * wz, sy * vy   # lxy == lx+ly == (wheelbase+track)/2
    return ((vx - y - k) / r,    # front_left
            (vx + y + k) / r,    # front_right
            (vx + y - k) / r,    # rear_left
            (vx - y + k) / r)    # rear_right


def forward(omega, geom):
    """Wheel rad/s -> (vx, vy, wz). Exact least-squares inverse of `inverse`."""
    fl, fr, rl, rr = omega
    sy, r = _layout_sign(geom.roller_layout), geom.wheel_radius
    return (r * (fl + fr + rl + rr) / 4.0,
            r * sy * (-fl + fr + rl - rr) / 4.0,
            r * (-fl + fr - rl + rr) / (4.0 * geom.lxy))


def slip_residual(omega):
    """0 for any physically consistent wheel-speed set; nonzero = the wheels fight each other.

    This is the null direction of the IK matrix — verified by test, not asserted by hand.
    """
    fl, fr, rl, rr = omega
    return fl + fr - rl - rr
```

Per-wheel `sign` (motor wiring and mounting) is independent of the kinematics and
composes by plain post-multiplication, applied *after* the clamp:

```
controller_rpm_i = sign_i · gear_ratio · rad_s_to_rpm(omega_i)
```

Because the clamp works on magnitudes, `sign` can never affect it — order-independent,
so apply it last where it is easiest to reason about.

## Proportional clamp

```python
def scale_to_limit(values, limit):
    """Scale a wheel-speed vector down uniformly so no element exceeds `limit`.

    Returns (scaled, k) with k in (0, 1].

    Why uniform, not per-wheel clipping: the inverse kinematics is LINEAR, so
    multiplying all four wheel speeds by k is exactly identical to having asked
    for k*(vx, vy, wz). The robot therefore follows the SAME path, just slower.
    Clipping wheels individually changes the ratios between them, which changes
    the DIRECTION — a commanded straight line becomes an arc and a commanded
    strafe picks up an unwanted yaw, precisely at the moment the operator has
    asked for the most speed and has the least margin.
    """
    peak = max((abs(v) for v in values), default=0.0)
    if peak <= limit or peak == 0.0:
        return list(values), 1.0
    k = limit / peak
    return [v * k for v in values], k
```

Then in `base.py`, rounding and the int16 guard — **the safety rail that exists
nowhere else in the stack**, since `word_from_int16()` is a bare `& 0xFFFF`
(`codec.py:24-26`):

```python
rpm = int(round(value)) * sign
rpm = max(-limit, min(limit, rpm))      # rounding moves a value by <= 1 rpm: no direction distortion
rpm = max(-32767, min(32767, rpm))
```

`drive()` returns `DriveResult(twist, wheel_rad_s, wheel_rpm, scale, clamped)` so the
teleop can render a live `LIMITED 62%` banner — the clamp stays visible to the
operator and directly unit-testable.

No per-wheel deadband and no `min_effective_rpm` boost: both would break direction
preservation. At low ramp values the rounded rpm is 0 and the robot simply does not
move for a fraction of a second. That is correct.

## Config schema

```yaml
version: 1
port: /dev/ttyUSB0          # null -> $MDROBOT_PORT
baudrate: 19200
timeout: 0.3

wheels:                     # exactly these four names; (slave_id, channel) must be unique
  front_left:  {slave_id: 1, channel: 1, sign:  1}
  front_right: {slave_id: 1, channel: 2, sign: -1}
  rear_left:   {slave_id: 2, channel: 1, sign:  1}
  rear_right:  {slave_id: 2, channel: 2, sign: -1}

roller_layout: unknown      # x | o | unknown — flips the vy column ONLY. Wrong value =
                            # strafe goes the wrong way and NOTHING else. Do not decide
                            # by eye; Step 8's floor test decides it.
geometry:
  wheel_radius: 0.05        # m
  track: 0.30               # m  LEFT-RIGHT wheel centre distance  -> ly = track/2
  wheelbase: 0.28           # m  FRONT-REAR wheel centre distance  -> lx = wheelbase/2
  gear_ratio: 1.0           # MOTOR revolutions per WHEEL revolution (>1 = reduction)

limits:
  max_motor_rpm: 100        # per motor. The proportional clamp target. THE hard cap.
  max_linear_x: 0.30        # m/s   } teleop target caps only — the base clamps at the
  max_linear_y: 0.30        # m/s   } wheel level, which is the single source of truth
  max_angular_z: 1.20       # rad/s
  accel_linear: 0.40        # m/s^2   software twist ramp, up
  decel_linear: 0.80        # m/s^2   down (must be >= accel)
  accel_angular: 1.60
  decel_angular: 3.20

runtime:
  loop_hz: 10.0
  idle_timeout: 10.0            # s, 0 disables
  max_comm_errors: 3
  use_batched_velocity: false   # PID_PNT_VEL_CMD(207) — UNVERIFIED. Only after Step 10.
  controller_ramp_s: 0.2        # slow-start/down on all 4 channels; null = leave as-is
  use_limit_sw: 0               # -1 leave as-is / 0 disable / 1 enable
  auto_enable: true
```

**Save by hand-rendering this commented template with values substituted** —
`safe_dump` throws the comments away, and this is the file the user lives with for
the life of the robot. Load with `yaml.safe_load`. Unknown keys are errors with a
`difflib` "did you mean" hint. `validate()` collects **all** problems into one
`ConfigError` rather than failing on the first.

Structural rule: the four `(slave_id, channel)` pairs must cover exactly **two
distinct slave ids × channels {1, 2}** — the only topology this package supports.
On load, print the bus budget: `est. 4 writes × 12 ms = 48 ms/tick vs 100 ms period (48%)`,
and warn if the estimate exceeds the period.

## `MecanumBase` API

```python
from_config(cfg)                    # transport ×1 + client ×2 + DualMotorDriver ×2  [Step 3]
preflight()                         # USE_LIMIT_SW / ramps; report ENC_PPR, MAX_RPM   [Step 3]
enable() / stop() / torque_off() / close()                                          # [Step 3]
spin_one(addr, rpm)                 # one (slave_id, channel) only                    [Step 3]
drive(vx, vy, wz) -> DriveResult    # IK + clamp + both-stop                          [Step 5]
drive_wheels(rpm4) -> DriveResult   # bypass IK (identify / verify)                   [Step 5]
read_wheel_rad_s()                  # PNT_MONITOR ×2, undoes sign and gear_ratio      [Step 5]
odometry_twist()                    # forward() of the measured wheel speeds          [Step 5]
__enter__ / __exit__                # __exit__ = stop -> torque_off -> close
```

`__exit__` stops and torque-offs, unlike `mdrobot._DriverBase.__exit__` which only
closes. That upgrade is this layer's job.

### Partial-failure policy

Mirrors the twin both-stop policy at `mdrobot_system.cpp:551-560`. Two mecanum
wheels driving while two are dead is *worse* than a diff-drive half-failure — the
robot slews unpredictably.

```python
try:
    for sid in self._slave_order:        # id 1 then id 2, back to back
        self._write_pair(sid, rpm)       # set_velocities(rpm_ch1, rpm_ch2)
except MdrobotError as exc:
    self._emergency_stop_all()           # per-driver try/except, NEVER raises;
    raise DriveError(...) from exc       # falls back to torque_off_both() on failure
```

`set_velocities` is two separate 0x06 writes, so channel 1 can succeed and channel 2
fail on the *same* board — this covers that too.

Tolerance lives in the **caller**, not the base: the base always both-stops and
raises; the teleop counts consecutive failures and aborts at `max_comm_errors`. That
matches the repo's layering — generic layer strict, application layer decides.

## Teleop design

### Input model: latched

A tty has no key-release event, only bytes. Hold-to-drive would have to be emulated
as "any byte within T ms keeps going", and T is wedged between two OS constants:
auto-repeat delivers **one** byte, waits **~500 ms**, then repeats at ~25-30 Hz.
`T < 500 ms` makes a held key stutter; `T > 500 ms` keeps the robot driving ~600 ms
*after* release — strictly worse than latched, because the operator's mental model
says "I let go, so it stopped". Latched at least tells the truth.

Safety comes from four properties, not a short timeout:
1. **Space = hard stop** — zeroes the target *and* the ramp, written immediately, out of band
2. **Any unmapped key = soft stop of everything** — a startled operator mashing keys stops the robot
3. **Ctrl-C / Ctrl-\ / lone ESC = stop + torque_off + exit**
4. The idle watchdog, which guards a *dead terminal*, not a released key

### Terminal mode

**`tty.setcbreak(fd)`, not `tty.setraw(fd)`.** `setcbreak` leaves `ISIG` enabled, so
Ctrl-C still raises `KeyboardInterrupt` through the normal path and the existing
`try/finally` just works. `setraw` would need hand-decoded `0x03`, and a bug there
strands a driving robot. Restore with `tcsetattr(fd, TCSADRAIN, saved)` in an
**inner** `finally`, so the motors stop first and the terminal is restored even if
stopping raised. Also install a `SIGTERM` handler so `kill` stops the robot.

### Key map

```
  Translation pad — sets vx and vy together, leaves rotation alone:
      u  i  o          u = (+x,+y)    i = (+x, 0)    o = (+x,-y)
      j  k  l          j = ( 0,+y)    k = ( 0, 0)    l = ( 0,-y)
      m  ,  .          m = (-x,+y)    , = (-x, 0)    . = (-x,-y)

  WASD aliases:  w -> i   s -> ,   a -> j (strafe left)   d -> l (strafe right)
  Rotation — sets wz, leaves translation alone:
      q = +wz (CCW / left)    e = -wz (CW / right)    r = wz -> 0
  Arrows:  Up/Down = ±vx      Left/Right = ±wz
  Speed:   z/c linear ∓10%    v/b angular ∓10%    1..5 both to 20/40/60/80/100%
  Safety:  SPACE hard stop · any other key soft stop · Ctrl-C/ESC stop+torque_off+exit
```

`a`/`d` are strafe, not turn — the mecanum-native choice; the always-visible legend
removes the diff-drive muscle-memory ambiguity. Arrow keys need a small parser: on
`0x1B`, attempt two more reads with a ~10 ms deadline (`[A/[B/[C/[D`); a lone ESC is
quit.

### Ramping: the twist in software, not the wheels in the controller

The four channels' `slow_start`/`slow_down` ramps run **independently**, so during
any acceleration they break the wheel-speed ratio — the robot curves while speeding
up and un-curves at the top. Worst on a strafe, which needs the ratio exactly.
Because the IK is linear, ramping the *twist* keeps the ratio exact at every
instant. Set the controller ramps short and fixed (`controller_ramp_s = 0.2 s`, long
enough to protect the drive stage, short enough not to distort at 10 Hz) and shape
the real ~1 s response in software with `accel_*` (growing) and `decel_*` (shrinking,
must be ≥ accel).

### Loop, watchdog, bus budget

- **Send every tick unconditionally** — even unchanged, even zero. Re-assertion
  covers the ~1 s controller start delay, survives a controller reset, and makes a
  yanked RS485 line *detectable* rather than silently ignored.
- Writes-only per tick ≈ 4 × 12 ms = **48 ms**. At `loop_hz: 10` that is 48 % duty
  with ~52 ms headroom. 15 Hz is comfortable; 20 Hz needs Step 10. Ship 10 and
  display the measured loop period.
- **Do not read `PNT_MONITOR` by default** — 2 reads is another 34 ms for
  information the operator does not need. `--monitor` adds it every 5th tick for the
  status display and the `slip_residual` diagnostic.
- **Idle watchdog** (`idle_timeout`, default **10 s**, `0` disables): no keypress at
  all while a *nonzero* twist is latched → ramp to a stop with a banner.
  Deliberately long, because latched teleop legitimately drives for a long time on
  one keypress — its job is a dead ssh session or an absent operator, not a released
  key. Live countdown in the last 3 s; any keypress resets it.
- **Bus-fault abort**: `max_comm_errors` (3) consecutive `DriveError`s → stop,
  torque_off, exit non-zero. Fewer show as a running count in the status line.
- Warn when a tick exceeds 2× the period.

### Status display

Redraw in place (`\r` + `\x1b[K` + cursor-up), throttled to ~5 Hz. Shows target
twist, ramped twist, per-wheel rpm labelled FL FR RL RR, speed scales, `LIMITED 62%`
in reverse video when `k < 1`, comm-error count, measured loop period, idle
countdown, enable state, and the key legend.

### Exit path

```python
finally:
    try:
        retry(3, base.stop)          # mirrors motor_driver_node._retry_on_shutdown
        time.sleep(0.3)              # the ~1 s start delay means a late lurch is possible
        retry(3, base.stop)          # so assert zero twice before cutting torque
        retry(3, base.torque_off)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        base.close()
```

A SIGINT landing mid-transaction leaves a stale response tail that the next
`flush_input()` cannot catch, so sleep 50 ms and retry rather than giving up
(`motor_driver_node.py:336`). If both exhaust their retries, print the loud
`COULD NOT STOP THE MOTORS — cut power` banner the ROS node already uses.

## Floor symptom table

| Command | Symptom | Cause | Fix |
|---|---|---|---|
| `+vx` | goes backward | all four `sign` inverted | flip all four |
| `+vx` | **rotates in place** | one side's pair inverted | flip both `sign` on that side |
| `+vx` | **pure sideways** | one diagonal pair inverted | flip that diagonal's two `sign` |
| `+vx` | curves and slides | a single wheel's `sign` inverted | flip that one |
| `+vx` | one wheel does not move | dead / un-armed channel | recheck `ENC_PPR`, `USE_LIMIT_SW`, wiring |
| `+vy` | goes **right** | `roller_layout` wrong | flip `x` ↔ `o` |
| `+vy` | rotates instead of strafing | front/rear swapped in the map | re-run the wheel wizard |
| `+wz` | **barely turns, tyres scrub** | rollers in the non-rotating handedness pattern | **swap the left and right wheels physically** — no software fix exists |
| `+wz` | wrong way, but `vx`/`vy` were right | left/right swapped in the map | re-run the wheel wizard |

## Packaging

`setup.py` — `install_requires=["setuptools", "PyYAML>=6"]`. **Do not list
`mdrobot`**: it is not on PyPI and pip would try to fetch it
(`mdrobot_ros2_driver/setup.py` sets this precedent); the dependency belongs in
`package.xml`. Version follows the workspace rule (all packages released together) —
bump everything to `1.4.0` at release time.

`setup.cfg` — the ament `script_dir` / `install_scripts` sections plus
`[tool:pytest] testpaths = test`, `pythonpath = . ../mdrobot`. A side effect of the
ament lines under plain pip is that console scripts land in
`<venv>/lib/mdrobot_mecanum/` rather than on `PATH`, **so console scripts are not
documented as the way to run this**. The three documented invocations are:

- **bare clone, zero setup** (primary, mirrors `examples/quickstart.py`):
  `python3 examples/mecanum_teleop.py` — a ~25-line wrapper that inserts
  `src/mdrobot` and `src/mdrobot_mecanum` into `sys.path`
- **after pip install**: `python3 -m mdrobot_mecanum.teleop_keyboard`
- **after colcon**: `ros2 run mdrobot_mecanum teleop_keyboard`

`package.xml` — format 3, `ament_python`; exec_depend `python3-serial`,
`python3-yaml`, `mdrobot`; test_depend `python3-pytest`.

Root `pytest.ini`:
```ini
pythonpath = src/mdrobot src/mdrobot_mecanum
testpaths = src/mdrobot/test src/mdrobot_mecanum/test
```
Passing both test dirs makes the common ancestor `src/`, which has no ini file, so
pytest walks up to the repo root and picks this file — which is why the per-package
`setup.cfg` duplicate is still needed for running one package's tests alone.

`.github/workflows/ci.yml` — add `pip install -e 'src/mdrobot_mecanum[dev]'` to the
`python` job and run `pytest src/mdrobot/test src/mdrobot_mecanum/test -v`; add
`mdrobot_mecanum` to the `ros2` job's `colcon test --packages-select`.

### Tests

`test_base.py` reuses the `FakeDevice` pattern from `src/mdrobot/test/test_device.py:17-61`.
One `FakeDevice` can serve *both* clients unchanged, because `_respond` echoes
`req[0]` rather than hard-coding the slave id; `frames` then records the interleaved
traffic, which is exactly what needs asserting. The new file gets its own
`w1(sid, pid, word)` / `wN(sid, pid, words)` helpers (the existing ones hard-code
slave 1) so `src/mdrobot/test` stays untouched.

## Open questions

- **Is `PID_ENC_PPR(156)` per-channel or global on PNT50?** The repo's verification
  history is single-channel MD400 only. Settled in Step 3 by read-back.
- **`counts_per_rev` is not needed here.** Velocity commands are raw rpm and never
  touch the hall counter; it only matters for odometry. Measured PNT50 = 12.0.
- **Gear ratio** — if there is a gearbox, set `gear_ratio` (motor revs per wheel
  rev). If unknown, leave 1.0 and back it out from Step 8's tape-measure check.
- **`latency_timer` is lost on reboot and replug** — check it every session.

## Out of scope (structure left open)

`kinematics.py` / `config.py` / `base.py` are rclpy-free, so a separate
`mdrobot_mecanum_ros2` package can import them directly and subscribe to `/cmd_vel`
(Twist) → `teleop_twist_keyboard` now, `teleop_twist_joy` for a gamepad later. The
orthodox `mecanum_drive_controller` / `ros2_control` route needs a `quad`
`device_type` in the C++ plugin (which currently rejects anything but 1 or 2 joints,
`mdrobot_system.cpp:81-85`) — a much larger job and a separate decision.
