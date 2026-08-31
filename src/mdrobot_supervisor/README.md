# mdrobot_supervisor

The decision layer. Everything the machine does is decided here; the layers
below only report and actuate.

```
transmitter --RF--> STM32 board --> rc_bridge_node --> [ supervisor ]
                                         ^                   |
                    equipment  ~/command +                   | four wheel rpm
                                                             v
                                                   mecanum_driver_node
                                                             |
                                              ttyUSB0 (one RS485 bus, 2 controllers)
```

Bring the whole thing up with
`ros2 launch mdrobot_supervisor bringup.launch.py`.

## Interface

| | | |
|---|---|---|
| sub | `~/rc` | `std_msgs/Int32MultiArray` — the bridge's ten channels |
| pub | `~/cmd_wheel_rpm` | `std_msgs/Float64MultiArray` — `[FL, FR, RL, RR]` motor rpm |
| pub | `~/command` | `std_msgs/Int32MultiArray` — `[lift, brake, drill, actuator, solenoid]` |
| pub | `~/mode` | `std_msgs/String` — base / mecanum / autonomous |
| pub | `~/auto_phase` | `std_msgs/String` — sequence phase, empty outside autonomous |
| sub | `~/plate_offset` | `geometry_msgs/Point` — from `mdrobot_plate_ocr` |
| sub | `~/joint_states` | `sensor_msgs/JointState` — four wheel positions |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

The launch file wires `~/rc` and `~/command` to `mdrobot_rc_bridge` and the drive
topics to `mecanum_driver_node`.

## Drive

Four mecanum wheels. Steer and throttle become a body twist and the twist becomes
four wheel speeds ([kinematics.py](mdrobot_supervisor/kinematics.py)), published
as one vector. Which controller and channel each wheel hangs off belongs to
`mecanum_driver_node` — both controllers share one RS485 bus, so exactly one node
owns the port.

The transmitter has two axes, so `vy` (strafe) is always 0 — there is no third
axis to drive it until a mode assigns one.

Wheel speeds are capped by scaling all four together, never per wheel: the
kinematics is linear, so uniform scaling is exactly "the same path, slower",
while clipping wheels one at a time bends a commanded straight line into an arc.

## Modes

The switch reports -1, 0 or +1. Throttle always means forward/back; what the
steer stick means is the whole difference between the two manual modes.

| | mode | steer stick |
|---|---|---|
| -1 | base | **yaws** the machine left/right |
| 0 | mecanum | **strafes** — slides sideways, heading unchanged |
| 1 | autonomous | runs the approach sequence below |

## Autonomous approach

The machine puts out electric-vehicle fires by getting under the car, drilling
through the underbody and spraying water in. The operator drives to the front of
the vehicle by hand and flips the switch; from there
[autonomous.py](mdrobot_supervisor/autonomous.py) runs:

| phase | what it does | leaves when |
|---|---|---|
| `wait_plate` | holds still | the camera has a plate |
| `align` | creeps forward, strafes onto the plate | the plate drops out of view |
| `enter` | drives **blind** under the car | `auto_entry_distance` on the encoders |
| `drill` | stops, runs the drill | `auto_drill_seconds` elapse |
| `find_hole` | waits for the upward camera to pick out the hole | a hole reading arrives |
| `align_hole` | shuffles in both axes to put the hole over the actuator | both axes within `auto_hole_tolerance` |
| `raise` | drives the actuator up into the hole | `auto_actuator_seconds` elapse |
| `spray` | opens the solenoid; water goes through the hole | `auto_spray_seconds` elapse |
| `done` | holds still | operator takes over |

**`find_hole` onwards is off by default** (`auto_hole_stage: false`): the
upward-facing camera is not fitted, so nothing publishes `~/hole_offset` and the
sequence finishes at the drill. Turn it on when the camera and its detector
exist.

`abort` replaces any phase when a guard trips. `done` and `abort` are both
terminal — the operator has to leave autonomous and come back, which is the
deliberate act that should be needed to re-arm a drill.

Alignment uses `~/plate_offset` from `mdrobot_plate_ocr`: `x` is normalised to
[-1, 1] with positive meaning the plate sits right of centre, so it feeds
straight in as an error signal.

It does **not** depend on the plate being read. The detector locates the band to
the pixel on frames whose text comes out wrong, and steering onto the plate only
needs the position, so `mdrobot_plate_ocr` publishes the offset for any located
region (`offset_from_detection`). That matters on this machine: the camera is at
its focus limit at working distance, and the characters read unreliably while
the position does not.

Hole alignment expects `~/hole_offset`, the same shape from an upward-facing
camera. The target is where the **actuator** appears in that frame
(`auto_hole_target_x/y`), not the frame centre. **No node publishes this yet** —
see Known gaps.

### Guards

Selecting the mode is the arming action. The sequence aborts on:

- **brake** pressed
- **RC link lost** (`rc_timeout`)
- **no wheel odometry** — without `~/joint_states` from both controllers there
  is no way to know how far under the car it has gone
- **`auto_max_align_seconds` / `auto_max_entry_seconds` /
  `auto_max_find_hole_seconds` / `auto_max_hole_align_seconds`** exceeded
- **losing sight of the hole while lining up** — carrying on would push the
  actuator up through whatever happened to be above it
- **implausible odometry** — travel faster than the machine was ever commanded
  to move, which is what raw encoder counts read as if taken for radians

Switching out of autonomous stops it immediately and resets it.

### The weak point

`enter` is dead reckoning. Once the plate is out of view nothing is left to
correct against, and mecanum wheels slip more than most — the rollers are meant
to. The travelled distance is an estimate, not a measurement, and it decides
where a hole gets drilled. Keep `auto_entry_distance` short and
`auto_entry_speed` low, and treat the timeouts as real guards. An IMU would help
and is not fitted yet.

## Safety

Nothing here replaces the board's failsafe or a physical e-stop. What the node
does guarantee:

- `rc_timeout` exceeded → zero rpm to both controllers, idle equipment
- brake → drive zeroed in the same tick it is seen
- wheel speeds never exceed `max_motor_rpm`
- out-of-range equipment values never reach hardware (the bridge clamps them)

## Known gaps

- **`roller_layout` is `unknown`.** It computes as `x` so the base drives, but
  the strafe direction is unverified. It cannot be settled by eye — the roller
  you see from above is the mirror of the ground-contact one that does the work.
  Only a floor test settles it, and it matters only once something commands `vy`.
- **`lift_input` is assumed `tristate`** (-1/0/+1). The channel was never seen
  moving. If it actually carries a pulse width the node holds lift at 0 and logs
  an error rather than scaling a 1500 into full-speed lift — set `lift_input:
  pwm` in that case.
- **`limit_gating` is off**, because the switch polarity is unknown. A gate with
  the polarity backwards either blocks lift forever or never fires. Measure which
  value a *triggered* switch reports, set `limit_active_value`, then enable.
- **Nothing publishes `~/hole_offset` yet**, so `auto_hole_stage` is off and the
  sequence ends at the drill. A detector needs to publish a `geometry_msgs/Point`
  with `x`/`y` normalised to [-1, 1] against the frame.
- **`auto_hole_gain_x/y` signs are unverified.** Which way the machine has to
  move to reduce an offset depends on how that camera ends up mounted. A wrong
  sign drives away from the hole until the timeout trips. Check on the bench.
- **The encoders are not calibrated, so autonomous will not run.**
  `wheel_position_units` starts at `unset` and autonomous refuses to arm: the
  drive node publishes raw counts until its `counts_per_rev` is measured, and
  counts taken for radians overstate travel by roughly 76x — `enter` would end in
  a single tick and the drill would fire at the entry point. Measure with
  `python3 examples/calibrate_counts_per_rev.py --type dual --port /dev/ttyUSB0`,
  put the values in the drive node's `counts_per_rev`, then set
  `wheel_position_units: rad` here. `odom_max_speed_factor` stays as a backstop.
- **The LED is not wired.** The plate-recognition LED has nowhere to go: the
  board's downlink carries only lift, brake, drill, actuator and solenoid.
  `~/auto_phase` reports the state in the meantime.
- The actuator is driven up for a fixed time and then released to 0 during the
  spray, rather than held against a stop. Whether it stays up on its own is not
  established, and the limit switches are wired to the lift, not to it.
- The geometry, gear ratio and wheel map are copied from the untracked
  `mecanum.yaml`; the `mdrobot_mecanum` package it belonged to is no longer in
  the repository. Confirm they still describe the machine. `max_linear_x/y` are
  0.19 rather than its 0.2, which asked for 611 motor rpm against the 600 cap.
- No IMU. `enter` and `align_hole` both run open-loop on attitude.
