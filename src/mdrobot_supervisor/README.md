# mdrobot_supervisor

The decision layer. Everything the machine does is decided here; the layers
below only report and actuate.

```
transmitter --RF--> STM32 board --> rc_bridge_node --> [ supervisor ]
                                         ^                   |
                    equipment  ~/command +                   | four wheel rpm
                                                             v
                                        md1 (slave 1)   md2 (slave 2)   ttyUSB0
```

## Interface

| | | |
|---|---|---|
| sub | `~/rc` | `std_msgs/Int32MultiArray` — the bridge's ten channels |
| pub | `~/cmd_velocity_1`, `~/cmd_velocity_2` | `std_msgs/Float64MultiArray` — `[ch1_rpm, ch2_rpm]` per controller |
| pub | `~/command` | `std_msgs/Int32MultiArray` — `[lift, brake, drill, actuator, solenoid]` |
| pub | `~/mode` | `std_msgs/String` — base / mecanum / autonomous |
| pub | `~/auto_phase` | `std_msgs/String` — sequence phase, empty outside autonomous |
| sub | `~/plate_offset` | `geometry_msgs/Point` — from `mdrobot_plate_ocr` |
| sub | `~/joint_states_1`, `~/joint_states_2` | `sensor_msgs/JointState` — wheel positions |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

The launch file wires `~/rc` and `~/command` to `mdrobot_rc_bridge` and the two
drive topics to `/md1` and `/md2`.

## Drive

Four mecanum wheels on two dual-channel MD controllers. Steer and throttle
become a body twist, the twist becomes four wheel speeds
([kinematics.py](mdrobot_supervisor/kinematics.py)), and the wheel map splits
those across the controllers.

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
| `done` | holds still | operator takes over |

`abort` replaces any phase when a guard trips. `done` and `abort` are both
terminal — the operator has to leave autonomous and come back, which is the
deliberate act that should be needed to re-arm a drill.

Alignment uses `~/plate_offset` from `mdrobot_plate_ocr`: `x` is normalised to
[-1, 1] with positive meaning the plate sits right of centre, so it feeds
straight in as an error signal.

### Guards

Selecting the mode is the arming action. The sequence aborts on:

- **brake** pressed
- **RC link lost** (`rc_timeout`)
- **no wheel odometry** — without `~/joint_states` from both controllers there
  is no way to know how far under the car it has gone
- **`auto_max_align_seconds` / `auto_max_entry_seconds`** exceeded

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
- **`max_linear_x: 0.2` asks for 611 motor rpm against a 600 cap**, so full
  throttle is always scaled to 0.98 and diagnostics sit at WARN. Both numbers are
  from `mecanum.yaml` as-is.
- **The LED is not wired.** The plate-recognition LED has nowhere to go: the
  board's downlink carries only lift, brake, drill, actuator and solenoid.
  `~/auto_phase` reports the state in the meantime.
- **Wheel odometry needs `counts_per_rev` set on the drivers**, so their
  `~/joint_states` carries radians. The supervisor aborts autonomous rather than
  guessing if the topic is missing, but it cannot tell radians from raw counts —
  set the drivers up, or set `counts_per_rev` here.
- Water spray is not part of the sequence; the solenoid stays on the operator's
  switch.
- The wheel map, geometry and gear ratio are copied from the untracked
  `mecanum.yaml`; the `mdrobot_mecanum` package it belonged to is no longer in
  the repository. Confirm they still describe the machine.
