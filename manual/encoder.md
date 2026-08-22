# Using an encoder

MD-series controllers close their control loops on the motor's hall sensors by
default. An attached incremental encoder can do two things: improve the
**velocity** loop, and — on firmware that supports it — replace the hall counter
as the **position** source. Single-channel (`SingleMotorDriver`) only. API
tables: [Python](python.md#encoder) / [C++](cpp.md#singlemotordriver).

## Velocity feedback (`ENC_PPR`)

Wire the encoder, then set its **rated** pulses-per-rev — not the ×4 quadrature
count:

```python
d.set_encoder_ppr(1000)   # the value printed on the encoder
d.disable_encoder()       # back to hall closed-loop (ENC_PPR = 0)
```

- **A PPR larger than the real one makes the motor turn faster than commanded**,
  and neither the reported speed nor the reported position reveals it. A
  too-small value only runs slow — start low when unsure.
- A nonzero `ENC_PPR` with no encoder wired trips `ENC_FAIL` about 0.6 s after
  the motor starts. Recent firmware ships in encoder mode, so send
  `ENC_PPR = 0` once if you run without an encoder.
- This feeds the velocity loop only: reported position stays on the hall counter
  (`3 × pole count` per revolution) and `counts_per_rev` does not change.

## Position source (`USE_EPOSI`)

On firmware that has register 46 (verified on MD400 v8.6), position can be
switched onto the encoder — requires a nonzero `ENC_PPR`:

```python
d.set_use_encoder_position(True)   # position = encoder counts
d.reset_position()
d.move_by(4000, speed=25)          # 4000 counts = one revolution (1000 PPR)
d.set_use_encoder_position(False)  # back to hall counts
```

- One revolution becomes **4 × the rated PPR** in counts (1000 PPR →
  4000 counts/rev, 0.09° per count); arrival accuracy of about ±1 count was
  measured.
- **The physical sign convention flips**: `+` commands and increasing position
  turned the verified motors **CW**, where hall-mode `+` is CCW. Remap signs in
  anything above the driver — odometry, `counts_per_rev` users, direction logic.
- The in-position window is in counts, so the arrival flag can latch slowly or
  not at all while the physical error is a fraction of a degree — always give
  `wait_in_position()` a timeout.
- Switch only while the motor is stopped, then call `reset_position()`.
