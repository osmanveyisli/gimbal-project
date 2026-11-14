# Operation and serial interface

[README](../README.md) · [Limitations](limitations.md)

## Startup

1. Verify mechanical clearance, sensor mounting and power connections. Startup can move both servos to 90°.
2. Keep the mechanism stationary. After sensor configuration and servo centering, gyro calibration waits 1 s then collects 500 paired samples with 5 ms delays. Actual total time includes I²C work.
3. Initialization accepts acceleration in `[0.85,1.15] g`, builds both matrices and captures a geometric reference. Failure prints an error and halts; calibration/orientation failure detaches both servos.
4. Open the serial monitor at **115200 baud**. Wait for `READY / HOLD` and at least 20 good readings.
5. If needed, send `0`, let the mechanism settle, then `z`. Send `l` for a gravity-level target, and `a` to enable both axes. Verify servo direction using small controlled motions.

`z` captures the current attitude, not necessarily level. The default mounting offsets are zero. A target command changes the target but does not select an automatic mode.

## Commands

| Input | Preconditions and effect |
|---|---|
| `s` | Selects HOLD; stops both automatic updates, retaining servo PWM |
| `0` | HOLD only; writes 90° immediately to both servos and resets goals/command states; references are not recaptured |
| `z` | HOLD only; no sensor pause, ≥20 good reads, and absolute value of all three gyro components ≤1.5°/s on both IMUs; captures references and current-attitude target |
| `l`, `L` | Sets target 0° roll, 0° pitch if sensor data are healthy and geometry allows it |
| `t <roll> <pitch>` | Also accepts uppercase `T`; requires CR or LF terminator; healthy data, each angle in inclusive ±30°, and reachable geometry |
| `r` | From HOLD: enables roll updates only |
| `p` | From HOLD: enables pitch updates only |
| `a` | From HOLD: enables both axes |

Arming needs ≥20 good readings, no pause and base reference-relative angles inside limits. It does not require stillness, and geometry/range validation is completed on the next control update. To switch automatic modes, send `s` first. In single-axis mode the inactive servo retains its command; two-axis target feasibility is still checked, so single-axis mode cannot generally achieve an arbitrary roll/pitch pair.

Examples, one line at a time:

```text
s
0
z
l
a
t 10 -5
s
```

Allow settling between `0` and `z`; do not paste the entire example as a timing-safe sequence. `t` must end with newline. One-letter commands execute immediately only when the serial buffer is empty. If an incomplete `t` line is pending, send a newline to clear/complete it before `s`. The 40-byte line buffer accepts up to 39 characters; overflow resets it, but remaining characters in that same input stream can be interpreted as new commands. This is a parser limitation, not a verified hardware failure.

## Telemetry

| Field | Meaning |
|---|---|
| `HOLD`, `ROLL`, `PITCH`, `BOTH` | Selected user mode |
| `/SENSOR_PAUSE`, `/SENSOR_HOLD` | Temporary or sustained sensor pause appended to the mode |
| `baseR`, `baseP` | Wrapped base angles relative to the saved reference, degrees |
| `errR`, `errP` | Wrapped platform sensor attitude minus target plus mounting-offset reference, degrees |
| `targetR`, `targetP` | User target in physical-platform convention, degrees |
| `goalR`, `goalP` | Last accepted servo goals, command degrees |
| `cmdR`, `cmdP` | Internal slew-limited commands, not measured joint positions |
| `acc` | Base acceleration magnitude, then platform magnitude, in g |

`errR/errP` mean measured angle minus `(target + mounting offset)`. On a read-failure path, normal status output is skipped, so missing telemetry is not evidence of a stable attitude. A rejected goal may leave telemetry showing the previous accepted goal.

## Fault behavior

| Condition | Response |
|---|---|
| Either MPU burst read fails, sensor `dt ≤0` or `dt ≥0.10 s` | Freeze automatic command updates; reset control timing |
| Control `dt ≤0` or `dt >0.10 s` | Enter sensor-pause recovery path |
| Five consecutive valid samples while total pause age ≤100 ms | Quick recovery if no sustained-fault flag |
| Clean interval ≥800 ms and ≥20 good reads | Stable recovery; prior mode resumes |
| Unresolved pause age ≥1,000 ms | Mark sustained fault; still recoverable |
| Absolute wrapped base roll or pitch deviation >90° | Block arming or latch HOLD |
| Either required geometric solve fails | Latch HOLD |
| Either active goal outside its command range | Latch HOLD before either servo write |

Recovery conditions are evaluated in software time; a single bad sample restarts the clean interval. During a pause, sensor estimation continues on accepted samples, but no missing angular motion is reconstructed. The `0` command in HOLD still explicitly writes servo positions during a sensor pause. Sensor-pause “freeze” therefore means automatic updates, not an absolute prohibition on manual centering.

HOLD does not detach servos. An I²C operation that blocks indefinitely may prevent serial processing; the sketch does not enable Wire's optional timeout. Do not treat `s` as an independent hardware emergency stop.
