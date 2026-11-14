# Limitations and technical review findings

[README](../README.md) · [Mathematics](mathematics.md) · [Testing](testing.md)

The source remains unchanged. The following are source-derived constraints, engineering risks or missing validation, not experimentally measured defects unless explicitly stated.

## Estimation and geometry

- **Linear acceleration:** the 0.80–1.20 g correction gate rejects some disturbed measurements, but motion can tilt the apparent gravity direction while staying inside the gate. Rejected acceleration leaves gyro-only integration.
- **Drift and heading:** fixed startup gyro bias does not compensate temperature changes or later bias variation. Gravity cannot correct yaw. The solver uses gravity vectors and does not need absolute heading for its ideal tilt objective, but this does not provide yaw stabilization.
- **Numerical drift:** successive finite-precision matrix products have no explicit re-orthonormalization. Long-duration orthogonality error has not been measured. `gravityInSensor()` normalizes its output, whereas the filter's predicted vector is not explicitly normalized.
- **Axis alignment:** the base FRD mapping is marked approximate in the source. Misalignment affects both the solver and feedback. The two mounting-offset constants are zero and are simple Euler offsets, not a general mounting rotation.
- **Geometry:** orthogonal axes, the outer-roll/inner-pitch sequence, aligned coordinate frames and the upright solution branch are assumptions. Off-axis mounting, skewed axes and combined-axis behavior need external measurement.
- **Euler limitations:** pitch extraction is restricted to ±90°, and roll/pitch feedback becomes ill-conditioned near vertical pitch. Body X/Y rates are not exact roll/pitch derivatives at general combined attitude.

## Actuation and tuning

The code provisionally interprets physical geometric degrees as servo-command degrees at 1:1 scale. Servo horn zero, direction, usable pulse range, angular response, load dependence and backlash are unmeasured. `Servo.attach(pin)` uses the Servo library defaults rather than measured endpoint pulse widths. D10 is roll and D9 is pitch in software, but wiring and direction need physical verification.

The gains (FF 0.85, P 0.30, D 0.050 on each axis) are supplied values, not the result of a documented tuning study. There is no integral term. Residual error, saturation behavior and oscillation risk require testing rather than inference from the gains alone. The 80°/s limit controls command change, not measured mechanical speed. Integer `Servo.write` inputs quantize commands. Startup and `0` bypass the slew limiter.

Photos show exposed jumper wiring. Wire torque, flexible attachments, mechanical play and servo imperfections are plausible disturbance sources; their effects were not isolated experimentally. Payload, torque reserve, speed, power ripple, stall current and runtime are unmeasured.

## Exact operating gates

| Gate | Supplied code |
|---|---|
| Reference-relative base roll/pitch | Strictly greater than 90° triggers stop/block |
| Base normalized down component | `g.z < 0.20` rejected; implies approximately 78.46° maximum total absolute tilt for this gate alone |
| Other geometry | Norm ≥0.8, YZ magnitude ≥0.2, target Y fraction in ±1.001 before clamp, solved joint angles in ±89° |
| User `t` targets | Each coordinate in inclusive ±30°; geometry must also be reachable |
| Roll goal | Inclusive 45–180 command degrees |
| Pitch goal | Inclusive 15–165 command degrees |

These intersecting limits are not a guaranteed operating envelope. A valid target at one base pose can become unreachable at another. Both the held-reference and requested-target solves must succeed. Mechanical obstruction may occur within software limits.

## Software review items requiring separate approval before alteration

1. **Reference capture is not transactional.** `saveReferences()` overwrites base/platform references and reference commands before attempting the geometric solve. If the solve fails, several references have already changed, while target/geometric-reference state may remain from the preceding capture. A failed `z` should not be assumed to leave the old calibration intact. Investigate a future change that computes all candidates locally and commits them only on success.
2. **I²C timeout is not enabled.** Recovery handles reads that return failure, not a bus/library call that never returns. Enabling and validating an appropriate Wire timeout would require a separate code change and hardware fault tests.
3. **Buffered commands affect stop handling.** `s` is immediate only at an empty parser buffer. A pending target line or overlength input can alter interpretation. Consider a separately reviewed stop/parser redesign.
4. **Recovery uses estimated health, not restored attitude accuracy.** Five valid reads or 800 ms of clean samples are heuristic recovery thresholds. Motion during an outage is not integrated, and gravity correction cannot restore heading. Automatic resumption under real fault conditions has not been demonstrated.
5. **Target limits are not universal.** Startup and `z` copy current attitude without the ±30° `setTarget()` limit. Arming does not pre-check all geometric/servo goals; those checks run during control.
6. **Runtime margins remain unmeasured.** The build leaves 1,264 bytes beyond global allocations, but stack usage, matrix temporaries, worst-case loop time and serial-print timing on the Uno have not been instrumented. Successful linking alone does not establish those margins.

## Validation priorities

First document power and measure sensor/joint alignment and servo mappings. Then collect synchronized base/platform telemetry plus an independent angle reference for single-axis, combined-axis and target-step trials. Record repetitions, payload and supply conditions. Only afterward assess error statistics and tune gains. Test sensor disconnection, reconnect, timing faults and boundary behavior with physical restraint and controlled travel. These are proposed tests, not completed results.
