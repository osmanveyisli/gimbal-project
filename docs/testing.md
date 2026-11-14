# Testing and evidence record

[README](../README.md) · [Media](../media/README.md) · [Build validation](validation.md)

Review date: 14 November 2026. Evidence consists of the final source, two supplied photographs visible in the conversation, and two uploaded demonstration clips. No serial logs, independent angle measurements, test protocol, calibration dataset or firmware recording identifier was supplied.

## Observed demonstration procedure

Both videos show an operator holding and changing the orientation of the assembled mechanism over a workbench. Sensor indicator lights are illuminated, jumper wires remain attached, and the platform's orientation relative to the surrounding frame changes. No commanded target, serial mode selection, synchronized reference measurement or controlled disturbance magnitude is displayed.

The review used decoded frames sampled every 0.5 s across each complete clip, with additional larger frames for hardware inspection. This is a visual evidence review, not a frame-by-frame motion measurement or audio analysis.

| Source | Duration | Visual observations and limits |
|---|---:|---|
| `WhatsApp Video 2026-11-14 at 13.10.17.mp4` | 13.933 s | Early frames show the front of the platform; the assembly is subsequently tilted and turned substantially, with the inner plate changing pose relative to the outer structure. Camera orientation also changes. The clip does not establish a constant world-relative platform angle. |
| `WhatsApp Video 2026-11-14 at 13.30.04.mp4` | 18.033 s | The frame is rocked by hand through several positions and moved toward the workbench. Relative platform/frame motion is visible. No numerical residual error, response time or active-axis identification can be extracted reliably from this footage alone. |
| Supplied overhead photograph | Still image | Shows the frame, platform-mounted sensor module, second sensor module and nearby controller/wiring. Cannot establish every wire's destination or supply settings. |
| Supplied front/side photograph | Still image | Shows the nested mechanism, platform sensor and servo/frame arrangement, with a hand supporting the assembly. A static photo does not demonstrate stabilization. |

The recorded behavior is qualitatively consistent with an operating articulated prototype. It cannot distinguish all contributions of active control, hand contact, camera motion and mechanism motion. The builder reports a working prototype; this review does not independently establish that the final uploaded firmware was the version running in the clips.

## Completed checks

- Source preserved byte for byte and SHA-256 recorded.
- Original sketch compiled and linked for `arduino:avr:uno`; warnings and memory figures recorded.
- Documentation traced to source functions, constants and conditions.
- Media dimensions, durations and container metadata inspected.

See [validation](validation.md) for the final link, formatting and source-integrity checks. No physical upload, closed-loop hardware test, fault injection or instrumented accuracy test was performed during documentation work.

## Claims not established

No numerical stabilization accuracy, settling time, overshoot, bandwidth, payload capacity, disturbance rejection ratio, maximum validated tilt or battery runtime is claimed. Successful independent-axis or combined-axis tracking, arbitrary target-angle accuracy, automatic recovery and exact cutoff behavior are not experimentally verified by the supplied clips.

## Proposed reproducible test plan — not yet executed

1. Record firmware hash, component identities, supply voltage, payload, sensor alignment and servo calibration.
2. Fix an independent angle reference or calibrated optical measurement; log timestamps and both IMUs, targets, goals, commands and mode.
3. Measure stationary drift and repeat small roll-only and pitch-only base disturbances at fixed targets.
4. Repeat combined-axis base disturbances and target steps within measured mechanical travel, including different initial references.
5. Report measured platform error relative to the independent reference, repeatability, settling criteria and test duration. Keep single-axis and combined-axis results separate.
6. Test reachable/unreachable targets, software boundary conditions and brief/sustained sensor failures. Confirm whether the retained command and automatic recovery are mechanically acceptable.
7. Measure power integrity and current under representative load; inspect wiring interference and joint play.

Suggested log columns: `timestamp_us, firmware_sha256, mode, base_roll, base_pitch, platform_roll, platform_pitch, target_roll, target_pitch, goal_roll, goal_pitch, cmd_roll, cmd_pitch, external_roll, external_pitch, supply_V, payload_g`. Some columns require extra instrumentation or a separately approved logging change; the existing serial output does not provide all of them.
