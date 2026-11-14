# Validation record

[README](../README.md) · [Compiler output](../validation/compile-uno.log) · [Source checksum](../validation/source.sha256)

Date: 14 November 2026. Target board: Arduino Uno R3 / ATmega328P, FQBN `arduino:avr:uno`. Validation was performed in a Linux x86-64 workspace without attached hardware.

## Source preservation

The `.ino` is an exact byte-for-byte copy of the supplied final pasted sketch. No control constants, geometry, sensor mappings, fault behavior, comments or formatting were changed. The file begins with the same leading newline as the upload.

SHA-256:

```text
051eebb4c735de2965e74301c238f0f4300ab94406b3457afe61fcfbf4e1b39d
```

Verify from repository root:

```sh
sha256sum -c validation/source.sha256
```

## Compilation

**Result: compilation and linking succeeded for the unchanged sketch.** An ELF/HEX build was produced in a scratch build directory; generated binaries and toolchains are not committed.

| Component | Version |
|---|---|
| Arduino CLI | 1.6.0, commit `50a0e80da` |
| Arduino AVR Boards | 1.8.8 |
| Compiler | avr-g++ (GCC) 7.3.0; package `7.3.0-atmel3.6.1-arduino7` |
| Servo library | 1.3.0 |
| Wire | Bundled with AVR core (library metadata 1.0) |
| Warning level | `all` |

Actual build command (workspace paths shown for reproducibility):

```sh
/workspace/work/tools/arduino-cli \
  --config-file /workspace/work/arduino-cli.yaml \
  compile --fqbn arduino:avr:uno --warnings all \
  --build-path /workspace/work/uno-build \
  /workspace/work/arduino-2axis-gimbal-stabilizer/firmware/gimbal_stabilizer
```

The CLI configuration redirected its data, downloads and user directories into the writable workspace. CLI-managed index downloads initially failed with connection-refused errors. Official Arduino package/library indexes and archives were downloaded with `curl`; archive SHA-256 values were checked against the indexes before installation into the configured directories. The installed board core and libraries were then used by the actual Arduino CLI build. This was not a host-only syntax check.

For a normally configured environment, the portable commands are:

```sh
arduino-cli core update-index
arduino-cli core install arduino:avr@1.8.8
arduino-cli lib install Servo@1.3.0
arduino-cli compile --fqbn arduino:avr:uno --warnings all firmware/gimbal_stabilizer
```

| Build metric | Result |
|---|---:|
| Program storage | 25,430 / 32,256 bytes, 78% |
| Global variables | 784 / 2,048 bytes, 38% |
| RAM remaining beyond globals | 1,264 bytes; runtime stack headroom is not measured |
| Errors | 0 |
| Warnings | 4 unused `tag` parameters in AVR core `new.cpp`, lines 59, 68, 103, 106 |
| Sketch warnings | None reported |

The [stored log](../validation/compile-uno.log) retains compiler messages and memory output, with only the machine-specific Arduino data prefix replaced by `<ARDUINO_DATA>`. Compilation confirms that this source builds with the stated environment. It does not prove hardware operation, runtime timing, stack safety, correct wiring or stabilization performance.

## Documentation and publication checks

The documentation records actual constants, strict/inclusive boundary conditions, the noncommutative joint model, body-rate damping, target/reference semantics, parser caveats and recovery behavior. English and Azerbaijani versions have reciprocal navigation. The final QA results are recorded in [repository-qa.txt](../validation/repository-qa.txt).

Repository creation was attempted using the authorized GitHub account, but GitHub rejected `CreateRepository` because the token lacks permission. The intended repository name is `arduino-2axis-gimbal-stabilizer`; no successful remote publication is claimed until the repository exists and the uploaded commit can be verified. People-containing media is prepared locally and remains subject to the owner's permission confirmation.

## Not performed

No physical upload, calibrated angle measurement, electrical measurement, runtime profiler, stack watermarking or fault-injection test was performed. Videos cannot verify firmware identity or numerical accuracy. Proposed repairs in [limitations](limitations.md) have not been applied.
