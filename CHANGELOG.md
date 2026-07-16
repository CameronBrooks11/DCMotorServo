# Changelog

All notable changes to this library are documented here. Follows
[Keep a Changelog](https://keepachangelog.com) and
[Semantic Versioning](https://semver.org). History before 1.0.1 was
reconstructed from the git log.

## [1.1.0] — 2026-07-16

Verified by code review and compile checks; hardware bench validation
is tracked in
[#3](https://github.com/CameronBrooks11/DCMotorServo/issues/3).

### Added

- Extrema sensing on `DCMotorServo` (closes #1):
  - software travel limits — `setTravelLimits()` / `clearTravelLimits()`
    clamp `move()`/`moveTo()` targets in encoder counts
  - physical endstops — `attachEndstops()` callbacks; while the target lies
    beyond a triggered stop, `run()` holds the motor braked and pulls the
    target to the held position (stable hold, no chatter, no lunge on
    switch release)
  - encoder-based stall detection — `enableStallDetection()` /
    `disableStallDetection()` / `isStalled()` / `clearStall()`; latched
    fault when the motor is driven but the encoder does not advance
  - non-blocking homing — `startHoming(direction, pwm, max_travel)` /
    `isHoming()` / `isHomed()`; terminates on the matching endstop or a
    stall (sensorless), zeroes the encoder at the extreme; `max_travel`
    failsafe aborts unhomed on a dead switch
- `l298n_homing` usage demo; "Extrema sensing" section in `docs/api.md`.

### Changed

- `stop()` also cancels an in-progress homing move, syncing the target to
  the current position.
- `DCMotorTacho::run()` holds its speed loop while the wrapped servo is
  homing or stall-latched (no setpoint windup against a frozen encoder, no
  silent homing cancellation), resuming automatically afterwards.

### Fixed

- `finished()` PWM check was vacuous (`_PWM_output` was never written in
  `run()`); now updated each cycle and homing-aware.
- Zero PID output was mapped to `+pwm_skip` drive instead of no drive
  (jolt after `stop()`).
- Constructor initializer-list order (`-Wreorder` warning).

### Docs

- README rewritten for the driver-agnostic API (the old pin-based
  sections were stale).
- Example driver-library dependencies (L298N, LMD18200) documented.
- Tuning walkthrough READMEs: servo PID (03), rotational/CPR-fudge
  calibration (04), tacho procedure incl. speedInterval selection, and
  speed-PID tuning.
- Config sketches refactored around a per-sketch `baseSpecs.h` spec
  sheet (driver, pins, motor specs, determined values) with a
  documented fill-in-and-copy-forward workflow; drifted tuning values
  unified to the reference rig's.
- `TODO.md` and the README TODO section retired; all outstanding work
  tracked as GitHub issues.

## [1.0.1] — 2026-03-15

First published release (PlatformIO registry: `cameronbrooks11/DCMotorServo`).

### Added

- PlatformIO manifest (`library.json`) and registry publication.
- Docs suite: `getting-started.md`, `tuning.md`, `api.md`.

## [1.0.0] — 2026-02

Driver-agnostic rewrite of the original pin-based library.

### Changed

- Hardware interface replaced with four function pointers
  (`MotorWriteFunc`, `MotorBrakeFunc`, `EncoderReadFunc`,
  `EncoderWriteFunc`) — any driver/encoder library works without
  touching the control code.

### Added

- `DCMotorTacho`: cascaded speed-control (RPM) loop wrapping the
  position loop.
- Guided configuration examples (`servo_config` 01–04, `tacho_config`)
  and driver-specific usage demos (L298N, LMD18200).

## Pre-1.0

Fork-and-continuation of
[julester23/DCMotorServo](https://github.com/julester23/DCMotorServo)
(pin-based API, PID + Encoder feedback), inspired by AccelStepper.
