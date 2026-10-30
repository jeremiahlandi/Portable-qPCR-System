# Portable qPCR: feature requirements

Derived from two sources:

- **Legacy**: what the Kalix Labs firmware (`Firmware/Legacy_Archive`) did or tried to do. See `LEGACY_FIRMWARE_REVIEW.md`.
- **Research**: the Bio-Rad Real-Time PCR Applications Guide (Bulletin 5279), the Sensors 2010 system-modeling paper, and the Lee/Chen/Lee Biosensors review.

Each item is tagged with its origin and a priority:

- Origin: **L** = legacy had it, **LB** = legacy had it but broken, **R** = new from research, **N** = new engineering need.
- Priority: **P1** = required for the first pilot units, **P2** = required for a sellable instrument, **P3** = later.

This document is board-agnostic. Nothing here assumes the Octopus, the ESP32-P4 or the Nano ESP32.

## 1. Protocol model

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 1.1 | Protocol is an ordered list of steps; each step has target temperature, hold time, optional ramp-rate limit, and an optional fluorescence-read flag | R | P1 |
| 1.2 | Loop construct: repeat a group of steps N times (cycles) | L, R | P1 |
| 1.3 | Melt-curve step: start temp, end temp, increment (down to 0.2 °C), hold per increment, read at every increment | R | P2 |
| 1.4 | Lid pre-heat step before the first block step, with wait-for-lid-at-temperature | L | P1 |
| 1.5 | Built-in protocol templates matching the legacy run types: reverse transcription (1 to 3 holds), PCR 2-step and 3-step with optional activation, initial hold and final hold, extraction (1 hold) | L | P1 |
| 1.6 | Long initial hold support (e.g. 95 °C for 3 to 10 min hot-start) | R | P1 |
| 1.7 | Preset protocol library keyed by assay or disease, stored on device, editable | L (SD `kalds.txt`) | P1 |
| 1.8 | Protocols are data (file or structured record), not code paths; versioned with a schema number | N | P1 |
| 1.9 | Protocol validation before start: temperatures within block range, hold times sane, loop counts bounded | N | P1 |
| 1.10 | Estimated total time and remaining time, updated during the run | L | P1 |
| 1.11 | Pause and resume that actually holds the current temperature and stops the clock | LB | P1 |
| 1.12 | Abort from UI and from app, with controlled cooldown | L | P1 |

## 2. Thermal control

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 2.1 | Block temperature control with both heating and cooling (bidirectional Peltier drive via H-bridge) | LB (cool only on ramps) | P1 |
| 2.2 | Block sensor resolution 0.1 °C or better, accuracy ±0.25 °C after calibration at 60 and 95 °C | R | P1 |
| 2.3 | Block temperature range: ambient to 100 °C working, 105 °C ceiling | R | P1 |
| 2.4 | Ramp rate at least 1.5 °C/s heating and 1 °C/s cooling on the first units; target 3 °C/s | R | P1 / P2 |
| 2.5 | Overshoot at a hold below 0.5 °C, settle within a defined time | R | P1 |
| 2.6 | Well-to-well uniformity ±0.5 °C if more than one well | R | P2 |
| 2.7 | Lid heater control to a configurable setpoint above 100 °C (default 105 °C) with its own sensor | LB (fixed 60 °C) | P1 |
| 2.8 | PID with input smoothing, integral windup limit, max-power limit, and separate tunings for heat and cool (ported from Klipper `heaters.py`) | N | P1 |
| 2.9 | PID autotune routine on the device (ported from Klipper `pid_calibrate.py`) | N | P2 |
| 2.10 | Fixed-rate control loop (10 to 20 Hz), no blocking calls in the control path | N (legacy ran ~3 Hz with blocking) | P1 |
| 2.11 | Temperature calibration stored in non-volatile memory with offset and gain per sensor | N | P1 |
| 2.12 | Peltier driven with filtered, low-ripple current (PWM at 20 kHz plus LC filter) | N | P1 |

## 3. Safety

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 3.1 | Sensor fault detection on every sensor: open, short, NaN, out-of-range, stuck value. Any fault trips a safe state | LB (open thermocouple caused full-power heating) | P1 |
| 3.2 | Hard overtemperature cutoff on block and lid that no UI or protocol state can disable | LB (gated on a UI flag) | P1 |
| 3.3 | Heater-verification check: if an output is on and the temperature is not rising at the expected rate, trip (ported from Klipper `verify_heater.py`) | N | P1 |
| 3.4 | Hardware watchdog; outputs default off on reset or brownout | N | P1 |
| 3.5 | Output refresh timeout: any PWM output not refreshed by the control loop within a window turns off (Klipper `max_duration` concept) | N | P1 |
| 3.6 | Safe state: all outputs off, fans on, visible and audible alarm, reason logged, app notified | L (partial) | P1 |
| 3.7 | Fault recovery requires explicit acknowledgement, not a power cycle alone, and the reason is shown | LB (`while(1)`) | P1 |
| 3.8 | Lid-open interlock pauses the run and stops the lid heater | N | P2 |
| 3.9 | Fused supply, reverse polarity protection, overcurrent on the Peltier bridge | N | P1 |
| 3.10 | Self-test at boot: sensors plausible, outputs respond, memory and storage ok | N | P2 |

## 4. Optics (the qPCR part)

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 4.1 | At least one fluorescence channel for FAM/SYBR Green (excitation ~470 to 490 nm, emission ~520 nm) | R (legacy had none working) | P1 |
| 4.2 | Second channel for an internal control (HEX/VIC ~535/556 nm or Cy5 ~650/670 nm) | R | P2 |
| 4.3 | Excitation LED under firmware control with a filtered excitation path; emission filter on the detector | N | P1 |
| 4.4 | Read at the step flagged in the protocol (typically end of anneal/extend), once per cycle per well per channel | R | P1 |
| 4.5 | Dark reading (LED off) subtracted from every read | N | P1 |
| 4.6 | Detector integration time and gain configurable and stored with the data | N (legacy set ATIME by truncation accident) | P1 |
| 4.7 | Melt-curve reads at every increment of a melt step | R | P2 |
| 4.8 | Per-well addressing: either one detector per well or a scanned head over a row of wells | N | P1 (design decision) |
| 4.9 | Baseline subtraction and threshold Ct calling on the device | R | P1 |
| 4.10 | Second-derivative Ct as an alternative; efficiency from a standard curve | R | P3 |
| 4.11 | Qualitative call per sample: positive, negative, invalid (internal control failed), with configurable thresholds | R, L (vet diagnostic use) | P1 |
| 4.12 | No-template control handling: NTC wells flagged, run invalid if NTC amplifies | R | P1 |

## 5. Data and records

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 5.1 | Run record: start and end timestamps, protocol used (with version), operator, device serial, firmware version | L (date, animal, disease log) | P1 |
| 5.2 | Sample metadata per well: sample ID, animal ID, assay or disease code, control type | L (`animal`, `disease`, `num_set`) | P1 |
| 5.3 | Raw per-cycle, per-well, per-channel fluorescence with block temperature at each read, stored for every run | R | P1 |
| 5.4 | Temperature log at 1 Hz for the whole run (block, lid, output duty) for diagnostics | N | P1 |
| 5.5 | Results: Ct per well per channel, call per sample | R | P1 |
| 5.6 | Export as CSV (or JSON) over the app link and to removable storage | L (SD) | P1 |
| 5.7 | Real-time clock or time sync from the app; runs are timestamped correctly | L (DS1307) | P1 |
| 5.8 | Fault and event log retained across power cycles | N | P1 |
| 5.9 | Storage full and corruption handling | N | P2 |

## 6. User interface

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 6.1 | Full color touchscreen | L (Nextion), Kalix request | P1 |
| 6.2 | Home screen: date, time, block and lid temperature, status, start | L (page1) | P1 |
| 6.3 | Protocol select and edit screens for each run type; preset library | L (set pages) | P1 |
| 6.4 | Run screen: stage name, heating/cooling/hold trend, block and lid temps, cycle n of N, step and total time remaining, progress | L (page36) | P1 |
| 6.5 | Live amplification curve during the run | R | P2 |
| 6.6 | Results screen: curves, Ct, call per sample, export | R | P1 |
| 6.7 | Pause, resume, abort with confirmation | LB | P1 |
| 6.8 | Fault screen with reason and acknowledgement | N | P1 |
| 6.9 | Settings: lid setpoint, calibration, network, time, device info | N | P1 |
| 6.10 | Status LEDs (heating, cooling, hold, done, fault) and buzzer for done and fault | L | P1 |
| 6.11 | Diagnostics screen: live sensor values, output duty, manual output test, self-test | N | P2 |

## 7. Connectivity

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 7.1 | Phone or tablet link over WiFi (device-hosted access point or joined network); BLE optional | L (Bluetooth Classic, not available on current chips) | P1 |
| 7.2 | Framed, checksummed, versioned message protocol; no magic terminator bytes | LB | P1 |
| 7.3 | Message set preserving the legacy semantics: configure run, start, status every second, pause, resume, abort, run complete, results | L | P1 |
| 7.4 | Loss of link never affects a running protocol; reconnect resumes status | LB (blocking loops) | P1 |
| 7.5 | Firmware update over the link or from removable storage | N | P2 |
| 7.6 | Time sync from the app | N | P1 |

## 8. Platform and software structure

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 8.1 | Board-agnostic control core in plain C++ with a hardware interface (sensors, outputs, time); builds for STM32 and ESP32 targets | N | P1 |
| 8.2 | Core unit-tested on the host (PlatformIO native) with a replay test against the Klipper Python originals for the ported algorithms | N | P1 |
| 8.3 | Modules: sensors, actuators, thermal control, safety, protocol runner, optics, data, UI, comms; no global-variable state machine | N (legacy was ~300 globals) | P1 |
| 8.4 | CI build for every target on every push (extend the existing GitHub workflow) | N | P1 |
| 8.5 | License decision recorded (GPLv3 if Klipper code is ported) | N | P1 |
| 8.6 | Single controller; no inter-MCU byte handshake. If a separate display or radio head is used, it talks a standard line protocol | N | P1 |

## 9. Validation (definition of done for the instrument)

| # | Requirement | Origin | Pri |
| --- | --- | --- | --- |
| 9.1 | Thermal: measured ramp rates, overshoot, settle time and hold stability against an external reference probe in a tube | R | P1 |
| 9.2 | Thermal uniformity across wells with reference probes | R | P2 |
| 9.3 | Ten-fold dilution series in triplicate: Ct spacing ~3.32 cycles, slope -3.1 to -3.6, efficiency 90 to 105 %, R² > 0.98, replicate Ct SD ≤ 0.25 | R | P1 |
| 9.4 | No-template control flat; SYBR melt shows a single peak | R | P1 / P2 |
| 9.5 | Safety tests: unplug each sensor during a run, short each sensor, stall the heater, pull power mid-run. Every case ends in the safe state with a logged reason | N | P1 |
| 9.6 | Link tests: disconnect the app mid-run, send malformed frames; run unaffected | N | P1 |

## 10. Explicitly out of scope for the first units

- Annealing temperature gradient across the block (96-well instrument feature).
- More than two optical channels.
- Classic Bluetooth compatibility with the old Kalix app.
- Cloud sync and user accounts (can sit in the app, not the device).
- On-device machine-learning classification (the legacy TensorFlow Lite stubs).

## 11. Open decisions that shape the hardware

1. Well count and format: single 0.2 ml tube, strip of 8, or small block. Drives optics (4.8), uniformity (2.6) and Peltier size.
2. Block sensor: PT1000 with MAX31865 (recommended) versus precision thermistor.
3. One detector per well versus a scanned head.
4. WiFi only, or WiFi plus BLE.
5. GPL acceptance for ported Klipper code.
6. Controller board: Octopus STM32, ESP32-P4, or Nano ESP32. The core (8.1) is built so this can be decided last.
