# Veloster N ESP32-S3 OBD Gauge

A DIY OBD gauge for a 2018 Hyundai Veloster N with a manual transmission. An ESP32-S3 drives a 3.5-inch ILI9488 display to show vehicle data.

## Photos and video

![Gauge operating in the vehicle](media/gauge-driving.png)

[Watch or download the operation video (MP4)](media/gauge-operation.mp4)

The photo and video are the original files supplied by the owner. The photo shows the central gear indicator, RPM, vehicle speed, tire pressures and temperatures, and engine-related readings. The factory speedometer and the gauge show different speeds in this photo; this alone does not establish calibration accuracy. The video is included as supplied and has not undergone frame-by-frame validation. File hashes and the video duration are listed in the [media manifest](media/manifest.json).

## Hardware

- ESP32-S3 N16R8 development board
- 3.5-inch 480 × 320 SPI TFT with an ILI9488 controller
- Existing 3.3 V CAN transceiver module
- Vehicle 12 V supply through the existing 5 V BEC regulator
- OBD CAN: 11-bit identifiers at 500 kbit/s; standard engine response ID `0x7E8`
- Active diagnostic requests; this configuration differs from the earlier RX-only logger

## Displayed data

| Reading | Implementation and limitations |
|---|---|
| RPM and vehicle speed | Combined queries for standard OBD PIDs `0C` and `0D` |
| ACC | Accelerator pedal PID normalized from raw 38–213 to 0–100%; not throttle plate position |
| REL boost | `(MAP − BARO) / 100 + 0.60` bar, with MAP and BARO in kPa. The owner-selected display offset is not a verified match to the factory boost gauge |
| Coolant, intake temperature, ECU voltage | Supported ECU readings. ECU voltage is not an independent measurement at the battery terminals |
| Oil temperature | Reference reading derived from an observed extended diagnostic response |
| Tire pressures and temperatures | Reference readings from extended TPMS responses; wheel mapping, conversion, and sensor update latency have validation limits |
| Forward gear | Estimated from the RPM-to-speed ratio; not a confirmed gear position sensor reading |
| Reverse | Based on an observed diagnostic response candidate |
| Brake | ON/OFF reference indication, not hydraulic pressure in bar |

Unconfirmed neutral, continuous clutch position, brake pressure, and driving mode data are not presented as supported features. The tire size used for gear estimation is **235/35 R19**.

## Wiring

![Development-board wiring overview](docs/wiring.svg)

See the [wiring notes](docs/wiring.md) for connections and power conditions. This diagram covers the current development-board assembly, not the separate integrated PCB design.

## Firmware status — November 2, 2026

| Version or feature | Status |
|---|---|
| Last firmware flashed to the board | `20261102_r8_sweep`: a brief screen-off transition was observed after the sweep and before CAN data appeared |
| Next firmware prepared | `20261102_r8_canfirst`: confirm CAN/RPM → startup sweep → live readings. Host-tested and built, **not yet flashed** |
| Sleep in the prepared version | Enter sleep after fresh RPM and speed readings remain zero for 5 seconds; remove the redundant 3-second delay after confirmed readiness |
| Ignition-on waiting | Only while valid zero-RPM replies continue: at least 5 seconds between checks, for up to 2 minutes. No-response and fault limits remain in place |

The firmware version shown in the supplied media has not been established from the photo or video. The media is not evidence that the unflashed version works on hardware.

Five host sleep test scripts, 19 existing regression test scripts, and builds for both the development-board and PCB configurations passed. Additional seven-day policy simulations confirmed no further transmission after three unanswered boot probes, and a maximum of 24 queries with continuous valid zero-RPM replies. These were simulations, not current measurements or seven-day vehicle tests.

## Remaining validation

- Actual sweep-to-live transition and repeated engine-stop/start recovery with the CAN-first version
- Total standby current at the 12 V input, using vehicle power with USB disconnected
- Reducing unnecessary timer wakeups during long parking periods
- Comparing boost, tire, and temperature readings against the factory displays

A dark screen does not mean the entire assembly is powered off. The development board, BEC, and CAN module still draw standby current, so long-term parking safety has not yet been established.

This repository contains operation notes, media, and wiring documentation. It does not include the complete development environment, raw vehicle logs, ECU identity records, personal filesystem paths, or credentials.
