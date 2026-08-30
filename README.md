# Intelligent Remote-Controlled Antenna Matrix for Shortwave Radio

## Overview

This bachelor-thesis project implements an **8-to-2 remotely controlled antenna matrix for HF radio (1.8–30 MHz)**. It combines relay-based RF routing, grounded PARK states, local touchscreen control, Blynk remote control, software interlocks, watchdog supervision, and forward/reflected detector acquisition.

The completed device is an engineering prototype. Its switching, control, safety-state logic, raw detector acquisition, and selected RF paths have been validated at bench level. Calibrated power/SWR accuracy and continuous 1 kW operation have not yet been qualified.

![Final antenna-matrix prototype](images/matrix.jpeg)

## Original Design Objectives

The project was developed against the following requirements:

- 8 HF antenna ports and 2 transceiver ports
- operation from 1.8 to 30 MHz in a 50 Ω system
- 1 kW continuous-power design objective
- VSWR ≤ 1.10 and insertion loss ≤ 0.20 dB targets
- forward and reflected power feedback for both transceivers
- local control through a CYD ESP32 touchscreen
- remote control through Blynk
- safe BUS1, BUS2, and grounded PARK states
- watchdog and interlock functions

These are **design objectives**, not a statement that every value has been achieved or qualified. The measured prototype performance is reported separately below.

## Implemented Features

- 8 antenna inputs and 2 transceiver outputs
- relay-based BUS1/BUS2 routing
- fail-safe grounded PARK state for unused antennas
- central state manager shared by touchscreen, Blynk, and serial control
- software interlocks that reject conflicting assignments
- manual TX1/TX2 locks that block switching on the corresponding bus
- local CYD ESP32 touchscreen interface
- Blynk remote dashboard with synchronized state
- two Bruene directional couplers
- four ADS1115 channels for TX1/TX2 forward and reflected detector voltages
- provisional power estimation and SWR calculation
- watchdog and Wi-Fi reconnection handling
- separate RF and control PCBs in a metal enclosure

Frequency measurement appeared in an earlier design revision but was removed from the final implementation.

## System Architecture

```text
8 HF antennas
      |
      v
Selector + ground-clamp relay matrix ----> BUS1 / BUS2
                                               |
                                               v
                                      Bruene couplers
                                               |
                                Forward/reflected DC outputs
                                               |
                                               v
                                          ADS1115 ADC
                                               |
                                               v
                                         CYD ESP32
                                      /      |       \
                              Touchscreen  Blynk  Relay control
                                                   |
                                                   v
                                         MCP23017 + ULN2803
```

## Hardware

### RF section

- TE Connectivity Axicom IM06DGR 12 V DPDT relays
- both relay poles paralleled on each RF path to reduce effective contact resistance and share current
- separate selector and ground-clamp relay functions
- Bruene couplers with FT114-43 ferrite toroids
- adjustable capacitors for bridge balance/directivity tuning
- 1N5711W Schottky detector diodes
- N-type bulkhead connectors connected to the PCB by short coaxial pigtails
- final 2-layer, 1.6 mm FR-4 RF PCB with a continuous bottom ground reference

The IM06DGR is a practical cost/size/current compromise rather than a dedicated high-power coaxial RF relay. Paralleling its contacts does not by itself prove continuous 1 kW capability.

### Control section

- CYD ESP32-2432S028R touchscreen module
- MCP23017 I²C I/O expander
- ULN2803 relay-driver arrays
- ADS1115 16-bit I²C ADC
- 12 V input and local 3.3 V regulation

## Relay Topology and Safe States

Each antenna supports three logical states:

| State | Meaning |
| --- | --- |
| `BUS1` | Antenna connected to transceiver bus 1 |
| `BUS2` | Antenna connected to transceiver bus 2 |
| `PARK` | Antenna disconnected from both buses and grounded |

The antenna is connected to the ground-clamp relay common contact. With the clamp coil off, the normally closed contact grounds the unused antenna. When the antenna is selected, the clamp releases it to the selector relay, which routes it to BUS1 or BUS2. This creates a defined grounded state when control power is absent.

All touchscreen, Blynk, and serial requests pass through the same state manager. Before actuating the relays, the firmware:

- accepts only BUS1, BUS2, or PARK assignments
- rejects conflicting antenna-to-bus combinations
- parks an existing bus assignment before connecting a replacement
- applies the manual TX1/TX2 switching locks
- keeps local and remote state displays synchronized

The TX locks are manual. Automatic RF-presence or PTT-based hot-switch prevention is not implemented.

## Local and Remote Interfaces

The CYD touchscreen shows antenna selection, BUS1/BUS2 assignments, PARK state, manual TX locks, and detector-derived values.

![CYD touchscreen GUI](images/cyd_gui.jpg)

The Blynk dashboard provides remote antenna commands and synchronized system status. Remote commands cannot bypass the central interlock logic.

![Blynk dashboard](images/blynk_dashboard.jpg)

### Blynk Virtual Pins

| Function | Virtual pin |
| --- | ---: |
| Selected antenna | V0 |
| BUS1 command | V1 |
| BUS2 command | V2 |
| PARK command | V3 |
| BUS1 antenna display | V4 |
| BUS2 antenna display | V5 |
| TX1 power estimate | V6 |
| TX1 SWR estimate | V7 |
| TX2 power estimate | V8 |
| TX2 SWR estimate | V9 |
| TX1 manual lock | V10 |
| TX2 manual lock | V11 |

## Forward/Reflected Measurement

Each transceiver path contains a Bruene coupler. The current transformer samples line current, while the capacitive network samples line voltage. Combining these components produces outputs corresponding to the forward and reflected waves. The 1N5711W detector stages convert the RF samples into DC voltages, and the ADS1115 digitizes four channels:

- TX1 forward
- TX1 reflected
- TX2 forward
- TX2 reflected

The four channels were acquired independently and produced stable raw readings during bench testing. This validates the acquisition chain, but it does **not** yet validate wattmeter or SWR accuracy.

### Power and SWR Processing

The firmware currently applies provisional conversion coefficients to the detector voltages. The simplified power relationship used during development assumes a nominal 50 Ω system and nominal coupler attenuation, but actual detector response, insertion loss, coupling factor, and diode behavior must be established by calibration.

SWR is derived from the forward/reflected relationship:

```text
|Γ| = sqrt(Preflected / Pforward)
SWR = (1 + |Γ|) / (1 - |Γ|)
```

If calibrated detector outputs are proportional to RF voltage rather than power, the corresponding calibrated voltage ratio is used directly for `|Γ|`. The firmware rejects invalid cases such as insufficient forward signal or reflected values outside the valid calibrated range.

Until comparison against a known RF power meter and 50 Ω dummy load is complete, displayed power and SWR values should be treated as estimates.

## RF PCB and 50 Ω Microstrip

An earlier RF-board revision used a 4-layer stack-up with the reference plane close to the top layer. Its calculated 50 Ω trace width was approximately 0.77 mm. Although theoretically impedance-controlled, the narrow conductor and discontinuities around relay and connector transitions were not suitable for the intended high-power prototype.

The final RF board uses:

- 2-layer, 1.6 mm FR-4 construction
- RF routing on the top layer
- a continuous ground reference on the bottom layer
- approximately 2.9 mm microstrip width from the fabricated stack-up calculation (about 50.16 Ω at 30 MHz)
- ground-via stitching and improved connector/relay transitions

![Earlier 4-layer microstrip calculation](images/microstrip_wrong.jpg)
![Final 2-layer microstrip calculation](images/microstrip_correct.jpg)

## RF Validation

Measurements were made over the HF range with a calibrated VNA and 50 Ω terminations. The figures below describe the latest validated markers for the final board; they are not all-path worst-case qualification results.

### Return Loss (S11)

The final 2-layer board measured approximately:

- **S11 ≈ −20 dB below 15 MHz**
- **S11 ≈ −13 dB at 30 MHz**
- corresponding VSWR range of approximately **1.22 to 1.58**

This is a clear improvement over the earlier 4-layer board, which measured −10.63 dB at 14.5 MHz and degraded toward approximately −7 dB at 30 MHz. The final result is acceptable for the engineering prototype at the lower part of the HF range, while the degradation toward 30 MHz shows that relay, connector, and PCB discontinuities still require improvement.

### Insertion Loss (S21)

The latest validated marker is:

- **S21 ≈ −0.36 dB at 26 MHz**, equivalent to **0.36 dB insertion loss**

This does not meet the original ≤0.20 dB objective and should not be presented as an all-band or all-path worst-case value.

### Isolation (S21)

The latest validated marker is:

- **S21 ≈ −42 dB at 26 MHz**, equivalent to **42 dB isolation**

This marker applies to the tested relay/port configuration. Full path-by-path characterization is still required before stating a system-wide worst-case isolation value.

### Validated Measurement Summary

| Measurement | Latest validated result | Scope |
| --- | ---: | --- |
| Return loss | S11 ≈ −20 dB below 15 MHz; ≈ −13 dB at 30 MHz | Final 2-layer board, selected path |
| Insertion loss | S21 ≈ −0.36 dB at 26 MHz | Marker on tested through-path |
| Isolation | S21 ≈ −42 dB at 26 MHz | Marker on tested isolated-path configuration |

No newer validated crosstalk value is claimed here. The June crosstalk marker has been removed from the result summary because it belongs to the earlier measurement set.

## Firmware

The ESP32 firmware uses the Arduino framework and includes:

- centralized relay state management
- MCP23017 output control through ULN2803 drivers
- touchscreen input and status rendering
- Blynk commands and state synchronization
- ADS1115 detector acquisition
- provisional power and SWR processing
- manual TX locks
- serial debug commands
- watchdog supervision
- Wi-Fi/Blynk reconnection handling

### Serial Commands

| Command | Function |
| --- | --- |
| `1b1 ... 8b1` | Connect antenna to BUS1 |
| `1b2 ... 8b2` | Connect antenna to BUS2 |
| `1p ... 8p` | Park antenna |
| `allp` | Park all antennas |
| `state` | Print antenna states |
| `sel1 ... sel8` | Select antenna in the GUI |
| `tx1on` / `tx1off` | Enable/disable TX1 manual lock |
| `tx2on` / `tx2off` | Enable/disable TX2 manual lock |

## Current Validation Status

Validated at bench level:

- 8-to-2 relay switching and grounded PARK behavior
- local CYD and remote Blynk control
- shared state management and software interlocks
- manual TX lock behavior
- four independent raw detector channels
- final-board S11 on a selected path
- the S21 insertion-loss and isolation markers listed above

Still requiring qualification:

- complete all-path RF characterization and repeatability testing
- calibrated forward power and SWR accuracy
- automatic RF/PTT interlocking
- continuous 1 kW thermal, contact-current, and safety testing
- worst-case isolation and crosstalk across all relay states

## Lessons Learned

- The fabricated PCB stack-up must be known before calculating microstrip width.
- RF traces are transmission lines; relay and connector transitions are part of the impedance path.
- A continuous, short return path is as important as the RF trace itself.
- A theoretically correct narrow microstrip can still be a poor practical choice for high-current RF.
- Relay selection must consider mismatch current, contact resistance, RF behavior, size, and cost.
- Local and remote commands must use the same interlock logic.
- Raw detector acquisition is not the same as calibrated RF measurement.
- Marker results must not be generalized into all-band or all-path specifications.

## Future Work

- repeat SOLT-calibrated S11/S21 tests for every RF path and relay state
- document full-band minima, maxima, and relay re-operation repeatability
- improve relay and connector impedance transitions
- improve shielding and physical separation between RF paths
- calibrate both Bruene couplers with a known source, power meter, and dummy load
- add per-channel calibration coefficients to the firmware
- implement automatic RF-presence or PTT-based switching inhibition
- perform supervised high-power thermal and safety testing

## Disclaimer

This is an educational engineering prototype. It has not been qualified for continuous unattended operation at 1 kW. Power and SWR displays remain provisional until reference-meter calibration is complete. High-power RF testing requires suitable dummy loads, shielding, measurement equipment, procedures, and supervision.

## Author

**Mahmoud Mostafa**

- Bachelor thesis 2025/2026
- Electronics-ICT — Embedded Hardware
- Thomas More University of Applied Sciences
