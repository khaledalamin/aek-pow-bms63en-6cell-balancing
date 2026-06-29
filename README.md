# Experimental L9963E 6-Cell Passive Balancing on ST AEK-POW-BMS63EN

[![Status](https://img.shields.io/badge/status-experimental-orange)](#safety-warning)
[![Hardware](https://img.shields.io/badge/hardware-AEK--POW--BMS63EN-blue)](#hardware-used)
[![License](https://img.shields.io/badge/license-MIT%20for%20original%20docs%2Fcode-green)](LICENSE)

Educational repository for an experimental ST AutoDevKit / SPC5Studio project using the ST AEK-POW-BMS63EN board with the L9963E battery monitoring IC. The project documents how a 14-channel battery monitoring board/holder was adapted for a non-standard 6-cell physical stack, with firmware guards to prevent passive balancing on unused shorted channels.

This is not an official ST reference design, not official ST support material, and not a safety-certified BMS.

## Hardware Used

| Item | Notes |
| --- | --- |
| ST AEK-POW-BMS63EN | L9963E-based battery monitoring board |
| ST AutoDevKit / SPC5Studio | Demo project environment targeting SPC58EC |
| SPC58EC target/debug setup | Flashed and debugged with UDE Starterkit 2021 |
| 14-cell holder/board setup | Only six physical cells mounted/used |
| Serial terminal | Used for manual and AUTO balancing commands |
| DMM | Required for continuity and voltage verification before power-up |

Example bench setup:

![Board and 14-cell holder bench setup](images/board_overview.jpg)

Example GUI monitoring view from the six-cell setup:

![GUI monitoring view showing six active cells](images/gui_monitoring_6cell.png)

## What This Solves

The original ST demo project is oriented toward standard board configurations and GUI-driven interaction. This project adapts the setup for six physical cells placed in a 14-channel holder/board arrangement:

- active physical cells are CELL1, CELL2, CELL3, CELL4, CELL13, and CELL14;
- CELL5 through CELL12 are unused and bridged/shorted in the sense/balance wiring according to the board requirements;
- firmware allows balancing only on the six active physical cells;
- manual and automatic balancing commands are available from the serial terminal;
- safety checks block balancing when measurements or diagnostic flags look unsafe;
- passive balancing energy is estimated and reported.

The goal is to help ST community users, embedded BMS developers, students, and researchers understand a reproducible method for this non-standard configuration.

## Safety Warning

Battery wiring mistakes can burn wires, damage the L9963E, damage the board, or create a fire and shock hazard. Do not copy this wiring blindly.

Before connecting the BMS IC or enabling balancing:

- verify every connection against the official ST documentation, board schematic, L9963E datasheet, and your own measurements;
- verify continuity and polarity with a DMM while power is off;
- verify no large voltage exists across the unused-channel bridges;
- start with streaming off and balancing off;
- enable balancing only near rest / near-zero current;
- stop immediately if a voltage, current, temperature, or fault flag does not make physical sense.

This repository is experimental educational material. It is not safety-certified and is not suitable for production battery systems without a complete engineering safety process.

See [DISCLAIMER.md](DISCLAIMER.md) and [docs/safety_and_faults.md](docs/safety_and_faults.md).

## 6-Cell Configuration

The physical setup uses a 14-cell holder/board path, but only six cells are installed:

```text
Physical cells used:
  CELL1, CELL2, CELL3, CELL4, CELL13, CELL14

Unused / bridged channels:
  CELL5, CELL6, CELL7, CELL8, CELL9, CELL10, CELL11, CELL12
```

### Active Cell Map

| Physical cell | Firmware bit | Hex bit | Balancing allowed |
| --- | ---: | ---: | --- |
| CELL1 | 0 | `0x0001` | Yes |
| CELL2 | 1 | `0x0002` | Yes |
| CELL3 | 2 | `0x0004` | Yes |
| CELL4 | 3 | `0x0008` | Yes |
| CELL5 | 4 | `0x0010` | No |
| CELL6 | 5 | `0x0020` | No |
| CELL7 | 6 | `0x0040` | No |
| CELL8 | 7 | `0x0080` | No |
| CELL9 | 8 | `0x0100` | No |
| CELL10 | 9 | `0x0200` | No |
| CELL11 | 10 | `0x0400` | No |
| CELL12 | 11 | `0x0800` | No |
| CELL13 | 12 | `0x1000` | Yes |
| CELL14 | 13 | `0x2000` | Yes |

```c
#define AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK  ((uint16_t)0x300FU)
```

Cells 5-12 must never be balanced. All manual and AUTO balancing requests are sanitized through `ACTIVE_CELL_MASK = 0x300F`.

## Wiring / Bridging Overview

The board/holder path remains a 14-channel measurement chain. The unused channels 5-12 are bridged/shorted in the sense/balance harness so the device sees a valid continuous stack while only six cells are physically present.

In the photographed setup, the unused-channel bridge area ties the CELL4-side node through the unused sense/balance nodes. The user-provided notes describe the following unused nodes being connected to the CELL4 reference region: `S5`, `B6_5`, `S6`, `S7`, `B8_7`, `S8`, `S9`, `B10_9`, `S10`, `S11`, `B12_11`, and `S12`.

Verify this against the official ST documentation and board schematic before reproducing it.

![Shorted unused-cell area](images/shorted_unused_cells_area.jpg)

More detail: [docs/wiring_6cell_14holder.md](docs/wiring_6cell_14holder.md).

## Firmware Features

- active-cell safety mask for CELL1, CELL2, CELL3, CELL4, CELL13, CELL14;
- hard block for CELL5-CELL12 balancing;
- manual balancing from serial terminal;
- AUTO balancing with start/stop delta thresholds;
- 180 s ON / 60 s OFF pulsed balancing in the current firmware snapshot;
- default one-cell-at-a-time `FAIR_SINGLE` strategy;
- experimental `MULTI2` strategy for up to two non-adjacent high cells with constraints;
- sticky/tie-aware selection to reduce ADC-noise-driven cell hopping;
- safety gate before any balancing command is applied;
- energy accounting for passive balancing removed charge and energy;
- serial diagnostics for fault, trim/calibration, safety, AUTO state, and balancing energy;
- UDE debug workflow notes for watching firmware variables.

## Passive Balancing Physics

The board balancing resistor value used for the estimate is approximately 39 ohm, based on observed board behavior and BOM resistor values. Verify the actual value on your hardware.

```text
Ibal = Vcell / Rbal
Pbal = Vcell^2 / Rbal
```

At a 3.5 V cell voltage:

```text
Ibal = 3.5 V / 39 ohm = 0.0897 A, about 90 mA
Pbal = 3.5^2 / 39 = 0.314 W
```

With the current 180 s ON / 60 s OFF pulse schedule, the long-term average balancing current is about 75 percent of the ON-current, assuming AUTO remains active and no safety/cooldown condition interrupts it. Passive balancing is still slow: useful validation should rely on ON-time and removed-mAh accounting, not only instantaneous cell-voltage drop. Voltage drop is not linear with removed mAh because lithium-ion cell voltage depends on SOC curve shape, relaxation, temperature, load history, internal resistance, and measurement timing.

Balancing is normally done near rest so the voltage comparison is not dominated by load current or recovery effects.

More detail: [docs/energy_accounting.md](docs/energy_accounting.md).

## AUTO Balancing Algorithm

Default thresholds used in this experimental firmware:

| Parameter | Value | Reason |
| --- | ---: | --- |
| Start delta | about 30 mV | Avoid balancing on noise or insignificant spread |
| Stop delta | about 12-15 mV | Stop before chasing unrealistic zero difference |
| Tie margin | about 6 mV | Treat nearly equal high cells as tied |
| Rest current threshold | about 0.1 A | Balance only near rest; raised from 0.05 A after the board idle draw was observed |
| Bench voltage window | about 3.30-4.18 V | Block impossible or unsafe bench measurements |
| Pulse ON time | 180 s | Controlled removal with measured bench thermal behavior |
| Pulse OFF time | 60 s | Cooldown / relaxation interval |

Zero-difference balancing is not realistic. ADC noise, cell relaxation, wiring resistance, temperature differences, and cell chemistry all move the reported voltage by several millivolts. The firmware therefore balances only when the spread is meaningful and stops when the pack is close enough for this bench configuration.

More detail: [docs/balancing_algorithm.md](docs/balancing_algorithm.md).

## Serial Commands

| Command | Purpose |
| --- | --- |
| `S0` | Disable serial stream |
| `S1` | Enable serial stream |
| `BAL OFF` | Force all balancing off |
| `BAL AUTO` | Enable AUTO balancing |
| `BAL?` | Print balancing mode/masks/state |
| `BAL SUMMARY?` | Print active cell summary and balance status |
| `BAL ENERGY?` | Print per-cell ON time, removed mAh, Wh, estimated SOC removed |
| `BAL ENERGY RESET` | Reset balancing energy counters |
| `BAL SAFETY?` | Print safety gate status |
| `BAL STATE?` | Print AUTO/balancing state |
| `BAL TEST?` | Print compact safety, SPI, VREF, current, strategy, and mask diagnostics |
| `BAL PHASE?` | Print current phase, active/cooldown status, and remaining timing |
| `AUTO?` | Print AUTO diagnostic details |
| `FAULT?` | Print relevant fault/status flags |
| `TRIM?` | Print trim/calibration/CRC status |
| `BAL STRATEGY?` | Print current balancing strategy |
| `BAL STRATEGY SINGLE` | Select default one-cell strategy |
| `BAL STRATEGY MULTI2` | Select experimental two-cell strategy |

Additional manual mask commands are implemented in the firmware. Any requested manual mask is sanitized through `0x300F`; unused channels are rejected or stripped.

Full reference: [docs/serial_commands.md](docs/serial_commands.md).

## Example Serial Outputs

Good trim/calibration state:

```text
TRIM;TRIM_CAL_OK_DIAG;1;TRIM_CAL_OK_MEAS;1;EEPROM_CALOFF;0;EEPROM_CALRAM;0;EEPROM_SECT0;0;RAMCRC;0;GSW;0;
```

Good safety state:

```text
BAL_SAFETY;SAFE;1;VREF_OK;1;TRIM_OK;1;EEPROM_OK;1;RAMCRC_OK;1;GND_OK;1;TEMP_OK;1;CURRENT_OK;1;CELL_V_OK;1;
```

AUTO stream example:

```text
AUTO_DBG;MODE;BALANCING;NODE_VREF;4.9700;CURR;0.0040;MASK;0x0008;ACTIVE;1;COOLDOWN;0;
AUTO_DBG;MIN_CELL;1;MIN_V;3.4440;MAX_CELL;4;MAX_V;3.5180;DELTA_MV;74.0;SEL_CELL;4;
AUTO_DBG;VC1;3.4440;VC2;3.4680;VC3;3.4900;VC4;3.5180;VC13;3.5120;VC14;3.5060;
```

Energy accounting example:

```text
BAL_ENERGY;CELL;4;ON_TIME_S;960;REMOVED_MAH;24.0;REMOVED_WH;0.084;SOC_REMOVED_PCT;0.69;
```

What good looks like:

- VREF stable around 4.97 V in the tested setup;
- pack voltage around 20.8-21.0 V for six cells near 3.5 V;
- active cell voltages physically realistic and close to DMM measurements;
- trim/calibration and CRC flags OK;
- only non-critical wake/status flags present, such as `wuIsoLine` in the observed test;
- CELL5-CELL12 never appear in the applied balancing mask.

What bad looks like:

- all cells reading around an impossible value such as 5.5 V;
- VREF missing or far outside expected range;
- trim/calibration, EEPROM, RAM CRC, ground/reference, or overtemperature fault active;
- balancing mask includes any of CELL5-CELL12;
- pack current not near zero while AUTO balancing is trying to start.

## Build in AutoDevKit / SPC5Studio

1. Install the required ST AutoDevKit / SPC5Studio environment and packages for the AEK-POW-BMS63EN / SPC58EC demo.
2. Recreate or import the ST `BMS_CHAIN` demo project using ST-provided materials.
3. Build the original demo once to confirm the toolchain, components, linker script, and board configuration are correct.
4. Reproduce the six-active-cell generated configuration: CELL1, CELL2, CELL3, CELL4, CELL13, and CELL14 enabled; CELL5-CELL12 disabled.
5. Apply or port the modified files in [firmware/modified_files](firmware/modified_files/), especially:
   - `AEK_POW_BMS63CHAIN_app_mng.c`
   - `AEK_POW_BMS63CHAIN_app_mng.h`
   - `main.c`
   - `AEK_POW_BMS63CHAIN_chain_cfg.c`
   - `AEK_POW_BMS63CHAIN_iso_mng.c`
6. Rebuild.
7. Review compiler warnings carefully, especially around toolchain library declarations and generated component APIs.

Details: [firmware/modified_files/ADDITIONAL_MODIFIED_FILES.md](firmware/modified_files/ADDITIONAL_MODIFIED_FILES.md).

If your ST license does not allow redistributing the complete generated project, publish only patches and modified files, and instruct users to recreate the base project using ST AutoDevKit.

## Flash / Debug with UDE

1. Open the generated project in SPC5Studio / AutoDevKit.
2. Build the firmware and confirm the output ELF/HEX/MOT files are generated.
3. Open the UDE Starterkit 2021 workspace/configuration for the SPC58EC target.
4. Connect the debug probe and power the board according to the ST documentation.
5. Flash the firmware.
6. Use watch variables to inspect:
   - active cell voltages;
   - balancing requested mask;
   - actual applied balancing mask;
   - AUTO state;
   - safety gate result;
   - trim/calibration flags;
   - energy counters.
7. Keep `BAL OFF` as the first terminal command after boot until measurements are confirmed.

Example UDE watch window:

![UDE watch variables for balancing debug](images/ude_watch_balancing_variables.png)

## Safe Test Procedure

Short bench bring-up sequence:

```text
S0
BAL OFF
TRIM?
FAULT?
BAL SAFETY?
BAL SUMMARY?
BAL ENERGY RESET
BAL AUTO
S1
```

During the first test, run AUTO only briefly. Confirm the selected cell is one of CELL1, CELL2, CELL3, CELL4, CELL13, or CELL14, and confirm CELL5-CELL12 remain blocked.

Full protocol: [docs/testing_protocol.md](docs/testing_protocol.md).

## Verification and Data Notes

Bench verification for the current firmware included:

- same physical wiring before and after the firmware fixes;
- stable realistic active-cell readings for CELL1-CELL4 and CELL13-CELL14;
- VREF observed around 4.97 V;
- healthy SPI/BMS status during normal operation;
- heating problem removed during normal operation;
- manual and AUTO balancing constrained to `ACTIVE_CELL_MASK = 0x300F`;
- CELL5-CELL12 blocked from all applied balancing masks;
- AUTO/MULTI2 tested successfully after the pack was above the configured minimum-cell gate.

See [docs/verification_summary.md](docs/verification_summary.md). Raw acquisition datasets are not included in this checkout; add or link them separately if publishing dataset files.

## Repository Layout

```text
.
|-- README.md
|-- LICENSE
|-- NOTICE.md
|-- DISCLAIMER.md
|-- CHANGELOG.md
|-- CONTRIBUTING.md
|-- docs/
|-- firmware/
|   |-- modified_files/
|   |-- patches/
|   |-- full_project/
|   `-- notes/
|-- images/
`-- .github/
```

## Known Limitations

- This is an experimental bench project, not a production BMS.
- The wiring is specific to the photographed 14-channel holder/board setup and must be verified against official ST documentation.
- The safety gate is a firmware guard, not a complete safety concept.
- The voltage safety window of about 3.30-4.18 V is chosen for the current bench tests and may need adjustment for other cells or SOC ranges.
- Passive balancing estimates assume approximately 39 ohm balancing resistance; verify your board.
- `MULTI2` is experimental and not the default.
- Full ST-generated code may be subject to ST license restrictions; see [NOTICE.md](NOTICE.md).

## Future Work

- Add or link data acquisition logs and scripts for long balancing characterization.
- Add plots for cell voltage, selected balancing mask, removed mAh, and temperature.
- Add a screenshot from the ST GUI showing an active CELL13 balancing event.
- Add measured resistor thermal data during long ON/OFF cycles.
- Add a clean patch workflow against a recreated ST base project.
- Document any changes needed for different cell holders or pack layouts.

## Disclaimer

This project is provided for education and community discussion only. It is not endorsed by ST, not official ST support, not a reference design, and not safety-certified. You are responsible for verifying all wiring, firmware changes, limits, and test procedures before connecting real batteries.
