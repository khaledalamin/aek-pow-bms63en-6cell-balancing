# Patch Notes: `AEK_POW_BMS63CHAIN_app_mng.c`

This document explains the experimental changes in `AEK_POW_BMS63CHAIN_app_mng.c` for the six-active-cell AEK-POW-BMS63EN / L9963E setup.

The file may include ST-generated/demo code around the modified sections. Check your ST license before redistributing a complete source file.

Additional modified files were found after scanning the supplied ST project ZIP. See [ADDITIONAL_MODIFIED_FILES.md](ADDITIONAL_MODIFIED_FILES.md).

## 1. Active-Cell Mask

Added a hard mask for the physical cells that exist:

```c
#define AEK_POW_BMS63CHAIN_ACTIVE_CELL_MASK  ((uint16_t)0x300FU)
```

Allowed cells:

| Cell | Bit | Mask |
| --- | ---: | ---: |
| CELL1 | 0 | `0x0001` |
| CELL2 | 1 | `0x0002` |
| CELL3 | 2 | `0x0004` |
| CELL4 | 3 | `0x0008` |
| CELL13 | 12 | `0x1000` |
| CELL14 | 13 | `0x2000` |

Blocked cells:

```text
CELL5-CELL12
```

All balancing paths sanitize requested masks through `0x300F`.

## 2. Manual Balancing Commands

Added serial command handling for manual balancing:

```text
BAL OFF
BAL MAN <mask>
BAL CELL <1-14> ON
BAL CELL <1-14> OFF
BAL?
BAL SUMMARY?
```

Manual commands reject or strip unused cells. For example, requesting `0x3FFF` becomes `0x300F`, and requesting `BAL CELL 5 ON` returns an error.

## 3. AUTO Balancing Mode

Added `BAL AUTO` and AUTO state/debug reporting.

AUTO logic:

- reads active cell voltages only;
- finds min and max active cells;
- starts above about 30 mV spread;
- stops around 12-15 mV spread;
- requires near-zero current;
- uses 60 s ON / 60 s OFF pulsed balancing;
- defaults to one cell at a time.

Reported states:

```text
OFF
BLOCKED_SAFETY
BALANCING
COOLDOWN
STOPPED_DELTA_SMALL
WAIT_DELTA
READY
```

## 4. Strategy Layer

Added strategy selection:

```text
BAL STRATEGY?
BAL STRATEGY SINGLE
BAL STRATEGY MULTI2
```

`FAIR_SINGLE` is the default. It balances one high cell and uses energy-aware tie selection.

`MULTI2` is experimental. It can select up to two cells under non-adjacent and estimated-power constraints. It should be used only after single-cell behavior and thermal response are validated.

## 5. Safety Gate

Added a safety check before balancing output is applied.

The gate blocks balancing when:

- VREF or VTREF is outside the expected bench range;
- trim/calibration flags are not OK;
- EEPROM calibration/section CRC flags are set;
- RAM CRC flag is set;
- critical ground/reference faults are active;
- chip overtemperature is active;
- pack current is not near zero;
- active-cell voltage is outside the configured bench window;
- an unused-cell bit would be applied.

The gate was added specifically to block impossible measurement states, such as all cells reading around 5.5 V.

## 6. Energy Accounting

Added per-active-cell balancing accounting:

```text
BAL ENERGY?
BAL ENERGY RESET
```

The estimate uses:

```text
Ibal = Vcell / 39 ohm
removed_mAh += Ibal * dt
removed_Wh += Vcell * Ibal * dt
```

Observed validation:

```text
CELL4 ON_TIME about 960 s
removed about 24 mAh
expected current about 3.5 V / 39 ohm = 90 mA
```

This is a bench estimate, not certified fuel gauging.

## 7. Diagnostics

Added or extended serial diagnostics:

```text
S0
S1
TRIM?
FAULT?
BAL SAFETY?
BAL SUMMARY?
AUTO?
BAL STATE?
BAL ENERGY?
```

These commands make it possible to test without relying only on the GUI.

## 8. UDE Debugging Support

The code keeps key state in variables suitable for UDE watch windows:

- manual mask;
- AUTO requested mask;
- actual applied mask;
- selected AUTO cell;
- min/max active cell;
- delta voltage;
- ON/cooldown state;
- energy counters;
- safety-gate relevant diagnostic flags.

## 9. Integration Notes

When applying this file to a recreated ST project:

1. Reproduce the six-cell generated chain configuration.
2. Apply or port the matching `AEK_POW_BMS63CHAIN_app_mng.h`.
3. Confirm generated type names match.
4. Confirm chain and device index constants match your project.
5. Confirm floating-point `sprintf` works in your toolchain settings.
6. Build with warnings enabled.
7. Use serial commands and UDE watches before connecting a real pack.

If your ST demo version differs, use these notes to port the modifications rather than assuming the file is drop-in compatible.
