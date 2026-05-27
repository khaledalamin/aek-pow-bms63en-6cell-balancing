# Troubleshooting

This guide lists common bench issues seen or expected when adapting the AEK-POW-BMS63EN / L9963E demo for a six-cell physical stack in a 14-channel setup.

## `BAL SAFETY?` Reports `BLOCKED`

Run:

```text
TRIM?
FAULT?
BAL SUMMARY?
AUTO?
```

Check:

- VREF is within the expected range;
- VTREF is valid;
- active-cell voltages are within the configured bench window;
- pack current is below about 0.05 A;
- trim/calibration flags are OK;
- EEPROM/RAM CRC flags are clear;
- no critical ground/reference or overtemperature fault is active.

Do not bypass the safety gate to make balancing start.

## All Cells Read Around an Impossible Voltage

Example bad pattern:

```text
VC1;5.5000;VC2;5.5000;VC3;5.5000;...
```

Likely areas to inspect:

- VREF/reference path;
- board power state;
- trim/calibration status;
- sense harness connector orientation;
- bridge wiring around unused channels;
- generated project configuration;
- stale debug variables in UDE.

Expected action:

```text
BAL OFF
S0
```

Then inspect wiring and diagnostics before trying again.

## CELL5-CELL12 Appear in an Applied Mask

This must not happen.

Expected invariant:

```text
(actual_mask & 0x0FF0) == 0
```

If violated:

1. Stop with `BAL OFF`.
2. Check that the modified `AEK_POW_BMS63CHAIN_app_mng.c` is the file actually built.
3. Confirm `ACTIVE_CELL_MASK` is `0x300F`.
4. Watch manual mask, AUTO requested mask, and actual applied mask in UDE.
5. Inspect any direct writes to the ST balancing command fields.

## Manual Command Rejects a Cell

If this command fails:

```text
BAL CELL 5 ON
```

That is correct. CELL5-CELL12 are unused/bridged and blocked.

Allowed cells:

```text
CELL1, CELL2, CELL3, CELL4, CELL13, CELL14
```

## AUTO Does Not Start

AUTO may be waiting intentionally.

Check `AUTO?` and `BAL SUMMARY?`.

Common valid reasons:

- spread is below the 30 mV start threshold;
- spread already below stop threshold;
- current is above rest threshold;
- safety gate is blocked;
- the selected cell voltage is outside bench window;
- cooldown is active.

AUTO is not meant to force balancing when the pack is already close enough.

## AUTO Jumps Between Cells

Small changes can happen due to ADC noise and relaxation. The firmware includes tie and sticky logic, but jumping may still be visible if high cells are nearly equal.

Check:

- tie margin, about 6 mV;
- removed mAh per tied cell;
- whether a cell is in cooldown;
- DMM readings;
- whether pack current is truly near zero.

## Energy Counters Look Too Small

Passive balancing is slow. With a 39 ohm resistor and 3.5 V cell:

```text
Ibal ~= 90 mA while ON
average ~= 45 mA with 60 s ON / 60 s OFF
```

Expected removed charge:

```text
20 min ~= 15 mAh
30 min ~= 22.5 mAh
960 s ON ~= 24 mAh
```

Small numbers are normal.

## Voltage Does Not Drop Much

This can be normal. Mid-SOC lithium-ion voltage is relatively flat, and relaxation can hide or reverse immediate voltage changes. Use removed mAh and Wh for validation, not only instantaneous voltage drop.

## UDE Shows Old or Unexpected Values

Check:

- the project rebuilt after replacing the modified C file;
- the flashed binary is the latest build output;
- optimization level is not hiding variables unexpectedly;
- watch expressions point to the correct chain and node index;
- the serial terminal and UDE are observing the same run.

Example watch window from the bench setup:

![UDE watch variables](../images/ude_watch_balancing_variables.png)

## Build Fails After Replacing the File

Check:

- project generated from the same ST demo family;
- include paths and generated types match the modified file;
- C library provides required functions or explicit declarations;
- floating-point `sprintf` support is available in the toolchain settings;
- code was inserted into the correct source path.

If the base ST project version differs, use the patch notes as the migration guide instead of assuming the file is drop-in compatible.

## `MULTI2` Gets Too Hot

Stop:

```text
BAL OFF
BAL STRATEGY SINGLE
```

`MULTI2` is experimental. Validate resistor temperature, board temperature, and total estimated power before using it for longer tests.

## Fault Flag Meaning Is Unclear

Use the official L9963E documentation and ST board material as the source of truth. This repo avoids assigning official meanings beyond what was used for the experimental safety gate.

When asking for help, include:

- board revision;
- firmware commit;
- cell voltages by DMM;
- serial output from `TRIM?`, `FAULT?`, `BAL SAFETY?`, and `AUTO?`;
- clear photos of the bridge wiring;
- whether the issue appears in manual mode, AUTO mode, or both.
