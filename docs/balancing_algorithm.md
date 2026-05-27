# Balancing Algorithm

This document explains the experimental passive balancing logic added around the ST AEK-POW-BMS63EN / L9963E demo application for the six-active-cell setup.

## Design Goals

The firmware is conservative by default:

- balance only physical cells that exist;
- never balance CELL5-CELL12;
- block balancing when diagnostic or measurement state is unsafe;
- balance near rest, not during meaningful charge/discharge current;
- use pulsed balancing to limit heat and allow cell relaxation;
- default to one cell at a time;
- report enough state for serial terminal and UDE debugging.

## Active-Cell Mask

Only these cells can be balanced:

| Cell | Bit | Mask |
| --- | ---: | ---: |
| CELL1 | 0 | `0x0001` |
| CELL2 | 1 | `0x0002` |
| CELL3 | 2 | `0x0004` |
| CELL4 | 3 | `0x0008` |
| CELL13 | 12 | `0x1000` |
| CELL14 | 13 | `0x2000` |

```c
ACTIVE_CELL_MASK = 0x300F
```

All requested masks pass through:

```c
safe_mask = requested_mask & ACTIVE_CELL_MASK;
```

If a user requests CELL5-CELL12, the command is rejected or the unused bits are stripped before anything reaches the balancing output command.

## Thresholds

| Parameter | Experimental value | Purpose |
| --- | ---: | --- |
| Start delta | 30 mV | Start only when active-cell spread is meaningful |
| Stop delta | 12-15 mV | Stop before chasing measurement noise |
| Tie margin | 6 mV | Treat nearly equal high cells as equivalent candidates |
| Sticky energy margin | 1 mAh | Avoid jumping between tied cells too often |
| Rest-current threshold | 0.05 A | Balance only near rest |
| Bench min cell voltage | 3.30 V | Block unrealistic or low bench readings |
| Bench max cell voltage | 4.18 V | Block impossible or high bench readings |
| Pulse ON | 60 s | Controlled removal and thermal limit |
| Pulse OFF | 60 s | Cooldown / relaxation interval |

These values are for the current bench setup. Users must review them for their cells, wiring, thermal conditions, and test objective.

## AUTO State Model

The firmware reports these AUTO states:

| State | Meaning |
| --- | --- |
| `OFF` | Balancing control is off |
| `BLOCKED_SAFETY` | Safety gate failed; outputs forced off |
| `BALANCING` | A balancing pulse is currently active |
| `COOLDOWN` | OFF portion of the pulse cycle |
| `STOPPED_DELTA_SMALL` | Spread is already below stop threshold |
| `WAIT_DELTA` | Spread is below start threshold |
| `READY` | AUTO is enabled and ready to select a cell |

## AUTO Selection Flow

```text
Read active cell voltages
  |
  v
Check safety gate
  |
  +-- unsafe -> force all balancing OFF
  |
  v
Find min active cell and max active cell
  |
  v
delta = max - min
  |
  +-- delta <= stop threshold -> stop balancing
  |
  +-- delta < start threshold -> wait
  |
  v
Select high-cell candidate
  |
  v
Apply active-cell mask and strategy limits
  |
  v
60 s ON pulse
  |
  v
60 s OFF cooldown
  |
  v
repeat
```

## FAIR_SINGLE Strategy

`FAIR_SINGLE` is the default strategy. It balances one cell at a time. Among high cells within the tie margin, it prefers the cell with the least previously removed charge. Sticky selection reduces hopping due to ADC noise.

This is the recommended strategy for community reproduction because it is simple to inspect, reduces simultaneous heat, and makes current/energy accounting easier to validate.

## MULTI2 Strategy

`MULTI2` is experimental. It allows up to two non-adjacent high cells to be balanced when the cells meet the same safety and voltage conditions and when the estimated total balancing power remains below the configured cap.

Use `MULTI2` only after:

- `FAIR_SINGLE` has been validated;
- thermal rise is measured;
- actual applied masks are watched in UDE or serial logs;
- current and cell voltage measurements are stable.

It is intentionally not the default.

## Why Zero-Difference Balancing Is Not Realistic

Trying to balance to exactly 0 mV difference is not useful in this setup:

- ADC readings have noise and quantization;
- cell voltage relaxes after load removal;
- balancing current creates a local voltage shift;
- harness and contact resistance affect readings;
- lithium-ion voltage is a weak SOC indicator over flat parts of the curve;
- temperature changes cell voltage and internal resistance.

The practical target is a small, stable spread, not mathematical equality.

## UDE Debug Variables to Watch

Exact names can change as the ST generated project changes, but useful classes of variables are:

- active-cell voltage array;
- `AEK_POW_BMS63CHAIN_balManualMask`;
- `AEK_POW_BMS63CHAIN_balAutoRequestedMask`;
- actual applied balancing mask;
- `AEK_POW_BMS63CHAIN_balAutoActive`;
- `AEK_POW_BMS63CHAIN_balAutoCooldown`;
- `AEK_POW_BMS63CHAIN_balAutoLastDeltaV`;
- selected AUTO cell index;
- removed mAh / Wh arrays;
- diagnostic flags used by the safety gate.

## Expected Behavior

With active cells around 3.44-3.52 V and near-zero pack current:

- AUTO waits if spread is below the start threshold.
- AUTO selects a high active cell when spread exceeds about 30 mV.
- CELL4, CELL13, or CELL14 may be selected depending on which is highest.
- CELL5-CELL12 never appear in the applied mask.
- After about 60 s ON, AUTO enters cooldown.
- Energy counters rise only for the applied active cell.
