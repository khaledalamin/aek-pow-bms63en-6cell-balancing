# Serial Command Reference

The firmware adds serial commands for stream control, balancing control, diagnostics, and energy accounting. Commands are intended for a simple terminal connected to the project serial interface.

Commands are parsed as uppercase strings in the modified application manager. Use the exact command text shown here unless your local code adds aliases.

## Quick Start

```text
S0
BAL OFF
TRIM?
FAULT?
BAL SAFETY?
BAL TEST?
BAL PHASE?
BAL SUMMARY?
BAL ENERGY RESET
BAL AUTO
S1
```

## Stream Commands

| Command | Aliases | Description |
| --- | --- | --- |
| `S0` | `STREAM OFF`, `MON OFF` | Disable periodic serial stream |
| `S1` | `STREAM ON`, `MON ON` | Enable periodic serial stream |
| `S?` | `STREAM?`, `MON?` | Report stream state |

## Balancing Commands

| Command | Description |
| --- | --- |
| `BAL OFF` | Force all balancing off and clear AUTO/manual state |
| `BAL AUTO` | Enable automatic balancing |
| `BAL?` | Report balancing mode, strategy, masks, and active/cooldown state |
| `BAL STATE?` | Report the current AUTO/balancing state |
| `BAL SUMMARY?` | Report active-cell min/max, delta, selected cell, and energy summary |
| `BAL SAFETY?` | Report whether the safety gate allows balancing |
| `BAL TEST?` | Compact safety, SPI, VREF/current, strategy, mask, and recovery status |
| `BAL PHASE?` | Current OFF/BALANCING/COOLDOWN/BLOCKED phase and timing |
| `BAL ENERGY?` | Report per-active-cell ON time, removed mAh, Wh, and estimated SOC removed |
| `BAL ENERGY RESET` | Reset balancing energy counters |
| `BAL STRATEGY?` | Report current balancing strategy |
| `BAL STRATEGY SINGLE` | Select default `FAIR_SINGLE` strategy |
| `BAL STRATEGY MULTI2` | Select experimental `MULTI2` strategy |

## Manual Balancing Commands

The modified firmware also supports manual balancing by mask and by cell number:

| Command | Description |
| --- | --- |
| `BAL MAN 0x0008` | Manually request a mask, sanitized through `0x300F` |
| `BAL CELL 4 ON` | Turn manual balancing on for CELL4 |
| `BAL CELL 4 OFF` | Turn manual balancing off for CELL4 |

Compatibility aliases in the code include CELL13 shortcuts such as `B13 ON`, `BAL13 ON`, and `CELL13 ON`.

Manual commands are guarded:

- CELL1, CELL2, CELL3, CELL4, CELL13, and CELL14 are allowed.
- CELL5-CELL12 are rejected or stripped.
- If safety gate fails, actual outputs are forced off even if the requested mask is non-zero.

Example:

```text
> BAL MAN 0x3FFF
WARN;BAL;UNUSED_CELLS_REMOVED;REQ;0x3FFF;SAFE;0x300F;
OK;BAL;MODE;MANUAL;STRATEGY;FAIR_SINGLE;MASK;0x300F;MAN;0x300F;AUTO_REQ;0x0000;ACTUAL;0x300F;ACTIVE;1;COOLDOWN;0;
```

For first bench tests, prefer one cell:

```text
BAL CELL 4 ON
BAL?
BAL CELL 4 OFF
```

## Diagnostics

| Command | Description |
| --- | --- |
| `TRIM?` | Print trim/calibration and EEPROM/RAM CRC status |
| `FAULT?` | Print relevant fault/status flags |
| `AUTO?` | Print AUTO debug details |
| `BAL SAFETY?` | Print safety gate summary |
| `BAL TEST?` | Print compact safety and communication summary |
| `BAL PHASE?` | Print current balancing phase |
| `BAL RECOVER` | Clear software recovery state only if real measurements are healthy |
| `BAL AUTORECOVER ON` | Enable automatic software recovery attempts |
| `BAL AUTORECOVER OFF` | Disable automatic software recovery attempts |

Advanced software-only injection commands are present for parser/safety testing:

```text
BAL INJECT VREF
BAL INJECT CELL
BAL INJECT FRAME
BAL INJECT RX
BAL INJECT CLEAR
```

These commands should not call low-level SPI-clearing functions directly. They are intended to test safety blocking and serial responsiveness without forcing hardware recovery operations from the command parser.

Example trim report:

```text
OK;TRIM_REPORT;BEGIN;
TRIM;CACHED;EEPROM_DONE;1;TRIM_CAL_OK_DIAG;1;TRIM_CAL_OK_MEAS;1;DRDY_VTREF;1;
TRIM;CRC;EEPROM_CALOFF;0;EEPROM_CALRAM;0;EEPROM_SECT0;0;RAMCRC;0;
OK;TRIM_REPORT;END;
```

Example fault report when no blocking faults are active:

```text
OK;FAULT_REPORT;BEGIN;
OK;FAULT_REPORT;ACTIVE_COUNT;0;
OK;FAULT_REPORT;NO_ACTIVE_FLAGS;
OK;FAULT_REPORT;END;
```

Observed testing allowed a non-critical wake/status flag such as `wuIsoLine` to remain active without blocking balancing. Verify flag meaning in the official documentation.

## AUTO Debug Output

`AUTO?` prints a compact multi-line state:

```text
AUTO_DBG;MODE;BALANCING;NODE_VREF;4.9700;CURR;0.0040;MASK;0x0008;ACTIVE;1;COOLDOWN;0;
AUTO_DBG;MIN_CELL;1;MIN_V;3.4440;MAX_CELL;4;MAX_V;3.5180;DELTA_MV;74.0;SEL_CELL;4;
AUTO_DBG;VC1;3.4440;VC2;3.4680;VC3;3.4900;VC4;3.5180;VC13;3.5120;VC14;3.5060;
AUTO_DBG;EN1;1;EN2;1;EN3;1;EN4;1;EN13;1;EN14;1;
AUTO_DBG;CMD1;0;CMD2;0;CMD3;0;CMD4;1;CMD13;0;CMD14;0;EN_STS;1;
```

## Strategy Commands

Default:

```text
BAL STRATEGY SINGLE
```

Experimental:

```text
BAL STRATEGY MULTI2
```

Use `MULTI2` only after `SINGLE` behavior, thermal response, and actual masks are validated.

## Error and Warning Patterns

| Output | Meaning |
| --- | --- |
| `ERR;BAL;CELL_NOT_ALLOWED;` | Requested cell is not in `0x300F` |
| `ERR;BAL;BAD_CELL;` | Cell number was outside 1-14 or not parsed |
| `WARN;BAL;UNUSED_CELLS_REMOVED;...` | Mask included CELL5-CELL12 and was sanitized |
| `STATUS;BLOCKED` in safety output | Safety gate failed and outputs should remain off |

## Recommended Logging

For characterization, log:

- timestamp;
- `AUTO?`;
- `BAL?`;
- `BAL SUMMARY?`;
- `BAL TEST?`;
- `BAL PHASE?`;
- `BAL ENERGY?`;
- selected applied mask;
- DMM spot checks;
- ambient and board temperature if available.
