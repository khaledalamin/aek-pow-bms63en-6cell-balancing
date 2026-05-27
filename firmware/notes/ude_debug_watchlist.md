# UDE Debug Watchlist

Use UDE Starterkit 2021 to confirm that serial output, internal state, and actual balancing command fields agree.

Exact variable names may change with ST generated project versions. The modified file in this repository contains the local symbols to search for.

Example UDE watch capture:

![UDE watch variables](../../images/ude_watch_balancing_variables.png)

## Core Variables

Watch:

- active cell voltage array for CELL1, CELL2, CELL3, CELL4, CELL13, CELL14;
- `AEK_POW_BMS63CHAIN_balCtrlMode`;
- `AEK_POW_BMS63CHAIN_balManualMask`;
- `AEK_POW_BMS63CHAIN_balAutoRequestedMask`;
- actual applied balancing command/mask fields;
- `AEK_POW_BMS63CHAIN_balAutoActive`;
- `AEK_POW_BMS63CHAIN_balAutoCooldown`;
- `AEK_POW_BMS63CHAIN_balAutoLastDeltaV`;
- `AEK_POW_BMS63CHAIN_balAutoSelectedCellIdx`;
- min/max active-cell indexes;
- per-cell ON time;
- removed mAh and Wh arrays.

## Variables Visible in the Captured Watch Window

The included screenshot shows the type of values that were useful during bring-up:

| Watch expression | Purpose |
| --- | --- |
| `dbg_bms63_raw_vcell1` / `dbg_bms63_raw_vcell2` / `dbg_bms63_raw_vcell13` / `dbg_bms63_raw_vcell14` | Raw converted readings used to compare channels |
| `dbg_bms63_vcell1_alt_9235` and related alternate voltage variables | Cross-check active-cell conversion paths |
| `dbg_bms63_vcell1_current` and related current voltage variables | Current active-cell voltage values used during debug |
| `AEK_POW_BMS63CHAIN_balManualMask` | Manual requested balancing mask |
| `AEK_POW_BMS63CHAIN_balAutoMask` | AUTO requested balancing mask in that firmware revision |
| `AEK_POW_BMS63CHAIN_balAutoActive` | AUTO ON-pulse active flag |
| `AEK_POW_BMS63CHAIN_balAutoCooldown` | AUTO cooldown flag |
| `AEK_POW_BMS63CHAIN_balCtrlMode` | OFF, manual, or AUTO balancing control mode |
| SPI frame/status fields | Confirms whether communication is completing or reporting errors |

The screenshot is a debug example, not a pass/fail certificate. Interpret SPI and fault/status fields against the current firmware and official documentation.

## Safety Variables

Watch the data used by the safety gate:

- VREF / VTREF measurements;
- trim/calibration OK flags;
- EEPROM CRC flags;
- RAM CRC flag;
- ground/reference fault flags;
- chip overtemperature flag;
- pack current;
- active-cell enabled flags;
- active-cell voltages.

## Good Watch Pattern

- requested mask and actual mask are zero after `BAL OFF`;
- actual mask never has bits `0x0FF0`;
- selected AUTO cell is one of 1, 2, 3, 4, 13, 14;
- energy counters increase only when actual mask is non-zero;
- safety-blocked state forces actual mask to zero;
- serial `AUTO?` matches watched state.

## Bad Watch Pattern

Stop with `BAL OFF` if:

- actual mask includes CELL5-CELL12;
- energy counters rise while actual mask is zero;
- active cells show impossible voltages;
- VREF is invalid;
- safety gate reports blocked but command fields still enable balancing.
