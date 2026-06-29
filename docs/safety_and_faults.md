# Safety Gate and Fault Handling

Balancing is allowed only when the firmware safety gate considers the measurement and diagnostic state acceptable. If the gate fails, the firmware forces balancing commands off.

This is a bench guard, not a complete safety architecture.

## Why a Safety Gate Is Required

The six-cell setup intentionally uses a non-standard physical arrangement in a 14-channel board/holder path. That makes it important to block balancing whenever the firmware cannot trust the measurements.

The most important failure classes are:

- wrong or incomplete sense wiring;
- impossible active-cell voltage readings;
- invalid VREF/reference measurement;
- trim/calibration not ready or invalid;
- EEPROM or RAM CRC flags;
- ground/reference faults;
- IC overtemperature;
- pack current not near rest;
- accidental request to balance unused shorted channels.

## Safety Gate Inputs

| Check | Why it matters | Good condition |
| --- | --- | --- |
| VREF valid | Cell readings depend on a valid reference | Around expected range, observed about 4.97 V |
| VTREF valid | Confirms reference measurement path | Same expected range as bench setup |
| Trim/calibration OK | Prevents using uncalibrated readings | `TRIM_CAL_OK_DIAG = 1`, `TRIM_CAL_OK_MEAS = 1` |
| EEPROM CRC OK | Calibration/config integrity | `EEPROM_CALOFF = 0`, `EEPROM_CALRAM = 0`, `EEPROM_SECT0 = 0` |
| RAM CRC OK | Runtime memory integrity | `RAMCRC = 0` |
| Ground/reference faults absent | Prevents invalid measurements | no AGND/DGND/CGND/GNDREF loss |
| Chip overtemperature absent | Prevents heating an already hot device | OT flag clear |
| Pack current near zero | Balancing comparison should be near rest | about `abs(current) <= 0.1 A` |
| Active cells realistic | Blocks impossible measurement states | about 3.30-4.18 V for this bench test |
| Unused cells blocked | Avoids heating shorted channels | applied mask never includes CELL5-CELL12 |

## Why VREF Must Be Valid

If VREF is wrong, every cell reading may be wrong. Balancing based on a bad reference can select the wrong cell or keep balancing after the real voltage is no longer high.

Observed good behavior in this bench setup:

```text
VREF about 4.97 V
```

Do not hard-code this as a universal board guarantee. Verify against the official ST documentation and the actual board readings.

## Why Trim and EEPROM Flags Must Be OK

The L9963E measurement path relies on trim/calibration data. If trim flags or EEPROM CRC flags report a problem, cell voltage values may not be reliable enough for balancing decisions.

Observed good trim/calibration state:

```text
TRIM_CAL_OK_DIAG = 1
TRIM_CAL_OK_MEAS = 1
EEPROM_CALOFF = 0
EEPROM_CALRAM = 0
EEPROM_SECT0 = 0
RAMCRC = 0
GSW = 0
```

## Why Impossible Cell Readings Block Balancing

The safety gate explicitly blocks unrealistic active-cell voltages. One practical failure case during development was an impossible state such as all cells reading around 5.5 V. That state must never be used to enable passive balancing.

For the current bench tests, the firmware uses an active-cell voltage window around:

```text
3.30 V <= active cell <= 4.18 V
```

This is a bench validation window, not a universal cell limit.

## Why Pack Current Should Be Near Zero

Balancing compares cell voltages to decide which cell is highest. During load or charge current, measured voltage includes IR drop, contact resistance, harness resistance, and relaxation effects. AUTO balancing should usually run near rest, so the voltage spread more closely reflects cell open-circuit behavior.

Current threshold used here:

```text
abs(pack_current) <= about 0.1 A
```

## Why Unused Cells Must Be Blocked

CELL5-CELL12 are not physical cells in this setup. They are bridged/shorted in the unused-channel region. Enabling passive balancing on those channels could place balancing current into wiring or shorted nodes rather than a real cell.

The firmware therefore uses:

```text
ACTIVE_CELL_MASK = 0x300F
unused mask      = 0x0FF0
```

Any applied balancing mask must satisfy:

```text
(applied_mask & 0x0FF0) == 0
```

## Good Safety Report

Example:

```text
OK;BAL_SAFETY;STATUS;OK;STATE;READY;VREF;4.9700;CURR;0.0040;
```

What good looks like:

- `STATUS;OK`;
- VREF near expected range;
- current near zero;
- state is `READY`, `WAIT_DELTA`, `STOPPED_DELTA_SMALL`, `COOLDOWN`, or `BALANCING`;
- no critical trim/fault report problems.

## Blocked Safety Report

Example:

```text
OK;BAL_SAFETY;STATUS;BLOCKED;STATE;BLOCKED_SAFETY;VREF;5.9000;CURR;0.0030;
```

What bad looks like:

- `STATUS;BLOCKED`;
- VREF outside expected range;
- active-cell voltages impossible;
- current above rest threshold;
- critical fault flags active;
- actual mask still non-zero after safety failed.

If the last item happens, stop testing and inspect the firmware path immediately.

## Fault Report Guidance

Run:

```text
FAULT?
TRIM?
BAL SAFETY?
```

Interpret fault names with the official L9963E documentation. This repo intentionally avoids inventing exact official fault meanings where they must be verified against ST material.

Observed in the validated bench setup:

- trim/calibration flags OK;
- EEPROM/RAM CRC flags OK;
- ground/reference critical faults absent;
- chip overtemperature absent;
- only `wuIsoLine` remained active as a non-critical wake/status flag.

Verify that behavior on your own board and firmware version.
