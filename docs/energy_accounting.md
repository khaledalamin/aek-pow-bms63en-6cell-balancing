# Passive Balancing Energy Accounting

The modified firmware estimates the charge and energy removed by passive balancing. The estimate is intended for bench characterization and sanity checking, not for certified fuel gauging.

## Assumptions

| Parameter | Value used | Notes |
| --- | ---: | --- |
| Balancing resistor | about 39 ohm | Based on observed board behavior and BOM resistor values; verify your board |
| Cell capacity for SOC estimate | 3500 mAh | Example value used for percentage estimate |
| Balancing current | `Vcell / Rbal` | Recomputed using measured cell voltage |
| Balancing energy | `Vcell * Ibal * dt` | Integrated while output is active |

The resistor value must be verified against the actual board revision and ST documentation. Do not treat 39 ohm as an official universal value.

## Current and Power

Passive balancing burns energy through a resistor:

```text
Ibal = Vcell / Rbal
Pbal = Vcell^2 / Rbal
```

At 3.5 V with 39 ohm:

```text
Ibal = 3.5 / 39 = 0.0897 A, about 90 mA
Pbal = 3.5 * 3.5 / 39 = 0.314 W
```

With 60 s ON / 60 s OFF pulsing:

```text
average current ~= 90 mA * 0.5 = 45 mA
average power   ~= 0.314 W * 0.5 = 0.157 W
```

## Removed Charge

Charge removed is integrated while the balancing output is active:

```text
removed_mAh += Ibal_A * dt_hours * 1000
```

Example for 960 s ON time:

```text
dt = 960 s = 0.2667 h
Ibal ~= 0.090 A
removed ~= 0.090 * 0.2667 * 1000 = 24 mAh
```

This matches the observed example where CELL4 had about 960 s ON time and about 24 mAh removed.

## Removed Energy

Energy removed is estimated as:

```text
removed_Wh += Vcell * Ibal_A * dt_hours
```

Example at 3.5 V and 90 mA for 960 s:

```text
removed_Wh ~= 3.5 * 0.090 * 0.2667 = 0.084 Wh
```

## Estimated SOC Removed

The firmware can estimate SOC removed from a configured nominal cell capacity:

```text
soc_removed_percent = removed_mAh / cell_capacity_mAh * 100
```

For 24 mAh and a 3500 mAh cell:

```text
24 / 3500 * 100 = 0.69 %
```

This is only a rough indication. It does not replace coulomb counting or calibrated cell modeling.

## Why Passive Balancing Is Slow

Passive balancing current is small because the board dissipates excess charge as heat through resistors. At about 90 mA ON current and 50 percent duty cycle, the average removal rate is about 45 mA.

That means:

```text
20 min at 45 mA average ~= 15 mAh
30 min at 45 mA average ~= 22.5 mAh
```

So a 20-30 minute test may remove only about 15-25 mAh from a selected high cell. That is enough to validate the accounting, but not necessarily enough to visibly equalize a large imbalance.

## Why Voltage Drop Is Not Linear

Voltage does not drop linearly with removed mAh because:

- lithium-ion voltage versus SOC is not linear;
- the curve is flat over much of the mid-SOC region;
- cell voltage relaxes after balancing turns off;
- balancing current creates small voltage shifts through internal resistance;
- temperature changes voltage and resistance;
- measurement timing matters.

For this reason, energy counters are often more useful than expecting a simple voltage drop after each pulse.

## Why Balance Near Rest

During charge or discharge, measured cell voltage includes load-dependent effects. If AUTO balancing runs while current is high, the firmware may select a cell that only appears high because of transient conditions.

The bench firmware uses a near-rest current threshold around:

```text
abs(pack_current) <= 0.05 A
```

For characterization logs, record pack current with every balancing event.

## Example Output

```text
OK;BAL_ENERGY;BEGIN;
BAL_ENERGY;CELL;1;ON_TIME_S;0.0;REMOVED_mAh;0.000;REMOVED_Wh;0.00000;SOC_REMOVED_PCT;0.0000;
BAL_ENERGY;CELL;2;ON_TIME_S;0.0;REMOVED_mAh;0.000;REMOVED_Wh;0.00000;SOC_REMOVED_PCT;0.0000;
BAL_ENERGY;CELL;3;ON_TIME_S;0.0;REMOVED_mAh;0.000;REMOVED_Wh;0.00000;SOC_REMOVED_PCT;0.0000;
BAL_ENERGY;CELL;4;ON_TIME_S;960.0;REMOVED_mAh;24.000;REMOVED_Wh;0.08400;SOC_REMOVED_PCT;0.6857;
BAL_ENERGY;CELL;13;ON_TIME_S;0.0;REMOVED_mAh;0.000;REMOVED_Wh;0.00000;SOC_REMOVED_PCT;0.0000;
BAL_ENERGY;CELL;14;ON_TIME_S;0.0;REMOVED_mAh;0.000;REMOVED_Wh;0.00000;SOC_REMOVED_PCT;0.0000;
OK;BAL_ENERGY;END;
```

## What Good Looks Like

- ON time rises only for active cells.
- Estimated current is near `Vcell / 39 ohm`.
- About 960 s ON time at 3.5 V gives about 24 mAh.
- Energy counters stop increasing during cooldown or `BAL OFF`.
- CELL5-CELL12 never appear in the energy report.

## What Bad Looks Like

- Removed mAh increases when actual balancing mask is zero.
- CELL5-CELL12 have energy counters.
- Current estimate is far from the expected resistor calculation.
- ON time increases during safety-blocked state.
- Reported voltage is impossible but energy still accumulates.

If any bad pattern appears, stop balancing and inspect the apply-mask and accounting code paths.
