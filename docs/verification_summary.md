# Bench Verification Summary

This document records the verification observations used to align the repository with the final working firmware snapshot.

It is not a certification report. It summarizes bench behavior observed on the experimental six-cell AEK-POW-BMS63EN / L9963E setup.

## Verified Configuration

- Pack: six physical cells.
- Active logical positions: CELL1, CELL2, CELL3, CELL4, CELL13, CELL14.
- Unused positions: CELL5-CELL12.
- Active-cell mask: `0x300F`.
- Physical wiring: unchanged during the final fix; the working result came from firmware changes.

## Firmware Snapshot Checked

The repository firmware snapshot was synchronized from the working AutoDevKit project:

```text
source/AEK_POW_BMS63CHAIN_app_mng.c
source/AEK_POW_BMS63CHAIN_app_mng.h
components/aek_pow_bms63chain_component_rla/cfg/AEK_POW_BMS63CHAIN_chain_cfg.c
components/aek_pow_bms63chain_component_rla/lib/src/AEK_POW_BMS63CHAIN_iso_mng.c
main.c
```

Key final parameters:

```text
ACTIVE_CELL_MASK                 0x300F
AUTO start delta                 30 mV
AUTO stop delta                  12 mV
AUTO tie margin                  6 mV
AUTO minimum active-cell voltage 3.30 V
AUTO maximum active-cell voltage 4.18 V
AUTO rest-current gate           0.1 A
AUTO pulse ON                    180 s
AUTO cooldown                    60 s
MULTI2 max selected cells        2
MULTI2 estimated power cap       0.75 W
```

## Healthy Measurement Pattern

Healthy operation was identified by:

```text
VREF around 4.97 V
cell voltages realistic and stable
SPI_TXRX = 3
SPI_FRAME = 4
SPI_GSW = 0
actual balancing mask contains only bits from 0x300F
CELL5-CELL12 never selected for balancing
```

The normal firmware safety path blocks balancing if VREF, SPI state, current, trim/fault state, or active-cell voltage plausibility is not acceptable.

## Balancing Verification

Manual and AUTO balancing were checked with the unused-cell guard active.

Expected safe behavior:

```text
BAL OFF        -> ACTUAL 0x0000
BAL CELL 5 ON  -> ERR;BAL;CELL_NOT_ALLOWED;
BAL MAN 0x0010 -> unused-cell request stripped or rejected
BAL AUTO       -> balances only when safety is OK and cell-voltage gates allow it
```

AUTO/MULTI2 was verified after the pack was charged enough to satisfy the configured `3.30 V` minimum active-cell gate. Valid selected masks include combinations such as:

```text
0x1000
0x2000
0x1008
0x2008
```

Cooldown is expected behavior:

```text
ACTIVE;0;COOLDOWN;1;ACTUAL;0x0000
```

## Invalid Measurement Pattern

The firmware is expected to block balancing when communication or measurement state is corrupted.

Bad patterns include:

```text
VREF far outside the expected range
all cells reading the same impossible voltage
totV physically impossible
SPI_TXRX not completed
SPI_FRAME not in the expected no-error value
```

In those cases the correct behavior is:

```text
STATE;BLOCKED_...
AUTO_REQ;0x0000
ACTUAL;0x0000
ACTIVE;0
```

## Dataset Note

No raw acquisition dataset files were present in this local repository checkout during this review. If acquisition data is published, keep it separate from the firmware snapshot or add a dedicated `data/` or release artifact with:

- raw serial/BMS logs;
- instrument-side logs;
- timestamps and synchronization notes;
- firmware commit/hash used during acquisition;
- pack configuration and cell mapping.
