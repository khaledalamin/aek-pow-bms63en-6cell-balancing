# Draft ST Community Post

Title suggestion:

```text
AEK-POW-BMS63EN / L9963E: experimental 6-cell passive balancing on a 14-cell holder setup
```

Post draft:

```text
Hello ST Community,

I am sharing an experimental educational project using the ST AEK-POW-BMS63EN board based on the L9963E battery monitoring IC, with an AutoDevKit / SPC5Studio project targeting SPC58EC and debug through UDE Starterkit 2021.

The setup is non-standard: the mechanical holder/board path is a 14-cell configuration, but I am using only six physical cells:

- CELL1
- CELL2
- CELL3
- CELL4
- CELL13
- CELL14

The unused middle channels, CELL5 to CELL12, are bridged/shorted in the sense/balance wiring according to the board requirements so that the device sees a valid continuous stack. I verified the bridge region with a DMM before enabling the BMS board. The exact wiring must be checked against the official ST documentation, schematic, and the user's own measurements before anyone tries to reproduce this.

In firmware I added an active-cell mask:

ACTIVE_CELL_MASK = 0x300F

That means balancing is allowed only for CELL1, CELL2, CELL3, CELL4, CELL13, and CELL14. CELL5-CELL12 are always blocked. Manual balancing commands and automatic balancing requests are both sanitized through this mask.

The project adds:

- serial commands for manual balancing and AUTO balancing;
- safety checks before any balancing output is applied;
- trim/EEPROM/RAM CRC and fault diagnostics;
- active-cell voltage sanity checks;
- near-zero current requirement for AUTO balancing;
- pulsed passive balancing, currently 180 s ON / 60 s OFF in this experimental firmware snapshot;
- one-cell-at-a-time balancing as the default strategy;
- optional experimental two-cell strategy for non-adjacent cells;
- passive balancing energy accounting in mAh and Wh.

The safety gate blocks balancing if VREF is invalid, trim/calibration flags are not OK, EEPROM/RAM CRC flags are set, critical ground/reference or chip overtemperature flags are present, pack current is not near zero, or active cell voltages are physically unrealistic.

Observed bench behavior:

- VREF was stable around 4.97 V.
- Pack voltage was around 20.8-21.0 V.
- Active cells were around 3.44-3.52 V.
- Trim/calibration flags were OK.
- EEPROM and RAM CRC flags were clear.
- Only wuIsoLine remained active as a non-critical wake/status flag in my tests.
- Manual balancing worked on allowed cells.
- AUTO balancing selected CELL4, CELL13, or CELL14 depending on which was highest.
- CELL5-CELL12 remained blocked.
- Energy accounting matched the expected passive balancing current, about 3.5 V / 39 ohm = 90 mA.

Example: about 960 s of ON time on CELL4 corresponded to about 24 mAh removed, which is consistent with the resistor estimate.

I also added a guard for impossible measurement states, such as all cells reading around 5.5 V. In that case balancing is forced off.

This is not an official ST reference design and not a safety-certified BMS. It is an experimental implementation for learning, debugging, and community discussion. I would welcome corrections, especially on the recommended wiring treatment for unused channels, diagnostic flag interpretation, and whether there is a cleaner ST-supported way to configure a reduced physical cell count on this board family.

Repository:
https://github.com/khaledalamin/aek-pow-bms63en-6cell-balancing

Thank you.
```

## Attachments to Include

- Board overview photo.
- Close-up of unused-channel bridge/short area.
- GUI monitoring screenshot showing CELL1-CELL4 and CELL13-CELL14 active, with CELL5-CELL12 unused.
- UDE watch screenshot showing balancing masks, selected voltages, and control state.
- Serial terminal output from `TRIM?`, `FAULT?`, `BAL SAFETY?`, `AUTO?`, and `BAL ENERGY?`.
- Optional GUI screenshot showing CELL13 balancing.
- Optional GUI diagnostics screenshot, with notes explaining any non-critical status flags.

## Tone Notes

Keep the post humble:

- say "experimental";
- ask for corrections;
- avoid claiming official support;
- include measured observations separately from assumptions;
- tell readers to verify against official ST documentation.
