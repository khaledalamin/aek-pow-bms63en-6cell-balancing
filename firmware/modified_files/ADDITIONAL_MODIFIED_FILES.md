# Additional Modified Files Found in the ST Project ZIP

The first repository pass documented `AEK_POW_BMS63CHAIN_app_mng.c` as the main modified file. A follow-up scan of the supplied ST project ZIP found additional files that appear to be part of the working experimental firmware setup.

Because the official ST baseline project is not available in this repository, this classification is based on local evidence: custom symbols, custom comments, project timestamps, and functional coupling to the modified application manager.

## Files to Treat as Part of the Experimental Firmware Set

| File in this repo | Original project path | Why it matters |
| --- | --- | --- |
| `AEK_POW_BMS63CHAIN_app_mng.c` | `source/AEK_POW_BMS63CHAIN_app_mng.c` | Main serial command parser, active-cell mask, AUTO balancing, safety gate, energy accounting |
| `AEK_POW_BMS63CHAIN_app_mng.h` | `source/AEK_POW_BMS63CHAIN_app_mng.h` | Declares balancing control enum, serial polling/API additions, `app_serialStep_GUI`, and `BAL_ALLOWED_MASK = 0x300F` |
| `main.c` | `main.c` | Calls `AEK_POW_BMS63CHAIN_app_serialStep_GUI()` from `main_core0()` for GUI/serial servicing |
| `AEK_POW_BMS63CHAIN_chain_cfg.c` | `components/aek_pow_bms63chain_component_rla/cfg/AEK_POW_BMS63CHAIN_chain_cfg.c` | Encodes the six active cell enables and ISO-SPI pin choices used in the bench setup |
| `AEK_POW_BMS63CHAIN_iso_mng.c` | `components/aek_pow_bms63chain_component_rla/lib/src/AEK_POW_BMS63CHAIN_iso_mng.c` | Contains bench-specific ISO-SPI behavior: LOW frequency, guarded TXEN drive, and TXAMP not driven when unmapped |

## Important Configuration Evidence

The generated chain configuration enables only the physical cells:

```text
CELL1  enabled
CELL2  enabled
CELL3  enabled
CELL4  enabled
CELL5  disabled
CELL6  disabled
CELL7  disabled
CELL8  disabled
CELL9  disabled
CELL10 disabled
CELL11 disabled
CELL12 disabled
CELL13 enabled
CELL14 enabled
```

That matches:

```text
ACTIVE_CELL_MASK = 0x300F
```

The `configuration.xml` file in the original ST project ZIP also shows the same cell-enable pattern. It is not copied into this repo because it is a large generated project file; users should recreate the base project in AutoDevKit/SPC5Studio and reproduce these settings.

## UDE Workspace Files

The ZIP also contains UDE workspace files with watched variables:

```text
UDE/debug.wsx
UDE/debug_052226180357.wsx
UDE/debug_052526114921.wsx
```

These are useful for local debugging, but they are tool/workspace artifacts rather than firmware source. The repository documents the watched variables in:

```text
firmware/notes/ude_debug_watchlist.md
```

## Files Not Treated as Source Modifications

The ZIP contains many build outputs and regenerated component files with recent timestamps:

```text
build/
.dep/
*.o
*.lst
*.elf
*.hex
*.map
```

Those are not source files and should not be published as the reproducible change set.

Several generated component headers/sources also have recent timestamps because the project was regenerated or rebuilt. Unless a file contains custom bench-specific comments or symbols, it should be treated as generated ST project material and recreated from AutoDevKit/SPC5Studio rather than manually copied.

## Recommended Apply Order

After recreating the base ST demo project:

1. Reproduce the AutoDevKit/SPC5Studio configuration for one node and active cells 1, 2, 3, 4, 13, and 14.
2. Confirm the generated `AEK_POW_BMS63CHAIN_chain_cfg.c` enables only those six cells.
3. Apply `AEK_POW_BMS63CHAIN_iso_mng.c` only if your ISO-SPI wiring/debug behavior matches this bench setup.
4. Replace `AEK_POW_BMS63CHAIN_app_mng.h`.
5. Replace `AEK_POW_BMS63CHAIN_app_mng.c`.
6. Replace or manually update `main.c` so core 0 services `AEK_POW_BMS63CHAIN_app_serialStep_GUI()`.
7. Rebuild and validate with `BAL OFF`, `TRIM?`, `FAULT?`, and `BAL SAFETY?`.
