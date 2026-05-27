# Firmware

This folder separates the experimental modifications from the ST-generated/demo project.

## Layout

```text
firmware/
|-- modified_files/
|   |-- AEK_POW_BMS63CHAIN_app_mng.c
|   `-- AEK_POW_BMS63CHAIN_app_mng_PATCH_NOTES.md
|-- patches/
|   `-- README.md
|-- full_project/
|   `-- README.md
`-- notes/
    `-- ude_debug_watchlist.md
```

## Recommended Publication Model

The safest open-source layout is:

1. Publish original documentation, notes, and your own modifications.
2. Do not publish a complete ST-generated project unless your ST license explicitly allows it.
3. Instruct users to recreate the base `BMS_CHAIN` project in ST AutoDevKit / SPC5Studio.
4. Tell users to replace or patch only the modified files.

## Modified File

The main modified file is:

```text
firmware/modified_files/AEK_POW_BMS63CHAIN_app_mng.c
```

It was extracted from the supplied project ZIP in this workspace because it already contains the active-cell mask, balancing command parser, AUTO balancing, safety gate, and energy accounting additions.

Review [modified_files/AEK_POW_BMS63CHAIN_app_mng_PATCH_NOTES.md](modified_files/AEK_POW_BMS63CHAIN_app_mng_PATCH_NOTES.md) before applying it to another ST project version.

## Build Flow

1. Recreate/import the ST BMS chain demo in AutoDevKit / SPC5Studio.
2. Build the original demo once.
3. Replace the original `source/AEK_POW_BMS63CHAIN_app_mng.c` with the modified file.
4. Rebuild.
5. Flash with UDE.
6. Keep balancing off until serial diagnostics and UDE watch variables look correct.

## Important License Note

If your ST license does not allow redistributing the complete generated project, publish only patches and modified files, and instruct users to recreate the base project using ST AutoDevKit.

See [../NOTICE.md](../NOTICE.md).
