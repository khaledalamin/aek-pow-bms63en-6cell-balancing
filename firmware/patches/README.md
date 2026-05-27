# Patches

This directory is reserved for clean patches against a recreated ST base project.

Recommended future workflow:

1. Recreate the ST `BMS_CHAIN` demo in AutoDevKit / SPC5Studio.
2. Commit or archive a clean local baseline if your license allows local version control.
3. Apply the modified `AEK_POW_BMS63CHAIN_app_mng.c`.
4. Generate a patch that contains only the original additions and not unrelated generated/build files.

Example local command after a clean local baseline:

```text
git diff -- source/AEK_POW_BMS63CHAIN_app_mng.c > firmware/patches/aek_pow_bms63chain_app_mng_6cell_balancing.patch
```

Do not publish patches that include ST proprietary material beyond what your license allows.
