# Changelog

## Unreleased - 2026-06-29

Updated the repository to match the final working AutoDevKit firmware snapshot.

Changed:

- synchronized `AEK_POW_BMS63CHAIN_app_mng.c` and `.h` from the working workspace project;
- documented final AUTO parameters: 180 s ON, 60 s cooldown, 0.1 A near-rest gate, and 0.75 W `MULTI2` cap;
- added `BAL TEST?`, `BAL PHASE?`, recovery, and software-only injection command documentation;
- added a bench verification summary covering active-cell mask, healthy VREF/SPI state, unused-cell blocking, and AUTO/MULTI2 validation;
- updated the ST community draft with the public GitHub repository URL.

## 0.1.0 - 2026-05-27

Initial repository organization for the experimental six-cell AEK-POW-BMS63EN / L9963E passive balancing project.

Added:

- top-level README;
- wiring, balancing, command, safety, energy, testing, troubleshooting, and community-post docs;
- extracted modified `AEK_POW_BMS63CHAIN_app_mng.c`;
- patch notes for the modified firmware file;
- image placeholders and copied bench photos;
- license, notice, disclaimer, contribution guide, issue templates, and PR template.

## 0.1.1 - 2026-05-27

Expanded the modified firmware set after scanning the supplied ST project ZIP.

Added:

- `AEK_POW_BMS63CHAIN_app_mng.h`;
- `main.c`;
- `AEK_POW_BMS63CHAIN_chain_cfg.c`;
- `AEK_POW_BMS63CHAIN_iso_mng.c`;
- findings note explaining why these files are part of the experimental firmware set.
