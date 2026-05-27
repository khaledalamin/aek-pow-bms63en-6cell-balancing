# Contributing

Contributions are welcome if they improve clarity, safety, reproducibility, or portability.

## Safety First

Do not submit changes that bypass the safety gate, enable balancing on CELL5-CELL12, or present experimental wiring as officially supported.

When changing firmware behavior, include:

- exact hardware setup;
- cell map;
- serial logs from `TRIM?`, `FAULT?`, `BAL SAFETY?`, `AUTO?`, and `BAL ENERGY?`;
- UDE variables watched during validation;
- explanation of thermal and current assumptions.

## Documentation Rules

- Separate observed behavior from assumptions.
- Use "verify against official ST documentation and board schematic" when a detail depends on board revision or ST material.
- Do not invent official claims.
- Keep warnings practical and actionable.

## Firmware Rules

- Keep `ACTIVE_CELL_MASK = 0x300F` for this six-cell configuration unless the repository is explicitly generalized.
- Ensure any requested balancing mask is sanitized before being applied.
- Keep `FAIR_SINGLE` as the default strategy.
- Mark multi-cell balancing as experimental.
- Add serial output examples when adding commands.

## Licensing

Do not contribute ST-generated files, ST libraries, or full demo projects unless you have confirmed redistribution is allowed.

Prefer patches, notes, and small original modifications that users can apply to a recreated ST base project.
