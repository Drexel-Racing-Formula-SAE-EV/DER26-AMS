# Current Source Status

This repository cleanup is synchronized from the latest **complete AMS source snapshot available to the cleanup workspace**, corresponding to the v0.5.15 safety-review line. It intentionally does not pretend that later review ideas are present unless the source is actually in this tree.

## Known open review items in this source snapshot

The latest principal-level review identified items that should be treated as open until their source changes and target validation are committed here:

1. **CAN DETAIL publisher stack usage** — review identified a very large automatic DETAIL-generation object relative to the CAN task stack. The architectural fix is to move staging storage out of the task stack rather than only increasing the task allocation.
2. **CAN interrupt coverage** — verify dedicated TX and status/error interrupt vectors match every enabled HAL CAN notification.
3. **Stack-overflow panic ordering** — physical BMS fail-low action should precede best-effort diagnostics that may depend on corrupted RTOS state.
4. **Runtime version/provenance** — the current source contains stale semantic-version/fingerprint fields that should be reconciled with the release/source revision.
5. **Target stack qualification** — real Cortex-M7 high-water measurements are required under worst-case CAN/ADBMS/estimator/CLI load.
6. **Release artifact qualification** — final vehicle releases need target ELF/HEX/MAP, toolchain identity, configuration manifest, and hardware/HIL evidence.

These are intentionally tracked in one current-status document instead of leaving many dated patch-review files scattered through the repository.

## What this cleanup changed

The cleanup itself is organizational:

- synchronized the old repository's `AMS/` project to the latest complete source snapshot;
- removed generated `Debug/` output and IDE-local workspace metadata;
- removed obsolete dated patch/review/release-note clutter from the repository root;
- retained HIL, reference tools, CI, vendor code, and active firmware contracts;
- added a single documentation index and maintainer-oriented repository map.

No unresolved firmware item above should be considered fixed merely because the repository was reorganized.
