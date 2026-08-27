# P42A/75s6p integration snapshot

The current generated source remains model 1.67 and uses:

- 75 series groups, six parallel cells;
- five segments of 15 groups;
- 120 temperature outputs;
- 0.1 s sample time;
- 8 mV deterministic voltage spread;
- 1% deterministic SoC spread;
- surface sensors with every sixth location weighted 65% toward core.

The application now reaches it through the stable adapter. Host regression
proves the adapter preserves the frozen source behavior across six scenarios.

The CAN image loops use manifest counts, offsets, and flattened index maps
instead of literal 15/24 topology assumptions. START, generation-tagged data,
received bitmaps, timeout, COMMIT, and CRC32 now make the AMS replacement image
atomic. The packed segment/local-index address remains one byte, so any future
topology must fit that transport contract or version the protocol.

The checked-in model is not a new candidate-cell model and is not a calibrated
fanless pack. See `../HIL_REFACTOR_VALIDATION_REPORT.md` for the evidence
boundary.
