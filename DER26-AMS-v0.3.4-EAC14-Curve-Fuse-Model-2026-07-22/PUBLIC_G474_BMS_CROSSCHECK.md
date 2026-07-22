# Public g474-bms Mixed-Chain Cross-Check

Authored by Mahad Faisal, 2026.

## Scope and source boundary

The public Manchester Stinger Motorsports repository was reviewed as an
independent historical implementation clue:

<https://github.com/ManchesterStingerMotorsports/g474-bms>

No source code, constants, tables, or comments were copied from that project.
The repository has no root license file in the reviewed revision, and several
of its driver paths omit protections required by this AMS firmware. Device
behavior and transaction legality were therefore decided from Analog Devices
documentation, not from the public implementation.

## What the public history established

- Commit [`538caf4`](https://github.com/ManchesterStingerMotorsports/g474-bms/commit/538caf4b9731b97b6f5ba027828f990abc0c4fc1)
  records a tested ADBMS6830 + ADBMS2950 RDSID daisy chain.
- Commit [`a313507`](https://github.com/ManchesterStingerMotorsports/g474-bms/commit/a3135075f35692897059923509aff617cf88fbdc)
  records a simpler driver tested with both device types.
- Commit [`6c2b354`](https://github.com/ManchesterStingerMotorsports/g474-bms/commit/6c2b354e5afff637e3154fd8ba19da3018a1f01c)
  records later ADBMS2950 current-measurement work.
- The reviewed current revision, `919a8a8`, configures `TOTAL_AD29` as zero.
  It is therefore not evidence that its latest production image still runs a
  2950 in the chain.

This history is useful evidence that the two products can coexist on one
isoSPI chain and respond to common commands. It is not a drop-in driver or a
safety argument for the DER26 topology.

## Authoritative protocol conclusions

The final DER26 ring remains:

```text
String A -> 5 x ADBMS6830B SMB -> 1 x ADBMS2950B APM -> String B
```

The [ADBMS2950B data sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adbms2950b.pdf)
states that a read or write may stop at a device boundary: after the four-byte
command/PEC prefix, each accessed device contributes eight bytes. Devices not
accessed by that leading subset do not increment their command counter.
Consequently:

- a five-device SMB write from String A is 4 + (5 x 8) = 44 bytes;
- a one-device APM write from String B is 4 + (1 x 8) = 12 bytes;
- padding the SMB write with an APM packet is neither necessary nor desired;
- sending either subset from the opposite end would select different physical
  devices and is rejected.

The [ADBMS6830B data sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6830b.pdf)
defines the 6830B device ID as `0x03` in RDSID byte 1 bits [6:1]. The 2950B
device ID is `0x06` in its documented SID field. A correct PEC therefore does
not prove that the expected product occupies a position.

## Implemented safeguards

- All five String-A devices must return product ID `0x03` before the first SMB
  configuration write.
- The existing String-B APM check continues to require product ID `0x06`
  before any APM-specific write.
- Each driver records an immutable `write_string`; subset register writes are
  rejected if the active string differs from that owner.
- The accumulator validates exact counts, embedded-array pointers, shared
  SPI/timer/CS bindings, active ends, and write owners before physical reads,
  mux writes, or balance writes.
- CLI health output exposes the final-ring topology result, current and sticky
  SID identity masks, each 6830 device ID, and both active/write strings.
- The legacy `spi probeb` command now uses the one-device ADBMS2950 RDSID path
  instead of parsing a reverse-end APM-plus-SMB response as five SMB packets;
  standalone probe/scope traffic forces SMB counter resynchronization.
- Host regressions lock the 44-byte and 12-byte frame lengths and prove that
  opposite-end writes generate no SPI or GPIO activity.

## Deliberately not adopted

The public project contains useful bring-up history, but the reviewed paths
also include unbounded HAL waits, incomplete command-counter validation, and
configuration variants that disable the ADBMS2950. None of those behaviors
were imported. DER26 retains its bounded waits, PEC and command-counter checks,
single transaction owner, readback verification, stale-data invalidation, and
advisory-only APM safety boundary.

## Validation boundary

Host SIL can prove encoding, ownership, bounds, and failure propagation. It
cannot prove transformer polarity, physical device order, target timing, APM
shunt polarity/scaling, or successful communication on the assembled ring.
Those remain current-limited LV bench tests with BMS_OK and balancing
physically inhibited.
