# v0.5.46 — five-SMB CLI/inhibit recovery

## What was compared

Sources inspected: v0.5.45, September 3 v2.6.16, August 25 v0.5.13,
August 18 v0.5.9 five-SMB CS_B temperature bench, and August 13 v0.5.5.
The August packages are historical candidates, not proof of the exact binary
used on the user's earlier successful test.

The August 18 package explicitly selects five SMBs via String B/CS_B.
The current default five-SMB BENCH Validation selects String A/CS_A.
Both use SPI mode 3 and 1 ms low / 1 ms high wake intervals per device.
POST was already present and enabled in the August 13/18 source. It was not
simply introduced after September 3.

Another material change: August 18 established read-only measurement transport
from identity/timer evidence independently of POST safety qualification. The
new source requires clean startup/config/POST evidence before periodic
acquisition. A failed startup can therefore prevent ordinary measurements,
even when some raw communication works. v0.5.46 retains the current readiness
checks; use the reported startup stage and per-device transport masks to
identify the actual failing condition.

## Implemented changes

- Missing off-state proof/write failure is a maintenance request. Service at
  most once per scan, then once per second after three consecutive failures.
- Maintenance does not cancel conversion or temperature-settle waits.
- Real output/balance edges still cancel waits and can be serviced mid-scan.
  SPI-owned shadow activity is upgraded to an edge at scan entry.
- New edge generations survive completion of an older transaction. SPI
  ownership extends through physical-result publication, including CLI writes.
- Successful inhibit clears the write fault/failure streak. Redundant
  write-fault-only clear transactions are removed. Active-balance recovery
  uses the scan-boundary clear result rather than issuing it a second time.
- Emergency inhibit uses a guarded transaction session. Writes reuse the
  session except during SNAP. Guard expiry still performs a full wake.
  Pulse durations and SPI speed are unchanged.
- Status, configuration and automatic cell open-wire scheduling use kernel
  deadlines, skip overdue slots, and handle tick wrap. Scan count is telemetry.
- CLI clear uses the existing bounded CLI try-lock. Busy queues an urgent
  clear and reports that completion is unverified; it does not enter lock panic.
- Five-SMB BENCH Validation CLI priority is 9, below error/current/ADBMS/CAN,
  above estimator/fan/AIR/IMD. One-SMB remains 13; other profiles remain 4.
- Boot banner prints SMB count, physical count, selected A/B path, scan rate
  and CLI priority. `status` includes inhibit streak/deferred/edge/deadline.

Temperature acquisition remains enabled with its existing scan cadence.
Error stack remains 512 words. BMS_OK and balancing remain inhibited by the
bench profile. No off-state success, missing cell, or PEC result is fabricated.

## Build and connect

Import the AMS project from this archive into a fresh CubeIDE workspace or
replace the existing source deliberately. Clean and build
`BENCH Validation 5-SMB Debug`. Confirm the debugger uses that build's ELF,
not `Bench1_Debug`. This package contains source, not an ARM-built ELF.

Default five-SMB path: AMS A/J1 to the SMB chain's A entry. The historical
August 18 bench used AMS B/J2. Confirm the actual harness before concluding
that a five-device failure is a register bug.

For an intentional comparison using the old B-port harness, the source already
supports defining `ACCUMULATOR_SMB_STRING=STRING_B` for the five-SMB bench.
Apply the define consistently to every compiled source, clean/rebuild, and
verify `path:B/J2` in the boot banner. Do not make this change for an installed
mixed SMB/APM ring. The standard shipped five-SMB configuration remains A/J1.

## First bench commands

Keep the pack test in the inhibited bench scope. Run short diagnostics first:

```text
ver
spi onboard
status
uart status
rtos
spi lockdiag
spi status
volt
temp
```

Expected banner: v0.5.46, five physical SMBs, A/J1, CLI priority 9.
`spi status` may still report busy while a scan owns the transport; `spi onboard`
and `spi lockdiag` do not take that lock. Retry a busy status command after
the scan. A busy response is not a transport validation pass.

With the target halted, collect:

```gdb
set pagination off
frame 0
bt 12
p app.acc.smb.post
p app.acc.smb_transport_ready
p app.acc.last_balance_mute_ok
p app.acc.last_balance_durable_zero_verified
p app.adbms_urgent_mute_fail_streak
p app.adbms_urgent_mute_service_count
p app.adbms_urgent_mute_fail_count
p app.adbms_last_scan_duration_ms
p app.adbms_scan_deadline_miss_count
p/x app.acc.smb.health.last_pec_pass_mask
p/x app.acc.smb.health.sticky_pec_fail_mask
p/x app.acc.smb.health.sticky_cmd_counter_mismatch_mask
p/x app.acc.smb.health.balance_cfgb_mismatch_mask
p/x app.acc.smb.health.balance_pwma_mismatch_mask
p/x app.acc.smb.health.balance_pwmb_mismatch_mask
p app.acc.smb.session.full_wake_count
p app.acc.smb.delay_timeout_count
```

A complete five-SMB PEC mask is 0x001F. Interpret partial masks in the driver's
device ordering, not blindly as a physical break location. Sample scan/wake
counters twice to measure rates. Repeated failed maintenance should now have
a bounded rate; it must still leave off-state qualification false.

## Verification and remaining limits

Host tests cover persistent failure, bounded retry rate, recovery becoming
quiet, newer edge preservation, busy CLI queueing, wall-clock diagnostics,
tick wrap, a missing tail of a five-device chain, guarded write reuse, expiry,
nested sessions, and SNAP exclusion. Production BMS ownership and bench/testday
compile locks are tested separately. Full SIL and driver unit suites were run,
including AddressSanitizer/UndefinedBehaviorSanitizer.

Reproduce the focused tests with `make -C AMS/host_tests inhibit-regression-test`.
Run normal suites with `make -C AMS/host_tests test unit`.

No ARM compiler is available here. No target timing, hardware test or physical
five-SMB recovery is claimed. The initial five-SMB inhibit failure is not yet
identified: port selection, power/link integrity, PEC/counter integrity and
register verification remain candidates.

The earlier UNALIGNED UsageFault with corrupted r7 is unresolved. This release
does not claim to fix it. Preserve the compiled map/ELF; if it recurs, collect
the fault registers, stacked exception frame and adjacent stack allocations.

Healthy five-SMB scans may still exceed 100 ms because of full-chain wakes.
This revision restores a better opportunity for bench CLI execution and fixes
time-based diagnostics; it does not certify acquisition or vehicle timing.
