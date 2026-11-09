# v0.5.57-r7: transaction-specific mux clock

Source revision TIMING6-MUX-CLOCK. Incremental from r6. Source only: no ARM build/link, target flash or hardware qualification performed.

## Correction

The r6 faster presets also accelerated STCOMM. SMB BOM U2/U3/U4 are ADG728BRUZ, rated for 400 kHz I2C. ADBMS6830B Table 36 gives SCL=SCK/2, so /128 and /64 exceeded the mux rating. The r6 synthetic packet model did not detect that electrical constraint. This release corrects the policy; historical r6 timing results are superseded for full temperature acquisition.

Normal ADBMS traffic uses the selected /256, /128 or /64 rate. Every STCOMM frame (command 0x0723) temporarily uses /256: 421.875 kHz SPI and approximately 210.938 kHz generated SCL at PCLK2=108 MHz. WRCOMM and RDCOMM register transfers can remain fast; STCOMM is what clocks the external I2C bus.

## Ownership and failure behavior

- Both write and combined write/read leaf paths identify STCOMM. The existing recursive SPI lock covers the complete clock-change/CS/transfer/restore interval.
- Preflight requires the actual CS input high, SPI6 idle, the HAL handle ready/unlocked, and DMA requests inactive. Requests without automatic CS ownership are rejected. A failed preflight changes neither clock nor CS and invalidates the link proof.
- The peripheral is disabled briefly while changing BR. SPI mode, other CR1 bits, original prescaler and original enable state are preserved. No HAL reinitialization, new wake, reference delay or RTOS delay is added.
- Timeout and transfer-error cleanup stops/resets SPI6 before restoring the prior configuration, rather than changing BR on a still-busy peripheral or using a long HAL abort loop. Failed samples remain rejected and link evidence is invalidated.
- An unexpected active-DMA-request violation at restoration is rejected without retiming active hardware. In that exceptional state restoration is deliberately refused; ordinary success/timeout/error paths restore the previous rate. No successful sample is claimed in this case.
- Physical-activity timestamp arithmetic uses the actual slow frame rate before restoration. Frame-specific timeout calculation also sees the actual transfer rate.
- Normal 13-byte STCOMM frames remain polling even in DMA builds. Subsequent large register transfers resume DMA at the original fast rate.

CLI `spi status` states the mux clock policy. No protection thresholds, channel requirements, CAN bitrate, reference readiness policy or output-authority gates were changed.

## Build configurations

Existing seven configurations remain. Added:

- `VEHICLE COMMS 1M SPI64 POLL Debug`
- `VEHICLE COMMS 1M SPI64 DMA Debug`

Both are inhibited profile-4 bench candidates. The normal Debug/Release configurations retain /256 polling. The /128 candidates now also use the corrected mux policy. Gated vehicle release configurations remain gated; no evidence macros were fabricated.

## Validation

Full `make -C host_tests vehicle-ci`: PASS, including nine exact CubeIDE configuration contracts, identity, profile/protection checks, production wake/session tests, DMA tests and rate SIL.

New production tests exercise /64, /128 and /256 starting rates: /256 STCOMM wire timing; exact prescaler/CR1 restoration; timeout with synthetic stuck BSY and handle recovery; CS-high and idle refusal; both leaf transfer paths; and no added wakes.

An integrated real-driver + real-DMA-transport fixture exercises fast DMA reads -> slow polling STCOMM -> fast DMA reads, including mux timeout recovery. The packet engine, MMIO and RTOS/IRQs are synthetic; this is not target execution. Native DMA-enabled target-source syntax passes at all three rates.

ASan/UBSan production clock-policy and integrated DMA fixtures: PASS; leak detection disabled in this environment. Evidence is under `validation/MUX_CLOCK_*`, `MUX_DMA_TEST.txt` and `SPI_RATE_SIL_DIV*.csv`.

Updated actual-driver/accumulator wire-time SIL, 200 scans per scenario:

| Normal divider; STCOMM always /256 | Nominal mean / max | Nominal accepted | 2.5 ms preemption cases over 100 ms | Nominal warm wakes |
|---|---|---:|---:|---:|
| /256 | 42.732 / 66.007 ms | 200/200 | 40/200 | 600 |
| /128 | 34.412 / 53.269 ms | 200/200 | 1/200 | 600 |
| /64 | 30.243 / 46.885 ms | 200/200 | 0/200 | 600 |

Compared with the old all-fast /64 model, correct mux timing costs 0.555 ms per nominal scan, excluding actual CPU/register-switch overhead. Primary AUX/mux timing assertions and persistent PEC rejection remain active. These results do not establish CPU utilization, electrical signal integrity or full RTOS worst-case latency.

## Bench acceptance still required

Compare baseline, /128 polling, /64 polling, then the corresponding DMA candidates on the same five-SMB harness. Scope SCK/CS at the bridge and SCL/SDA at a mux: confirm /64 register traffic, /256 STCOMM, correct start/stop/ACK timing, and actual rise/fall times. Verify bridge CS setup/hold/high intervals; this change does not add blanket CS delays or claim those intervals have been measured. Check PEC/counter errors, temperature freshness/accuracy, wake/session restarts, CAN latency, task timing and failure recovery under diagnostic load.

The 6822/6830B nominal 2 MHz link rating supports evaluating 1.6875 MHz; it does not qualify this installed harness. DMA suspension/abort and SRAM/cache behavior still need target evidence. Long diagnostic collisions remain separate timing work.

## Primary references

- ADG728/729 datasheet, serial timing: https://www.analog.com/media/en/technical-documentation/data-sheets/adg728_729.pdf
- ADBMS6830B datasheet, Table 36: https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6830b.pdf
- ADBMS6821/6822 timing: https://www.analog.com/media/en/technical-documentation/data-sheets/adbms6821-adbms6822.pdf
- Supplied SMB BOM: `project_sources/01-smbbom.csv`, ADG728BRUZ U2/U3/U4.
