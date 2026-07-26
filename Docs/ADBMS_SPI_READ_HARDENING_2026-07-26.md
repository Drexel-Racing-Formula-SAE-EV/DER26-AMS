# DER26 AMS ADBMS SPI Read Hardening

**Target baseline:** DER26 AMS v0.3.6
**Implementation date:** 2026-07-26
**Scope:** ADBMS6830B SMB read integrity and shared STM32-to-ADBMS6822 SPI timing, with matching ADBMS2950 transport hardening

## 1. Disposition

This change closes three confirmed software defects in the ADBMS read path and adds an explicit local chip-select timing contract. It deliberately does **not** shorten the existing wake sequence or claim normal 10 Hz scan qualification.

The 1 Hz `AMS_HW_BRINGUP` profile remains the intended next hardware profile. Promotion to the normal 10 Hz vehicle scan remains blocked on measured full-ring timing and communication evidence.

## 2. Implemented changes

### 2.1 Cell reads no longer erase temperature freshness

`adbms6830_read_cell_voltages()` now resets only:

- `last_cell_updated_mask[]`
- `last_cell_pec_mask[]`

It no longer clears `last_temp_updated_mask[]`. Cell and AUX/temperature acquisitions are separate freshness epochs. A voltage read, including a failed voltage read, cannot invalidate the previous temperature epoch as an unrelated side effect.

### 2.2 Checked reads now include packet integrity in success semantics

`adbms6830_rd48_checked()` now returns `HAL_OK` only when:

- topology is valid;
- wake succeeds;
- the SPI transaction succeeds;
- every expected IC packet passes PEC10;
- every valid-PEC packet passes command-counter validation.

The function still parses every expected IC packet before returning so the per-IC PEC and counter masks retain full diagnostic coverage.

The new `adbms6830_read_result_t` records the root category:

- `ADBMS6830_READ_RESULT_OK`
- `ADBMS6830_READ_RESULT_TOPOLOGY_ERROR`
- `ADBMS6830_READ_RESULT_TRANSPORT_ERROR`
- `ADBMS6830_READ_RESULT_PEC_ERROR`
- `ADBMS6830_READ_RESULT_COUNTER_ERROR`
- `ADBMS6830_READ_RESULT_PEC_AND_COUNTER_ERROR`

A completed SPI transfer with corrupt packet data is no longer representable as a successful checked register read.

### 2.3 Removed dead last-IC-wins PEC state

The file-scope `rx_pec_error` variable was removed. It was reset by each passing IC, could therefore hide an earlier failing IC, and was never consumed. PEC state is now accumulated locally across the entire expected chain and feeds the checked-read result.

### 2.4 Explicit CS setup and hold timing

Both ADBMS6830 and ADBMS2950 local SPI transports now enforce:

```c
#define ADBMS_SPI_CS_SETUP_US 1u
#define ADBMS_SPI_CS_HOLD_US  1u
```

Transaction order is now:

```text
lock shared SPI
CS low
verified setup delay
HAL SPI transfer
verified hold delay
CS high
unlock shared SPI
```

Failure behavior:

- setup delay failure: CS is raised, no SPI clocks are emitted, and the operation fails;
- HAL failure: hold/deassert cleanup still runs and the HAL failure is returned;
- hold delay failure: CS is raised and the operation fails even if the wire transfer completed;
- every exit leaves the selected CS inactive.

The 1 microsecond values are conservative software margins. Their exact requirement must be checked against the controlled ADBMS6822 datasheet revision and confirmed on the assembled AMS with a logic analyzer.

## 3. Tests added or strengthened

Host unit coverage now proves:

- a failed cell read preserves prior temperature update masks;
- checked read succeeds only for all-valid packets;
- one PEC failure is not cleared by a later passing IC;
- a valid-PEC counter mismatch fails the checked read;
- simultaneous PEC and counter faults are both reported;
- setup-delay failure emits no SPI transaction and leaves CS high;
- hold-delay failure returns failure after the transfer and leaves CS high;
- ADBMS2950 setup timing failure also emits no SPI transaction and leaves CS high.

## 4. Intentionally unchanged

The following were not changed by this patch:

- SPI6 mode, bitrate, byte order, or pin assignment;
- ADBMS command encodings;
- five-SMB logical read count;
- physical six-device ring wake count;
- `WAKEUP_US_DELAY`;
- `WAKEUP_BW_DELAY`;
- SNAP/RDCVA-RDCVF/UNSNAP sequencing;
- 1 Hz bring-up versus 10 Hz normal profile selection;
- ADBMS2950 authority or pack-current source policy;
- BMS_OK or balancing release gates.

## 5. Future work retained in code and release gates

### 5.1 Measure the existing target counters

No new scan instrumentation is required. On the STM32F767, capture:

```text
adbms_last_scan_duration_ms
adbms_max_scan_duration_ms
adbms_scan_deadline_miss_count
```

Record these for:

- cold power-up;
- first successful full-ring scan;
- warm repeated scans;
- long idle followed by a scan;
- String A SMB acquisition;
- APM/String B traffic enabled;
- periodic diagnostics enabled;
- at least several minutes of continuous operation.

### 5.2 Do not qualify 10 Hz by shortening wake constants blindly

The current cell path reaches a checked wake before each register-group read. With six physical devices and the checked-in 1 ms low plus 1 ms between-wake delays, one wake costs approximately 12 ms. The repeated wake architecture is therefore the dominant timing cost.

`ADBMS6830_CORE_WAKE_MAX_US` and the transport wake pulse/gap constants do not automatically describe the same requirement. Do not reduce both wake constants merely because a nearby core-wake constant is 500 microseconds.

Any shorter timing experiment must separately test:

- cold ring wake;
- warm wake;
- long-idle recovery;
- first-command success;
- all five SMB responses;
- APM passage through the physical ring;
- PEC and counter error rates;
- scan duration and deadline misses.

### 5.3 Preferred 10 Hz architecture

If target measurements still miss the 100 ms period, add an internal helper such as:

```c
static HAL_StatusTypeDef adbms6830_rd48_checked_locked_no_wake(...);
```

Use it only inside one mutex-owned, demonstrably awake acquisition epoch:

```text
lock
cold/recovery wake once
conversion
SNAP
RDCVA no wake
RDCVB no wake
RDCVC no wake
RDCVD no wake
RDCVE no wake
RDCVF no wake if retained
UNSNAP
unlock
```

Keep the public standalone checked-read path self-contained with its own wake for CLI and diagnostics after arbitrary idle time.

This refactor must define:

- maximum allowed inter-command gap while assuming awake;
- recovery after one failed no-wake read;
- whether the remaining groups are attempted after failure;
- counter expectation handling after ambiguous transport failure;
- shared-ring interaction with ADBMS2950 transactions;
- bounded mutex hold time.

### 5.4 Hardware CS timing evidence

Capture at the MCU-side ADBMS6822 interface:

- selected CS;
- SPI6 SCK;
- MOSI;
- MISO.

Confirm:

- SCK idles high;
- data is sampled in SPI Mode 3;
- at least the configured setup delay exists from CS low to first sampling edge;
- at least the configured hold delay exists from final sampling edge to CS high;
- CS remains low across the complete command plus response clocking;
- the non-selected string CS remains high.

### 5.5 Target build and hardware limitations

Host validation does not prove:

- the ARM target compiles and links;
- the delay timer is actually clocked at 1 MHz;
- the GPIO/SPI waveform meets the physical timing contract;
- the five SMBs and APM respond in the expected physical order;
- 10 Hz operation meets deadline and mutex budgets;
- hardware PEC/counter error rates are acceptable.

These remain required hardware evidence.

## 6. Bring-up acceptance boundary

After this patch and host tests, the software is suitable for controlled **1 Hz low-voltage bring-up** with BMS_OK and balancing inhibited.

Normal 10 Hz operation is not qualified until:

- target scan timing is recorded;
- `adbms_scan_deadline_miss_count` remains zero in the required test window;
- cold/warm/idle wake behavior passes;
- full physical-ring PEC and counter behavior passes;
- any wake optimization is independently reviewed and measured.
