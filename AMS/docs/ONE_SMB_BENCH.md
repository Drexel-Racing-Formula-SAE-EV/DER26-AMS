# One-SMB onboard ADBMS6822 bench build

Firmware **0.5.41**, AMS bundle **2.6.38**. Clean-build **BENCH Validation 1-SMB Debug** and flash the generated `AMS/Bench1_Debug/DER26-AMS.elf`. This configuration selects **B/J2, PE4 CS_B**, one downstream SMB, **SPI Mode 3**, and periodic cell and automatic temperature mux acquisition. `AMS_SPI_LINK_DIAGNOSTIC_ONLY` is no longer defined by either 1-SMB CubeIDE configuration. The five-SMB BENCH Validation configurations also run automatic temperature scans. Both still compile-inhibit BMS_OK and balancing.

The previously accepted five-SMB temperature validation is retained. This package does not assert that the newly ordered SMB has passed the same hardware test; this run checks its temperature path. The separate vehicle temperature pull-up and threshold release acknowledgements have not been changed.

Connect AMS **J2 String B** to the single SMB **String A** input. Use the known low-voltage bench setup. At 115200 8N1:

```text
ver
spi onboard
status
spi lockdiag
spi probe
spi sid
spi status
spi stat
spi cdump
temp
tempbus idle
tempsns 0 0
rtos
```

Check `ver` for **0.5.41** and `spi onboard` for **Build SMBs:1 ... path:B/J2 PE4**. The startup banner must **not** say `SPI LINK ONLY LV`. `ver` reports `auto_temp_scan:1`, and `temp`/`tempbus idle` report `auto_scan:1`. Wait for at least a full mux cycle before judging aggregate `temp`; `tempsns 0 0` makes one explicit diagnostic attempt. A valid 6830 SID/PEC establishes the link, while temperature values require successful mux selection, AUX read, and plausibility checks. A CLI `ADBMS SPI busy` response means the periodic scan owned the bus; retry after the scan and inspect `spi lockdiag` if it persists.

If you need the former CLI-exclusive link test, add `AMS_SPI_LINK_DIAGNOSTIC_ONLY=1` to the **1-SMB** configuration explicitly and rebuild. That mode parks periodic acquisition, so `temp` will not update automatically. See [SPI-only instructions](SPI_LINK_ONLY_BENCH.md).

This archive is source only; the ARM ELF and target stack/RAM usage must be checked in CubeIDE.

The retained v0.5.38 fix skips the settling wait after a failed balance clear and blocks for at
least one tick after releasing SPI when a scan overruns. `spi lockdiag` reports
the actual schedule interval and scan durations. Occasional busy responses
during an active scan are bounded; persistent busy responses need these counters.

Since v0.5.39, checked MUTE and verified zero discharge registers can run before
startup POST passes. Repeated BMS_OK-low decisions stop queuing mute requests
only after that physical off proof succeeds. A failed proof still requests mute.
This does not set transport/safety readiness or bypass POST.

The submitted v0.5.38 log already proves onboard communication: valid RDCFGA
and SID DB6AE40786B2, no PEC/counter failures, and no CLI busy responses. It
does not pass startup POST. Until POST passes, aggregate `volt`/`temp` stay
unavailable; default/stale displayed temperatures are not measurements.

After flashing v0.5.41, run the commands above separately. `spi status` reports
retained state; `spi stat` attempts fresh status diagnostics. Capture `spi status`
again after `spi stat`, `spi cdump`, and `tempsns 0 0`. Check that urgent mute
failures and interrupted waits stop increasing, then inspect the new
`POST failed_step` line. Stage numbers: 1 baseline, 2 oscillator fast,
3 oscillator slow, 4 supply UV, 5 supply OV, 6 THSD, 7 NVM ED, 8 NVM MED,
9 TMOD, 10 SPIFLT, 11 configuration restore, 12 final baseline; 0 means none.
`restore_HAL` and `result_HAL`: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT.
The failure detail describes the last bounded POST attempt.

## Retained v0.5.40 POST conversion correction

The v0.5.39 debugger run returned HAL_ERROR at POST_SUPPLY_UV with
restoration HAL_OK, then reached cli_task_start. The timer wait completed.
Each FLAG_D stimulus now clears preceding diagnostic latches and starts a
bounded redundant single-shot ADC conversion before reading diagnostic flags.
Previously POST read status immediately after writing FLAG_D, with no new ADC
operation. The supply flag descriptions specify detection during ADC operation.
This is a sequencing fix, not evidence of a physically undervoltage supply.
The host ASIC model reproduces the old failure and passes the correction;
the target must still confirm POST.

Before resuming at app.c:770 after a fresh flash, run:

```gdb
p app.acc.smb_init_status
p app.acc.smb.post
tbreak cli_task_fn
continue
```

After cli_task_fn is reached, continue again to allow UART processing.
Use the UART commands above. Expect POST pass:1 only if every stage and the
final baseline/configuration readback pass. Do not force readiness in GDB.

## v0.5.41 stimulus removal before SPIFLT

The target v0.5.40 run passes the FLAG_D stages and fails SPIFLT with
failed_ic_mask=0, unexpected_ic_mask=1. Its retained image reports spiflt=1
and tmodchk=1. The normal configuration is now written to hardware before
SPIFLT, removing FLAG_D[7], followed by a fresh clean conversion baseline.
The SPIFLT test still rejects any unexpected flag. Final mandatory restoration
and baseline remain. Confirm manifest v0.5.41 and repeat the GDB/CLI checks.
