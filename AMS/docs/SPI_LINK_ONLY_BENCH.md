# One-SMB onboard ADBMS6822 SPI link diagnostic

Firmware **0.5.41**, AMS bundle **2.6.38**, 2026-10-05. This optional mode requires explicitly adding `AMS_SPI_LINK_DIAGNOSTIC_ONLY=1` to **BENCH Validation 1-SMB Debug** before a clean build. The stock 1-SMB configuration runs periodic cell and temperature acquisition; see [ONE_SMB_BENCH.md](ONE_SMB_BENCH.md). When selecting SPI-only mode, flash its newly generated ELF from `AMS/Bench1_Debug/`, then check `ver` for 0.5.41, `spi onboard` for `Build SMBs:1 ... path:B/J2 PE4`, and the UART startup banner `SPI LINK ONLY LV`. The version alone cannot distinguish these two modes.

This image performs the existing one-time accumulator startup, then parks the periodic ADBMS task. The CLI has sole post-startup SPI bus access. The ADBMS measurement heartbeat intentionally goes stale, BMS_OK stays low, and balancing remains inhibited. The diagnostic-only conversion wait no longer aborts on a queued urgent-mute flag, because the parked task cannot consume that flag. The pending flag remains visible in `spi status`; no physical mute service is claimed. This image is only for low-voltage isolated bench communication testing, not operation of the pack or vehicle. An SPI transfer that stays stuck in HAL can still time out; this switch only removes periodic scan contention.

Connect AMS **J2 (String B)** to one SMB **String A** input, pin 1 to pin 1 and pin 2 to pin 2; power the SMB independently. Remove the old J15/EVAL-6822 jumper arrangement. Do not connect the high-voltage load or enable contactors. With power off, check connector orientation and no short on the isoSPI pair. If a MOSFET heats rapidly, remove power and resolve that electrical fault before this test.

On the UART at 115200 8N1, issue each command separately:

```text
ver
spi onboard
spi pins
spi clear
spi enable
spi coldwake
spi probe
spi sid
spi status
```

`spi probe` uses the selected **B/J2** SMB path. `spi probeb` addresses an APM path and is not this test. The ADBMS6822 has no readable ID; success means a valid downstream ADBMS6830 packet with correct PEC, then a valid SID, without transport errors. `spi onboard` reports the pin map and SPI Mode 3 setup, not actual chip detection. If `spi probe` hangs, inspect the HAL/SPI call stack; with the periodic scan parked, a mutex owner timeout now points to another caller or a stuck CLI transaction. Capture the entire UART output and the debugger stack.

Service commands now wait up to 50 RTOS ticks for the shared SPI lock and print `ADBMS SPI busy; retry after scan` on contention. `spi onboard` and `spi lockdiag` remain accessible without the lock. Long SPI reports are formatted under the lock and transmitted over UART after unlock; `spi lockdiag` shows the CLI busy count. If a command still hangs after it acquires the lock, capture the debugger call stack and `rtos` task high-water marks. The Error task is allocated 512 stack words and its flight-recorder snapshot is static storage.

The new U1 uses SPI6 PG13 SCK, PG14 MOSI, PG12 MISO, PE4 CS_B, Mode 3 (CPOL high, CPHA second edge), approximately 422 kHz. `spi scope b read 20` is available for a scope check after the basic probe, but its HAL completion alone does not establish successful communication. See `ONBOARD_6822_BRINGUP.md` for the electrical pin and measurement details.

This package is source. No STM32 target ELF/HEX has been generated in this environment.
