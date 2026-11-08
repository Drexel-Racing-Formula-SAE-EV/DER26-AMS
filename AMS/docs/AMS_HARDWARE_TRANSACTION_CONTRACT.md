# DER26 AMS bounded hardware-transaction contract

**Status:** initial audit, no production timeout retuning yet
**Origin:** Muad'Dib ADC/EMI hang lesson

## Rule

Every blocking hardware interaction must have:

1. a finite local deadline;
2. a failure meaning;
3. an explicit data-validity consequence;
4. bounded retry/recovery;
5. escalation ownership;
6. a diagnostic counter/timestamp where the subsystem is safety relevant.

A finite timeout is not automatically acceptable. It must ultimately satisfy:

```text
transaction timeout <= owning task/system deadline - cleanup margin
```

This first pass inventories the current source and deliberately does **not** shorten ADBMS timeouts without timing evidence.

## Current inventory

| Subsystem | Operation / source | Caller | Current bound | Owner cadence/deadline | Failure behavior today | Audit action |
|---|---|---|---:|---:|---|---|
| DHAB ADC high range | `stm32f767z_adc_read_checked()` / `current_sensor.c` | current task | 5 ms | 20 ms / 50 Hz | failed conversion returns invalid result; current task fault policy owns escalation | Keep; measure target normal/max conversion time |
| DHAB ADC low range | same | current task | 5 ms | 20 ms / 50 Hz | same | Keep; verify combined two-channel WCET margin |
| Generic STM32 ADC wrapper | `stm32f767z.c` | current/diagnostic callers | caller supplied, default 5 ms | caller dependent | start/poll/stop status returned | Keep explicit caller budgets |
| ADBMS6830 SPI command/read | `adbms6830.c` | ADBMS task / service CLI | `SPI_TIMEOUT = 500 ms` | normal ADBMS period 100 ms | HAL status propagated; higher layers invalidate/recover | **Measure before retuning.** 500 ms is bounded but wider than the owning periodic budget |
| ADBMS2950 SPI | `adbms2950.c` | ADBMS/APM path | `SPI_TIMEOUT = 500 ms` | ADBMS cadence / advisory APM schedule | HAL status propagated | Measure actual frame/session duration; derive operation-specific budget |
| ADBMS6830 µs wait | `adbms6830_us_delay()` | ADBMS timing | timer target + spin budget | operation dependent | timer non-progress becomes `HAL_TIMEOUT` | Already embodies Muad'Dib lesson; preserve in Zephyr adapter |
| ADBMS6830 conversion poll | checked poll path | ADBMS task | `timeout_us` + spin budget + finite SPI timeout | conversion-specific | timeout returned; result not published as good | Preserve two-dimensional time/spin bound |
| ADBMS2950 µs wait | `adbms2950_us_delay()` | APM path | timer target + spin budget | operation dependent | timer non-progress becomes `HAL_TIMEOUT` | Preserve |
| CLI UART TX | `cli.c` | low-priority CLI task | 20 ms per line/newline call | diagnostic only | error returned/counted by CLI path | Never allow CLI completion to own safety progress |
| CAN TX | callback-driven scheduler | CAN task/ISR | nonblocking mailbox model + scheduler deadlines | protected frames 100 ms | mailbox abort/recovery; authority revoked by policy | Preserve scheduler/event-time model, do not convert to blocking send |
| CAN bus-off recovery | `canbus.c` | ISR + CAN task | 300 ms ECU stale / 500 ms AMS discharge policy | state dependent | immediate or bounded fail-low | Preserve exact event-time policy |
| IWDG register update | `ams_safety.c` | startup | explicit 100000-spin bound | boot/startup | failed start keeps BMS_OK inhibited | Keep; target reset/start evidence still required |
| ADBMS SPI mutex | `app.c` | ADBMS/service users | finite mutex timeout | safety transport ownership | timeout panics/fail-low | Keep finite; measure contention/hold metrics already exposed |
| Current-window mutex | `app.c` | current/ADBMS | finite mutex timeout | 20/100 ms producers | timeout panics/fail-low | Keep finite and short |

## Immediate code-review gate

`host_tests/tools/check_bounded_io.py` fails authored source when it detects:

- `HAL_MAX_DELAY`;
- `osWaitForever`;
- `portMAX_DELAY`;
- an obvious peripheral-status `while` loop that lacks an explicit timeout/spin bound in its body.

The gate is intentionally narrow. It is not a proof of real-time correctness and does not ban computational loops.

## Next measurement work

Do not change `SPI_TIMEOUT` yet. Measure on target:

- one ADBMS command frame;
- one full C-voltage read;
- wake + command + conversion + read session;
- redundant C/S diagnostic session;
- AUX/temperature mux operation;
- ADBMS2950 sample;
- worst observed SPI-mutex wait and hold time.

Then split the broad timeout into operation-specific constants only where the evidence supports it.
