# DER26 AMS Renode Bring-Up Harness

This directory is an experimental Renode path for pre-hardware firmware
bring-up. The first goal is modest: boot the STM32 image far enough to see the
UART CLI banner and exercise safe bring-up commands in a deterministic virtual
machine.

Renode is useful here because it can run Cortex-M firmware with modeled
peripherals and scripted machines. It is not cycle-accurate hardware validation,
and it does not prove ADBMS6822/isoSPI electrical behavior, ADBMS6830 response,
charger CAN transceiver behavior, ADC noise, GPIO pin routing, or real timing.

## Expected Use

Run this from the repository root after installing Renode and the ARM GCC
toolchain:

```bash
AMS_RENODE=1 ./ci/stm32/build_ams_headless_gcc.sh
renode --console -e "include @AMS/renode/scripts/ams_f767_cli_smoke.resc"
```

On Windows/MSYS2, Renode may not understand `/c/...` paths. Use a Windows-style
absolute path if the relative include fails:

```bash
AMS_RENODE=1 ./ci/stm32/build_ams_headless_gcc.sh
renode --console -e 'include @C:/DER_AMS/git/DER26-AMS/AMS/renode/scripts/ams_f767_cli_smoke.resc'
```

You can also run `scripts/run_renode_smoke.bat` from Windows after building the
ELF with `AMS_RENODE=1`.

The `AMS_RENODE=1` profile does not change normal hardware builds. By default
it enables firmware-side fake backends for:

- `AMS_RENODE_FAKE_ADBMS=1`: five-IC ADBMS6830 chain with PEC/counter-aware
  readback and fault injection.
- `AMS_RENODE_FAKE_CURRENT=1`: DHAB current sensor model that feeds realistic
  ADC counts into the real current conversion path.
- `AMS_RENODE_FAKE_CHARGER=1`: TC/Elcon-style charger CAN command/readback
  model with control-byte polarity, RX timeout, fault flags, and TX failure
  injection.
- `AMS_RENODE_FAKE_TEMP=1`: SMB thermistor model that writes realistic raw AUX
  codes into the normal accumulator temperature path.
- `AMS_RENODE_CAN_CAPTURE=1`: firmware-side CAN TX capture counters for ECU,
  logger, estimator, and charger frames.

Set any fake macro to `0` if you want to remove that modeled backend from a
Renode build.

If your Renode build does not accept `-e`, start `renode` and run this in the
Monitor:

```text
include @AMS/renode/scripts/ams_f767_cli_smoke.resc
```

The script loads:

```text
AMS/build/DER26-AMS.elf
```

## First Success Criteria

A good first Renode result is:

```text
~~~~~~~~~~ DER AMS FW ...
Build:hw-bringup or Build:normal
Build:renode when built with AMS_RENODE=1
ADBMS6822 SPI6 expected: mode3 CPOL HIGH CPHA 2EDGE
Type 'help' for list of commands
```

Once the banner appears, the next useful manual CLI commands are:

```text
help
status
bringup board
bringup ready
fault
spi status
apm status
adbmsfake status
spi probe
spi sid
spi stat
spi cfgchk
bringup adbms6830
volt
currentfake status
current
chargerfake status
charger
tempfake status
temp
canlog
scenario help
```

With the default fake ADBMS backend, `spi probe`, `spi sid`, `spi stat`,
`spi cfgchk`, `volt`, and `bringup adbms6830` use modeled ADBMS6830 readback
frames with valid PEC/counter bytes. If `AMS_RENODE_FAKE_ADBMS=0`, physical
ADBMS commands are refused because there is no modeled ADBMS6822/ADBMS6830/
ADBMS2950 chain behind SPI6.

The fake ADBMS backend also supports deterministic fault injection through the
UART CLI:

```text
adbmsfake status
adbmsfake reset
adbmsfake all 3700
adbmsfake ov 0 3
adbmsfake uv 2 7
adbmsfake pec 0x0004
adbmsfake missing 0x0002
adbmsfake counter 0x0010
adbmsfake reset
```

Use these controls to mutate the modeled five-IC ADBMS6830 chain, then observe
the normal firmware paths with `spi ...`, `volt`, `temp`, `fault`, and
`bringup adbms6830`. The fake model exercises AMS parser, PEC, command-counter,
diagnostic, voltage, and temperature software paths. It still does not prove
physical SPI timing, ADBMS6822 isoSPI signaling, transformer polarity, cabling,
or real analog behavior.

The fake current backend exercises the same DHAB math as hardware:

```text
currentfake status
currentfake reset
current
currentfake amps 70
current
currentfake charge 10
current
currentfake mismatch 20 0
current
currentfake rail low
current
currentfake fail on
current
currentfake reset
```

Use positive current for discharge and negative current for charge/regen. The
`charge` helper takes a positive number and applies it as negative current. Raw
ADC overrides use the real AMS mapping: low ADC is `C_SENSE_L` / DHAB 50 A,
high ADC is `C_SENSE_H` / DHAB 800 A.

The fake charger backend exercises the real charger command path:

```text
chargerfake status
chargerfake reset
state charge
charger
chargerfake rxgood
charger
chargerfake flags 0x02
charger
chargerfake txfail on
charger
chargerfake txfail off
chargerfake timeout on
charger
state start
chargerfake reset
```

The command payload follows the published charger convention used by the AMS
firmware: bytes 1-2 are voltage in 0.1 V units, bytes 3-4 are current in 0.1 A
units, and datasheet BYTE5 is `data[4]` in C. `data[4] = 0` allows charging;
`data[4] = 1` disables charger output for battery protection. In Renode, the
fake can auto-reply with charger readback, go silent to exercise the 5 s RX
timeout, report hardware flags, or force CAN TX failure.

The fake temperature backend writes modeled thermistor raw values into the same
accumulator temperature path used after ADBMS AUX reads:

```text
tempfake status
tempfake reset
temp
tempfake all 25
tempfake hot 0 0
temp
fault
tempfake cold 0 1
temp
tempfake missing 2 0x000003
temp
tempfake invalid 3 0x000010
temp
fault
tempfake reset
fault reset-temp
```

`tempfake missing` models sensors that did not update in this scan. By default
the missing mask is one-shot; use `tempfake holdmissing on` when you want the
missing channels to persist long enough to exercise stale-timeout behavior.

The CAN capture helper summarizes what the firmware tried to transmit without
needing a modeled CAN bus peer:

```text
canlog clear
state discharge
canlog
state charge
chargerfake rxgood
charger
canlog
```

`canlog` is intentionally a counter/last-frame view. It does not prove physical
CAN bit timing, transceiver wiring, or another ECU's parser. It does catch
packetization, frame-ID bucket, and charger-command-order regressions in the
Renode firmware image.

For repeatable smoke cases, the `scenario` command sets the fakes into useful
states:

```text
scenario healthy
status
fault
volt
temp
current
canlog

scenario ov
volt
fault

scenario hot
temp
fault

scenario charge-ready
charger
canlog

scenario charger-timeout
charger
fault

scenario current-trip
current
fault
```

`scenario current-trip` intentionally uses `STATE_START` so a 70 A fake current
hits the precharge overcurrent policy. Use `fault reset-current` and
`currentfake reset` before moving to a different current test. `fault reset-all`
clears voltage, temperature, current, and charger software latches and forces
BMS_OK low so the next task pass has to earn it again.

## Platform Files

| File | Purpose |
|---|---|
| `platforms/ams_f767zi_stm32f746_compat.repl` | Preferred first attempt. Reuses Renode's built-in STM32F746 platform because the AMS image uses STM32F7-family core/peripheral blocks at compatible addresses. |
| `platforms/ams_f767zi_minimal.repl` | Local fallback/skeleton for debugging platform-description issues. It maps the F767 flash/RAM and the main AMS peripherals but is intentionally incomplete. |
| `scripts/ams_f767_cli_smoke.resc` | Loads the compatibility platform, loads the headless ELF, opens the USART3 analyzer, and starts emulation. |
| `scripts/ams_f767_cli_scenario_console.resc` | Same boot path as the smoke script, but named for the fake-input scenario workflow. |
| `scripts/cli_scenario_commands.txt` | Pasteable UART CLI command sequence for ADBMS, temperature, current, charger, CAN capture, and latch-reset checks. |

## Staged Plan

1. **Boot smoke:** ELF loads, reset handler reaches `main`, FreeRTOS starts,
   UART CLI banner prints.
2. **CLI smoke:** `help`, `status`, `bringup board`, and `bringup ready` work
   without hard faults or stuck UART output.
3. **Fake-input firmware profile:** use the ADBMS, temperature, current,
   charger, scenario, and CAN-capture CLI controls so Renode can drive full
   safety states without physical peripherals.
4. **Renode peripheral models:** if the firmware-side fake profile stops being
   enough, add external Python or C# peripherals for SPI/CAN peers. The useful
   first external ADBMS model should cover command parsing, daisy-chain byte
   ordering, PEC generation/checking, readback registers, fault injection, and
   stuck-bus/error cases.
5. **CI:** add a Renode test job that boots the ELF and waits for expected UART
   lines.

## Likely Failure Points

| Symptom | Meaning | Next Step |
|---|---|---|
| Hangs before UART banner | Clock/RCC, SysTick, HAL init, or missing peripheral model | Try the minimal platform, bypass PLL in a Renode build flag, or inspect with GDB. |
| UART banner appears, then tasks stall | FreeRTOS tick/SysTick or modeled timer issue | Check `nvic` systick frequency and task delays. |
| CAN/SPI calls fail or time out | Fake backend disabled, command sent before model is ready, or a fake failure was injected | Check `adbmsfake status`, `currentfake status`, `chargerfake status`, and reset the fake backend. |
| `bringup board` reports current warning | Current fake disabled, ADC fail injected, raw rails injected, or current task has not run yet | Run `currentfake reset`, wait one task tick, then check `current`. |
| Temperature stays invalid | Temp fake disabled, missing/invalid mask still injected, or ADBMS task has not refreshed stats yet | Run `tempfake reset`, `fault reset-temp`, wait one task tick, then check `temp`. |
| Charger fault stays latched | Charger fake TX failure/timeout was injected or charger fault state is still latched | Run `chargerfake reset`, `fault reset-charger`, then `state charge` and `charger`. |

## What This Does Not Replace

The first real AMS-board proof is still:

```text
spi clear
spi enable
spi coldwake
spi probea
spi probeb
spi sid
spi stat
bringup adbms6830
```

Renode can make the firmware less mysterious before the board arrives. It cannot
prove real SPI mode, chip-select routing, transformer polarity, isoSPI cable
integrity, ADBMS PEC behavior, or charger output behavior.
