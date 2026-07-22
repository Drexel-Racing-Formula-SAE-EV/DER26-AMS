# Limitations


Authored by Mahad Faisal, 2026.
This harness is intentionally host-side. It is a fast regression tool, not a safety certification tool.

It cannot validate:

- real ADBMS6830 SPI timing
- real PEC behavior
- real board-level cell/AUX wiring
- actual thermistor values
- ADBMS2950 scaling
- charger byte polarity unless verified against the real charger protocol
- IMD capture timing on STM32 timers
- CAN timing on the physical bus
- CubeIDE linker/startup behavior
- RTOS priority/race behavior on STM32
- grounding, noise, EMI, connector faults, or harness faults

Required next steps after host pass:

1. Import/build in STM32CubeIDE.
2. Flash to an STM32F767 target.
3. Run low-voltage bench tests with BMS_OK observed on a meter/scope.
4. Verify ADBMS chain wakeup, read, PEC, and channel mapping.
5. Verify charger CAN with a CAN sniffer before connecting high voltage.
6. Verify ECU receives all AMS packets and handles the 75s layout correctly.
7. Validate faults one at a time with safe injected hardware conditions.

## Sensor Filtering / Plausibility Notes

The AMS firmware now records thermistor open/short classification, implausible
temperature jumps, temperature rate-of-rise masks, cell-voltage jump masks, and
stuck-cell masks. These are diagnostics unless the sensor becomes invalid/stale
or an existing threshold/latch condition is met. The established safety path is
not allowed to use slow filtered values as the only trip source.

Fan control is open-loop. The current hardware exposes PWM fan-zone drivers but
no confirmed fan tach/RPM feedback, so firmware can report commanded duty and
PWM driver call failures only. It cannot prove a fan is physically spinning.

Rail monitoring, AMS board temperature, DHAB-local thermal compensation, and
AIR auxiliary feedback are not implemented in firmware because no confirmed
ADC/tach/aux inputs are available in the current AMS schematic set. Do not fake
these values in software; add routed hardware signals first.
