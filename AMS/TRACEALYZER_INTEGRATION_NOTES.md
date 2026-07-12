# Tracealyzer SWO integration notes

This project uses TraceRecorder 4.12.0 with the ARM_ITM stream port.

Key fixes in this package:

- CMSIS-RTOS2 owns `SysTick_Handler`; the FreeRTOS port exports `xPortSysTickHandler`.
- TraceRecorder hooks are enabled only when `AMS_ENABLE_TRACEALYZER=1`.
- PB3 is reserved as asynchronous SWO in `DER26-AMS.ioc`.
- Initial ST-LINK V2 profile disables OS-tick events and uses the internal event buffer.
- ITM host-command input is disabled because this setup starts with `xTraceEnable(TRC_START)`.

After importing, run **Project > Clean**, then rebuild Debug. Old `port.o` and
`cmsis_os2.o` files contain the previous SysTick symbol names and must not be reused.
