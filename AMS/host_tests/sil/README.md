# Acquisition SIL

Run from any directory:

    python3 host_tests/sil/run_acquisition_sil.py
    python3 host_tests/sil/run_acquisition_sil.py --baseline /path/to/extracted/original-project

Requires Python 3, make and a native C compiler. Uses the existing host-test Makefile's compiler/include settings. Results go to validation/ACQUISITION_SIL_*.csv and *.stderr.txt. Assertions cause a nonzero exit.

This compiles the REAL adbms6830.c and accumulator.c, with virtual HAL SPI/GPIO/time. Seven deterministic scenarios, 200 voltage+temperature scans each, cross the microsecond counter wrap. Covers ideal/rounded waits, repeated 2.5 ms and intermittent 8 ms coherent-read pauses, intermittent PEC corruption and HAL timeout, persistent PEC failure, raw required-mask completeness on accepted scans, CS cleanup and lock balance.

Model limits: packets are synthetic valid codes/counters; no analogue plant, transistor mux response, isoSPI electrical/idle-state model, actual FreeRTOS scheduling, ISR runtime, CLI/CAN concurrent tasks, diagnostic scheduler execution or CPU performance. Pauses are injected after simulated wire time, modelling descheduling before software completion. Wait rounding rounds requested durations, not a complete RTOS tick-phase model. Deadline counts only cover voltage+temperature operations, excluding production diagnostics and processing. Do NOT interpret the CSV as target WCET or proof of 100 ms scan feasibility. This is an acquisition fault/timing SIL, not a complete virtual ECU/AMS.

Fault placement is by coherent-read number. Baseline and candidate issue different numbers of reads; their injected faults therefore do not occur at equal wall times or necessarily the same register group. Compare nominal transfer workload directly; fault recovery counts are scenario evidence, not normalized reliability statistics.

The candidate-only regression now requires the 2.5 ms interruption case to succeed without a wake. The 200-scan stress also requires all scans accepted with zero coherent restarts. The production wake test exhausts all 1000 microsecond phases relative to the millisecond tick at 2999/3000 us, tests stale proof after a full microsecond wrap, both clocks' wrap, and logical session reopen. These regressions gate vehicle-ci.
