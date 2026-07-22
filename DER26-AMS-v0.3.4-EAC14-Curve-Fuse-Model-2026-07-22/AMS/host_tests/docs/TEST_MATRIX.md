# Test Matrix


Authored by Mahad Faisal, 2026.
| Test function | Purpose |
|---|---|
| `test_accumulator_stats_and_balance` | Verifies voltage min/max/total stats, full-usable scan gating, bounded SMB count, PWM balance selection, DCC stays clear, and balance clear. |
| `test_voltage_stats_boundaries_and_fuzz` | Fuzzes valid/invalid cell-code combinations and verifies only usable/fresh readings affect pack stats. |
| `test_voltage_fault_policy_and_strict_scan_freshness` | Verifies voltage fault staging, charge-stop behavior, hard OV/UV latching, and strict full-fresh-scan behavior where any PEC/missed cell update drops BMS_OK immediately. |
| `test_system_sil_boot_ready_and_bms_conjunction` | System-level SIL: verifies BMS_OK cannot assert before both current and voltage are valid, and that either current-invalid or voltage-not-ready keeps BMS_OK low. |
| `test_system_sil_single_pec_miss_drops_bms_then_recovers` | System-level SIL: verifies one PEC/missed cell update immediately marks the voltage scan invalid, drops BMS_OK, and recovers only after a full fresh scan returns. |
| `test_system_sil_persistent_voltage_stale_drops_bms_ok` | System-level SIL: verifies repeated missed/PEC-failed voltage scans stay fail-closed, track stale data, and recover when a full fresh scan returns. |
| `test_system_sil_charge_stop_allows_balance_before_hard_ov` | System-level SIL: verifies 4.18 V charge-stop keeps BMS_OK and still allows controlled balancing, while 4.20 V hard OV latches, clears balancing, and drops BMS_OK. |
| `test_system_sil_state_transition_guard_and_audit` | Verifies a same-state service cleanup cannot race the supervisor into reasserting BMS_OK, applied transitions publish previous/current state, reason/tick/count, charge exit creates a blocking charger-shutdown request, corrupt enums normalize to ERROR, and counters saturate. |
| `test_system_sil_voltage_uv_ov_severe_diagnostics_and_latch` | System-level SIL: verifies soft UV warning, hard UV latch, severe OV diagnostic reason, and severe UV diagnostic reason through the ADBMS task/BMS_OK path. |
| `test_system_sil_current_warning_fast_trip_and_latch_persistence` | System-level SIL: verifies current warning-only behavior, fast discharge trip, BMS_OK drop, and latch persistence after current returns normal. |
| `test_system_sil_current_stale_adc_pair_fails_safe` | System-level SIL: verifies one stale ADC channel makes current invalid, ADBMS gating drops BMS_OK, and repeated ADC failures become a confirmed current sensor fault. |
| `test_system_sil_regen_and_charge_current_placeholders` | System-level SIL: verifies low regen is warning-only while regen/charge overcurrent levels latch and drop BMS_OK. This caught and fixed the regen-warning masking bug. |
| `test_system_sil_2950_advisory_sampling_and_cli` | System-level SIL: verifies ADBMS2950 sample success/failure, stale-value handling, non-gating behavior, scan exclusion, and the `status`, `sid`, `config`, `sample`, and bounded `scope` CLI paths. |
| `test_system_sil_2950_final_ring_init_ownership` | System-level SIL: verifies String A owns the five SMBs and only global reset, String B owns the one APM, divider controls default off, and APM init failure remains observable without erasing SMB readiness. |
| `test_system_sil_final_ring_topology_corruption_fails_closed` | Corrupts SMB/APM active and write strings and verifies physical scans, balancing and APM sampling stop without reusing a full-ring token. |
| `test_system_sil_combined_fault_precedence_and_reset_path` | System-level SIL: verifies current and voltage latches can coexist, reset independently, and BMS_OK only reasserts after both are cleared and healthy. |
| `test_system_sil_harsh_timeline_no_false_enable` | System-level SIL: runs a harsh multi-phase timeline through precharge, voltage read loss, PEC miss fail-closed behavior, precharge current fault, recovery, and persistent voltage loss without false BMS_OK enable. |
| `test_system_sil_current_invalid_immediate_bms_drop_and_recovery` | System-level SIL: verifies a current ADC/status failure immediately drops BMS_OK, holds it low until current recovers, and only reasserts after the next healthy voltage/current conjunction. |
| `test_system_sil_hard_fault_and_corrupt_smb_config_fail_closed` | System-level SIL: verifies hard_fault cannot be overwritten by the ADBMS task, corrupted/low SMB count cannot reduce the required 75-cell scan, and null SMB driver pointers fall back safely to internal storage instead of crashing. |
| `test_system_sil_voltage_threshold_exact_edges` | System-level SIL: verifies exact edge behavior at 4.179/4.180/4.199/4.200 V and 2.501/2.500 V through task-level BMS_OK gating. |
| `test_system_sil_charger_disable_from_dynamic_gates` | System-level SIL: verifies charge-stop, current-invalid, BMS_NOT_OK-only, and warning-only charge-current cases drive the charger disable byte correctly, and that soft voltage charge-stop disables the charger without taking ownership of BMS_OK. |
| `test_system_sil_deterministic_fault_injection_invariants` | System-level SIL: runs a deterministic 192-case cross-product of state, current, voltage, stale/PEC masks, hard/fuse/charger faults, and ADC failures, asserting that any BMS_OK true state satisfies all safety invariants. |
| `test_system_sil_task_order_permutations_fail_closed` | System-level SIL: permutes current, ADBMS, charger/CAN, and error-task ordering to ensure once a fault is observed no later task can incorrectly reassert BMS_OK. |
| `test_system_sil_recovery_and_latch_reset_paths` | System-level SIL: verifies warning-only current/voltage states recover automatically, while latched current and voltage hard faults require explicit latch reset before BMS_OK can return. |
| `test_system_sil_current_boundary_timing_edges` | System-level SIL/policy boundary test: verifies 499 ms does not trip a 500 ms debounce, 500 ms does; 99 ms does not trip a 100 ms fast fault, 100 ms does, including current-task integrated timing. |
| `test_system_sil_cli_can_diagnostic_consistency` | System-level SIL: verifies stale voltage and current ADC faults appear in CLI fault/voltage/current diagnostics and that unusable stale cells zero-fill in CAN voltage telemetry instead of reporting misleading data. |
| `test_current_service_calibration_boundary` | Verifies service calibration clear is refused while BMS_OK may be active and that an authorized calibration mutation invalidates the active current epoch and scalar readiness until a fresh current-task sample is published. |
| `test_software_heartbeat_monitor_faults_and_recovery` | System-level SIL: verifies startup heartbeat grace, critical task heartbeat stale fail-closed behavior, heartbeat recovery, and non-safety logger/dashboard stale reporting. |
| `test_system_sil_bringup_status_and_bmsok_inhibit` | System-level SIL: verifies BMS_OK output inhibit blocks attempted assertions, counts blocked assertions, CLI release/inhibit commands work, and status reports Mode 3 SPI settings. |
| `test_bringup_cli_board_ready_and_adbms_summaries` | Verifies `bringup board`, `bringup ready`, and `bringup adbms6830` report board-only readiness, avoid BMS_OK mutation, classify stuck ADBMS readback, and show PEC/SID/status pass states. |
| `test_bringup_cli_apm_and_charger_phase_split` | Verifies `bringup apm2950`, `bringup charger-lv`, and `bringup charger-battery` identify the APM as final-ring/advisory, document `BYTE5/data[4]` charger polarity, expose the allow frame, and split LV charger checks from battery-connected checks. |
| `test_system_sil_contradictory_dhab_vs_2950_observable_non_gating` | System-level SIL: injects contradictory DHAB and ADBMS2950/APM current values to verify the final-ring APM path remains observable but non-safety-gating until intentionally validated and promoted. |
| `test_system_sil_startup_garbage_never_enables_bms` | System-level SIL: fuzzes uninitialized-looking current ADC counts and cell codes before the first full voltage scan and verifies BMS_OK never asserts. |
| `test_system_sil_long_run_seeded_fuzz_invariants` | System-level SIL: runs 10,000 deterministic cycles with randomized state, current, ADC failures, PEC/stale masks, OV/UV injections, charge/precharge/regen cases, and task-order permutations while checking safety invariants after every cycle. |
| `test_system_sil_concurrent_heartbeat_starvation_and_recovery` | System-level SIL: runs current repeatedly while ADBMS/temp/CAN/logger heartbeats starve, verifies BMS_OK drops, then verifies recovery requires heartbeat clear, error-task hard-fault aggregation clear, and a later ADBMS pass. |
| `test_system_sil_concurrent_charger_tx_recovery_ordering` | System-level SIL: injects charger CAN TX failure, verifies BMS_OK stays low through conservative recovery ordering, and confirms the charger only re-enables after TX, charger-fault, BMS_OK, and fresh charger-RX conditions recover in sequence. |
| `test_system_sil_concurrent_seeded_scheduler_abuse` | System-level SIL: runs deterministic concurrent scheduler abuse across current, ADBMS/temp, CAN/charger/logger, fan, error, heartbeat starvation, CAN mailbox backpressure, CAN TX failures, OV/UV, stale/PEC masks, and temp faults while checking invariants after every step. |
| `test_temp_stats` | Verifies temperature max/average behavior while skipping invalid raw channels. |
| `test_temp_invalid_and_cold_valid_fault_behavior` | Ensures all-invalid temps fault and valid cold temps do not fault. |
| `test_can_telemetry_packets` | Verifies ECU AMS packet headers, CAN ID, 8-byte layout, status/current/voltage/temp/fan payloads. |
| `test_can_telemetry_pacing_and_snapshot` | Verifies slow detail frames are phased instead of burst, compact fields remain frozen when live application state mutates mid-bundle, and all electrical data come from one measurement snapshot. |
| `test_can_priority_metrics_and_deadlines` | Injects critical-shutdown and compact-traffic failures, verifies failure classes cannot overwrite each other, confirms compact failure suppresses best-effort detail, and checks task-duration/deadline counters. |
| `test_telemetry_absent_segments_and_invalid_channels` | Verifies missing SMB segments and invalid channels zero-fill instead of reading out of bounds. |
| `test_charger_rx_and_tx` | Verifies charger receive parse, charger fault bits, charger command transmit. |
| `test_can_rx_filter_matrix` | Verifies exact bxCAN charger/HIL filter packing and configuration failure, then verifies bad RX status, remote/wrong ID type, wrong ID, and short DLC are ignored before application parsing. |
| `test_charge_state_disable_matrix` | Verifies charge command disable byte for fault, BMS disabled, charger fault, and timeout cases. |
| `test_charger_state_exit_shutdown_burst` | Verifies leaving charge prioritizes three zero-demand disable frames ahead of telemetry, never enables in the same iteration, blocks BMS_OK until the burst completes, and retries a failed queue operation without consuming a frame or erasing the fault. |
| `test_charger_command_priority_tx_failure_and_cli` | Verifies the charger command is transmitted before telemetry in charge mode, TX failure drops BMS_OK and is exposed in charger CLI diagnostics. |
| `test_current_sensor_measurement_model` | Verifies DHAB 0.6-divider reconstruction, zero-current offset, design-file C_SENSE_L=50A / C_SENSE_H=800A mapping, range selection, saturation, mismatch, and implausible ADC handling. |
| `test_current_task_measurement_state` | Verifies current task propagation of `current_valid`, selected range, measurement reason, stale-current hold on ADC read failure, and confirmed sensor-fault timing. |
| `test_current_task_threshold_faults` | Verifies discharge overcurrent debounce/latch behavior and low-current precharge fast-fault behavior using the corrected DHAB mapping. |
| `test_adbms_voltage_scan_timing_contract` | Verifies the conversion waits, wrap-safe absolute scheduling, no unconditional recovery delay, and measured verified balance on/off intervals at or above their configured minima. |
| `test_measurement_epoch_contract` | Verifies current integration/invalid-gap accounting, tick-wrap handling, immutable voltage/current/temperature publication, sequence rollover, reader pinning, drop-on-busy behavior and recovery without a long critical-section copy. |
| `test_estimator_epoch_sequence_and_timing` | Verifies one update per measurement sequence, missed/repeated epoch accounting, actual voltage-to-voltage `dt`, tick wrap and invalid timing rejection. |
| `test_estimator_model_domain_flags` | Verifies low/high temperature-model clamp flags, core-temperature clamp flags and explicit model-domain validity reporting. |
| `test_estimator_task_hil_and_hardware_paths` | Verifies synthetic HIL can exercise R0 observation while hardware R0 remains frozen without both persistent calibration confidence and the build-time physical-polarity gate. |
| `test_fan_current_and_null_guards` | Verifies fan driver edge cases and defensive null handling. |
| `test_periods_and_driver_edge_cases` | Verifies task timing period increments, NaN/Inf fan input handling, CAN send guard behavior. |
| `test_task_iterations_with_injected_signals` | Runs one fake iteration of CAN, ADBMS, error, fan, and current tasks. |
| `test_fault_matrix_extra` | Verifies hard/soft fault aggregation, confirmed current sensor soft fault behavior, and BMS_OK safety behavior. |
| `test_safety_panic_reset_watchdog_and_log` | Verifies panic handling forces BMS_OK low without HAL/RTOS dependency, preserves reset/panic cause state, and records fault-log entries. |
| `test_retained_fault_log_integrity_recovery` | Injects uncommitted records, CRC corruption, corrupt ring metadata, and full-ring rollover; verifies only committed valid entries survive and sequence/order metadata is reconstructed. |
| `test_watchdog_feed_gate` | Verifies watchdog feeding is blocked during startup/stale/fault states and only allowed after voltage, current, temp, and heartbeat health are all valid. |
| `test_can_busoff_sets_fault_and_recovers` | Verifies CAN bus-off sets AMS CAN fault state, schedules recovery, records bus-off/recovery counters, and clears recovery-pending state after successful CAN restart. |

Unit-only additions:

| Unit test | Purpose |
|---|---|
| `test_current_sensor_conversion_zero_and_range_selection` | Verifies explicit DHAB conversion at zero current, 50A-range selection, and 800A fallback when the 50A channel saturates. |
| `test_current_sensor_invalid_conditions` | Verifies sensor saturation, 50A/800A channel mismatch, implausible ADC rails, stale/missing channel data, and reason/range strings. |
| `test_current_sensor_requires_fresh_pair_and_channel_mapping` | Regression test for the stale-channel bug: conversion requires both ADC channels fresh and preserves C_SENSE_L=50A / C_SENSE_H=800A mapping. |
| `test_current_sensor_read_adc_status_path` | Verifies checked ADC status propagation for normal reads, timeout/error reads, and null ADC handles. |
| `test_current_sensor_calibration_record_integrity` | Verifies fixed-width record creation, CRC/schema/range rejection, explicit live-zero proof, fail-closed restore, record-reference-based live/stored offset agreement (including wrong-board rejection), uncertainty-based SoH confidence, provenance clearing and reference-change invalidation. |
| `test_r0_observability_and_accounting` | Verifies R0 update gates/reject accounting, convergence/confidence flags, unit scaling, and removal of advisory-valid status once the last accepted observation becomes stale while retaining historical convergence. |
| `test_current_fault_policy` | Verifies discharge debounce, precharge fast trip, persistent sensor-fault confirmation, and placeholder regen-unexpected warning policy. |
| `test_air_feedback_scaffold` | Verifies the absent-hardware profile never reports auxiliary feedback healthy and that an enabled-but-uninitialized future monitor starts fail-closed. |
| `test_air_monitor_nominal_sequence_and_weld_clear` | Exercises boot-open proof, the complete Off -> Precharge -> Run -> Shutdown sequence, authorized transition permission, voltage-settle checks, simultaneous welded AIR+/AIR- detection, persistent latching, and verified all-open controlled clear. |
| `test_air_monitor_faults_freshness_and_tick_wrap` | Exercises boot into Run rejection, stale command/contact recovery, direct Off -> Run rejection, failed-close deadlines, line-supervision faults, open-state bus-voltage plausibility, multiple fault masks, and 32-bit tick rollover. |
| `test_air_monitor_seeded_invariants` | Runs 10,000 deterministic malformed/stale/transition/contact/voltage combinations and proves permit, readiness, active-fault and latched-fault invariants remain internally consistent. |
| `test_current_fault_threshold_edges_and_recovery` | Verifies warning-only recovery, fast-trip debounce/latch timing, latch reset behavior, charge overcurrent, and regen-disabled placeholder behavior. |
| `test_voltage_fault_thresholds_latch_and_reset` | Verifies voltage threshold boundaries for OV warning, charge stop, hard/severe OV, soft/hard/severe UV, latched reasons, and cell location propagation. |
| `test_voltage_fault_read_failure_precedence_and_strings` | Verifies not-ready, partial/stale/PEC read-fault precedence, strict full-fresh-scan validity, and voltage fault reason strings. |
| `test_adbms_spi_debug_write_and_full_duplex_paths` | Verifies SPI debug state for write and full-duplex read paths, CS wrapping, dummy-byte TX padding, RX extraction, HAL error propagation, and debug counters/previews. |
| `test_adbms_spi_debug_rd48_pec_masks_and_clear` | Verifies `rd48` debug command capture, PEC pass/fail masks across multiple ICs, command-counter capture, debug clear/enable behavior, and SPI op strings. |
| `test_adbms_spi_scope_activity` | Verifies the bench scope traffic helper preserves the selected string, emits the visible MOSI pattern, repeats valid RDCFGA command bursts, and rejects invalid repeat counts. |
| `test_adbms_spi_sid_status_and_counter_mismatch` | Verifies ADBMS6830 RDSID product-ID validation, valid/identity-mismatch masks, RDSTATC/RDSTATD/RDSTATE parsing, and command-counter mismatch detection. |
| `test_adbms_spi_coldwake_and_clear_flags` | Verifies conservative cold-wake pulse generation and CLRFLAG all-flag packing/command dispatch. |
| `test_adbms6830_diagnostic_commands_and_cli_health` | Verifies ADBMS6830 config readback, cell ADC diagnostic hook, full open-wire command hooks, AUX/GPIO diagnostic hook, sticky health counters, and CLI visibility. |
| `test_adbms_periodic_diagnostics_and_safe_open_wire` | Verifies real-time status/config/open-wire scheduling, config mismatch fail-closed behavior, and automatic full open-wire evaluation in charge/discharge/balance states. |
| `test_adbms_cli_scan_guard_and_cs_probe_commands` | Verifies CLI SPI probes, PE4/PF4 candidate pin pulsing, and scope traffic are refused during active ADBMS scans, CS_A/CS_B probe commands select the intended chip-select path, scope preset/toggle mode drives default `spi scope`, and manual full/phase open-wire commands are limited to safe service states. |
| `test_adbms_open_wire_full_measurement_and_fault_injection` | Links the real driver and verifies both S-ADC parity commands, conversion waits, five-group reads for 15 populated cells, exact fault-bit mapping, bad PEC/counter rejection, stale-buffer preservation, and stopped-timer failure. |
| `test_adbms2950_spi_debug_write_and_full_duplex_paths` | Verifies ADBMS2950/APM SPI write and full-duplex read debug state, CS wrapping, dummy-byte TX padding, RX extraction, HAL error propagation, and counters/previews. |
| `test_adbms2950_pec_write_is_bounded_and_reference_equal` | Regression for the sanitizer-discovered write-PEC overflow: verifies a six-byte write payload is not mutated or indexed past its end and matches the independent PEC calculation. |
| `test_adbms2950_spi_probe_pec_masks_and_clear` | Verifies ADBMS2950/APM RDCFGA probe debug capture, PEC pass/fail masks, command-counter capture, debug clear/enable behavior, and SPI op strings. |
| `test_adbms2950_mixed_chain_init_identity_and_readback` | Verifies bounded one-device topology, String-B identity before writes, no second chain reset, byte-for-byte config readback, wrong-product rejection, divider-off defaults, and fail-low cleanup after a configuration write failure. |
| `test_adbms2950_sid_probe_and_primary_sample_integrity` | Verifies RDSID identity, RDSTAT calibration, RDIVB1 signed scaling, PEC rejection, coherent command-counter pairs, reset/clear sentinel rejection, and preservation of the last good sample on failure. |
| `test_adbms6830_final_ring_subset_write_owner` | Verifies the five-device String-A subset is exactly 44 bytes and that the same register write from String B is rejected before GPIO/SPI activity. |
| `test_adbms2950_final_ring_subset_write_owner` | Verifies the one-device String-B subset is exactly 12 bytes and that an APM register write from String A is rejected before GPIO/SPI activity. |

Add every hardware-discovered bug as a new host regression case when possible.

Hardware bring-up notes:

| Area | Purpose |
|---|---|
| ADBMS6830 SPI debug CLI | Firmware exposes `spi status`, `spi preset`, `spi toggle`, `spi scope`, `spi probe`, `spi sid`, `spi stat`, `spi staterr`, `spi cfgchk`, `spi cellst`, `spi owcheck`, `spi oweven`, `spi owodd`, `spi auxdiag`, `spi wake`, `spi coldwake`, `spi clrflag`, `spi clear`, `spi diagclear`, `spi enable`, and `spi disable` for board-side ADBMS6830 chain bring-up. Host tests cover command/result logic; physical thresholds, timing, and harness mapping still require injected-open LV testing. |
| ADBMS2950/APM final-ring CLI | Firmware exposes `apm status`, `apm health`, `apm sid`, `apm config`, `apm sample`, `apm scope [1-100]`, `apm clear`, `apm enable`, and `apm disable`. The APM is enabled by default on String B, divider controls default off, and its data remains advisory/non-gating. |
| Dedicated APM SIL gate | `make apm-sil` links the real ADBMS2950 driver for unit/injection tests and separately runs topology, fault-isolation, and CLI behavior in the system SIL harness. |
| Hardware bring-up BMS_OK inhibit | Firmware supports `AMS_HW_BRINGUP=1`, which defaults BMS_OK output inhibited until `bmsok release` is run from CLI. `bmsok inhibit` forces it low again. |
| Staged bring-up CLI summaries | Firmware exposes `bringup board`, `bringup adbms6830`, `bringup apm2950`, `bringup charger-lv`, `bringup charger-battery`, `bringup ready`, `bringup snapshot`, and `bringup evidence` to make bench phase decisions repeatable without mutating safety state. |
| Shared ADBMS SPI lock | ADBMS6830 and ADBMS2950 low-level SPI transfers call shared `adbms_spi_lock()` / `adbms_spi_unlock()` hooks so CLI probing cannot collide with periodic reads on SPI6. |
| Fan fail-safe/ramp | Fan task drives max PWM when temperature data is invalid/stale/unavailable, and ramps PWM between low/high temperature thresholds when temperature data is valid. |
| Software heartbeat monitor | Error task checks ADBMS, current, temperature, CAN, fan, IMD (when enabled), and logger/dashboard heartbeats. ADBMS/current/temp/CAN/fan/IMD stale is safety-critical and drops BMS_OK; logger/dashboard stale is diagnostic-only. |
| CPU panic safety path | HardFault, MemManage, BusFault, UsageFault, NMI, and Error_Handler force BMS_OK low through a direct GPIO reset path before entering the fault loop. |
| Reset/panic record | `.noinit` RAM keeps the last panic reason, ARM fault status registers, reset flags, and a versioned diagnostic event ring with record sequence, CRC, commit-last publication, and recovery from torn/corrupt entries. |
| Deterministic RTOS allocation | All nine application tasks, the ADBMS recursive mutex, idle task, timer task, and timer queue use static storage; `make static-allocation-gate` rejects dynamic application task creation. |
| Target project metadata | `make target-project-gate` parses CubeIDE XML, rejects stale DER25 paths/identity, requires project-relative Debug/Release directories, checks explicit warning flags and confirms the STM32F767 flash linker script. |
| Watchdog gate | Optional IWDG support is disabled by default and, when compiled in, is fed only by the error task after safety heartbeat and sensor freshness gates pass. |
| CAN bus-off recovery | CAN error polling tracks HAL error code, bus-off count, recovery count, and attempts a delayed peripheral restart after bus-off. |
| Hardware SPI bring-up guide | `docs/HARDWARE_SPI_BRINGUP.md` documents first-flash setup, logic-analyzer channels, expected SPI mode 3 behavior, CLI command order, fault isolation, APM probing, and BMS_OK release criteria. |

## SoP black-box property and characterization tools

| Target / source | Purpose |
|---|---|
| `make power-metamorphic` / `sop/sop_metamorphic_oracle.c` | CI-gating deterministic test over 20,000 drive and 20,000 charge states. Verifies horizon nesting, monotonic response to stricter voltage/resistance/uncertainty/temperature/SoC/current-ceiling conditions, charge blocking below the minimum temperature, and fail-zero behavior for NaN, infinity, and stale data. Reports the first failing seed. |
| `make power-sensitivity` / `sop/sop_sensitivity_probe.c` | Non-gating characterization of 30-second DCL sensitivity to thermal-network, uncertainty, temperature-limit, and R0-prior changes at representative operating points. Used to identify when static current ceilings mask the battery model. |

## Main-fuse independent oracle and replay

| Target / source | Purpose |
|---|---|
| `make fuse-oracle` / `fuse/fuse_oracle_test.c` | CI-gating independent validation of the production fuse observer. Checks an exact `long double` zero-order-hold reference against high-resolution trapezoidal integration, then compares 50,000 randomized production updates for no thermal-state underestimate, no nonconservative current cap, bounded numeric error, conservative latch behavior, and fail-closed invalid inputs. |
| `Tools/fuse_replay/fuse_replay` | Strict per-sample CSV replay through production and independent reference paths. Reports state/cap deltas, initialization, authority, exhaustion, recovery, reason flags, and reset behavior. |
| `make fuse-replay` | Non-gating generation and characterization of pulse, corner-exit, autocross, endurance, invalid-input, and warm-reset traces over startup/reset, budget-fraction, and cooling-time policies. Does not enable fuse authority or select final calibration. |
