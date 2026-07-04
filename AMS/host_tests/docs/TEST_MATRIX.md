# Test Matrix

| Test function | Purpose |
|---|---|
| `test_accumulator_stats_and_balance` | Verifies voltage min/max/total stats, full-usable scan gating, bounded SMB count, balance mask generation, balance clear. |
| `test_voltage_stats_boundaries_and_fuzz` | Fuzzes valid/invalid cell-code combinations and verifies only usable/fresh readings affect pack stats. |
| `test_voltage_fault_policy_and_stale_tolerance` | Verifies voltage fault staging, charge-stop behavior, hard OV/UV latching, and single-scan PEC/miss tolerance versus persistent stale-cell fail-closed behavior. |
| `test_system_sil_boot_ready_and_bms_conjunction` | System-level SIL: verifies BMS_OK cannot assert before both current and voltage are valid, and that either current-invalid or voltage-not-ready keeps BMS_OK low. |
| `test_system_sil_single_pec_miss_tolerated_then_recovers` | System-level SIL: verifies one PEC/missed cell update is observable as a warning but tolerated while prior cell data is fresh, then clears on recovery. |
| `test_system_sil_persistent_voltage_stale_drops_bms_ok` | System-level SIL: verifies repeated missed/PEC-failed voltage scans age into stale data, drop BMS_OK, and recover when a full fresh scan returns. |
| `test_system_sil_charge_stop_allows_balance_before_hard_ov` | System-level SIL: verifies 4.18 V charge-stop keeps BMS_OK and still allows controlled balancing, while 4.20 V hard OV latches, clears balancing, and drops BMS_OK. |
| `test_system_sil_voltage_uv_ov_severe_diagnostics_and_latch` | System-level SIL: verifies soft UV warning, hard UV latch, severe OV diagnostic reason, and severe UV diagnostic reason through the ADBMS task/BMS_OK path. |
| `test_system_sil_current_warning_fast_trip_and_latch_persistence` | System-level SIL: verifies current warning-only behavior, fast discharge trip, BMS_OK drop, and latch persistence after current returns normal. |
| `test_system_sil_current_stale_adc_pair_fails_safe` | System-level SIL: verifies one stale ADC channel makes current invalid, ADBMS gating drops BMS_OK, and repeated ADC failures become a confirmed current sensor fault. |
| `test_system_sil_regen_and_charge_current_placeholders` | System-level SIL: verifies low regen is warning-only while regen/charge overcurrent levels latch and drop BMS_OK. This caught and fixed the regen-warning masking bug. |
| `test_system_sil_2950_debug_non_safety_until_integrated` | System-level SIL: verifies ADBMS2950/APM debug errors do not affect BMS_OK while the 2950 path remains debug-only and not safety-integrated. |
| `test_system_sil_combined_fault_precedence_and_reset_path` | System-level SIL: verifies current and voltage latches can coexist, reset independently, and BMS_OK only reasserts after both are cleared and healthy. |
| `test_system_sil_harsh_timeline_no_false_enable` | System-level SIL: runs a harsh multi-phase timeline through precharge, voltage read loss, PEC miss tolerance, precharge current fault, recovery, and persistent voltage loss without false BMS_OK enable. |
| `test_system_sil_current_invalid_immediate_bms_drop_and_recovery` | System-level SIL: verifies a current ADC/status failure immediately drops BMS_OK, holds it low until current recovers, and only reasserts after the next healthy voltage/current conjunction. |
| `test_system_sil_hard_fault_and_corrupt_smb_config_fail_closed` | System-level SIL: verifies hard_fault cannot be overwritten by the ADBMS task, corrupted/low SMB count cannot reduce the required 75-cell scan, and null SMB driver pointers fall back safely to internal storage instead of crashing. |
| `test_system_sil_voltage_threshold_exact_edges` | System-level SIL: verifies exact edge behavior at 4.179/4.180/4.199/4.200 V and 2.501/2.500 V through task-level BMS_OK gating. |
| `test_system_sil_charger_disable_from_dynamic_gates` | System-level SIL: verifies charge-stop, current-invalid, and warning-only charge-current cases drive the charger disable byte correctly through the CAN charge task. |
| `test_system_sil_deterministic_fault_injection_invariants` | System-level SIL: runs a deterministic 192-case cross-product of state, current, voltage, stale/PEC masks, hard/fuse/charger faults, and ADC failures, asserting that any BMS_OK true state satisfies all safety invariants. |
| `test_system_sil_task_order_permutations_fail_closed` | System-level SIL: permutes current, ADBMS, charger/CAN, and error-task ordering to ensure once a fault is observed no later task can incorrectly reassert BMS_OK. |
| `test_system_sil_recovery_and_latch_reset_paths` | System-level SIL: verifies warning-only current/voltage states recover automatically, while latched current and voltage hard faults require explicit latch reset before BMS_OK can return. |
| `test_system_sil_current_boundary_timing_edges` | System-level SIL/policy boundary test: verifies 499 ms does not trip a 500 ms debounce, 500 ms does; 99 ms does not trip a 100 ms fast fault, 100 ms does, including current-task integrated timing. |
| `test_system_sil_cli_can_diagnostic_consistency` | System-level SIL: verifies stale voltage and current ADC faults appear in CLI fault/voltage/current diagnostics and that unusable stale cells zero-fill in CAN voltage telemetry instead of reporting misleading data. |
| `test_system_sil_bringup_status_and_bmsok_inhibit` | System-level SIL: verifies BMS_OK output inhibit blocks attempted assertions, counts blocked assertions, CLI release/inhibit commands work, and status reports Mode 3 SPI settings. |
| `test_system_sil_contradictory_dhab_vs_2950_observable_non_gating` | System-level SIL: injects contradictory DHAB and ADBMS2950/APM current values to verify the 2950 debug path remains observable but non-safety-gating until intentionally integrated. |
| `test_system_sil_startup_garbage_never_enables_bms` | System-level SIL: fuzzes uninitialized-looking current ADC counts and cell codes before the first full voltage scan and verifies BMS_OK never asserts. |
| `test_system_sil_long_run_seeded_fuzz_invariants` | System-level SIL: runs 10,000 deterministic cycles with randomized state, current, ADC failures, PEC/stale masks, OV/UV injections, charge/precharge/regen cases, and task-order permutations while checking safety invariants after every cycle. |
| `test_temp_stats` | Verifies temperature max/average behavior while skipping invalid raw channels. |
| `test_temp_invalid_and_cold_valid_fault_behavior` | Ensures all-invalid temps fault and valid cold temps do not fault. |
| `test_can_telemetry_packets` | Verifies ECU AMS packet headers, CAN ID, 8-byte layout, status/current/voltage/temp/fan payloads. |
| `test_telemetry_absent_segments_and_invalid_channels` | Verifies missing SMB segments and invalid channels zero-fill instead of reading out of bounds. |
| `test_charger_rx_and_tx` | Verifies charger receive parse, charger fault bits, charger command transmit. |
| `test_can_rx_filter_matrix` | Verifies bad RX status, wrong ID type, wrong ID, and short DLC are ignored. |
| `test_charge_state_disable_matrix` | Verifies charge command disable byte for fault, BMS disabled, charger fault, and timeout cases. |
| `test_charger_command_priority_tx_failure_and_cli` | Verifies the charger command is transmitted before telemetry in charge mode, TX failure drops BMS_OK and is exposed in charger CLI diagnostics. |
| `test_current_sensor_measurement_model` | Verifies DHAB 0.6-divider reconstruction, zero-current offset, design-file C_SENSE_L=50A / C_SENSE_H=800A mapping, range selection, saturation, mismatch, and implausible ADC handling. |
| `test_current_task_measurement_state` | Verifies current task propagation of `current_valid`, selected range, measurement reason, stale-current hold on ADC read failure, and confirmed sensor-fault timing. |
| `test_current_task_threshold_faults` | Verifies discharge overcurrent debounce/latch behavior and low-current precharge fast-fault behavior using the corrected DHAB mapping. |
| `test_fan_current_and_null_guards` | Verifies fan driver edge cases and defensive null handling. |
| `test_periods_and_driver_edge_cases` | Verifies task timing period increments, NaN/Inf fan input handling, CAN send guard behavior. |
| `test_task_iterations_with_injected_signals` | Runs one fake iteration of CAN, ADBMS, error, fan, and current tasks. |
| `test_fault_matrix_extra` | Verifies hard/soft fault aggregation, confirmed current sensor soft fault behavior, and BMS_OK safety behavior. |

Unit-only additions:

| Unit test | Purpose |
|---|---|
| `test_current_sensor_conversion_zero_and_range_selection` | Verifies explicit DHAB conversion at zero current, 50A-range selection, and 800A fallback when the 50A channel saturates. |
| `test_current_sensor_invalid_conditions` | Verifies sensor saturation, 50A/800A channel mismatch, implausible ADC rails, stale/missing channel data, and reason/range strings. |
| `test_current_sensor_requires_fresh_pair_and_channel_mapping` | Regression test for the stale-channel bug: conversion requires both ADC channels fresh and preserves C_SENSE_L=50A / C_SENSE_H=800A mapping. |
| `test_current_sensor_read_adc_status_path` | Verifies checked ADC status propagation for normal reads, timeout/error reads, and null ADC handles. |
| `test_current_fault_policy` | Verifies discharge debounce, precharge fast trip, persistent sensor-fault confirmation, and placeholder regen-unexpected warning policy. |
| `test_current_fault_threshold_edges_and_recovery` | Verifies warning-only recovery, fast-trip debounce/latch timing, latch reset behavior, charge overcurrent, and regen-disabled placeholder behavior. |
| `test_voltage_fault_thresholds_latch_and_reset` | Verifies voltage threshold boundaries for OV warning, charge stop, hard/severe OV, soft/hard/severe UV, latched reasons, and cell location propagation. |
| `test_voltage_fault_read_failure_precedence_and_strings` | Verifies not-ready, partial/stale/PEC read-fault precedence, warning-only partial usable scans, and voltage fault reason strings. |
| `test_adbms_spi_debug_write_and_full_duplex_paths` | Verifies SPI debug state for write and full-duplex read paths, CS wrapping, dummy-byte TX padding, RX extraction, HAL error propagation, and debug counters/previews. |
| `test_adbms_spi_debug_rd48_pec_masks_and_clear` | Verifies `rd48` debug command capture, PEC pass/fail masks across multiple ICs, command-counter capture, debug clear/enable behavior, and SPI op strings. |
| `test_adbms_spi_sid_status_and_counter_mismatch` | Verifies ADBMS6830 RDSID parsing, RDSTATC/RDSTATD/RDSTATE diagnostic parsing, and command-counter mismatch detection. |
| `test_adbms_spi_coldwake_and_clear_flags` | Verifies conservative cold-wake pulse generation and CLRFLAG all-flag packing/command dispatch. |
| `test_adbms6830_diagnostic_commands_and_cli_health` | Verifies ADBMS6830 config readback, cell ADC diagnostic hook, open-wire command hooks, AUX/GPIO diagnostic hook, sticky health counters, and CLI visibility. |
| `test_adbms2950_spi_debug_write_and_full_duplex_paths` | Verifies ADBMS2950/APM SPI write and full-duplex read debug state, CS wrapping, dummy-byte TX padding, RX extraction, HAL error propagation, and counters/previews. |
| `test_adbms2950_spi_probe_pec_masks_and_clear` | Verifies ADBMS2950/APM RDCFGA probe debug capture, PEC pass/fail masks, command-counter capture, debug clear/enable behavior, and SPI op strings. |

Add every hardware-discovered bug as a new host regression case when possible.

Hardware bring-up notes:

| Area | Purpose |
|---|---|
| ADBMS6830 SPI debug CLI | Firmware exposes `spi status`, `spi probe`, `spi sid`, `spi stat`, `spi staterr`, `spi cfgchk`, `spi cellst`, `spi oweven`, `spi owodd`, `spi auxdiag`, `spi wake`, `spi coldwake`, `spi clrflag`, `spi clear`, `spi diagclear`, `spi enable`, and `spi disable` for board-side ADBMS6830 chain bring-up. This is hardware-debug support; host tests still cover firmware transaction/debug logic, not physical SPI timing or cable integrity. |
| ADBMS2950/APM SPI debug CLI | Firmware exposes `apm status`, `apm probe`, `apm clear`, `apm enable`, and `apm disable`. APM initialization remains gated behind `AMS_ENABLE_APM_2950_DEBUG=1` until the NDA datasheet and board bring-up are complete. |
| Hardware bring-up BMS_OK inhibit | Firmware supports `AMS_HW_BRINGUP=1`, which defaults BMS_OK output inhibited until `bmsok release` is run from CLI. `bmsok inhibit` forces it low again. |
| Shared ADBMS SPI lock | ADBMS6830 and ADBMS2950 low-level SPI transfers call shared `adbms_spi_lock()` / `adbms_spi_unlock()` hooks so CLI probing cannot collide with periodic reads on SPI6. |
| Fan fail-safe/ramp | Fan task drives max PWM when temperature data is invalid/stale/unavailable, and ramps PWM between low/high temperature thresholds when temperature data is valid. |
| Hardware SPI bring-up guide | `docs/HARDWARE_SPI_BRINGUP.md` documents first-flash setup, logic-analyzer channels, expected SPI mode 3 behavior, CLI command order, fault isolation, APM probing, and BMS_OK release criteria. |
