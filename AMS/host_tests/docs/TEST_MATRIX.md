# Test Matrix

| Test function | Purpose |
|---|---|
| `test_accumulator_stats_and_balance` | Verifies voltage min/max/total stats, full-usable scan gating, bounded SMB count, balance mask generation, balance clear. |
| `test_voltage_stats_boundaries_and_fuzz` | Fuzzes valid/invalid cell-code combinations and verifies only usable/fresh readings affect pack stats. |
| `test_voltage_fault_policy_and_stale_tolerance` | Verifies voltage fault staging, charge-stop behavior, hard OV/UV latching, and single-scan PEC/miss tolerance versus persistent stale-cell fail-closed behavior. |
| `test_temp_stats` | Verifies temperature max/average behavior while skipping invalid raw channels. |
| `test_temp_invalid_and_cold_valid_fault_behavior` | Ensures all-invalid temps fault and valid cold temps do not fault. |
| `test_can_telemetry_packets` | Verifies ECU AMS packet headers, CAN ID, 8-byte layout, status/current/voltage/temp/fan payloads. |
| `test_telemetry_absent_segments_and_invalid_channels` | Verifies missing SMB segments and invalid channels zero-fill instead of reading out of bounds. |
| `test_charger_rx_and_tx` | Verifies charger receive parse, charger fault bits, charger command transmit. |
| `test_can_rx_filter_matrix` | Verifies bad RX status, wrong ID type, wrong ID, and short DLC are ignored. |
| `test_charge_state_disable_matrix` | Verifies charge command disable byte for fault, BMS disabled, charger fault, and timeout cases. |
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

Add every hardware-discovered bug as a new host regression case when possible.

Hardware bring-up notes:

| Area | Purpose |
|---|---|
| ADBMS isoSPI debug CLI | Firmware now exposes `spi status`, `spi probe`, `spi clear`, `spi enable`, and `spi disable` for board-side SPI/isoSPI bring-up. This is hardware-debug support; host tests still cover safety logic, not physical SPI timing or transformer/isoSPI behavior. |
