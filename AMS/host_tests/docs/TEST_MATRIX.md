# Test Matrix

| Test function | Purpose |
|---|---|
| `test_accumulator_stats_and_balance` | Verifies voltage min/max/total stats, invalid-cell skip, bounded SMB count, balance mask generation, balance clear. |
| `test_voltage_stats_boundaries_and_fuzz` | Fuzzes valid/invalid cell-code combinations and verifies only valid readings affect pack stats. |
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
| `test_current_fault_policy` | Verifies discharge debounce, precharge fast trip, persistent sensor-fault confirmation, and placeholder regen-unexpected warning policy. |

Add every hardware-discovered bug as a new host regression case when possible.
