# P42A AMS HIL qualification plan

This plan starts only after the static/host checks pass. Record the repository
commit, generated component hash, MATLAB/Simulink versions, ESP-IDF version,
AMS binary identity, wiring revision, CAN bitrate, and test equipment in the
test report.

## Toolchain gate

Run the ordered automation from the repository root:

```bash
python3 HiL/tools/run_toolchain_qualification.py --promote
```

It executes:

1. `run_all_tests` in MATLAB;
2. require the independent P42A electrical holdout to report `PASS`;
3. staged Simulink code generation;
4. fresh generated-C versus configured-Simulink parity;
5. copy the candidate into a temporary ESP-IDF project;
6. `idf.py set-target esp32` and `idf.py build` against that staged component;
7. atomically promote only after every prior gate passes, preserving the old
   snapshot.

Use `--skip-idf` on a MATLAB-only machine for parity evidence only.
`--promote --skip-idf` is forbidden. Without `--promote`, the staged IDF build
still runs but the frozen snapshot remains unchanged.

## Bench setup gate

- Verify isolated power, grounds, 120-ohm termination, CAN-H/CAN-L polarity,
  MCP2515 oscillator configuration, and 250 kbit/s timing.
- Keep AIR actuation and BMS_OK authority inhibited for initial traffic tests.
- Capture both the physical bus and ESP32/AMS serial diagnostics.
- Confirm `0x200` pack voltage is 10 mV/count; cell frames are 1 mV/count;
  temperatures use their documented signed scaling; and all fields are
  big-endian.

## Atomic-image fault matrix

For each case, prove that the AMS publishes either the prior complete image or
the new complete image, never a mixture:

| Injection | Required result |
|---|---|
| Drop one cell frame | COMMIT rejected; prior image retained |
| Drop one temperature frame | COMMIT rejected; prior image retained |
| Duplicate unchanged frame | New image may commit |
| Duplicate changed frame | Generation rejected |
| Reverse all data frames | New image commits |
| Delay an old-generation frame | Frame ignored; active generation remains valid |
| Replay a complete prior generation | Rejected while current image is fresh |
| Corrupt one payload bit | CRC failure; prior image retained |
| Omit COMMIT | Assembly times out; prior image retained |
| Reset ESP32 mid-image | Partial image times out; later generation resynchronizes |
| Command plant soft reset | No zero image; next generation advances monotonically |
| Reset AMS mid-image | No publication until a new complete generation |
| Power-cycle either node | Safe stale behavior, then deterministic recovery |
| Force bus-off | BMS_OK drops; recovery does not resurrect a partial image |

## Timing and resource evidence

Run US06 and endurance-style loops long enough to exercise counter wrap,
thermal rise, profile wrap, and recovery paths. Record minimum/mean/p95/p99/
maximum values and event counts for:

- plant-step execution time (`step_us`, `max_step_us`);
- 67-frame image transmission time (`image_us`, `max_image_us`);
- complete 70-frame burst time (`burst_us`, `max_burst_us`);
- plant and CAN task deadline misses;
- MCP2515 frame failures and retries;
- MCP2515 arbitration-loss, TX-error, abort, bus-off, timeout, warning,
  passive, SPI, and RX-overflow counters;
- AMS CAN RX queue high-water mark and drops;
- AMS image age, accepted/rejected/timeout/CRC/replay counts;
- ESP32 and AMS task stack high-water marks;
- bus-off count and recovery duration.

Use `bringup hil-image` on the AMS CLI to capture committed generation, image
age, staging state, accept/reject/timeout/CRC/duplicate/replay/resync counters,
CAN RX queue high-water/drop counts, and bus-off recovery state.

Acceptance requires zero mixed-image observations, zero unexplained queue
drops, zero missed 100 ms deadlines in the qualification run, deterministic
stale-image fail-closed behavior, and documented timing margin. If the
single-TXB0 MCP2515 driver cannot meet that margin, qualify a multi-buffer or
interrupt-driven transmitter before increasing HIL authority.

## Evidence boundary

Passing this plan qualifies the P42A open-loop AMS-estimator HIL interface. It
does not calibrate fanless pack thermal behavior, create independent 75-group
generated physics, or qualify the future closed-loop vehicle HIL.
