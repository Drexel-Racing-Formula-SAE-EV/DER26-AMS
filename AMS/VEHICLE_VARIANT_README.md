> Historical vehicle variant documentation from October 25, 2026.
> For the consolidated v0.5.57 source and its validation, read `README.md`.

# Non-charging vehicle-bus AMS source variant

25 October 2026. Based on CLI timing stage 4, including TEMP1-3 acquisition,
freshness and wake-policy fixes. This is a separate project; it does not replace
the 250-kbit/s charge-only package. Project: DER26-AMS-VEHICLE-1M-CLI4.

## Select the configuration by purpose

| CubeIDE configuration | Build profile | Intended behavior |
|---|---:|---|
| VEHICLE COMMS 1M Debug / Release | 4 | Real five-SMB acquisition, ECU CAN, CLI, permanently inhibited BMS_OK and balancing |
| VEHICLE RELEASE GATED 1M Debug / Release | 3 | Drive source configuration; deliberately rejects build until target-validation evidence is recorded |

The default header selects profile 4. The communications image cannot authorize
propulsion. It is not a wheels-up or driving release. Profile 3 has not been
target-built, flashed, or hardware-qualified here. No ELF/HEX is included.

## Applied changes

- Runtime CAN, all four CubeIDE configurations and CubeMX agree on 1 Mbit/s:
  54-MHz CAN clock / prescaler 3 / (1 + 15 + 2) time quanta. SJW is 2 TQ.
- Charger-only compile branches are off. Vehicle compact/status/power bundles,
  coherent detail publication, ECU feedback filtering and mission-CAN support
  are restored. Optional tuning traffic is off; ordinary vehicle publications
  remain enabled. Protected completion again owns vehicle CAN authority.
- Charge-state requests are rejected by the CLI and central transition policy.
  CHARGE is never a BMS_OK-permitted state in this variant. Charger demand
  construction is inhibited, and the final hardware-load guard forces normal
  charger ENABLE frames to DISABLE with zero voltage/current, even if runtime
  state/permission variables claim otherwise. Existing shutdown-disable traffic
  is retained. Charging is performed with the separate charger project.
- The vehicle topology was stale: six physical devices and ADBMS2950 were
  previously required by the VEHICLE gate. This installed variant now uses
  exactly five SMBs, no APM2950, and the existing onboard String-B path. Defaults
  and vehicle consistency checks agree. The fan task remains disabled.
- TESTDAY retains the previous diagnostic SMB0/C1 exclusion (74 observed out of
  75 physical). It retains its immutable BMS_OK/balance inhibition. VEHICLE uses
  no C1 exclusion and requires all 75 cell readings. No unseen cell is invented.
- STOP consumption now applies whenever the CLI is enabled, rather than only
  in charger builds. It inhibits BMS_OK and uses the vehicle START transition;
  its stale-command generation protection, RX bypass and bounded TX remain.
- Split an overlong status line exposed by the longer vehicle profile name.
  Updated project identity, revision suffix and configuration fingerprint.

## Remaining drive-release blockers

The source still has no valid reading for the previously discussed C1 hardware
channel, and S-path routing was previously reported defective. The drive profile
requires complete observation and redundant C/S voltage authority by default.
The diagnostic 74-cell/C-only mode is not a drive release. Those hardware and
authority differences must be resolved or formally redesigned and validated.

Existing compile-time gates require actual evidence for IMD polarity/input,
watchdog reset, current polarity/calibration/uncertainty, balance thermal/current
limits, CAN freshness/mission and ECU fail-zero behavior, SoP model/calibration/
timing, fuse thermal/extrapolation model, open-wire restore behavior, temperature
pull-ups/thresholds and revision identities. None was marked validated here.
The native test fixture uses synthetic acknowledgements solely to type-check
the drive source; those flags are absent from all four CubeIDE configurations.

Changing compile-time acknowledgements is not the validation itself. Review
the actual measurements and calibration records against ams_build_profile.h.
CLI is compiled out in VEHICLE. Drive BMS permission still depends on its real
supervisor, freshness, SoP, IMD and CAN authority checks. ECU torque readiness
and an ECU drive-profile release are separate work; ECU firmware is not included.

## Verification completed

- Native integration executes actual state handlers, supervisor, CAN receive
  filter, final-load charger guard and one real CAN task iteration. It verifies
  discharge transitions, STOP, rejected charge, zero-demand disable, ECU RX and
  restored protected telemetry reaching the mailbox fixture.
- Real UART-driver IT queue/ordering/overflow/stall tests pass.
- CAN scheduler unit/stress tests pass. Production acquisition/wake-policy and
  measurement-integrity tests pass; acquisition ASan/UBSan passes (leak scanner
  disabled in container).
- Changed acquisition, supervisor, CLI, UART/ISR, CAN, main and app sources pass
  native syntax checks in communications profile. Drive sources pass native
  syntax checks with synthetic evidence, including static assertions for five
  SMBs, no APM/fans, full 75-cell observation and disabled release CLI.
- The actual drive configuration rejects missing evidence. Four CubeIDE
  configurations and runtime/Cube CAN settings are checked programmatically.
- No ARM compiler was available: no ARM compile/link/map/RAM/stack verification,
  physical CAN ACK test, ECU interoperability test, or target timing proof.

Reproduce supported variant checks from the project's host_tests directory:

    make vehicle-variant-test cli-async-test can-tx-scheduler-test vehicle-project-check
    make adbms-wake-policy-test measurement-integrity-test

Older charge/bench/all-profile test targets are inherited historical harness
targets and are not the supported configuration suite for this dedicated project.

## Import and console

Extract the ZIP, then import the DER26-AMS-VEHICLE-1M-CLI4 directory with
STM32CubeIDE: File > Import > General > Existing Projects into Workspace.
Choose VEHICLE COMMS 1M Debug or Release for the inhibited ECU/AMS CAN test.
Use clean/rebuild; confirm the new project name and -DAMS_BUILD_PROFILE=4 plus
-DDER26_CAN_BITRATE_KBPS=1000 in the build output. The older stray bare 'm'
linker-input fix is retained; the standard math library remains linked by -lm.

Communications console: USART3 PD8 TX / PD9 RX, 115200 8N1, no flow control,
3.3-V UART, CR termination. TX/RX crossed and common LV signal ground.

    ver
    status
    uart
    can
    temp
    temp detail 0
    volt
    volt detail 0
    state stop

Connect the ECU/AMS CAN bus at 1 Mbit/s with the intended two end terminators.
Both nodes must ACK frames; a single powered transmitter cannot prove a working
bus. Inspect CAN RX/TX/error and protected-completion counters on both nodes.
No charging or propulsion permission is granted by this communications image.

Evidence includes test output, baseline hash, changed-file inventory and patch.
Documents under history describe earlier charger work, not current permissions.
