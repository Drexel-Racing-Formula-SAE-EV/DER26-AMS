# Vehicle main-branch source fixes — v0.5.57-r1

Based on `DER26-AMS-v0.5.57-VEHICLE-MAIN-1M-CONSOLIDATED-SOURCE.zip`.

1. Set SoC initialization to 27% in both the vehicle communications (profile 4) and vehicle-release (profile 3) configurations; all five segment EKFs and coulomb counter share the prior.
2. Correct inherited `.ioc` / `.project` checks for `DER26-AMS-VEHICLE-1M-CLI4`, including four valid build configurations, 1 Mbps assertions and unique build output paths.
3. Change inherited CAN clock gate to check the actual vehicle `.ioc` at 1 Mbps, rather than the deleted 250 kbps charger project.
4. Replace obsolete multi-profile topology test targets with two vehicle-specific startup topology checks.
5. Link the host-only HAL and RTOS tick shims into `make unit` to fix unresolved symbols from ADBMS2950B source. These are NOT compiled into the vehicle firmware. The old unit suite now executes but has 68 failing legacy assertions; do not mistake link repair for a full-suite pass.
6. Preserve AIR auxiliary feedback **disabled** because the hardware is not installed, and leave the planned dual-CAN gateway out of this version.
7. Correct the legacy profile gate's stale six-device/APM assumptions to a five-SMB, no-APM vehicle topology and add `make vehicle-ci` for supported tests.

No target-validity acknowledgment flags were asserted in CubeIDE profiles. The profile-3 evidence gate remains intact.
