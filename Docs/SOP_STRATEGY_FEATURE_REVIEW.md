# DER26 SoP strategy review and v0.3 implementation decision

## Safety boundary

The AMS remains the independent physical-envelope authority. It solves all
four 0.1, 1, 10, and 30 second electrothermal horizons every 100 ms against
all 75 series-group voltages, five DADEKF segment states, 120 thermistors,
SoC, uncertainty, SoH, and fixed current-path ceilings. Strategy code may only
reduce those results. It may not change a voltage, SoC, temperature, current,
or uncertainty constraint.

The ECU remains the torque allocator. An ESP32 logger, GPS, IMU, or mission
switch is not a source of AMS measurement truth and cannot increase the hard
envelope.

## Decision summary

| Proposal | Decision | v0.3 implementation |
|---|---|---|
| A. Stationary thermal priming | Do not automate on present hardware | Thermal-readiness, energy-deficit, and natural self-heat diagnostics only |
| B. Mission profiles | Implement with a corrected safety boundary | CRC/counter/freshness request; Qualify, Endurance, and latched Limp Home only derate the hard envelope |
| C. Track-aware horizon scaling | Do not remove or skip safety horizons | All horizons always run; track context may later guide an ECU torque allocator inside the envelope |
| D. Fuse I2t | Implement conservatively | EAC14-80 observer is subtractive and calibration-gated; it never raises 118/80/70/70 A |
| E. Cold-start SoH excitation | Do not inject unsolicited torque | Existing DADEKF natural-excitation R0 learning retained; confidence/progress is exposed |
| F. Segment shedding | Reject | One series current path prevents segment-current redistribution; retain weakest-segment global limiting |
| G. Cause-specific recovery | Implement | Voltage, thermal, current-path/fuse, and SoC recovery have different rates and release conditions |

## A. Thermal readiness instead of autonomous motor heating

There are 450 P42A cells. The production two-node model assigns 55 J/K core
and 15 J/K surface heat capacity per cell, so the modeled cell thermal capacity
is about 31.5 kJ/K. Raising every cell from 15 to 25 degC therefore requires at
least 315 kJ, or 87.5 Wh, before enclosure and environmental losses.

The P42A data sheet gives a typical 16 mOhm DC impedance at 10 A for one cell.
A rough new-cell pack value is:

```text
R_pack = 75 / 6 * 0.016 = 0.20 ohm
P_cell_loss at 80 A = I^2 R = 1.28 kW
ideal 10 degC heat-up time = 315 kJ / 1.28 kW = 246 s
```

This is an order-of-magnitude check, not a heater calibration. Cold resistance,
airflow, enclosure mass, gradients, SoH, and the actual DC current produced by
an inverter injection mode materially change it. More importantly, motor
windings heat the motor and inverter. They heat the accumulator only through
the battery's own electrical losses; there is no thermal path from the motor
to the pack. A high DC load can also consume substantial energy before a run.

The installed CM200 command/configuration evidence supplied with this release
does not prove a stationary AC-injection or guaranteed-zero-torque heating
mode. An automatic implementation could create torque, violate the normal
Ready-to-Drive sequence, or operate in an inappropriate competition location.
No CAN command or inverter behavior is invented in firmware.

v0.3 instead reports:

- minimum DADEKF core temperature;
- modeled energy required for every segment to reach 25 degC;
- present pack self-heating power from measured current and segment R0;
- idealized time at the present self-heating rate;
- a thermal-ready flag.

If preconditioning is later desired, use a purpose-designed accumulator heater
or an operator-controlled service/dyno procedure with the wheels physically
restrained, verified CM200 configuration, torque monitoring, shutdown-chain
supervision, time/SoC/temperature bounds, and rule/event approval.

## B. Mission manager

Mission selection is a soft strategy over a hard envelope, not a different set
of battery safety limits.

| Mode | Discharge recommendation | Transition behavior |
|---|---|---|
| Qualify | Uses the 1 s DCL; all four hard horizons are still calculated and transmitted | Entry requires two fresh matching requests and the ECU stationary-confirmed bit |
| Endurance | Caps all transient recommendations to the current 30 s DCL | Default at boot, stale request, malformed request, or blocked Qualify entry |
| Limp Home | Caps all horizons to `min(30 s DCL, 35 A)` | Automatically latches when the weakest segment 3-sigma SoC lower bound reaches 30%; remains latched until reset |

The 35 A Limp Home ceiling is a conservative commissioning baseline equal to
half the 70 A sustained system ceiling. It is not yet a proven maximum-distance
operating point. A true distance optimizer needs measured vehicle road load,
inverter/motor efficiency maps, auxiliary power, speed, remaining distance,
and usable-energy uncertainty. Until those data exist, firmware must not call
the 35 A value energy-optimal.

The ECU-to-AMS request is standard CAN ID `0x688`, DLC 8, protocol version 1,
CRC-8/SAE-J1850 bound to the CAN ID, modulo-16 counter, two-message recovery,
and 250 ms freshness. CRC and counters detect corruption and discontinuity;
they are not cryptographic authentication. A stale or invalid request selects
Endurance immediately. The separate `0x689` advisory status reports the active
profile, selected horizon, fuse utilization, thermal readiness, and R0
bootstrap progress. Loss of `0x689` does not invalidate the four-frame hard
power bundle.

Qualify never permits a predicted threshold violation. It removes only the
Endurance transient suppression and therefore cannot exceed the hard 1 s
capability. It does not intentionally overheat the battery.

## C. Why track geometry does not scale away a safety horizon

Battery polarization, fuse heating, SoC, and cell-core temperature carry state
between a straight and the next corner. A two-second hairpin does not erase a
30-second thermal history. Skipping the long solve would make the result depend
on an IMU/GPS classification that is neither required nor safety-qualified.

The measured v0.2 host solve averaged about 122 microseconds, far below the
100 ms estimator period. Target DWT WCET is still required, but there is no
current evidence that deleting horizons is necessary.

Future improvement should be an ECU-side trajectory-aware torque allocator:
the ECU can forecast a current sequence from speed, torque, inverter
efficiency, and track context, then keep the entire sequence inside the latest
AMS envelopes. If target WCET later requires multirate computation, cached long
horizons must remain conservative, age-bounded, and invalidated immediately by
material state changes. No ESP32 input belongs in the authority path.

## D. Conservative EAC14-80 fuse observer

The installed main fuse is an Eaton EAC14-80-SCT. Eaton publishes 80 A rating,
approximately 1.03 mOhm typical cold resistance, 150 mV typical voltage drop,
and 8020 A2s typical melting I2t measured at 10 times rated current. The same
data sheet gives a very broad 0.1 to 15 second clearing band at three times
rated current. Therefore 8020 A2s is not a guaranteed usable boundary.

The v0.3 observer uses only 25% of the typical value by default:

```text
I2t_budget = 0.25 * 8020 = 2005 A2s
dE/dt = max(I_adverse^2 - I_continuous(T)^2, 0)
E[k+1] = exp(-dt / 300 s) E[k] + dE/dt * dt
I_cap(h) = sqrt(I_continuous(T)^2 + (I2t_budget - E) / h)
```

`I_adverse` includes current uncertainty. `I_continuous(T)` uses a conservative
piecewise interpretation of Eaton's temperature-derating curve. Because this
hardware has no diagnosed fuse-temperature sensor, the hottest cell-surface
temperature plus 15 degC is used and explicitly flagged as a proxy.

For every horizon, the applied cap is:

```text
min(static 118/80/70/70 A cap, dynamic fuse cap)
```

It can never add current. When its budget is exhausted it requests zero until
utilization cools below 50%. After MCU startup it remains shadow-only until the
model-validation build gate is asserted and the measured current stays at or
below 5 A for a 300-second known-cool initialization. A reboot during a hot run
therefore cannot manufacture dynamic fuse headroom; the existing static SoP
and independent overcurrent trips remain in force.

Installed-fuse, holder, busbar, cable, contactor, ambient, pulse-cycling, ageing,
and power-interruption tests are required before asserting
`AMS_FUSE_MODEL_VALIDATED=1`. Busbars and connectors are not silently modeled
as the fuse. Add separate characterized observers if they later have diagnosed
temperature/current models.

## E. Cold-start R0 and SoH

The five DADEKFs already estimate R0 from real voltage/current response. The
SoH layer accepts a segment observation only with calibrated current, at least
20 A magnitude, at least a 5 A current change, 10-90% SoC, 5-40 degC, valid
innovation/model state, and suitable confidence. Fifty accepted observations
qualify a segment; until then the retained upper bound or the conservative 1.25
resistance-growth prior limits SoP. Capacity SoH remains anchor-to-anchor and
cannot be learned in a 30-second drive.

v0.3 exposes the minimum five-segment R0 confidence as bootstrap progress. It
does not command a sinusoidal torque. Normal driver excitation can qualify R0
without injecting unexpected vehicle motion or corrupting lap operation. If a
curated identification input is desired, make it an explicit service/dyno
procedure with wheels restrained, operator arming, amplitude/frequency/current
bounds, abort conditions, independent torque feedback, and recorded fit
quality. It must never run automatically on the first competition drive.

## F. Why segment-level shedding is not implemented

All five segments are in one series traction path, so their instantaneous
current is identical. A 10 ms torque reduction rests every segment, not only
the weak one. Healthy segments cannot supply the missing series current during
that interval.

For a fixed average current, pulsing also cannot reduce total ohmic loss:

```text
mean(I^2) >= mean(I)^2
```

The greater RMS current increases cell, fuse, busbar, and connector heating.
It may also add driveline torque ripple and NVH, while the AMS estimator runs at
100 ms and cannot supervise a 10 ms waveform. The existing weakest-cell and
weakest-segment binding is the correct architecture: lower global current when
the weak segment binds, preserve its polarization states in the DADEKF, and
use the multi-horizon envelope for any ECU burst allocation.

True segment shedding would require a different accumulator architecture with
independently controlled parallel paths or segment-level DC/DC conversion.

## G. Cause-scheduled recovery

All reductions remain immediate. Only an increase is scheduled, using the
previous binding cause:

| Previous binding | DCL recovery | CCL recovery | Release condition |
|---|---:|---:|---|
| Cell UV/OV polarization | 800 A/s | 120 A/s | Normal fresh solve; roughly an 80 A DCL restoration can occur in 100 ms |
| Core/surface/charge temperature | 8 A/s | 2 A/s | Fresh model stays below limit; thermal recovery remains slow |
| Current path or fuse | 20 A/s times fuse headroom | 4 A/s | Fuse state remains valid; no recovery at 100% utilization |
| SoC low/high | 5 A/s after release | 5 A/s after release | At least 0.5% SoC movement and 453.6 As of net charge/discharge in the restoring direction |
| Other/startup | 40 A/s | 5 A/s | Existing generic recovery |

Invalid data, lost direction authorization, numeric failure, or an infeasible
zero-current state still goes to zero immediately. The ECU consumer still
requires two complete valid bundles before restoring nonzero authority after a
transport fault.

## Further iterations supported by real data

1. Fit a vehicle energy model from coastdown, motor/inverter efficiency, tire,
   auxiliary-power, and lap-distance data; then replace the provisional Limp
   cap with a bounded remaining-distance controller inside the AMS envelope.
2. Add a diagnosed fuse-body temperature sensor and persistent two-slot fuse
   thermal state so the proxy and known-cool startup restriction can be retired.
3. Extend HIL to five independent segment states and exercise mission changes,
   fuse accumulation/cooldown, SoC recovery, resets, packet loss, and repeated
   Qualify bursts.
4. Add an ECU predicted-current trajectory interface only after its units,
   timing, trust boundary, and fail-stale behavior are reviewed. Keep the AMS
   constant-current hard envelope as the independent fallback.
5. Characterize installed busbars, connectors, AIRs, and cables independently;
   do not infer their thermal capacity from the fuse curve.

## Authoritative component sources

- Molicel INR-21700-P42A specification:
  https://www.molicel.com/wp-content/uploads/INR21700P42A-V4-80092.pdf
- Eaton EAC14 EV fuse data sheet:
  https://www.eaton.com/content/dam/eaton/products/electronic-components/resources/data-sheet/eaton-eac14-14x38-ev-fuse-data-sheet-elx1308-en.pdf
- 2026 Formula SAE Rules:
  https://www.fsaeonline.com/cdsweb/gen/DownloadDocument.aspx?DocumentID=278fd4d7-aa27-4e33-bc4a-090148e662a0

The schematics supplied with this project establish the 75s6p series topology,
five 15s6p segments, 80 A fuse, current sensor, and absence of an independent
segment-current actuator or diagnosed fuse-temperature channel.
