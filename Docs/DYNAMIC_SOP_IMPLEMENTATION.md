# DER26 robust finite-horizon SoP and SoH design

## Purpose and release status

This implementation computes a dynamic discharge-current limit (DCL),
charge/regen-current limit (CCL), and corresponding power envelope for the
DER26 75s6p Molicel P42A accumulator. It also estimates capacity and resistance
State of Health (SoH).

The software path is complete from coherent AMS measurements through the five
segment DADEKF states, SoH, predictive SoP, immutable publication, protected CAN
encoding, dashboard decoding, and a fail-zero ECU reference consumer. A vehicle
build remains compile-locked until model, calibration, target timing, and CAN
integration evidence are explicitly acknowledged. Source completeness is not a
substitute for commissioning evidence.

## Why this is a predictive envelope solver, not a generic MPC controller

SoP asks a bounded feasibility question: what is the largest constant pack
current that can be sustained for a specified time without violating any cell,
SOC, thermal, or current-path constraint? For that problem, a robust forward
predictor plus a scalar safeguarded bisection is preferable to a general QP/MPC
controller:

- it directly returns the 0.1, 1, 10, and 30 second DCL/CCL envelope;
- every trial current is checked against all 75 cells and all five thermal and
  DADEKF segment states;
- the feasibility boundary is monotonic over the bounded operational domain;
- the solver is deterministic, heap-free, and easy to differential-test;
- there is no optimizer infeasibility recovery, warm-start state, or matrix
  factorization in the safety path.

An ECU may use MPC or another optimizer later to allocate requested torque
inside this envelope. The AMS should remain the independent physical limit
authority and should not optimize driver demand.

## Runtime data flow

```text
ADBMS/current snapshot (one coherent epoch)
              |
              v
five segment DADEKF instances + covariance + residuals
              |
              +----> rest-anchor capacity SoH
              +----> DADEKF R0 resistance SoH
              |
              v
robust 75-cell electrothermal predictor
              |
              v
four-horizon hard DCL/CCL + binding constraint + confidence flags
              |
              v
immediate decrease / cause-scheduled recovery
              |
              v
subtractive fuse cap + mission strategy (never raise recovered hard envelope)
              |
              v
immutable snapshot -> four required power frames + advisory strategy status
              |
              v
ECU fail-zero consumer
```

The estimator task runs at 10 Hz. SoH and SoP execute synchronously after a new
coherent measurement epoch is consumed. The CAN task copies only the small
immutable publication snapshot under the existing short critical section; it
does not share mutable solver state across task priorities.

## Electrical model

Project current convention is positive for accumulator discharge and negative
for charge/regen. Pack current is divided across the six parallel cells:

```text
i_cell = i_pack / 6
```

For each segment, the DADEKF supplies SOC, two polarization states, R0, core
temperature, covariance terms, innovation, and validity. Production P42A LUTs
supply OCV, R0, inverse C1, and inverse time constant as functions of SOC and
temperature. The second RC branch uses the project HIL values R2 = 4 mOhm and
C2 = 12000 F.

The RC states use exact zero-order-hold transitions for every prediction step:

```text
v_p[k+1] = exp(-dt/tau) v_p[k]
           + R (1 - exp(-dt/tau)) i_cell

SOC[k+1] = SOC[k] - i_cell dt / Q_cell
```

Predicted terminal voltage is:

```text
v_cell = OCV(SOC,T) - v_p1 - v_p2 - R0_upper i_cell + measured_bias
```

The bias is computed from the present measured cell voltages and the DADEKF
prediction at the present measured current. It contains both the segment
common-mode residual and each cell's offset from its segment average. This
anchors the forecast to the real measurement while preserving weak-cell
dispersion instead of replacing all cells with a segment average.

## Uncertainty treatment

The predictor uses conservative bounds rather than a nominal trajectory alone:

- pack-current uncertainty is applied in the adverse direction with a 0.5 A
  floor;
- DADEKF SOC and electrical covariance terms are expanded by 3 sigma;
- R0 uses the larger of the DADEKF state bound and the P42A LUT multiplied by
  the resistance-SoH upper bound;
- capacity uses the conservative capacity-SoH lower bound;
- voltage includes 5 mV measurement, 20 mV model, covariance, and innovation
  margins;
- thermal limits include 1.5 degC measurement plus 1.5 degC model margin;
- missing capacity or resistance observations use 0.80 capacity and 1.25
  resistance-growth priors and remain visible in flags.

Unknown SoH therefore reduces capability; it never silently assumes a new
battery.

## Thermal model

Each segment uses the same two-node structure and parameters as the DER26 HIL
plant:

```text
C_core dT_core/dt = q - (T_core - T_surface)/R_core_surface
C_surface dT_surface/dt = (T_core - T_surface)/R_core_surface
                           - (T_surface - T_ambient)/R_surface_ambient
```

Heat is conservatively approximated as:

```text
q = i_cell^2 (R0_upper + R1 + R2)
```

The coupled thermal state is discretized with the bilinear/Tustin method, which
is stable at the mixed 0.1, 0.5, and 1 second prediction steps. The present
hardware has no independent ambient sensor; the hottest measured surface is
used as an ambient proxy, giving no initial cooling benefit. The proxy is
always reported in the reason flags.

## Feasibility constraints

Every trial current must satisfy, for every step, segment, and cell:

- cell voltage: 2.80 V minimum discharge and 4.15 V maximum charge;
- SOC: 0.05 minimum and 0.98 maximum, including SOC uncertainty;
- discharge core and surface temperature: 55 degC maximum;
- charge core and surface temperature: 42 degC maximum;
- charge surface temperature: 3 degC minimum;
- horizon-specific pack-current path limit;
- finite model domain, complete 5 x 15 cell topology, all 120 usable
  thermistors, valid DADEKF state, current calibration/polarity, and data age;
- independent drive, charger, and regen authorization.

The four horizons are solved with 16 bisection iterations. The result is forced
to a nested envelope so a longer horizon cannot exceed a shorter-horizon
current. If zero current is physically infeasible, the direction returns a
valid zero limit with the binding constraint rather than a fabricated numeric
failure.

## Publication behavior

Constraint tightening, invalid input, stale data, lost authority, and numeric
failure reduce the published current immediately, including to zero. Recovery
is binding-specific: voltage polarization may restore DCL at 800 A/s, thermal
limits restore at 8 A/s, current-path/fuse limits at up to 20 A/s, and an SoC
limit remains held until measured SoC and net coulomb throughput demonstrate
recovery. Generic recovery remains 40 A/s DCL and 5 A/s CCL. Recovery is
applied to the raw hard result first; the fuse and mission overlays are then
applied as the final subtractive step. The raw model capability, recovered hard
target, strategy-limited target, and authorized publication remain separate.

The EAC14-80 observer and mission manager are subtractive overlays. The fuse
observer applies no authority until its model gate and thermal initialization
are valid, and it never exceeds the static 118/80/70/70 A ceilings. A stale or
invalid mission request defaults to Endurance. Qualify uses the hard 1-second
capability, Endurance suppresses transients to the 30-second capability, and
Limp Home latches from the weakest segment's conservative 30% SoC threshold.
All horizons are still solved in every profile. See
`Docs/SOP_STRATEGY_FEATURE_REVIEW.md`.

The SoP output does not directly assert or release BMS_OK or AIRs. In the
vehicle profile its estimator-task heartbeat is supervised, while the external
torque consumer independently fails to zero on stale/invalid power frames.

## SoH method

Capacity SoH is learned only from coulomb throughput between two observable
rest anchors. An anchor requires 60 seconds with current plus uncertainty at or
below 0.5 A, 10-40 degC average cell temperature, at most 50 mV cell spread, no
unrecovered balancing, SOC sigma at most 0.015, innovation at most 15 mV per
cell, and residual polarization at most 20 mV.

An accepted window additionally requires at least 0.15 SOC change, at least
3 Ah throughput, correct current/SOC direction, a 50-110% nominal-capacity
candidate, and no more than 20% deviation from the learned mean. Accepted
capacity uses Welford online statistics. The conservative value is the mean
minus 3 sigma, with a 0.5 Ah uncertainty floor. Two accepted windows are
required before capacity is marked valid; otherwise the 0.80 prior remains.

Resistance SoH aggregates the five DADEKF R0 health observations. Every segment
must be valid with at least 50% confidence before resistance SoH is valid. The
published upper bound includes a 0.05 floor plus a confidence-dependent margin.
Combined SoH is the more conservative of capacity lower-bound health and the
inverse resistance-growth bound.

SoH persistence uses schema-3 CRC32 records with generation counters, retained
per-segment resistance upper bounds, and a two-slot newest-valid selector. The
core intentionally does not invent a flash
address; the target storage adapter must provide two independently erasable
records, write body and CRC before commit metadata, read-back verification, and
power-interruption testing.

## Deterministic failure rules

The following conditions publish zero DCL and CCL with explicit reasons:

- missing, stale, nonfinite, incomplete, or out-of-range measurement data;
- false current calibration or polarity provenance;
- pack-only estimator topology or any invalid segment DADEKF;
- incomplete 75-cell/120-thermistor coverage;
- model-domain or configuration failure;
- stale solver publication at the CAN task;
- invalid protocol version, CRC, counter bundle, DLC, frame type, or age at the
  ECU consumer.

Unknown SoH alone is not fatal because explicit conservative priors are used.
Direction authorization is independent: drive may be valid while regen is
zero, or charger current may be valid while vehicle regen remains locked out.

## Verification architecture

The target C tests cover invalid input/configuration, NaN/Inf, stale data,
calibration and polarity gates, 75-cell topology, direction authorization,
voltage/SOC/thermal bindings, SoH priors, cause-scheduled recovery, fuse
accumulation/cooldown/authority, mission request integrity and latching,
brute-force boundary checks, SoH observability and persistence, and CAN bit
corruption.

The Python oracle independently parses the HIL plant calibration tables rather
than importing the target LUT source. It differential-tests nominal, weak-cell,
hot, high-SOC, aged, and seeded randomized cases, plus boundary feasibility and
monotonic conservatism. The ECU consumer and dashboard decoder have separate
pure-C tests.

Host timing and GCC stack-frame evidence are useful regression bounds, not an
STM32F767 WCET proof. Vehicle authority still requires DWT cycle measurements,
interrupt-load testing, and a live FreeRTOS stack high-water margin.

## Source map

- `AMS/Core/Src/sop/ams_sop.c`: predictive solver and cause-scheduled recovery.
- `AMS/Core/Src/sop/ams_fuse_observer.c`: subtractive EAC14-80 thermal budget.
- `AMS/Core/Src/sop/ams_power_strategy.c`: mission arbitration and readiness diagnostics.
- `AMS/Core/Src/soh/ams_soh.c`: capacity/resistance SoH and persistence records.
- `AMS/Core/Src/sop/ams_power_state.c`: measurement/DADEKF integration.
- `AMS/Core/Src/sop/ams_power_can.c`: versioned CRC/counter CAN contract.
- `Tools/sop_reference/model.py`: independent differential oracle.
- `Tools/sop_reference/ecu_power_consumer.c`: fail-zero ECU reference consumer.
- `Tools/esp32_ams_dashboard_logger`: passive decoder and display.

## Research and component basis

- Molicel, INR-21700-P42A product specification:
  https://www.molicel.com/wp-content/uploads/INR21700P42A-V4-80092.pdf
- Eaton, EAC14 EV fuse data sheet:
  https://www.eaton.com/content/dam/eaton/products/electronic-components/resources/data-sheet/eaton-eac14-14x38-ev-fuse-data-sheet-elx1308-en.pdf
- TE Connectivity, ECK100B contactor product data:
  https://www.te.com/en/product-2071583-4.html
- Ma, Ren, and Guo, multi-constraint battery peak-power estimation and
  electrothermal predictive control review, 2025, DOI 10.33961/jecst.2024.00724.

The literature supports combined voltage/SOC/temperature constraints and online
capacity/resistance updates. The exact DER26 current ceilings remain governed by
the installed vehicle protection chain and must be commissioned as described in
`SOP_SOH_CALIBRATION_AND_LIMITS.md` and `SOP_SOH_COMMISSIONING.md`.
