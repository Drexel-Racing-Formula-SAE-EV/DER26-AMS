# DER26 AMS MiL Verification Specification — Working Draft

## 1. Purpose

Qualify the AMS battery-state algorithms against the existing high-fidelity distributed MATLAB plant before vehicle authority depends on those estimates. MiL shall answer whether the estimator/constraint theory behaves correctly under known truth, uncertainty and faults. It is intentionally separate from embedded timing, RTOS and physical-I/O qualification.

## 2. Plant and interfaces

MiL shall reuse the existing P42A 75s6p distributed reference plant. The plant evolves 75 series-group states grouped into five 15-series-group segments. Hidden truth includes group SoC, two polarization states, thermal states and parameter multipliers.

Two buses are mandatory:

1. **Truth bus** — inaccessible to estimator/reference algorithms; used only by scoring and independent oracles.
2. **Measurement bus** — derived from truth through realistic measurement behaviour: current dual-range response, noise, bias, scale, polarity, quantization, voltage/temperature errors, timestamps, freshness and validity/fault injection.

Static checks shall fail if independent estimator-reference code accesses `truth` directly.

## 3. Algorithms under verification

- five segment 2RC EKFs: SoC, Vp1, Vp2 and R0-related behaviour;
- innovation gating and covariance consistency;
- production estimator C parity;
- capacity/resistance SoH logic and confidence/observability;
- production finite-horizon SoP for 0.1/1/10/30 s;
- independent truth-forward SoP safe-current oracle;
- current sensor assumptions and calibration effects;
- fuse observer/model consistency as a separate evidence track;
- validity/freshness/fail-closed power-authority behaviour.

## 4. Deterministic scenario classes

### Nominal/dynamic

- US06 nominal 25 C;
- endurance trace;
- qualifying-style high-power bursts;
- low-SoC dynamic operation;
- high-SoC charge/regen operation;
- 5 C and 40 C boundary operation.

### Estimator initialization/model mismatch

- +20% / -20% estimator initial SoC error;
- weak group capacity;
- weak group R0/R1;
- segment temperature mismatch;
- plant/estimator parameter spread;
- timestamp jitter and irregular `dt`.

### Current sensing

- positive and negative zero bias;
- gain error;
- polarity reversal;
- dual-range disagreement;
- 50 A and 800 A channels stuck low/high;
- dual-range dropout and rail/saturation cases;
- zero-calibration validity and accumulated SoC bias.

### Cell/temperature acquisition

- single-cell voltage bias;
- temperature bias;
- stale cell image;
- stale temperature image;
- segment PEC-invalid image;
- missing-cell/open-wire-like cases with stale-last numeric values and invalid freshness.

### SoH

- rest/discharge/rest capacity-observability windows;
- independent capacity fade and resistance growth;
- temperature-only resistance increase to test false-aging rejection;
- mixed aging + temperature + dynamic current.

## 5. EKF scoring

For each segment, record and score:

- SoC error, RMSE, p95 absolute error, maximum absolute error;
- directed startup settling into a defined band after wrong initialization; the
  legacy `convergence_time_s` implementation reports the timestamp immediately after
  the final out-of-band excursion and is therefore a release gate only in scenarios
  that explicitly require directed convergence;
- post-convergence SoC accuracy scored separately from the intentional
  acquisition transient in directed bad-initialization cases;
- Vp1/Vp2 errors where a meaningful segment truth aggregate exists;
- R0 estimate vs effective plant resistance where observable; raw R0 accuracy is
  an applicability-gated release property and must be explicitly required by
  scenarios claiming `EKF-R0`;
- voltage innovation and innovation variance;
- accepted/rejected update fraction;
- covariance symmetry/finite/positive-health checks;
- NIS and, where state truth mapping is valid, NEES.

For scalar voltage innovation:

`NIS_k = nu_k^2 / S_k`.

NIS/NEES acceptance shall be statistical over seeded campaigns, not enforced sample-by-sample. Confidence bounds and alpha shall be frozen in qualification configuration before final campaigns.

The independent reference estimator uses the observable state vector
`[SoC,Vp1,Vp2]`. R0 is evaluated from the reviewed SoC/temperature LUT, including
its SoC derivative in the voltage Jacobian. Adaptive R0 remains a separately
scored production-C behavior; it is not estimated as an ungated fourth state
from the same scalar segment-voltage sample.

Initial development defaults are intentionally provisional: SoC RMSE <=2%, p95 <=4%, worst <=7.5%, directed settling to +/-3% within 60 s where `convergence_required=true`, R0 p95 relative error <=15% where `r0_accuracy_required=true`, and rejection fraction <=5% for nominal scenarios. These are not final hardware claims.

## 6. Independent SoP oracle

At selected checkpoints, the oracle snapshots hidden group truth and searches the maximum feasible pack current separately for discharge and charge over 0.1, 1, 10 and 30 s.

For each candidate current it forward-propagates all 75 groups independently and rejects the candidate when any configured constraint is violated:

- cell UV/OV;
- SoC minimum/maximum;
- charge/discharge temperature boundaries;
- configured hardware current cap;
- fuse cap from the independent production/reference replay comparison.

Search uses bisection. The propagation deliberately differs from production SoP: hidden physical states, exact RC ZOH updates and a distinct thermal discretization are used. The oracle must not call `ams_sop_*`.

Primary safety metric:

`unsafe_overprediction = max(0, I_production - I_oracle)`.

Qualification is asymmetric: unsafe overprediction is a safety failure; conservatism is primarily a performance metric. Current draft tolerance is 2 A / 2% pending model-correlation evidence.

## 7. SoH verification

Capacity and resistance degradation shall be varied independently. Tests must distinguish irreversible degradation from temperature-dependent resistance changes. A SoH result may only be scored when its required excitation/observability conditions are satisfied; lack of excitation must produce low confidence/invalid evidence rather than a false precise estimate.

Physical SoH qualification requires independent aged-cell data. MiL alone demonstrates algorithm behaviour and observability logic.

## 8. Fuse verification

Production fuse state shall be compared against an independent reference/damage implementation and synthetic/race traces. Until the reference is correlated to the exact installed fuse and thermal environment, results are **model-consistency evidence only**, not physical fuse qualification.

## 9. Monte Carlo

Seed all campaigns explicitly and record seeds in artifacts. Vary at minimum:

- initial SoC and group imbalance;
- R0/R1/C1/R2/C2/capacity multipliers;
- thermal parameter multipliers;
- ambient/initial temperature;
- current bias/gain/noise;
- voltage bias/noise;
- temperature bias/noise;
- timestamp jitter;
- weak-group location;
- measurement dropout/freshness faults.

Implemented campaign tiers after deterministic correctness is established:

- PR/smoke: 32 seeded cases;
- nightly: 256 seeded cases;
- release qualification: 1,000 seeded cases plus all frozen deterministic cases.

The 1,000-case release tier is a computational baseline, not a statistical claim.
Increase it when measured parameter distributions and the target confidence bound
justify a larger sample size.

Distributions must be tied to measured/component evidence before results are called qualification rather than robustness screening.

## 10. Data split

Maintain separate scenario sets:

- development/tuning;
- regression;
- frozen qualification/holdout.

Do not change estimator calibration after observing qualification failures without incrementing the calibration/version and rerunning the complete frozen set. Plant HPPC-fit data must remain distinct from electrical holdout data.

## 11. Production parity

The host runners link checked-in production source directly; they shall not duplicate firmware equations. Identical measurement vectors shall be fed to MATLAB reference and production C. Compare with explicit floating-point tolerances rather than requiring bit identity.

MiL reference vs production differences must be classified as one of:

- intended algorithmic difference;
- numerical/discretization difference;
- production defect;
- reference defect;
- unresolved.

## 12. Fail-closed expectations

Fault scenarios shall explicitly check that invalid/stale/untrusted measurements cannot create additional power authority. Examples include current invalidity, polarity not validated, stale cell images and whole-segment PEC invalidity. The exact BMS_OK assertion policy remains a firmware safety requirement and should be evaluated at the supervisory layer rather than inferred solely from estimator output.

## 13. Artifacts/provenance

Every exported run should contain at minimum:

- scenario ID and schema;
- deterministic seed;
- plant configuration hash;
- parameter hash/source;
- MiL configuration hash;
- firmware/source revision where production runners are used;
- summary metrics/pass-fail;
- time-series measurement, truth, reference and production outputs as applicable.

Generated results remain out of source control by default.

## 14. Implementation status

Implemented in this working tree:

- distributed-plant runner and truth instrumentation;
- truth/measurement adapters;
- current measurement model and sensor fault injection;
- independent MATLAB segment EKF;
- production EKF host runner;
- independent SoP oracle;
- production SoP host runner;
- capacity/resistance SoH reference/production scoring and false-aging metrics;
- exact C0-C8 core matrix plus extended deterministic fault inventory;
- tiered deterministic and Monte Carlo campaigns;
- result export/summaries;
- independent fuse replay and fused truth-envelope reporting;
- requirements-to-scenario traceability generation;
- frozen-input readiness audit;
- static architecture checks;
- 1-Mbit/s AMS build/CubeMX/manifest contract with 500/250-kbit/s fallbacks.

Still required before calling MiL complete:

- execute and debug the full MATLAB scenario matrix on licensed MATLAB;
- freeze qualification parameter distributions and acceptance thresholds;
- validate 1-Mbit/s CAN timing and error margin on target hardware with every vehicle node;
- compare the final production supervisory/fuse-limited authority against the combined truth envelope;
- add measured DER26 SD/CAN replay adapter;
- import the exact ECU `CAN###.BIN` record source and add byte-exact log generation/golden vectors;
- add frozen qualification datasets/results after real sensor/pack characterization.
