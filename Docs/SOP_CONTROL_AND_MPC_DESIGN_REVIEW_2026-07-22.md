# DER26 SoP — Control-Theory & Systems-Integration Design Review

**Date:** 2026-07-22  ·  **Scope:** `ams_sop.c` (+ `ams_fuse_observer.c`, `ams_power_can.c`, estimator coupling) as the battery-side authority beneath a future ECU MPC / CM200DX command path.
**Backing evidence:** independent metamorphic SoP oracle (40,000 seeded states) and a parameter-sensitivity probe, both built this slice and reproducible from `AMS/host_tests/sop/sop_metamorphic_oracle.c` and the sensitivity probe. Schematics passed over once (conditioning + tractive sheets).

This is a design/control review, not a bug list. The finding register (`DER26_AMS_FINDINGS.csv`) carries the corrected defects.

---

## 1. What the SoP is, control-theoretically

For each direction and horizon *H* ∈ {0.1, 1, 10, 30 s}, the solver computes

> sup { |I| : the constrained 5-segment forward model, driven by a **constant** pack current I, satisfies every cell-voltage, SoC, and two-node thermal constraint at every step over [0, H] }

i.e. a **constant-input viability boundary** of the constrained battery state. It is solved by 16-iteration bisection on |I|, which is valid **only because the feasible set is downward-closed in current** — every constraint (UV, OV, SoC, core/surface temperature) is monotone-worsening in |I|. The oracle confirms this empirically: over 20,000 states, greater resistance, greater uncertainty, lower min-cell voltage, lower SoC, higher temperature, and tighter ceilings **never** raised the discharge boundary (zero violations). So the conservative-margin construction — uncertainty added to current, 3σ subtracted from SoC, margins added to temperature, `r0_upper` used, voltage margins signed correctly — is sign-correct throughout.

Three structural facts matter for everything downstream:

- **The four horizons are four independent boundaries from the *same* t=0 state**, not a single trajectory. `check_current` re-`memcpy`s the initial model each call. "30 s DCL" = largest constant current holdable for 30 s *from now*; "0.1 s DCL" = same for 0.1 s. Longer H ⇒ more time for SoC/heat to accumulate ⇒ tighter. Nesting is then explicitly enforced and re-checked (lines 1038-1096).
- **The map is memoryless per solve.** SoP(x̂, P) is a static nonlinear function of the estimator state and covariance. It has no notion of accumulated usage on its own.
- **The published envelope is a `min()`**, not the battery model alone: `published = ratelimit( min( viability(x̂,P), fuse_cap(I²t), static_ceilings ) )`.

## 2. The dominant finding: static ceilings mask the battery model at nominal operation

The sensitivity probe is unambiguous. At 50 % SoC / 30 °C the **30 s DCL sits at exactly 70.0 A = `discharge_current_max_a[30s]`**, and ±20 % perturbations to core-surface thermal resistance, surface-ambient resistance, core thermal capacity, σ multiplier, and the R0 growth prior move it by **0.0 A**. The battery-physics envelope only becomes binding at the edges — I had to push a hot (48 °C) state and lower the surface-temperature ceiling to make it collapse to 0 A.

Implication for the whole architecture:

- Under nominal conditions the elaborate DADEKF + 2-RC + two-node thermal predictor is a **guard band**, not the primary limiter. The primary limiter is the hard current-path ceiling table (118/80/70/70 A), which is fuse/contactor/harness-derived. This is *correct* safety layering — the hardware ceiling should bite first — but it reshapes priorities:
  - **Calibration priority.** Nominal driveability is set almost entirely by the ceiling table and its reconciliation to the installed fuse/harness (this is finding F-02), *not* by the battery thermal constants. Effort spent tuning thermal `R`/`C` yields near-zero change in the published steady limit until the pack is genuinely near a cell/thermal edge. The battery-model calibration matters for the **cliff**, not the plateau.
  - **If the ceilings are ever raised** (bigger fuse, uprated harness) the battery model *immediately* becomes binding and thermal calibration suddenly governs driveability. Any ceiling change must be paired with battery-thermal validation.
  - **MPC design.** Because steady DCL ≈ a flat 70 A shelf, the MPC's discharge headroom is essentially constant in normal running; the battery limits appear as a *cliff* in corners / heat / low-SoC. The MPC's steady-state job is therefore current-path / fuse-budget management (§4), with the battery envelope as an occasionally-active constraint, not a continuously-active one.

## 3. Where the dynamics live — and the horizon-semantics trap

Because SoP(x̂,P) is memoryless, all of the system's *temporal* protection is carried by three stateful elements composed around it:

1. **The DADEKF** produces x̂ and P; SoP conservatism scales with P (3σ terms on SoC, R0, vp1/vp2, plus innovation). A poorly-excited or diverging estimator therefore **automatically shrinks** the envelope — fail-conservative, good — but couples driveability to estimator health and to the SoH observability windows (rest-anchor capacity, per-segment R0 confidence). Cold / early-session / low-excitation ⇒ high P and conservative priors (0.80 capacity, 1.25 R0) ⇒ smaller envelope by design.
2. **The fuse I²t observer** — the one genuine *resource/energy* state with memory. This is the element that is *supposed* to prevent sustained abuse of the short horizons. Per findings F-01/F-02 it is currently either inert on-vehicle (300 s continuous-quiescent init unreachable) or, if armed, over-biting (2005 A²·s budget). **This matters more than it looks**, because it is the AMS-side answer to the horizon-reset problem below.
3. **The asymmetric recovery rate limiter** (`ams_sop_apply_recovery`) — immediate reductions, rate-limited increases, with the *previous published limit* as the integrator state and the rise-rate selected by the *previous binding cause* (voltage fast, thermal slow, SoC gated on a recovered flag, fuse-thermal scaled by headroom). This is the anti-chatter / de-bounce that turns the jumpy per-state viability boundary into a limit that cannot pump upward.

**The horizon-reset hazard (handoff §13.4, restated in control terms).** The four horizons are a *set-valued* constraint — a viability tube parameterised by dwell time — not four scalar pointwise limits. A naive ECU that reads "0.1 s DCL = 118 A" and re-requests it every 10 ms **resets its own horizon**: each AMS solve recomputes 0.1 s-feasibility from the new (barely-moved) state, so 118 A stays "0.1 s-feasible" far longer than 0.1 s. The long horizons exist precisely to bound sustained draw, but nothing in the AMS forces the ECU to select the horizon matching its intended dwell. The AMS publishes the *instantaneous* tube; the *time-integral* the tube is meant to bound is deliberately **not** published as a scalar — it lives only in (a) the fuse I²t state and (b) the estimator's slowly-evolving x̂. So the safety net against a horizon-resetting MPC is exactly the fuse observer + the accumulating thermal/SoC state feeding back into SoP — with lag. **This is why F-01/F-02 are architecturally important, not just fuse-hygiene:** the fuse observer is the intended fast resource-memory that closes the horizon-reset loop, and it is currently the weakest link.

Correct ECU-side resolutions (in preference order): (i) a reduced-order shadow model in the MPC carrying polarization + thermal + I²t resource states; (ii) rolling energy/charge/I²t constraints derived from the four horizons; (iii) AMS additionally publishes physical resource states (fuse utilisation is already on 0x689 strategy; thermal-energy-to-target is too). The AMS remains the final clamp regardless.

## 4. Cascade / closed-loop behaviour with the MPC

The intended loop is a **cascade with an outer safety projection**: MPC (fast, optimises tracking) → AMS clamp (slow-rising, immediate-dropping saturation) → CM200DX inner current loop. Structurally this is a relaxation system: request torque → draw current → deplete headroom → AMS derates DCL *immediately* → MPC clamps → headroom recovers → AMS slews DCL up at the cause-specific recovery rate. The immediate-down / rate-limited-up asymmetry is the standard defence against fast limit cycles, but two couplings deserve dyno characterisation:

- **Recovery-rate ↔ MPC-weight ↔ estimator-lag are not independent.** If the MPC is aggressive and the recovery rate is mistuned relative to the thermal/estimator time constants, the loop can surge at the recovery timescale (slow torque oscillation on corner exit). These three calibrations must be tuned together, not in isolation.
- **The transmit-path re-read is load-bearing** (handoff §13.5): because the MPC may compute against a limit that is reduced to zero microseconds later, the final `0x0C0` encode must re-read the newest accepted DCL/CCL and clamp. The AMS provides the newest-authority semantics; the ECU must honour them. This is the single rule that keeps the cascade safe under stale-request timing.

## 5. Model-form notes (control / electrochem)

- **Two-node thermal, hottest-surface-as-ambient proxy.** No cooling credit (conservative). No pack ambient sensor. The proxy is reported in diagnostics. Fine for a subtractive guard; the thermal *constants* remain HIL-plant values unvalidated against real segment construction (matters only in the cliff regime, per §2).
- **CCL is intentionally non-monotonic in temperature.** The oracle initially "failed" a colder-→-lower-CCL check on 16,668/20,000 charge states. This is **not a bug** — above `charge_temp_min` (3 °C), colder gives more headroom to the 42 °C charge ceiling, so colder legitimately *raises* the thermally-limited CCL; below 3 °C charging is hard-blocked (verified: sub-3 °C ⇒ CCL = 0 on all states). **The audit's stated invariant "colder charging temperature cannot increase CCL" is therefore incorrect as a universal property.** Design consequence: any downstream charge-temperature derating must be a *peak/interval* shape (optimal-temperature window), never a monotone table. Discharge, by contrast, tested monotone in temperature over 20,000 states.
- **`docv_dtemp` is a counter-monotone term** in the voltage prediction (higher current → higher core temp → possibly higher OCV, offsetting IR sag). In principle this can break bisection's downward-closed-feasibility assumption. Empirically it never bit (discharge monotonicity held on 20,000 states with the P42A LUT as-parameterised), but it is a latent risk if the OCV(T) LUT is ever re-fit with larger dOCV/dT. Recommend a cheap monotonicity guard or an assertion on the sign/magnitude of the temperature-voltage coupling relative to the IR term.

## 6. Independent verification performed this slice

- **Metamorphic SoP oracle**, 20,000 drive + 20,000 charge seeded states, black-box over `ams_sop_solve` (does not reuse production feasibility as its own oracle). **All corrected invariants pass, zero violations:** nesting (both directions); DCL non-increasing under lower-min-voltage / higher-R / higher-uncertainty / higher-temperature / lower-SoC / tighter-ceiling; sub-`charge_temp_min` ⇒ CCL = 0; NaN-injected input ⇒ never OK-with-authority. Reproducible by seed (xorshift32, `seed·2654435761+1`).
- **Sensitivity probe:** established the §2 ceiling-dominance result.
- **Not done (calibration, needs hardware):** the oracle validates *structure and sign*, not numeric calibration. It cannot tell you whether 70 A is the right ceiling or whether the thermal constants predict the real segment's temperature rise. Those require instrumented segment/HIL characterisation — the oracle is the guard that any future re-calibration hasn't broken monotonicity or fail-zero.

## 7. Recommendations (design-level, no code changed)

1. **Treat the current-path ceiling table as the primary nominal-driveability calibration** and reconcile it to the installed fuse/harness/contactor evidence (ties to F-02). Battery-thermal calibration is second-order until the pack is near an edge or the ceilings are raised.
2. **Make the fuse I²t observer actually usable on-vehicle** before relying on the horizon design against a horizon-resetting MPC (F-01): a known-cold pit-arm init and/or a conservative persisted warm-start with CRC + age, plus a defined post-reset behaviour that assumes warm rather than unprotected. (Proposed, not patched.)
3. **Design the ECU MPC with an internal resource/dwell state** (shadow I²t + polarization + thermal), not pointwise horizon reads; keep the AMS DCL/CCL as the final re-read clamp at transmit.
4. **Co-tune recovery rates, MPC weights, and estimator process noise on the dyno**; characterise corner-exit surge.
5. **Charge-temperature policy must be interval-shaped**, per §5.
6. Optional: add a bisection-monotonicity guard covering the `docv_dtemp` counter-term.
