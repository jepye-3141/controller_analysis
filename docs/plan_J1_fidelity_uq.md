# Plan J1 — Physical Fidelity + Uncertainty Quantification

**Goal:** make the paper's central empirical claim (priority-weighted saturation handling restores ballistic-recovery reachability) survive a JGCD reviewer's two standard attacks: *"your plant is not a real vehicle"* and *"your result is one deterministic run."*
**Status:** partly executed. W1.2 (bus-carried cap) and W1.3 (the T/W sensitivity sweep, which grew into the saturation-cap campaign) are DONE on journal-dev and behavior-neutral. W1.1, W2, W3, W4 and W5 are still proposals and still need author sign-off before any model edit — W2 and W3 change simulation behavior, so all current results shift.
**Companion docs:** `docs/archived/plan_B_cem_outer_loop.md` + `plan_D_cvar_covariance_steering.md` (banked stochastic machinery, relevant to W4 only if an optimization loop is later wanted).

## Why this blocks acceptance

- `system_dynamics.m` applies zero aerodynamic force/moment at deploy speeds of 45–76 m/s (measured from the stored `ballistic_solution` at the sweep's arc-length-uniform deploys); there are no rotor dynamics; commands are instantaneous.
- The actuator is notional (`b=5, d=2, Ω²max=2`, T/W 5.10, normalized units, `d/b ≠ k_MT`). W1.3 has since **characterized** that sensitivity rather than removing it: the cap does move the result, but the dSMC-sat landing fraction is flat at ≈0.607 for every cap ≥ 1 (0.514 at 0.6), so the headline number is on a plateau rather than a cliff. What is still unjustified is the *choice* of 2 — W1.1 (grounding `b`, `d`, `Ω²max` in real thrust-stand data) remains open, and the campaign showed the cap changes which controller wins, so the choice is not cosmetic.
- Every number comes from one launch condition with no dispersions, no wind, no sensor/estimator noise, no CIs.
- Criterion soft spots: paper says all clauses hold "for every t" but the position clause is final-time-only in `ballistic_success.m`; no touchdown (altitude/descent-rate) clause; constants unstudied.

## Workstreams

### W1 — Ground the actuator physically (prerequisite for honest saturation results)
1. Pick a reference vehicle consistent with the 0.8 kg / l=0.2 m airframe (published thrust-stand data for a ~200-size class, or a tube-launched design point à la SQUID). Derive physical `b` (N per rad²/s²), `d`, `Ω²max`, hover fraction; document the mapping to the normalized units (one paragraph + one table in the paper).
2. **DONE (journal-dev `3ec427f`).** `Omega2_max` moved from a `dsmc_constraints.m` source literal to a `constants_struct` bus field (default `2`), read by the allocation, by STEP-12b and by `apply_rotor_clip.m`, and overridden per call via `traj_params.Omega2_max` in every sweep driver. `set_omega2_max.m` and the regexprep hack are gone (`docs/refactor_backlog.md` H6 retired). Regression gate met at cap 2.
2b. **NOT done.** `Omega2_min` is still a source literal (`0.0`) in four files. Promoting it needs a behavior-neutral re-verification run for no behavioral gain, so it was deliberately left alone — pick it up only if a nonzero idle floor is ever wanted (which would be a real physics change, not a refactor).
3. **DONE (journal-dev, 2026-07-23 campaign).** The T/W sensitivity curve became the saturation-cap campaign: `sweep_saturation_campaign.m` plus the `run_cap_grid_stage{1,2}` extensions sweep caps 0.5–100 (T/W ≈ 1.3–255) for dSMC-sat, dSMC-naive and SE(3) (`figs/CAP_01..CAP_05`), and `run_envelope_cap_campaign.m` does the same on the envelope (`figs/ENV_CAP_01`). It delivered more than a curve: the cap **moves which controller wins** on both metrics, and dSMC's ascending-station cliff turned out to be cap-invariant — an architectural wall, not an authority limit. Runbook and findings: `docs/plan_saturation_cap_campaign.md`. The paper write-up (Thread D) is still open.

### W2 — Minimal aero model + wind input
1. Add to `system_dynamics.m`, gated by new bus fields (default off ⇒ bit-exact backward compatibility): flat-plate/quadratic parasitic drag `F_d = −½ρ C_D A ‖v_rel‖ v_rel` with `v_rel = v − R(ψ,θ,φ)ᵀ w_wind`, and (optional, phase 2) a linear rotor-plane H-force. Constants from published quadrotor drag studies for the chosen W1 vehicle; cite them.
2. Wind enters as a constant-plus-gust inertial vector (`w_wind` on the bus; Dryden-lite gust = filtered noise seeded per trial, if wanted later — start with steady wind).
3. Re-run the sweep with drag on / wind 0: quantify the shift from the current no-aero results (expect degraded reachability at the fast 20–40% arc deploys). Then wind sweeps (e.g. 0/5/10 m/s from worst-case azimuth) → reachability-vs-wind curve.
4. Paper text: one subsection "Aerodynamic effects," one figure, honest statement of what remains unmodeled (blade flapping, vortex ring during descent).

### W3 — Rotor dynamics
First-order lag on Ω² between allocation and plant (τ ∈ {0, 20, 40, 60 ms}), implemented as a discrete filter either inside `dsmc_constraints.m`'s post-allocation stage or as a Simulink block before the plant (author's call — the .m route avoids re-wiring four models but technically lags the *command*, not the rotor; the block route is physically cleaner). Report success-vs-τ. Theory note for the paper: the lag is an unmodeled input dynamic; the ISS bound tolerates it as an additional bounded disturbance for small τ — state this, don't overclaim.

### W4 — Dispersion Monte Carlo (evaluation, not optimization)
1. **Design:** disperse per-trial rather than per-sweep to keep cost linear: for each of the 140 grid trials draw `N_mc ≈ 20` dispersion samples ⇒ 2800 trials ≈ 20 sweep-equivalents ≈ 2–5 h warm on 12 workers. Reuses the existing parsim wiring (per-trial `xi`/`xf_ballistic` already flow through `setVariable`).
2. **Dispersion set** (à la Ilg/Rogers/Costello 6-DOF dispersion practice; see 2026-07-09 lit review Theme 6): launch scatter σ(Vo, el, az, spins) at mortar-typical levels; air density ±5%; mass/inertia ±10–20% through the existing `m_uncertain` path (already plumbed and unused in the ballistic case); steady wind from W2. Document the table of σ's with sources.
3. **Statistics:** Wilson 95% CIs on all success fractions; report dispersed reachability alongside the deterministic value (60.7% on the current gains; this plan was drafted against 55.0%); per-clause failure attribution from `results(...).criteria` (which clause kills trials under dispersion).
4. **Estimator noise** (phase 2, optional): zero-mean noise on the controller's state input in the four `*_single` models (attitude/rate noise at tactical-grade IMU levels, position at GPS levels); success-vs-noise-level curve. This addresses the "GPS+IMU+EKF with sufficient precision" assertion at root.tex L236.

### W5 — Criterion rigor (cheap; pure post-processing + text)
1. Fix the paper wording: position clause is terminal, velocity whole-trace, rotation post-grace (root.tex eq:ballistic-success paragraph).
2. Add (or justify omitting) a touchdown clause — final altitude and descent rate — by re-scoring the **saved logs** (they carry full `pos/vel` traces; no resimulation) and reporting how the headline count (now 85/140) changes.
3. Sensitivity table: re-score under ReachTol ∈ {5,10,20} m, ratio bounds ∈ {1.5,2,3}, grace ∈ {0.5,1,2} s. All post-hoc on saved logs ⇒ hours of scripting, no sim time.
4. Solver: one fixed-step (or tightened-tolerance) re-run of a sweep arm to show insensitivity to the "auto/default" integrator choice; one sentence in the paper.

## Order and effort

W5 (0.5–1 d, no sim) → W1 (1–2 d + 1.5 h compute) → W2 (2–4 d + reruns) → W3 (1–2 d) → W4 (3–5 d + overnight compute). Total ≈ 2–3 weeks elapsed. W1.2's regression gate protects everything downstream.

## Verification

- W1.2: bit-identical sweep at Ω²max=2 — **met**. (The regression number is now 85/140 on the retuned gains; 77/140 was the pre-retune value this plan was written against.)
- W2/W3 flags default-off: re-run one sweep arm, assert identity with current logs before enabling anything.
- W4: reproduce the deterministic result as the σ→0 limit of the MC pipeline (smoke test with all dispersions zeroed).
- Every new paper number regenerated by script, no hand-carried values (the Table-4 ↔ log check in the audit is the model).

## Paper deliverables

Revised eq:ballistic-success text + sensitivity table (W5); actuator-grounding table + T/W-vs-reachability figure (W1); aero subsection + wind curve (W2); lag curve (W3); dispersed-reachability results with CIs replacing/alongside Table 4, and a dispersion-table appendix (W4). Together these rewrite §"Saturated Control Results" into a robustness-characterized result.

## Decision points needing author input

(a) Reference vehicle/actuator data source for W1. (b) Whether rotor lag lives in the .m or as a Simulink block (W3). (c) Dispersion σ table values (W4.2). (d) Whether a touchdown clause becomes part of the headline criterion or a reported variant (W5.2 — changes the headline number).
