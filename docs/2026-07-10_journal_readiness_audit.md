# Journal-Readiness Audit — ATLIS Sims + SciTech Paper (2026-07-10)

**Target bar:** AIAA JGCD (or a comparable intelligent-systems journal, e.g. JAIS). **Scope:** full read of `docs/scitech-paper/root.tex` (1539 lines, rebuilt 2026-07-08), the program docs, and the active code. **Status:** audit only — no source files were changed.

## 1. Verified current state (facts checked against logs/code, not from memory)

- The paper **is current** with the post-`h0`-fix simulation state. Table 4 was verified directly against `logs/trajectory_optimization_log_sat_on.mat` (2026-07-08): reachability 0.550 (= 77/140), peak ratio 0.981, half-radius 257.60 m, centroid (564.39, −84.80). Abstract (55.0% vs 0%) matches. `root.pdf` built 2026-07-08 15:12. The regeneration demanded by the 2026-07-07 `eom2` fix has therefore **already happened** for the sweep, envelope, and paper. (CLAUDE.md still describes 59.3% as canonical — housekeeping item H1.)
- Post-fix envelope masks (verified in `logs/analysis_log.mat`, 2026-07-08): 20 samples; LQR 4/20, PID 9/20, cSMC 10/20, dSMC-unconstrained 20/20, dSMC-saturated 7/20. The paper's "dSMC is the only controller that recovers from every sampled state" refers to the unconstrained arm and holds.
- The **targeting pipeline logs remain doubly stale**: `logs/centroid_lookup_log.mat` (2026-05-09) and `logs/surrogate_optimize_log.mat` (2026-05-12) predate both the `ballistic_success` criterion and the 2026-07-07 ballistics fix.
- Deploy speeds at the sweep's eight arc-length-uniform deploy points (20–80%): **45–76 m/s** (interpolated from the stored `ballistic_solution` exactly as `sweep_landing_centroid.m:86-91` does); apogee speed ≈ 44 m/s, near-launch speeds ~100 m/s.

## 2. Gap inventory (ranked by acceptance impact)

### G1 — Simulation evidence is one deterministic scenario on a low-fidelity plant *(top 3 → Plan J1)*
- `system_dynamics.m` applies **zero aerodynamic force or moment** to the airframe, at deploy speeds of 45–76 m/s. Wind cannot even be modeled until an aero model exists. No rotor/ESC dynamics — commands apply instantaneously.
- The actuator is **notional**: `b=5, d=2, Ω²∈[0,2]` in normalized units, T/W = 5.10, and the paper itself flags these as "effective values … not derived from the physical mixing of Eq. (1)" (`d/b = 0.4` vs `k_MT = 0.1`). Per project memory, `Omega2_max = 2` sits just past the empirical failure breakpoint — i.e. the headline result is **sensitive to an unjustified constant**.
- All results derive from **one launch condition** (Vo=100, el=45°, az=15°, fixed spins) with no launch dispersion, no atmosphere/wind variation, no sensor or estimator noise (full-state feedback; "EKF with sufficient precision" is asserted, not modeled), and no parameter uncertainty in the ballistic case (the `m_uncertain` machinery exists but is only used in the unpublished STEP swarm study). No confidence intervals anywhere.
- Success criterion soft spots: the paper (eq:ballistic-success text) says all three clauses hold "for every t", but `ballistic_success.m` enforces position at **final time only**; there is **no touchdown clause** (no altitude or descent-rate condition — a "landing" is a 60-s final horizontal miss ≤ 10 m); the constants (10 m, ratio 2, ratio 2, 1 s grace) have no sensitivity study. Solver is "auto, default settings" with no tolerance check.

### G2 — The headline comparison is vulnerable to a strawman critique *(top 3 → Plan J2)*
- The only saturated baseline is the naive per-rotor clip (0/140). The handler bundles **two mechanisms** — priority-weighted allocation and the auxiliary anti-windup state — with no ablation attributing the 0%→55% gain. No implemented published competitor (Faessler's iterative mixer is compared only in prose).
- The saturated arm recovers only **7/20** envelope samples (35%) vs 20/20 unconstrained; the paper does not foreground this.
- **Confirmed baseline defect (new finding):** the cSMC pitch channel uses `Jzz` where the Lyapunov cancellation requires `Jyy` — in the paper (eq:smc3) *and* in the implementation (`zoh_smc_controller.slx` chart: `uhat_My = (-cY*Jzz/cos(phi))*(...)`). With `Jzz=1.5e-3 ≠ Jyy=1.8e-3`, eq:sdot_final's exact reduction is false; the residual `(1−Jzz/Jyy)·c_My·(ω_y cφ − ω_z sφ)` ≈ (1/6)·8·(rates ~π during tumble) is not negligible and erodes the claimed certificate of a *baseline the paper defeats*. The paper's cSMC equations also carry `v_y sθ sφ` where the EOM's ż has `v_y cθ sφ`, and a sign difference on the `c_T` term vs the block. The cSMC section never received the verification-campaign treatment the saturation section got.
- Baseline tuning provenance (LQR Q/R, PID, cSMC gains: "manual tuning") is undocumented — reviewers will ask whether baselines were tuned for the ballistic task at all.
- Intro (L86) promises evaluation "by energy efficiency and settling time"; the results deliver neither (the material exists as unreleased figs 1–38 in `analysis.m`).

### G3 — No journal delta; the natural one is sitting in the repo, stale *(top 3 → Plan J3)*
- A journal version of a conference paper needs substantial new material. The obvious candidate — the inverse launch-parameter targeting problem — is explicitly "future work" in the paper while a working pipeline exists (`centroid_lookup_table.m` → 4 GP surrogates → 8-start SQP).
- But: logs doubly stale (pre-criterion, pre-ballistics-fix); the LHS box's `p ∈ [−2,2]` excludes the operating spin −8.379; `gp_reach` is fit but unused; the `half_radius=NaN` clamp distorts 52/59 training samples *(corrected 2026-07-10: the saved log holds N=59, not the script-default 20, and the NaN share is 88%, not 12/20)*; λ=50 fixed with no Pareto view; deploy-at-apogee never relaxed. The 2026-07-09 strategies doc defines the roadmap (step zero + Strategies A/E/C); step zero is now staged — see `plan_J3_runbook.md`.

### G4 — Theory tightening *(secondary)*
The ISS development is genuinely strong post-deconfliction (exact ν/γ, envelope certificate for M_D). Remaining reviewer pokes: the reaching identity Δs = −E s̃ + D Δu presumes exact equivalent-control inversion — no robustness term for the two-step-extrapolated-model vs true-plant mismatch; the ±2 s-clip's interaction with the ISS argument is not carried through the saturated analysis; the M_D envelope is checked empirically only on successful trials (selection bias); "saturation not permanent" is assumed, with stage-4 = full motor cutoff (F1's preferred mid-band re-centering remedy unimplemented — picked up in Plan J2 W4).

### G5 — Citation and related-work closure *(secondary)*
Xiong2016 remains unverified (paywalled) while the paper *prints Xiong's allegedly flawed control laws* and rests its "corrected derivation" contribution on them — must be pulled via UT library before journal submission. 11 uncited bib entries (`Arnold2019 … copilot`) need cite-or-remove. Related work (~19 cited refs) is thin for a journal; the 2026-07-09 review supplies ~20 candidate additions for the targeting section alone.

### H — Housekeeping
H1: CLAUDE.md still calls 59.3% canonical and lists the post-fix regeneration as pending (done 2026-07-08). H2: paper criterion wording ("for every t") vs code (final-time position). H3: `replot_*` scripts point at pre-criterion logs. H4: stale May-21 `TO_06/07_sat_off` EPS copies in `docs/scitech-paper/figs/`. H5: `root_with_alts.tex` ready to archive. H6: `set_omega2_max.m` mutates source on disk (superseded if J1-W1 lands the bus-carried bound).

## 3. Top three, and why these three

1. **Plan J1 — Physical fidelity + uncertainty quantification.** The most likely outright rejection at JGCD is evidentiary: every headline number is one deterministic run of one scenario on a plant with no aerodynamics, no actuator dynamics, and admittedly notional limits. J1 converts the claim "the handler restores reachability" into a claim that survives dispersions, wind, noise, and an honestly parameterized vehicle. (Fidelity must precede UQ: without an aero model there is no wind to disperse.)
2. **Plan J2 — Baseline strength + mechanism attribution.** The 0%-strawman critique and the two-mechanisms-one-knob conflation are the most likely *major-revision* demands; the cSMC `Jzz/Jyy` defect is a correctness liability in a "Lyapunov-certified" baseline. Cheapest plan (each new arm ≈ one 6–16 min sweep) with the highest rigor-per-hour.
3. **Plan J3 — Inverse targeting as the journal's new contribution.** Without a delta, the journal version risks desk rejection as a republished conference paper; the targeting pipeline is the delta the repo has already half-built, and it upgrades the paper from "a controller" to "a deployment-planning capability" (JGCD/JAIS-friendly). Requires the step-zero regeneration regardless.

G4/G5 items ride along: cSMC math repair is in J2; criterion rigor in J1; Xiong2016 retrieval and bib closure are author actions listed in §2 (no plan needed).

## 4. Suggested sequencing

J2 first (days, mostly compute-light; fixes correctness debts), J1 phases 1–2 in parallel with J3's step-zero overnight runs, then J1 UQ and J3 optimization on the regenerated foundation. Detailed plans: `plan_J1_fidelity_uq.md`, `plan_J2_baseline_attribution.md`, `plan_J3_targeting_delta.md`.
