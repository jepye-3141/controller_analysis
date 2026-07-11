# Judge 3 — Time-to-MVP & Development Risk

## Summary ranking (by recommended sequencing)

| Order | Plan | When | MVP gate |
|-------|------|------|----------|
| 1 | Plan B (CEM) | Week 1, days 3–5 | A recommended `(Vo, el, az, p)` from a 200-eval CEM run (~6.7 h sim wall) using only MATLAB toolboxes already installed. |
| 2 | Plan A (BO/GP) | Week 1–3 | EI-plus run on stock `bayesopt` ready by day 7; rKG via BoTorch by day 13. Recommended launch + calibrated GP for Pareto/sensitivity post-processing. |
| 3 | Plan D Track-1 (CVaR) | Week 4 (after A or B exists) | One-line `out.miss_per_cell` patch + `cvar_objective.m` re-runs A or B with tail-aware functional. ~5 dev-days. |
| 4 | Plan E (multi-fidelity) | Week 4–5, only if multi-mission | Validate `ρ_S(J_lo, J_hi) ≥ 0.6` on a 6×6 grid before committing. Saves ~2 h per re-run. |
| 5 | Plan F (NN surrogate) | Hard gate at N ≥ 100 cached sweeps | Sub-millisecond `nn_predict(θ)` for warm-starts and `p_target` re-optimization. |
| 6 | Plan C (PCE) | Park; revisit only if dSMC itself becomes a surrogate | Sobol' indices justifying ξ-treatment; ~0% wall savings under current bottleneck profile. |
| 7 | Plan D Track-2 (cov-steering) | Drop indefinitely | Multi-month SDP + LTV linearization of a sliding-mode law; no requirement justifies this. |

## Per-plan critique

### Plan A — BO/GP

- **Days to first result.** 14 dev-days claimed (line 57); EI-plus result by day 7 (line 52), BoTorch+rKG by day 13 (line 54). Realistic: 16–18 days for full rKG, 9–10 for EI-plus fallback. Sim wall (4.5 h) is honest.
- **Critical blocker.** `pyenv`/PyTorch+BoTorch coexisting with MATLAB (line 79). rKG benefit is real but not critical — EI-plus still ships if Python breaks.
- **Slip risk.** Low. Day-7 EI-plus is wrapping `bayesopt` around `sweep_landing_centroid`. Hidden cost: MC noise per evaluation (`σ_Q ≈ 0.05`, line 81) could double iteration count to 200+ (9 h sim wall) — annoying not fatal.
- **Failure-mode value.** High. Even without rKG: recommended launch + posterior + GP length-scale sensitivity. `bo_log.mat` reusable by Plans D, E, F.

### Plan B — CEM

- **Days to first result.** 9 dev-days claimed (line 56), 11 standalone. Realistic: 7–10 days. Sim wall 6.7 h vs Plan A's 4.5 h.
- **Critical blocker.** Outer-MC noise drowning elite selection (line 76). With 84 inner trials and `σ_J ≈ 0.05`, two `θ` within ~0.1 are statistically indistinguishable; covariance refit is partially noise-driven. Covariance-floor smoothing is standard but needs tuning.
- **Slip risk.** Very low. ~100 lines of `mvnrnd` + sort + refit (line 86). Zero external dependency. Bregman-centroid extension is conditional.
- **Failure-mode value.** High. Even if CEM stalls, ~200 sweeps generated feed Plan A's GP, Plan F's NN, and Plan E's HF anchors.

### Plan C — PCE

- **Days to first result.** Plan C is honest: line 9 says "should be parked, not built" until Plan A/B grow a true ξ-MC. Line 75: "if step 1 fails (likely outcome), total is ~2 dev-days plus the parking note."
- **Critical blocker.** Wrong cost-center (line 95). PCE accelerates `eom2` (~50 ms, 0.04% of wall), not dSMC (~30 s/eval/worker). The report's "10–100×" claim does not survive Plan C's own profiling.
- **Slip risk.** Irrelevant — the plan recommends parking itself.
- **Failure-mode value.** Medium as a Sobol' analysis: ξ-sensitivity decomposition justifies whether Plan A can hold `(w_z0, w_y0)` deterministic. ~3 days; the rest is overkill.

### Plan D — CVaR + cov-steering

- **Days to first result.** Track 1: 5 dev-days *after* Plan A or B (line 98). One-line patch + ~30 line wrapper. Honest. Track 2: 4–8 dev-weeks just to install YALMIP/MOSEK and linearize a sliding-mode controller (line 60); Plan D itself recommends dropping it (line 137).
- **Critical blocker.** Track 1: the "tail" doesn't exist yet — current code is deterministic in ξ, so CVaR over 84 cells is a Conditional-Worst-Cell statistic, not standard CVaR_α (line 41). Sample-CVaR variance at α=0.1, N=84 is ~30% (line 27). Track 2: no published precedent for covariance steering on a sliding-mode controller (line 58).
- **Slip risk.** Track 1: very low. Track 2: catastrophic — easily 6 months for an unusable result.
- **Failure-mode value.** Track 1: high (bootstrap CIs alone diagnose Plan A/B). Track 2: low.

### Plan E — Multi-fidelity

- **Days to first result.** 11 dev-days claimed (line 53). Realistic: 12–15 — day-3 rank-correlation validation often iterates. Hard gate `ρ_S ≥ 0.6` (line 47).
- **Critical blocker.** Cheap-fidelity rank correlation. The reduced-sweep `n_deploy=n_ct=n_nb=1` cheap fidelity (line 25) is sound in concept but unmeasured. If dSMC reachability is dominated by 8-deploy diversity rather than per-deploy success, `ρ_S < 0.4` and Plan E aborts.
- **Slip risk.** Moderate. Stacks SMT/Python on Plan A's BoTorch — two Python dependencies.
- **Failure-mode value.** Medium. Even an aborted Plan E gifts a 36-point validation grid to Plans A and F. Payback (line 68): "≈ 2 h on Plan A and ≈ 1 h on Plan B" — only at 5+ missions.

### Plan F — NN surrogate

- **Days to first result.** Hard gate N ≥ 100 cached sweeps (line 40). Post-gate: ~7 dev-days for MVP (lines 42–47).
- **Critical blocker.** Insufficient data + dSMC saturation cliff (lines 67–68). N=20 is hopelessly under-determined for a 64×64 MLP (~5000 parameters). Plan F flags GP > MLP for N < 200 with a sharp cliff. MEMORY.md confirms the cliff (Omega2_max breakpoint between 2–3).
- **Slip risk.** High if launched prematurely; low if the gate is respected.
- **Failure-mode value.** Medium. Sub-ms `nn_predict` is useful for `λ`/`p_target` sweeps. Cliff classifier (line 68 mitigation a) is a useful diagnostic.

## Recommended sequence

1. **Phase 1 (week 1, days 1–7): Plan B v1.** Implement `cem_objective.m` (shared with Plan A) + `cem_run.m` against the simulator. Gate: a recommended `θ*` and `cem_log.mat` with ≥120 sweeps. Pure MATLAB, no Python toolchain risk. **This is the MVP.**
2. **Phase 2 (week 2–3): Plan A on top of Plan B's data.** Reuse `cem_log.mat` as initial design for the GP (saves ~1 h sim wall). Ship EI-plus by day 7, BoTorch+rKG by day 13. Gate: side-by-side comparison of θ*_BO and θ*_CEM on the same `(p_target, λ)`. If they agree to within 5% in normalized box coordinates, the MVP is *cross-validated* — the strongest possible result.
3. **Phase 3 (week 4): Plan D Track-1 only if a worst-case requirement is articulated.** 5 dev-days, runs on top of A/B's cached `bo_log.mat`/`cem_log.mat` augmented with `miss_per_cell`.
4. **Phase 4 (week 4–5): Plan E only if multi-mission re-runs are confirmed.** Gate on Plan E day-2 Spearman validation. If `ρ_S < 0.6`, abort with a 36-point dataset gift to Plan F.
5. **Phase 5 (after N ≥ 200 cached sweeps): Plan F.** Train MLP ensemble for `nn_predict` warm-starts and multi-`p_target` exploration.

## What gets pruned

- **Plan C (PCE)** — *parked indefinitely.* Plan C diagnoses its own irrelevance: `eom2` is 0.04% of wall, so PCE saves ~0% on the dominant kernel. Worth considering only as a 2–3 day Sobol' index analysis, never as a full pipeline replacement. Keep the parking note.
- **Plan D Track-2 (covariance steering)** — *dropped.* Sliding-mode controllers have no published covariance-steering precedent (Plan D line 58); the user's offline launch problem has no certifiable-bound deliverable to justify 4–8 dev-weeks of SDP work. Plan D's own §"When to pursue Plan D" recommends dropping it.
- **Plan F before N ≥ 100** — *blocked.* Training a 5000-parameter MLP on 20 LHS rows is wishful thinking. The plan itself enforces this gate; respect it.

## Recommendation

Run **Plan B first** (lowest dependency risk, MATLAB-only, ~7 dev-days to a recommended launch parameter set), then **Plan A** seeded with Plan B's cache. This sequence converts what either plan presents as 9–14 standalone days into a ~14-day combined effort that produces *two* independent estimates of the optimum — the best possible MVP. Plans C, D-Track-2, E (premature), and F (premature) are all explicitly conditional follow-ups and should not be built on speculation.
