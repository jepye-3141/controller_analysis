# Judge 1 — Compute & Integration Feasibility

## Summary ranking

| Rank | Plan | Verdict |
|------|------|---------|
| 1 | Plan B (CEM) | Pure-MATLAB, zero new deps, slots into `sweep_landing_centroid.m` unchanged. Most tractable today. |
| 2 | Plan D Track 1 (CVaR) | Trivial bolt-on; ~5 dev-days; only Track 1 is feasible — Track 2 needs MOSEK + a multi-month research project. |
| 3 | Plan A (BO/GP) | Realistic wall budget; pyenv/BoTorch is the main risk. MATLAB-only EI-plus fallback is publishable. |
| 4 | Plan F (NN surrogate) | `fitrnet` is in-toolbox and cheap; but is a parasitic surrogate gated on N≥100 sweeps from Plans A/B first. |
| 5 | Plan E (Multi-fidelity) | Honest self-assessment that savings are ~2 h on 4.3 h baseline; layered pyenv risk on top of Plan A. |
| 6 | Plan C (PCE) | Plan acknowledges its own gating step will likely fail. Wrong cost-center; should be parked. |

## Per-plan critique

### Plan A — BO/GP

- **Compute estimate vs reality**: 30 LHS + 100 KG = 130 sweeps × 2 min = 4.3 h (line 65) is consistent with the 2-min/sweep baseline. Assumes the 4-worker pool stays saturated by internal parsim parallelism, which `sweep_landing_centroid.m:183-189` actually does. No GPU assumed. Realistic.
- **Toolchain risk**: Medium. Plan flags `pyenv` + BoTorch as the schedule risk (lines 31, 79) and provides a defensible MATLAB-only fallback via stock `bayesopt` with `expected-improvement-plus` (line 33), publishable on its own.
- **Integration**: Excellent. Wrapper consumes `sweep_landing_centroid.m` unchanged except a one-line patch to surface `ctrl_trial` (line 41). No Simulink edits, no `parsim` rewiring (line 46).
- **Hidden refactors**: None of substance. Option (i) — bumping inner-MC for ξ-sampling (line 45) — is correctly flagged as multiplying wall cost, not a pipeline refactor.

### Plan B — CEM

- **Compute estimate vs reality**: `N_pop=40, gens=5 = 200 evals × 2 min = 6.7 h` (lines 56, 61) is honest and conservative. The 16.7 h ceiling at `N_pop=100×5` is correctly labelled an upper bound. No outer-loop parallelism beyond the existing 4 workers.
- **Toolchain risk**: Low. `mvnrnd` + sort + refit, ~100 lines (line 7). Statistics & ML Toolbox is already required. No Python, no SDP, no GPU. Bregman-centroid extension is conditional, not blocking.
- **Integration**: Excellent. Same one-line `sweep_landing_centroid.m` patch as Plan A (line 42); `parsim` untouched (line 44). LHS = "what CEM does in iteration 0 with maximally diffuse Σ" (line 43), so existing `centroid_lookup_log.mat` is reusable as gen-0 elites.
- **Hidden refactors**: None. Outer-population parallelism would need `M × 4` workers and is correctly flagged as off the critical path.

### Plan C — PCE

- **Compute estimate vs reality**: The plan's own step-1 gate (line 68) admits PCE will likely shave <5% of wall time. Cost accounting on line 59 is devastating: `eom2` is ~50 ms vs ~30 s of dSMC per sweep (0.04% saving); even at N_ξ=16, eom2 is 2.6%. The report's "10–100× speedup" (line 83) is reframed as conditional on abandoning the dSMC heatmap entirely.
- **Toolchain risk**: Medium. UQLab v2 is BSD-3 and pure-MATLAB (line 44), so the dependency is benign. chaospy/PoCET/hand-rolled alternatives noted but not recommended.
- **Integration**: Awkward. Requires a new `eom2_for_pce.m` wrapper, Magnus/drag perturbation through `env`, and only delivers value if Plan A is also extended with a ξ-MC inner loop the project does not have (lines 54-58). A surrogate looking for a bottleneck.
- **Hidden refactors**: The whole plan is conditional on a refactor that hasn't happened. Line 60 honestly admits the analytic-moments path requires "giving up the dSMC reachability heatmap... which contradicts the project's whole motivation." Plan's self-parking advice is sound.

### Plan D — CVaR + cov-steering

- **Compute estimate vs reality**: Track 1 = effectively zero marginal cost (`prctile`/`bootci` over 84 numbers, line 115). Track 2 claims sub-minute SDP solves at horizon N=3000 (line 84) — plausible for LTI-MOSEK; plan admits LTV+chance constraints are 1–10× slower and that LTV linearization itself is dominant (line 116).
- **Toolchain risk**: Track 1 = low (already-required toolbox). Track 2 = high: YALMIP (free) + MOSEK (commercial; academic license) is a new dep stack, plus 4–8 dev-weeks of SDP/LMI tooling (line 60). MOSEK is a hard wall on this lab setup.
- **Integration**: Track 1 = one-line patch to `sweep_landing_centroid.m:233` for `out.miss_per_cell` (line 74) and a 30-line wrapper. Track 2 = a parallel pipeline that doesn't share code with the existing one (line 94).
- **Hidden refactors**: Plan D flags the most important assumption error in the original report: the existing 84-cell sweep is *deterministic* in ξ, so "CVaR over the tail" doesn't yet exist in the pipeline (lines 36-42). Cleanest self-correction across the six plans.

### Plan E — Multi-fidelity

- **Compute estimate vs reality**: Plan is honest about marginal value: ~2 h saved on Plan A's 4.3 h baseline (lines 64, 68) for ~11 dev-days plus pyenv complexity (line 87). Cheap-fidelity wall of "≈10 s/eval (single Simulink build dominates)" is realistic — the dominant cost is Simulink compilation, not trial count.
- **Toolchain risk**: Compounded. Layered on top of Plan A's pyenv, adds SMT 2.0 + MF-MES PyTorch code (lines 30, 86). Plan correctly gates Plan E on Plan A's pyenv being already operational.
- **Integration**: Cleaner than expected. `sweep_landing_centroid_lo.m` is a thin wrapper with `n_deploy=n_ct=n_nb=1` (line 36); reuses parsim plumbing, no `static_vars` change. 20-LHS history is HF-only (line 40), so separate cheap-fidelity LHS run needed.
- **Hidden refactors**: The line-70 sketch — that 200 LF samples without HF anchors is "not a useful surrogate unless ρ_S ≥ 0.7" — punctures naive "trade fidelity for samples" intuition. Day-2 ρ_S gate is the right discipline.

### Plan F — NN surrogate

- **Compute estimate vs reality**: Training a 64×64 MLP on N≤500 with `fitrnet` is seconds on CPU (line 52). Data acquisition is correctly identified as the bottleneck (3.3 h for N=100, line 51) — Plan F doesn't reduce per-`p_target` cost, it amortizes existing sweep cost across many `p_target` re-runs.
- **Toolchain risk**: Low if MATLAB-native. `fitrnet` is in Statistics & ML Toolbox (line 24), already required. PyTorch fallback only if heteroscedastic NLL is needed. No GPU mandated.
- **Integration**: Excellent. `nn_predict.m`/`nn_train.m` are pure cached-data pipelines; never invokes Simulink (line 35). Drop-in replacement for `bo_objective`'s expensive call.
- **Hidden refactors**: One explicit one: tagging every cached row with the git commit of `dsmc_constraints.m` (line 71) turns surrogate drift into a noisy retraining cost. The N≥100 gate (line 40) is correctly imposed; without it, a 5000-parameter MLP on N=20 is fitting noise.

## Cross-cutting observations

1. **The 2-min/sweep baseline is the binding constraint, not the optimizer math.** Plans A, B, C, E all evaluate the same 4-D objective via the same `sweep_landing_centroid.m`. Plan C's eom2-is-0.04%-of-wall gate and Plan E's Simulink-compile-dominates gate point at the same truth: any plan that doesn't shrink or share dSMC trials cannot change the budget. Plan F escapes by amortizing across many `p_target` re-runs.

2. **`pyenv` + BoTorch/SMT is the shared dependency that decides plan composition.** Plans A, E recommend it; Plan F can adopt it; Plan B and Plan D Track 1 explicitly avoid it. If pyenv is brittle, Plans A/E degrade to MATLAB-only fallbacks (EI-plus / hand-rolled Le Gratiet) and Plan F gains relative attractiveness. MOSEK (Plan D Track 2) is a different risk tier — commercial license on top of YALMIP-LMI tooling, not just environment management.

3. **Inner-MC noise is the most underestimated cost.** All four optimizer plans (A, B, D, F) evaluate noisy `J(θ)` from 84 trials with `σ_Q ≈ 0.05`. Plan A handles it with robust-KG; Plan B's vanilla CEM does not (line 76); Plan D's CVaR at α=0.1 has ~30% std error on 9-of-84 elites (line 28); Plan F can't drive RMSE below the MC floor (line 70). All four plans' budgets assume 84 trials is enough; if not, all four double.

## Recommendation

**Plan B (CEM)** and **Plan A with the MATLAB-only EI-plus fallback** are tractable today with zero new infrastructure beyond a one-line patch to `sweep_landing_centroid.m`. **Plan D Track 1 (CVaR)** is a 5-day bolt-on after A or B is running. **Plans E and F** stack on top of Plan A only once `pyenv` (or hand-rolled co-Kriging) is established, and both self-assess as marginal-value follow-ups. **Plan C (PCE)** and **Plan D Track 2** require either a refactor that contradicts the project's framing or commercial-licensed SDP solvers plus months of research effort; both correctly recommend their own deferral.
