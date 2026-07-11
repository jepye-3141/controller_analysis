# Plan A — Bayesian Optimization with GP/Kriging Surrogate over the 4-D Launch Space

> Source: Recommendation 1 of [`docs/ballistic_targeting_optimization.md`](archived/ballistic_targeting_optimization.md) lines 154–164.

## Overview

Treat launch-parameter selection as the first-stage decision in a two-stage stochastic program (report lines 4, 83–87) and optimize an expectation `J(launch) = E[Q] − λ E[U]` over the 4-D space `(Vo, el, az, p)` using a Gaussian-process surrogate updated with a Knowledge-Gradient (KG) acquisition. Each acquisition step calls the existing `sweep_landing_centroid.m` (≈2 min, ≈84 inner trials) once or twice, so a 100-iteration BO loop is ≈3–6 hours wall on the existing 4-worker pool. The intent is to produce a single recommended launch plus a calibrated GP that can also be queried for sensitivity and Pareto trade-offs.

## Mathematical formulation

- Decision: `launch = (Vo, el, az, p) ∈ R^4`. The other ranges in `centroid_lookup_table.m` (`w_z0, w_y0`) are uncertainty inputs ξ, not decision variables.
- Per-launch objective decomposition (report lines 117–124):
  - `Q(launch) = E_ξ[ ∫ R(x_t | x_d(launch, ξ)) p_target(x_t) dx_t ]` — target-weighted reachability.
  - `U(launch) = E_ξ[ ∫ ‖u(t; x_d)‖² dt ]` — expected control effort.
  - `J(launch) = Q(launch) − λ U(launch)`.
- Mapping to existing outputs of `sweep_landing_centroid.m`:
  - `R(x_t | x_d)` is implicitly built inside the function as `landing_ratio` (lines 254–256) and continuously interpolated as `Rg = F_heat(Xg, Yg)` (line 268). Multiplying `Rg` elementwise by a discrete `p_target(Xg, Yg)` and summing gives `Q` directly. `p_target` becomes a new argument to a wrapper.
  - `U(launch)` is computable from the existing per-trial `ctrl_trial` field (line 211) — currently consumed only for the TO_06 figure. The wrapper must integrate `vecnorm(ctrl_trial,2,2).^2` over time per stable trial and average.
  - `reachability_pct` and `radial_profile.half_radius` already returned are scalar summaries that can be logged but should not replace `Q` (they ignore `p_target`).
- Surrogate: separable Matérn-5/2 GP on `(Vo, el, az, p)`, automatic relevance determination, log-transformed `Vo`/`p` to flatten the response. MC noise is treated as heteroscedastic by storing per-evaluation sample variance from the 84 inner trials (Bernoulli-style for `Q`, sample-variance for `U`).
- Acquisition: **robust Knowledge Gradient** (Pearce & Le, *Engineering Optimization* 56(1) 2024, Tandfonline DOI 10.1080/0305215X.2022.2145604) — explicitly designed for the case where the objective is itself an expectation over disturbances. Falls back to one-shot qKG (BoTorch) if the wind/aero noise is collapsed into the inner MC.
- Batch / MC noise model: each `sweep_landing_centroid` call already averages 84 inner trials; report it to the GP as a single `(launch_i, μ_i, σ_i²)` triple. No need for batch-q acquisition unless a second worker pool is added.

## Required infrastructure

- **New MATLAB code**:
  - `bo_objective.m` — wraps `sweep_landing_centroid` to return `(Q, U, σ_Q², σ_U²)`. Adds `p_target` argument (default = Gaussian centred on a user-provided point with a chosen σ; falls back to the existing power-weighted centroid logic if `p_target` is omitted).
  - `bo_run.m` — driver that owns the BO loop: initial LHS via existing `lhsdesign`, GP fit, acquisition optimisation, evaluation, and persistence to `bo_log.mat`.
  - `bo_validate.m` — replays held-out launch points with high-MC ground-truth (e.g., 5× the inner sweep with finer grid) and reports surrogate RMSE.
- **External dependencies**:
  - **Recommended path: BoTorch via MATLAB `pyenv`**. MATLAB's `bayesopt` (Statistics and Machine Learning Toolbox) supports only `expected-improvement{,-plus,-per-second{,-plus}}`, `probability-of-improvement`, `lower-confidence-bound`, and the documentation explicitly forbids custom acquisition functions ([`bayesopt` docs](https://www.mathworks.com/help/stats/bayesopt.html); MATLAB Answers thread 391772). KG and robust-KG therefore require Python. BoTorch ships `qKnowledgeGradient` ([botorch.org/docs/tutorials/one_shot_kg](https://botorch.org/docs/tutorials/one_shot_kg/)). Out-of-process pyenv mode avoids MKL/OpenMP collisions with MATLAB. Requires Python ≥3.11, PyTorch ≥2.2, GPyTorch ≥1.15.
  - Fallback: hand-implement KG. Frazier 2018 §4.4 gives the math; Ungredda–Pearce–Branke (arXiv 2209.15367) gives the practical one-shot Hybrid KG recipe. Estimated 1–2 dev-weeks; not recommended unless air-gap requirements forbid Python.
  - Acceptable shortcut: stock MATLAB `bayesopt` with `expected-improvement-plus`. Works out of the box, but the Pearce–Le paper (Tandfonline 2024) shows EI underperforms rKG by a wide margin when the objective is a noisy expectation — exactly this case. Use only as a sanity check.
  - Pareto-front extension only: SEGOMOE (ONERA, Bartoli et al. arXiv 2504.09930) Python package, or BoTorch's `qNEHVI`. Not on the day-1 critical path.
- **New data products**:
  - `bo_log.mat` — append-only history `(launch_i, Q_i, U_i, σ²_i, J_i)`, the fitted GP at the end, plus diagnostics (acquisition values, posterior at recommended optimum).
  - Optional `target_distributions/` — JSON or .mat files defining the operationally relevant `p_target` instances so different missions are reproducible.

## Integration with existing codebase

- `sweep_landing_centroid.m` is reused **unchanged**. The wrapper consumes `out.radial_profile`, `out.p_centroid`, and (extended) the per-trial `ctrl_trial` data, computing `Q` and `U` from them. Only patch needed inside the function: stop discarding `ctrl_trial` after the TO_06 figure — return it (or a reduced-form integral) in `out`.
- `centroid_lookup_table.m` is **replaced** by `bo_run.m`. The current `lookup` struct array becomes a `bo_history` table whose schema is:
  - columns: `iter, Vo, el, az, p, w_z0, w_y0, Q, U, sigma_Q, sigma_U, J, acq_value, source ∈ {LHS, KG, validation}`.
  - the static `params.alpha_0/beta_0/x_0/y_0/z_0/t_max` block stays as a header struct.
- Nuisance noise inputs `(w_z0, w_y0)` are **not** decision variables in BO. Three options: (i) sample them per evaluation via LHS over their range, growing inner-MC count from 84 to ≈84·N_ξ — multiplies wall cost; (ii) hold them at deterministic worst-case values for screening; (iii) add a second BO inner-loop level (robust-KG handles this analytically for Gaussian disturbances per Pearce–Le 2024). Recommend (ii) for the first 30 LHS samples, then switch to (i) with N_ξ=4 once the GP is roughed in.
- **Simulink touch-points: none.** `discrete_smc_swarm_single.slx` is invoked only via the existing `sweep_landing_centroid.m` parsim wiring. No `static_vars` change needed (CLAUDE.md "parsim wiring" caution).

## Implementation roadmap

1. **Days 1–2: surrogate-readiness audit.** Open the existing `centroid_lookup_log.mat` (N=20 LHS) and fit a GP in MATLAB on the 4-D slice `(Vo, el, az, p)`. Verify `reachability_pct` is smooth enough for Matérn-5/2 (variance explained > 0.6 with diffuse priors). Output: `surrogate_audit.png` + a 1-page note. **Verifiable: GP cross-val R² printed.**
2. **Days 3–5: `bo_objective.m`.** Patch `sweep_landing_centroid.m` to return per-trial `ctrl_trial` integrals. Add `Q` (with `p_target` argument) and `U` computation. Sanity-check on three launch points that `Q` ≤ `reachability_pct` for Gaussian `p_target` covering the centroid. **Verifiable: unit-test asserting `Q(p_target=uniform) == reachability_pct`.**
3. **Days 6–7: `bo_run.m` skeleton.** LHS init (30 points), MATLAB `bayesopt` loop with EI-plus, save `bo_log.mat`. Confirm parity with `centroid_lookup_table.m` runtime (≈60 min for 30 LHS). **Verifiable: a recommended optimum within the existing LHS range.**
4. **Days 8–11: BoTorch+rKG path.** Set up `pyenv` out-of-process, port the GP to GPyTorch (or use `botorch.models.SingleTaskGP`), wire `qKnowledgeGradient` per the BoTorch one-shot tutorial. Run 30 LHS + 70 KG iterations against a synthetic 4-D quadratic-in-log-space test (deterministic) to validate the loop end-to-end before paying simulation cost. **Verifiable: KG loop converges to known optimum within 50 evaluations.**
5. **Days 12–13: full BO run.** 30 LHS + 100 KG = 130 sweeps × 2 min ≈ 4.3 h. Save `bo_log.mat` + recommended launch + held-out validation. **Verifiable: acquisition value below threshold; recommended `J*` documented.**
6. **Day 14: documentation.** Update `docs/README.md` with the BO entry-point and the new `bo_log.mat` schema.

Total: ≈ 14 dev-days (one engineer). The single biggest schedule risk is `pyenv`/PyTorch setup; budget a half-day buffer.

## Computational budget

- **Single sweep**: 84 inner trials in 2 min wall on 4 workers (CLAUDE.md, `sweep_landing_centroid.m` line 188).
- **Initial design**: 30 LHS points × 2 min = 60 min.
- **BO iterations**: 100 KG steps × 2 min = 200 min ≈ 3.3 h.
- **Validation**: 5 held-out points × 4 min (high-MC ground truth, ≈5× inner count) = 20 min.
- **Total offline budget**: ≈ 4.5 h wall for one `(p_target, λ)` combination. The Frazier 2018 tutorial notes BO is "best-suited for continuous domains of less than 20 dimensions" with stochastic noise tolerance and "very fast" rates for smooth targets in low-d (Calvin–Žilinskas O(n^(-3+δ)) for 1-D, degrading with d). 100–130 evaluations for 4-D is comfortably inside the regime where EGO-style BO has converged in published aero applications (Bartoli et al. arXiv 2504.09930 reports "minimal number of function evaluations" for ONERA aero problems of similar size). The report's "tens to a few hundred" claim (line 18) is consistent with this; 130 is on the conservative side.
- **No online phase.** The launch decision is made offline; GP queries afterward are sub-second.

If multiple `p_target` instances are required, observe that the *expensive* artifact is the ballistic+sweep run, **not** the convolution. With minor refactoring `Q` for any new `p_target` can be recomputed from the cached `Rg` field of each historical sweep — no additional simulation. This decouples target re-targeting from BO budget.

## Validation strategy

- **Synthetic benchmark.** Replace `bo_objective` with a closed-form `J_test(Vo,el,az,p) = -((Vo-100)/20)² - ((el-45)/10)² - ((az-15)/15)² - ((p-0)/2)² + Gaussian noise σ=0.05`. Confirm KG loop converges within 50 evaluations to within 1% of the analytical optimum at `(100, 45, 15, 0)`. This validates the wiring before paying simulation cost.
- **Cross-validation on day-1 audit.** Leave-one-out on the 20 existing LHS points; report posterior-predictive log-likelihood.
- **Held-out high-MC ground truth.** Pick 5 launch points distributed across the BO trajectory (initial LHS, mid-search, recommended optimum). Re-evaluate with `n_deploy=16, n_ct=5, n_nb=8` (≈10× inner count) to obtain a low-variance reference, and compute surrogate-vs-ground-truth bias. Acceptable threshold: |ΔQ| < 0.05, |ΔU| < 10% relative.
- **Reproducibility.** Fix `rng(0)` for LHS; record the BoTorch RNG seed in `bo_log.mat`.

## Risks & mitigations

1. **MATLAB↔Python overhead and brittleness.** `pyenv` out-of-process is robust but adds setup risk and serialization tax. *Mitigation*: build the EI-plus MATLAB-only path in step 3 first; treat BoTorch as a swap-in for the acquisition step only. If pyenv blocks, the EI-plus run is publishable on its own.
2. **Reachability surface non-stationarity at launch-envelope edges** (report caveat lines 209, 214). Sharp drop-offs (e.g., crossing the dSMC stability boundary) can break stationary GPs. *Mitigation*: classify reachability=0 trials explicitly (existing `tot_w > 0` assertion in `sweep_landing_centroid.m` line 275 already errors on full failure). Treat those as a binary infeasibility classifier and exclude from the GP regression — fit BO on the feasible region only. Deep-kernel GP is a fallback (Profile BO arXiv 2512.23581) but probably overkill at d=4.
3. **MC variance per evaluation is high relative to objective gradient.** With 84 inner trials, `σ_Q` per eval is `~√(p(1−p)/84)` ≈ 0.05 near 50% reach. *Mitigation*: increase `n_deploy × n_ct × n_nb` to 168 (double inner count, 4 min/eval) once the GP shows low signal-to-noise — robust-KG handles the heteroscedasticity natively. Or use Pearce–Poloczek–Branke's common-random-numbers KG (Operations Research 2022, DOI 10.1287/opre.2021.2208) which reuses RNG seeds across launches.
4. **Pareto front demand.** If stakeholders want hit-prob vs effort trade-offs, scalarized J=Q−λU is suboptimal. *Mitigation*: scalarize for v1 with λ swept post-hoc on the cached GP (cheap because no resimulation needed). Move to qNEHVI / SEGOMOE only if v1 reveals genuine λ-sensitivity. **Do not pursue multi-objective BO from day 1** — it adds GP-per-objective complexity, hypervolume-acquisition overhead, and obscures debug.
5. **Decision-variable / nuisance-variable coupling.** If wind direction strongly couples with `az`, treating `(w_z0, w_y0)` as nuisance breaks the BO assumption. *Mitigation*: a quick sensitivity scan during step 1 — fit a 6-D GP to the LHS-20 data and inspect length-scales. If the wind length-scales are comparable to the launch length-scales, promote them into the optimization or use the robust-KG formulation that integrates the disturbance analytically.

## Citations

Research-report grounding:
- [`docs/ballistic_targeting_optimization.md`](archived/ballistic_targeting_optimization.md) lines 4, 18, 83–87, 117–124, 154–164, 209, 214.

External references (verified via web fetch):
- Frazier, P. I. (2018). "A Tutorial on Bayesian Optimization." [arXiv:1807.02811](https://arxiv.org/abs/1807.02811). Knowledge Gradient and noisy-EI definitions; convergence-rate discussion.
- Pearce, M. A. L., Le, S., & Branke, J. (2024). "Using the knowledge gradient acquisition function in Bayesian optimization when searching for robust solutions." *Engineering Optimization* 56(1) 36–53. [DOI 10.1080/0305215X.2022.2145604](https://www.tandfonline.com/doi/full/10.1080/0305215X.2022.2145604). Robust KG for objectives that are expectations over disturbances; analytic forms for uniform/normal noise.
- Ungredda, J., Pearce, M., & Branke, J. (2022). "Efficient computation of the Knowledge Gradient for Bayesian Optimization." [arXiv:2209.15367](https://arxiv.org/abs/2209.15367). One-shot Hybrid KG implementation.
- Pearce, M., Poloczek, M., & Branke, J. (2022). "Bayesian Optimization Allowing for Common Random Numbers." *Operations Research* 70(6) 3457–3472. [DOI 10.1287/opre.2021.2208](https://pubsonline.informs.org/doi/10.1287/opre.2021.2208). Seed-aware KG for noise reduction.
- Bartoli, N., Lefebvre, T., Lafage, R., Saves, P., et al. (2024). "Multi-objective Bayesian Optimization with Mixed-Categorical Design Variables for Expensive-to-Evaluate Aeronautical Applications." [arXiv:2504.09930](https://arxiv.org/abs/2504.09930). SEGOMOE / mixed-variable BO for ONERA aero problems.
- Leonard, A., Klein, B., Jumonville, C., Rogers, J., Gerlach, A., & Doman, D. (2017). "A Probabilistic Algorithm for Ballistic Parachute Transition Altitude Optimization." *J. Guidance, Control, and Dynamics* 40(12) 3037–3049. [DOI 10.2514/1.G002243](https://arc.aiaa.org/doi/10.2514/1.G002243). Direct precedent for stochastic launch-decision optimization.
- BoTorch one-shot KG tutorial: [https://botorch.org/docs/tutorials/one_shot_kg/](https://botorch.org/docs/tutorials/one_shot_kg/).
- MATLAB `bayesopt` documentation (acquisition list): [https://www.mathworks.com/help/stats/bayesopt.html](https://www.mathworks.com/help/stats/bayesopt.html). Confirms KG is unavailable and custom acquisition functions are unsupported.
