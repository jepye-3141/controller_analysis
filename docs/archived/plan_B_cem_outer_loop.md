# Plan B — Cross-Entropy Method over the 4-D Launch Space

> Source: Recommendation 2 of [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 166–175.

## Overview

Replace the Bayesian-optimization outer loop of [Plan A](../plan_A_bo_gp_surrogate.md) with the Cross-Entropy Method (CEM) over the same 4-D decision vector `(Vo, el, az, p)`. CEM iteratively fits a Gaussian (or Gaussian-mixture) over the launch space to the top fraction of sampled launches scored by `J = E_ξ[Q] − λ E_ξ[U]`, with the existing `sweep_landing_centroid.m` providing a per-sample noisy estimate of `J`. CEM is the right hedge against (a) a non-smooth or multi-modal reachability landscape that a stationary GP cannot fit and (b) the appeal of a 100% MATLAB-native, gradient-free, embarrassingly parallel implementation that does not depend on `pyenv`/BoTorch. A vanilla, no-frills CEM in MATLAB is roughly 100 lines.

## Mathematical formulation

- Decision vector identical to Plan A: `θ = (Vo, el, az, p) ∈ R^4`. Other ranges in [`centroid_lookup_table.m`](../centroid_lookup_table.m) (`w_z0, w_y0`) are uncertainty inputs ξ, not decision variables.
- Objective identical to Plan A (report lines 117–124):
  - `Q(θ) = E_ξ[ ∫ R(x_t | x_d(θ, ξ)) p_target(x_t) dx_t ]`
  - `U(θ) = E_ξ[ ∫ ‖u(t; x_d)‖² dt ]`
  - `J(θ) = Q(θ) − λ U(θ)`
- CEM proposal: `θ ~ N(μ_t, Σ_t)` truncated to the box from `centroid_lookup_table.m` lines 8–14 (`Vo ∈ [80,120]`, `el ∈ [35,55]`, `az ∈ [0,30]`, `p ∈ [-2,2]`). Initial `μ_0` = box center, `Σ_0` = `diag((box_width/4)^2)` so ±2σ covers the box.
- Per-iteration update with elite fraction ρ (default 0.20):
  1. Sample `{θ_i}_{i=1}^{N_pop}` from `N(μ_t, Σ_t)`, reject-and-resample any out-of-box.
  2. Evaluate `J_i = J(θ_i)` via `sweep_landing_centroid` (≈84 inner trials each, already an MC average — see "noise variance" below).
  3. Sort, retain top `K = ρ·N_pop` elites.
  4. Refit: `μ_{t+1} = mean(elites)`, `Σ_{t+1} = cov(elites) + α·Σ_t` (smoothing `α ∈ [0.1, 0.5]` per Kroese tutorial §3 to prevent premature collapse).
  5. Stop on `‖μ_{t+1} − μ_t‖ < tol` or `tr(Σ_{t+1}) < tol`, or after a budget cap.
- Diagonal vs full covariance: start diagonal (4 free variances), promote to full only if late-iteration elite scatter is visibly anisotropic. Full covariance has 10 free entries in 4-D — well-supported by `K = 0.2 · 80 = 16` elites.

## Required infrastructure

- **New MATLAB code**:
  - `cem_objective.m` — same wrapper used by Plan A (`bo_objective.m`); returns `(Q, U, σ_Q², σ_U², J)`. The wrapper is a planning artifact shared between Plan A and Plan B; whichever plan ships first owns the file.
  - `cem_run.m` — driver that owns the CEM loop. Pure MATLAB: `mvnrnd`, sort, refit, smooth. Persistence to `cem_log.mat`.
  - `cem_validate.m` — replays the recommended optimum and a held-out cross-section with high-MC ground truth, identical philosophy to `bo_validate.m`.
- **External dependencies**: **none beyond the existing toolboxes**. `mvnrnd` is in Statistics and Machine Learning Toolbox, already required by Plan A. No Python, no PyTorch, no `pyenv`. This is the single biggest implementation-risk advantage over Plan A. Reference algorithms: Kroese et al. ["A Tutorial on the Cross-Entropy Method"](https://people.smp.uq.edu.au/DirkKroese/ps/aortut.pdf) §3 (continuous CEM); MATLAB Central Simple Multi-Objective Cross Entropy Method (Haber et al., IEEE Access 2017) as a sanity-check against an external reference.
- **Optional dependencies (not on day-1 critical path)**:
  - Bregman-Centroid Guided CEM ([Gu et al., arXiv 2506.02205](https://arxiv.org/abs/2506.02205), 2025) — only adopt if §"Multi-modality risk" below triggers. Adds ~50 lines (multi-worker ensemble + Bregman centroid update of underperformers).
  - iCEM ([Pinneri et al., arXiv 2008.06389](https://arxiv.org/abs/2008.06389), 2020) — see §"Temporal-correlation question" below; the answer is "do not adopt iCEM verbatim, but borrow elite-memory."
  - CEM-GD ([Huang et al., arXiv 2112.07746](https://arxiv.org/abs/2112.07746), 2021) — only relevant if a differentiable surrogate to the ballistic+sweep map becomes available; not the case for v1.
- **New data products**:
  - `cem_log.mat` — append-only history `(iter, θ_i, Q_i, U_i, J_i, is_elite, μ_t, Σ_t)`, plus the elite-memory buffer (Plan A's `bo_log.mat` is a near-isomorphic struct; consider a shared schema).
  - Optional `target_distributions/` — same artifact as Plan A; reusable.

## Integration with existing codebase

- [`sweep_landing_centroid.m`](../sweep_landing_centroid.m) is reused **unchanged**, with the same one-line patch as Plan A (return per-trial `ctrl_trial` integrals so `U(θ)` is computable). Beyond that there is zero coupling between CEM and the simulator.
- [`centroid_lookup_table.m`](../centroid_lookup_table.m) is **supplanted** by `cem_run.m`. The current Latin-Hypercube design becomes the **first generation** of CEM (`N_pop = 20–80`, μ = box center, Σ very wide), so existing data in `centroid_lookup_log.mat` is reusable as generation-0 elites if the schemas align. This is a strict generalization: LHS is what CEM does in iteration 0 with maximally diffuse Σ.
- `parsim` wiring is **untouched**. CEM samples are evaluated **serially** through the existing 4-worker `parsim` inside `sweep_landing_centroid` — i.e., the existing parallelism is at the *inner* MC level, not at the outer CEM-population level. Outer-level parallelism (running multiple population members concurrently) would require a second `parpool` and is not on the critical path; see "Computational budget".
- **Simulink touch-points: none.** Same as Plan A.

## Implementation roadmap

1. **Days 1–2: shared objective wrapper.** If `bo_objective.m` from Plan A already exists, reuse it. Otherwise build it: patch `sweep_landing_centroid.m` to surface per-trial `ctrl_trial` integrals; add `Q` (with `p_target` argument) and `U` computation. Sanity-check `Q(p_target=uniform) ≈ reachability_pct` on three launch points. **Verifiable: unit test passes.**
2. **Day 3: synthetic CEM smoke test.** Apply `cem_run` to a closed-form 4-D quadratic-in-log-space `J_test` (same as Plan A's day-7 synthetic). Confirm convergence within 5 generations × `N_pop = 50`. **Verifiable: `‖μ − argmax J_test‖ < 0.05` of box width, repeatable across 5 seeds.**
3. **Days 4–5: `cem_run.m` against the real simulator.** First run with `N_pop = 40, ρ = 0.2, smoothing α = 0.3, max_iter = 5`. Total ≈40 × 5 × 2 min = 6.7 h. Save `cem_log.mat`. **Verifiable: `J*` improves monotonically across iterations; final Σ trace shrunk by ≥10×.**
4. **Day 6: head-to-head against Plan A on the same `(p_target, λ)`.** Compare recommended optima `θ*_BO` vs `θ*_CEM`, total wall hours, and final `J(θ*)`. Document in `docs/README.md`.
5. **Days 7–8 (CONDITIONAL).** If the CEM run shows multi-modal posterior (visualizable as the cluster of elites in the final iteration), enable Bregman-centroid guidance per arXiv 2506.02205 — runs 4 CEM workers in parallel from random inits and pulls underperformers toward the performance-weighted centroid in the dual (Bregman) space. **Verifiable: distinct elite clusters survive into the final iteration; reported best `J*` improves vs vanilla CEM by >5%.**
6. **Day 9: documentation.** README.md entry-point note for `cem_run`, `cem_log.mat` schema, and the comparison table against Plan A.

Total: ≈ 9 dev-days (one engineer), assuming `cem_objective.m` is shared with Plan A. Standalone (Plan A not built first): ≈ 11 dev-days. Schedule risk is dominated by the simulator wall time (≥6 h), not by code volume — `cem_run.m` itself is ~100 lines.

## Computational budget

- **Per-evaluation cost**: identical to Plan A — one `sweep_landing_centroid` call ≈ 2 min wall on 4 workers, ≈84 inner trajectories, no upstream changes.
- **Population × generations**: the prompt's reference figure is `N_pop = 100 × 5 = 500 evals × 2 min = 16.7 h`. This is the *uncritical* ceiling. Recommended for v1: `N_pop = 40, generations ≤ 5`, total `200 × 2 min = 6.7 h`. iCEM-style elite memory (re-using top fraction of previous-generation elites in the next generation's pool, [Pinneri et al. arXiv 2008.06389](https://arxiv.org/abs/2008.06389)) cuts effective `N_pop` by ~30% per generation after the first, dropping the budget toward `~5 h`. iCEM's claimed 2.7–22× sample efficiency over vanilla CEM is for *sequential control planning* with temporally-correlated noise; for static parameter optimization the elite-memory component alone is the relevant transfer (see "Temporal-correlation question" below).
- **Parallelization headroom**: Each `sweep_landing_centroid` call already saturates the 4-worker pool *internally*. Outer-level parallelism (running `M` population members concurrently) requires `M × 4` workers and an outer `parpool` — currently the project has 4 logical workers, so no headroom without provisioning. If a 16-core node is available, outer-population parallelism cuts the wall clock by 4× to ≈90 min for the recommended budget; this is the most direct way to make CEM competitive with BO's 4.5-h wall.
- **Comparison vs Plan A**: at `N_pop = 40 × 5 generations`, CEM is 1.5× the BO wall (6.7 h vs 4.5 h) and has fewer iterations (5 vs 100), so the per-iteration information gain in CEM is much higher (large batch). Empirically, on smooth low-d objectives BO wins; on rugged or multi-modal landscapes CEM wins. **The decision criterion (report line 175) is whether multi-modality is suspected, *not* whether CEM is faster.**
- **No online phase** — same as Plan A; the launch decision is offline.

## Validation strategy

- **Synthetic benchmark** (shared with Plan A's day-7 step). Closed-form `J_test(Vo,el,az,p)` with known optimum at `(100, 45, 15, 0)`. CEM should converge `μ` within 0.05 of the optimum (in normalized box coordinates) in ≤5 generations × 50 samples.
- **Multi-modal synthetic.** A bimodal `J_test = max(J_low_el_high_v, J_high_el_low_v)` per the report's "low-elevation/high-velocity vs. high-elevation/low-velocity" hypothesis (line 175). Vanilla CEM should fail (collapse to one mode); BC-EvoCEM should succeed. This is the unit test that gates the day-7 conditional branch.
- **Cross-comparison vs LHS baseline.** The existing 20-point `centroid_lookup_log.mat` defines the LHS-best `J_LHS*`. CEM should beat it by ≥10% on the real simulator within 3 generations (≈120 evaluations vs 20). If not, the GP smoothness assumption is probably violated and Plan A would also have struggled — useful diagnostic either way.
- **Held-out high-MC ground truth.** Re-evaluate `θ*_CEM` and 4 elite alternatives with `n_deploy=16, n_ct=5, n_nb=8` (≈10× inner count). Acceptable: `|J(θ*) − J_groundtruth(θ*)| < 0.05`. Same threshold as Plan A.
- **Reproducibility.** Fix `rng(0)` at top of `cem_run.m`; persist all per-iteration `(μ_t, Σ_t)` so a run is replayable.

## Risks & mitigations

1. **Outer MC noise vs CEM elite selection.** Each `J_i` is a noisy estimate (84 inner trials). When two true `J` values are within the inner-MC standard error, the elite selection is essentially random and `Σ_{t+1}` over-shrinks toward whichever sample happened to win. Vanilla CEM does *not* model noise. *Mitigation*: enforce a noise floor on `Σ` (`Σ ← Σ + σ_floor² · I` after refit, per Kroese §3 smoothing) or, more rigorously, score by `J_i − c·σ_i` (lower confidence bound) so noisy losers cannot crowd out plausible winners. The report does not require this but iCEM's Pinneri et al. note that explicit noise handling is a known weakness of vanilla CEM.
2. **Premature unimodal convergence (multi-modality risk, report line 175).** CEM fits a unimodal Gaussian; if the launch landscape is bimodal, CEM commits to whichever mode the initial elites favor and never recovers. *Mitigation*: Bregman-Centroid Guided CEM ([Gu et al., arXiv 2506.02205](https://arxiv.org/abs/2506.02205)) runs an ensemble of CEM workers and pulls underperformers toward a Bregman-weighted ensemble centroid. Cheap to add (4 workers × half population = same total budget) and gates on visible multi-modality in the day-5 elite scatter plot. If full bimodality is confirmed, GMM-CEM (mixture of two Gaussians, refit per Kroese §3.4) is the textbook fix.
3. **Box constraints + truncation bias.** Sampling from `N(μ, Σ)` and rejecting out-of-box draws biases the empirical mean inward when `μ` approaches a face. *Mitigation*: reparameterize via logit on each axis (`Vo ↦ logit((Vo − 80)/40)`, etc.) so the box becomes `R^4` in the optimization space; transform back for `sweep_landing_centroid`. Standard practice in Kroese §3.5.
4. **Sample efficiency vs Plan A.** For a smooth, low-d, low-noise surface, KG-driven BO converges in tens of evaluations and dominates CEM's hundreds. *Mitigation*: run both. Plan A is the priority pathway if compute is the binding constraint; Plan B is the priority pathway if implementation simplicity, robustness to non-stationarity, or multi-modality is the binding constraint. The two are mutually informative — converging to similar `θ*` cross-validates both.
5. **Inner-MC dominates outer optimization.** Each generation's wall is gated by 84-trial inner MC × 40 population, not by the CEM math. Outer-loop optimizations (any tweak to `N_pop`, `ρ`, smoothing, etc.) save minutes on hours-long runs. *Mitigation*: only invest in outer-loop tuning if Plan C (PCE) replaces the inner MC.

## Specific question answers

These are the questions the planning prompt asks to address explicitly. They are scattered through the sections above, but consolidated here for the reviewer.

- **MATLAB-native vs Python.** *Stay MATLAB-only.* CEM is roughly 100 lines of `mvnrnd` + sort + refit. The Plan A `pyenv`/BoTorch setup buys nothing for CEM. The optional Bregman-centroid extension is also trivial in MATLAB.
- **Population size & elite fraction.** `N_pop = 40, ρ = 0.2, generations ≤ 5` for v1 (≈ 6.7 h wall). The prompt's `N_pop = 100 × 5 = 16.7 h` is the upper bound; `40 × 5` is enough for 4-D in published CEM benchmarks (Kroese tutorial; Kobilarov RSS 2011). iCEM's 2.7–22× sample efficiency claim is for sequential control, not 4-D one-shot — but its elite-memory mechanism (carrying the top-k elites into the next generation's pool) generalizes and is recommended.
- **Outer-CEM noise vs inner-MC averaging.** Each `J_i` already averages 84 inner trials, so the outer noise variance is small. Vanilla CEM does not model it; mitigate via covariance-floor smoothing (Kroese §3) or a lower-confidence elite score. No need for an explicit noise model unless the inner MC is dropped to <30 trials per evaluation.
- **Temporal-correlation question.** iCEM's temporally-correlated noise is for *sequential* decisions (control trajectories at successive timesteps); the launch decision is one-shot 4-D. **Vanilla CEM with elite-memory is the right fit.** Do not import iCEM's colored-noise sampler.
- **Multi-modality risk.** Vanilla CEM commits to one mode and forgets the other. If the day-5 elite scatter shows two clusters, switch to Bregman-Centroid Guided CEM ([arXiv 2506.02205](https://arxiv.org/abs/2506.02205)) or GMM-CEM. Plan A's GP also handles multi-modality but only if the kernel is non-stationary; standard Matérn-5/2 will not.
- **Comparison vs Plan A (BO/GP).** Choose CEM over BO when (a) a Python toolchain is undesired or unavailable, (b) the objective is suspected multi-modal, (c) the reachability surface is non-smooth at envelope edges (report caveat 209, 214), or (d) outer-population parallelism is cheaper to provision than additional GP-hyperparameter tuning. Choose BO over CEM when (a) compute is the binding constraint, (b) sensitivity analysis / Pareto front is wanted from the same surrogate, or (c) the objective is smooth and low-noise.

## Citations

Direct citations to the research report and the source literature.

- Report Recommendation 2 (CEM) — [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 166–175.
- Report Section B.Cross-Entropy Method paragraph — [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 77.
- Kobilarov, "Cross-Entropy Randomized Motion Planning," RSS 2011, [IEEE Xplore 6301069](https://ieeexplore.ieee.org/document/6301069/) — foundational CEM-for-planning reference.
- Pinneri, Sawant, Blaes, Achterhold, Stueckler, Rolinek, Martius, "Sample-efficient Cross-Entropy Method for Real-time Planning," CoRL 2020, [arXiv 2008.06389](https://arxiv.org/abs/2008.06389) — iCEM, claims 2.7–22× sample efficiency for sequential control; only the elite-memory component is directly relevant to one-shot 4-D launch optimization.
- Huang, Lale, Rosolia, Shi, Anandkumar, "CEM-GD: Cross-Entropy Method with Gradient Descent Planner for Model-Based RL," 2021, [arXiv 2112.07746](https://arxiv.org/abs/2112.07746) — not on the v1 critical path; relevant only if a differentiable ballistic-plus-sweep surrogate becomes available.
- Gu, Cao, Caccamo, Hovakimyan, "Bregman Centroid Guided Cross-Entropy Method," 2025, [arXiv 2506.02205](https://arxiv.org/abs/2506.02205) — BC-EvoCEM, addresses CEM's premature unimodal convergence via ensemble + Bregman-weighted centroid update of underperforming workers.
- Williams, Aldrich, Theodorou, "Model Predictive Path Integral Control: From Theoretical Foundations to Real-Time Applications," JGCD 2017 — MPPI, the close cousin of CEM; relevant to the *downstream* powered controller, not the outer launch optimizer (per report line 81).
- Rogers & Slegers, "Robust Parafoil Terminal Guidance Using Massively Parallel Processing," JGCD 2013 — motivates GPU-accelerated inner-MC if Plan B's wall budget becomes binding (not v1).
- Kroese, Rubinstein, Cohen, Porotsky, Taimre, ["A Tutorial on the Cross-Entropy Method"](https://people.smp.uq.edu.au/DirkKroese/ps/aortut.pdf), Annals of Operations Research 2005 — standard reference for the continuous-CEM update rules including covariance smoothing.
- Haber, Beruvides, Quiza, Gonzalez, "A Simple Multi-Objective Optimization based on the Cross-Entropy Method," IEEE Access 5 (2017), DOI 10.1109/ACCESS.2017.2764047 — MATLAB Central package usable as an external sanity check. Multi-objective extension only needed if Pareto front over `(Q, U)` is required (Plan A's `qNEHVI` route is the alternative).
