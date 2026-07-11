# Comparative Survey — Implementation Pathways for Ballistic Targeting Optimization

> Source research report: [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md).
> Companion plans: [Plan A](../plan_A_bo_gp_surrogate.md) · [Plan B](plan_B_cem_outer_loop.md) · [Plan C](plan_C_polynomial_chaos.md) · [Plan D](plan_D_cvar_covariance_steering.md) · [Plan E](plan_E_multi_fidelity_surrogate.md) · [Plan F](plan_F_neural_reachability_surrogate.md).
> Judge reports: [Judge 1 — Compute & integration](judge_1_compute_integration.md) · [Judge 2 — Math soundness](judge_2_math_soundness.md) · [Judge 3 — Time-to-MVP & risk](judge_3_time_to_mvp.md).

## TL;DR

The three judges converge on a single recommended sequence:

1. **Phase 1 — Plan B (CEM) first.** ~7 dev-days. Pure MATLAB. No `pyenv`. ~6.7 h sim wall. Produces the first defensible `(Vo, el, az, p)` recommendation. *This is the MVP.*
2. **Phase 2 — Plan A (BO/GP) on top of Plan B's cache.** ~14 days incremental. EI-plus on stock `bayesopt` ships first (~day 7), BoTorch+rKG by day 13. Two independent optima cross-validate the recommendation.
3. **Phase 3 — Plan D Track 1 (CVaR).** ~5 days, only if a worst-case acceptance criterion is articulated. One-line patch to `sweep_landing_centroid.m` plus a 30-line wrapper.
4. **Phases 4–5 — Plan E and Plan F.** Both are conditional follow-ups. Plan E gated on `ρ_S(J_lo, J_hi) ≥ 0.6` and on multi-mission re-runs being expected. Plan F gated on `N ≥ 100` cached sweeps from A/B.
5. **Plan C (PCE) — parked.** Plan C diagnoses its own irrelevance under the current dSMC-dominated cost profile (`eom2` is ~0.04% of wall). Worth ~2–3 days as a Sobol' analysis to justify the ξ-treatment, never as a full pipeline.
6. **Plan D Track 2 (covariance steering) — dropped.** No published precedent for cov-steering on a sliding-mode controller; commercial MOSEK + 4–8 dev-weeks of LMI tooling for a deliverable the project does not require.

The strongest single result available with the current toolchain is **Plan B's recommended optimum + Plan A's GP surrogate posterior over the same 4-D box** — a cross-validated launch decision plus a calibrated surface that supports post-hoc Pareto/sensitivity exploration without further simulation.

## Comparative ranking matrix

| Plan | J1 (compute/integration) | J2 (math soundness) | J3 (time-to-MVP/risk) | Composite |
|------|--------------------------|---------------------|------------------------|-----------|
| **B** — CEM | **1**: pure MATLAB, 0 new deps, 1-line patch | 6: identical formulation to A; same deterministic-vs-ξ conflation | **1**: 7 dev-days, ~6.7 h sim wall, MVP | **1** |
| **A** — BO/GP | 3: realistic 4.5 h wall; pyenv risk; MATLAB-only fallback publishable | 3: correct canonical formulation, but inherits "84 trials = MC" framing | 2: 9–14 days; rKG benefit real but not critical | **2** |
| **D** — CVaR + cov-steering | **2** (Track 1): trivial bolt-on. **6** (Track 2): MOSEK + months of work | **1**: best diagnosis of the deterministic-vs-ξ conflation; Track 2 deferred for the right reasons | **3** (Track 1): 5 days after A/B exists | **3** (Track 1) / drop (Track 2) |
| **F** — NN surrogate | 4: `fitrnet` in-toolbox; gated on N≥100 | 5: solves a side problem (caching), open about that | 5: hard gate on N≥100 cached sweeps from A/B | **5** |
| **E** — Multi-fidelity | 5: pyenv on top of A; ~2 h saved on 4.3 h baseline | 4: inherits A's framing; cheap-fidelity recipe sound | 4: 11–15 days; gated on `ρ_S ≥ 0.6` and multi-mission demand | **4** |
| **C** — PCE | 6: wrong cost-center; plan parks itself | **2**: best epistemic discipline; honest about cost-center mismatch | 6: parked; ~2 days as Sobol' analysis only | **6** |

J1, J2, J3 columns reflect the three independent judging axes. The composite is the recommended overall sequencing — a weighted summary, not a strict average. Plan D Track 1 ranks #2 on math but #3 on time-to-MVP because it is sequentially after A or B.

## Per-plan summary cards

### Plan A — Bayesian Optimization with GP/Kriging surrogate · [→ full plan](../plan_A_bo_gp_surrogate.md)

**Source.** [Recommendation 1 of the research report](ballistic_targeting_optimization.md), lines 154–164.

**Architecture.** GP/Kriging surrogate over `(Vo, el, az, p) ∈ R^4`; robust Knowledge-Gradient (rKG) acquisition (Pearce–Le, *Engineering Optimization* 56(1) 2024, [DOI 10.1080/0305215X.2022.2145604](https://www.tandfonline.com/doi/full/10.1080/0305215X.2022.2145604)) over a per-launch noisy expectation `J = E_ξ[Q] − λ E_ξ[U]`. Each iteration calls `sweep_landing_centroid.m` once. 30 LHS init + 100 KG steps = ~4.5 h on 4 workers.

**Toolchain.** MATLAB + `pyenv` + BoTorch (recommended) or stock `bayesopt` with EI-plus (fallback). MATLAB's `bayesopt` does not support custom acquisitions ([MathWorks docs](https://www.mathworks.com/help/stats/bayesopt.html)), so KG requires Python.

**Strengths.** Correct canonical formulation; tightest sample efficiency on smooth low-d objectives; the GP posterior is reusable for sensitivity and Pareto-front post-processing for any new `(p_target, λ)` without resimulation.

**Weaknesses.** Math-soundness judging (J2) flagged that the plan implicitly identifies `landing_ratio` (line 17 of the plan, line 256 of `sweep_landing_centroid.m`) with `R(x_t | x_d)`, treating the 84-cell deterministic structural grid as if each cell were a Bernoulli sample of ξ. This propagates into the heteroscedastic noise model and the rKG acquisition, both of which assume the noise they observe is a sample average over the disturbance.

**Pyenv risk.** Schedule risk is dominated by `pyenv` setup. Day-7 EI-plus result is publishable on its own; rKG is the upgrade.

**Citations from the report.** Frazier 2018 ([arXiv 1807.02811](https://arxiv.org/abs/1807.02811)), Bartoli et al. 2024 ([arXiv 2504.09930](https://arxiv.org/abs/2504.09930)), Pearce & Le 2024, Leonard et al. JGCD 2017 ([DOI 10.2514/1.G002243](https://arc.aiaa.org/doi/10.2514/1.G002243)).

### Plan B — Cross-Entropy Method · [→ full plan](plan_B_cem_outer_loop.md)

**Source.** [Recommendation 2 of the research report](ballistic_targeting_optimization.md), lines 166–175.

**Architecture.** Same decision space, same objective, gradient-free distribution fitting. `N_pop=40, ρ=0.20, max_iter=5`, ~200 sweeps × 2 min = 6.7 h. ~100 lines of `mvnrnd` + sort + refit.

**Toolchain.** Pure MATLAB. Statistics & ML Toolbox already required. No Python.

**Strengths.** Lowest implementation risk of any plan. Cross-validates Plan A. Robust to non-smooth or multi-modal landscapes that a stationary GP cannot fit.

**Weaknesses.** Vanilla CEM does not model evaluation noise; with `σ_J ≈ 0.05` two `θ` within ~0.1 are statistically indistinguishable, biasing the covariance refit. iCEM's reported 2.7–22× sample efficiency (Pinneri et al. [arXiv 2008.06389](https://arxiv.org/abs/2008.06389)) is for sequential control, not 4-D one-shot — the plan correctly disclaims this.

**Multi-modality hedge.** Bregman-Centroid Guided CEM (Gu et al. [arXiv 2506.02205](https://arxiv.org/abs/2506.02205)) is conditional, not on the day-1 critical path.

**Citations from the report.** Kobilarov RSS 2011 ([IEEE Xplore 6301069](https://ieeexplore.ieee.org/document/6301069/)), iCEM ([arXiv 2008.06389](https://arxiv.org/abs/2008.06389)), Bregman-CEM ([arXiv 2506.02205](https://arxiv.org/abs/2506.02205)), Williams et al. JGCD 2017 (MPPI).

### Plan C — Polynomial Chaos Expansion · [→ full plan](plan_C_polynomial_chaos.md)

**Source.** [Recommendation 3 of the research report](ballistic_targeting_optimization.md), lines 177–186.

**Architecture.** Non-intrusive gPC on the ballistic transition kernel `x_d = F_ballistic(launch, ξ)`. UQLab v2 ([www.uqlab.com](https://www.uqlab.com/), BSD-3 since 2022); compressed-sensing PCE / LARS preferred over Smolyak at `d = 9`.

**Honest verdict (the plan parks itself).** `sweep_landing_centroid.m` is *deterministic in ξ* (line 58 of that file calls `eom2` exactly once). dSMC is ~30 s/eval/worker; `eom2` is ~50 ms (~0.04% of wall). Even at `N_ξ = 16` the eom2 cost is only ~2.6%. The report's "10–100× speedup" claim only holds if Plan A or B grows a true ξ-MC at the deployment-state level.

**What's worth keeping.** A one-time Sobol' index analysis (~2–3 days of UQLab work) would justify whether `(w_z0, w_y0)` can be held deterministic in Plans A/B — that's the highest-value subset of Plan C.

**Citations from the report.** Xiu–Karniadakis 2002 ([DOI 10.1137/S1064827501387826](https://epubs.siam.org/doi/10.1137/S1064827501387826)), Vittaldev–Russell 2016 (JGCD, [DOI 10.2514/1.G001571](https://arc.aiaa.org/doi/10.2514/1.G001571)), Boutselis et al. 2017 ([arXiv 1705.05506](https://arxiv.org/abs/1705.05506)), Nakka et al. 2021 ([arXiv 2106.02801](https://arxiv.org/abs/2106.02801)), Sharma et al. 2024 ([arXiv 2402.15115](https://arxiv.org/abs/2402.15115)).

### Plan D — CVaR / Chance-Constrained + Covariance Steering · [→ full plan](plan_D_cvar_covariance_steering.md)

**Source.** [Recommendation 4 of the research report](ballistic_targeting_optimization.md), lines 188–196.

**Track 1 (CVaR).** One-line patch to [`sweep_landing_centroid.m:233`](../sweep_landing_centroid.m) to surface `out.miss_per_cell`. Sample-CVaR via `prctile`/`bootci` (Statistics & ML Toolbox, already required). 5 dev-days. Drops cleanly into Plan A's `bo_objective.m` or Plan B's `cem_objective.m`. Recommend bootstrap CIs (Bentum & Asante 2024, [SSRN 5771802](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5771802)) because at `α=0.1, N=84` sample-CVaR has ~30% standard error.

**Track 2 (covariance steering).** YALMIP + commercial MOSEK + LTV linearization of dSMC closed-loop. **No published precedent for cov-steering on a sliding-mode controller** (Plan D §4.2). 4–8 dev-weeks just to install tooling; total effort 3+ months for a deliverable the offline launch problem does not require. **Drop.**

**Best math-soundness in the survey.** Track 1 of Plan D explicitly identifies that the existing 84-cell sweep is the fraction of the deterministic structural grid where dSMC stabilized — not a Monte-Carlo estimate of `P[hit]` (lines 36–43 of the plan). This is the hidden-assumption error the original research report glosses over and that Plans A, B, E, F all silently accept.

**Citations from the report.** Rockafellar–Uryasev 2000, Tsiotras' Mars descent papers (Ridderhof & Tsiotras [AIAA 2018-0611](https://dcsl.gatech.edu/papers/aiaa17c.pdf)), Liu et al. 2025 ([arXiv 2504.04705](https://arxiv.org/abs/2504.04705)), Greco–Vasile AIAA SciTech 2020 ([PDF](https://strathprints.strath.ac.uk/71168/)), Mudrik–Oshman 2026 ([arXiv 2604.17811](https://arxiv.org/abs/2604.17811)).

**Citation correction.** The research report attributes [arXiv 2402.01370](https://arxiv.org/abs/2402.01370) to "Pezzato et al."; the actual authors are **Brudermüller et al.**, "CC-VPSTO: Chance-Constrained Via-Point-based Stochastic Trajectory Optimisation" (2024).

### Plan E — Multi-Fidelity Surrogate Stack · [→ full plan](plan_E_multi_fidelity_surrogate.md)

**Source.** Cross-cutting from [Section C and Recommendations 1–2 of the research report](ballistic_targeting_optimization.md), lines 127–131.

**Architecture.** AR1 co-Kriging (Le Gratiet & Garnier IJUQ 2014, [arXiv 1210.0686](https://arxiv.org/abs/1210.0686)) via SMT 2.0's MFK class ([arXiv 2305.13998](https://arxiv.org/abs/2305.13998)) accessed through `pyenv`. Cheap fidelity is **a reduced-sweep `sweep_landing_centroid_lo` (1×1×1 grid, ~10 s/eval)**, NOT a 3-DoF point-mass replacement for `eom2`. Acquisition is MF-MES (Takeno et al. [arXiv 1901.08275](https://arxiv.org/abs/1901.08275)) or BOCA (Kandasamy et al. [arXiv 1703.06240](https://arxiv.org/abs/1703.06240)).

**Hard validation gate.** Spearman `ρ_S(J_lo, J_hi) ≥ 0.6` on a 6×6 paired-evaluation grid before committing to the MFBO loop ([*Nature Computational Science* 2025](https://www.nature.com/articles/s43588-025-00822-9)). Below 0.4 abort.

**Honest budget.** ~2.2 h vs Plan A's 4.3 h. Saves ~2 h per Plan A run for ~11 dev-days of work plus pyenv complexity. **Worth it only at 5+ `(p_target, λ)` re-runs.**

**Why not 3-DoF point-mass?** The bottleneck is the dSMC powered-phase Simulink sweep, not `eom2`. A 3-DoF substitute can only rank-correlate with the high-fidelity through the deploy state, not through the dSMC reachability map — the cheap fidelity must share the dSMC physics with the HF, which only the reduced sweep does.

**Citations from the report.** Kandasamy et al. 2017 ([arXiv 1703.06240](https://arxiv.org/abs/1703.06240)), Yondo et al. 2018 (*Progress in Aerospace Sciences*, [DOI 10.1016/j.paerosci.2017.11.003](https://www.sciencedirect.com/science/article/abs/pii/S0376042117300611)), Foumani et al. 2024 ([DOI 10.1115/1.4064160](https://asmedigitalcollection.asme.org/mechanicaldesign/article/146/6/061703/1171649)).

### Plan F — Neural-Network Reachability Surrogate · [→ full plan](plan_F_neural_reachability_surrogate.md)

**Source.** [Section A items 5–7 (lines 53–56) and Section C "Neural-network surrogates"](ballistic_targeting_optimization.md), lines 114–115.

**Honest reposition.** **DeepReach (Bansal–Tomlin), NeuralPARC ([arXiv 2409.13195](https://arxiv.org/abs/2409.13195)), CARe ([arXiv 2503.23912](https://arxiv.org/abs/2503.23912)) are not applicable** — they require an HJB-PDE residual or a piecewise-affine ReLU trajectory model, neither of which the user's stochastic Simulink dSMC sweep produces. Plan F is supervised regression on Monte-Carlo data, not certified Hamilton–Jacobi reachability. It is best understood as a *cached amortizer* underneath Plans A–E rather than a competing optimizer.

**Architecture.** MATLAB-native `fitrnet([64 64], 'Activation', 'relu')`. Predict `(reach_pct, cx, cy, U)` jointly from a shared MLP trunk with four output heads. Optional 5-MLP deep ensemble for uncertainty.

**Hard prerequisite.** N ≥ 100 cached sweeps; with the existing 20-LHS lookup, a 5000-parameter MLP is hopelessly under-determined. Plans A and B will generate this data as a by-product.

**Cliff risk.** dSMC saturation produces a sharp non-stationarity — MLPs with smooth activations interpolate confidently across discontinuities. Mitigations: (a) separate `fitcnet` binary classifier on `tot_w == 0` outcomes; (b) deep-kernel GP fallback (Profile BO [arXiv 2512.23581](https://arxiv.org/abs/2512.23581)); (c) deliberately oversample the cliff during active learning.

**Citations from the report.** DeepReach (Bansal–Tomlin), NeuralPARC ([arXiv 2409.13195](https://arxiv.org/abs/2409.13195)), CARe ([arXiv 2503.23912](https://arxiv.org/abs/2503.23912)), Xiang et al. 2020 ([arXiv 2004.12273](https://arxiv.org/abs/2004.12273)).

**Citation correction.** The research report cites the active-sampling recipe at `arXiv 1910.02500` and attributes it to "Devolder & Hewing." The actual authors are **Devonport & Arcak**, "Data-Driven Reachable Set Computation Using Adaptive Gaussian Process Classification and Monte Carlo Methods" (2019).

## Cross-cutting findings

### 1. The 84-trial sweep is a deterministic structural grid, not Monte Carlo over ξ

The single most important finding from the survey, surfaced cleanly only in Plans C and D and confirmed by Judge 2.

`sweep_landing_centroid.m` calls `eom2` *exactly once* per launch with the launch tuple, then sweeps a deterministic `8 deploy × 3 cross-track × 4 neighbor` structural grid (~84 valid cells) for that single ballistic trajectory. **There is zero ξ-randomness inside the existing pipeline.**

Implications:

- `landing_ratio` is *not* a Monte-Carlo estimate of `P[hit]`; it is the fraction of the 84-trial deterministic grid where dSMC stabilized.
- Plan A's heteroscedastic GP noise model (Bernoulli-style `σ_Q ≈ √(p(1−p)/84) ≈ 0.05`) treats the 84 cells as if they were i.i.d. samples — they are not.
- Plan A's robust-KG acquisition assumes the noise is a sample average over the disturbance — at exactly the moment the wrapper is averaging over a deterministic structural grid. This is the single most acute mathematical fault in Plan A.
- Plan B's CEM elite selection has the same issue, but CEM degrades more gracefully under model misspecification — it just spends more budget.
- Plan D Track 1 correctly relabels the cell-level statistic as a *Conditional-Worst-Cell* statistic, not a CVaR_α in the standard sense.

**Recommended action.** Before shipping Plan A or B v1, decide explicitly whether (a) to add a true ξ-MC at the deployment-state level (multiplies wall cost by `N_ξ`) or (b) to redefine the canonical objective as a structural-grid average and document that as the working definition. Option (b) is what the existing pipeline *already* computes; option (a) is what the research report *describes*.

### 2. The 2-min/sweep wall is the binding constraint on every optimizer plan

Plans A, B, C, E all evaluate the same 4-D objective via the same `sweep_landing_centroid.m`. Any plan that does not shrink or share dSMC trials cannot change the budget. Plan F is the only plan that escapes — by amortizing across many `(p_target, λ)` re-runs at <10 ms per query — but only after Plans A or B have generated the cache.

This means: outer-optimizer math is not the binding cost. *Sample efficiency* — how many `sweep_landing_centroid` calls each plan needs — is. On smooth low-d objectives BO wins (tens of evals); on rugged or multi-modal landscapes CEM wins (hundreds). The decision criterion is the landscape, not the math.

### 3. `pyenv` brittleness is the shared dependency risk that decides plan composition

Plans A and E recommend `pyenv` (BoTorch / SMT 2.0). Plan F can adopt it as a fallback. Plans B and D Track 1 explicitly avoid it. Plan D Track 2 and Plan C with the chaospy fallback need it; PCE via UQLab does not.

If `pyenv` setup is brittle in the lab environment, Plans A and E degrade to MATLAB-only fallbacks (EI-plus / hand-rolled Le Gratiet) and Plan B's relative attractiveness rises. **Plan B's pure-MATLAB profile is the strongest hedge against toolchain risk.**

MOSEK (Plan D Track 2) is a different risk tier — commercial license on top of YALMIP-LMI tooling, not just environment management. Combined with the absence of any published cov-steering precedent for sliding-mode controllers, this justifies dropping Track 2.

### 4. Where does `p_target` enter operationally?

The research report (line 117 of [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md)) frames `Q` as a convolution `∫ R(x_t | x_d) p_target(x_t) dx_t`. **No plan addresses how `p_target` is supplied operationally.**

- Plan A invents `p_target` as a wrapper argument with a Gaussian default (line 27 of the plan).
- Plans B and D accept this without challenge.
- Plan C does not address it (consistent with its scope being the ballistic kernel only).
- Plan E inherits Plan A.
- Plan F mentions a closed-form Gaussian convolution at inference (line 33).

The existing `sweep_landing_centroid.m` already uses a power-weighted centroid with `p_centroid = 2`, which is a mild risk-emphasised target. **It is plausible that the existing power-weighted centroid is already the de facto `p_target` for the application,** in which case Plan D's CVaR retrofit adds less than the report implies. Confirm with the project owner before allocating Plan D effort.

### 5. The `U(x_d)` patch is small but no plan has applied it

Plans A, B, D, F all correctly identify that `ctrl_trial` (currently consumed only for the TO_06 figure in `sweep_landing_centroid.m`) is the right source for `U(launch)` — the expected control effort. The patch is one line. **No plan has actually applied it.** This should be the single first step of Phase 1; it unblocks all four optimizer plans simultaneously.

### 6. Citation corrections from the survey

- The research report attributes [arXiv 1910.02500](https://arxiv.org/abs/1910.02500) to "Devolder, Hewing, Lewkowycz et al." (lines 28, 53, 128). The actual authors are **Devonport & Arcak**, "Data-Driven Reachable Set Computation Using Adaptive Gaussian Process Classification and Monte Carlo Methods" (2019). Caught by Plan F's verification pass.
- The research report attributes [arXiv 2402.01370](https://arxiv.org/abs/2402.01370) to "Pezzato et al." (line 65). The actual authors are **Brudermüller et al.**, "CC-VPSTO: Chance-Constrained Via-Point-based Stochastic Trajectory Optimisation" (2024). Caught by Plan D's verification pass.
- The research report cites the Yao et al. *Progress in Aerospace Sciences* paper as 2016 (line 59). The canonical paper is 2011 ([DOI 10.1016/j.paerosci.2011.05.001](https://doi.org/10.1016/j.paerosci.2011.05.001)). The 2018 *PAS* paper is by **Yondo, Andrés, Valero**, a different work.

### 7. Inner-MC noise is the most underestimated cost across optimizer plans

Plans A, B, D Track 1, and F all evaluate noisy `J(θ)` from 84 trials with `σ_Q ≈ 0.05`. Plan A handles it with robust-KG (assuming the deterministic-vs-ξ issue above is resolved); Plan B's vanilla CEM does not; Plan D's CVaR at α=0.1 has ~30% std error on 9-of-84 elites; Plan F cannot drive RMSE below the MC floor. **All four plans' budgets assume 84 trials is enough; if not, all four double.** The cheapest mitigation is doubling inner count (84 → 168), at +2 min/eval.

## Recommended sequence (consolidated)

| Phase | Plan | Days | Wall | MVP gate | Dependency |
|-------|------|------|------|----------|------------|
| **1** | Patch `sweep_landing_centroid.m` to surface `ctrl_trial` integral and `miss_per_cell` | 0.5 | — | Unit tests pass | — |
| **1** | **Plan B (CEM)** | 7 | 6.7 h | Recommended `θ*` + `cem_log.mat` ≥120 sweeps | Pure MATLAB |
| **2** | **Plan A (BO/GP), EI-plus fallback** | +5 | 4.5 h | EI-plus result on stock `bayesopt`, side-by-side vs Plan B | MATLAB only |
| **2** | **Plan A (BO/GP), BoTorch+rKG** | +6 | 4.5 h | rKG result, ≥5% improvement vs EI-plus | `pyenv`, BoTorch ≥0.10 |
| **3** | **Plan D Track 1 (CVaR)** *if a worst-case requirement is articulated* | +5 | 0 h marginal | α-sweep over `bo_log.mat` / `cem_log.mat` | None new |
| **4** | **Plan C — Sobol' analysis only** *if ξ-treatment needs justification* | +3 | <1 h | Per-dimension Sobol' indices over `(launch, ξ)` | UQLab |
| **5** | **Plan E (multi-fidelity)** *if multi-mission re-runs confirmed* | +12 | save ~2 h/run | `ρ_S ≥ 0.6` on 6×6 grid | `pyenv`, SMT 2.0 |
| **6** | **Plan F (NN surrogate)** *after N ≥ 100 cached sweeps from A/B* | +7 | save ~2 min/query | RMSE within MC floor; ρ ≥ 0.85 | MATLAB Statistics & ML |
| — | ~~Plan C as full pipeline~~ | parked | — | — | — |
| — | ~~Plan D Track 2 (cov-steering)~~ | dropped | — | — | YALMIP+MOSEK, multi-month |

The combined Phases 1+2 produce two independent estimates of the optimum — `θ*_CEM` and `θ*_BO` — over the same simulator and cache. Cross-validation by agreement is the strongest possible MVP signal.

## Why this sequencing

Three orthogonal axes converged on B → A → D Track 1:

1. **J1 (compute & integration):** Plan B has zero new dependencies and a one-line patch. Plan A degrades safely to a MATLAB-only EI-plus fallback. Plan D Track 1 is a 5-day bolt-on. Plans C, D Track 2 need either commercial solvers or refactors that contradict the project framing.
2. **J2 (math soundness):** Plans C and D explicitly diagnose the deterministic-vs-ξ confusion that Plans A, B, E, F silently inherit. Plan D Track 1 is therefore the most mathematically honest reformulation of the cell-level statistic. Plan A's rKG choice is the most acutely affected by the unrecognised conflation, but the EI-plus fallback degrades safely.
3. **J3 (time-to-MVP & risk):** Plan B's 7 dev-days + 6.7 h wall is the fastest credible path to a recommended launch. Plan A on top reuses Plan B's cache as initial design, saving ~1 h sim wall. Plan D Track 1 retrofits both. Plan F is gated on N≥100, Plan E on `ρ_S ≥ 0.6` and multi-mission demand, Plan C parks itself.

The single point of disagreement among judges is small: J1 ranks Plan D Track 1 #2 (because it's a trivial bolt-on); J2 ranks it #1 (because it's the most mathematically honest); J3 ranks it #3 (because it's sequentially after A or B). All three agree that *if* Track 1 is done, it goes after A or B, not before.

## Risks specific to the recommended sequence

1. **Plan B noise floor.** Vanilla CEM does not model `σ_J ≈ 0.05` noise; the covariance refit can over-shrink toward whichever sample happened to win. Mitigation: covariance-floor smoothing per Kroese §3 ([Kroese tutorial](https://people.smp.uq.edu.au/DirkKroese/ps/aortut.pdf)) or score by `J_i − c·σ_i` (lower confidence bound). If late-iteration elite scatter shows two clusters, escalate to Bregman-Centroid Guided CEM.
2. **Plan A pyenv brittleness.** `pyenv` out-of-process is the recommended setup but adds serialization tax and a setup-day. Mitigation: the EI-plus path on stock `bayesopt` ships standalone; treat BoTorch as a swap-in for the acquisition step only. If pyenv blocks, EI-plus is publishable.
3. **Reachability surface non-stationarity at envelope edges.** Sharp drop-offs (crossing the dSMC stability boundary) can break the stationary GP. The existing `tot_w > 0` assertion in `sweep_landing_centroid.m:275` already errors on full failure; treat those as a binary infeasibility classifier and exclude from the GP regression. Deep-kernel GP fallback (Profile BO [arXiv 2512.23581](https://arxiv.org/abs/2512.23581)) is overkill at d=4.
4. **CEM and BO converging to *different* optima.** This is a feature, not a bug: a multi-modal objective surface would explain it. If observed, follow up with Bregman-CEM (Plan B day-7 conditional branch) and document both modes. Do *not* discard one to make the other look right.
5. **The structural-grid-vs-ξ conflation is not resolved by Plans A or B alone.** The recommended sequence treats the existing 84-cell sweep as the working objective. If the project later requires a true ξ-Monte-Carlo over `(w_z0, w_y0, c_Magnus, c_drag)`, Plan C's Sobol' analysis (Phase 4) becomes the natural justification step — and PCE may then earn its keep.

## What was *not* recommended and why

- **Plan C as a full pipeline.** PCE accelerates `eom2` (~50 ms; ~0.04% of wall). The dominant cost is dSMC (~30 s/eval/worker). The report's "10–100× speedup" (line 186) only holds if the dSMC reachability heatmap is *replaced* by an analytic deployment-moment cost — which contradicts the project's stated motivation for Plans A/B. The plan's own step-1 gate parks itself; respect it. *Keep* the Sobol' analysis as a 2–3 day sub-deliverable.
- **Plan D Track 2 (covariance steering).** No published precedent exists for cov-steering on a sliding-mode controller. The literature (Tsiotras' Mars descent papers; Ridderhof–Tsiotras [JGCD DOI 10.2514/1.G005400](https://arc.aiaa.org/doi/10.2514/1.G005400)) linearizes around a nominal trajectory under affine state feedback, not a sliding-surface law. The user's offline launch-parameter problem has no certifiable-bound deliverable to justify 4–8 dev-weeks of SDP/LMI tooling. Plan D's own §"When to pursue Plan D" recommends dropping it.
- **Plan F before N ≥ 100.** A 5000-parameter MLP on N=20 LHS rows is fitting noise. The plan itself enforces this gate; respect it. Until then, Plan A's GP serves the same caching role with calibrated posteriors.
- **Plan E without a multi-mission requirement.** Plan E saves ~2 h on Plan A's 4.3 h baseline for ~11 dev-days plus pyenv complexity. At one mission, the ROI is negative. At 5+ missions, it pays back.

## Open questions for the project owner

1. Does the application have a worst-case acceptance criterion (e.g., "P[miss > 30 m] < 10%") that justifies Plan D Track 1, or is the existing power-weighted centroid (`p_centroid = 2`) already the operational risk-emphasised target?
2. Is the existing 84-cell deterministic structural sweep the *intended* canonical objective, or is the eventual goal a true ξ-Monte-Carlo over `(w_z0, w_y0, c_Magnus, c_drag)`? This decision determines whether Plan C escapes its parking note and whether Plan A's robust-KG noise model is correctly aligned.
3. How many distinct `(p_target, λ)` re-runs are anticipated? <2 makes Plan F a marginal investment; ≥5 makes both Plan E and Plan F worth the budget.
4. Is `pyenv` available and stable in the lab environment? If no, Plan A degrades to EI-plus and Plans E/F lose half their attractiveness.

## Citations and provenance

- **Research report.** [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md). The four numbered Recommendations (lines 154–196), the cross-cutting sections (lines 138–148), the Caveats (lines 209–214), and the canonical objective (lines 117–124) are the load-bearing inputs for every plan.
- **Plans.** [Plan A](../plan_A_bo_gp_surrogate.md) (BO/GP), [Plan B](plan_B_cem_outer_loop.md) (CEM), [Plan C](plan_C_polynomial_chaos.md) (PCE), [Plan D](plan_D_cvar_covariance_steering.md) (CVaR + cov-steering), [Plan E](plan_E_multi_fidelity_surrogate.md) (multi-fidelity), [Plan F](plan_F_neural_reachability_surrogate.md) (NN surrogate).
- **Judging reports.** [Judge 1 — Compute & integration feasibility](judge_1_compute_integration.md), [Judge 2 — Mathematical soundness & theoretical fidelity](judge_2_math_soundness.md), [Judge 3 — Time-to-MVP & development risk](judge_3_time_to_mvp.md).
- **Local pipeline references.** [`sweep_landing_centroid.m`](../sweep_landing_centroid.m) (84-trial dSMC sweep, deterministic in ξ), [`centroid_lookup_table.m`](../centroid_lookup_table.m) (existing 20-LHS lookup-table generator), [`CLAUDE.md`](../CLAUDE.md) (project conventions, `parsim` wiring, dSMC architecture).
