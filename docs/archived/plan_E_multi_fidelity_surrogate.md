# Plan E — Multi-Fidelity Surrogate Stack

> Source: Cross-cutting from Section C and Recommendations 1–2 of [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md), specifically lines 127–131 (multi-fidelity BO when 3-DoF point-mass and 6-DoF + Magnus models are both available).

## Overview

Plan E is not a standalone optimizer. It is a **drop-in replacement for the per-iteration objective evaluator** used by [Plan A](../plan_A_bo_gp_surrogate.md) (BO/KG) or [Plan B](plan_B_cem_outer_loop.md) (CEM), in which a cheap proxy of `J(launch)` is queried many times for every expensive ground-truth call. The acquisition over the 4-D launch space `(Vo, el, az, p)` stays unchanged; only the evaluator is augmented from one fidelity tier to two. The high-leverage cheap fidelity is **not** a 3-DoF point-mass replacement for `eom2.m` — that saves seconds out of a 2-min budget. The high-leverage cheap fidelity is a **reduced-sweep `sweep_landing_centroid_lo` (1×1×1 grid, ≈10 s)** plus optionally a closed-form LQR reachability proxy. Treat Plan E as a pure-MATLAB extension to either Plan A or Plan B, switched on once Plan A's GP shows >50 % evaluation cost is spent on launch points the cheap proxy could have screened out.

## Mathematical formulation

Let `J_hi(θ)` be the existing 84-trial dSMC sweep objective and `J_lo(θ)` the cheap proxy. Use **the recursive auto-regressive (AR1) co-Kriging model** of Kennedy & O'Hagan (Biometrika 2000) reformulated by [Le Gratiet & Garnier (IJUQ 2014)](https://arxiv.org/abs/1210.0686) and implemented in [SMT 2.0 as MFK](https://smt.readthedocs.io/en/latest/_src_docs/applications/mfk.html):

  `f_hi(θ) = ρ(θ) · f_lo(θ) + δ(θ)`,    `f_lo, δ ~ independent GPs`,   `ρ(θ)` either constant or a GP in θ.

In the recursive form, fitting reduces to two independent kriging problems on a nested DOE `D_hi ⊆ D_lo`. The posterior at the high fidelity is the same as the original Kennedy–O'Hagan formulation but with O(s · n³) instead of O((s · n)³) cost.

Acquisition over `θ` for the outer loop chooses **both the next launch and the next fidelity**:

- For Plan A: replace `qKnowledgeGradient` with **MF-MES** ([Takeno et al., ICML 2020, arXiv:1901.08275](https://arxiv.org/abs/1901.08275)) or **BOCA** ([Kandasamy et al., ICML 2017, arXiv:1703.06240](https://arxiv.org/abs/1703.06240)). Both pick `(θ*, fidelity*)` jointly. MF-MES has analytic forms for the cross-fidelity entropy that are cheap to evaluate at d=4 and a discrete two-tier setup, and a public PyTorch impl ([takeuchi-lab/MF-MES](https://github.com/takeuchi-lab/MF-MES)).
- For Plan B: leave CEM unchanged. The cheap proxy is used as a **pre-screen**: sample `N_pre = 4·N_pop` candidates at low fidelity, retain the top `N_pop` by `J_lo`, evaluate those at high fidelity. This is the simplest possible multi-fidelity layer and requires no co-Kriging code.

## Required infrastructure

- **Cheap simulator (single tier, picked from below)**:
  1. **Reduced-sweep wrapper `sweep_landing_centroid_lo.m`** — same code path, but `n_deploy = 1`, `n_ct = 1`, `n_nb = 1` (i.e. one centred trial at the 50 % deploy). Wall ≈ 10 s/eval (one Simulink build dominates over the single trial). Output: a binary `stable` flag interpolated against `p_target` mass at the deploy's nominal landing point. **Recommended primary cheap fidelity.**
  2. **Closed-form LQR reachability proxy** — replace the dSMC powered-phase Simulink call with a discrete-time LQR Lyapunov-stability surrogate. Per-deploy cost: a single matrix solve, ≈ 1 ms. The discrete LQR gains `Kp_d, Ki_d` are already computed in `constants.m` (CLAUDE.md: "Architecture" section). The LQR's reachable set under a saturated control `‖u‖ ≤ u_max` is the sub-level set of the steady-state Riccati cost, which gives a closed-form yes/no per landing point. **High variance vs the dSMC; use only as a *second* fidelity tier underneath the reduced sweep, never as the only fidelity.**
  3. **3-DoF point-mass propagator** — Plan E rejects this as a primary cheap fidelity. The existing `eom2.m` already runs in ~3 s; replacing it shaves ~2 s out of the 120-s wall. Rank correlation impact: zero, because the ballistic phase isn't where launch-parameter rankings diverge — the powered-phase reachability dominates the spread of `J(θ)`. Mention only in "future work."
- **High-fidelity simulator (existing)**: [`sweep_landing_centroid.m`](../sweep_landing_centroid.m), unchanged. Two-min wall, 84 inner trials.
- **Multi-fidelity BO library**:
  - **Recommended**: [SMT 2.0](https://github.com/SMTorg/smt) via MATLAB's `pyenv` ([Saves et al. 2024, arXiv:2305.13998](https://arxiv.org/abs/2305.13998)). Its `MFK` class is the AR1 model recursively. Plan A already provides for `pyenv` in its day-8 step, so the Python toolchain is reused. SEGOMOE [(Bartoli et al. arXiv:2504.09930)](https://arxiv.org/abs/2504.09930) provides MF on top, but its license is mixed and it adds complexity over plain SMT MFK.
  - **Hand-rolled fallback in pure MATLAB**: 4 GP fits with `fitrgp` (Statistics & Machine Learning Toolbox) plus the Le Gratiet recursive equations (≈100 lines). Forrester–Sóbester–Keane [(Proc. R. Soc. A 2007, DOI 10.1098/rspa.2007.1900)](https://royalsocietypublishing.org/doi/10.1098/rspa.2007.1900) is the textbook with the equations spelled out. Acceptable if `pyenv` is contraindicated; **not** recommended over SMT for MFBO acquisition — MF-MES/BOCA implementations are non-trivial.
  - **Pre-screen-only fallback (zero new dependencies)**: for Plan B coupling, a `for k = 1:N_pre, J_lo(k) = sweep_landing_centroid_lo(θ_k); end` loop is sufficient. No GP, no co-Kriging.

## Integration with existing codebase

- New file `sweep_landing_centroid_lo.m` is a thin wrapper around the existing function with hard-coded `n_deploy = 1; n_ct = 1; n_nb = 1` and target/landing logic frozen at the 50 %-arc deploy. Reuses the same `parsim` plumbing; `static_vars` list (CLAUDE.md "parsim wiring" caution) is identical, so no Simulink dependency surgery.
- The existing 4-worker `parpool` runs the cheap call in ≈ 10 s — or, if a 4-trial cheap fidelity is wanted, `parsim` returns in ~10 s anyway because the 4 trials saturate the workers.
- Plan A coupling: `bo_objective.m` becomes `bo_objective.m` (high) + `bo_objective_lo.m` (low). The acquisition code calls one or the other based on the MF acquisition's fidelity choice. The `bo_log.mat` schema gains a `fidelity ∈ {hi, lo}` column.
- Plan B coupling: `cem_run.m` adds an outer pre-screen step. No co-Kriging needed.
- The existing `centroid_lookup_log.mat` (20 LHS samples) is **not directly reusable as low-fidelity data** because it was generated with the full 84-trial sweep. It contributes 20 high-fidelity samples to the AR1 model's `D_hi`. A separate fast LHS run (e.g. 80 cheap samples in ≈14 min) populates `D_lo`.

## Implementation roadmap

Single cheap fidelity, validate before adding more.

1. **Day 1: build `sweep_landing_centroid_lo.m`.** Copy `sweep_landing_centroid.m`; gate the sweep loops on `n_deploy = n_ct = n_nb = 1` early returns; reuse the `Q` and `U` extraction logic from Plan A's `bo_objective.m`. **Verifiable**: on a single launch point, returns in ≤ 15 s and produces a finite `J_lo`.
2. **Days 2–3: cheap-fidelity validation grid.** Pick a 6×6 grid in `(Vo, el)` at fixed `(az=15, p=0)`. Evaluate both `J_lo` and `J_hi`. Report **Spearman rank correlation `ρ_S(J_lo, J_hi)`**. Threshold for go/no-go on Plan E: `ρ_S ≥ 0.6` ([Best practices for MFBO in materials, *Nature Computational Science* 2025](https://www.nature.com/articles/s43588-025-00822-9), and ranking-fidelity calibration practice). Below 0.4, the cheap fidelity is unreliable and Plan E reduces to extra noise; abort and stay with Plan A or B vanilla. **Verifiable**: a saved scatter `J_lo` vs `J_hi` and the printed ρ_S.
3. **Days 4–6: SMT MFK wiring.** Set up `pyenv` (reuses Plan A day-8 work). Fit MFK on the 6×6+20 dataset. Cross-validate the high-fidelity posterior leave-one-out. **Verifiable**: posterior-predictive log-likelihood at high fidelity comparable to or better than a single-fidelity GP fit on the 20 LHS samples alone.
4. **Days 7–9: MF-MES acquisition driver `bo_run_mf.m`.** Wraps SMT MFK + an MF-MES acquisition implemented from [Takeno et al. arXiv:1901.08275](https://arxiv.org/abs/1901.08275) (or imported from [takeuchi-lab/MF-MES](https://github.com/takeuchi-lab/MF-MES)). On the synthetic 4-D quadratic test from Plan A day-7, confirm MFBO recovers the optimum in fewer high-fidelity evaluations than vanilla KG. **Verifiable**: ≥ 30 % reduction in HF-eval count at fixed regret on the synthetic.
5. **Day 10: full MFBO run.** 80 LF + 40 HF total budget (see budget below); persist to `bo_log_mf.mat`. **Verifiable**: recommended `θ*` and a high-fidelity `J(θ*)` reported.
6. **Day 11: documentation & comparison.** Add an entry to `docs/README.md` and a head-to-head row in the Plan A/B comparison table: wall time, HF evals, final `J*`, and `ρ_S` of the cheap fidelity used.

Total: ≈ 11 dev-days. Days 1–3 are gating: if rank correlation fails, stop here and the loss is small.

## Computational budget

The leverage point is hf-eval count, not wall time per eval (the cheap eval is still 10 s; only ~10× cheaper than the 120 s hf eval, not 1000×).

- **Cheap eval**: ≈ 10 s wall on the 4-worker pool (single Simulink build dominates). Internal MC noise is high — one trial gives `Q ∈ {0, hit}` rather than a 0..1 ratio. Workaround: bump cheap `n_nb = 4` so the cheap eval averages 4 trials at one deploy, ≈ 10 s wall but `σ_J_lo` shrinks `≈ 2×`.
- **Plan A vanilla baseline**: 30 LHS + 100 KG = 130 HF evals × 2 min = **260 min ≈ 4.3 h** (Plan A day-12 figure).
- **Plan E with MFBO (target operating point)**:
  - 80 LF samples × 10 s = 800 s ≈ 13 min.
  - 30 HF LHS + 30 HF acquisition steps = 60 HF evals × 2 min = 120 min.
  - Total: **≈ 2.2 h**, roughly half of vanilla Plan A. The wall-time saving is ≈ 2 h.
- **Plan E with CEM pre-screen (Plan B coupling)**:
  - Per generation: pre-screen `4 × N_pop = 160 cheap evals × 10 s = ≈ 27 min`, then `N_pop = 40 HF × 2 min = 80 min`. Total per generation ≈ 107 min vs vanilla CEM's 80 min — *slower* unless the pre-screen lets us shrink `N_pop` to 20 elites. Recommended: shrink `N_pop_HF = 20`, total ≈ 67 min/gen, and accept slightly noisier elite refits. 5 generations: 5.6 h vs vanilla CEM's 6.7 h. Marginal.

**Honest read**: Plan E saves ≈ 2 h on Plan A and ≈ 1 h on Plan B. Whether that's worth ≈ 11 dev-days plus pyenv complexity depends on how many `(p_target, λ)` re-runs are anticipated. If exactly one mission, **don't do Plan E** — run Plan A or B. If 5+ scenarios, the amortized saving (10 h) starts to pay back.

> Sketch for the prompt's "200 mixed-fidelity in 40 min vs 20 high-fidelity in 40 min" hypothesis: 200 LF × 10 s = 33 min ≈ 40 min, ✓. But 200 LF samples without HF anchors is **not** a useful surrogate for the HF objective unless `ρ_S ≥ 0.7` — at `ρ_S = 0.6` the AR1 posterior at the HF level is dominated by the 0–5 HF samples, not the 200 LF. Realistic mixed-fidelity claim at 40 min wall: ~150 LF + 5 HF = 25 min + 10 min = 35 min, with HF posterior tighter than 5-LHS-only.

## Validation strategy

The MFBO assumption is that low-fidelity rankings of launch candidates correlate with high-fidelity rankings, *not* that values agree.

- **Day-2 Spearman ρ_S validation grid (mandatory).** 36 paired evaluations on a 6×6 product grid. Report ρ_S, Kendall τ, and the residual `r(θ) = J_hi − ρ̂ J_lo` distribution. Thresholds: ρ_S ≥ 0.6 → proceed; 0.4 ≤ ρ_S < 0.6 → restrict cheap fidelity to a sub-region (e.g. only `el ∈ [40, 50]`) and re-validate; ρ_S < 0.4 → abort.
- **Top-k rank recovery test.** From the 36-point grid, the top-5 by `J_hi` should overlap with the top-10 by `J_lo` in ≥ 4 of 5 entries. This is the operationally meaningful check for the CEM pre-screen.
- **Heteroscedastic-residual check.** The AR1 model assumes `δ(θ)` is a stationary GP. Plot `|J_hi − ρ̂ J_lo|` vs each launch dimension; if the residual variance scales with `Vo` or `el`, fit a separate length-scale per dimension or reject the cheap fidelity in those regions.
- **Held-out HF cross-validation.** Leave-one-out on the HF set, re-fit MFK each time, record posterior-predictive log-likelihood. Acceptable: matches single-fidelity GP CV on the HF-only set, plus reduced posterior variance.
- **Reproducibility.** Persist `rng` state and SMT random seed alongside `bo_log_mf.mat`.

## Risks & mitigations

1. **Cheap fidelity produces high-confidence wrong answers (the central MFBO failure mode).** With a single dSMC trial per cheap eval, MC noise can flip the sign of `J_lo − J_lo'` between two near-equal `θ`. The MFK model will then learn a `δ(θ)` that fits the noise. *Mitigation*: bump cheap `n_nb` to 4 (still ≈ 10 s); set the SMT MFK noise variance to the empirical `σ²` of the 36-point grid; consult [rMFBO (Mikkola et al. arXiv:2210.13937)](https://arxiv.org/abs/2210.13937) for explicitly robust MFBO when the LF source is unreliable.
2. **Local correlation only** (the [Foumani et al. *J. Mech. Des.* 2024](https://asmedigitalcollection.asme.org/mechanicaldesign/article/146/6/061703) failure mode). The cheap fidelity may track HF in `(el, Vo)` but diverge in `(az, p)` because the dSMC's lateral authority differs from the LQR's. *Mitigation*: stratify the day-2 validation grid across the four launch dims, not just two; if local correlation is detected, fit a non-stationary GP for `δ(θ)` (deep kernel) or restrict cheap fidelity to the in-correlation region.
3. **Pyenv brittleness**. SMT's MFK and the MF-MES PyTorch code add a second Python dependency on top of Plan A's BoTorch path. *Mitigation*: gate Plan E on Plan A's pyenv being already running; if Plan A used the EI-plus MATLAB-only fallback, do not pursue Plan E with SMT — use the hand-rolled Le Gratiet recursive co-Kriging in MATLAB or the CEM pre-screen variant.
4. **Engineering complexity vs marginal saving**. Honest assessment: at ≈ 2 h saved per Plan A run for ≈ 11 dev-days of work, Plan E pays back at ~5 missions if engineer time is valued at simulator-wall parity. *Mitigation*: do not propose Plan E as v1. Plan A or B should produce the headline result first; Plan E is a follow-up if multiple `(p_target, λ)` scenarios are confirmed downstream and the pyenv path from Plan A is already operational.
5. **The "wrong cheap fidelity" risk dominates everything else**. The 3-DoF point-mass propagator was tempting (cheap, well-precedented in [Hainz & Costello, JGCD 2005, DOI 10.2514/1.8027](https://arc.aiaa.org/doi/10.2514/1.8027)) but doesn't address the bottleneck — the powered-phase Simulink sweep — and a 3-DoF substitute will rank-correlate with `J_hi` only through the deploy state, not through the dSMC reachability map. *Mitigation*: explicitly do **not** use a 3-DoF cheap fidelity in v1. The reduced-sweep cheap fidelity (option 1 above) shares the dSMC physics with the HF and is therefore the only candidate with credible rank correlation in `(az, p)`.

## Citations

Research-report grounding:

- [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 127–131 (multi-fidelity BO with 3-DoF/6-DoF), 154–164 (Recommendation 1, BO/GP), 166–175 (Recommendation 2, CEM), 210 (multi-fidelity BO when heatmap is itself expensive).

External references (verified via web fetch):

- Kandasamy, K., Dasarathy, G., Schneider, J., & Póczos, B. (2017). "Multi-fidelity Bayesian Optimisation with Continuous Approximations." ICML 2017. [arXiv:1703.06240](https://arxiv.org/abs/1703.06240); [PMLR](https://proceedings.mlr.press/v70/kandasamy17a.html). BOCA acquisition that picks (θ, fidelity) jointly.
- Kennedy, M. C., & O'Hagan, A. (2000). "Predicting the output from a complex computer code when fast approximations are available." *Biometrika* 87, 1–13. Foundational AR1 co-Kriging. (Cited via Le Gratiet 2014 below.)
- Le Gratiet, L., & Garnier, J. (2014). "Recursive co-kriging model for design of computer experiments with multiple levels of fidelity." *International Journal for Uncertainty Quantification* 4(5), 365–386. [arXiv:1210.0686](https://arxiv.org/abs/1210.0686). Recursive AR1 reformulation — same posterior as Kennedy–O'Hagan, decoupled to s independent kriging fits.
- Forrester, A. I. J., Sóbester, A., & Keane, A. J. (2007). "Multi-fidelity optimization via surrogate modelling." *Proceedings of the Royal Society A* 463, 3251–3269. [DOI 10.1098/rspa.2007.1900](https://royalsocietypublishing.org/doi/10.1098/rspa.2007.1900). Co-Kriging optimization classic; equations directly usable for hand-rolled MATLAB fallback.
- Forrester, A. I. J., Sóbester, A., & Keane, A. J. (2008). *Engineering Design via Surrogate Modelling: A Practical Guide* (Wiley). Textbook reference for the same equations.
- Yondo, R., Andrés, E., & Valero, E. (2018). "A review on design of experiments and surrogate models in aircraft real-time and many-query aerodynamic analyses." *Progress in Aerospace Sciences* 96, 23–61. [DOI 10.1016/j.paerosci.2017.11.003](https://www.sciencedirect.com/science/article/abs/pii/S0376042117300611). Survey covering multi-fidelity DOE in aerospace.
- Saves, P., Lafage, R., Bartoli, N., Diouane, Y., et al. (2024). "SMT 2.0: A Surrogate Modeling Toolbox with a focus on Hierarchical and Mixed Variables Gaussian Processes." *Advances in Engineering Software*. [arXiv:2305.13998](https://arxiv.org/abs/2305.13998); [GitHub](https://github.com/SMTorg/smt); [MFK docs](https://smt.readthedocs.io/en/latest/_src_docs/applications/mfk.html). Recommended Python implementation.
- Bartoli, N., Lefebvre, T., Lafage, R., Saves, P., et al. (2024). "Multi-objective Bayesian Optimization with Mixed-Categorical Design Variables for Expensive-to-Evaluate Aeronautical Applications." [arXiv:2504.09930](https://arxiv.org/abs/2504.09930). SEGOMOE; supports mixed-fidelity on top of SMT.
- Takeno, S., Fukuoka, H., Tsukada, Y., Koyama, T., et al. (2020). "Multi-fidelity Bayesian Optimization with Max-value Entropy Search and its Parallelization." ICML 2020. [arXiv:1901.08275](https://arxiv.org/abs/1901.08275); [PMLR](https://proceedings.mlr.press/v119/takeno20a.html); [code](https://github.com/takeuchi-lab/MF-MES). MF-MES acquisition with analytic cross-fidelity entropy.
- Mikkola, P., Martinelli, J., Filstroff, L., & Kaski, S. (2023). "Multi-Fidelity Bayesian Optimization with Unreliable Information Sources." [arXiv:2210.13937](https://arxiv.org/abs/2210.13937). rMFBO; mitigates the "high-confidence wrong answer" failure mode.
- Foumani, Z. Z., Yousefpour, A., Shishehbor, M., & Bostanabad, R. (2024). "Safeguarding Multi-Fidelity Bayesian Optimization Against Large Model Form Errors and Heterogeneous Noise." *J. Mech. Des.* 146(6), 061703. [DOI 10.1115/1.4064160](https://asmedigitalcollection.asme.org/mechanicaldesign/article/146/6/061703/1171649). Local-correlation MFBO failure mode.
- "Best Practices for Multi-fidelity Bayesian Optimization in Materials and Molecular Research." *Nature Computational Science* 2025. [DOI 10.1038/s43588-025-00822-9](https://www.nature.com/articles/s43588-025-00822-9). Practical ρ_S thresholds for cheap-fidelity selection.
- Hainz, L. C., & Costello, M. (2005). "Modified Projectile Linear Theory for Rapid Trajectory Prediction." *Journal of Guidance, Control, and Dynamics* 28(5), 1006–1014. [DOI 10.2514/1.8027](https://arc.aiaa.org/doi/10.2514/1.8027). Rejected as cheap fidelity for Plan E (rationale in §"Risks").
- Ilg, M., Rogers, J., & Costello, M. (2011). "Projectile Monte-Carlo Trajectory Analysis Using a Graphics Processing Unit." AIAA Atmospheric Flight Mechanics Conference, AIAA 2011-6266. [DOI 10.2514/6.2011-6266](https://arc.aiaa.org/doi/10.2514/6.2011-6266). GPU-batch ballistic MC; relevant only if a future revision moves to a 3-DoF GPU cheap fidelity.
- McCoy, R. L. (2012). *Modern Exterior Ballistics: The Launch and Flight Dynamics of Symmetric Projectiles* (2nd ed.). Schiffer. Modified point-mass model definition.
- Plan A: [`docs/plan_A_bo_gp_surrogate.md`](../plan_A_bo_gp_surrogate.md). Pyenv setup, `bo_objective.m` schema, and the 4.3-h vanilla-BO baseline that Plan E targets for reduction.
- Plan B: [`docs/plan_B_cem_outer_loop.md`](plan_B_cem_outer_loop.md). CEM driver pre-screen integration target.
