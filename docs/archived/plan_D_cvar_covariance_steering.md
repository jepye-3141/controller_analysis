# Plan D — CVaR / Chance-Constrained Reformulation + Covariance-Steering Analysis

> Source: Recommendation 4 of [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 188–196.

## Overview

Plan D bundles two semi-independent ideas. **Track 1 (CVaR)** swaps the expected-hit objective for a tail-aware functional `CVaR_α(miss)` so the launch decision is robust to rare ballistic dispersions. It is a one-line reformulation on top of an already-running outer loop (Plan A or Plan B), and it does *not* touch dSMC, Simulink, or `parsim`. **Track 2 (covariance steering)** is a distinct, multi-month research project that would replace or augment dSMC with a controller whose terminal landing-state covariance is upper-bounded by an SDP solve. The two tracks share only their motivation (tail risk) and otherwise progress on independent timelines.

**Honest read up front.** The user's existing pipeline (`sweep_landing_centroid.m`) computes empirical hit rates `landing_ratio` and a `radial_profile` — both already capture some tail information. The current power-weighted centroid (`p_centroid = 2`) and `half_radius` summary are expected-value framings. **Track 1 is worth ~5 dev-days of follow-on work after Plan A/B is operational** if the user has a stated safety-critical requirement; it is *not* worth pursuing pre-emptively. **Track 2 is not justified by anything in the current research goal** (offline launch-parameter selection, ~80-trial inner sweep) and should be deferred or dropped.

## Mathematical formulation

### CVaR objective (Track 1)

For a loss `L(θ, ξ)` with `θ = (Vo, el, az, p)` and ballistic disturbance `ξ`, the [Rockafellar–Uryasev](https://sites.math.washington.edu/~rtr/papers/rtr179-CVaR1.pdf) formula is

```
CVaR_α(L; θ) = min_τ { τ + (1/α) E_ξ[(L(θ,ξ) − τ)+] }.
```

The candidate **loss** is per-trial miss distance `L_i = ‖p_land,i − p_target‖₂` (or any negative-of-utility), or the binary indicator `L_i = 1{trial i fails}`. The **sample-CVaR estimator** from `N` trials is closed form:

1. Sort `{L_i}` ascending.
2. Let `K = ⌈α · N⌉`.
3. `CVaR_α ≈ mean(L_(N−K+1), …, L_(N))` (mean of the worst `K`).

For `α = 0.1` (worst decile) and `N = 84`, `K = 9`. **This is the report's worry made concrete: 9 samples is exactly the order at which sample-CVaR has an O(1/√(αN)) standard error of ~30–40% of the CVaR value, dominated by tail rarity (Deo & Murthy 2021; Glasserman, Heidelberger, Shahabuddin 2000).** Three remediations are credible:

- **Bump inner-ensemble size.** Doubling `N` from 84 → 168 takes wall time from 2 → 4 min per launch. Halves CVaR variance. Cheapest fix; recommend this first if signal-to-noise is the binding constraint.
- **Bootstrap the tail.** `B = 1000` non-parametric bootstrap resamples of the existing 84-trial output give a confidence interval on `CVaR_α` essentially for free (microseconds per launch). Use BCa intervals for skewed losses (Bentum & Asante 2024). Bootstrap does *not* reduce variance — it merely quantifies it — but for an outer-loop optimizer trying to compare two `θ` values, knowing the CVaR uncertainty is decisive.
- **Importance sampling on `ξ`.** Re-sample wind / aero perturbations preferentially toward the failure manifold (Deo & Murthy 2021, "self-structuring" IS). Yields O(log²) variance reduction in theory but requires identifying which ξ-direction drives miss-distance — a 1–2-week study. *Not* on the critical path.

**Choice of α.** Defer α-selection to the user's risk tolerance; α ∈ {0.10, 0.20} is conventional. The report's "α = 0.1" example is illustrative, not prescriptive. Reporting CVaR at three α values from the same sorted list is free and informative.

### Where the heatmap tail comes from

The report's intuition needs sharpening. `sweep_landing_centroid.m` sweeps three things — deploy index `i ∈ {1..8}`, cross-track `k ∈ {1..3}`, neighbor offset `nb ∈ {1..4}` — totalling 96 nominal trials, of which ~84 are valid (boundary `nb` are dropped). Each (`i, k, nb`) cell is a *single deterministic dSMC trial* against a single ballistic trajectory. **The current sweep has zero ξ-randomness inside it** — `Vo, el, az, w_z0, w_y0, p` are scalars passed once into `eom2`. So `landing_ratio` is *not* a Monte Carlo estimate of P[hit] in the statistical sense; it is the fraction of the 84-trial deterministic grid where dSMC stabilized.

Two implications:

- **The "tail" of the miss distribution does not exist yet.** To run CVaR honestly, the user must either (a) randomize the inputs `(w_z0, w_y0, p)` etc. across trials *inside* the sweep, or (b) interpret the existing 84-cell grid as a worst-case envelope and apply CVaR over the cell-level miss distances `‖p_land − p_target‖`. The latter is a Conditional-Worst-Cell statistic, not a CVaR_α in the standard sense — it is what the existing `landing_ratio` captures already.
- **The per-trial miss distance `dist_to_target` exists in the code** ([`sweep_landing_centroid.m:223`](../sweep_landing_centroid.m)) but is consumed only inside the success indicator. Returning the per-trial *minimum* `dist_to_target` along the trajectory (or final-time miss) gives a continuous loss that supports a sample-CVaR over the 84 cells immediately.

**Concrete CVaR plumbing.** Modify `sweep_landing_centroid.m` (one line in the post-pass) to record `min(dist_to_target)` per cell into `out.miss_per_cell` (84-vector). Then `CVaR_α(miss; θ)` is a sort + mean over `out.miss_per_cell` and drops directly into Plan A's `bo_objective.m` or Plan B's `cem_objective.m`. **No new MC infrastructure needed for v1; only retire `mean(stable_arr)` in favor of the CVaR functional.**

### Covariance steering (Track 2, subordinate)

Tsiotras-style covariance steering ([Ridderhof & Tsiotras 2018](https://dcsl.gatech.edu/papers/aiaa17c.pdf)) solves: given an LTV system `x_{k+1} = A_k x_k + B_k u_k + w_k` with `w_k ~ N(0, W_k)` and a feedback law `u_k = K_k(x_k − μ_k) + v_k`, find `(K_k, v_k)` such that

- `μ_K = μ_target`,
- `Σ_K ⪯ Σ_target`,
- subject to chance constraints `P[h(x_k) ≥ 0] ≤ ε_k`,

minimizing a quadratic cost. The problem is convex in the SDP-tractable variables `K_k W_k^{1/2}` after a Schur-complement reformulation; standard solvers are YALMIP+MOSEK or CVX+SeDuMi.

**Three obstacles for the user's setting**:

1. **dSMC is fully nonlinear and discontinuous.** Saturation (per Plan A+ §3) and the priority-weighted allocation make the closed loop *non-smooth*, not just non-LTV. There is no published precedent for covariance steering on a sliding-mode controller; the existing literature (Tsiotras' Mars descent papers) linearizes around a nominal trajectory under affine state feedback, not a sliding-surface law. "Designing dSMC for bounded terminal covariance" is therefore not well-defined.
2. **The user's noise model is not Gaussian and not additive on the state.** Ballistic dispersion enters through the deployment IC (a 12-vector at apogee), then propagates through dSMC's reaching-law dynamics. The closed-loop terminal landing covariance is the pushforward through a 60-second nonlinear simulation — analytically intractable.
3. **MATLAB tooling overhead.** YALMIP+MOSEK is not currently a project dependency. MOSEK is commercial (academic license available); YALMIP is free. Installing both, learning the LMI calculus, and building the time-varying linearization around dSMC nominal trajectories is 4–8 dev-weeks before the first SDP runs.

**Three recommended tracks for "robust terminal landing covariance" (in increasing order of effort)**:

- **(i) Empirical covariance bound, no controller redesign.** From the existing sweep, compute the empirical covariance of `p_land` per `θ` cell, report its trace as the "spread" the launch optimizer should account for. **This is what the current `radial_profile.half_radius` already approximates.** Zero new SDP machinery. Recommended.
- **(ii) Outer-loop covariance-steering MPC wrapper around dSMC.** Compute a one-shot LTV linearization of dSMC's *closed-loop* dynamics around a nominal trajectory; design `K_k, v_k` as a *correction* on top of the dSMC command. Tractable but rebuilds the simulation harness. ~3 months. Not justified.
- **(iii) Replace dSMC with a covariance-steering MPC.** Drops the entire Plan A+ effort. Off the table.

Adopt **(i)** in v1. Reconsider **(ii)** only if the user explicitly states that a *certifiable* upper bound on `Σ_landing` is a deliverable.

## Required infrastructure

**Track 1 (CVaR)**:

- One-line patch to [`sweep_landing_centroid.m:233`](../sweep_landing_centroid.m) to surface `out.miss_per_cell`. Already-private MATLAB code knows everything needed.
- A new `cvar_objective.m` wrapper (~30 lines) that wraps the Plan A `bo_objective` / Plan B `cem_objective` and replaces the J-functional with `CVaR_α(miss) − λ U(θ)` (or a lexicographic ε-constraint). Optionally returns BCa bootstrap CI.
- No new toolboxes. `prctile`, `bootci` are in Statistics & Machine Learning Toolbox (already required for `bayesopt`).
- No Simulink touch-points. No `parsim` change. No SDP solver.

**Track 2 (covariance steering)**:

- YALMIP (free, MATLAB-native): https://yalmip.github.io/.
- MOSEK 10+ (commercial; academic license): https://www.mosek.com/. Required for chance-constrained SDP at any reasonable scale. SDPT3 / SeDuMi are slower drop-ins.
- A *new* analysis stack: linearization of dSMC about a nominal landing trajectory (Plan A+ already gives the closed-form per-channel A_d, so only the outer-loop position dynamics need linearizing).
- A *new* harness: replace the `parsim`-driven 84-trial inner sweep with an SDP solve per launch. The wall time per SDP solve at horizon `N=3000` (60 s × 50 Hz) is sub-minute on MOSEK 10+ for the LTI case; LTV with chance constraints is 1–10× slower. Net: still tractable but operationally a different beast.

## Integration with existing codebase

**Track 1 (CVaR)**:

- Drops cleanly into Plan A's `bo_objective.m` *after* Plan A v1 is operational. Recommend: ship Plan A with `J = E[Q] − λ U` first, then add `CVaR_α` as an alternate scoring function once the BO loop is debugged. Robust-KG handles tail-quantile objectives identically to expectation objectives.
- Drops cleanly into Plan B's `cem_objective.m` similarly. Note: **CEM elite selection on a CVaR score is more variance-prone than on an expectation**, because each evaluation's CVaR uses fewer effective samples. Compensate with covariance-floor smoothing (Plan B risk #1).
- The cached `bo_log.mat` / `cem_log.mat` schemas should add a `miss_per_cell` field so post-hoc CVaR-α sweeps over different α values are free (no resimulation).

**Track 2 (covariance steering)**: would require building a *separate* analysis pipeline that does not touch `sweep_landing_centroid.m`. The only shared artifact is the ballistic-IC distribution from `eom2`.

## Implementation roadmap

**Track 1 — recommended path (5 dev-days, after Plan A or B is operational)**:

1. **Day 1.** Patch `sweep_landing_centroid.m` to return per-cell miss distance. Add a unit test asserting `mean(miss_per_cell <= reach_tol) == reachability_pct` (sanity check vs the existing aggregator).
2. **Day 2.** Implement `cvar_objective.m`: wraps existing Plan A/B objective wrapper, computes sample-CVaR_α with chosen α, returns `(CVaR, σ_CVaR_bca, U)`. Unit-test against a synthetic input with a known CVaR.
3. **Day 3.** Synthetic heavy-tailed test. Generate a 4-D objective whose miss distribution is normal at the centre and Pareto-tailed at the edge (fat right tail near `el = 35°`, the launch boundary). Confirm CVaR-driven BO/CEM converges *away* from the heavy-tailed corner relative to expected-value-driven BO/CEM. Verifiable: the recommended `θ*` differs measurably between expected-value and CVaR runs on this synthetic.
4. **Days 4–5.** One real BO/CEM run with `α = 0.1` against the simulator. Compare to the Plan A or B baseline. Document.

**Track 2 — deferred**:

1. **Months 1–2.** Build LTV linearization of dSMC closed-loop about a nominal landing trajectory. Validate against a 100-trial dSMC ensemble. *Gate*: linearization residual < 10% of state magnitude over 60 s. If failed, abandon track 2 — covariance steering will not be useful.
2. **Month 3.** YALMIP+MOSEK SDP for one-shot covariance bound at fixed launch. Compare bound vs empirical sample covariance from the existing sweep.
3. **Month 4+.** Integrate as outer-loop wrapper or controller replacement.

The expected outcome of months 1–2 is that the linearization residual is large enough (>20%) to make covariance steering offer no certifiable bound advantage over empirical Monte Carlo. **This is an informed pessimism, not a proof.**

## Computational budget

- **Track 1 marginal cost.** Effectively zero. `prctile` + `bootci` over 84 numbers per launch is sub-millisecond. **Track 1 does *not* increase per-launch wall time** unless the user opts to double the inner ensemble (84 → 168) for variance reduction, which costs +2 min per launch.
- **Track 2.** YALMIP+MOSEK SDP at horizon `N=3000` is single-digit seconds for LTI, 30 s – 5 min for LTV with chance constraints (problem-dependent). The LTV linearization itself is the dominant cost: a single closed-loop dSMC run plus matrix bookkeeping is ~30 s/launch on top of the existing simulation. Roughly +1 min/launch in the outer-loop wrapper formulation.

## Validation strategy

- **Track 1 — synthetic heavy-tail benchmark.** Construct `J_test_heavy_tail(θ) = Z(θ) + 1{rand < 0.05} · Pareto(θ)` so 5% of trials draw from a fat-tailed loss conditional on `θ`. Confirm `argmax CVaR_0.1` differs from `argmax E[J]`. This validates the sort-and-mean estimator and the CI plumbing before any simulator runs. Verifiable: published heavy-tail synthetic produces measurable `θ*` shift.
- **Track 1 — bootstrap CI calibration.** For 100 synthetic launches, compute true CVaR (analytical) vs sample CVaR + BCa CI from N=84. Coverage should be ~95% at α=0.1 (Bentum & Asante 2024). If coverage drops below 90%, increase inner `N`.
- **Track 1 — closed-loop end-to-end.** Re-run Plan A/B at `α = {1.0, 0.5, 0.1}` (1.0 reduces to expected-value baseline) and compare recommended `θ*`. **Acceptance: at α=1.0 results match Plan A/B baseline exactly; at α=0.1 results shift toward higher elevation / lower velocity** (intuitively the safer corner of the launch envelope).
- **Track 2 — closed-loop MC verification of bounds.** For the SDP bound `Σ_target` at fixed `θ`, run 1000 MC dSMC trials and check `tr(empirical Σ_landing) ≤ tr(Σ_target)` with high empirical probability. **If the bound is loose by >5×, covariance steering is not certifying anything useful** and should be abandoned for track (i) above.

## Risks & mitigations

1. **CVaR estimator variance dominates BO/CEM signal at α=0.1.** Most acute risk for Track 1. The 9-of-84 worst decile has ~30% standard error. *Mitigation*: report bootstrap CI alongside CVaR; have the outer-loop optimizer weight by inverse variance (robust-KG handles this, vanilla CEM does not). Increase inner N from 84 to 168 if BO/CEM stalls. Alternative: pick larger α (e.g., 0.25), which trades tail focus for lower variance.
2. **The "tail" interpretation is mismatched between report and code.** Report assumes a stochastic ξ-ensemble per launch; code currently runs a deterministic 84-cell sweep over (deploy, cross-track, neighbor). *Mitigation*: be explicit about whether CVaR is over the cell-level grid (current code) or over a true random `ξ`-ensemble (requires nesting another randomization layer). The former is cheaper and answers a slightly different question (worst-cell-rather-than-worst-trial). Recommend the former for v1.
3. **Covariance steering is high-effort, low-payoff for this application.** The user's pipeline is offline; speed and certifiable bounds are not in the requirement. *Mitigation*: do not start track 2 unless safety-critical certification is added to the goal explicitly.
4. **Risk-averse `θ*` may overfit to the 84-cell grid.** A grid-CVaR optimum that hugs the boundary of the deterministic grid may collapse under genuine ξ-randomness later. *Mitigation*: hold out a high-MC validation set per Plan A's day-13 protocol but at the worst-decile level rather than the mean.
5. **Risk-aware sequencing alignment.** The user's [`sweep_landing_centroid.m:271`](../sweep_landing_centroid.m) uses `p_centroid = 2` for the power-weighted centroid — already a mild risk-emphasis (concentrates weight on high-reachability points). The existing pipeline is *not* fully expected-value framed; it has soft tail emphasis. **Plan D's marginal value over the current setup is therefore smaller than the report implies.** Confirm risk requirements before allocating Track 1 effort.

## When to pursue Plan D

- **Pursue Track 1 (CVaR)** *only after* Plan A or B is operational, *only if* the user can articulate a worst-case acceptance criterion (e.g., "P[miss > 30 m] < 10%" rather than "minimize average miss"). The CVaR sort-and-mean estimator is so cheap that retrofitting it onto a working BO/CEM loop is essentially free; pre-emptively building it is misallocated effort.
- **Defer Track 2 (covariance steering)** indefinitely. The user's application is offline launch parameter selection, not certified safety bounds on a deployed controller. The Tsiotras line of work is targeted at Mars EDL, where certifiable terminal-state covariance is a contractual deliverable. The user's pipeline already produces empirical landing covariance via `radial_profile`; that is sufficient for tradespace exploration.
- **Aligned with the user's current framing?** No. The lookup-table generator, the power-weighted centroid (`p_centroid = 2`), and the empirical reachability heatmap are all expected-value or mildly-tail-emphasized framings. The user's framing **is right for the application** unless the project later acquires a certified-safety deliverable. **Plan D should be queued behind Plan A and Plan B, not run in parallel.**

## Citations

Research-report grounding:
- [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 188–196 (Recommendation 4); related context lines 24, 65, 79.

CVaR foundational and practical:
- Rockafellar, R. T. & Uryasev, S. (2000). "Optimization of Conditional Value-at-Risk." *Journal of Risk*, 2(3), 21–42. [PDF](https://sites.math.washington.edu/~rtr/papers/rtr179-CVaR1.pdf). DOI [10.21314/JOR.2000.038](https://www.risk.net/journal-risk/2161159/optimization-conditional-value-risk). Closed-form min-formula for sample-CVaR optimization.
- Deo, A. & Murthy, K. (2021). "Efficient Black-Box Importance Sampling for VaR and CVaR Estimation." [arXiv:2106.10236](https://arxiv.org/abs/2106.10236). Self-structuring IS for tail estimation; quantifies the O(1/β) sample-blowup at quantile β.
- Glasserman, P., Heidelberger, P. & Shahabuddin, P. (2000). "Variance Reduction Techniques for Estimating Value-at-Risk." *Management Science* 46(10) 1349–1364. Foundational IS-for-VaR reference.
- Bentum, W. & Asante, A. A. (2024). "Bootstrap Confidence Intervals for Small Samples: A Comprehensive Monte Carlo Simulation Study." [SSRN 5771802](https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5771802). BCa method for n<40 performance.

Chance-constrained MPC (cited in report; Track 1 context):
- Liu, Z., Ma, L. & Chen, Y. (2025). "Trajectory Optimization of Stochastic Systems under Chance Constraints via Set Erosion." [arXiv:2504.04705](https://arxiv.org/abs/2504.04705). Set-erosion conversion of chance constraints to deterministic safety constraints — relevant if the user later wants chance-constrained outer formulations rather than CVaR.
- Brudermüller, L., Berger, G., Jankowski, J., Bhattacharyya, R., Jungers, R. & Hawes, N. (2024). "CC-VPSTO: Chance-Constrained Via-Point-based Stochastic Trajectory Optimisation." [arXiv:2402.01370](https://arxiv.org/abs/2402.01370). Note: the [research report's "Pezzato et al." attribution](ballistic_targeting_optimization.md) at this arXiv ID is **incorrect**; the authors are Brudermüller et al.
- Ren, K., Chen, C., Sung, H., Ahn, H., Mitchell, I. M. & Kamgarpour, M. (2024). "Recursively Feasible Chance-Constrained Model Predictive Control under Gaussian Mixture Model Uncertainty." [arXiv:2401.03799](https://arxiv.org/abs/2401.03799). GMM-CC-MPC.

Covariance steering (Track 2):
- Yin, J., Zhang, Z., Theodorou, E. & Tsiotras, P. (2021/2022). "Trajectory Distribution Control for MPPI using Covariance Steering" (CC-MPPI). [arXiv:2109.12147](https://arxiv.org/abs/2109.12147). IEEE [Xplore 9811615](https://ieeexplore.ieee.org/document/9811615/). Combines MPPI with covariance steering — relevant *only* if dSMC is replaced with MPPI (out of scope for this user).
- Ridderhof, J. & Tsiotras, P. (2018). "Uncertainty Quantification and Control during Mars Powered Descent and Landing using Covariance Steering." AIAA GNC Conf., AIAA 2018-0611. [PDF](https://dcsl.gatech.edu/papers/aiaa17c.pdf). The primary Tsiotras-style precedent.
- Ridderhof, J. & Tsiotras, P. "Minimum-Fuel Closed-Loop Powered Descent Guidance with Stochastically Derived Throttle Margins." *J. Guidance, Control, and Dynamics*. [DOI 10.2514/1.G005400](https://arc.aiaa.org/doi/10.2514/1.G005400). Saturated-actuator covariance steering — closest precedent for the user's saturated-dSMC question, but still on a smooth nonlinear plant rather than sliding-mode law.
- Greco, C., Campagnola, S. & Vasile, M. (2020). "Robust Space Trajectory Design using Belief Stochastic Optimal Control." AIAA SciTech 2020 Forum, AIAA 2020-1471. [PDF](https://strathprints.strath.ac.uk/71168/). Belief-MDP framework over uncertainty distributions; alternative to covariance steering.
- Balci, I. M., Bakolas, E., Vlahov, B. & Theodorou, E. (2021). "Constrained Covariance Steering Based Tube-MPPI." [arXiv:2110.07744](https://arxiv.org/abs/2110.07744). Tube-MPPI variant with safety guarantees.

Probability-of-hit / kill-probability:
- Mudrik, L. & Oshman, Y. (2026). "Kill-Probability-Maximization Guidance: Breaking from the Miss-Distance-Minimization Paradigm." [arXiv:2604.17811](https://arxiv.org/abs/2604.17811). The "smooth lethality" objective the report cites; supports CVaR as a natural risk-aware extension.

MATLAB tooling:
- YALMIP solver list: [https://yalmip.github.io/allsolvers/](https://yalmip.github.io/allsolvers/). Confirms MOSEK is the recommended SDP backend.
- MOSEK SDO toolbox: [https://docs.mosek.com/10.0/toolbox/tutorial-sdo-shared.html](https://docs.mosek.com/10.0/toolbox/tutorial-sdo-shared.html).
- MATLAB Statistics & Machine Learning Toolbox (`bootci`, `prctile`) — already required by Plan A.
