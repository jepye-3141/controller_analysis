# Deterministic Nonlinear Optimization — Approach (iteration 1)

> Down-selected from [`comparative_survey_targeting_optimization.md`](archived/comparative_survey_targeting_optimization.md). [Plan A (BO/GP surrogate)](plan_A_bo_gp_surrogate.md) supplies the active reference for the Kriging machinery (acquisition layer not needed in the deterministic case). Plans B–F archived to `docs/archived/`; Plans B (CEM) and D Track 1 (CVaR) banked there for the eventual stochastic phase.

## Decision

The current `sweep_landing_centroid.m` pipeline is **deterministic in ξ** (single call to `eom2` per launch — see [`comparative_survey_targeting_optimization.md`](archived/comparative_survey_targeting_optimization.md) Cross-cutting Finding #1). Until wind, aero, and IC dispersions are added, there is no Monte-Carlo signal for the outer optimizer to average over, and CEM / CVaR / Bayesian-optimization-with-heteroscedastic-noise machinery is solving a problem we do not have yet.

The right framing for the present problem is **deterministic black-box nonlinear optimization over a low-dimensional bounded launch space**, where:

- **Decision vector** `θ = (Vo, el, az, p) ∈ R^4` (box from [`centroid_lookup_table.m`](../centroid_lookup_table.m) lines 8–14). `w_z0, w_y0` are exit-attitude rates — for now decision variables, later promotable to disturbances.
- **Outputs of interest** (two, not one):
  1. **Landing centroid position** `p_centroid(θ) ∈ R^2` — where dSMC will land the swarm on average.
  2. **Centroid area / sharpness** — how wide the high-reachability zone is around the centroid. The existing `radial_profile.half_radius` is the natural summary statistic; the full `radial_profile.mean_ratio` curve is the underlying object.
- **No stochasticity** in the inner sweep right now. One call to `sweep_landing_centroid` returns deterministic `(p_centroid, reachability_pct, radial_profile)` for a given `θ`.

## Recommended approach: Kriging surrogate over the existing LHS, on-surrogate optimization

> **Status (2026-05-12):** Implemented as [`surrogate_optimize.m`](../surrogate_optimize.m). The sketch below is preserved as the design narrative; the actual driver matches it modulo (i) the field-name workaround for the `centroid_lookup_table.m` save-shadowing quirk, and (ii) a NaN-clamp on `half_radius` (`y_hr(isnan) = 1.2 * max`) in lieu of the integrated-reach metric proposed below — the latter remains the cleaner formulation for a future iteration.

### Why

The N=20 LHS run completed (`logs/centroid_lookup_log.mat`, May 8 2026) and produced 20 paired `(θ_i, p_centroid_i, reachability_pct_i, half_radius_i)`. At ~16 min/eval, the LHS is the expensive part; the cheapest next step is to **extract a Kriging / Gaussian-process surrogate from that existing data and do all optimization on the surrogate**. This is the canonical Design and Analysis of Computer Experiments (DACE) recipe — fit a deterministic interpolant, optimize on the interpolant, verify with one or two ground-truth calls (Sacks et al. 1989; Jones et al. 1998).

This pattern subsumes both user-stated outputs in one model: the GP for `p_centroid` answers "where does it land?" and the GP for `half_radius` (or integrated `mean_ratio`) answers "how wide is the high-reach zone?". Predicted-mean slices through the 4-D box become readable contour plots — landscape characterization comes for free.

### What it is *not*

This is not Plan A (BO/GP) from the surveyed plan set. Plan A's machinery — robust Knowledge-Gradient acquisition, BoTorch via `pyenv`, heteroscedastic-Bernoulli noise model — is justified only when the inner evaluation is stochastic. Here the inner evaluation is deterministic, so the surrogate noise variance is zero and the acquisition function collapses to "argmax of the GP mean." That's a one-line `fmincon` call.

### Sketch

```matlab
% 1. Load LHS data.
load('logs/centroid_lookup_log.mat')   % lookup, ranges, X (N=20×6), fields, N

% 2. Build training matrix in normalized [0,1]^4 coordinates (4-D — Vo, el, az, p).
keep   = [1 2 3 6];                                       % indices into fields
X_train = X(:, keep);                                     % N×4
y_cx   = arrayfun(@(s) s.p_centroid(1),    lookup);       % N×1
y_cy   = arrayfun(@(s) s.p_centroid(2),    lookup);
y_reach = [lookup.reachability_pct].';
y_hr   = [lookup.half_radius].';                          % NaN where never crossed

% 3. Fit four GPs (Statistics & Machine Learning Toolbox — already required).
gp_cx    = fitrgp(X_train, y_cx,    'KernelFunction','ardmatern52', 'Standardize', true);
gp_cy    = fitrgp(X_train, y_cy,    'KernelFunction','ardmatern52', 'Standardize', true);
gp_reach = fitrgp(X_train, y_reach, 'KernelFunction','ardmatern52', 'Standardize', true);
% half_radius is NaN ↔ "wider than the grid extent." Handle as below.

% 4. Optimize on the surrogate (fmincon — Optimization Toolbox, already required).
J = @(t) norm([predict(gp_cx,t); predict(gp_cy,t)] - p_target).^2 ...
         - lambda * predict(gp_reach, t);
theta_opt = fmincon(J, t0, [], [], [], [], zeros(1,4), ones(1,4));

% 5. Verify with one ground-truth call.
theta_phys = denormalize(theta_opt, ranges, fields(keep));
out = sweep_landing_centroid(theta_phys, false);
```

That is the whole MVP: ~30 lines of MATLAB, no new toolboxes, one (≈16 min) verification simulation. Landed as [`surrogate_optimize.m`](../surrogate_optimize.m) (driver) and `figs/SO_01_predicted_J_slices.png` for the slice plots; per-run outputs are saved to `logs/surrogate_optimize_log.mat`.

### Handling the `half_radius = NaN` cases

12 of 20 LHS samples returned `half_radius = NaN` because their reachability profile never decayed below `0.5 × peak_ratio` inside the radial grid (a documented success-case outcome in `sweep_landing_centroid.m`). Three credible options, in increasing order of work:

1. **Replace half-radius with integrated reachability.** `area_proxy = trapz(r_bins, mean_ratio)` is finite for every sample and captures the same "wide vs narrow" axis. Trivial to compute from the saved `radial_profile.mean_ratio`. **Recommended.**
2. **Clamp NaN to grid extent.** Treat NaN as `r_bins(end)` — pessimistic in that it under-states the area for the wide-zone cases. Useful only as a sanity comparison against (1).
3. **Classifier + regressor split.** `fitcnet` on `isnan(half_radius)`, `fitrgp` on the finite subset. Doubles the model count for marginal gain at N=20. Reserve for if (1) proves insufficient.

### Two-objective question

The user explicitly named two outputs (centroid position, centroid area), not a single scalar. Three standard ways to combine them (Marler & Arora 2004):

1. **Weighted-sum scalarization** (sketch above): `J = ||p_centroid − p_target||² − λ · area_proxy`. Simplest. Requires `(p_target, λ)` from the user. λ is a knob, not a constant — sweep it to recover the Pareto front in one extra script.
2. **ε-constraint**: `min ||p_centroid − p_target||² s.t. area_proxy ≥ ε`. Cleaner operational semantics ("hit target while keeping reach-zone above ε"). One `fmincon` call per ε.
3. **Pareto front via NBI / random scalarizations** (Das & Dennis 1998). Overkill at d=4 with one GP-evaluation cost; do it post-hoc on the surrogate.

For iteration 1 default to **(1)** with `λ` from a 5-point sweep, plotted as a Pareto trace.

## Alternative: pure local search (no surrogate)

If "simple" means "no GP at all," the minimum is:

```matlab
J = @(theta) norm(sweep_landing_centroid(theta_to_params(theta), false).p_centroid(1:2) ...
                  - p_target).^2;
theta_opt = fminsearch(J, theta0, optimset('Display','iter','MaxFunEvals',50));
```

This is Nelder–Mead (Nelder & Mead 1965; convergence analysis in Lagarias et al. 1998). Tradeoffs vs. the surrogate route:

| Axis | Nelder–Mead direct | GP surrogate + on-surrogate `fmincon` |
|------|--------------------|---------------------------------------|
| Lines of code | ~10 | ~30 |
| New evaluations | 30–80 × 16 min ≈ 8–21 h | 1–3 × 16 min ≈ 30 min |
| Reuses existing N=20 LHS | No | Yes (fully) |
| Characterizes centroid-area landscape | No | Yes (slice plots) |
| Handles two-objective formulation | Manual | Native via GP for half-radius |
| Global-optimality argument | Multi-start only | Surrogate min + bounded GP uncertainty |

The surrogate route dominates on every axis except raw line count, because the LHS is already paid-for. Reserve `fminsearch` for a sanity-check second opinion at one operating point.

## Banked stochastic plans

When wind / aero / IC dispersions are introduced, the deterministic pipeline becomes a Monte-Carlo pipeline and the surrogate's noise model becomes non-trivial. At that point:

- **Plan B (CEM)** — [`archived/plan_B_cem_outer_loop.md`](archived/plan_B_cem_outer_loop.md). Pure MATLAB, gradient-free, robust to noise and multi-modality. Right hedge if the noisy landscape turns out non-smooth at envelope edges.
- **Plan D Track 1 (CVaR)** — [`archived/plan_D_cvar_covariance_steering.md`](archived/plan_D_cvar_covariance_steering.md). Five-day MATLAB bolt-on once `out.miss_per_cell` is surfaced. Needed if a worst-case acceptance criterion (e.g., "P[miss > 30 m] < 10%") is articulated.

Plan D Track 2 (covariance steering) remains dropped.

## Verification

1. **Surrogate cross-validation.** Leave-one-out RMSE on `p_centroid_x`, `p_centroid_y`, `reachability_pct`, `area_proxy` against the N=20 LHS. Acceptable thresholds need calibration on the data — first run produces them.
2. **Surrogate-optimum vs ground-truth.** Call `sweep_landing_centroid` once at the predicted optimum. Acceptable if `||p_centroid_predicted − p_centroid_actual||₂ < 50 m` (≈ 1 inner-grid cell).
3. **Slice plots vs LHS scatter.** Each pairwise contour slice should be visually consistent with the LHS-sample overlay; large discrepancies indicate under-sampling along that axis and motivate ≤ 4 in-fill points.

## Citations

- McKay, Beckman, Conover, "A Comparison of Three Methods for Selecting Values of Input Variables in the Analysis of Output from a Computer Code," *Technometrics* 21(2), 1979, [JSTOR 1268522](https://www.jstor.org/stable/1268522). — Latin Hypercube Sampling, the design behind `centroid_lookup_table.m`.
- Sacks, Welch, Mitchell, Wynn, "Design and Analysis of Computer Experiments," *Statistical Science* 4(4), 1989, [DOI 10.1214/ss/1177012413](https://doi.org/10.1214/ss/1177012413). — DACE, the canonical "fit a Kriging surrogate to deterministic simulator output" paper.
- Jones, Schonlau, Welch, "Efficient Global Optimization of Expensive Black-Box Functions," *Journal of Global Optimization* 13, 1998, [DOI 10.1023/A:1008306431147](https://doi.org/10.1023/A:1008306431147). — EGO: Kriging + expected-improvement acquisition. The acquisition is unnecessary in the deterministic case but the surrogate framing transfers directly.
- Rasmussen & Williams, *Gaussian Processes for Machine Learning*, MIT Press 2006, [free PDF](https://gaussianprocess.org/gpml/). — Textbook reference for the `fitrgp` machinery (ARD Matérn-5/2 kernel, marginal likelihood fitting).
- Box & Wilson, "On the Experimental Attainment of Optimum Conditions," *JRSS-B* 13(1), 1951, [JSTOR 2983966](https://www.jstor.org/stable/2983966). — Response Surface Methodology, the precursor to DACE for physical experiments.
- Marler & Arora, "Survey of multi-objective optimization methods for engineering," *Structural and Multidisciplinary Optimization* 26, 2004, [DOI 10.1007/s00158-003-0368-6](https://doi.org/10.1007/s00158-003-0368-6). — Scalarization, ε-constraint, NBI: the menu used in the two-objective section above.
- Nelder & Mead, "A Simplex Method for Function Minimization," *Computer Journal* 7(4), 1965, [DOI 10.1093/comjnl/7.4.308](https://doi.org/10.1093/comjnl/7.4.308). — `fminsearch`, the alternative route.
- Lagarias, Reeds, Wright, Wright, "Convergence Properties of the Nelder–Mead Simplex Method in Low Dimensions," *SIAM J. Optim.* 9(1), 1998, [DOI 10.1137/S1052623496303470](https://doi.org/10.1137/S1052623496303470). — Convergence-proof companion to Nelder–Mead, the citation MATLAB itself points to in `doc fminsearch`.
- Das & Dennis, "Normal-Boundary Intersection: A New Method for Generating the Pareto Surface in Nonlinear Multicriteria Optimization Problems," *SIAM J. Optim.* 8(3), 1998, [DOI 10.1137/S1052623496307510](https://doi.org/10.1137/S1052623496307510). — Pareto-front construction if scalarization is rejected.

## Open questions

1. **Target `p_target` and weight `λ`.** Iteration-2 input. A nominal Pareto-trace experiment can fix `λ` empirically — but `p_target` is operational and must come from the project owner.
2. **Promote `w_z0, w_y0` to noise variables or keep as decisions?** Operationally these are exit-attitude rates, set by the mortar's actual launch conditions. If the launch optimizer controls them they are decision variables; if they vary per shot they are disturbances. Decision affects the surrogate's input dimensionality (4 vs 6) and the eventual stochastic phase's noise model.
3. **Area metric.** `area_proxy = trapz(r_bins, mean_ratio)` proposed above; if the user has an operational definition of "wide reach zone" (e.g., area where `mean_ratio ≥ 0.5`), substitute it.
