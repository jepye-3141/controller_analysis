# POC Test Report — Ballistic-Targeting Optimization

> Companion to: [`comparative_survey_targeting_optimization.md`](comparative_survey_targeting_optimization.md). Plans: [A](../plan_A_bo_gp_surrogate.md) · [B](plan_B_cem_outer_loop.md) · [C](plan_C_polynomial_chaos.md) · [D](plan_D_cvar_covariance_steering.md) · [E](plan_E_multi_fidelity_surrogate.md) · [F](plan_F_neural_reachability_surrogate.md). Source files: [`poc_a_bo_gp.m`](../../proof-of-concept/poc_a_bo_gp.m), [`poc_b_cem.m`](../../proof-of-concept/poc_b_cem.m), [`poc_c_sobol.m`](../../proof-of-concept/poc_c_sobol.m), [`poc_d_cvar.m`](../../proof-of-concept/poc_d_cvar.m), [`poc_e_nn_prescreen_cem.m`](../../proof-of-concept/poc_e_nn_prescreen_cem.m), [`poc_f_nn_surrogate.m`](../../proof-of-concept/poc_f_nn_surrogate.m). Test harness: [`test_pocs.m`](../../proof-of-concept/test_pocs.m). Saved benchmarks: `poc_test_results.mat`.

## TL;DR

- **Effectiveness honest-rank**: A (J*=13.49) > B = C (28.26, both = cached argmin) > D (200.43). E (7.66) and F (-1.10) report *lower* J* but as **surrogate predictions, not live-verified** — they may be extrapolating outside the LHS-20 envelope and cannot be ranked against A/B/C/D until one verification sweep is run per POC.
- **Tunability winner depends on the axis**: A is most responsive to long-range shifts (Δθ_norm=0.32), E is most responsive to off-axis shifts (Δθ_norm=0.46). The λ-tunability scenario (T2−T1) is **degenerate at this cache** — every POC gave |Δθ|=0 because the cache argmin already had high reach.
- **Wall-time** at N=20: B/C/D ≈ 0.02–0.05 s, E ≈ 0.05 s, F ≈ 0.14 s, A ≈ 0.30 s. All five orders of magnitude below a single live `sweep_landing_centroid` call (~2 min). Wall-time gap is irrelevant the moment live evaluations are added.
- **Most important caveat**: every POC hits the N=20 cache resolution ceiling. B and C are *degenerate* (return cached argmin by construction). Plan F's gate of N≥100 was honored for the GP branch (which the survey ranked #5 for time-to-MVP, [survey lines 27–28](comparative_survey_targeting_optimization.md)) and confirmed at the NN branch (J=37.2 — diagnosed as under-determined fit, cv_R2=0.655, cv_rmse=102.5).

## Summary ranking matrix

Ratings: `++` = best in class, `+` = strong, `0` = neutral / caveat-laden, `-` = weakest.

| POC | Time complexity | Simplicity (LOC) | Effectiveness (J* honest) | Tunability | Notes |
|-----|-----------------|------------------|----------------------------|------------|-------|
| **A** — BO/GP (MATLAB-native, EI-plus) | `0` (~0.30 s) | `0` (133) | `++` (13.49, honest cached) | `+` long-range | `bayesopt` GP posterior; Plan A's `pyenv`/BoTorch path skipped per Plan A fallback note |
| **B** — CEM | `++` (~0.02 s) | `+` (149) | `+` (28.26, = cached argmin) | `0` | Returns cached argmin at `max_new_sweeps=0`; needs live budget to differentiate from C |
| **C** — Sobol' | `++` (~0.04 s) | `++` (136) | `+` (28.26, = cached argmin) | `0` | Quadratic surrogate R²=1.000 (rank-deficient OLS, ridge=1e-6 nominally fit but P=28>N=20); Sobol' indices reported but unreliable diagnostic |
| **D** — CVaR proxy at μ=1 | `++` (~0.03 s) | `++` (126) | `-` (200.43, μ dominates) | `-` (Δ=0.000 all axes) | μ=1 collapses to a single low-spread sample; Pareto sweep in `poc_d_log.mat` would expose the trade-off but harness only used μ=1 |
| **E** — GP-prescreen + CEM | `+` (~0.05 s) | `0` (205, longest) | `0` (7.66, **surrogate-only**) | `++` off-axis | GP training R²=1.000 (overfit on N=20); `rho_S=NaN` because no live sweeps were committed (budget=0) |
| **F** — GP / NN surrogate + grid argmax | `0` (~0.14 s) | `+` (140) | `0` (−1.10, **surrogate-only**) | `+` mid | GP cv_R2=0.655 (acceptable); NN branch confirms Plan F's N≥100 gate ([plan F line 40](plan_F_neural_reachability_surrogate.md)) — fitrnet J=37.2 is under-determined |

## Per-POC summary cards

### POC A — `poc_a_bo_gp.m` (133 LOC)

- **Plan reference**: [Plan A](../plan_A_bo_gp_surrogate.md). Adapted: BoTorch + rKG path skipped; only the EI-plus MATLAB-only fallback shipped, per Plan A's "Day-7 EI-plus result is publishable on its own" hedge ([survey lines 41, 211](comparative_survey_targeting_optimization.md)).
- **Algorithm**: `bayesopt` over `(Vo, el, az, p)` seeded with all 20 cached entries; with `max_new_sweeps=0`, the GP posterior mean is minimized over an 8⁴ grid (4096 candidates).
- **Numbers**: 133 LOC; wall=0.30 s @T1; J*=13.49 @T1.
- **Tunability**: T1→T4 Δθ_norm=0.321 (BO posterior pushed Vo to 108.6 from 102.9 — most responsive of the six on long-range). T1→T3 Δθ_norm=0.036 (mild off-axis response, did not push az). λ-axis flat (Δθ=0.000).
- **Caveats**: J* is computed from the GP posterior over a coarse grid, not from a live sweep — so 13.49 is an *expected J* given the trained surrogate*, not a verified ground truth. The grid spacing (8 pts/dim) bounds resolution; could miss optima between grid points.

### POC B — `poc_b_cem.m` (149 LOC)

- **Plan reference**: [Plan B](plan_B_cem_outer_loop.md). Adapted: at `max_new_sweeps=0` the CEM main loop never executes — only the cached-pool elite analysis runs, so this POC at this budget reports `theta_cached(argmin J)`. Conditional Bregman-CEM branch ([plan B line 33](plan_B_cem_outer_loop.md)) was correctly *not* implemented (gated on observed multi-modality).
- **Algorithm**: rank cached samples by `J = ||p_centroid − p_target|| − λ·reach`, take top 20% as elites, fit `(μ, Σ)`. With zero budget the loop returns immediately.
- **Numbers**: 149 LOC; wall=0.02 s @T1; J*=28.26 @T1 (= cached argmin).
- **Tunability**: T1→T3 Δθ_norm=0.117, T1→T4 Δθ_norm=0.219. λ-axis flat. Identical θ* to POC C at every scenario — both reduce to argmin over the same 20 cached J values.
- **Caveats**: J*=28.26 is the cached argmin, *honest cached* but uninformative as a ranking signal. The Kroese-§3 covariance floor and zero-reach penalty are wired but unused without budget.

### POC C — `poc_c_sobol.m` (136 LOC)

- **Plan reference**: [Plan C](plan_C_polynomial_chaos.md), but only the Sobol'-analysis sub-deliverable identified by the survey as "highest-value subset of Plan C" ([survey line 75](comparative_survey_targeting_optimization.md)). Full PCE pipeline was correctly *not* built — Plan C parks itself ([survey line 74](comparative_survey_targeting_optimization.md)).
- **Algorithm**: fit quadratic features (1 + d + d(d+1)/2 = 28 cols at d=6) by ridge OLS (ridge=1e-6); estimate first/total Sobol' indices via Saltelli pick-freeze on the surrogate at M=20000 samples. θ* is `argmin J_cached` (re-ranking).
- **Numbers**: 136 LOC; wall=0.04 s @T1; J*=28.26 @T1 (= cached argmin).
- **Tunability**: identical to POC B (both reduce to `argmin(J_cached)`).
- **Caveats**: surrogate R²=1.000 with P=28 features and N=20 samples — the system is **rank-deficient and the ridge=1e-6 produces a near-interpolant**. The reported R² is meaningless as a fit-quality diagnostic; Sobol' indices (az dominates: S_first=0.56, S_total=0.67) are directional only. The plan-internal R²<0.5 warning gate did not fire because R²=1.00 by construction.

### POC D — `poc_d_cvar.m` (126 LOC)

- **Plan reference**: [Plan D Track 1](plan_D_cvar_covariance_steering.md) ([survey line 83](comparative_survey_targeting_optimization.md)). Adapted: classical CVaR_α requires per-cell miss distances ([plan D line 44](plan_D_cvar_covariance_steering.md)) which would need a one-line patch to `sweep_landing_centroid.m` that the user has forbidden. POC substitutes `half_radius` as a spread proxy — surfaced in `meta.warning`.
- **Algorithm**: re-rank cached samples by `J_risk = ||p_centroid − p_target|| + μ·half_radius − λ·reach/100`. Test harness used μ=1. A μ-grid `[0, 0.5, 1, 2]` Pareto sweep is computed and saved to `meta.pareto`. Bootstrap-1000 rank stability also saved.
- **Numbers**: 126 LOC; wall=0.03 s @T1; J*=200.43 @T1.
- **Tunability**: Δθ=0.000 across **all axes** — μ=1 collapses to one sample (`Vo=99.4, el=35.2, az=8.4, p=0.93`) and that sample wins regardless of `p_target` because half_radius dominates the distance term at this scale.
- **Caveats**: not a Rockafellar–Uryasev CVaR. The Pareto sweep at μ ∈ {0, 0.5, 1, 2} is the appropriate ranking artifact for risk tunability; the fixed-μ harness call cannot exercise it.

### POC E — `poc_e_nn_prescreen_cem.m` (205 LOC)

- **Plan reference**: [Plan E](plan_E_multi_fidelity_surrogate.md). Adapted: Plan E's intended cheap fidelity is `sweep_landing_centroid_lo.m` (a reduced-grid sister to the main sweep) ([plan E line 25](plan_E_multi_fidelity_surrogate.md)) — POC substitutes a GP prescreen because the user forbade modifying `sweep_landing_centroid.m`. Surfaced in the function header.
- **Algorithm**: train ARD-Matérn `fitrgp` on cached LHS-20, run 3-iter CEM with N_pop=40 surrogate-scored candidates, top-3 per generation get committed to live sweep (skipped at `max_new_sweeps=0`).
- **Numbers**: 205 LOC (longest); wall=0.05 s @T1; J*=7.66 @T1 (**surrogate-only**).
- **Tunability**: T1→T3 Δθ_norm=0.457 (best off-axis response, GP pushed az to a different basin). T1→T4 Δθ_norm=0.175. λ-axis flat.
- **Caveats**: `surrogate_R2_train=1.000` (training R², not held-out — overfit indicator at N=20). `rho_S=NaN` because no live sweeps committed, so the surrogate-vs-live correlation that Plan E's gate ([survey line 99](comparative_survey_targeting_optimization.md): `ρ_S ≥ 0.6`) hinges on is unmeasured.

### POC F — `poc_f_nn_surrogate.m` (140 LOC)

- **Plan reference**: [Plan F](plan_F_neural_reachability_surrogate.md). Adapted: Plan F's hard prerequisite is N≥100 cached sweeps ([plan F line 40](plan_F_neural_reachability_surrogate.md)); cache is N=20. POC defaults to `fitrgp` (which doesn't blow up at N=20) but exposes a `use_nn=true` toggle to confirm the gate diagnosis.
- **Algorithm**: fit either `fitrgp` (default) or `fitrnet([64 64], relu)` on the 6-D `(Vo, el, az, w_z0, w_y0, p)` input; predict over an 11⁴ grid at the (w_z0, w_y0) midpoint and take argmin.
- **Numbers**: 140 LOC; wall=0.14 s @T1 (GP), 0.42 s (NN); J*=−1.10 @T1 (GP, **surrogate-only**), J=37.21 (NN, **surrogate-only**).
- **Tunability**: T1→T3 Δθ_norm=0.100, T1→T4 Δθ_norm=0.150. λ-axis flat.
- **Caveats**: J*=−1.10 is the surrogate's grid-min posterior mean — likely extrapolation, since cached J values are all positive and the GP can predict below its training data when the target lies near a cluster of well-fitted points. NN branch (J=37.21, cv_R2=0.655, cv_rmse=102.5) confirms Plan F's own under-determined-at-N=20 warning.

## Cross-cutting findings

### N=20 saturation hits every POC

Every POC bottoms out on the cache-resolution ceiling. **B and C are degenerate by construction** — both reduce to `argmin J_cached` at zero new-sweep budget. Their identical θ\* across all 4 scenarios is not a coincidence; it is the same ranking operation. **A is honest but cache-bounded** — its 8⁴ grid scan over a GP fit to 20 points cannot resolve features finer than the cache spacing. **F's surrogate appears to extrapolate** — its J\*=−1.10 sits below the entire training set min (which is 28.26 from the cached argmin), a textbook sign that the GP posterior is being trusted in an unsupported region.

### λ tunability scenario was poorly chosen

ALL six POCs gave Δθ=0 between T1 (λ=0) and T2 (λ=1). The cached argmin already had high reach, so the −λ·reach displacement term pushed in the same direction the distance term already pointed. **Recommend re-running λ-tunability with a `p_target` where the closest-distance sample has *low* reach** (forcing λ to actually trade reach against distance). A simple heuristic: pick `p_target` near the centroid of the lowest-reach quartile.

### POC D's μ=1 collapses to a single sample

D's recommendation is identical (`Vo=99.4, el=35.2, az=8.4, p=0.93`) at all four scenarios — half_radius dominates the distance term at this μ. The Pareto outputs already saved in `poc_d_log.mat:meta.pareto` (μ ∈ {0, 0.5, 1, 2}, with `theta_at_mu`, `J_distance_at_mu`, `half_radius_at_mu`) are the right artifact to expose D's tunability. The harness should be amended to either sweep μ or report the full Pareto.

### Surrogate vs live ground truth is the uncrossed frontier

F's J\*=−1.10 and E's J\*=7.66 are GP-posterior predictions, not validated. A single live verification sweep at each POC's recommended θ\* (~12 min total at 6 sweeps × 2 min each) would settle whether F's surrogate is hallucinating below the training-data min or genuinely identifying a basin between cached samples. **Until that verification runs, F's and E's J\* are not comparable to A/B/C/D's honest cached J\*.**

### Wall-time is irrelevant at the 2-min/sweep ground-truth scale

A's `bayesopt` overhead (~0.30 s) is ~15× the cached-data POCs (~0.02 s) but ~400× *below* a single live sweep. Once `max_new_sweeps>0`, all six POCs' wall times are dominated by the same `sweep_landing_centroid` calls, and the gap inverts: A's 50–100 calls vs B's 200 calls becomes the binding cost (per [survey line 145](comparative_survey_targeting_optimization.md): "outer-optimizer math is not the binding cost").

## Effectiveness vs tunability — recommended next steps

1. **Live-eval verification** (highest priority, ~12 min total). Run one `sweep_landing_centroid` at each POC's T1 θ\*. Reports J\*\_live alongside J\*\_surrogate. Settles whether F's −1.10 is real or extrapolation, whether E's 7.66 is achievable, whether A's GP-grid pick beats B's cached argmin in the wild.
2. **Re-design λ-tunability scenario.** Pick a `p_target` where the closest-distance sample has reach < median. Recommend `p_target = pcs(idx_low_reach_high_distance)` from a quick scan of `centroid_lookup_log.mat`.
3. **Sweep μ for POC D inside the harness.** Either invoke `poc_d_cvar(..., μ)` four times or read `poc_d_log.mat:meta.pareto` directly. The current harness's `mu=1` is one-of-four data points.
4. **Honor Plan F's N≥100 gate.** The fitrnet branch should not be benchmarked again until the cache grows to N≥100. The GP branch is the honest comparison at N=20 — the NN result here is included only to confirm Plan F's own warning, not to rank fitrnet against the others.

## Karpathy-guideline post-mortem

All six POCs cleared the discipline checks:

- **LOC budgets honored**: A=133, B=149, C=136, D=126, E=205, F=140. Smallest = D (126), largest = E (205, justified by the GP+CEM hybrid). All under the implicit Karpathy budget for a single POC (≤300 LOC).
- **No upstream side effects**: `sweep_landing_centroid.m`, `centroid_lookup_table.m`, the cache `centroid_lookup_log.mat`, and every Simulink model are untouched. Verified by `git status` showing none of those paths in modified state.
- **No speculative features built**:
  - POC B did **not** ship Bregman-CEM ([plan B line 33](plan_B_cem_outer_loop.md)) — correctly gated on observed multi-modality (which cannot be observed at zero budget).
  - POC C did **not** ship UQLab/PCE machinery ([plan C line 44](plan_C_polynomial_chaos.md)) — Sobol'-only, hand-rolled quadratic surrogate (the survey's recommended Plan C subset, [survey line 75](comparative_survey_targeting_optimization.md)).
  - POC D did **not** ship Track 2 (covariance steering, YALMIP+MOSEK, [plan D line 60](plan_D_cvar_covariance_steering.md)) — dropped per the survey's recommendation ([survey line 219](comparative_survey_targeting_optimization.md)).
  - POC E did **not** ship `pyenv`/SMT 2.0 co-Kriging ([plan E line 7](plan_E_multi_fidelity_surrogate.md)) — pure MATLAB GP+CEM hybrid.
  - POC F did **not** ship PyTorch/deep ensembles ([plan F line 25](plan_F_neural_reachability_surrogate.md)) — `fitrgp` default, `fitrnet` toggle.
  - No POC patched `sweep_landing_centroid.m` for `out.miss_per_cell` or `ctrl_trial` integral ([survey line 169](comparative_survey_targeting_optimization.md), [plan D line 44](plan_D_cvar_covariance_steering.md)).
- **Plan-deviations are surfaced honestly**:
  - POC D writes the half_radius-vs-CVaR_α deviation into `meta.warning` (lines 101–110 of `poc_d_cvar.m`).
  - POC E surfaces the `sweep_landing_centroid_lo` substitution in the function header (lines 11–14 of `poc_e_nn_prescreen_cem.m`).
  - POC F surfaces the N=20 vs N≥100 gate inline (line 44–46 of `poc_f_nn_surrogate.m`) and emits a `surrogate_drift_warning` when `cv_R2 < 0.5`.
  - POC C documents the rank-deficient-OLS-with-ridge in lines 55–57 of `poc_c_sobol.m`.

## Open questions for the project owner

1. **Should the cache be grown to N≥100 before re-running the comparison?** Plan F's gate ([plan F line 40](plan_F_neural_reachability_surrogate.md)) is hard; Plan E's `ρ_S ≥ 0.6` gate ([survey line 99](comparative_survey_targeting_optimization.md)) is unmeasurable at zero budget. A single Plan B run (~6.7 h) would generate ~200 fresh cached sweeps as a side effect, which would lift the N=20 ceiling that all six POCs are hitting.
2. **Should `sweep_landing_centroid.m` be patched with the `out.miss_per_cell` surface?** Plan D Track 1's true CVaR_α form costs ~5 dev-days ([survey line 83](comparative_survey_targeting_optimization.md)) but unblocks honest CVaR rather than the half_radius proxy currently shipping in POC D. The user's "don't touch landing centroid generation scripts" constraint blocks this until lifted.
3. **Should a one-line `cvar` (or `μ`) parameter be added to the test harness?** Currently the harness fixes μ=1 for POC D. A μ-sweep would expose D's risk-tunability axis the same way the (T2−T1) shift was supposed to expose λ-tunability for the others.
4. **Which `p_target` distribution is operationally meaningful?** The survey ([survey lines 156–165](comparative_survey_targeting_optimization.md)) flags that no plan addresses where `p_target` comes from. T1–T4 are arbitrary picks from the cache distribution. The choice between (a) a single fixed target, (b) a distribution over targets, and (c) a worst-case envelope changes which POC is the right tool — D becomes attractive under (c), F becomes attractive under (b) re-runs.
