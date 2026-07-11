# Plan F — Neural-Network Reachability Surrogate

> Source: Section A items 5–7 (lines 53–56) and Section C "Neural-network surrogates for reachability/capture sets" (lines 114–115) of [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md).

## Overview

Train a small feed-forward neural network on the existing LHS lookup data so that a 4-D launch vector `(Vo, el, az, p)` maps to `(reach_pct, p_centroid, U)` in <10 ms, replacing the 2-min `sweep_landing_centroid.m` call inside any outer optimizer. **This plan is supervised regression on Monte-Carlo data, not certified Hamilton–Jacobi reachability.** The certified-reach-set methods cited in the report (DeepReach, NeuralPARC, CARe) all require either an HJB-PDE residual or known piecewise-affine dynamics; the user's reachability map is produced by a stochastic dSMC closed-loop sweep over a Simulink black-box, so neither route is applicable without re-engineering the simulator. The pragmatic alternative — a small MLP fit to existing sweep outputs — is directly tractable and is the recommended route. Plan F is best understood as the *cached surrogate* underneath Plans A–E rather than a free-standing optimizer.

## Mathematical formulation

- **Inputs**: 4-D launch decision `θ = (Vo, el, az, p)`, ranges from [`centroid_lookup_table.m`](../centroid_lookup_table.m) lines 8–14 (`Vo ∈ [80,120]`, `el ∈ [35,55]`, `az ∈ [0,30]`, `p ∈ [-2,2]`). Optionally extend to 6-D by appending the ballistic disturbance variates `(w_z0, w_y0)` for an "all-in-one" deterministic surrogate; otherwise keep them implicit and let MC noise sit in the regression target.
- **Outputs** (recommended, in priority order):
  1. **Scalar `reach_pct`** — single most useful target; smoothest under MC noise; directly comparable across the LHS history.
  2. **2-vector `p_centroid = (cx, cy)`** — needed if the outer optimizer is matching a target distribution centred on a specific landing point (Plan A's `p_target` convolution).
  3. **Scalar `U`** — expected control effort, once `sweep_landing_centroid.m` is patched to integrate `vecnorm(ctrl_trial,2,2).^2` (see Plan A patch).
- **Recommendation**: predict `(reach_pct, cx, cy, U)` jointly from a single MLP trunk with one shared hidden representation and four output heads. Training one trunk uses sample efficiency better than four independent nets. Do **not** target the full 200×200 `landing_ratio` heatmap (`Rg` from `sweep_landing_centroid.m` line 268) — that is a 40 000-D output, vastly more parameters than the dataset can support, and the centroid + reachability_pct + half_radius are sufficient summaries for downstream optimization.
- **Loss**: heteroscedastic Gaussian NLL where the per-sample noise variance comes from the inner MC (`σ²_reach ≈ p(1-p)/84`, `σ²_centroid` from per-sample bootstrap). Falls back to plain MSE with a per-output normalization if NLL gives no benefit on the held-out set; the MC noise is roughly homoscedastic across the LHS box anyway.
- **Optional uncertainty calibration**: deep ensemble of 5 MLPs with different seeds. Predictive mean = ensemble average, predictive variance = ensemble disagreement + per-sample MC noise. This is the cheap NN analogue of a GP posterior and is what active sampling will key on. A deep-kernel GP (Profile BO arXiv 2512.23581) is a stricter alternative if formal calibration is required; given d=4 and N≤500, **a plain heteroscedastic MLP ensemble is sufficient and a GP is competitive — Plan F is mainly worth its cost if the GP in Plan A turns out to be a poor fit (cliff, multimodality)**.

## Required infrastructure

- **Training data**: the lookup struct array produced by [`centroid_lookup_table.m`](../centroid_lookup_table.m) and any subsequent BO/CEM history from Plans A–E (these have the same schema: launch + `(reach_pct, cx, cy, half_radius, U)`). All cached in `centroid_lookup_log.mat`, `bo_log.mat`, `cem_log.mat`. The first training run reuses whatever has been generated so far; the surrogate is retrained whenever a new sweep is appended.
- **NN library**:
  - **Recommended: MATLAB Statistics and Machine Learning Toolbox `fitrnet`** ([fitrnet doc](https://www.mathworks.com/help/stats/fitrnet.html)). Fully native, no `pyenv`, supports `LayerSizes=[64 64]`, ReLU activations, L2 regularization, automatic standardization, GPU acceleration via `gpuArray` if available, and a built-in cross-validation flow (`crossval(Mdl)`). Fits the 4-D → 4-output regression in seconds on CPU. The legacy `feedforwardnet` is being deprecated; `fitrnet` is the supported successor.
  - **Fallback**: PyTorch via `pyenv` (same harness as Plan A's BoTorch path). Only worth the setup cost if a deep ensemble or heteroscedastic NLL must be implemented and `fitrnet`'s built-in regression head proves limiting. PyTorch also enables exporting to ONNX for cross-platform use.
  - **Not recommended**: `dlnetwork` directly, unless you already need its custom-layer flexibility — overkill for a 64-unit MLP.
- **Active-sampling loop** (optional refinement, see Roadmap step 6):
  - Score each candidate launch in a dense 4-D LHS pool by ensemble disagreement (or, if using a GP surrogate, posterior variance). Pick the top-K, run them through `sweep_landing_centroid.m`, append to the dataset, retrain. This is exactly the [Devonport & Arcak 2019 (arXiv:1910.02500)](https://arxiv.org/abs/1910.02500) recipe (note: the paper is *Devonport & Arcak*, not Devolder/Hewing as in the lit-review draft) — they used a GP classifier; here the same algorithm runs on an MLP-ensemble variance estimate.
  - For *optimization-aware* refinement (where the goal is finding the optimum, not refining the surface uniformly), add an Expected-Improvement-style acquisition on top of the surrogate; in d=4 this collapses into Plan A's BO loop with the GP swapped for an MLP.

## Integration with existing codebase

- **API contract**: `nn_predict(theta) → (Q, U)` in <10 ms. Concretely, a thin MATLAB function `nn_predict.m` loads a saved `RegressionNeuralNetwork` (or ensemble of them) from `nn_surrogate.mat`, applies the same input standardization used at training time, and returns the four outputs. `Q` is computed downstream by convolving the predicted `(reach_pct, cx, cy, half_radius)` with the user's `p_target` distribution — the convolution itself is a closed-form inner product when `p_target` and the reachability radial profile are both Gaussian, microseconds either way.
- **Drop-in for outer optimizers**: in [`plan_A_bo_gp_surrogate.md`](../plan_A_bo_gp_surrogate.md), the `bo_objective.m` wrapper currently calls `sweep_landing_centroid` (2 min). Replace that call with `nn_predict` (sub-millisecond) for the inner objective inside synthetic BO experiments, sensitivity sweeps, or multi-`p_target` re-optimization (Plan A line 68). Outer optimizers in Plans A, B, C, D, E all consume the same 4-D → scalar interface, so a single Plan F surrogate is shared across them.
- **No Simulink touch-points.** Training runs entirely on cached data; the surrogate never invokes Simulink at inference time.
- **No changes to the parsim wiring** (CLAUDE.md "parsim wiring" section). Plan F only adds: a trainer script, a saved model, and a thin predictor wrapper.

## Implementation roadmap

Plan F has a **gating prerequisite**: do not invest in NN training until the codebase has accumulated **≥100 sweeps**. With the existing 20-point LHS, training a 4-D regressor is mostly an empty exercise — cross-validated R² on N=20 will be dominated by leave-one-out variance. The following steps assume that gate has been met.

1. **Day 1: data audit.** Concatenate `centroid_lookup_log.mat` + any `bo_log.mat`/`cem_log.mat` rows into a single table `(Vo, el, az, p, reach_pct, cx, cy, U, half_radius)`. Inspect distributions, log-transform skewed columns, drop rows with `tot_w == 0` (the centroid-undefined assertion in `sweep_landing_centroid.m` line 275). **Verifiable: N ≥ 100, all columns finite.**
2. **Day 2: baseline regressor.** Fit a separable Matérn-5/2 GP (MATLAB `fitrgp`) and a 2-layer MLP `fitrnet([64 64], 'Activation', 'relu')` on the same train/test split (80/20). Report 5-fold cross-validated RMSE for `reach_pct` and centroid components. **Verifiable: GP and MLP within 20% of each other on RMSE; both materially better than the constant-mean baseline.**
3. **Day 3: ensemble and uncertainty.** Train 5 MLPs with different seeds; record ensemble mean and variance. Sanity-check that ensemble variance is high in low-data regions (corners of the LHS box) and low in dense regions. **Verifiable: variance vs. nearest-neighbour-distance is monotonic.**
4. **Day 4: predictor wrapper.** `nn_predict.m`, `nn_train.m`, `nn_validate.m`. Save model + standardization scalars to `nn_surrogate.mat`. **Verifiable: round-trip a held-out point through the saved file in <10 ms.**
5. **Days 5–6: held-out validation.** Pick 5 launch points from the held-out 20% (some near LHS-box edges, some interior), re-evaluate at high-MC ground truth (5× inner count, ≈10 min each). Compare surrogate prediction vs. ground truth. **Verifiable: |Δreach_pct| < 0.05, |Δcentroid| < 5 m.**
6. **Day 7+: active sampling (optional).** Generate a dense LHS pool of 1000 candidates; pick top-K (K=4) by ensemble variance; run sweeps; append. Repeat until either ensemble RMSE plateaus or the active set is exhausted by stagnation. Cite [Devonport & Arcak (2019, arXiv:1910.02500)](https://arxiv.org/abs/1910.02500) for the formal active-sampling recipe.

## Computational budget

- **Data acquisition** (the cost): 100 sweeps × 2 min ≈ 3.3 h wall on the existing 4-worker pool, sweep-only. 500 sweeps ≈ 16.7 h (≈2 working days). Active sampling can substitute for ~50% of that volume by concentrating samples where they matter, per Devonport & Arcak's reported sample-reduction; budget 100 LHS + 100 active = 200 sweeps ≈ 6.7 h end-to-end as a **realistic plan**.
- **Training**: seconds for `fitrnet` on a CPU at d=4, N≤500. Deep ensembles: 5× that, still <1 minute.
- **Inference**: <1 ms per launch in MATLAB, well under the 10 ms budget. Suitable for tight inner loops in any outer optimizer.
- **Retraining cadence**: re-fit whenever ≥10 new rows accumulate in `bo_log.mat` / `cem_log.mat`. Cheap.
- **Net**: Plan F amortizes only when the surrogate is queried ≥1000× more often than it is trained — i.e., when an outer optimizer re-evaluates many `p_target` instances, runs sensitivity sweeps, or warm-starts an online query. For a single `p_target` and a single launch decision, the BO loop in Plan A directly is cheaper.

## Validation strategy

- **Hold-out RMSE.** 80/20 split, 5-fold cross-val, on each output channel. Acceptable thresholds: `RMSE(reach_pct) < 0.07`, `RMSE(cx), RMSE(cy) < 5 m`. These are roughly the inherent MC noise floor of the 84-trial inner sweep; getting closer than that with N=200 is unrealistic.
- **Rank correlation.** Spearman ρ between predicted and observed `reach_pct` across the held-out set. Acceptable: ρ > 0.85. This is the metric that matters for *optimization* — absolute miscalibration is forgivable as long as the surrogate ranks launches correctly.
- **Cliff diagnostics.** Deliberately sample 5 launches near the dSMC-saturation cliff (high `Vo`, high `el`, large `p` — the regions where `MEMORY.md` saturation-sweep entries flag non-monotonicity at i=11). Compare predicted vs. observed; if the MLP smooths over a discontinuity, the cliff samples will land outside its calibrated variance band. Document the failure modes; they motivate the GP-vs-NN comparison below.
- **Ensemble variance calibration.** On the held-out set, the predicted variance should be proportional to squared error. Plot variance vs. squared error; report Spearman ρ.
- **Reproducibility.** Fix `rng(0)` for the data shuffle, record the `fitrnet` seed in `nn_surrogate.mat`. Re-fit must be deterministic given the same inputs.

## Risks & mitigations

1. **Insufficient training data.** d=4, N=20 is hopelessly under-determined for a 64×64 MLP (~5000 parameters). *Mitigation*: gate Plan F on N ≥ 100 (above). Until then, a 4-D GP on the LHS-20 is more honest about its uncertainty than any MLP fit on the same data, and Plan A's GP serves the same role with calibrated posteriors. **Plan F is wasted effort if launched before Plans A or B have generated their BO/CEM histories.**
2. **Cliff non-stationarity (the dSMC saturation boundary).** The dSMC reachability surface has a sharp drop-off when the deployment state lands outside the powered controller's authority — the report itself flags this in its Caveats (lines 209, 214). MLPs with smooth (ReLU) activations interpolate across discontinuities and produce confidently wrong predictions on the steep side. *Mitigation*: (a) detect the cliff with a separate binary classifier (`fitcnet`) trained on `tot_w > 0` vs. `== 0` outcomes, and gate the regression on classifier confidence; (b) deep-kernel or non-stationary GP (Profile BO arXiv 2512.23581) instead of MLP for the cliff region; (c) deliberately oversample the cliff during active learning so the MLP has at least a chance of resolving it. **GP outperforms NN here when N is small (<200) and the cliff is sharp** — the GP's local-kernel structure refuses to extrapolate into low-data regions, while the MLP confidently does. Once N exceeds ~500 with cliff-region oversampling, the MLP's flexibility starts to pay off.
3. **Distribution shift between training and online use.** If the LHS box used for training (lines 8–14 of `centroid_lookup_table.m`) does not include the operationally interesting launch envelope, the surrogate is being asked to extrapolate. *Mitigation*: explicit out-of-distribution check using ensemble variance — refuse to predict (return NaN) when variance exceeds a threshold calibrated on the held-out set. The outer optimizer can then either fall back to the true `sweep_landing_centroid` or skip the candidate.
4. **MC noise floor masquerading as model bias.** With 84 inner trials, `σ_reach ≈ 0.05` is irreducible; trying to drive RMSE below that with more NN capacity overfits to MC noise. *Mitigation*: bake the MC noise into the heteroscedastic NLL loss; track training RMSE explicitly against the noise floor and stop adding capacity when training RMSE ≈ noise floor.
5. **Surrogate drift as the dSMC controller evolves.** Plan A+ (`dsmc_constraints.m`, `MEMORY.md` saturation-sweep entries) is under active development; any change to the controller invalidates the cached `(reach_pct, centroid, U)` and therefore the surrogate. *Mitigation*: tag every cached row with the git commit of `dsmc_constraints.m` at sweep time (`git rev-parse HEAD`); the trainer rejects rows whose commit predates the current controller hash. This converts a silent correctness bug into a noisy retraining cost.

## Citations

Research-report grounding:
- [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 53–56 (NN-surrogate options for reach-set construction), 114–115 (NN surrogate paragraph in §C), 127–132 (active learning), 209, 214 (cliff caveats).

Reachable-set surrogates (verified via web):
- Bansal, S., & Tomlin, C. J. (2021). "DeepReach: A Deep Learning Approach to High-Dimensional Reachability." *2021 IEEE ICRA*. [arXiv:2011.02082](https://arxiv.org/abs/2011.02082) · [DOI 10.1109/ICRA48506.2021.9561949](https://doi.org/10.1109/ICRA48506.2021.9561949) · [smlbansal/deepreach](https://github.com/smlbansal/deepreach). Sinusoidal-network HJB-PDE solver. **Not applicable here** because the user's reachability map is not the value function of an HJB; flagged as the canonical certified-reachability route for future reference.
- Chung, L., Long, W., Kousik, S., et al. (2024). "Guaranteed Reach-Avoid for Black-Box Systems through Narrow Gaps via Neural Network Reachability" (NeuralPARC). [arXiv:2409.13195](https://arxiv.org/abs/2409.13195). ReLU-network reach-avoid via Reachable Polyhedral Marching. **Not applicable** because it requires a piecewise-affine ReLU trajectory model, which the user's stochastic Simulink dSMC sweep does not produce.
- Solanki, P., Vertovec, N., Schnitzer, Y., van Beers, J. J., de Visser, C. C., & Abate, A. (2025). "Certified Approximate Reachability (CARe): Formal Error Bounds on Deep Learning of Reachable Sets." [arXiv:2503.23912](https://arxiv.org/abs/2503.23912). SMT-CEGIS certification of HJ-learned reach sets. **Not applicable** for the same reason as DeepReach.
- Xiang, W., Tran, H.-D., Yang, X., & Johnson, T. T. (2020). "Reachable Set Estimation for Neural Network Control Systems: A Simulation-Guided Approach." *IEEE TNNLS* 32, 1821–1830. [arXiv:2004.12273](https://arxiv.org/abs/2004.12273). Simulation-guided reach estimation for NN-controlled CPS — closer in spirit to the user's setup, but still presupposes an interval-arithmetic abstraction of the controller. Useful theoretical context, not a drop-in implementation.
- Devonport, A., & Arcak, M. (2019). "Data-Driven Reachable Set Computation Using Adaptive Gaussian Process Classification and Monte Carlo Methods." [arXiv:1910.02500](https://arxiv.org/abs/1910.02500). The active-sampling recipe Plan F adopts. (Author names corrected: lit-review draft listed "Devolder & Hewing", but the paper is by Devonport & Arcak; cite carefully.)

NN tooling and surrogate-modeling references:
- MathWorks. "fitrnet — Train neural network regression model." [mathworks.com/help/stats/fitrnet.html](https://www.mathworks.com/help/stats/fitrnet.html). The recommended MATLAB native path.
- MathWorks. "feedforwardnet — (To be removed) Generate feedforward neural network." [mathworks.com/help/deeplearning/ref/feedforwardnet.html](https://www.mathworks.com/help/deeplearning/ref/feedforwardnet.html). Legacy; do not use for new work.
- Dahinden, V., et al. (2025). "Profile Bayesian Optimization for Expensive Computer Experiments." [arXiv:2512.23581](https://arxiv.org/abs/2512.23581). Deep-kernel GP fallback if the cliff is too sharp for plain MLPs; cited in the report at line 115 as the deep-kernel/DGP option.

Companion plans:
- [`plan_A_bo_gp_surrogate.md`](../plan_A_bo_gp_surrogate.md) — primary outer-loop optimizer; Plan F replaces its inner objective for warm-start and multi-`p_target` use.
- [`plan_B_cem_outer_loop.md`](plan_B_cem_outer_loop.md) — alternate outer-loop optimizer; same drop-in relationship.
