# GP Surrogate Recipe — Centroid-Targeting Forward and Inverse Maps

Companion to [`surrogate_optimize.m`](../surrogate_optimize.m) and the design narrative in [`deterministic_optimization_approach.md`](deterministic_optimization_approach.md). The other two docs explain *why* this approach was chosen; this one is the **recipe**: the minimum specification needed to reproduce or cite the method without reading the code.

## Problem statement

Given a desired landing centroid `p_target ∈ R²` for a dSMC-stabilized swarm deployed from a 120 mm mortar trajectory, find the six-element launch vector

```
θ = (Vo, el, az, w_z0, w_y0, p) ∈ R⁶
```

such that the simulated landing centroid `p_centroid(θ)` lands at `p_target` while keeping the surrounding high-reachability zone (`half_radius(θ)`) wide.

Each evaluation of `p_centroid(θ)` requires one call to `sweep_landing_centroid(θ, false)`, which costs ≈ 16 minutes on 4 workers. The recipe below replaces direct optimization over that simulator with a Gaussian-process surrogate trained on a pre-computed Latin-hypercube sample.

## Forward map — Gaussian-process surrogates

Four independent Gaussian-process regressors are fit, one per scalar output of interest:

| Surrogate | Quantity |
|---|---|
| `μ_cx(θ̃)`   | Landing centroid, x-coordinate |
| `μ_cy(θ̃)`   | Landing centroid, y-coordinate |
| `μ_reach(θ̃)` | Fraction of dSMC sweep cells stable, ∈ [0, 1] |
| `μ_hr(θ̃)`   | Half-radius of the radial-reachability profile |

Inputs `θ` are normalized to `θ̃ ∈ [0,1]⁶` against the LHS sampling box (`ranges` in `centroid_lookup_table.m`).

### Posterior mean — closed form

For a single output with training set `{(θ̃_n, y_n)}_{n=1..N}` and constant-mean basis `m(·) = m_0`:

```
μ(θ̃) = m_0 + k(θ̃, X̃)ᵀ [K(X̃, X̃) + σ²I]⁻¹ (y − m_0·1)
              └─────────────── α ───────────────┘
       = m_0 + Σ_{n=1..N} α_n · k(θ̃, θ̃_n)
```

The posterior mean is a sum of `N = 20` ARD Matérn-5/2 kernel basis functions centred at the training inputs, weighted by precomputed coefficients `α`. Evaluable in closed form, but not typesettable as a polynomial in the launch parameters.

### Kernel

ARD Matérn-5/2 with per-input length-scale `ℓ_i`:

```
k(θ̃, θ̃') = σ_f² · (1 + √5·r + (5/3)·r²) · exp(−√5·r)
r = √( Σ_{i=1..d} (θ̃_i − θ̃'_i)² / ℓ_i² )
```

ARD = "automatic relevance determination": each input dimension has its own length-scale, so the fit identifies which launch parameters matter most for each output (large `ℓ_i` = irrelevant axis).

### Hyperparameters

- `(σ_f, σ, ℓ_1..ℓ_6, m_0)` fit by marginal-likelihood maximization (Rasmussen & Williams 2006, §5.4.1).
- Done internally by `fitrgp` (MATLAB Statistics & Machine Learning Toolbox) — no external optimizer.
- Outputs are standardized (zero-mean, unit-variance) during fitting; `predict()` reverses the transformation.
- Noise variance `σ²` is fit but small — the simulator is deterministic, so `σ²` absorbs only numerical jitter.

Fitted hyperparameters for the current run are stored inside the saved GP objects `gp_cx`, `gp_cy`, `gp_reach`, `gp_hr` in `logs/surrogate_optimize_log.mat`. The length-scales after training are the most informative artifacts — they say which input axes the GP found important per output.

## Training set

| Item | Value |
|---|---|
| Sampling design | Latin Hypercube (McKay et al. 1979) |
| Generator | `lhsdesign(N, d)` |
| Seed | `rng(0)` |
| `N` | 20 |
| `d` | 6 (`Vo, el, az, w_z0, w_y0, p`) |
| Box | `Vo ∈ [80,120]`, `el ∈ [35,55]`, `az ∈ [0,30]`, `w_z0 ∈ [−1,2]`, `w_y0 ∈ [−1,1]`, `p ∈ [−2,2]` |
| Per-sample cost | One `sweep_landing_centroid(θ_n, false)` call, ≈ 16 min on 4 workers |
| Outputs collected per sample | `p_centroid (R²)`, `reachability_pct (scalar)`, `half_radius (scalar, NaN-able)` |

Reproducer: run `centroid_lookup_table.m`. Output: `logs/centroid_lookup_log.mat`.

### `half_radius = NaN` clamp

12 of 20 LHS samples returned `half_radius = NaN` — the radial-reachability profile never decayed below `0.5 × peak_ratio` inside the sweep grid (a documented success-case outcome). These are clamped to `1.2 × max(finite half_radius)` so the GP treats them as "at least this wide," not as missing data. A cleaner formulation (integrated-reachability area metric) is sketched in [`deterministic_optimization_approach.md`](deterministic_optimization_approach.md) §"Handling the `half_radius = NaN` cases" but not yet implemented.

## Inverse map — targeting via on-surrogate optimization

The targeting problem `θ*(p_target)` has no closed-form solution. It is defined implicitly as the minimizer of a scalarized two-objective (Marler & Arora 2004):

```
θ*(p_target) = argmin_{θ̃ ∈ [0,1]⁶}  J(θ̃; p_target)

J(θ̃; p_target) = ( μ_cx(θ̃) − p_target,x )²
               + ( μ_cy(θ̃) − p_target,y )²
               − λ · μ_hr(θ̃)
```

- First two terms: squared miss distance from desired centroid.
- Third term: rewards a wider high-reachability zone around the centroid.
- `λ ≥ 0`: operational weight. Current default `λ = 50` (m, since `half_radius` has units of metres and the miss term is m²). Sweep `λ` to recover the Pareto front of (accuracy, robustness) trade-offs.

Solver:

| Item | Value |
|---|---|
| Routine | `fmincon` (Optimization Toolbox) |
| Algorithm | SQP |
| Box constraints | `[0,1]⁶` |
| Restarts | 8 |
| Restart design | `lhsdesign(8, 6)` with `rng(0)` |
| Per-restart budget | `MaxFunctionEvaluations = 500` |

Best objective across restarts wins. Implemented at [`surrogate_optimize.m:49–71`](../surrogate_optimize.m).

After optimisation, `θ̃*` is denormalised to physical units by `θ_phys,i = ranges.f_i(1) + θ̃*_i · diff(ranges.f_i)`.

## Reproducibility checklist

To reproduce the targeting method from this repo:

1. **Generate training data.** Run `centroid_lookup_table.m` with `N = 20`, `rng(0)`. Writes `logs/centroid_lookup_log.mat` (`lookup`, `ranges`, `N`).
2. **Fit surrogates and optimise.** Edit `p_target` and `lambda` at the top of `surrogate_optimize.m`, then run it. It reads the lookup, fits four `fitrgp` GPs with `KernelFunction='ardmatern52'`, `Standardize=true`, `BasisFunction='constant'`, runs the 8-start SQP, writes `logs/surrogate_optimize_log.mat` and `figs/SO_01_predicted_J_slices.png`.
3. **Verify (optional).** Set `verify = true` in `surrogate_optimize.m` to run one ground-truth `sweep_landing_centroid` call at `theta_phys` and print predicted-vs-actual centroid, reachability, and half-radius. ~16 min extra.

Required MATLAB toolboxes:
- Statistics & Machine Learning Toolbox (`fitrgp`, `lhsdesign`)
- Optimization Toolbox (`fmincon`)
- Simulink + Control + Navigation Toolboxes (for the inner sweep — not the surrogate fit itself)

## Paper-ready paragraph

> Launch-parameter targeting is performed by Kriging surrogate-based optimisation (Sacks et al. 1989). For each of four outputs of interest — landing-centroid x and y, reachability fraction, and half-radius of the radial-reachability profile — an independent Gaussian-process regressor with an ARD Matérn-5/2 kernel and constant mean basis is fit (`fitrgp`, MATLAB Statistics & Machine Learning Toolbox; Rasmussen & Williams 2006) over N = 20 Latin-hypercube samples (McKay et al. 1979) of the six-dimensional normalised launch box `(Vo, el, az, w_z0, w_y0, p)`. Hyperparameters (per-input length-scales, output variance, noise variance, mean offset) are determined by marginal-likelihood maximisation. The targeting problem is then a scalarised two-objective (Marler & Arora 2004), `J(θ) = ‖μ_centroid(θ) − p_target‖² − λ · μ_hr(θ)`, minimised over the unit box by multi-start SQP (`fmincon`, 8 Latin-hypercube restarts). The surrogate optimum is verified by one ground-truth simulator call.

## Pointers

- Code: [`surrogate_optimize.m`](../surrogate_optimize.m), [`centroid_lookup_table.m`](../centroid_lookup_table.m), [`sweep_landing_centroid.m`](../sweep_landing_centroid.m).
- Design rationale and alternatives considered: [`deterministic_optimization_approach.md`](deterministic_optimization_approach.md).
- Reference plan including the acquisition-layer machinery we deliberately *skip* (deterministic case): [`plan_A_bo_gp_surrogate.md`](plan_A_bo_gp_surrogate.md).
- Saved fitted models: `logs/surrogate_optimize_log.mat` (objects `gp_cx`, `gp_cy`, `gp_reach`, `gp_hr`; coefficients accessible via `gp.Alpha`, length-scales via `gp.KernelInformation.KernelParameters`).

## Key references

- McKay, Beckman & Conover, "A Comparison of Three Methods for Selecting Values of Input Variables in the Analysis of Output from a Computer Code," *Technometrics* 21(2), 1979.
- Sacks, Welch, Mitchell & Wynn, "Design and Analysis of Computer Experiments," *Statistical Science* 4(4), 1989.
- Rasmussen & Williams, *Gaussian Processes for Machine Learning*, MIT Press 2006.
- Marler & Arora, "Survey of multi-objective optimization methods for engineering," *Structural and Multidisciplinary Optimization* 26, 2004.

Full citation list with DOIs in [`deterministic_optimization_approach.md`](deterministic_optimization_approach.md).
