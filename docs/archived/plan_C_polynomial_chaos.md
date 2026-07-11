# Plan C — Polynomial Chaos / Stochastic-Collocation Surrogate for the Ballistic Transition Kernel

> Source: Recommendation 3 of [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 177–186.

## Overview

Plan C is an *augmentation* of [Plan A](../plan_A_bo_gp_surrogate.md) or [Plan B](plan_B_cem_outer_loop.md), not a replacement. It builds a non-intrusive generalized polynomial chaos (gPC) surrogate of the **ballistic transition kernel only** — the map `x_d = F_ballistic(launch, ξ)` from launch parameters and aero/wind/IC noise ξ to the deployment-state vector at apogee — and uses it to replace any future ξ-Monte-Carlo at the entry to `sweep_landing_centroid.m`. It does not replace the dSMC Simulink rollouts; those remain the dominant cost.

The honest read up front: in the **current** pipeline the inner cost is utterly dominated by the dSMC Simulink sweep (≈84 trials × ~1.4 s wall ≈ 2 min) and the single deterministic `eom2` propagation that feeds it (~50 ms — 0.04% of wall). PCE only becomes valuable if and when Plan A or Plan B introduces a true ξ-MC at the deployment-state level (e.g., Plan A §"Integration with existing codebase" option (i), N_ξ=4 inner MC). Until then, Plan C should be **parked**, not built.

## Mathematical formulation

Let `ζ = (launch, ξ) ∈ R^d` collect every input dimension under one symbol:

- Decision/launch: `θ = (Vo, el, az, p)`, d_θ = 4.
- Uncertainty (per `centroid_lookup_table.m` ranges + report Caveats): `ξ = (w_z0, w_y0, alpha_0, beta_0, c_Magnus, c_drag)`, d_ξ ≈ 6. (Magnus and drag coefficient scaling factors are *not* in the current LHS sweep but the report's Caveat at line 211 explicitly demands they be treated as random inputs.)
- Total `d = d_θ + d_ξ = 9–10`.

The ballistic-phase output of interest is the deployment-state apogee vector — or a low-dimensional summary of it — that `sweep_landing_centroid.m` actually consumes when building each `xi` (the 12-state initial condition for the dSMC sim, line 142). Concretely the consumed quantities are: `deploy_pos(:, i)`, `deploy_vel_earth(:, i)`, `deploy_rvel_earth(:, i)`, `deploy_rot_deg(:, i)` for each of the `n_deploy = 8` arc-length stations, i.e. 12 scalars per station × 8 stations = 96 outputs. The PCE surrogate target is the vector-valued map `F: R^d → R^{96}` (or, more cheaply, just the apogee station's 12 outputs if Plan A uses single-deploy stabilization).

**gPC basis.** Per Xiu & Karniadakis (SIAM J. Sci. Comput. 2002), the natural Wiener–Askey choice is:
- Uniform inputs (launch box, `alpha_0`, `beta_0`) → Legendre.
- Gaussian inputs (`w_z0`, `w_y0`, scaled aero coeffs) → Hermite.
- Mixed inputs → tensor-product Legendre/Hermite.

Total-order-`p` truncation: `P+1 = (d+p)!/(d!·p!)`. For `d = 9, p = 3`: 220 terms. For `d = 9, p = 4`: 715 terms. For `d = 10, p = 3`: 286.

**Non-intrusive collocation.** Because `eom2` is an opaque black box (it also has discrete events — the `impactEvent` function on line 236 of `eom2.m` — but those terminate after apogee, so the apogee-state map remains smooth in ζ), use non-intrusive spectral projection or compressed-sensing regression rather than intrusive Galerkin. Two viable schemes:

1. **Smolyak sparse-grid collocation** (Witteveen–Iaccarino; sparse-grid review arXiv 1509.01462). Level-`l` isotropic Smolyak in `d` dims has O(2^l · d^l / l!) nodes — far fewer than full tensor-product `(p+1)^d`. For d=9, l=3 (capturing third-order interactions): roughly 1000–2500 nodes.
2. **Compressed-sensing PCE / weighted ℓ1 minimization** (Peng–Hampton–Doostan JCP 2014; Adcock–Brugiapaglia–Webster arXiv 1703.06987; ETH RSUQ-2020-002C survey). Exploits coefficient sparsity in lower (downward-closed) sets; a few hundred quasi-random samples typically recover a degree-3 PCE in d=9. UQLab's LARS solver implements this directly.

Compressed-sensing PCE is the right default at d=9: it is robust to non-uniform sample placement (compatible with reusing existing LHS samples from `centroid_lookup_log.mat`), tolerates outliers, and produces sparse coefficient vectors that double as Sobol' sensitivity indicators. Smolyak is the right choice only if d is held strictly to `d ≤ 6`.

**Magnus/drag uncertainty.** The report (line 211) flags Magnus as the dominant aero uncertainty, with point estimates 10–30% off. Treating Magnus as an additional Gaussian (with σ ≈ 15% of nominal `C_M_pa`) is *adequate* for non-intrusive PCE — gPC handles non-Gaussian inputs natively via Wiener–Askey, and the Magnus moment enters `eom2.m` on line 308 (`C_tilde_M_pa = rho*S*(d^2)*C_M_pa*p / (2*I_y)`) as a smooth scalar multiplier. Same for drag (line 301). The wrapper would multiply the aero-coefficient lookup output by random scaling factors `1 + ε_drag` and `1 + ε_Magnus` injected through the `env` struct. This is a clean d_ξ extension — not a workflow change.

## Required infrastructure

- **New MATLAB code**:
  - `pce_build.m` — driver that (a) defines the input distributions for ζ, (b) generates the experimental design (Smolyak nodes or LHS for compressed sensing), (c) calls `eom2` once per node with perturbed `env` (Magnus/drag scalings injected), (d) extracts the 12-vector deployment state at apogee, (e) fits the PCE coefficients per output channel (UQLab `uq_createModel` with `Type = 'Metamodel', MetaType = 'PCE'`).
  - `pce_eval.m` — wrapper that takes a launch sample `θ` plus an N×d_ξ table of ξ samples (or analytic moments) and returns the corresponding 12-vector deployment-state samples (or its mean/covariance/quantiles), using UQLab's `uq_evalModel`.
  - `pce_validate.m` — large-MC ground truth via `eom2`, ≥10× the design size; reports per-channel RMSE, normalized error, and Sobol' indices.
- **External libraries** (recommendation in priority order):
  1. **UQLab v2.x** ([www.uqlab.com](https://www.uqlab.com/)) — ETH Zurich, BSD-3-clause since 2022 (also v2.0 release news on the ETH RSUQ page). Pure MATLAB, integrates with the existing LHS workflow, ships LARS/OMP/SP for compressed-sensing PCE, has a Smolyak module, computes Sobol' indices analytically. **This is the recommendation.**
  2. **chaospy** via `pyenv` ([github.com/jonathf/chaospy](https://github.com/jonathf/chaospy)). Excellent toolbox but adds the same MATLAB↔Python serialization tax that Plan A already evaluates, with no PCE-specific advantage over UQLab. *Do not adopt unless UQLab licensing changes or a Python-native pipeline already exists.*
  3. **PoCET** ([arXiv 2007.05245](https://arxiv.org/abs/2007.05245); GitHub `MrFelixP/PoCET`) — Petzke–Mesbah–Streif IFAC 2020. Good for *intrusive* projection-based PCE on user-specified ODEs, but `eom2` is treated as a black box here; PoCET's Galerkin engine offers no benefit over UQLab's regression engine for non-intrusive PCE. *Skip.*
  4. **Hand-rolled** Hermite/Legendre + tensor product. Justifiable only if every dependency must be removed; 1–2 dev-weeks; UQLab v2 makes this an active anti-recommendation.
- **New data products**:
  - `pce_log.mat` — input distributions, design matrix (ζ → 96-vector outputs), fitted UQLab model, per-channel sparsity & validation metrics.
  - Optional Sobol' index report — already produced by UQLab; useful for justifying which ξ dimensions can be held deterministic in Plans A/B.

## Integration with existing codebase

- **Where the inner MC actually lives.** It does not, currently. `sweep_landing_centroid.m` is *deterministic in ξ*: it calls `eom2` exactly once at line 58 with the launch tuple, then sweeps the 8 deploy × 3 cross-track × 4 neighbor *structural* grid (line 122, 84 trials) for a single ballistic trajectory. The 84 trials are deliberate dSMC reachability sweeps under the dSMC's own internal noise (Simulink solver noise + dSMC saturation), not random samples of ξ. **PCE replaces nothing in the current code.**
- **Where PCE *would* enter once Plan A/B grows a ξ-MC.** The Plan A roadmap (lines 38–46 of `plan_A_bo_gp_surrogate.md`) outlines option (i): grow the inner-MC count from 84 to ≈84·N_ξ by sampling `(w_z0, w_y0)` per evaluation. That step is exactly where PCE substitutes. Specifically:
  - Today: each BO/CEM evaluation calls `sweep_landing_centroid(θ_i)` once → one ballistic trajectory → 84 dSMC sims.
  - With ξ-MC: `sweep_landing_centroid(θ_i, ξ_j)` for `j = 1..N_ξ` → N_ξ ballistic trajectories → 84·N_ξ dSMC sims.
  - With PCE: surrogate `F_PCE(θ, ξ)` returns the deployment-state vector analytically; **the dSMC sims still run 84·N_ξ times** because the dSMC reachability map is what defines stable. The PCE skips only the `eom2` ODE solve.
- **Cost accounting.** `eom2` is ~50 ms. dSMC trial is ~1.4 s × 84 trials ≈ 2 min, divided by 4 workers ≈ 30 s. PCE eliminates the ~50 ms `eom2` call, an **0.04% saving**. Even at N_ξ=16 the eom2 cost is ~0.8 s/eval vs ~30 s of dSMC — **2.6%**. This is the killer realism check the report's "1–2 orders of magnitude" claim has to be measured against; that claim refers to MC-replacement of the *integrated outer cost*, but here the integrated outer cost is dSMC, not eom2. **PCE does not accelerate the dominant kernel.**
- **Where PCE actually has value.** Two places, both contingent:
  1. *Sensitivity attribution.* PCE Sobol' indices over `(launch, ξ)` will quantify how much the deployment-state covariance is driven by Magnus/drag/wind vs by `(Vo, el, az, p)`. This is genuinely useful as a one-time analysis — and it's free if Plan C is ever attempted. If Sobol' shows ξ-coupling is small, Plan A's option (ii) (hold ξ deterministic) is justified rigorously instead of by hope.
  2. *Analytic deployment moments.* If the Plan A objective is reformulated to use deployment-state moments directly (e.g., `J = function-of-mean-and-covariance(x_d)`) rather than convolved through dSMC, then PCE replaces the entire ξ-MC. This is the regime Vittaldev–Russell–Linares JGCD 2016 operates in for spacecraft uncertainty propagation; it would give the report's claimed 10–100× speedup. But it requires giving up the dSMC reachability heatmap as the recourse value, which contradicts the project's whole motivation for Plan A/B.
- **`eom2.m` wrapper.** Two small needs: (a) inject Magnus/drag scaling factors via the `env` struct without rebuilding the lookup tables — multiply the four lines reading `C_M_pa`/`C_M_a0`/`C_D_0`/`C_D_del2` (lines 280–294 of `eom2.m`) by `(1+ε_*)`; (b) return the apogee-state vector directly so PCE training does not have to re-run the arc-length sampling. A new function `eom2_for_pce(t_max, θ, ξ, env_base)` that returns just `x_apogee ∈ R^12` is the right shape. **No edit to existing `eom2.m`.**
- **Interface to Plan A/B's outer optimizer.** PCE is a drop-in for the inner ξ-MC. The outer BO/CEM code is unchanged — it sees the same `(Q, U, σ_Q², σ_U²)` tuple from `bo_objective`/`cem_objective`; the wrapper just calls `pce_eval` instead of `eom2` to obtain the deployment-state samples, then feeds those into `sweep_landing_centroid`'s parsim wiring. Because of the cost accounting above, the outer wall stays roughly the same.

## Implementation roadmap

1. **Days 1–2: scope-confirmation gate.** Profile a single `sweep_landing_centroid` call with `tic/toc` around (i) `eom2`, (ii) the per-trial Simulink calls, (iii) the post-pass aggregations. **Verifiable: the eom2 fraction is logged. If it is <5% of wall, the plan stops here, gets parked, and a one-line note is added to `docs/README.md` explaining why.** This is the single most important step in this plan.
2. **Days 3–4: UQLab install + smoke test.** Register at uqlab.com, install in MATLAB, run the bundled PCE example. Confirm version compatibility with the project's MATLAB release. Build a 2-D sanity-test PCE on `eom2` over `(Vo, el)` only with Legendre basis, degree 3, ~50 LHS samples, validate against a 500-sample MC.
3. **Days 5–7: full input-distribution definition.** Define UQLab `Input` for ζ = (4 launch + 6 ξ) with the report's distributions (Caveat line 211 for Magnus; existing LHS box for launch; reasonable Gaussian for `w_z0`/`w_y0`/`alpha_0`/`beta_0` calibrated against `Mortar_Sim.m` outputs). Write `eom2_for_pce.m` that perturbs `env` and returns 12-vector apogee state.
4. **Days 8–11: PCE fit + cross-validation.** Generate 300–500-sample design (LHS or Sobol'), run `eom2_for_pce` per sample (~30 s wall), fit one PCE per output channel via UQLab LARS. Cross-validate on a held-out 100-sample MC; target normalized Q² ≥ 0.95 per channel.
5. **Days 12–13: Sobol' analysis + Plan A/B coupling.** Compute Sobol' indices analytically. If Magnus/drag indices are negligible, recommend Plan A option (ii) (deterministic ξ) and *halt Plan C development*. If they are large, build `pce_eval.m` and wire it into `bo_objective.m`/`cem_objective.m` as the inner-MC replacement. Run a side-by-side BO comparison on 20 launch points: brute MC inner vs PCE inner. Threshold: `|J_PCE − J_MC| < 0.02` per launch.
6. **Day 14: documentation.** Update `docs/README.md` with the entry-point, the Sobol' findings, and (if applicable) the new schema for `bo_log.mat`/`cem_log.mat` reflecting PCE-replaced inner MC.

Total: ≈ 14 dev-days *if step 1 passes*. If step 1 fails (likely outcome), total is ~2 dev-days plus the parking note.

## Computational budget

- **eom2 cost** (one trajectory): ~50 ms (rough estimate from `ode45` over a 100-second flight with the existing aerodynamic LUTs).
- **PCE training** (d=10, p=3, compressed sensing, 500 samples): 500 × 50 ms = 25 s for the eom2 calls; UQLab fit < 5 s. Total: < 1 minute. **Trivially small.**
- **PCE evaluation**: O(P) flops for P ≈ 286 terms, sub-millisecond per call. Effectively free vs eom2.
- **Smolyak alternative** (level-3, d=10): ~2000 nodes, ~100 s of eom2. Still trivial.
- **Plan A/B cost-saving claim**: report claims "10–100× speedup" (line 186). For the *current* pipeline (ξ-MC = 1, dSMC dominant) the realized speedup is ~0%. For a *hypothetical* future pipeline where the deployment-state moments themselves are the objective and the dSMC convolution is replaced by an analytic cookie-cutter on `(mean(x_d), cov(x_d))`, the speedup is consistent with the report's claim — but that is a different project (closer to Vittaldev–Russell–Linares' spacecraft-uncertainty regime).
- **Verdict.** Computationally free to build, but not where the bottleneck lives.

## Validation strategy

- **PCE convergence study.** Sweep total order p ∈ {2, 3, 4, 5}; plot leave-one-out error per output channel. Per Xiu–Karniadakis 2002, smooth maps converge exponentially in p; a hard plateau or oscillatory error signals a non-smooth output (e.g., one of the 12 outputs is dominated by a discrete event in `eom2`'s integrator). Per the multi-element PCE literature (Pettit–Beran-style ME-PCM; ScienceDirect S0021999122008269), a Gibbs-like signature would require a multi-element decomposition — but `eom2`'s apogee-state map is expected to be smooth in ζ because the impact event terminates *after* apogee. Confirm empirically.
- **MC ground truth.** N_MC = 10,000 `eom2_for_pce` calls (≈8 minutes wall) is the canonical reference. Compare PCE-derived mean and covariance per output to MC mean/cov; threshold `|μ_PCE − μ_MC| / σ_MC < 0.05` per channel.
- **Sobol' index sanity.** Analytic Sobol' indices from PCE coefficients (UQLab does this for free); confirm the sum equals 1 ± 0.02 and that one or two dimensions dominate (consistent with physical intuition: `Vo` and `el` should dwarf the others at high ranges).
- **Coupling validation.** If proceeding past day 13, repeat one BO iteration with PCE-inner vs MC-inner; recommended `θ*` should differ by < 1% in normalized box coordinates.

## Risks & mitigations

1. **Wrong-cost-center risk (DOMINANT).** PCE accelerates `eom2`, not the dSMC sweep. If the bottleneck stays dSMC, Plan C delivers ~0% speedup to the outer optimization. *Mitigation*: hard gate at step 1 of the roadmap. Park Plan C unless and until dSMC is itself surrogated (out of scope for this plan).
2. **Dimensionality.** d=9–10 is right at the edge where Smolyak still beats MC by an order of magnitude (per the sparse-grid review on Stokes–Darcy and finance applications, level-3 in d=10 is ~tractable). Above d=10 the curse bites. *Mitigation*: lock the input dimension during step 5; if Sobol' shows three or more `ξ` dims dominate, drop the rest. Compressed-sensing PCE (Peng–Hampton–Doostan; ETH RSUQ-2020-002C) tolerates higher d than Smolyak — this is the principal reason to default to LARS over sparse grids at d=9.
3. **Magnus uncertainty being non-Gaussian or heavy-tailed.** A symmetric Gaussian on `c_Magnus` is convenient but the CFD-UQ literature reports asymmetric distributions for spinning-projectile Magnus moments. *Mitigation*: treat Magnus as a uniform on `[0.7, 1.3] × C_M_pa_nominal` (Legendre basis) for v1 — bounded support, no tail-extrapolation hazard. Refit with a fitted distribution only if validation fails. Wiener–Askey supports either.
4. **Non-smooth outputs.** A handful of the 12 outputs (notably the Euler-angle channels) might cross discontinuities in ζ even before the impact event. *Mitigation*: leave-one-out error per channel will surface this. If one channel exhibits Gibbs behavior, switch *that channel only* to a multi-element PCE (ME-PCM, ScienceDirect S0021999122008269) with manually placed split planes; the smooth channels stay on global PCE. UQLab does not natively ship ME-PCM but it's a wrapper-only addition.
5. **Reproducibility of the random aero perturbations.** `eom2_for_pce` injects multiplicative random scalings; if those scalings are not seed-controlled, the PCE training cannot be reproduced. *Mitigation*: require `pce_build.m` to fix `rng(seed)` before generating the design and pass `seed` into `pce_log.mat`. Trivial but easy to forget.

## Citations

Research-report grounding:
- [`docs/ballistic_targeting_optimization.md`](ballistic_targeting_optimization.md) lines 22, 113, 177–186, 209–214.

Foundational gPC and stochastic collocation:
- Xiu, D., & Karniadakis, G. E. (2002). "The Wiener–Askey Polynomial Chaos for Stochastic Differential Equations." *SIAM J. Sci. Comput.* 24(2), 619–644. [DOI 10.1137/S1064827501387826](https://epubs.siam.org/doi/10.1137/S1064827501387826).
- Witteveen, J. A. S., & Iaccarino, G. simplex stochastic collocation (Stanford CTR). Sparse-grid review of moderate-d Smolyak: [arXiv:1509.01462](https://arxiv.org/abs/1509.01462).

Aerospace precedents (PCE for trajectory uncertainty):
- Vittaldev, V., Russell, R. P., & Linares, R. (2016). "Spacecraft Uncertainty Propagation Using Gaussian Mixture Models and Polynomial Chaos Expansions." *J. Guidance, Control, and Dynamics* 39(12), 2615–2626. [DOI 10.2514/1.G001571](https://arc.aiaa.org/doi/10.2514/1.G001571).
- Boutselis, G. I., Pan, Y., De La Torre, G., & Theodorou, E. A. (2017). "Stochastic Trajectory Optimization for Mechanical Systems with Parametric Uncertainties." [arXiv:1705.05506](https://arxiv.org/abs/1705.05506); journal: SIAM J. Sci. Comput. 2019.
- Nakka, Y. K., & Chung, S.-J. (2021). "Trajectory Optimization of Chance-Constrained Nonlinear Stochastic Systems for Motion Planning Under Uncertainty" (gPC-SCP). [arXiv:2106.02801](https://arxiv.org/abs/2106.02801); IEEE T-RO 2023.

Sparse / compressed-sensing PCE:
- Sharma, H., Novák, L., & Shields, M. D. (2024). "Physics-constrained polynomial chaos expansion for scientific machine learning and uncertainty quantification." *CMAME* 431, 117314. [arXiv:2402.15115](https://arxiv.org/abs/2402.15115).
- Adcock, B., Brugiapaglia, S., & Webster, C. G. (2017). "Polynomial approximation of high-dimensional functions via compressed sensing." [arXiv:1703.06987](https://arxiv.org/abs/1703.06987).
- Lüthen, N., Marelli, S., & Sudret, B. (2020). "Sparse Polynomial Chaos Expansions: Literature Survey and Benchmark." [ETH RSUQ-2020-002C](https://ethz.ch/content/dam/ethz/special-interest/baug/ibk/risk-safety-and-uncertainty-dam/publications/reports/RSUQ-2020-002C.pdf).
- Peng, J., Hampton, J., & Doostan, A. (2014). "A weighted ℓ1-minimization approach for sparse polynomial chaos expansions." JCP.

Multi-element PCE for non-smooth maps:
- Multi-element non-intrusive PCM with agglomerative clustering for irregular/discontinuous QoIs. *J. Comput. Phys.* 2023. [ScienceDirect S0021999122008269](https://www.sciencedirect.com/science/article/abs/pii/S0021999122008269).

Aerospace UQ review:
- Yao, W., Chen, X., Luo, W., van Tooren, M., & Guo, J. (2011). "Review of Uncertainty-Based Multidisciplinary Design Optimization Methods for Aerospace Vehicles." *Progress in Aerospace Sciences* 47(6), 450–479. [DOI 10.1016/j.paerosci.2011.05.001](https://doi.org/10.1016/j.paerosci.2011.05.001). (Note: the report cites this as 2016, but the canonical paper is 2011.)

MATLAB/Python toolboxes:
- UQLab v2 (Marelli & Sudret, ETH Zürich). BSD-3-clause since 2022. [www.uqlab.com](https://www.uqlab.com/) and [release news](https://sudret.ibk.ethz.ch/news-events/UQLab-news/2022/02/UQLab-V2-0.html).
- chaospy (Feinberg & Langtangen, J. Comput. Sci. 2015). [github.com/jonathf/chaospy](https://github.com/jonathf/chaospy).
- PoCET (Petzke, Mesbah, Streif, IFAC 2020). [arXiv:2007.05245](https://arxiv.org/abs/2007.05245).

Related Plan files:
- [`docs/plan_A_bo_gp_surrogate.md`](../plan_A_bo_gp_surrogate.md).
- [`docs/plan_B_cem_outer_loop.md`](plan_B_cem_outer_loop.md).
