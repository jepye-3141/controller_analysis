# Implementation Strategies: Optimizing Launch Parameters and the Transition Point for Landing Accuracy

*Compiled 2026-07-09 for the ATLIS Sims project. Companion to `2026-07-09_literature_review_launch_parameter_targeting.md`, which supplies the citations referenced below by author/year. This is a strategy document, not a set of code changes — several steps below need an explicit go-ahead or an operational input from you before implementation, and those are flagged.*

## What the codebase already does

The targeting stack today is a deterministic, surrogate-based pipeline. `eom2` (McCoy Ch. 9, 6-DOF) maps the six launch parameters `θ = (Vo, el, az, w_z0, w_y0, p)` to an apogee deployment state. `sweep_landing_centroid(traj_params, visualize)` deploys the dSMC swarm from that state across a deploy × cross-track × neighbor grid (~140 valid trials), scores each trial with `ballistic_success.m` (full-duration survival, final XY within 10 m, velocity-ratio bound, and the windowed rotation-rate bound), and returns a power-weighted landing centroid `p_centroid`, a `reachability_pct`, and a 20-bin `radial_profile` with a `half_radius`. `centroid_lookup_table.m` LHS-samples the launch box and calls the sweep per sample; `surrogate_optimize.m` fits four `fitrgp` GPs (cx, cy, reach, half_radius) and minimizes `J(θ) = ‖(c_x,c_y) − p_target‖² − λ·half_radius` by 8-start SQP, with `p_target = (600, 0, 0)` and `λ = 50` as defaults.

This already implements the surrogate-based-optimization recipe (Forrester & Keane; the DACE lineage) over a low-dimensional launch box. The literature review's job was to find what the field does that this pipeline does not yet do, and where each addition plugs in. Five candidate strategies follow, ordered roughly by increasing effort. They are not mutually exclusive — A is a prerequisite for the rest, and B/C/E compose.

## Two prerequisites that gate everything

Before any new optimization work, two regeneration items from the project history have to be settled, because every strategy below consumes their outputs.

The stored lookup and surrogate logs are criterion-stale. `logs/centroid_lookup_log.mat` (2026-05-09) and `logs/surrogate_optimize_log.mat` (2026-05-12) predate the `ballistic_success` velocity- and rotation-ratio clauses, so the success ratios, centroids, and every GP fit on them are inconsistent with the current criterion. `centroid_lookup_table.m` needs a re-run (~5 h) before any surrogate is trusted next to the paper's numbers. This is noted in CLAUDE.md under Targeting optimization; I am re-flagging it because it is the true starting line.

The ballistics changed on 2026-07-07. The `eom2` `h0` cross-product fix plus the deliberate `launch.p = −8.379 rad/s` axial spin shifted the deploy state (apogee 9.54 → 9.90 s). Any launch-parameter study must be built on the post-fix `eom2`; results generated before that date are not comparable. Since `centroid_lookup_table.m` has to be re-run for the criterion anyway, the two regenerations fold into one pass.

Recommendation: treat "re-run `centroid_lookup_table.m` on current `eom2` + current `ballistic_success`, then re-fit `surrogate_optimize.m`" as step zero. It costs one overnight run and makes the existing pipeline citable again, independent of which strategy you pick next.

## Strategy A — Tighten and extend the existing deterministic surrogate

**What it is.** Keep the current GP-surrogate-plus-SQP architecture and improve it along the two axes the review flagged as under-exploited: the objective and the decision vector. First, replace the `half_radius = NaN` clamp with the integrated-reachability area metric already sketched in `deterministic_optimization_approach.md` (`area_proxy = trapz(r_bins, mean_ratio)`), which is finite for every sample and removes the 12-of-20 clamp artifact. Second, sweep `λ` to trace the accuracy-versus-robustness Pareto front rather than fixing it at 50, which is cheap because it is post-hoc on the fitted GPs (no resimulation).

**Backing literature.** Forrester & Keane (surrogate-based optimization and infill); Frazier (the deterministic case collapses the acquisition to argmax-of-mean, which is what the SQP already does). This is the "canonical DACE recipe, done more carefully" path.

**Integration points.** `surrogate_optimize.m` lines 25–34 (the `y_hr` / NaN-clamp block) and 51–53 (the `J_obj` definition); `sweep_landing_centroid.m` already returns `radial_profile.mean_ratio`, so `area_proxy` needs no new simulation. Add a `λ`-sweep loop around the existing `fmincon` multi-start and a Pareto-trace plot alongside `SO_01`.

**Effort / risk.** Low, roughly 1–2 days after step zero. Pure MATLAB, no new dependencies, no Simulink touch. Risk is minimal: the changes are to the surrogate/objective layer, and the existing `verify=true` ground-truth check bounds any regression.

**Verification.** Leave-one-out RMSE on the four GPs against the re-run LHS; one ground-truth `sweep_landing_centroid` at the predicted optimum, accepting if the predicted and actual centroids agree within one inner-grid cell (~50 m), reusing the assertions already at `surrogate_optimize.m:167–173`.

## Strategy B — Turn the sweep into a data-efficient reachability model

**What it is.** The current sweep grids the deploy × cross-track × neighbor space uniformly, which spends most of its ~140 trials away from the interesting boundary — the edge of the dSMC's authority, which CLAUDE.md notes is sharp. Replace or augment the uniform grid with an adaptive Gaussian-process *classifier* on feasible-versus-infeasible deployment states, sampling new points where the classifier is least certain (i.e., near the boundary). This produces the same reachable-set object the sweep approximates, at a fraction of the trials, and cleanly handles the zero-success regions that currently make `half_radius` return NaN and force the degenerate-case handling.

**Backing literature.** Devonport & Arcak (adaptive GP-classification reachable sets with probabilistic correctness); Blackmore et al. (the minimum-landing-error convex program is the analytic statement of the same reachable/unreachable boundary, useful as a cross-check on a few points).

**Integration points.** A new wrapper around `sweep_landing_centroid`'s inner trial loop that (i) treats the per-trial `stable` flag from `ballistic_success` as the binary label, (ii) fits a `fitcgp`/`fitcsvm`-style classifier on the deploy/cross-track/neighbor coordinates, and (iii) proposes the next trial at the maximum-uncertainty point instead of the next grid node. The parsim wiring in `sweep_landing_centroid.m` (the 7 static + 3 per-trial `setVariable`s) is reused unchanged — only the *order and choice* of sampled points changes, not the model interface.

**Effort / risk.** Medium, roughly 4–7 days. Statistics & ML Toolbox is already required. Main risk is that adaptive sampling interacts with the 12-worker parsim batching (adaptive points are inherently sequential); mitigate by sampling in small parallel batches at the current uncertainty frontier rather than one at a time.

**Verification.** Hold out a dense uniform sweep as ground truth; check that the classifier's decision boundary and the resulting reachability/half-radius agree with the uniform result within tolerance while using materially fewer trials.

## Strategy C — Change the objective from centroid miss to target-weighted reachability

**What it is.** The current objective minimizes squared distance from the landing *centroid* to `p_target`. Both the airdrop and missile-guidance communities argue for maximizing the overlap of a probabilistic hit function with the target distribution instead. The ATLIS reachability field is already such a function: `sweep_landing_centroid` builds a `scatteredInterpolant` of the landing-success ratios (the object behind TO_07's centroid). Convolve that field with a target distribution `p_target(x)` to get `Q(θ) = ∫ R(x | θ)·p_target(x) dx`, and optimize `Q` (optionally minus a control-effort penalty `U`, computable from the per-trial `ctrl` traces already stored in the trajectory struct). This rewards launches whose *whole reachable footprint* covers the target, not just launches whose mean lands on it.

**Backing literature.** Leonard et al. 2017 (desired impact distribution as an input, dispersion shaping); Mudrik & Oshman (optimize the probabilistic hit function directly rather than miss distance); Rogers & Slegers (rank candidates by simulated landing statistics).

**Integration points.** A wrapper — call it `target_weighted_reachability(out, p_target)` — that consumes the sweep's existing landing-ratio interpolant and a user-supplied `p_target(x)` kernel. Feed `Q` (and optionally `Q − λ·U`) into `surrogate_optimize.m` as the GP training target in place of, or alongside, the centroid GPs. The control-effort surface `U` reuses the per-trial `ctrl` field the sweep already retains (currently consumed only for the TO_06 figure).

**Effort / risk.** Medium, roughly 3–5 days after A. The one required input is operational and is yours to specify: what is `p_target` as a *distribution* (a point, a Gaussian with some σ, an obstacle-shaped mask)? The current pipeline assumes a point. Risk is low and additive — the centroid objective can stay as a baseline for comparison.

**Verification.** Assert `Q` reduces to `reachability_pct` when `p_target` is uniform over the grid (a unit test the banked Plan A already proposed); confirm the `Q`-optimal launch differs from the centroid-optimal launch only when the footprint is asymmetric about the target.

## Strategy D — Introduce dispersion and move to a stochastic outer loop

**What it is.** Everything above is deterministic in the drone initial condition, as CLAUDE.md and the in-repo notes state plainly. The airdrop and projectile-dispersion literature lives on the stochastic side, where the object of interest is the deployment-state *distribution* produced by launch, wind, and aero uncertainty. This strategy injects those dispersions into `eom2` and the deployment hand-off, making the sweep a Monte-Carlo pipeline, and then adopts a noise-tolerant outer optimizer. Two banked plans already exist for this in `docs/archived/`: Plan B (cross-entropy method) as the pure-MATLAB MVP, and Plan A (knowledge-gradient Bayesian optimization) on top of its cache. iCEM's colored-noise sampling and elite reuse cut the number of expensive sweeps per iteration. For risk-aware acceptance criteria ("P[miss > 30 m] < 10%"), Plan D Track 1 (CVaR) and covariance steering (Ridderhof & Tsiotras) supply the terminal-dispersion machinery.

**Backing literature.** Ilg–Rogers–Costello and Głębocki–Jacewicz (6-DOF Monte Carlo dispersion, the forward model, and a near-exact MATLAB/Simulink methodological match); Kobilarov and Pinneri et al. (CEM / iCEM outer loop); Frazier (knowledge-gradient BO for noisy expectations); Ridderhof & Tsiotras and Chen et al. (covariance/dispersion control for risk-awareness).

**Integration points.** Add dispersion inputs to `eom2` (wind, aero-coefficient, and launch-rate scatter) and sample the deployment state per Monte-Carlo draw; the sweep's parsim structure extends naturally since new outputs need no wiring. The outer loop is a new driver (`cem_run.m` or the banked `bo_run.m`) replacing `centroid_lookup_table.m`'s uniform LHS. This is the point at which `w_z0, w_y0` are promoted from decision variables to disturbances, per the open question in `deterministic_optimization_approach.md`.

**Effort / risk.** High, roughly 2–4 weeks depending on how much of Plans A/B is reused. This is the largest step and the one to defer until a stochastic requirement is actually articulated — the review is explicit that until dispersions exist, the noise-tolerant machinery is solving a problem the pipeline does not have. The compute-feasibility concern (many sweeps × Monte-Carlo draws) is answered by the GPU-Monte-Carlo precedents, but ATLIS runs on a 12-worker CPU pool, so budget accordingly (CEM at ~200 sweeps ≈ hours-to-overnight per mission).

**Verification.** Validate the CEM/BO loop on a synthetic closed-form objective before paying simulation cost (both banked plans specify this); hold out a small set of launch points evaluated with large Monte-Carlo ensembles as ground truth for the surrogate.

## Strategy E — Promote the transition/deployment point to a decision variable

**What it is.** ATLIS deploys at apogee by construction. The one airdrop paper that lets the transition point vary as a decision variable (Leonard et al. 2020) reports accuracy gains from co-optimizing it. For a tumbling dSMC-stabilized swarm there is a real trade to explore: deploying before apogee gives more altitude margin for tumble arrest but a more energetic separation state; deploying after apogee gives a gentler state but less time and altitude. This strategy adds the deployment trigger (a time offset from apogee, or an altitude/state threshold) as a seventh decision variable and lets the optimizer choose it.

**Backing literature.** Leonard et al. 2020 (variable transition altitude improves accuracy and dispersion shaping); Delaune et al. (deliberately choosing the mid-air hand-off state to make the powered phase succeed).

**Integration points.** The deployment state is currently taken at the `eom2` apogee index. Parameterize the extraction point (e.g., `t_deploy = t_apogee + Δt`, with `Δt` a bounded decision variable) in whatever wrapper calls `eom2` before `sweep_landing_centroid`, then let `surrogate_optimize.m` (or the Strategy-D outer loop) range over the extended vector. This is cross-cutting: it composes with A, C, and D.

**Effort / risk.** Medium, roughly 3–6 days, mostly in re-parameterizing the deploy-state extraction and re-running the lookup with the extra dimension (which enlarges the LHS budget). Risk: adding a dimension raises the LHS/sweep count for the same coverage; keep the range tight around apogee at first.

**Verification.** Confirm that `Δt = 0` reproduces the current apogee-deployment results exactly; check whether the optimizer moves `Δt` off zero and, if so, quantify the accuracy gain against the fixed-apogee baseline.

## Optional augmentation — reduced-order ballistics for multi-fidelity screening

If the ballistic phase ever becomes the compute bottleneck (it is currently ~0.04% of sweep wall time, per the in-repo notes, so this is not urgent), the Modified Projectile Linear Theory (Hainz & Costello) or the approximate-moment predictor (Demir & Singh) can serve as a cheap screening fidelity to pre-filter the launch box before spending 6-DOF `eom2` calls, in a multi-fidelity optimizer. The standardized STANAG 4355 / NABK kernel (Corriveau) is worth knowing if an auditable, standards-traceable ballistic model is ever wanted alongside the McCoy implementation. Both are noted for completeness, not recommended now — the dSMC sweep, not `eom2`, is the cost center.

## Strategy comparison

| Strategy | Core papers | New deps | Simulink touch | Effort | When to pick it |
|---|---|---|---|---|---|
| Step zero — regenerate LUT/SO | (criterion + eom2 fix) | none | none | 1 overnight | Always first |
| A — tighten deterministic surrogate | Forrester–Keane; Frazier | none | none | Low (1–2 d) | Default next step |
| B — adaptive reachability classifier | Devonport–Arcak; Blackmore | none | none | Med (4–7 d) | If the sweep grid is wasteful near the stability edge |
| C — target-weighted reachability objective | Leonard 2017; Mudrik–Oshman | none | none | Med (3–5 d) | If the footprint is asymmetric about the target, or `p_target` is a distribution |
| D — stochastic pipeline + CEM/BO | Ilg–Rogers–Costello; Kobilarov; Pinneri; Frazier; Ridderhof–Tsiotras | possibly pyenv (BO) | eom2 dispersion inputs | High (2–4 wk) | Only once wind/aero/IC dispersion is a stated requirement |
| E — transition point as decision var | Leonard 2020; Delaune | none | deploy-state extraction | Med (3–6 d) | To test whether apogee is the best hand-off; composes with A/C/D |

## Recommended sequence

Do step zero regardless. Then Strategy A makes the existing pipeline criterion-consistent and Pareto-aware for low cost. Strategy E is the highest-insight-per-day addition after A, because it tests an assumption (deploy-at-apogee) that has never been relaxed and that the closest published analog says is worth relaxing. Strategy C is the natural objective upgrade once you can state `p_target` as a distribution. Strategies B and D are larger investments: B whenever the sweep's uniform grid becomes the bottleneck, and D only when a genuine stochastic/robustness requirement is articulated — at which point the banked Plans A/B/D in `docs/archived/` are the right starting points and this review's Theme 6 is the reading list.

Per the project's working style, none of the above should land as code without your sign-off on (i) the step-zero regeneration, (ii) the operational definition of `p_target` for Strategy C, and (iii) whether the transition point is genuinely free to vary or fixed at apogee by mission constraint. Those three answers determine which strategies are even in scope.
