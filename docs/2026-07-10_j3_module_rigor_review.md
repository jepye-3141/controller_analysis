# J3 Optimization Modules — Rigor / Feasibility / Quality Review (2026-07-10)

**Scope.** The optimization modules under consideration for the journal delta: the **executed foundation** (`j3_lut_regen.m` + `j3_surrogate_refit.m` + the promoted canonical `logs/surrogate_optimize_log.mat`), the **remaining W1 items** (N-raise, probe verifications, W1.1 port), **W2 / Strategy E** (deployment point as a decision variable), **W3 / Strategy C** (target-weighted reachability objective), and the **Strategy-B deprioritization** decision. Strategy D (stochastic outer loop) stays banked and was not re-reviewed.

**Method.** Four independent Fable reviewers (GP/statistics rigor; W2 design; W3 design + Strategy B; code quality/consistency), each with full source access, plus an orchestrator-run numerical integrity check of the two canonical logs. The two highest-consequence claims (the eom2 unit-mixing bug and the W2 premise mismatch) were then **independently re-verified against source by the orchestrator** before inclusion. All file:line anchors below were produced by a reviewer that read the cited file.

## 1. Verdict summary

| Module | Rigor | Feasibility | Quality |
|---|---|---|---|
| Executed foundation (LUT + refit + promoted SO log) | Sound skeleton (per-fold LOO, exact determinism check, ground-truth-anchored claims); 3 journal-grade gaps: area_proxy support incommensurability, zero posterior-variance usage, unsupported "LOO overstates local error" claim atop a missed cx acceptance | All fixes cheap: hr recompute needs no resim; maximin infill + local batch ≈ 1 h at measured warm rates | Code clean and well-guarded; safety rests on prose not in-script guards (two overwrite traps, unscripted promotion, thin provenance) |
| W1 remaining | Adequate once probe placement and infill scheme are corrected (no LHS-union) | Trivial (~1 h sim + minutes of fitting) | — |
| W2 / Strategy E | **Fails as specified** — premise targets an apogee extraction the sweep does not perform; Δ-comparison confounded 3 ways (deploy-local velocity normalization, no ground plane, flight-time-growing attitude artifact) | High once respecified (arc-window shift Δs; transect-first ≈ 20–30 min sim) | Genuine headline contribution if re-scoped; would not survive review as drafted |
| W3 / Strategy C | **Spec written against data the pipeline discards**; unit test ill-posed as stated | High after a ~1–1.5 h schema-extended LUT re-run (~10 KB/row) | Planning layer lags the code's actual data flow ("without regret" gate, false ctrl-availability claim) |
| Strategy-B deprioritization | **Category error** — outer-box evidence used to refute an inner-grid strategy | — | Verdict text needs rewriting on cost/estimand grounds |

**Bottom line.** The foundation is fit to *search* with — its headline claim (θ\* verified 9.4 m / reach 0.779 vs operating point 92.0 m / 0.550) is ground-truth-vs-ground-truth and stands. It is **not yet fit to quote as an emulator** (cx LOO 85.8 m > 50 m acceptance, no σ reporting, hr target incommensurable across samples). Both planned modules need respecification before any samples are spent, and one **newly confirmed pre-existing bug in `eom2.m`** (finding B4) needs an author decision because it gates whether the next regeneration is the *last* one.

## 2. Numerical confirmations (orchestrator log check, 2026-07-10)

From `logs/surrogate_optimize_log.mat` + `logs/centroid_lookup_log.mat`:

- SO log schema complete and self-consistent: `hr_mode=area_proxy`, `lambda=50`, `theta_best == lam_results(λ=50).theta` exactly; `ok_centroid` 41/41; `loo = {cx 85.8, cy 55.7, reach 0.111, hr 48.1}` matches all doc quotes; `y_reach ∈ [0.143, 0.914]`, `y_hr ∈ [79.8, 400.6]`, no NaN.
- **`r_bins` outer edge spans 286.8 → 852.7 m across the 41 rows (min/median/max 286.8/528.8/852.7)** — a ~3× spread in the radial-profile support, numerically confirming finding A1.
- A naive `trapz(r_bins, mean_ratio)` on rows 1 and 41 returns **NaN** — `mean_ratio` contains NaN bins (empty annuli), so the stored `y_hr` values come from the refit's NaN-masked integration; the *effective* integration support is irregular per sample (also feeds A1).
- **`gp_hr` predicts 415.7 at θ\*, above the training maximum 400.6** — the optimizer selected a point predicted to beat every observation on the robustness axis (mild extrapolation; the ground truth ~408 held up, but this is exactly the pattern that demands σ reporting — A2). Cross-check: `J_best = −20779 ≈ 8 − 50·415.7` ✓.
- LUT row fields: `params, p_centroid, reachability_pct, half_radius, radial_profile, ballistic_solution, is_reference` — **no per-trial landing coordinates, labels, or traces anywhere in the log** (confirms C1). `is_reference` on row 41 only ✓.
- Reference ballistics (row 41): apogee altitude **391.0 m**, impact **20.09 s**, range 866.8 m, cross-range 376.4 m. Speed profile about apogee: **44.0 m/s at apogee → 47 m/s at ±2 s → 55–56 m/s at ±4 s**; altitude 331–334 m at ±4 s. (Confirms B2's velocity-bound loosening; bounds B3's Δ envelope.) Footnote: the `ballistic_solution.apogee` **field holds the apogee altitude (391.01), not the apogee time** — docs quoting "apogee 9.90 s" mean `time(apogee_idx)`; worth one clarifying line wherever the field is described.

## 3. Findings A — executed foundation (Strategy A core)

**A1. MAJOR — `area_proxy` integrates over a per-sample, θ-dependent support measured from a per-sample center; the 41 training values are not strictly commensurable.**
Anchor: `sweep_landing_centroid.m:306-355`, `j3_surrogate_refit.m:84, 200-208`; numeric §2.
The radial profile's grid extent is the bounding box of that sample's landing points (scales with Vo/el/az), radii are measured from that sample's own power-weighted centroid, `r_edges = linspace(0, max(r_grid), 21)` makes the support the centroid-to-farthest-corner distance (measured spread 287–853 m), and NaN (hull-exterior / empty-annulus) bins are dropped, making outer bins angular-partial conditional means. For the 8/41 "wide" plateau samples the integral is grid-truncation-limited and confounds "wide reach zone" with "long trajectory footprint" — and footprint is monotone in the same variables that drive cx, so `−λ·hr` can partially reward *range* rather than robustness. The verified point suggests bounded damage (pred 415.7 vs actual ~408, −1.8%), but the definition is not journal-defensible as "integrated reachability."
**Action:** recompute the hr target on a common fixed support `R_common = min_n max(r_bins_n)` by `interp1` of the *stored* profiles (no resimulation, minutes), optionally 2πr-weighted for a true area; refit `gp_hr`; confirm θ\* stability; state the final definition (center, support, coverage weighting) explicitly in the paper.

**A2. MAJOR — pure posterior-mean optimization; posterior variance is never computed or reported anywhere in the pipeline.**
Anchor: `j3_surrogate_refit.m:127-133`, `surrogate_optimize.m:51-53` (no `[pred, sd] = predict(...)` exists in either script).
Textbook symptom observed twice: clamp-mode λ>0 optima chased imputed values to the p box edge, and θ\* pins w_y0 at its bound with a monotone slice — low-density regions where σ is highest; §2's above-training-max hr prediction is a third instance. The single T5 verification validates the mean at θ\* only.
**Action (probe placement for W1.4):** P1 the λ=0 optimum (pure-targeting end of the front); P2 the maximum-posterior-σ point within the θ\* basin/along the front; P3 θ\* perturbed ±1 ARD length-scale in its most sensitive dimension (launch tolerance — operationally meaningful); P4 (budget permitting) one maximin point as an unbiased global check. Score each as *held-out* prediction ± σ vs actual before ingesting into training; report the table.

**A3. MAJOR — "global LOO overstates error near the optimum" is an n=1 post-hoc rationalization; the honest paper position is that the cx acceptance was missed.**
Anchor: `j3_surrogate_refit.m:210-220`; runbook §6 item 4; delta plan W1.3.
The LOO itself is rigorous (per-fold hyperparameter refit; reference row included and separately reported). But 6.6 m at a single self-selected point cannot support a claim about the spatial error distribution. The mechanism (residuals dominated by sparse corners) is checkable from `pred_*` residuals vs nearest-neighbor distance — not yet done.
**Action:** either raise N until cx LOO ≤ 50 m before quoting emulator accuracy, or frame the GP as a *search heuristic whose output claims are all ground-truth-verified*, reporting: per-fold-refit LOO per target, the explicit cx miss, posterior SD at θ\*, and the multi-point verification table.

**A4. MAJOR — N=41 in 6-D is below the n≈10d guideline, and the planned "+40 samples" must not be an independent-LHS union.**
Anchor: delta plan W1.3; `j3_lut_regen.m:36, 76-96`.
Two unioned `lhsdesign` draws preserve neither stratification nor maximin distance. Also operational: `j3_lut_regen.m` hard-asserts `N_LHS==40` and the locked ranges against its checkpoint, so an infill batch needs a **new driver** (see E-group), not an in-place edit.
**Action:** greedy maximin selection from a Sobol candidate pool *conditioned on the existing 40* (~20–25 global points) + the ~10–15 θ\*-local points already planned — fixes global cx accuracy and accuracy-where-claimed simultaneously at ~1 h of sim.

**A5. MINOR — optimizer diagnostics absent.** Anchor: `j3_surrogate_refit.m:109-133` (exitflag discarded at :131).
A start that exhausts `MaxFunctionEvaluations=500` is indistinguishable from a converged one; only the winning (θ, J) per λ is kept. Surrogate evals cost milliseconds. **Action:** ≥64 starts, capture exitflags, cluster terminal points, report basin statistics (one-line paper credibility statement). Add λ ∈ {2, 5} — the Pareto knee between 0 and 10 is unresolved.

**A6. MINOR — the 92.0 m baseline is weak; also quote the best training sample.**
No circularity in the headline (both endpoints are simulator ground truth; T2 discharged the cross-driver consistency risk exactly) — but the operating point was never target-optimized. **Action:** report min ground-truth miss (and reach) over the 41 stored rows as the no-surrogate baseline; θ\*'s margin over *that* is the persuasive number. Free from the log.

**A7. NOTE — `p_target(3)` ignored is structurally immaterial (landing z ≡ 0 by construction); make the R² definition explicit in code + paper. The recipe doc (`docs/gp_surrogate_recipe.md`) still describes the superseded pipeline (N=20 basis, p ∈ [−2,2], NaN-clamp, λ "in meters") — rewrite before W4 lifts text into the manuscript.**

## 4. Findings B — W2 / Strategy E (deployment point as decision variable)

**B1. BLOCKER — the module's premise does not match the code: the sweep never deploys at apogee, and there is no single `t_deploy` to offset.** *(Independently verified by orchestrator.)*
Anchor: `sweep_landing_centroid.m:82-94, 167-175` vs delta plan §W2, runbook §5, strategies doc §E ("The deployment state is currently taken at the `eom2` apogee index" — false for the sweep).
The pipeline extracts **eight** deploy states by linear interpolation in *arc length* at 20–80% of the trajectory (`deploy_arc = linspace(0.20*arc_3d(end), 0.80*arc_3d(end), 8)`), each with its own position, velocity, angular rate, and attitude; `apogee_idx` is never referenced in the sweep (its only consumer is the `analysis.m` single-drone test). Current LUT/GP results therefore already *average over hand-off points bracketing apogee*. `t_deploy = t_apogee + Δt` is ill-posed against this pipeline.
**Action:** respecify the 7th coordinate as an additive **arc-length window shift Δs** applied at `sweep_landing_centroid.m:86` (`deploy_arc + Δs`), one code path — Δs = 0 is algebraically a no-op, giving **bit-exact W0 identity and optimizer continuity for free** (a time-based interpolation breaks exactness; a `Δt==0` branch breaks continuity). Reframe the paper question to "where along the ballistic arc should the hand-off window sit," which is what the pipeline can actually answer. Correct the premise in all three planning docs.

**B2. MAJOR — the success criterion is not Δ-comparable: the velocity bound is deploy-local and loosens off-apogee; the 1 s rotation grace is apogee-region empirics.**
Anchor: `sweep_landing_centroid.m:251-253`, `ballistic_success.m:22-24, 63, 71-82`; numeric §2 (44.0 m/s at apogee → 55–56 m/s at ±4 s, i.e. the absolute bound 2·v₀ loosens ~26% within ±4 s).
`v0_norm` is each trial's own deploy speed (already varies across today's 8 stations), so success — hence centroid, reach, area_proxy, and J(θ, Δs) — is biased toward energetic off-apogee hand-offs as a *normalization artifact*, exactly the effect the study claims to measure. The grace window's justification ("violations start ≈0.06 s, none past 3.8 s") was measured on the current lattice. Also: the landing-target lattice itself translates with the deploy window, so `p_centroid` is lattice-relative — must be stated in the paper.
**Action:** freeze scoring *before* any samples: use a Δ-invariant velocity reference (the trajectory's apogee speed per θ — one scalar, physically the gentlest-state reference — or an absolute airframe limit); keep RotGraceT = 1 s *fixed* across Δs reframed as an operational allowance; verify empirically at the Δ extremes whether arrest completes within it; report the headline Δ curve under both fixed-reference and legacy scoring. Cheap: `ballistic_success` already takes `v0_norm` as an argument.

**B3. MAJOR — no ground plane anywhere; a descending deploy can "succeed" through the floor.**
Anchor: `ballistic_success.m:73-75` (final XY only; z never checked), `sweep_landing_centroid.m:117` (`land_pos(3,:,:) = 0`), `eom2.m:166-170` (ballistics stops at altitude 0; the drone plant is a free-space ODE with only the 1e5 blowup guard).
Already latent for today's descending stations (up to 80% arc); acute for late Δs (apogee altitude 391 m, impact 20.1 s, 331–334 m at ±4 s). A naive `z(t) > 0` clause would misfire (hover at z = 0 has small negative excursions).
**Action:** (i) zero-cost now — mine `logs/trajectory_optimization_log_sat_on.mat` per-trial trajectories for min-z during arrest to quantify current violations; (ii) add an altitude-margin diagnostic to `ballistic_success` `info` (min-z before first target arrival); (iii) bound Δs by an altitude floor derived from `ballistic_solution` (deploy altitude ≥ observed arrest altitude loss + margin); (iv) cap extraction strictly inside the stored solution domain (interp past the impact-terminated grid returns NaN → errored trials).

**B4. MAJOR — `eom2.m` mixes degrees and radians in the Euler-angle channel; deploy attitude is a flight-time-dependent artifact.** *(Independently verified by orchestrator: `o0` at `eom2.m:53-55` is built from degree-valued `el_0+alpha_0` etc. — the header comment at :32 claims rad — while `do = h` at :305-307 integrates rad/s; the sweep then `deg2rad`s the whole column at `sweep_landing_centroid.m:168`.)*
Consequence: the integrated (in-flight) part of the Euler state is suppressed ~57.3×, so the deploy attitude fed to the drone stays near the *launch* attitude at every station — the true nose-follows-trajectory rotation of a fin-stabilized round (order 45° by apogee, ~90°+ late) is almost entirely absent, and the residual mis-scaled drift *grows with flight time*, making it a first-order confounder for a hand-off-timing study. Additionally `do = h` is not a proper Euler kinematic relation even unit-fixed (Euler-angle rates ≠ angular-velocity components); the physically propagated attitude of the axisymmetric projectile is the pointing vector `r` (cols 7:9). Pre-existing: this biases the deploy attitude of **all current sweep results** (a θ-dependent bias in the LUT too), though the h→ω hand-off and everything else the criterion measures are unaffected.
**Action (author decision — this is the freeze-point gate):** preferred fix is deriving deploy attitude from the pointing vector `r` (renormalized after interpolation) plus spin about it, replacing cols 13:15 consumption; alternatively document and bound the approximation in the paper. If accepted, it changes deploy states → **schedule it into the same regeneration as the C1 schema extension** so there is one re-run, not three (delta plan W4.4 consistency rule).

**B5. MAJOR — a 10–15-sample local augmentation with the w_y0 box extension folded in is confounded and underpowered for the stated claim.**
Anchor: delta plan §W2; runbook §5 Strategy-E note.
All 41 existing points lie in the Δs = 0 hyperplane, so the GP's Δs length-scale would be identified from 10–15 points that *simultaneously* step w_y0 into never-sampled territory — the two effects are unseparable in one batch. And "apogee is/isn't the optimal hand-off" as a *global* claim is not supported by a θ\*-local design.
**Action (minimal defensible design):** (i) a 1-D Δs transect at θ\*, 7–9 values including 0 — **the Δs = 0 point is free** (it *is* the T5 verification sweep: centroid (607.3, −6.0), reach 0.779) — ~10 min warm; (ii) a *separate* 1-D w_y0 transect (4–6 samples, ~7 min), gated on B6; (iii) only if both transects show effect, a joint local LHS ≥ ~10/active-dimension. A global 7-D claim needs a fresh 7-D LHS (N ≳ 60–80). Scope the paper's claim to whichever design actually runs.

**B6. MINOR — w_y0 is physically muzzle tip-off dispersion, not a launch knob.**
Anchor: `Mortar_Sim.m:18-19`, `eom2.m:20-25`; strategies doc §D already plans demoting w_z0/w_y0 to disturbances in the stochastic phase.
Extending the box to [−1, 2] because the optimizer pins at +1.00 is pin-chasing an asymmetric range for a parameter no launcher plausibly commands at 2 rad/s. **Action:** author ruling first (knob → justify actuation and extend symmetrically to a physically-argued bound; disturbance → w_y0 leaves θ and the pin dissolves). Either way, its own transect — never folded into the Δs batch.

**B7. MINOR — free hand-off data is already in-repo and ignored by the plan.**
Anchor: `sweep_landing_centroid.m:501-503` (per-station `deploy_ratio`, TO_03), `analysis.m` envelope (deploys along the *whole* flight, same criterion, fig 39), `logs/trajectory_optimization_log_sat_on.mat` (full per-trial results).
**Action:** add `deploy_ratio` (8-vector) and per-trial criteria extrema (`max_speed`, `max_rot_post`, `final_miss`, `t_end`) to `out`/LUT rows (tiny; enables re-scoring under alternative bounds *without resimulation*, defusing part of B2); mine the envelope + sat_on logs now to anchor or pre-empt the Δs transect.

**B8. NOTE — 7-D mechanics mostly generalize** (ranges/`field_names`/`lhsdesign`/`params.(f)` are d-generic; `eom2` ignores unknown fields). Required specifics: injected reference row must gain the explicit Δs = 0 coordinate (else `j3_surrogate_refit.m:56` errors); the Δs range must include 0 (box assert); the `reach == 0.55` injection assert is valid only under the frozen criterion — any B2 renormalization invalidates the free injection (re-score or re-simulate row 41); re-run the T2-style determinism check after any sweep-code change.

## 5. Findings C — W3 / Strategy C (target-weighted reachability)

**C1. BLOCKER — the required input R(x|θ) does not exist in any artifact the pipeline can reach; the stored radial profile is not a valid substitute.** *(Numerically confirmed: LUT rows carry no per-trial data — §2.)*
Anchor: `sweep_landing_centroid.m:380-389` (`out` = five summary fields; `visualize=false` returns before the whole-workspace save at :703), `j3_lut_regen.m:133-139`.
The 40 landing points, per-point success ratios, and per-trial `stable` labels die with the function workspace. The radial profile substitutes only under assumptions the code contradicts: the footprint is a ±200 m *ribbon* along a curved ground track (not azimuthally symmetric about the centroid); annulus means are conditional (NaN grid points dropped, no coverage fraction stored); the profile center is θ-dependent. A profile-convolved Q is defensible only for isotropic, broad p_target — where it degenerates to a function of centroid miss and adds nothing.
**Reconstruction inventory:** `trajectory_optimization_log_sat_on.mat` yields exactly one usable field (= LUT row 41); sat_off is 0/140 (all-zero field); the θ\* verification saved only summary `out_v`. Enough to *prototype* Q and the unit test; not enough to train any Q surrogate.
**Action (recommended):** extend the sweep's return struct with `landing_xy (40×2)`, per-point `ratio/total/success`, and per-trial `(i, k, nb, tgt, stable, final_xy, effort scalars)` — ~10 KB/row — and re-run the 41 sweeps (~1–1.5 h warm). **Sequence before the W2 augmentation** so those samples are born field-complete.

**C2. MAJOR — the uniform-reduction unit test is ill-posed as stated.**
Anchor: delta plan :29; strategies doc :55; `sweep_landing_centroid.m:294-297, 377-378`.
`reachability_pct` = Σ successes / 140 with per-target attempt counts t = [2,3,4,4,4,4,4,3] per cross-track column — not constant. Uniform-over-the-plane is non-normalizable; uniform-over-hull gives a different, interpolation-sensitive number; equal-weight-over-nodes gives mean(r) ≠ Σt·r/Σt. The exact identity: **Q = reachability_pct iff p_target = Σ (t_jk/140)·δ(x − x_jk)** — the attempt-weighted empirical measure on the landing nodes (implementable to FP precision since `scatteredInterpolant` is exact at nodes).
**Action:** restate the test as the weighted-node identity + add a constant-field quadrature test (R̂ ≡ c ⇒ Q = c·p_target-mass-inside-hull) to pin normalization and NaN conventions. (Also: Q is an overlap integral, not a convolution — fix the wording.)

**C3. MAJOR — Q inherits interpolant model error and an unspecified out-of-hull convention; for narrow p_target it measures the interpolant, not the data.**
Anchor: `sweep_landing_centroid.m:306-315` (`'natural','none'`); CLAUDE.md TO_07 method-sensitivity note.
The field is 40 scattered samples at ~100 m spacing with ratios quantized to multiples of 1/t (t ≤ 4). A point or σ ≪ 100 m target reduces Q to a pointwise interpolant evaluation between samples. Extrapolation `'none'` → NaN outside the hull; zero-fill vs renormalize changes Q materially near the edge, and zero-fill gives a zero-gradient plateau once target and footprint separate (survivable through a Q-GP; fatal for direct gradient use).
**Action:** spec the estimator: R̂ = F inside hull, ≡ 0 outside; fixed quadrature grid ≤ 5 m cells; p_target normalized analytically over R²; report in-hull mass fraction as a per-evaluation diagnostic; keep `'natural'` for TO_07 continuity but always report natural-vs-linear ΔQ as the model-error bar; constrain kernel scale ≳ the 100 m grid spacing.

**C4. MAJOR — the gate "skip W3 without regret if p_target undefined" is right for the objective work, wrong for data capture.**
Anchor: delta plan :29, :55(d); runbook §5.
Skipping W3 *and* the schema fix means the imminent W2 augmentation is generated field-blind and any later W3 pays a full re-run twice. **Action:** reword to "skip Q-*optimization* without regret; land the field-storage schema before W2 regardless." Recommended default to put to the author: **isotropic Gaussian, σ ≈ 100 m** (matches field sample spacing, cx LOO scale, ~2 acceptance cells) with σ ∈ {50, 100, 200} sensitivity — explicitly *not* σ ~ 5 m: `ReachTol` = 10 m is already baked into R's per-trial labels, so a tolerance-scale σ double-counts it and drops Q below the field's resolution.

**C5. MINOR — the strategies doc's control-effort claim is false on the LUT path.**
Anchor: strategies doc :51 ("reuses the per-trial `ctrl` field the sweep already retains") vs `sweep_landing_centroid.m:268-273, 380-389`.
`ctrl` survives only in the visualize-path whole-workspace saves; full traces are ~13 MB/sample (≈0.5 GB across 41), not "tiny." **Action:** store per-trial scalar summaries (∫‖u‖dt, ∫‖u‖²dt — 160×2 doubles/row) in the C1 schema; fix the doc sentence; define whether U averages successful or all trials before it enters any objective.

**C6. NOTE — architecture: field-level *storage* + per-target scalar Q-GP.** Refit-per-target is not a real cost (`fitrgp` at N=41 is seconds). An honest field surrogate R̂(x, θ) is an 8-D GP on 1640 binomial ratios with n ≤ 4 — a new methodology exposure the paper doesn't need while the scalar GPs already miss the cx acceptance. Field storage delivers the actual benefit (Q computable post-hoc for any target, zero re-simulation) with the surrogate layer unchanged.

**C7. NOTE — schema-change logistics:** the checkpoint config assert covers only `N_LHS`/`field_names`/`ranges`, so a new-schema run would *silently resume* onto old-schema rows. Bump a `schema_version` into the config fingerprint; delete the retained checkpoint for the from-scratch run; pick a reference-row policy (reconstruct its field from the sat_on whole-workspace log at zero sim cost, or re-simulate ~1.3 min).

## 6. Finding D — Strategy-B deprioritization (convergent: two reviewers independently)

**D1. MAJOR — the deprioritization rests on evidence from the wrong space.**
Anchor: runbook §5 B verdict vs strategies doc §B (:35-43); census runbook §6.
Strategy B targets the **inner deploy × cross-track × neighbor grid within one sweep** (per-trial `stable` flags as classifier labels). The cited evidence — zero zero-success samples among N=41, corr(|p|, reach) = −0.70 smooth — characterizes the **outer 6-D launch box**, which B never proposed to sample adaptively. The census actually proves the inner boundary exists in *every* sweep (reach 0.143–0.914 ⇒ ~12–120 failed trials per grid). "No boundary to learn" is factually wrong in B's space.
The deprioritization likely still stands, on grounds the verdict never states: warm sweeps at 1.3 min make the 140-trial grid a non-bottleneck; adaptive sampling changes the estimand (centroid/reach/radial_profile are defined on the fixed grid — non-uniform designs need reweighting to keep LUT rows comparable); sequential acquisition fights 12-worker parsim batching.
**Action:** rewrite the runbook §5 B verdict on cost/estimand grounds. Evidence that would legitimately settle it: per-sample inner-boundary sharpness statistics and a retrospective adaptive-replay (train on trial subsets, measure trials-to-recover the stable set) — both need the C1 per-trial labels; a one-off replay is possible today for the operating point from the sat_on log.

## 7. Findings E — lifecycle / quality (executed foundation)

Verified non-issues first (one line each): checkpoint write is atomic (tmp + `movefile "f"`); resume restores `X` from the checkpoint (never re-drawn — deterministic); config drift on `N_LHS`/`field_names`/`ranges` is fail-loud; archive-before-overwrite idempotent for this epoch; `ok_centroid` masking and wide/dead clamp split correct; LOO does honest per-fold refits; `out_v` `-append` correct; rendered SO_01 figure honest post-promotion; replot headers carry the units warning; **all J3-era numbers agree across CLAUDE.md / runbook / delta plan / replot headers** (9.4, 0.779, 92.0, 0.550, 1.3 min, 2.3 h, 8/41, LOO quartet, λ-flat, N=41, p box; 92.0 m and 6.6 m re-derived arithmetically ✓).

**E1. MAJOR — the regression trap lives in the one file with no warning, and that file would regress silently-successfully.**
Anchor: `surrogate_optimize.m:13-14, 33-35, 131, 134-138, 141`.
Its only in-file staleness comment (":13 — saved log is pre-criterion, stale") is now **false** (the LUT log it loads is current), inviting exactly the wrong conclusion; an as-is re-run passes the `isfinite` assert on the zero-dead log, silently re-imputes the 8 wide NaNs in meters, and overwrites both `figs/SO_01` and the canonical log *before* the verify gate can error. Recovery exists (re-promotion from the refit log), so MAJOR not BLOCKER.
**Action:** ~5-line interlock (refuse to save over a canonical log whose `hr_mode` exists and ≠ "clamp" without an explicit `allow_overwrite`), fix the false comment, add a header warning — subsumed by, not duplicating, the W1.1 port.

**E2. MAJOR — a completed, retained checkpoint makes "re-run j3_lut_regen" a no-op that re-stamps old data as fresh.**
Anchor: `j3_lut_regen.m:76-96, 148-153`; CLAUDE.md checkpoint bullet.
The config fingerprint excludes the fixed trajectory params, the reference-row literals, and all plant/criterion/sweep-code state. With the all-done checkpoint retained and the box unchanged, a re-launch skips every sim and re-saves old data to the canonical path with a fresh mtime and a success message — a realistic path to quoting stale physics as a fresh regen under the W4.4 rule ("if J1 fidelity upgrades land first, regeneration runs on the upgraded plant").
**Action:** on successful finalize, move the checkpoint to `logs/archive/` with a campaign tag (doubles as the accidental-overwrite backup — see E4); at minimum warn loudly when a resume finds `all(done)`. Extend the CLAUDE.md bullet: a retained complete checkpoint makes re-launch a re-save, not a re-simulation.

**E3. MAJOR — CLAUDE.md contradicts itself on the operating-point numbers the J3 campaign is anchored to (journal-readiness audit item H1, still open).**
Anchor: CLAUDE.md trajectory_optimization bullet ("current canonical: 83/140 = 59.3%, centroid (580.87, −241.32)") and ballistics-section lines ("must be regenerated before quoting" — it was, 2026-07-08; the superseded 19-sample envelope masks) vs the J3 sections built on the 2026-07-08 state (77/140 = 0.550, centroid (564.39, −84.80), miss 92.0).
An author sourcing headline numbers from the highest-traffic doc can pull retired values depending on which bullet they read.
**Action:** execute H1 — rewrite the status block and stale lines to the 2026-07-08 state (or add dated "superseded" markers), and annotate H1/H3 closure in the audit doc.

**E4. MINOR — `centroid_lookup_table.m` has no in-file supersession/overwrite warning**; one accidental run replaces the canonical N=41 log (no archive step) — `j3_surrogate_refit` then fails loud, but `replot_LUT_01` renders N=20 silently under a header claiming N=41. Two-line header fix mirroring CLAUDE.md.

**E5. MINOR — the promotion was performed by a script outside the repo and is otherwise unreproducible.**
The refit log omits the scalar `lambda` and `promotion_note` that the canonical schema carries; they were injected by a promotion script run from the session's temp area — grep confirms no `*promote*` script in the repo, and re-promotion will recur (every future refit). **The exact script is preserved in Appendix A of this document**; recommend committing it as `j3_promote_so_log.m` (it already asserts archive-exists, `hr_mode`, `out_v` presence, and θ_best/λ=50 consistency).

**E6. MINOR — refit provenance gaps**: no exitflags for any of the 48 fmincon solves (merged into A5); per-point LOO predictions (`pred_*`), `starts`, and optimizer options not in the save list (they back the printed reference-point check but survive only in `run_logs/`); `i50 = find(lambdas == 50)` fails cryptically if the λ list drops 50; (pedantic, clamp-only) the 1.2·max imputation is computed before the LOO split — leaks each held-out point into its own fold's imputed values.

**E7. MINOR — reproducibility metadata is thin for journal-cited logs; the weakest link is the unversioned ballistics folder.**
Captured: `wall_s`, `X`, targets, GPs, MAT-header platform/date. Missing: MATLAB/toolbox versions, driver git commit, rng seed as data, echoed fixed-params struct, and any fingerprint of `Ballistics_Simulation-master/` (gitignored, **no VCS history**) — finding B4 proves this matters twice over.
**Action:** save a `provenance` struct in both finalize blocks: `version`, datetime, `git rev-parse HEAD`, seed, fixed params, and checksums (`system("shasum ...")`) of `eom2.m`, `aero_constants.m`, and the two aero data files.

**E8. NOTE — the archive block is pinned to the 2026-05 epoch**: destinations are hard-coded `*_pre_criterion_2026-05-*`, which now exist, so the *next* campaign overwrites the J3 canonical log with no new backup (or, on a fresh clone, archives a J3-era log under a pre-criterion name). The `reach == 0.55` tripwire's "wrong log?" message also misdiagnoses the legitimate future case "sat_on regenerated on a changed plant." Timestamp archive names and cross-check the ref row against its stored launch params when next touched.

**E9. NOTE — residual doc rot (one-line fixes):** `j3_lut_regen.m` header still says 6–8 min/sample / 4.5–5.5 h (measured: 1.3 min / 2.3 h); runbook §3 "neither script has been through checkcode" is stale under an EXECUTED §0; `replot_LUT_01.m` title "LHS sample of N=41 trajectories" counts the injected reference row as LHS and never visually distinguishes it (caption dishonesty if the figure reaches the paper as-is); the delta plan's "H3 closes here" is over-broad (replot_TO_04/07 deliberately still point at the pre-criterion TO log); `ballistic_solution.apogee` holds *altitude*, not time (§2 footnote).

## 8. Recommended execution order and author decision gates

**Decision gates (author input required before the next simulation batch):**
1. **B4 — accept/reject the eom2 attitude fix** (pointing-vector-derived deploy attitude vs documented approximation). This is the freeze-point decision: if accepted later, it forces yet another regeneration of everything ballistic-dependent. Decide *before* the C1 re-run.
2. **B6 — w_y0: launch knob or disturbance.** Determines whether the box extends (symmetrically, physically argued) or the parameter leaves θ.
3. **C4 — p_target as a distribution** (recommended default: isotropic Gaussian σ = 100 m, {50, 100, 200} sensitivity) or explicit deferral of W3's objective work.

**Sequence (assuming gates resolved):**
0. Paper-blocking hygiene, no simulation: E3 (H1 doc fix), E1 (interlock + W1.1 port), A1 (common-support hr recompute from stored profiles + refit + θ\* stability check), A5/A6 (optimizer diagnostics, best-training-sample baseline), E5 (commit the promotion script).
1. **One schema-extended regeneration** (~1–1.5 h warm, new driver per A4/E2/E8/C7 with `schema_version`, timestamped archives, provenance struct): C1 fields + B7 station/extrema fields + C5 effort scalars — with the B4 attitude fix *in or explicitly out* per gate 1. This single run unlocks W3 prototyping, the legitimate Strategy-B check (D1), B2 re-scoring without resimulation, and the free Δs = 0 anchor.
2. W2 respecified per B1/B2/B5: scoring frozen first, then the Δs transect at θ\* (Δs = 0 point free), the separate w_y0 transect per gate 2, then a joint local LHS only if both transects show effect.
3. A2 probes (P1–P4) + A4 maximin infill (~1 h) → then, and only then, quote GP accuracy per A3.
4. W3 objective work per C2/C3/C6, only after gate 3.

---

## Appendix A — the 2026-07-10 promotion script (verbatim; recommend committing as `j3_promote_so_log.m`)

```matlab
% j3_promote_area_proxy.m -- Promote the area_proxy refit to the canonical
% SO log name (plan J3, author decision 2026-07-10). The pre-criterion
% canonical is already archived by copy; this overwrite is the promotion.
cd('/Users/johnpye/MATLAB/ATLIS Sims');

assert(isfile('logs/archive/surrogate_optimize_log_pre_criterion_2026-05-12.mat'), ...
    'pre-criterion archive missing -- do not overwrite the canonical log');

R = load('logs/surrogate_refit_j3_area_proxy.mat');
assert(R.hr_mode == "area_proxy", 'expected the area_proxy refit log');
assert(isfield(R, 'out_v'), 'refit log lacks the ground-truth out_v');

% Operative scalar lambda for canonical-schema compatibility; the refit's
% saved theta_best/theta_phys/J_best are already the lambda=50 entries.
lambda = 50;
i50 = find(R.lambdas == lambda, 1);
assert(isequal(R.theta_best, R.lam_results(i50).theta), ...
    'theta_best is not the lambda=50 optimum -- schema assumption broken');

gp_cx = R.gp_cx;  gp_cy = R.gp_cy;  gp_reach = R.gp_reach;  gp_hr = R.gp_hr;
theta_best = R.theta_best;  theta_phys = R.theta_phys;  J_best = R.J_best;
p_target = R.p_target;  ranges = R.ranges;  field_names = R.field_names;
Xn = R.Xn;  y_cx = R.y_cx;  y_cy = R.y_cy;  y_reach = R.y_reach;  y_hr = R.y_hr;
lambdas = R.lambdas;  lam_results = R.lam_results;  loo = R.loo;
ok_centroid = R.ok_centroid;  hr_mode = R.hr_mode;  out_v = R.out_v;

promotion_note = "Promoted 2026-07-10 from logs/surrogate_refit_j3_area_proxy.mat (plan J3 W0). " + ...
    "gp_hr/y_hr are the area_proxy target trapz(r_bins, mean_ratio) -- integrated reachability " + ...
    "(ratio-weighted meters), NOT half_radius meters. Trained on the 2026-07-10 " + ...
    "logs/centroid_lookup_log.mat (N=41 incl. reference row; zero-success samples would be " + ...
    "masked from the cx/cy GPs via ok_centroid -- none in this dataset). theta_best/J_best " + ...
    "are the lambda=50 entries of lam_results; out_v is the ground-truth sweep at theta*. " + ...
    "Pre-criterion canonical archived at logs/archive/surrogate_optimize_log_pre_criterion_2026-05-12.mat.";

save('logs/surrogate_optimize_log.mat', ...
    'gp_cx', 'gp_cy', 'gp_reach', 'gp_hr', ...
    'theta_best', 'theta_phys', 'J_best', ...
    'p_target', 'lambda', 'ranges', 'field_names', 'Xn', ...
    'y_cx', 'y_cy', 'y_reach', 'y_hr', ...
    'lambdas', 'lam_results', 'loo', 'ok_centroid', 'hr_mode', 'out_v', ...
    'promotion_note');
fprintf('promoted: wrote logs/surrogate_optimize_log.mat (lambda=%g, hr_mode=%s)\n', lambda, hr_mode);
```

*Review conducted 2026-07-10 by four independent Fable reviewer agents + orchestrator numerical verification. The two blocker-severity claims (B1, C1) and the B4 bug were each verified against source by at least two independent readers.*

---

## 9. Resolutions (2026-07-10, later same day — author decisions + B4 execution)

**Gate 1 — B4 eom2 attitude fix: APPLIED + VERIFIED (author-approved).**
- Implementation: new helper `ballistic_deploy_state.m` (body x-axis along the pointing vector r̂, φ = 0 roll about it; body→world `M` hand-built to match `system_dynamics.m`'s ZYX rows, world z-up / body z-down, `M(:,1) = r̂` exactly, world→body = `M.'`). Consumers switched: `sweep_landing_centroid.m` (interpolates cols 7:9 instead of 13:15) and both real `analysis.m` seed blocks; the five dead `rot0` lines in the fig-39 plotting loops removed. `eom2.m` changed **comment-only**.
- A finding *from* the fix: rescaling `o0` to radians — though the channel is dynamically passive — perturbed the whole trajectory (max delta 19.7, different ode45 step grid) because **solver error control weighs all state components**. The IC is therefore deliberately left in degrees (documented in-file); bit-exact reproduction of stored trajectories is preserved.
- Verification: (i) post-fix `eom2` reproduces the stored LUT row-41 trajectory to exactly 0 in all 15 columns + time; (ii) helper invariants at machine precision (nose alignment 2.2e-16, orthogonality 2.5e-16, exact `D·eul2rotm` match, plant-kinematics roundtrip 1e-14); (iii) the corrected apogee seed puts the known axial spin (h·r = −0.807) exactly on the body roll axis — an unengineered consistency check; (iv) old apogee seed quantified: φ = −0.78 rad (45° roll), body velocity [20.9, −24.6, 30.0] vs corrected [43.98, 1.38, 1.57] m/s.
- Impact (operating-point sweep, 0 errored, 4.0 min warm): reachability **0.550 → 0.293**, centroid (564.39, −84.80) → **(684.99, −197.96)**, miss-to-(600,0) 92.0 → **215.4 m**, half_radius 257.7 → 182.4, area_proxy 242.6 → 147.4. Station-resolved (ct0/nb0; old mined from the sat_on log, new = 9 single sims): **all four ascending stations fail** (stations 1/3 fly away at 7–10× the velocity bound; stations 2/4 blow up the Euler-rate signal ~2×10⁴ near the θ → ±90° kinematic singularity of the 12-state Euler plant — partly a model artifact, flag as a paper caveat / quaternion-plant future work), **all four descending stations and apogee stabilize with 0.00 m miss**. Old ct0/nb0 was [0 0 0 1 | 1 1 1 1]; new is [0 0 0 0 | 1 1 1 1] — the aggregate drop comes from the ascending half's ct/nb variants.
- Consequences now recorded in CLAUDE.md/runbook/delta plan: **every ballistic-dependent artifact is seeding-stale** (sat_on/off sweeps + TO figs, N=41 LUT log, promoted SO log + θ\*, envelope/fig 39, apogee test); `j3_lut_regen.m`'s reference-row injection assert is invalid post-fix (the next regen must re-simulate all 41 rows — no free injection); the §8 step-1 regeneration therefore carries B4 + the C1/B7/C5 schema extension together, as intended by the freeze-point gate. The station asymmetry is also a strong prior for W2: the hand-off window likely wants to shift **late**.

**Gate 1 follow-on — plan-2 eom2 remediation: EXECUTED (same day, author-approved).** The legacy `o` channel is removed from the ODE state (12-state `eom2`); `.trajectory` keeps 15 columns with 13:15 backfilled post-hoc as the derived attitude `[θ φ ψ]` (rad, NWU, the `ballistic_deploy_state` convention — exact-equality invariant verified over all 197 rows). Removal left the ode45 step grid **untouched**: the error control is an ∞-norm over components and the degree-scaled channel never bound it (rescaling to radians *did* bind — the 19.7 delta — which is why in-state unit conversion was rejected). Stored trajectories therefore reproduce **bit-exactly** (same grid, cols 1:12 delta exactly 0, apogee/impact identical), and the operating-point sweep reproduces the B4-fix numbers to `isequal` exactness (reach 0.292857, centroid, half_radius; 0 errored). eom2's exposure to dead-state scaling is permanently closed. **Tolerance tightening EXECUTED (same day, author decision):** RelTol 1e-8 / AbsTol 1e-10 (overridable via `launch.rel_tol`/`launch.abs_tol`) with a fixed 0.1 s dense-output `tspan` — decoupling accuracy from output density so `analysis.m`'s row-indexed envelope sampling keeps ~20 test points instead of silently inflating ~10×. Convergence verified against 1e-9/1e-11 (≤ 1.7e-6 m position, sub-µm impact quantities); the defaults had carried **~0.43 m true position error** (impact +0.25/+0.42 m, apogee +4.6 cm) — the earlier "~20 m admissible-grid spread" statement is hereby corrected: that figure was a row-misalignment artifact of comparing same-index rows on different time grids. Converged provisional operating point: **reach 0.300 (42/140), centroid (681.32, −196.33), 0 errored** — the ~0.4 m re-basing flips exactly one marginal trial vs the loose run (criterion-boundary sensitivity; the success metric's granularity is 1/140). Runtime cost: eom2 0.05 → ~0.5 s, called once per sweep — negligible.

**Gate 2 — w_y0: ruled a LAUNCH KNOB for now** (author, 2026-07-10); dispersions to be inserted later (Strategy D). B6 still governs any box widening (symmetric, physically argued bound; own transect, never folded into the Δ-shift batch).

**Gate 3 — p_target: ruled a GAUSSIAN distribution** (author, 2026-07-10). Working default per C4: isotropic, σ = 100 m, with a {50, 100, 200} m sensitivity row. W3 objective work unblocked pending the C1 schema.
