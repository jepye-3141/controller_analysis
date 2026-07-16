# Plan J3 Runbook — Step-Zero Regeneration + Refit (staged 2026-07-10)

**Status: EXECUTED 2026-07-10 (same day, T0–T7 all complete, zero failures).** See §7 for the execution record and the §6 review findings. Headline: the pipeline validated end-to-end — T2 determinism check exact to 0.0 in all four quantities, ground truth at the area-proxy θ\* landed **9.4 m** from the (600, 0) target with **reach 0.779** (vs 92.0 m / 0.550 at the paper's operating point), and clamp mode's λ>0 optima are demonstrably imputation artifacts. **Recommendation: promote `area_proxy` — ACCEPTED by the author and executed same day (2026-07-10):** canonical `logs/surrogate_optimize_log.mat` is now the area_proxy refit (historical schema + `lambda=50` + refit extras + `promotion_note`; pre-criterion fit preserved in `logs/archive/`), `figs/SO_01` regenerated from it, and both replot headers updated. Nothing here touched the LaTeX paper. **Post-execution note (2026-07-10, later same session): the deploy-attitude fix (module-review finding B4) was applied and verified, making this run's LUT/SO artifacts and θ\* seeding-stale** — the W0 methodology, determinism checks, and clamp-vs-area_proxy conclusions all stand, but the numbers need one regeneration on the corrected seeding (fold in the review's schema extension; the reference-row free injection is no longer valid — re-simulate row 41). See the **Deferred rigor backlog** section below (relocated from the 2026-07-10 J3 module review) and CLAUDE.md (Ballistic simulation).

## 1. Decisions locked (author, 2026-07-10)

- **LHS box:** `p` widened `[−2, 2] → [−12, 12]` (the operating spin −8.379 was *outside* the old box, so the old GPs extrapolated at the paper's launch condition); other ranges unchanged.
- **Budget:** `N_LHS = 40`, `rng(0)`, plus one injected **reference sample** — the operating point (Vo=100, el=45, az=15, w_z0=1, w_y0=0.5, p=−8.379) copied from the 2026-07-08 sat_on sweep log at zero simulation cost (`is_reference=true`, entry 41).
- **Log handling:** pre-criterion logs archived **by copy** to `logs/archive/*_pre_criterion_2026-05-*.mat` before anything is overwritten; the regen then overwrites canonical `logs/centroid_lookup_log.mat` (so `surrogate_optimize.m`/`replot_LUT_01.m` load it unchanged).

## 2. Corrections to prior docs (found during staging)

1. The saved May-9 LUT log holds **N = 59**, not the script-default 20 assumed by CLAUDE.md and the 2026-07-09 strategies doc.
2. `half_radius` is NaN on **52/59 samples (88%)**, not 12/20 — the 1.2·max clamp dominated the hr GP's training signal, so the historical surrogate optimum is suspect *independent of* criterion staleness.
3. The clamp conflates two opposite NaN meanings: "profile wider than the grid" (reach>0) vs "nothing landed" (reach=0). Imputing WIDE for a zero-success sample is exactly wrong. The refit splits them (wide → 1.2·max, dead → 0) and offers `area_proxy` as the clean alternative.
4. Expect regenerated reach values **lower** than the old 0.36–0.88 spread (stricter criterion + honest ballistics + wider spin box). Zero-success samples at extreme p are plausible — that is data, not failure; the refit masks them out of the centroid GPs only.

## 3. Files staged (new; no existing script modified)

| File | Role | Writes |
|---|---|---|
| `j3_lut_regen.m` | Checkpointed, resumable LUT regeneration | `logs/centroid_lookup_log.mat` (N=41), checkpoint `logs/centroid_lookup_ckpt_j3.mat`, archive copies |
| `j3_surrogate_refit.m` | GP refit; `hr_mode` = `clamp` \| `area_proxy`; LOO-RMSE; λ-sweep [0,10,25,50,100,200]; reference-point LOO check; optional `do_verify` sweep | `logs/surrogate_refit_j3_<hr_mode>.mat` |

Both carry header docs; both refuse to run outside the project root. The regen checkpoint saves after **every** sample — re-launching the same command resumes; at most one in-flight sample is lost.

*Lint status:* the MATLAB connector was down during staging, so neither script has been through `checkcode`/execution yet. Any residual syntax slip will surface in the first seconds of T1 (before simulation starts) — if it errors immediately, paste the message back rather than debugging solo; nothing will have been overwritten at that point (archives happen first and are copies).

## 4. Task list

**T0 — Preflight (1 min).** From the project root. Exactly one MATLAB running (the sweep's `pgrep -c MATLAB_maca64` guard errors otherwise). Note: T6 overwrites the tracked `figs/LUT_01_*` files.

**T1 — Regenerate (≈4.5–5.5 h, resumable).**
```
matlab -batch "j3_lut_regen"
```
Expected console: two `archived …` lines (first run only), `reference sample stored … (reach=0.550, cx=564.4, cy=-84.8)`, then `LHS 1/40 …` through `LHS 40/40 …` (~6–8 min each warm; first ~15 min), ending `wrote logs/centroid_lookup_log.mat: N=41 entries`. **After any interruption, re-run the same command.** If it asserts "checkpoint config mismatch", the config block was edited — delete `logs/centroid_lookup_ckpt_j3.mat` only if you intend a from-scratch restart.

**T2 — Determinism spot-check (optional, ~6–16 min).** The sweep is deterministic, so re-simulating the reference params must reproduce the injected row exactly:
```matlab
ref = struct('Vo',100,'el',45,'az',15,'w_z0',1,'w_y0',0.5,'p',-8.379, ...
             'alpha_0',2,'beta_0',-0.5,'x_0',0,'y_0',0,'z_0',0,'t_max',300);
load_system("discrete_smc_swarm_single"); addpath("Ballistics_Simulation-master/");
o = sweep_landing_centroid(ref, false);
fprintf('reach %.4f (expect 0.5500)  cx %.2f (564.39)  cy %.2f (-84.80)\n', ...
    o.reachability_pct, o.p_centroid(1), o.p_centroid(2));
```

**T3 — Refit, historical treatment (~10–15 min, LOO dominates).**
```
matlab -batch "j3_surrogate_refit"
```

**T4 — Refit, area-proxy treatment.**
```
matlab -batch "hr_mode='area_proxy'; j3_surrogate_refit"
```

**T5 — Optional ground truth at θ\* (~6–16 min extra).**
```
matlab -batch "hr_mode='area_proxy'; do_verify=true; j3_surrogate_refit"
```

**T6 — Figure.** `matlab -batch "replot_LUT_01"` — now renders from the fresh canonical log (title will read N=41). This closes audit item H3 for LUT_01. Leave `replot_SO_01.m` alone until a refit winner is promoted to the canonical SO log name.

**T7 — Report back.** Paste the T1 tail (last few LHS lines + `wrote …`), the full T3/T4 consoles (LOO-RMSEs, λ-table, reference-point check), and T5 if run. I then do the post-regen review (§6).

**Failure modes.** Sweep asserts >5% errored trials → sample-level problem; report the sample index (its params are `ranges`-mapped `X(n,:)`). parpool startup failure → a second MATLAB is holding seats; close it and re-run. Rollback: the pre-criterion logs are preserved in `logs/archive/`.

## 5. Strategies review — 2026-07-09 doc, updated after staging

**Step zero** — unchanged as the gate; now one command (T1). Its "~5 h / N=20" figure described the script default, not the actual N=59 artifact.

**Strategy A (tighten the deterministic surrogate)** — *upgraded from "default next step" to load-bearing.* With 88% of hr training values clamp-imputed and the clamp's wide/dead conflation, the historical `λ·half_radius` term optimized mostly against an artifact. Staging already implements A's core: `area_proxy` mode, the λ-Pareto sweep, LOO-RMSE reporting, and the zero-success masking. Remaining A decision (post-regen, data in hand): whether `gp_reach` enters J as a term or constraint, or stays report-only. **Post-regen (2026-07-10): A vindicated.** On the fresh data the NaN rate fell to 8/41 (all wide, zero dead), yet clamp mode still failed qualitatively — its λ>0 optima chase the 8 imputed 660.3 m values to the p ≈ +11.8 box edge with w_z0 pinned at −1, while area_proxy's optima are stable and near-interior across λ = 10–200 and its hr GP is better even after normalization (LOO/std 0.55 vs 0.68). Ground truth at the area-proxy θ\* validated the whole chain (miss 9.4 m, reach 0.779, area_proxy predicted 415.7 vs actual 408.2). On the gp_reach decision: with area_proxy, the robustness term in J *is* an integrated-reachability measure, and the θ\* it finds carries high reach (0.70 predicted / 0.78 actual) without any explicit reach term — keeping `gp_reach` report-only is now defensible with data; a reach-floor constraint remains the fallback if a future p_target sits in a low-reach region.

**Strategy B (adaptive reachability classifier)** — premise strengthened: the stricter criterion + wider spin box should sharpen the feasible/infeasible boundary and likely produces zero-success regions. Still post-regen: the fresh map's reach distribution tells us whether uniform LHS is wasting samples near the boundary. Decision input: reach histogram + count of zero-success samples from T1. **Post-regen (2026-07-10): premise refuted — deprioritize.** Zero zero-success samples in the whole box (min reach 0.143 at the widened p extremes); reachability degrades smoothly with spin (corr(|p|, reach) = −0.70; mean 0.655 at |p| ≤ 8 vs 0.365 above) but never reaches an infeasible region. There is no boundary for a classifier to learn, and uniform LHS wastes nothing on dead zones. Revisit only if a future box extension (e.g. wider w_y0, Δt dimension) actually produces failures.

**Strategy C (target-weighted reachability Q)** — unchanged, still gated on an operational `p_target` distribution. One staging discovery: C needs the per-sample 2-D landing-ratio field, which the LUT log does **not** store (the refit-relevant `radial_profile` now is stored, but it is 1-D radial). If C is selected, `j3_lut_regen.m` grows a small knob to also store per-sample landing scatter (positions + ratios; a few kB/sample). Flagged, not implemented.

**Strategy D (stochastic outer loop)** — unchanged: deferred until dispersions are a stated requirement; plan J1 owns dispersion for *evaluation*.

**Strategy E (deployment point as decision variable)** — sequencing refined: going 6-D→7-D at N=40 thins global coverage. After the 6-D map validates (LOO-RMSE acceptable), prefer a **local augmentation** around θ\* (~10–15 extra samples over (Δt, Vo, el) near the optimum) over a global 7-D redraw — E's insight at roughly a third of the sample bill. `Δt = 0` must reproduce the 6-D result exactly. **Post-regen (2026-07-10): the local augmentation should also open the w_y0 box.** θ\* pins at w_y0 = +1.00 (its upper bound) for every λ > 0, and the J-slice along w_y0 is monotone into that boundary — the surrogate wants more +w_y0 than the box offers. Fold a w_y0 extension (e.g. [−1, 2]) into the same augmentation batch as Δt; alternatively the author may rule w_y0 a disturbance (not a launch-controllable knob), which dissolves the pin as a modeling question. Note the wall-clock economics improved 3–5×: warm sweeps measured 1.3 min/sample (visualize=false), so a 10–15-sample augmentation is ~20 min, not hours.

**New, from this staging** — the reference-point injection (free GP anchor at the operating point + pipeline validation) is worth keeping as a permanent pipeline feature; the honest quality metric at that point is its LOO prediction, which the refit prints.

## 6. Post-regen review I will run on your T7 report

Reference row intact and LOO-predicted sanely; reach distribution + zero-success census over the 40 samples; NaN-hr census (wide vs dead); LOO-RMSE per GP (rough acceptance: cx/cy ≲ 50 m — one inner-grid cell; reach ≲ 0.1); λ-table shape (does the Pareto knee justify a λ?); θ\* interior vs boundary-pinned (pinned ⇒ box or GP artifact — do not trust a pinned optimum); clamp-vs-area_proxy comparison → promotion recommendation for the canonical SO log; then updates to `plan_J3_targeting_delta.md` (W1/W2 made data-driven), the strategies assessment above, and memory.

## 7. Execution record + §6 review findings (2026-07-10)

Executed same-day from Claude Code, T0–T7, all exit 0. Consoles in `run_logs/j3_{lut_regen,t2_spotcheck,surrogate_refit_clamp,surrogate_refit_area_proxy,replot_LUT_01}_2026-07-10.log`.

**T1** — 41/41 entries, **0 errored trials in all 40 sweeps**, total sim wall **2.3 h** (well under the 4.5–5.5 h estimate: samples 1–6 paid rapid-accel warm-up at 14–20 min, samples 7–40 ran at **1.3 min each** — the prior ~6 min/sweep figure included visualization + a 200–400 MB workspace save that `visualize=false` skips). Reference row injected with the expected values; checkpoint retained at `logs/centroid_lookup_ckpt_j3.mat`.

**T2 (determinism)** — fresh sweep of the reference params reproduces the injected row **exactly**: `d_reach = d_cx = d_cy = d_half_radius = 0.0`.

**T3/T4 consoles (key lines):**

```
samples: 41 total, 0 zero-success (masked from cx/cy GPs only)
clamp mode: 8 wide-NaN -> 660.3 m, 0 zero-success-NaN -> 0
LOO-RMSE: cx 85.8 m, cy 55.7 m, reach 0.111, hr[clamp] 116.61 | hr[area_proxy] 48.07
clamp      λ=0: miss 0.0, interior (p=-7.96) | λ≥10: p→+11.8 (box edge), w_z0 pinned -1, hr 625-743 (> max finite 550.3 — imputation extrapolation)
area_proxy λ=0: miss 0.0, interior            | λ=10..200: Vo≈109.3 el≈40.1 az≈15.2 w_z0≈0.01 w_y0=1.00* p≈+1.2, miss 0.6→10.0 m, hr 415.5→416.5
reference LOO: (cx,cy) pred (566.4,-149.8) vs actual (564.4,-84.8); reach 0.513 vs 0.550; hr[ap] 233.0 vs 242.6
```

**T5 (ground truth at area-proxy θ\*, λ=50)** — 0/140 errored: actual centroid **(607.28, −5.95)** vs pred (602.13, −1.88) → **6.6 m** GP error, **9.4 m** actual miss vs the (600, 0) target; actual reach **0.779** vs pred 0.705; actual half_radius 511.8 m; actual area_proxy 408.2 vs pred 415.7 (−1.8%).

**T6** — `figs/LUT_01_trajectories_and_centroids.{png,eps}` regenerated from the fresh log (title N=41). Audit item H3 closed for LUT_01 (`replot_SO_01` untouched pending promotion).

**§6 verdicts:**

1. **Reference row** — intact (T2 exact) and LOO-predicted sanely (cy is the weak axis at 65 m error ≈ 1.2× its LOO-RMSE).
2. **Reach census** — min 0.143 / median 0.571 / max 0.914, mean 0.556; **zero zero-success samples**. The widened spin box degrades but never kills reachability: corr(|p|, reach) = −0.70.
3. **NaN-hr census** — 8/41, **all wide, none dead** (down from 52/59 = 88% pre-criterion; the stricter criterion pulls profiles below half-peak inside the grid). The wide 8 cluster at moderate-to-high +p with above-median reach — broad plateaus, exactly what area_proxy measures honestly and the clamp inflates.
4. **LOO-RMSE acceptance** — cx **85.8 m > 50 m: not met**; cy 55.7 m and reach 0.111 marginal; normalized (RMSE/std): cx 0.43, cy 0.36, reach 0.57, hr[ap] 0.55 (vs hr[clamp] 0.68). The 6-D global fit at N=41 is informative but coarse — **global LOO overstates error near the optimum basin** (θ\* ground-truth errors: 6.6 m centroid, 0.074 reach). More samples (W1.3) or the Strategy-E local augmentation before quoting GP accuracy as a paper-grade number.
5. **λ-table shape** — area_proxy Pareto is **flat past λ ≈ 10** (miss 0.6→10.0 m, hr 415.5→416.5 over λ = 10–200): the knee justifies any λ in [10, 50]; keep λ=50 for continuity and report the sweep. Clamp's λ>0 rows are artifact-driven — do not use.
6. **θ\* pinning** — interior in 5 of 6 dims; **w_y0 pinned at +1.00** with the J-slice monotone into the bound (genuine box limitation, not fit noise). Ground truth still validated at the pinned point, so the cost is opportunity, not error. Action folded into Strategy E (widen w_y0 in the local augmentation, or declare it a disturbance).
7. **Promotion** — **area_proxy, unambiguous**: stable near-interior optima vs clamp's imputation chase; better normalized hr LOO; finite everywhere without an imputation policy; verified at θ\*; and the verified optimum **beats the paper's operating point on both axes** (miss 92.0 → 9.4 m, reach 0.550 → 0.779 — the operating point was not target-optimized, but that is precisely the pipeline's claim: given a target, it finds a better launch condition). **Promotion executed 2026-07-10 (author decision, same session):** the canonical SO log is the area_proxy refit — schema = historical vars + scalar `lambda=50` (whose λ-entry matches the saved `theta_best`) + `lambdas/lam_results/loo/ok_centroid/hr_mode/out_v/promotion_note`; `gp_hr`/`y_hr` are area_proxy units, not meters of half_radius. `figs/SO_01_predicted_J_slices` regenerated (θ\* interior in all six slices, p axes span the new [−12, 12] box) — audit item H3 now closed for SO_01 as well. Caution recorded in CLAUDE.md: re-running `surrogate_optimize.m` as-is would overwrite the promotion with a clamp-mode fit; port area_proxy into it (W1.1) or keep using `j3_surrogate_refit.m`.

---

## Deferred rigor backlog (from the 2026-07-10 J3 module review)

The 2026-07-10 J3 optimization-module rigor review (four independent reviewers + an orchestrator numerical integrity check of the two canonical logs) left the still-open items below; they are relocated here as their living home. The executed foundation is fit to *search* with — its headline claim (θ\* verified **9.4 m** / reach **0.779** vs operating point **92.0 m** / **0.550**) is ground-truth-vs-ground-truth and stands — but it is **not yet fit to quote as an emulator** (cx LOO **85.8 m > 50 m** acceptance, no posterior-σ reporting, the hr target incommensurable across samples). Author decision gates already resolved (2026-07-10): **B4 eom2 attitude fix APPLIED + VERIFIED**; **w_y0 ruled a launch knob** (B6 still governs any symmetric, physically-argued box widening — own transect, never folded into the Δ-shift batch); **p_target ruled an isotropic Gaussian** (default σ = 100 m, {50, 100, 200} m sensitivity). The remaining open work:

- **W2 / B1 — respecify Δt as an additive arc-length window shift Δs at `sweep_landing_centroid.m`.** The sweep never deploys at apogee: it extracts **eight** deploy states by linear interpolation in *arc length* at 20–80% of the trajectory (`deploy_arc = linspace(0.20*arc_3d(end), 0.80*arc_3d(end), 8)`), each with its own position/velocity/rate/attitude; `apogee_idx` is never referenced in the sweep. So `t_deploy = t_apogee + Δt` is ill-posed against this pipeline. **Action:** respecify the 7th coordinate as an additive arc-length window shift Δs applied at `sweep_landing_centroid.m:86` (`deploy_arc + Δs`), one code path — Δs = 0 is algebraically a no-op, giving bit-exact W0 identity and optimizer continuity for free (a time-based interpolation breaks exactness; a `Δt==0` branch breaks continuity). Reframe the paper question to "where along the ballistic arc should the hand-off window sit." Correct the premise in all three planning docs.

- **W3 / C1 — per-trial schema extension in the sweep return + LUT rows, then a field-complete re-run.** The required input R(x|θ) does not exist in any artifact: the 40 landing points, per-point success ratios, and per-trial `stable` labels die with the function workspace (`visualize=false` returns before the whole-workspace save; LUT rows carry only summary fields, no per-trial landing coordinates/labels/traces — numerically confirmed). **Action:** extend the sweep's return struct with `landing_xy (40×2)`, per-point `ratio/total/success`, and per-trial `(i, k, nb, tgt, stable, final_xy, effort scalars)` — ~10 KB/row — together with the B7 per-station `deploy_ratio` (8-vector) + per-trial criteria extrema (`max_speed`, `max_rot_post`, `final_miss`, `t_end`) and the C5 per-trial effort scalars (∫‖u‖dt, ∫‖u‖²dt) — then re-run the 41 sweeps (~1–1.5 h warm). **Sequence before the W2 augmentation** so those samples are born field-complete.

- **A1 — recompute the half-radius target on a common support, refit `gp_hr`, confirm θ\* stability.** `area_proxy` currently integrates over a per-sample, θ-dependent support (the `r_bins` outer edge spans 286.8 → 852.7 m across the 41 rows, ~3× spread) measured from a per-sample center, with hull-exterior / empty-annulus NaN bins dropped — the 41 training values are not strictly commensurable, and for the 8/41 "wide" samples the integral is grid-truncation-limited so `−λ·hr` can partially reward *range* rather than robustness. **Action:** recompute the hr target on a common fixed support `R_common = min_n max(r_bins_n)` by `interp1` of the *stored* profiles (no resimulation, minutes), optionally 2πr-weighted for a true area; refit `gp_hr`; confirm θ\* stability; state the final definition (center, support, coverage weighting) explicitly in the paper.

- **A2 — report posterior variance (σ) and run the P1–P4 probes.** The pipeline is pure posterior-mean optimization; posterior variance is never computed or reported anywhere (no `[pred, sd] = predict(...)` in either script), yet the symptom appeared three times (clamp λ>0 chasing imputed values to the box edge; θ\* pinning w_y0 at its bound on a monotone slice; the above-training-max hr prediction at θ\*). **Action (probe placement for W1.4):** P1 the λ=0 optimum (pure-targeting end of the front); P2 the maximum-posterior-σ point within the θ\* basin / along the front; P3 θ\* perturbed ±1 ARD length-scale in its most sensitive dimension (launch tolerance — operationally meaningful); P4 (budget permitting) one maximin point as an unbiased global check. Score each as *held-out* prediction ± σ vs actual before ingesting into training; report the table.

- **A4 — raise N toward the n≈10d guideline via a maximin/Sobol infill conditioned on the existing 40 (no LHS union).** N=41 in 6-D is below n≈10d, and the planned "+40 samples" must **not** be an independent-`lhsdesign` union (preserves neither stratification nor maximin distance). **Action:** greedy maximin selection from a Sobol candidate pool *conditioned on the existing 40* (~20–25 global points) + the ~10–15 θ\*-local points already planned — fixes global cx accuracy and accuracy-where-claimed simultaneously at ~1 h of sim. `j3_lut_regen.m` hard-asserts `N_LHS==40` and the locked ranges against its checkpoint, so the infill needs a **new driver**, not an in-place edit.

- **A5 — capture fmincon diagnostics: ≥64 starts, exitflags, basin statistics.** The optimizer `exitflag` is discarded (`j3_surrogate_refit.m:131`); a start that exhausts `MaxFunctionEvaluations=500` is indistinguishable from a converged one, and only the winning (θ, J) per λ is kept. Surrogate evals cost milliseconds. **Action:** ≥64 starts, capture exitflags, cluster terminal points, report basin statistics (one-line paper credibility statement). Add λ ∈ {2, 5} — the Pareto knee between 0 and 10 is unresolved.

- **E1 — add the `allow_overwrite` interlock to `surrogate_optimize.m`.** Its only in-file staleness comment (":13 — saved log is pre-criterion, stale") is now **false** (the LUT log it loads is current); an as-is re-run passes the `isfinite` assert on the zero-dead log, silently re-imputes the 8 wide NaNs *in meters*, and overwrites both `figs/SO_01` and the canonical SO log *before* the verify gate can error. Recovery exists (re-promotion from the refit log), so MAJOR not BLOCKER. **Action:** ~5-line interlock (refuse to save over a canonical log whose `hr_mode` exists and ≠ "clamp" without an explicit `allow_overwrite`), fix the false comment, add a header warning — subsumed by, not duplicating, the W1.1 area_proxy port.

- **E5 — commit the 2026-07-10 promotion script as `j3_promote_so_log.m`.** The promotion (area_proxy refit → canonical SO log) was performed by a script run from the session's temp area — grep confirms no `*promote*` script in the repo — and re-promotion will recur on every future refit. The refit log omits the scalar `lambda` and `promotion_note` the canonical schema carries. The exact script (which already asserts archive-exists, `hr_mode`, `out_v` presence, and θ_best/λ=50 consistency) is preserved verbatim here; commit it as `j3_promote_so_log.m`:

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

- **Schema-extended regeneration — the single re-run that unblocks W3 / D1 / B2.** Run **one** schema-extended regeneration (~1–1.5 h warm; new driver per A4/E2/E8/C7 with a `schema_version` in the config fingerprint, timestamped archives, and a `provenance` struct): the C1 per-trial fields + the B7 station/extrema fields + the C5 effort scalars, with the B4 attitude fix in. This single run unlocks W3 prototyping, the legitimate Strategy-B check (D1 — the inner deploy×cross-track×neighbor boundary exists in every sweep, reach 0.143–0.914, so ~12–120 failed trials per grid), and B2 re-scoring without resimulation, and it provides the free Δs = 0 anchor (the T5 verification sweep, centroid (607.3, −6.0), reach 0.779). Do not run three separate re-runs — fold all schema extensions and the B4 fix into this one.
