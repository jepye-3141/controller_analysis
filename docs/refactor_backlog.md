# Refactor backlog

Living backlog consolidating the 2026-07-12 release-pass deferred code/figure/log
refactors and the 2026-07-10 journal-readiness housekeeping items. Every item here
was verified **real** but deliberately left unapplied; unless noted, all are gated on
the pending post-bugsweep ballistic regeneration campaign, which is the trigger that
unblocks almost everything below. (Cross-references of the form `§1.x` point to the
judgment-calls section of the original 2026-07-12 release-pass audit and are not
reproduced here; the resolved/superseded items they name have already shipped.)

The single recurring blocker is **the whole-workspace `save()`**: both `analysis.m`
(`save("logs/analysis_log.mat")`, L2550) and `sweep_landing_centroid.m`
(`save("logs/trajectory_optimization_log<suffix>.mat")`, L710) persist the *entire*
workspace. Any refactor that introduces or removes intermediate variables changes
the saved-log schema — which is fine at a deliberate regeneration, but not while
exact/`isequal` log diffs are the verification instrument for the pending
post-bugsweep ballistic regeneration.

---

## 1. Deferred code / figure / log refactors

_Relocated verbatim from the 2026-07-12 release-pass audit §2. All items below were
verified **real** but not applied. Grouped by theme. Unless noted, the trigger is
**"after the post-bugsweep regeneration campaign lands"** and the verification recipe
is **"load the regenerated log, run old vs new in one workspace, `isequal`/pixel-diff
before deleting the old code."**_

### 1.1 `analysis.m` figure/computation block duplication

`analysis.m` is ~29% duplicated plotting scaffold. Each item is a separate loop-
or-helper consolidation; none changes numerics if transcribed correctly, but each
is a high-transcription-surface edit to the master driver with no test harness,
and each perturbs the whole-workspace log.

- **Figs 3–18 & 19–34: 32 near-identical 23-line heatmap blocks (`analysis.m:1060`).**
  Plan: add a trailing local function
  `distance_heatmap_fig(dmin, davg, dmax, title_template, case_label, fname, cmap, clims, hm_args)`.
  The two families need *different* title templates (`"%s euclidean distance between
  swarm members, %s"` vs `"Difference between %s euclidean distance between drones,
  and nominal separation, %s"`), so `case_label` alone is insufficient. Pass per-group
  args (group 1: `hot`, `clim [0,4]`, FontSize 14; group 2: `[flipud(jet);jet]`,
  `clim [-1 1]`). Drop the dead per-block `hold on`. Clear the spec vars before the
  L2550 save. Verify by byte-diffing all 32 PNG/EPS outputs from the cached workspace.

- **Fig 39 numerics-adjacent typography: FontSize 14-vs-10 divergence (`analysis.m:1450`).**
  This is the one figure-family item that is genuinely **numerics/pixel-touching and
  should be fixed regardless of the refactor.** The 48 delta-heatmap calls (figs
  19–34) pass only `'FontName','Times'` while the paired absolute-distance calls
  (figs 3–18) also pass `'FontSize',14`; `set_default_fonts()` cannot cover
  `HeatmapChart` (a standalone chart class), so figs 19–34 render at the 10 pt
  default. Confirmed on the Jul-11 regenerated PNGs. Plan: append `,'FontSize',14`
  to the 48 delta lines (or route both families through the §1.1 helper) **in the
  same figure refresh as the regeneration** so the typography change rides the
  physics change. Do **not** add an "intentional" comment — there is no evidence of
  intent.

- **Fig 36: 20 copy-pasted KE panels (`analysis.m:1901`).** Title/xlabel/ylabel/ylim
  sit *inside* the per-drone plot loops (10× redundant), the 10-name legend repeats
  16×, and vestigial `for i=1:1` loops survive on the four ballistic panels. Plan:
  build a panel spec (time vec, KE matrix, title, n_traces, optional ylim-reference)
  and render in one loop. **Transcribe the heterogeneous ylim references exactly** —
  panel 3 slaves to `step_smc_ke` (cSMC), panels 7/11/15 to LQR matrices, ballistic
  PID has *no* ylim (preserve the finding-18 autoscale comment). Legend driven by
  n_traces. Pixel-diff `figs/36`.

- **Settling-time: 3 near-identical blocks (`analysis.m:2229`).** Step (2231–2280),
  ballistic (2283–2340), uncertainty (2343–2392), each internally 4×-per-controller.
  The ballistic copy carries **dead multi-drone machinery** (max-over-drones on
  single-drone `T×1×3` data always returns index 1) and four identical aliases of
  `xf_cmd_ballistic`. Plan: add trailing `settling_time(interp_pos, xf, xi, tvec, tol)`
  reproducing the exact op order (max-abs-over-drones → normalize by that drone's
  `‖xf−xi‖` → latch first t ≤ tol). Call 12×, passing `interp_*_pos(end,:,:)` for
  step/uncertainty and `xf_cmd_ballistic` for ballistic. `isequaln` all three ts
  4-vectors before deleting.

- **Score plots: 48 Frobenius-ratio lines + 4 hand-assembled matrices (`analysis.m:1816`).**
  Plan: `delta_score = @(D) norm(D-ideal_distances,'fro')/norm(D+ideal_distances,'fro')`
  + `cellfun` over an explicitly ordered `{min;avg;max} × {lqr,smc,dsmc,pid}` cell
  array. `isequal` all four `*_score_mat` before deleting (guards against
  row/column permutation). **Note:** the *dead f8/spiral half* of this block was
  already deleted in the applied pass (its only consumers were commented-out
  heatmaps); only the live step/uncertainty consolidation remains.

- **Pairwise-percentage: same formula in 4 nested loops (`analysis.m:2180`).**
  `100*(a(i)-a(j))/(a(i)+a(j))` expanded in the KE-score and three settling-score
  loops; `conlabels` defined twice. Plan: `pairwise_pct = @(a) 100*(a(:)-a(:).')./(a(:)+a(:).')`
  (column-safe form), replace the 8 matrix builds (keep computing `ke_scores_f8`/
  `_spiral` — they persist into the log even though their plots are commented out),
  hoist the single `conlabels`. `isequaln` from the stored `unified_ke_*`/`ts` vectors.

- **Dead plotting generality — needs author decision (`analysis.m:606`).** The
  `3×10 offsets` matrix is only ever read at column 1 = `[0;0;0]`, and the four
  command plots + four apogee subplots are wrapped in degenerate `for i=1:1` loops
  (12 such loops total, incl. the fig-36 panels). **This one is gated on intent, not
  timing:** if the single-trace command plots are the intended final form, unwrap
  all 12 loops and delete `offsets` + the `+ offsets(k,i)` terms (and sync the
  CLAUDE.md Architecture bullet). If instead all 10 formation offsets should be
  shown, change the four command loops to `for i=1:10` and *keep* `offsets`. The
  author deliberately realigned all 10 columns to posout order on 2026-07-07, so
  the answer is not obvious — **ask before either edit.**

### 1.2 `sweep_landing_centroid.m` log bloat & dead fields

- **~560 MB redundant `sim_outputs` in every log (`sweep_landing_centroid.m:710`).**
  Measured on the Jul-11 sat_on log (635 MB): `sim_outputs` = 555.7 MB, a
  near-duplicate of the already-distilled `results(...).trajectory` traces. Plan:
  (1) *first* distill error forensics — `sim_outputs` is the only place per-trial
  `ErrorMessage`/`SimulationMetadata` live, so add
  `err_msgs = arrayfun(@(o) string(o.ErrorMessage), sim_outputs)` (or fold into
  `results(...).criteria`) before clearing; (2) `clear sim_outputs sim_inputs
  out_sim simIn simIn_template` immediately before L710; (3) keep the bare
  whole-workspace save (the documented `load(log,"out")` contract and the replot
  consumers survive); (4) update the header + CLAUDE.md size notes and regenerate
  *both* suffixed logs in one run so the pair shares a schema.

- **Write-only fields: `time_to_land`/`ct_name`/`nb_name`/`NB_idx` (`sweep_landing_centroid.m:262`).**
  Zero consumers repo-wide (verified across all `.m`, docs, and unzipped `.slx`);
  `time_to_land` also carries a misleading second definition vs TO_02's
  `settle_t_arr` (horizontal-only, no velocity gate). Plan: delete L262–272 incl. the
  `results(...).time_to_land` assignment; drop the field from the L141 struct init +
  L234 comment; shorten L78 and delete L79 (`vel_tol`); remove `ct_name`/`nb_name`
  init+assign and the now-orphaned `ct_names`/`nb_names` arrays; replace L294 with
  `[I_idx,K_idx] = ndgrid(1:n_deploy,1:n_ct,1:n_nb)` (drop `NB_idx`), verifying the
  `n_deploy×n_ct×n_nb` shape with a size assert.
  *Alternative:* if a per-trial settling time is wanted in the log, keep **one**
  definition (TO_02's) and have TO_02 consume it, eliminating the dual definition.

### 1.3 TO/SO/LUT figure-render duplication across driver+replot pairs

The paper's headline figures are rendered by two independently-drifted code paths:
the sweep/driver and the `replot_*` scripts. Consolidation is real maintenance
value but **figure-content-touching** (translucent-overlay draw order is iteration-
order-dependent, so unifying can shift pixels), so it needs render-and-diff
verification, not a static edit. All are `apply=false` for a last pass.

- **Landing-centroid + `landing_ratio` math, two implementations (`sweep_landing_centroid.m:311`).**
  `p=2` power-weighted centroid duplicated (sweep `313–341` vs `replot_TO_07.m:39–57`);
  `landing_ratio` in three forms (`accumarray` in the sweep vs triple loops in both
  replots), already diverged on edge cases (sweep: `tot_w==0`→warn+NaN; replot: hard
  `assert tot_w>0`). Plan: extract `[Xg,Yg,Rg,cx,cy,tot_w] = landing_centroid_heatmap(...)`
  **returning `tot_w`** (L346 branches on it) and keeping the NaN path + in-bounds
  asserts; extract the `accumarray` `landing_ratio` and call it from both replots;
  drop the replot's hard assert (the shared NaN path + drift assert already catch
  divergence). Verify: replots pixel-identical + one warm sweep `isequaln` on `out`.

- **TO_04 / TO_07 render blocks, three copies (`sweep_landing_centroid.m:535`,
  `replot_TO_04.m:15`, `replot_TO_04.m:33`).** ~95 + ~30 lines duplicated across the
  sweep and the two replots, with idiom drift (`isgraphics` vs `~isempty` legend
  guards; `accumarray` vs triple loops; `min-omitnan` vs growing array for
  `first_reach_deploy`). Plan: extract `render_TO_04(...)` / `render_TO_07(...)` that
  **recompute aggregates internally from the raw `results` struct** (so the replots
  keep working against the pre-criterion May-2026 log without saved intermediates);
  standardize on the sweep's vectorized idioms; keep `replot_TO_07`'s drift assert +
  cx/cy cross-check *in the replot*, outside the renderer. Account for the L710
  whole-workspace save (render-scope temps vanish from the log). *Cheap interim
  action available now:* add reciprocal "keep-in-sync" cross-reference comments at
  the four block tops (comment-only, safe in the release pass).

- **TO_01 / TO_06 overlay blocks, copy-paste twins (`sweep_landing_centroid.m:435`).**
  Identical stable-trial loop / `interp1`-to-ts / nebula coloring / colorbar
  boilerplate; only the metric, title/ylabel/ylim, and figure name differ. Plan:
  local fn `overlay_by_deploy(..., metric_fn, ..., ylims)` where `metric_fn` does
  its own interpolation — `@(tr) 0.5*constants_struct.m*sum(interp1(tr.time,tr.vel,ts).^2,2)`
  for TO_01, `@(tr) vecnorm(interp1(tr.time,tr.ctrl,ts),2,2)` for TO_06; `ylims=[0 5000]`
  for TO_01, `[]` for TO_06. Pixel-diff both figures.

- **SO_01 render block duplicated (`replot_SO_01.m:18`).** Lines 14–54 verbatim-
  duplicate `surrogate_optimize.m:93–131`, already drifted on a colorbar label glyph
  (`--` vs em-dash). Plan: fold into the **planned W1.1 `surrogate_optimize.m`
  rework** — delete the driver's inline render block, replace with a "render via
  `replot_SO_01.m` after" comment (matching the `j3_lut_regen.m` pattern). Do **not**
  auto-invoke `replot_SO_01.m` from the driver: its `clear; clc; close all` preamble
  runs in the caller's workspace and would wipe the verify block. If an automatic
  render is wanted, extract a shared `render_SO_01(...)` function called by both,
  standardizing on the `--` glyph that matches the published figure.

### 1.4 Cross-file math/parameter duplication (author sign-off)

- **`analysis.m` A/B literals — the `quad_lin_AB` variant (`analysis.m:54`).**
  Superseded by the applied `mkB` consolidation (§1.1 of the release-pass audit).
  Recorded only to note the alternative form a verifier proposed (a two-output
  `quad_lin_AB(K,m,g,Jxx,Jyy,Jzz)` local function) in case a future refactor prefers
  a named function over the anonymous builder. No action needed.

- **Trim-parameter helper (`j3_surrogate_refit.m:189`).** The launch *knobs* are
  centralized (`operational_launch.m`). The remaining duplication is the **trim
  block** (`alpha_0=2, beta_0=-0.5, x_0=y_0=z_0=0, t_max=300`) still hand-copied in
  the LHS-sample loops of `j3_lut_regen.m` and the superseded drivers.
  Plan (post-regen, standalone commit): add `base_launch_params(knobs)` returning
  the trim struct with disjoint knob fields merged on top (error on unknown fields);
  use at **all active sites**; bump `j3_lut_regen.m`'s `config_tag` (or assert no
  checkpoint exists) so a stale checkpoint can't resume across the refactor.
  *Low priority* — the trim values have never changed since inception.

- **`linear_nonlinear_comparison.m` hand-copied dynamics (`linear_nonlinear_comparison.m:57`).**
  `nl_dynamics` is a hand-copy of `system_dynamics.m` (already drifted — omits the
  `(kwt/kmt)*Jmp` gyroscopic terms, benign only because `Jmp=0`), and `lin_dynamics`
  hardcodes `constants.m`'s A/B. **This IS a math-heavy dynamics file → the standing
  conservatism rule applies → author sign-off required.** Safe last-pass action
  (comment-only, not yet applied): add a header note that both functions are frozen
  copies that must be re-synced if `system_dynamics.m` or `constants.m` changes. If
  the author approves delegation, prefer the local-struct form
  (`system_dynamics([],[],F,state,c)` with a 12-field `c`) over running `constants.m`
  inside the script (avoids clear-all ordering, the Simulink-bus/`lqrd` dependency,
  and console echoes).

### 1.5 Defensive-code footgun

- **`ballistic_success` silent skip paths (`ballistic_success.m:78`).** The `vel=[]`
  and `RotVel=[]` skip branches (clauses (c)/(d) pass vacuously) are never exercised
  today — both callers always pass both traces — so a future caller that omits
  `RotVel=` would silently drop the rotation clause and could report success on a
  tumbling trial. **Do not delete the skip paths** (they support re-scoring archived
  pre-rotvel logs, and the `*_checked` flags are persisted into the saved `criteria`
  schema). Minimal post-regen hardening: add a one-line
  `warning('ballistic_success:missing_trace', ...)` in each `else` branch — a no-op
  for every current call site, changes no numerics or saved content, and makes future
  misuse loud instead of silent.

---

## 2. Bug notes for the author (out of audit scope — not chased)

_Relocated verbatim from the 2026-07-12 release-pass audit §3. Surfaced by the
reviewers as possible correctness issues; the audit was a *quality* pass, so these
were noted, not fixed. Confirm before/during the regeneration._

1. **`analysis.m` step & uncertainty settling blocks still latch `xf` to each run's
   own endpoint** (`interp_*_pos(end,:,:)`) rather than the commanded target — the
   same defect class as bugsweep finding 16, which was fixed *only* for the ballistic
   block. Benign while every step run converges; a diverged run would earn a finite,
   competitive-looking settling time in fig-38 panels 1–2.
2. **Non-ballistic PID KE panels still slave `ylim` to 5× another controller's max**
   (`analysis.m:1933/1982/2031/2080`) — the finding-18 readability trap, fixed only
   for the ballistic panel.
3. **`analysis.m` never `addpath`s `Ballistics_Simulation-master/`** (the other
   drivers do) — errors at the `aero_constants` call unless the path was set
   externally. A trap for public-release users; documented in CLAUDE.md but not
   self-contained.
4. **`reachability_pct` holds a 0–1 fraction, not a percentage** (e.g. 0.593); the
   stats panel prints it verbatim. Units trap for readers; cross-file rename deferred.
5. Residual citation cleanups: "McCoy, 1998" survives in
   `Ballistics_Simulation-master/README.md` and `docs/BALLISTICS_REFERENCE.md`; the
   bib entry cites 1999 with no edition field.
6. The grace-window transient timings in `ballistic_success.m` (`t≈0.06 s … 3.8 s`)
   are from the 2026-07-03 pre-bugsweep campaign — re-measure from the 2026-07-11
   sat_on log before the paper's grace-window sentence is next re-derived.

Lower-severity latent traps (dormant at the current operating point): `eom2` Mach
clamp only guards `lut_C_D_0`'s high side; `cos_taoa` unclamped before `acos` in the
RHS; `sweep_landing_centroid` cross-track fallback comment says launch→impact but
computes origin→impact (identical only while `x_0=y_0=0`); `replot_TO_07`'s drift
assert fires spuriously on a legitimately-NaN centroid; `eom2` post-apogee `interp1`
errors if the impact event coincides exactly with a 0.1 s grid point.

---

## 3. Housekeeping (2026-07-10 journal-readiness audit §H)

_Still-open housekeeping items relocated verbatim from the 2026-07-10
journal-readiness audit §H._

- **H3:** `replot_*` scripts point at pre-criterion logs — `replot_TO_04.m`/
  `replot_TO_07.m` still load the pre-criterion May-2026
  `logs/trajectory_optimization_log.mat`.
- **H4:** stale May-21 `TO_06/07_sat_off` EPS copies in `docs/scitech-paper/figs/`.
- **H6:** `set_omega2_max.m` mutates source on disk (superseded if J1-W1 lands the
  bus-carried bound); pending J1-W1.
