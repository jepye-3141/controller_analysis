# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# ATLIS Sims

Quadrotor UAV swarm control comparison framework. Compares LQR, continuous SMC, discrete SMC, and PID controllers across step, figure-8, spiral, and ballistic stabilization trajectories using Simulink models. Includes a 6-DOF 120mm mortar ballistic simulation (McCoy Ch. 9) used to generate realistic deployment initial conditions.

**Branches (local).** `conference-dev` -- AIAA SciTech paper state; the LUT / GP-surrogate targeting modules were removed here. `journal-dev` -- ongoing journal work; carries the full targeting pipeline (`j3_lut_regen`, `j3_surrogate_refit`, `surrogate_optimize`, `replot_LUT_01`, `replot_SO_01`, `deprecated/centroid_lookup_table`). `release` -- public default / PR branch. CLAUDE.md is gitignored, so this single copy is shared across all branches; a module it describes as "journal-dev" is absent on `conference-dev`.

## Running simulations

No build/lint step -- everything runs inside MATLAB.

- Launch MATLAB from the project root with `Ballistics_Simulation-master/` on the path -- `analysis.m` and `trajectory_optimization.m` call `eom2()`/`aero_constants()` without `addpath`.
- `constants.m` must run before any simulation -- it defines physical parameters and computes the discrete LQR gains (`Kp_d`, `Ki_d`) used by the Simulink models. `analysis.m` calls it at the top.
- Full regeneration: run `analysis.m` from the project root. Runs every Simulink sim and writes `figs/1..39_*.{png,eps}`. Long-running; a prior workspace is cached in `logs/analysis_log.mat` (~82 MB) and can be `load`ed to skip re-simulating.
- Saturation comparison: run `trajectory_optimization.m` -> calls `sweep_landing_centroid(_, true)` twice (saturation on and off), then `plot_saturation_comparison`. Writes suffixed `figs/TO_01..TO_07_sat_{on,off}` + `logs/trajectory_optimization_log_sat_{on,off}.mat`, and `figs/SAT_CMP_summary`.
- Targeting (journal-dev): `j3_lut_regen.m` regenerates canonical `logs/centroid_lookup_log.mat` (N=41 LHS coverage, checkpointed/resumable); `j3_surrogate_refit.m` fits the GP surrogate (set `hr_mode`/`do_verify` first). Historical one-shot: `surrogate_optimize.m` (writes `logs/surrogate_optimize_log.mat` + `figs/SO_01`).
- Re-render figures without re-simulating: `replot_TO_04.m`/`replot_TO_07.m` (TO log), `replot_LUT_01.m` (LUT_01), `replot_SO_01.m` (SO_01).
- Ballistic sim alone: `Ballistics_Simulation-master/Mortar_Sim.m` writes `logs/ballistic_log.mat`.

Required toolboxes: Simulink, Control System (`lqrd`, `ctrb`, `obsv`, `ss`), Navigation (`eul2rotm`, `minsnappolytraj`), Optimization (`fmincon`), Statistics and Machine Learning (`lhsdesign`, `fitrgp`), Symbolic Math (deprecated scripts only), Aerospace (`gravitywgs84` in `aero_constants.m`).

## Entry points

- `analysis.m` -- Master driver (~2040 lines). Runs every swarm sim and builds figures `1..39`. Per-case `StopTime`: STEP 50 s, F8 150 s, SPIRAL 250 s, apogee single-drone 100 s, envelope trials 30 s (all `SimulationMode='Rapid'`). Figures `1..34` are trajectory/distance/heatmap plots; `35` is a formation delta-distance score table, `36` per-controller kinetic-energy panels, and `37`/`38` are pairwise kinetic-energy and settling-time **score matrices** (`100*(a_i-a_j)/(a_i+a_j)`); `39` is the ballistic envelope.
- `trajectory_optimization.m` -- Saturation ON/OFF comparison driver. Builds `base_params` from `operational_launch()`, splits into `p_on` (`saturation_on=true`) and `p_off` (`saturation_on=false`, the naive per-rotor-clipped baseline), calls `sweep_landing_centroid(_, true)` on each, then `plot_saturation_comparison`. Wired to `dsmc_constraints.m` via `discrete_smc_swarm_single.slx`; both arms scored post-hoc by `ballistic_success.m`. Regenerated 2026-07-11 on the retuned Vo94/el64 physics; the paper cites the result -- **sat 32.9% (46/140) vs the clipped baseline 7.9% (11/140)** (abstract, Table 4). See `docs/paper_code_agreement_saturation.md` + memory `project_regen_2026-07-10_post_bugsweep` for the campaign history.
- `sweep_landing_centroid.m` -- Reusable `out = sweep_landing_centroid(traj_params, visualize)`. Runs the parsim deploy x cross-track x neighbor dSMC sweep (8 deploy x 5 cross-track `[0,+-100,+-200]` x 4 neighbor `[-1,0,1,2]` = 160 max, ~140 valid after boundary exclusion) on a **12-worker** pool in rapid-accelerator mode (`StopTime=60`/trial), then computes the power-weighted landing centroid (`p_centroid=2`) and a 20-bin radial reachability profile. Returns `out` with `p_centroid` (`[cx;cy;0]`), `reachability_pct`, `radial_profile` (`r_bins, mean_ratio, peak_ratio, half_radius`), `ballistic_solution`, `n_errored`. `visualize=true` renders `figs/TO_01..TO_07` + saves `logs/trajectory_optimization_log.mat`; `visualize=false` returns silently. Optional `traj_params.saturation_on` and `.label` (suffixes figures/log). Per-trial `stable` scored by `ballistic_success` (per-condition diagnostics in `results(...).criteria`); degrades gracefully (warns, NaN centroid/half_radius) when zero trials succeed. Per-station deploy seeds built by `ballistic_deploy_state.m`.
- `j3_lut_regen.m` (journal-dev) -- Current LUT driver. Checkpointed/resumable LHS regeneration (N_LHS=40, `rng(0)`, over the 6-D box with `p in [-12,12]`) plus the operating point re-simulated as row 41, overwriting canonical `logs/centroid_lookup_log.mat` (`lookup` carries per-sample `radial_profile` + `is_reference`; `N=41`, `X` stays 40x6). Checkpoint `logs/centroid_lookup_ckpt_j3.mat` carries a physics/criterion `config_tag`; delete it for a from-scratch restart.
- `j3_surrogate_refit.m` (journal-dev) -- GP refit on the LUT log. Set `hr_mode` (`"clamp"`|`"area_proxy"`) and `do_verify` as workspace vars first. Masks zero-success samples from the cx/cy GPs; `area_proxy = trapz(r_bins, mean_ratio)` is the finite-everywhere half-radius target. Adds LOO-RMSE + a lambda-sweep. Writes non-canonical `logs/surrogate_refit_j3_<hr_mode>.mat`.
- `surrogate_optimize.m` (journal-dev) -- Historical one-shot: loads `logs/centroid_lookup_log.mat`, fits four GP surrogates (cx, cy, reach, half_radius; `fitrgp`, ARD Matern-5/2), runs 8-start `fmincon` (SQP) on `J(theta) = ||(cx,cy)-p_target(1:2)||^2 - lambda*half_radius` (**only the horizontal centroid and `half_radius` enter `J`**; `gp_reach` fit but unused, `p_target(3)` ignored). Defaults `p_target=[600;0;0]`, `lambda=50`. Writes `logs/surrogate_optimize_log.mat` + `figs/SO_01`. The canonical SO log is the promoted area_proxy refit, so `gp_hr`/`y_hr` are in **area_proxy units, not half_radius meters** -- re-running this script as-is would regress the promotion. See `docs/gp_surrogate_recipe.md`, `docs/deterministic_optimization_approach.md`.
- `deprecated/centroid_lookup_table.m` (journal-dev) -- Original LHS driver, superseded by `j3_lut_regen.m` (pre-J3 config: `p in [-2,2]`, N=20, no per-sample `radial_profile`, no checkpointing). Do not run.
- `constants.m` -- Physical parameters, linearized A/B/C/D, discrete LQR gains at 50 Hz (`lqrd(..., 1/50)`; Q diag-normalized by its 2-norm, `R=eye(4)`). Packs scalars into `constants_struct` and builds Simulink Bus `constants_struct_bus`. Bus fields: `g, l, Jmp, Jxx, Jyy, Jzz, kmt, kwt, dt, K, m, m_uncertain, saturation_on, unconstrained`. `m_uncertain` is separate from `m` for uncertainty injection; `saturation_on` (default `true`) gates the Omega^2 allocation in `dsmc_constraints.m`; `unconstrained` (default `false`) makes it delegate to `dsmc_no_constraints.m`. Echoes ctrb/obsv rank deficits to the console.
- `Ballistics_Simulation-master/Mortar_Sim.m` -- Standalone 6-DOF mortar sim entry point.
- `deprecated/smc_discrete_formulation.m` -- Standalone dSMC prototype; not called by `analysis.m`, does not currently parse. See `deprecated/README.md`.
- `linear_nonlinear_comparison.m` -- Sanity check comparing linearized vs nonlinear quadrotor dynamics.
- `deprecated/lqr_linearization_and_path_planning.m` -- Trajectory-planning experiments (BVP, multiple shooting, RRT*, symbolic time-varying LQR); not called, does not parse. See `deprecated/README.md`.

## Helper / utility scripts

Called by the entry points above (or run manually); not simulation drivers themselves.

- `operational_launch.m` -- **Single source of the operational mortar launch.** Returns the 12-field launch struct (`Vo=94, el=64, az=15, w_z0=1, w_y0=0.5, alpha_0=2, beta_0=-0.5, x_0=y_0=z_0=0, t_max=300, p=-8.379`). Consumed by `trajectory_optimization.m`, `analysis.m`, `j3_lut_regen.m` (journal-dev), `Mortar_Sim.m`. `az` is an off-vertical-plane angle (McCoy sec 9.3), NOT a compass heading; `alpha_0`/`beta_0` are pointing-vs-velocity offsets added to `el`/`az`. Retune the launch HERE only.
- `export_figure.m` -- Centralized figure writer `export_figure(name_base, opts)`: emits both `name_base.png` (ContentType `image`) and `.eps` (`vector` by default; pass `EPSContentType="image"` for surf/heatmap figures like TO_07) at `Width=2000, Height=1400, Resolution=300, Padding=40`. Use instead of raw `exportgraphics()`.
- `set_default_fonts.m` -- AIAA-style `groot` defaults (Times; 16 pt axes/text, 18 pt title, 14 pt legend/colorbar; 1.25 pt lines; grid alpha 0.25). Call at the top of any figure-producing script.
- `ryg_cmap.m` -- Red->yellow->green reachability colormap `ryg_cmap(n_half)` (0=red, 0.5=yellow, 1=green; default 64x3). Shared by all TO/LUT/SO reachability figures.
- `plot_saturation_comparison.m` -- `plot_saturation_comparison(out_on, out_off)` builds the saturation ON-vs-OFF summary (2x2) -> `figs/SAT_CMP_summary.{png,eps}`. Takes the two `sweep_landing_centroid` output structs directly (not the `.mat` logs).
- `ballistic_success.m` -- STABILIZE success criterion `[success, info] = ballistic_success(t, pos, vel, target_xy, v0_norm, expected_T, opts)`: full duration (blowup guard never tripped) + final XY <= `ReachTol` (10 m) + velocity ratio <= `VelRatioMax` (2) over the whole trace + rotation-rate ratio <= `RotRatioMax` (2, vs `RotRefNorm=||[pi pi pi]||`) for `t > RotGraceT` (default **1 s**, the separation-transient grace window). The rotation trace is passed via `RotVel=` (the raw `rotvelout` from the four `*_single` models -- the **Euler-angle rate** `[thetadot phidot psidot]`, not body rates); the function handles the shape internally (`[3x1xT]` lqr/dsmc vs `[Tx3]` pid/smc, via an `ndims==3` squeeze). All bounds are name-value configurable. Used by `analysis.m` (envelope) and `sweep_landing_centroid.m`.
- `ballistic_deploy_state.m` -- Deploy-seed builder `[xi, att] = ballistic_deploy_state(v, h, r, pos)`: maps ballistic trajectory slices (NWU) to the drone 12-state seed `xi = [vel_body; rotvel_body; theta; phi; psi; pos]`. Attitude from the **pointing vector** (cols 7:9, renormalized): body x-axis along `r`, zero roll (`psi=atan2(r2,r1)`, `theta=asin(r3)`, `phi=0`); velocities/rates map world->body via `M.'`. `rotvel0 = -(M.'*h)` -- the `-1` accounts for `det(M)=-1` (angular velocity is a pseudovector; bugsweep fix). Callers: `sweep_landing_centroid.m`, `analysis.m`. Interpolate cols 7:9 and derive -- never interpolate the angle columns (`psi` wraps).
- `set_omega2_max.m` -- `set_omega2_max(val)` rewrites the `Omega2_max = ...;` literal inside `dsmc_constraints.m` on disk (regexprep + fwrite). Manual helper for the Omega^2 sweep; leaves the source file git-dirty at the last value set.
- `init.m` -- Rebuilds the bus object via `Simulink.Bus.createObject(constants_struct)` AND re-creates `constants_struct_bus` (the variable the models' bus ports reference). Run after `constants.m` when a model needs the bus regenerated.
- `system_dynamics.m` -- Standalone nonlinear 12-state quadrotor ODE `xdot = system_dynamics(A, B, u, x, constants)` (A,B unused; `u=[T Mx My Mz]` with `T` absolute). **It IS the simulated plant** -- `actual_discrete_smc_controller.slx` wraps it in a one-line MATLAB-Function chart, and `discrete_smc_swarm_single.slx` pulls that in as the "Leader" subsystem. Applies `u` with no saturation; the model's only guard is a termination chart stopping STABILIZE sims when `norm(vel)` or `norm(rotvel)` >= 1e5 -- a numerical-blowup guard, deliberately NOT the success criterion (that is `ballistic_success.m`, post-hoc). See `docs/paper_code_agreement_saturation.md`.

## Control functions as MATLAB files

Control logic is migrating out of Simulink MATLAB-Function blocks into standalone `.m` files the models call (easier to read/diff/review):

- `dsmc_no_constraints.m` -- The user's original discrete SMC, lifted verbatim from the Simulink block. Mathematical structure is frozen; the canonical baseline. Deliberately **unclipped** (the only truly-unconstrained code path), invoked via the `constants.unconstrained` dispatch in `dsmc_constraints.m` (the `analysis.m` envelope "nosat" arm).
- `dsmc_constraints.m` -- Plan A+ augmented dSMC; the file under active development.

Both have signature `u = fn(A, B, state, xd, x0, constants)` (`A`,`B` unused, kept for Simulink interface compatibility). Both read `g, l, Jmp, Jxx, Jyy, Jzz, dt` and `m_uncertain` (the plant mass, so mass-uncertainty injection flows through it); `dsmc_constraints.m` additionally reads `saturation_on` and `unconstrained`. Both use `persistent` state and clip the `s_z`/`s_psi` sliding variables to `[-2, 2]` -- this clip is canonical and load-bearing (removing it lands zero successful sweep trajectories; see memory `project_dsmc_clip_required`).

### dsmc_constraints.m structure

Plan A+ augments the canonical law with:

1. **Auxiliary state `xi` (4-vector)**, one channel per actuator (thrust, Mx, My, Mz). Modified sliding variable `tilde_s = s - D*xi` substitutes for `s`. Deficit-coupling matrix `D` has same-channel (`D11..D44`) + cross-channel (`D21`, `D31`) entries.
2. **Priority-weighted allocation in Omega^2-space**: closed-form sequential scaling with operational priority `Mx, My > T > Mz` (`priority_weighted_allocate()`, local function). Bounds `Omega2_min=0.0`, `Omega2_max=2` -- a hard-coded literal that `set_omega2_max.m` rewrites in place (see memory `project_dsmc_saturation_sweep`).
3. **Matched contraction**: `k_xi = [nuz; nu3; nu4; nupsi] * dt`.
4. **POST-saturation feedback**: `ukm1` stores the saturated command, so gyroscopic `Omegar` reflects actual rotor speeds and the deficit `delta_u = u_unc - u_bar` drives the auxiliary update.
5. **Master toggle `saturation_on`** (bus): when `false`, bypasses allocation/anti-windup (`delta_u=0`, `xi=0`, `tilde_s=s`) so the laws match `dsmc_no_constraints.m` -- but the returned command is still clipped (item 6).
6. **Hard per-rotor clip (STEP 12b, both branches)**: `u_bar` is mixed to Omega^2-space, clamped to `[Omega2_min, Omega2_max]`, remixed. Under `saturation_on=true` this is a **no-op** (a material change there indicates an allocation bug -- a useful regression property); under `false` it makes the OFF branch the naive per-rotor-clipped baseline. `dsmc_no_constraints.m` stays unclipped.
7. **Mode dispatch `unconstrained`** (bus, read before persistent state): when `true`, delegates wholesale to `dsmc_no_constraints.m` and returns its raw unclipped command (the envelope "nosat" arm).

Design source of truth: `docs/dsmc_plan_A_plus.md`.

## Architecture

- Quadrotor: 12-state linearized model. State `[vx vy vz wx wy wz theta phi psi x y z]`; control `[T Mx My Mz]` with `T` as delta from hover weight (`m*g`).
- LQR uses an augmented 15-state model with 3 integral states on position; discrete at 50 Hz (`lqrd(..., 1/50)`).
- dSMC controllers run at 50 Hz (`constants.dt = 1/50`, on the bus). A stale `dt=1/200` comment survives in `dsmc_constraints.m`; the working value is `1/50` (confirmed intentional).
- Swarm: 10 drones (1 leader + 9 followers) initialized with X-axis offsets `[0, 1, 2, 3, 4, 5, -1, -2, -4, -3]` m -- the model's `posout` drone order (leader index 1). Mirrored by the plotting `offsets` matrix and the `initial_positions` reference feeding the delta-distance heatmaps; both were realigned to `posout` order (2026-07-07 fix, t=0 artifact now 0.000 m).
- Trajectory cases are integer-coded: `STEP=1`, `F8=2`, `SPIRAL=3`, `STABILIZE=4`, set via the `simcase` workspace variable.

## Simulink model naming

PID does **not** follow the `_swarm` suffix convention -- `analysis.m` loads four swarm models with mixed names:

- 10-drone swarm: `lqr_swarm.slx`, `smc_swarm.slx`, `discrete_smc_swarm.slx`, `pid_redux.slx`.
- 1-drone ballistic-stabilization: `lqr_swarm_single.slx`, `smc_swarm_single.slx`, `discrete_smc_swarm_single.slx`, `pid_redux_single.slx`.
- Controller-only blocks: `{lqr,smc,pid_redux}_controller.slx`, discrete variants `discrete_lqr_controller.slx`, `actual_discrete_smc_controller.slx`, `discrete_pid_redux_controller.slx`, and `zoh_smc_controller.slx`.

All swarm models consume workspace variables `xi` (**12-element** initial state, plant order `[vel_body(3); rotvel_body(3); theta; phi; psi; pos(3)]`), `xf`, `xf_ballistic`, `simcase`, per-group `A*/B*` when running uncertainty, and gains `Kp_d`/`Ki_d`. They output `posout` (Nx3xT), `velout` (Nx3xT), `cmdout` (3xT) as timeseries. The four `*_single` models also log `rotvelout` (leader Euler-angle rates; Data shape `[3x1xT]` in lqr/dsmc, `[Tx3]` in pid/smc).

Quirk: `discrete_smc_swarm_single.slx` has its `PostLoadFcn` set to the string `'init.m'` -- invalid (callbacks are commands, should be `init`), so every `load_system`/open warns. Harmless because `constants.m` builds the bus before any sim runs.

## parsim wiring in sweep_landing_centroid.m

The dSMC sweep runs trials via `parsim` with `'TransferBaseWorkspaceVariables','off'`. Every base-workspace variable the model needs is explicitly `setVariable`'d. The **7 static variables** (`A`, `B`, `constants_struct`, `constants_struct_bus`, `dt`, `x0_step`, `xf`) are applied once to a template `Simulink.SimulationInput` (`simIn_template`), copied per trial; the **3 per-trial variables** (`xi`, `xf_ballistic`, `simcase`) are set on each copy. If you edit `discrete_smc_swarm_single.slx` to reference a new base-workspace variable, add it to the static `setVariable` block -- else the sweep fails per-trial. Model **outputs** need no wiring (a new ToWorkspace block like `rotvelout` just appears on each `SimulationOutput`). Audit deps via `Simulink.findVars('discrete_smc_swarm_single')`.

## Parametric uncertainty

`analysis.m` first runs a STEP trajectory under mass/gain uncertainty. Leader and three follower groups each get their own `{A, B}`:

- Leader: `mL=1.6 kg`, `KL=0.8`.
- Followers: `m1=0.6 / m2=1.2 / m3=1.0 kg`; `K1=0.7 / K2=0.6 / K3=0.5`. The 10-drone assignment is `[mL; m1; m1; m2; m2; m2; m2; m2; m3; m3]`.

After this section the workspace is reset to nominal `m=0.8 kg` before the nominal STEP, figure-8, spiral, and ballistic sims.

## Ballistic simulation

Located in `Ballistics_Simulation-master/` (tracked on all branches, with commit history, despite the `.gitignore` folder entry). 6-DOF with a **12-state ODE**: `[velocity(3), angular rate h(3), pointing vector(3), position(3)]` (the legacy integrated Euler-angle channel was removed -- plan-2 remediation). The output `.trajectory` keeps **15 columns**: cols 1:12 are the NUE->NWU-rotated solver state; cols 13:15 are **backfilled post-hoc with the derived attitude `[theta phi psi]`** (rad, NWU; body x along the pointing vector, `phi=0` -- the `ballistic_deploy_state.m` convention). Internal frame is NUE. `mortar_propagate()` lives in `eom2.m` (filename != function name). `eom2(launch, env, visualize)` returns `ballistic_sol` with fields `time, trajectory (Nx15, NWU), alpha, beta, total_aoa, apogee, apogee_idx, impact_time, impact_range, impact_crossrange`. `aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx')` takes **two** file args.

**Gotchas.** (1) Interpolate attitude by interpolating cols 7:9 (pointing vector) and deriving via `ballistic_deploy_state.m` -- never interpolate the angle columns (`psi` wraps at +-pi). (2) State cols 4:6 = `h = H/I_y` (rad/s); recover spin as `p = (I_y/I_x)(h.r)`. At the operating `p=-8.379` the seeded `|omega|=0.825 rad/s` is an accepted approximation, not the physical axial rate. (3) The exported `.alpha`/`.beta` are velocity **direction-cosine angles from the NUE axes, NOT AoA/sideslip** (launch "alpha" ~ 46.9 deg vs true ~ 2 deg) -- use `.total_aoa` for the true angle of attack.

**Corrected physics (bugsweep 2026-07-10; all 19 findings fixed + verified, re-proven by the 2026-07-12 E2E audit).** The pre-fix chain flew a physically impossible 391 m apogee (eom2 lift ~V x /70-100 too strong, Magnus missed the `S` factor, Coriolis had a deg/rad + azimuth error, the deploy seed realized a *mirrored* tumble); corrected, the original Vo100/el45 launch drops to 234 m and the **operational Vo94/el64 arc** is apogee ~332.6 m, flight ~16.5 s. Tolerances RelTol 1e-8 / AbsTol 1e-10 with a fixed 0.1 s dense-output `tspan` (~200 rows, preserving `analysis.m`'s row-indexed envelope sampling density). Full record: `docs/2026-07-10_ballistic_bugsweep.md`, `docs/2026-07-12_ballistic_e2e_verification.md`; memories `project_ballistic_bugsweep_2026-07-10`, `project_eom2_h0_crossproduct_fix`, `project_b4_deploy_attitude_fix`.

**Ballistic-dependent results were regenerated 2026-07-11** on the retuned Vo94/el64 physics -- the sat sweeps / TO figs, the N=41 LUT log + promoted SO log + theta*, the envelope / fig 39, and the figures -- and the paper cites the current numbers (memories `project_release_pass_audit_2026-07-12`, `project_ballistics_e2e_audit_2026-07-12`; remaining paper-prose fixes are tracked in `docs/2026-07-12_ballistic_e2e_verification.md`). A further re-regeneration is pending only to fold in sub-mm post-audit code fixes (behavior-neutral to <0.016%) and the deferred refactors in `docs/2026-07-12_release_pass_audit_backlog.md`; it will not materially move the numbers.

`analysis.m` uses the mortar sim two ways: **(1)** extract the apogee state -> single-drone stabilization test (`*_single.slx`, 100 s); **(2)** envelope sweep -- every 10th trajectory sample -> 30-s stabilization per controller -> record successes in `ballistic_envelope_{lqr,pid,smc,dsmc_nosat,dsmc_sat}`. The dSMC single model runs twice per sample (nosat: `unconstrained=true` -> `dsmc_no_constraints.m`; sat: `saturation_on=true` -> Plan A+ + clip). All five arms scored post-hoc by `ballistic_success`. All four single models carry the same 1e5 numerical-blowup termination chart (STABILIZE only). Current envelope masks (17 test points, 2026-07-11 retuned-physics regen): LQR 5/17, PID 0/17, cSMC 0/17, dSMC-nosat 6/17, **dSMC-sat 8/17 -- the best arm, beating even the unconstrained nosat arm (which explodes on the hard deploy)**; see fig 39.

## Conventions and outputs

- Figures: written by `export_figure(name_base, opts)` -- both a PNG and an EPS at `Width=2000, Height=1400, Resolution=300, Padding=40`. Naming prefixes: `1..39_*` = `analysis.m` (`0_ballistic_trajectory` is written by `eom2.m`); `TO_01..TO_07` = `sweep_landing_centroid` (emitted as `_sat_on`/`_sat_off` pairs by `trajectory_optimization.m`); `SAT_CMP_summary` = `plot_saturation_comparison.m`; `LUT_01` = `replot_LUT_01.m`; `SO_01` = `surrogate_optimize.m`; `POC_*` = `proof-of-concept/`. The numeric-to-plot mapping is in `docs/README.md`.
- TO_04 landing-reachability: magenta colorbar step marks exactly-zero success ratio; bolds the trace from earliest deploy to first reach. TO_07 reports a power-weighted centroid (`p_centroid=2`) over a `'natural'` `scatteredInterpolant` of landing-success ratios (centroid is method-sensitive: `'linear'` shifts it ~1 m).
- `.slxc` files and `slprj/` are Simulink build cache -- gitignored, regenerate on next run. `*.asv` are MATLAB autosaves -- ignore.
- Saved-workspace logs live under `logs/` (gitignored; pwd = project root expected):
  - `logs/analysis_log.mat` (~82 MB) -- `analysis.m`.
  - `logs/trajectory_optimization_log_sat_{on,off}.mat` -- `trajectory_optimization.m`'s ON/OFF runs. (Unsuffixed `logs/trajectory_optimization_log.mat` is the pre-criterion May-2026 log read by `replot_TO_04/07.m`.)
  - `logs/centroid_lookup_log.mat` -- `j3_lut_regen.m` (N=41, per-row `radial_profile` + `is_reference`; `ranges`, `X` 40x6, `field_names`). Checkpoint: `logs/centroid_lookup_ckpt_j3.mat`.
  - `logs/surrogate_optimize_log.mat` -- promoted area_proxy refit (`gp_hr`/`y_hr` in area_proxy units); `logs/surrogate_refit_j3_{clamp,area_proxy}.mat` are the non-canonical refits.
  - `logs/ballistic_log.mat` -- `Mortar_Sim.m`. Historical iteration logs + POC artifacts under `logs/archive/`.

## Targeting optimization

The dSMC sweep is wrapped by an outer optimiser that picks launch parameters (`Vo, el, az, w_z0, w_y0, p`) for a given operational `p_target`. **Deterministic phase (current):** `sweep_landing_centroid` is deterministic in xi (one `eom2` call per launch). The outer loop is the LHS coverage pass (`j3_lut_regen.m`) followed by the GP surrogate + `fmincon` inverse step (`j3_surrogate_refit.m`; historical one-shot `surrogate_optimize.m`). Pure MATLAB, no `pyenv`. Design docs: `docs/deterministic_optimization_approach.md`, `docs/plan_A_bo_gp_surrogate.md`, `docs/plan_J3_runbook.md`, `docs/plan_J3_targeting_delta.md`. The canonical LUT + promoted SO log were **regenerated 2026-07-11 on the retuned physics** (seeding-stale warnings resolved; theta* current). A schema-extended regeneration (per `docs/2026-07-10_j3_module_rigor_review.md`) remains future journal work. Author rulings: `w_y0` stays a launch knob for now; the W3 `p_target` is an isotropic Gaussian (default sigma=100 m, {50,100,200} sensitivity). **Stochastic phase (banked):** once wind/aero/IC dispersions make the objective noisy, the CEM and CVaR plans (`docs/archived/plan_B*`, `plan_D*`) apply.

## Not tracked

`.gitignore` lists `CLAUDE.md`, `docs/`, `ATLIS Ongoing Research/`, and `Ballistics_Simulation-master/`, but only blocks *new* files there from being auto-added -- files committed before the rule stay tracked. **`CLAUDE.md` and most of `docs/` (104 files incl. `root.tex`) are tracked on the dev branches but kept off the public `release` branch (11 docs there); `Ballistics_Simulation-master/` (10 files, `eom2.m` with history) is tracked on all branches.** A new file under these paths is local-only until `git add -f`; edits to existing tracked files ARE version-controlled -- commit per branch and be deliberate about what reaches `release`. Worth reading for detail beyond this file: `docs/README.md`, `docs/BALLISTICS_REFERENCE.md`, `docs/dsmc_plan_A_plus.md`, `docs/deterministic_optimization_approach.md`, `docs/gp_surrogate_recipe.md`, the saturation audit trail (`docs/paper_code_agreement_saturation.md`, `docs/saturation_verification_report.md`), and the ballistic reports (`docs/2026-07-10_ballistic_bugsweep.md`, `docs/2026-07-12_ballistic_e2e_verification.md`). The citation-audit record is `docs/citation_audit.md`.

The AIAA SciTech manuscript lives in `docs/scitech-paper/` -- edit only `root.tex` and `bibliography/aiaa_refs.bib`; `root.aux/.bbl/.blg/.log/.out` and `figs/*-eps-converted-to.pdf` regenerate on build. `docs/scitech.tex` is a separate standalone draft. `docs/archived/` holds historical plans (B-F), surveys, judge docs, POC reports. `proof-of-concept/` holds the POC drivers (their `save`/`load` paths reference the pre-cleanup layout). `run_logs/` holds headless stdout/stderr. `design work/` (untracked, not gitignored) holds local design materials -- ignore for code work.

## Citation audit (completed 2026-07-01; full record in docs/citation_audit.md)

Every citation in `docs/scitech-paper/bibliography/aiaa_refs.bib` was checked against its usage in `root.tex` (30 bib entries; 19 cited, 11 uncited). Fixes 4-7 were applied in-source (Sarpturk1987 attribution reworded; Michelena2025 split-cited with Liang2019; Shao2022 "integral surface" claim dropped; `claude` bib title -> "Claude Opus 4"). Items 1-3 (Harper2025 MISMATCH, Tahir2020 PID->PD, Starks2024 overstatement) + Xiong2016 were reviewed/handled manually by the author. **Open items:** (a) **Xiong2016** -- the paper's core reference (11 cites) whose "corrected derivation" contribution rests on Xiong's z/psi laws violating the reaching condition -- remains **unverified** (Optik/Elsevier paywall + Cloudflare); needs a manual PDF fetch to confirm the equations at `root.tex` L674-678. (b) The **11 uncited bib entries** (`Arnold2019, Hicks2023, Barker2025, Devcom2025, Shahzad2023, AWS2025, Yu2013, Batra2021, Hossein2018, Choutri2018, copilot`) await an author decision (cite or remove).
