# ATLIS Sims -- Program Reference

## Overview

This project implements a comprehensive quadrotor UAV swarm simulation and control comparison framework. It compares four control strategies (LQR, continuous SMC, discrete SMC, and PID) across multiple trajectory types and conditions using Simulink models, then runs extensive post-simulation analysis in MATLAB. A separate 6-DOF ballistic mortar simulation provides realistic initial conditions for a ballistic deployment scenario.

The primary workflow is:
1. Define quadrotor physical constants and compute LQR gains (`constants.m`)
2. Run swarm Simulink models across test cases via `analysis.m`
3. Post-process results: inter-drone distances, kinetic energy, settling time, scoring heatmaps
4. Export publication-quality figures to `figs/`

---

## Directory Structure

```
ATLIS Sims/
|-- analysis.m                          # Main simulation driver and post-processing
|-- constants.m                         # Quadrotor parameters, state-space model, LQR gain computation
|-- linear_nonlinear_comparison.m       # Compares linearized vs nonlinear quadrotor dynamics
|-- trajectory_optimization.m           # Saturation ON/OFF driver: two sweep_landing_centroid runs + plot_saturation_comparison
|-- sweep_landing_centroid.m            # Reusable parsim sweep + centroid + radial-reachability summary
|-- j3_lut_regen.m                      # Current LHS driver: checkpointed regen of centroid_lookup_log.mat (N=41)
|-- j3_surrogate_refit.m                # GP refit on the LUT log (hr_mode clamp | area_proxy) + LOO-RMSE
|-- surrogate_optimize.m                # GP surrogate + fmincon over the launch space (historical one-shot)
|-- replot_TO_04.m                      # Re-render TO_04 from saved sweep log
|-- replot_TO_07.m                      # Re-render TO_07 from saved sweep log
|-- replot_LUT_01.m                     # Re-render LUT_01 from the LUT log
|-- replot_SO_01.m                      # Re-render SO_01 from the surrogate log
|-- dsmc_no_constraints.m               # Canonical (frozen) discrete SMC, lifted from Simulink block
|-- dsmc_constraints.m                  # Plan A+ augmented dSMC under active development
|-- deprecated/                         # centroid_lookup_table.m, smc_discrete_formulation.m,
|                                       #   lqr_linearization_and_path_planning.m -- do not run (see its README)
|
|-- se3_controller.m                    # Journal peer: geometric SE(3) (Lee-Leok-McClamroch)
|-- hinf_controller.m                   # Journal peer: H-inf runtime law (matrices from hinf_design.m)
|-- hinf_design.m                       # Offline mixed-sensitivity synthesis, run once by hand
|-- adrc_controller.m                   # Journal peer: linear ADRC (extended-state observer + PD)
|-- apply_rotor_clip.m                  # Shared naive per-rotor Omega^2 clip used by all three peers
|-- make_baseline_models.m              # Clones the dSMC single model + its subsystem ref per peer
|-- test_baseline_single.m              # Validates one peer model against ballistic_success
|-- sweep_ballistic_envelope.m          # The 17-point envelope loop, factored out of analysis.m
|-- sweep_saturation_campaign.m         # Canonical Omega2_max cap sweep -> CAP_01..CAP_04
|-- run_envelope_cap_campaign.m         # Envelope-vs-cap companion -> ENV_CAP_01
|-- run_cap_grid_stage1.m               # Cap-grid extension, stage 1 (checkpointed, merges committed logs)
|-- run_cap_grid_stage2.m               # Cap-grid extension, stage 2 -> CAP_05 + merged log
|-- replot_CAP_05.m                     # Re-render CAP_05 from the merged grid log
|-- forensic_envelope.m                 # Per-station dSMC-sat vs SE(3) divergence forensic
|-- run_crit_cap_sweep.m                # Re-flies the cap grid persisting per-trial crit_table
|-- rescore_criterion.m                 # Re-scores a stored crit_table at any VelRatioMax
|-- run_criterion_sensitivity.m         # Criterion-robustness sweep -> CRIT_01
|
|-- lqr_controller.slx                  # Continuous LQR controller (single drone)
|-- pid_redux_controller.slx            # PID controller (single drone)
|-- zoh_smc_controller.slx              # ZOH SMC controller (single drone) -- the cSMC block
|-- actual_discrete_smc_controller.slx  # Discrete SMC controller (single drone)
|-- actual_se3_controller.slx           # SE(3) controller (single drone)
|-- actual_hinf_controller.slx          # H-inf controller (single drone)
|-- actual_adrc_controller.slx          # ADRC controller (single drone)
|-- discrete_lqr_controller.slx        # Discrete LQR controller (single drone)
|-- discrete_pid_redux_controller.slx   # Discrete PID controller (single drone)
|
|-- lqr_swarm.slx                       # 10-drone swarm with LQR control
|-- smc_swarm.slx                       # 10-drone swarm with continuous SMC control
|-- discrete_smc_swarm.slx              # 10-drone swarm with discrete SMC control
|-- pid_redux.slx                       # 10-drone swarm with PID control
|
|-- lqr_swarm_single.slx                # Single-drone LQR (for ballistic scenario)
|-- smc_swarm_single.slx                # Single-drone cSMC (for ballistic scenario)
|-- discrete_smc_swarm_single.slx       # Single-drone dSMC (for ballistic scenario)
|-- pid_redux_single.slx                # Single-drone PID (for ballistic scenario)
|-- se3_swarm_single.slx                # Single-drone SE(3) (for ballistic scenario)
|-- hinf_swarm_single.slx               # Single-drone H-inf (for ballistic scenario)
|-- adrc_swarm_single.slx               # Single-drone ADRC (for ballistic scenario)
|
|-- *.slxc                              # Simulink cache files (auto-generated)
|
|-- logs/                               # All workspace logs live here (active scripts save/load with logs/ prefix)
|   |-- analysis_log.mat                # Saved workspace from analysis.m run
|   |-- trajectory_optimization_log.mat # Saved workspace from sweep_landing_centroid(_, true)
|   |-- centroid_lookup_log.mat         # Saved LHS lookup (lookup, ranges, X, N, field_names)
|   |-- surrogate_optimize_log.mat      # Fitted GPs + recommended theta_best
|   |-- ballistic_log.mat               # Saved workspace from Mortar_Sim.m
|   |-- saturation_campaign_log.mat     # Canonical cap anchors (+ _ckpt.mat)
|   |-- cap_grid_stage1_log.mat         # Cap-grid stage 1 (+ _ckpt.mat)
|   |-- cap_grid_full_log.mat           # Cap-grid stages merged, read by replot_CAP_05.m
|   |-- envelope_cap_campaign_log.mat   # Envelope-vs-cap results
|   |-- forensic_envelope_log.mat       # Per-station divergence forensic traces
|   |-- crit_cap_sweep_log.mat          # Cap grid with per-trial crit_table retained
|   |-- criterion_sensitivity_log.mat   # Counts re-scored at VelRatioMax in {2,3,5,10,Inf}
|   |-- hinf_design.mat                 # Synthesized H-inf controller (Ad_K..Dd_K, gamma, nK)
|   |-- archive/                        # Historical iteration logs, POC results, dense-sweep archive
|
|-- run_logs/                           # stdout/stderr from headless MATLAB runs (.log). No code references these.
|
|-- proof-of-concept/                   # Banked POC drivers from the down-selected plans (poc_{a..f}, test_pocs*, smoketest_full, diff_step1, atlis_targeting_app)
|
|-- figs/                               # Output figures (PNG+EPS): 0-39_*, TO_01..07 (+ _sat_on/_sat_off), SAT_CMP_summary, LUT_01, SO_01, CAP_01..05, ENV_CAP_01, POC_*
|-- docs/                               # This documentation folder (plans + design docs)
|   |-- archived/                       # Banked stochastic plans (B-F), surveys, judges, POC reports
|-- ATLIS Ongoing Research/             # Research materials (not tracked)
|
|-- Ballistics_Simulation-master/       # 6-DOF ballistic mortar simulation
    |-- Mortar_Sim.m                    # Entry point: sets initial conditions, calls eom2()
    |-- eom2.m                          # Core 6-DOF ballistic propagator + 15-state ODE
    |-- aero_constants.m                # Loads atmosphere table and aero coefficients
    |-- std_atm.csv                     # Standard atmosphere lookup table
    |-- Aerodynamic_Char_120mm_Mortar.xlsx  # Aero coefficients vs Mach (McCoy, 1998)
    |-- README.md                       # Original README for this sub-project
    |-- images/                         # Sample output images
```

---

## Quadrotor Model

### Physical Parameters (from `constants.m`)

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Gravity | g | 9.81 | m/s^2 |
| Arm length | l | 0.2 | m |
| Mass | m | 0.8 | kg |
| Axial MOI | Jxx | 1.8e-3 | kg*m^2 |
| Transverse MOI | Jyy | 1.8e-3 | kg*m^2 |
| Yaw MOI | Jzz | 1.5e-3 | kg*m^2 |
| Motor torque constant | kmt | 0.1 | m |

### State Vector (12 states)

The quadrotor state is a 12-element vector:
```
x = [vx; vy; vz; wx; wy; wz; theta; phi; psi; x; y; z]
     ----body vel----  --ang vel--  --Euler angles--  --position--
```

- States 1-3: Body-frame translational velocities (m/s)
- States 4-6: Body-frame angular velocities (rad/s)
- States 7-9: Euler angles -- theta (pitch), phi (roll), psi (yaw) (rad)
- States 10-12: Inertial-frame position (m)

### Control Inputs (4 inputs)

```
u = [T; Mx; My; Mz]
```
- T: Total thrust (N) -- delta from hover
- Mx: Roll moment (N*m)
- My: Pitch moment (N*m)
- Mz: Yaw moment (N*m)

These are computed from individual motor thrusts via the mixer matrix:
```
[T ]   [1    1    1    1  ] [F1]
[Mx] = [0   -l    0    l  ] [F2]
[My]   [l    0   -l    0  ] [F3]
[Mz]   [kmt -kmt  kmt -kmt] [F4]
```

### Linearized State-Space Model

The system is linearized about hover equilibrium into the standard form `xdot = Ax + Bu`, `y = Cx + Du`. The A, B matrices encode the coupling between translational and rotational dynamics through gravity terms. An augmented state-space model (15 states) adds 3 integral states for position tracking (x, y, z).

### LQR Gain Computation

Discrete LQR gains are computed at 50 Hz via `lqrd()`:
- Q: 15x15 diagonal weighting matrix emphasizing altitude (6000), angular rates (1080), heading integral (300)
- R: 4x4 identity
- Output: `Kp_d` (12-state proportional) and `Ki_d` (3-state integral) gain matrices

Controllability and observability are verified via `ctrb()` and `obsv()` rank tests.

---

## Control Strategies

### 1. LQR (Linear Quadratic Regulator)
- Discrete-time formulation at 50 Hz sample rate
- Augmented state-space with integral action on position
- Gains computed via `lqrd()` with normalized Q matrix
- Simulink models: `lqr_controller.slx`, `discrete_lqr_controller.slx`

### 2. Continuous SMC (Sliding Mode Control)
- Uses nonlinear 6-DOF dynamics directly
- Sliding surfaces defined on position/velocity error
- Implemented in `zoh_smc_controller.slx` (the orphan `smc_controller.slx`, which held the same law with two latent bugs, was deleted 2026-07-20)

### 3. Discrete SMC
- Discrete-time sliding mode formulation (prototyped in `deprecated/smc_discrete_formulation.m`; the shipped laws are `dsmc_no_constraints.m` and `dsmc_constraints.m`)
- Decoupled into actuated (z, yaw) and underactuated (x/roll, y/pitch) channels
- Sliding surfaces: `sz`, `spsi` for actuated; `sphi`, `stheta` for underactuated
- Uses reaching law with tunable gains (nuz, nupsi, nu3, nu4)
- Implemented in `actual_discrete_smc_controller.slx`, `discrete_smc_swarm.slx`

### 4. PID
- Cascaded PID architecture
- Implemented in `pid_redux_controller.slx`, `discrete_pid_redux_controller.slx`

---

## Simulink Model Hierarchy

### Single-Drone Controllers
These contain the inner-loop control law for one quadrotor:
- `lqr_controller.slx` / `discrete_lqr_controller.slx`
- `zoh_smc_controller.slx` / `actual_discrete_smc_controller.slx`
- `pid_redux_controller.slx` / `discrete_pid_redux_controller.slx`
- journal peers: `actual_se3_controller.slx` / `actual_hinf_controller.slx` / `actual_adrc_controller.slx`

### Swarm Models (10 drones)
These instantiate 10 copies of a controller, each with offset initial positions. They accept initial conditions from the MATLAB workspace (`xi`, `xf`, `simcase`) and output position (`posout`) and velocity (`velout`) timeseries, plus command (`cmdout`) on the SMC and dSMC models:
- `lqr_swarm.slx` -- LQR swarm
- `smc_swarm.slx` -- continuous SMC swarm
- `discrete_smc_swarm.slx` -- discrete SMC swarm
- `pid_redux.slx` -- PID swarm

### Single-Drone Swarm Models (ballistic scenario)
Same structure but with 1 drone, used for ballistic stabilization testing:
- `lqr_swarm_single.slx`
- `smc_swarm_single.slx`
- `discrete_smc_swarm_single.slx`
- `pid_redux_single.slx`
- journal peers: `se3_swarm_single.slx`, `hinf_swarm_single.slx`, `adrc_swarm_single.slx`

All seven also log `rotvelout` (leader Euler-angle rates) and `ctrlout` (the applied `[T Mx My Mz]`). `cmdout` is logged by the SMC, dSMC and peer models but **not** by the LQR or PID models, in either the swarm or single flavour.

---

## Main Simulation Script: `analysis.m`

This is the master driver (~2040 lines). It runs all simulations and generates all analysis outputs.

### Execution Flow

The line numbers below are indicative only. They were written against a ~2550-line `analysis.m` and the 2026-07-12 release pass cut it to ~2040, so treat them as ordering, not addresses -- find each phase by its section comment.

#### Phase 1: Setup (lines 1-15)
- Clears workspace, clears Simulink Data Inspector
- Adds ballistics simulation path
- Loads `constants.m`
- Defines trajectory case constants: STEP=1, F8=2, SPIRAL=3, STABILIZE=4
- Loads all four swarm Simulink models

#### Phase 2: Parametric Uncertainty Simulations (lines 16-145)
- Defines 4 mass/gain uncertainty sets:
  - Leader: mL=1.6 kg, KL=0.8
  - Follower group 1: m1=0.6 kg, K1=0.7
  - Follower group 2: m2=1.2 kg, K2=0.6
  - Follower group 3: m3=1.0 kg, K3=0.5
- Constructs per-agent A, B matrices (A1/B1, A2/B2, A3/B3)
- Runs STEP trajectory for all 4 controllers with uncertainty (50s sim, Rapid mode)

#### Phase 3: Nominal Simulations (lines 146-224)
- Resets to nominal parameters (m=0.8 kg)
- Runs three trajectory types across all 4 controllers:
  - **Step decrease**: x0=[2,4,10], xf=[0,0,0], 50s sim
  - **Figure-8**: x0=[0,0,0], xf=[2,4,10], 150s sim
  - **Arithmetic spiral**: same ICs, 250s sim

#### Phase 4: Ballistic Scenario (lines 226-273)
- Runs 6-DOF ballistic simulation to get trajectory of 120mm mortar
- Extracts apogee state (position, velocity, orientation)
- Converts to body frame via Euler rotation
- Runs single-drone stabilization from apogee state (100s sim)

#### Phase 5: Ballistic Envelope Testing (lines 275-324)
- Iterates along ballistic trajectory at every 10th point
- At each point, deploys a single drone with the ballistic state
- Tests each controller for 30s to see if it can stabilize
- Records which trajectory indices each controller can handle

#### Phase 6: Post-Processing and Visualization (lines 326-end)

**Position Plots (figs 1-2)**: 5x4 grid of 3D trajectory plots showing all test cases across all controllers.

**Distance Analysis (figs 3-18)**: For each controller x trajectory combination, computes min/avg/max Euclidean distance between every pair of swarm members (10x10 heatmaps).

**Delta-Distance Heatmaps (figs 19-34)**: Same as above but showing the difference from ideal (nominal) separation distances, highlighting formation maintenance quality.

**Separation Score Table (fig 35)**: Normalized similarity scores comparing actual vs nominal separations.

**Kinetic Energy Plots (fig 36)**: Time histories of per-drone kinetic energy for every scenario.

**KE Scores (fig 37)**: Pairwise relative integrated kinetic energy comparison between controllers.

**Settling Time Scores (fig 38)**: 5% settling time comparison using worst-case drone metric.

**Ballistic Envelope (fig 39)**: Which trajectory points allow successful stabilization per controller.

### Key Workspace Variables

| Variable Pattern | Type | Description |
|---|---|---|
| `step_out_lqr` | SimulationOutput | LQR step response results |
| `step_out_*_uncertainty` | SimulationOutput | Uncertainty variant results |
| `f8_out_*` | SimulationOutput | Figure-8 results |
| `spiral_out_*` | SimulationOutput | Spiral results |
| `step_out_*_ballistic` | SimulationOutput | Ballistic stabilization results |
| `*_min_distances` | 10x10 double | Min pairwise distance heatmap |
| `*_avg_distances` | 10x10 double | Avg pairwise distance heatmap |
| `*_max_distances` | 10x10 double | Max pairwise distance heatmap |
| `*_ke` | Tx10 double | Per-drone kinetic energy time series |
| `ballistic_envelope_*` | 1xN double | Trajectory indices where stabilization succeeds |

### Simulink Output Signals

The swarm models output three timeseries:
- `posout`: Position data, shape [n_drones x 3 x time]
- `velout`: Velocity data, shape [n_drones x 3 x time]
- `cmdout`: Command data (desired trajectory), shape [3 x time]

---

## Supporting Scripts

### `linear_nonlinear_comparison.m`
Compares linearized (`lin_dynamics`) and nonlinear (`nl_dynamics`) quadrotor dynamics using ODE45 over 1 second with small initial Euler angle perturbations (0.05 rad). Validates that the linearization is accurate near hover.

### `lqr_linearization_and_path_planning.m`
Contains multiple trajectory planning approaches, each in its own code section:

1. **Two-point BVP** (infeasible, labeled "not working"): Uses `fmincon` to find optimal control inputs that drive the system between two states via direct collocation with ODE45 integration.

2. **Multiple Shooting** (labeled "working"): Discretizes the trajectory into 100 nodes with 5 sub-steps each over 10 seconds. Uses 4th-order Runge-Kutta within each segment. Solves via `fmincon` with interior-point algorithm.

3. **RRT* with MATLAB** (labeled "sort of working"): Uses `minsnappolytraj` for minimum-snap trajectory generation with 500 samples. Computes differential flatness-based control inputs from position derivatives.

4. **Sampling-based controls planning**: Discretizes the control input space and builds combinatorial control action set.

5. **Symbolic linearization along trajectory**: Computes Jacobian matrices symbolically, evaluates at each trajectory point, and computes time-varying LQR gains.

### `smc_discrete_formulation.m`
Implements and tests the discrete-time sliding mode controller:
- `temporary_dynamics()`: Full nonlinear quadrotor dynamics with drag and gyroscopic terms
- `SMC_discrete()`: The discrete SMC control law with:
  - Actuated surfaces (z-altitude, psi-yaw): Direct sliding mode with reaching gains `nuz`, `nupsi`
  - Underactuated surfaces (x/roll, y/pitch): Cascaded sliding mode with gains `nu3`, `nu4`
- Test loop: Runs 4000 steps at dt=0.01s, propagating via ODE45 between steps
- Parameters: m=2 kg, Jxx=Jyy=1.25, Jzz=2.5, l=0.2m

---

## Ballistic Simulation (`Ballistics_Simulation-master/`)

### Overview
6-DOF simulation of a 120mm mortar round from McCoy, "Modern Exterior Ballistics" Ch. 9.

### `Mortar_Sim.m` (Entry Point)
Sets initial conditions:
- Muzzle velocity: 100 m/s
- Elevation: 45 deg, Azimuth: 15 deg
- Pitch/yaw rates: 1, 0.5 rad/s
- Angle of attack at exit: 2 deg pitch, -0.5 deg yaw
- No initial spin (p=0)

### `aero_constants.m`
Loads environmental and projectile data:
- **Environment**: WGS84 gravity, earth radius, standard atmosphere from CSV
- **Projectile**: 120mm mortar -- d=119.56mm, m=13.585 kg, I_x=0.02335, I_y=0.23187 kg*m^2
- **Aerodynamics**: 13 coefficient lookup tables from Excel (C_D_0, C_D_del2, C_L_a0, C_L_a2, C_M_a0, C_M_a2, CMq+CMa, C_N_pa, C_N, C_l_p, C_l_delta, C_M_pa)

### `eom2.m` (Core Propagator)
Contains two functions:

**`mortar_propagate()`** -- Main wrapper:
- Computes initial state from firing angles and angular rates (15 states: 3 velocity, 3 angular momentum, 3 pointing vector, 3 position, 3 Euler angles)
- Runs ODE45 with impact detection event function
- Post-processes: apogee, impact time/range/cross-range, impact angle/velocity
- Applies NUE-to-NWU coordinate rotation for output
- Optionally generates trajectory plots

**`sixdof_ballistics()`** -- ODE function:
- 15-state derivative computation
- Aerodynamic forces: drag, lift, Magnus, pitch damping, spin damping, fin cant
- Environmental: altitude-dependent density/Mach, gravity gradient, Coriolis (earth rotation)
- Rocket motor terms (disabled for mortar: r_t=0)

### State Vector (15 states)
```
x = [v(1:3); h(4:6); r(7:9); e(10:12); o(13:15)]
     -vel-   -ang mom-  -pointing-  -position-  -Euler-
```

### Coordinate System
- Range (x/north), Altitude (y/up), Cross-range (z/east) -- "NUE"
- Output rotated to NWU (North-West-Up) convention

---

## Targeting Optimization

The dSMC sweep is deterministic in launch state: one `eom2()` ballistic call seeds a fixed `parsim` deploy x cross-track x neighbor grid, and the same `(Vo, el, az, w_z0, w_y0, p)` always produces the same landing centroid and radial-reachability profile. That makes outer-loop targeting a low-dimensional deterministic black-box optimization problem rather than a stochastic one.

The current pipeline has two pieces:

1. **`j3_lut_regen.m`** -- Latin-Hypercube sample (McKay, Beckman & Conover 1979) over the 6-D launch box. Calls `sweep_landing_centroid(_, false)` once per sample and writes `logs/centroid_lookup_log.mat` plus `figs/LUT_01_trajectories_and_centroids.png`. Each sample produces `(p_centroid, reachability_pct, half_radius)` and a per-sample `radial_profile`. Checkpointed and resumable (N=40 LHS rows plus the operating point re-simulated as row 41). It supersedes `deprecated/centroid_lookup_table.m`, which used the pre-J3 box and stored no radial profile -- do not run that one.

2. **`surrogate_optimize.m`** -- DACE-style Kriging surrogate (Sacks, Welch, Mitchell & Wynn 1989) over the LHS data, optimized in-place rather than via expensive new samples. Four GPs are fit with `fitrgp` using an ARD Matern-5/2 kernel (Rasmussen & Williams 2006): one each for `p_centroid_x`, `p_centroid_y`, `reachability_pct`, and `half_radius`. Multi-start `fmincon` (SQP) on the scalarized objective
   ```
   J(theta) = ||p_centroid_pred(theta) - p_target||^2 - lambda * half_radius_pred(theta)
   ```
   yields `theta_best`. The scalarization is the weighted-sum variant from Marler & Arora (2004); `lambda` is a tunable knob trading centroid accuracy for reach-zone width. With `verify=true` the script runs one ground-truth `sweep_landing_centroid` at the surrogate optimum to bound the GP error. Outputs `logs/surrogate_optimize_log.mat` and `figs/SO_01_predicted_J_slices.png`.

   **Careful with this one.** `j3_surrogate_refit.m` is the current refit path, and the canonical `logs/surrogate_optimize_log.mat` is now the *promoted area_proxy refit* -- its `gp_hr`/`y_hr` are in area_proxy units, not half_radius metres. Re-running `surrogate_optimize.m` as-is would overwrite that log and regress the promotion. Note also that only the horizontal centroid and the half-radius term enter `J`: `gp_reach` is fit but unused, and `p_target(3)` is ignored.

The acquisition machinery from Jones, Schonlau & Welch (1998) (expected-improvement, EGO) is unnecessary here because the inner simulator is noise-free, so the GP variance only contributes to the verification confidence interval, not to a sequential design loop.

Design rationale and full citation list: [`docs/deterministic_optimization_approach.md`](deterministic_optimization_approach.md). Reference plan with the BO machinery that we re-use the GP layer of: [`docs/plan_A_bo_gp_surrogate.md`](plan_A_bo_gp_surrogate.md). Reproducibility / paper-ready recipe (forward GP math, training set spec, inverse-optimisation formulation, paper-ready paragraph): [`docs/gp_surrogate_recipe.md`](gp_surrogate_recipe.md). When wind, aero, and IC dispersions are added, the outer loop becomes Monte-Carlo and the stochastic plans in `docs/archived/` (CEM, CVaR) become the right tools again.

---

## Output Figures Reference

| # | Filename | Description |
|---|----------|-------------|
| 0 | `0_ballistic_trajectory.png` | 6-DOF ballistic trajectory profile |
| 1 | `1_flight_path.png` | 5x4 grid: all trajectories, all controllers |
| 2 | `2_flight_path_cmds.png` | Commanded trajectories (step, uncertain, f8, spiral) |
| 3-6 | `3-6_step_*_distances.png` | Step response distance heatmaps (LQR, SMC, dSMC, PID) |
| 7-10 | `7-10_step_*_uncertainty_distances.png` | Step w/ uncertainty distance heatmaps |
| 11-14 | `11-14_f8_*_distances.png` | Figure-8 distance heatmaps |
| 15-18 | `15-18_spiral_*_distances.png` | Spiral distance heatmaps |
| 19-22 | `19-22_heatmap_step_*_distances.png` | Step delta-distance heatmaps (actual - nominal) |
| 23-26 | `23-26_heatmap_step_*_uncertainty_distances.png` | Step uncertainty delta-distance heatmaps |
| 27-30 | `27-30_heatmap_f8_*_distances.png` | Figure-8 delta-distance heatmaps |
| 31-34 | `31-34_heatmap_spiral_*_distances.png` | Spiral delta-distance heatmaps |
| 35 | `35_score_table.png` | Separation similarity % scores |
| 36 | `36_kinetic_energy.png` | KE time histories for all scenarios |
| 37 | `37_kinetic_energy_scores.png` | Pairwise relative KE comparisons |
| 38 | `38_settling_time_scores.png` | Settling time comparisons |
| 39 | `39_ballistic_envelope.png` | Stabilization envelope along ballistic trajectory |
| TO_01 | `TO_01_kinetic_energy.png` | Per-cell KE time histories over the deploy x cross-track sweep |
| TO_02 | `TO_02_settling_time.png` | 5%-settling-time heatmap, deploy x cross-track |
| TO_03 | `TO_03_stabilization_envelope.png` | Cells that converged within the sim horizon |
| TO_04 | `TO_04_landing_envelope.png` | Per-cell landing positions; success ratio coloring with magenta zero-step |
| TO_05 | `TO_05_unique_reachability.png` | Reachability map (which cells can reach each landing patch) |
| TO_06 | `TO_06_control_norm.png` | Per-cell integrated control norm |
| TO_07 | `TO_07_landing_heatmap.png` | Power-weighted landing centroid over the interpolated success-ratio field (emitted as `_sat_on`/`_sat_off` pairs by `trajectory_optimization.m`) |
| LUT_01 | `LUT_01_trajectories_and_centroids.png` | 3D ballistic trajectories + per-sample landing centroid over the LHS launch space |
| SO_01 | `SO_01_predicted_J_slices.png` | Surrogate `J(theta)` contour slices over `(Vo, el, az, p)` pairs at the GP optimum |
| SAT_CMP | `SAT_CMP_summary.png` | Saturation ON-vs-OFF comparison: radial profile, centroids, stats (from `plot_saturation_comparison.m`) |
| CAP_01 | `CAP_01_reach_vs_tw.png` | Landing reachability vs actuator authority `T/W = 4b*Omega2_max/(mg)`, one line per arm (from `sweep_saturation_campaign.m`) |
| CAP_02 | `CAP_02_reach_stratified.png` | The same sweep split by deploy regime: ascending stations 1-3 vs feasible/descending 4-8 |
| CAP_03 | `CAP_03_clip_activity.png` | Clip-active fraction of timesteps vs cap (solid = upper ceiling, dashed = lower cutoff) |
| CAP_04 | `CAP_04_allocation_gap.png` | Reachability gap dSMC-sat minus dSMC-naive vs cap -- the value of priority-weighted allocation |
| CAP_05 | `CAP_05_se3_crossover_map.png` | SE(3) success vs per-rotor authority on both metrics (envelope /17 left, landing /140 right) over the extended cap grid, with dSMC-sat/nosat flat reference lines and the SE(3) peak starred (from `run_cap_grid_stage2.m`; re-render with `replot_CAP_05.m`) |
| CRIT_01 | `CRIT_01_velratio_sensitivity.png` | Success counts re-scored at `VelRatioMax` in {2, 3, 5, 10, Inf} across the cap grid -- the criterion-robustness check (from `run_criterion_sensitivity.m`) |
| ENV_CAP_01 | `ENV_CAP_01.png` | Envelope successes out of 17 vs cap for the three cap-varying arms (from `run_envelope_cap_campaign.m`) |

---

## Swarm Configuration

The swarm consists of 10 drones (1 leader + 9 followers) initialized with X-axis offsets:
```
offsets = [0, 1, -1, 2, 3, 4, -2, 3, 5, -4] meters
```
All Y and Z offsets are zero. The leader is drone index 1.

### Trajectory Cases
- **STEP (1)**: Start at [2,4,10], command to origin. Tests step response and settling.
- **F8 (2)**: Figure-8 pattern starting from origin targeting [2,4,10].
- **SPIRAL (3)**: Arithmetic spiral pattern, same endpoints.
- **STABILIZE (4)**: Stabilization from ballistic deployment state (high velocity, arbitrary orientation).

---

## Scoring Methodology

### Separation Similarity Score
```
score = norm(actual_distances - ideal_distances) / norm(actual_distances + ideal_distances)
```
Reported as `100 * (1 - score)` percentage. Higher is better (closer to nominal separation).

### Relative Kinetic Energy Score
```
ke_score(i,j) = 100 * (KE_i - KE_j) / (KE_i + KE_j)
```
Positive means controller i uses more energy than j. Computed from trapezoidal integration of leader KE.

### Settling Time
5% criterion on worst-case drone: finds the earliest time at which the maximum positional error (normalized by initial-to-final distance) across all drones is below 5%.
