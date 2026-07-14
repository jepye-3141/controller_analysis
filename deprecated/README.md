# Deprecated

Superseded prototypes kept for reference and future analysis. They are **not**
part of the active pipeline (`analysis.m` and the targeting scripts never call
them). The two original prototypes below (`smc_discrete_formulation.m`,
`lqr_linearization_and_path_planning.m`) also do **not** run as-is: each mixes
script code with local function definitions, which MATLAB rejects with
*"Function definitions in a script must appear at the end of the file."* Fix
that ordering before reusing them.

- `centroid_lookup_table.m` — original LHS lookup driver, superseded by
  `j3_lut_regen.m` (checkpointed/resumable, per-sample `radial_profile`,
  current criterion + widened Vo/el/p ranges) with figures rendered by
  `replot_LUT_01.m`. Unlike the other files here it parses and runs — do
  **NOT** run it: it saves to the canonical `logs/centroid_lookup_log.mat`
  and would silently overwrite the current N=41 log with a stale-box N=20
  one (p [-2,2], el [35,55], no `radial_profile`).
- `smc_discrete_formulation.m` — standalone discrete-SMC prototype, superseded by
  `dsmc_no_constraints.m` (frozen baseline) and `dsmc_constraints.m` (Plan A+).
  Uses a different state ordering and physical parameters than the production
  controllers. Note the sliding-gain typo `cos(xk(4)*cos(xk(6)))` (should be
  `cos(xk(4))*cos(xk(6))`) and the pitch-gyro term dividing by `Jxx` (should be
  `Jyy`).
- `lqr_linearization_and_path_planning.m` — trajectory-planning experiments
  (BVP, multiple shooting, RRT*, symbolic time-varying LQR). The only consumer of
  the Symbolic Math Toolbox and `minsnappolytraj`; references `C`/`Jxx` that are
  defined only if `constants.m` was run first.
