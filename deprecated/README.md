# Deprecated

Superseded prototypes kept for reference and future analysis. They are **not**
part of the active pipeline (`analysis.m` and the targeting scripts never call
them) and do **not** run as-is: each mixes script code with local function
definitions, which MATLAB rejects with *"Function definitions in a script must
appear at the end of the file."* Fix that ordering before reusing them.

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
