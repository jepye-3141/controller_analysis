# Ballistic Simulation Reference (`Ballistics_Simulation-master/`)

## Overview

6-DOF (six degree-of-freedom) ballistic flight simulation for a 120mm mortar round. Based on McCoy, "Modern Exterior Ballistics," 2nd ed., Ch. 9. Propagates a rigid body projectile through the atmosphere using modified point-mass equations with full aerodynamic coefficient lookups.

## Entry Point: `Mortar_Sim.m`

Sets initial firing conditions and calls `eom2()`. Default test case:

| Parameter | Value | Unit |
|-----------|-------|------|
| Muzzle velocity (Vo) | 100 | m/s |
| Elevation angle | 45 | deg |
| Azimuth angle | 15 | deg |
| Pitch rate (w_z0) | 1 | rad/s |
| Yaw rate (w_y0) | 0.5 | rad/s |
| Exit pitch AOA (alpha_0) | 2 | deg |
| Exit yaw AOA (beta_0) | -0.5 | deg |
| Initial spin (p) | 0 | rad/s |
| Max sim time | 300 | s |

## Core Propagator: `eom2.m`

### Function: `mortar_propagate()`

**Inputs**: t_max, Vo, el_0, az_0, w_z0, w_y0, alpha_0, beta_0, p, x_0, y_0, z_0, env, output_flag

**Processing**:
1. Computes initial angular rate vector `dx0` from pitch/yaw rates and firing geometry
2. Sets initial velocity `v0` from muzzle velocity and elevation/azimuth
3. Sets initial pointing vector `r0` including angle-of-attack offsets
4. Computes initial angular momentum `h0` incorporating axial spin and transverse rates
5. Propagates a 12-state ODE via `ode45()` with ground impact event detection (the legacy integrated Euler-angle channel was removed 2026-07-10; attitude is derived post-hoc from the pointing vector)
6. Post-processes: apogee, impact interpolation, angle/velocity at impact
7. Applies NUE-to-NWU coordinate transform on output

**Output struct** (`ballistic_sol`):
- `.time` -- time vector
- `.trajectory` -- 15-column state history (NWU coordinates): cols 1:12 are the integrated 12-state solver output; cols 13:15 `[theta phi psi]` (rad) are the derived attitude backfilled post-hoc from the pointing vector (body x-axis along r, phi = 0 -- the `ballistic_deploy_state.m` convention). Interpolate attitude by interpolating r (cols 7:9), never the angle columns (psi wraps at +-pi).
- `.alpha`, `.beta` -- velocity **direction-cosine** angles from the up/east axes (NOT angle of attack / sideslip; at the canonical launch `.alpha` starts at ~47 deg while the true AoA is ~2 deg)
- `.total_aoa` -- true total angle of attack (deg): angle between velocity and the pointing vector (the quantity the aero lookups consume)
- `.apogee` -- maximum altitude (m)
- `.apogee_idx` -- index of apogee in trajectory
- `.impact_time` -- time of ground impact (s)
- `.impact_range` -- range at impact (m)
- `.impact_crossrange` -- cross-range at impact (m)

### Function: `sixdof_ballistics(t, x, env)`

12-state ODE function. State partitioning:

| Indices | Symbol | Description |
|---------|--------|-------------|
| 1-3 | v | Inertial velocity (m/s) |
| 4-6 | h | Angular momentum vector (rad/s, scaled by I_y) |
| 7-9 | r | Unit pointing vector (body axis direction) |
| 10-12 | e | Earth-fixed position (m) |

(The legacy 13-15 `o` Euler-angle channel was removed from the ODE state 2026-07-10 -- it was never a valid attitude; `.trajectory` cols 13:15 are now backfilled post-hoc from r, see the output section above.)

**Aerodynamic model**:
- Drag: C_D = C_D_0 + C_D_del2 * sin^2(alpha)
- Lift: C_L_a = C_L_a0 + C_L_a2 * sin^2(alpha)
- Pitching moment: C_M_a = C_M_a0 + C_M_a2 * sin^2(alpha)
- Damping: CMq_CMa = CMq_CMa_0 + CMq_CMa_2 * sin^2(alpha)
- Magnus force (C_N_pa), Magnus moment (C_M_pa): 2D interpolation (Mach, alpha^2)
- Spin damping (C_l_p), fin spin torque (C_l_delta)

**Environmental effects**:
- Altitude-dependent air density and speed of sound (std_atm.csv interpolation)
- Gravity gradient (altitude and range corrections from spherical earth)
- Coriolis effect from earth rotation (omega = 7.292e-5 rad/s)
- Firing latitude dependency (env.L)

**Rocket motor terms** (disabled for mortar, r_t=0):
- J_tilde_DF: thrust damping force term
- J_tilde_DM: thrust damping moment term

## Aerodynamic Constants: `aero_constants.m`

Loads projectile parameters and 13 aerodynamic coefficient tables:

| Table | Excel Range | Description |
|-------|-------------|-------------|
| C_D_0 | A5:B11 | Zero-yaw drag coefficient |
| C_D_del2 | A15:B22 | Yaw-dependent drag increment |
| C_L_a0 | A26:B30 | Zero-yaw lift slope |
| C_L_a2 | A34:B41 | Yaw-dependent lift increment |
| C_M_a0 | A45:B51 | Zero-yaw pitching moment |
| C_M_a2 | A55:B63 | Yaw-dependent pitching moment increment |
| CMq_CMa_0 | A67:B72 | Pitch damping + moment rate |
| CMq_CMa_2 | A76:B83 | Yaw-dependent damping increment |
| C_N_pa | A87:C109 | Magnus force (3-col: Mach, alpha^2, coeff) |
| C_N | A113:B114 | Normal force derivative |
| C_l_p | A118:B125 | Spin damping moment |
| C_l_delta | A129:B130 | Fin cant spin torque |
| C_M_pa | A134:C180 | Magnus moment (3-col) |

Data source: McCoy, 1998, p. 220.

## Coordinate System

The simulation uses a North-Up-East (NUE) inertial frame internally:
- x = range (north)
- y = altitude (up)
- z = cross-range (east)

Output is rotated to North-West-Up (NWU) via block-diagonal rotation matrix.

## Integration with ATLIS Swarm Simulation

In `analysis.m`, the ballistic simulation provides realistic initial conditions:
1. Full ballistic trajectory is computed
2. State at apogee is extracted (position, velocity, orientation)
3. Velocity/angular rates are transformed from earth to body frame via Euler rotation
4. Initial conditions are scaled (divided by 3) and fed to single-drone controllers
5. Controllers attempt stabilization from this high-energy state
6. Envelope testing iterates along the trajectory to find the operational deployment window
