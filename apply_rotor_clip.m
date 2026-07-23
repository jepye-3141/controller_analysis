function u = apply_rotor_clip(u, constants) %#ok<INUSL>
% Shared naive per-rotor saturation clip for the baseline comparison
% controllers (H-inf, ADRC, SE(3)). Factored verbatim from the dSMC STEP-12b
% clip in dsmc_constraints.m so every arm respects IDENTICAL actuator limits and
% the control LAW is the only variable across the comparison -- which is what
% makes the dSMC's smart priority-weighted Omega^2 allocation a fair, meaningful
% win over the others' naive clip.
%
% Maps the commanded u = [T; Mx; My; Mz] into rotor-speed-squared (Omega^2)
% space through the mixer inverse, clamps each rotor to [Omega2_min, Omega2_max],
% then remixes. Under saturation the clamp distorts the command DIRECTION (it
% does not preserve the requested thrust/moment ratios) -- that distortion is
% precisely the deliberately-"naive" baseline the dSMC is meant to beat.
%
% Codegen-safe: plain matrix math, gains as literals, no persistent state.
% constants is accepted for interface uniformity with the control laws (and to
% ease a future promotion of these coefficients onto the bus) but is UNUSED
% today: b, d, and the Omega^2 bounds are literals kept byte-identical to
% dsmc_constraints.m, which remains their canonical source of truth. If those
% literals change there (e.g. via set_omega2_max.m), mirror the change here.

% Mixing-matrix coefficients (Omega^2 -> u), from dsmc_constraints.m
b = 5;   % thrust coefficient
d = 2;   % yaw torque coefficient

% Per-rotor saturation bounds in Omega^2 space (shipped dSMC values). At
% Omega2_max = 2 the total thrust ceiling is 4*b*Omega2_max = 40 N (T/W = 5.10
% at m*g = 7.848 N); hover Omega^2 per rotor = m*g/(4*b) = 0.392 lies inside.
Omega2_min = 0.0;
Omega2_max = 2;

% Mixer (same TM as dsmc_constraints STEP 12b). Rows: [T; Mx; My; Mz].
TM = [b   b   b   b;
      0  -b   0   b;
     -b   0   b   0;
      d  -d   d  -d];
TM_inv = inv(TM); %#ok<MINV>  % constant 4x4; explicit inverse mirrors the source

% Mix -> clamp each rotor -> remix (STEP 12b, verbatim).
Om2 = TM_inv * u;
Om2 = min(max(Om2, Omega2_min), Omega2_max);
u   = TM * Om2;

end
