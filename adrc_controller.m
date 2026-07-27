function u = adrc_controller(A, B, state, xd, x0, constants) %#ok<INUSL>
% Linear Active Disturbance Rejection Control (LADRC) for the symmetric
% quadrotor; runs inside a Simulink MATLAB-Function block (codegen-compiled,
% rapid-accelerator). Baseline peer to dsmc_constraints.m for the single-drone
% ballistic-stabilization comparison. Plan: docs/controller_baselines_plan.md
% section 5.2.
%
% ENVELOPE RESULT: 0/17 on the 17-point ballistic-stabilization envelope (SE(3)
% 14/17, dSMC-sat 9/17; docs/controller_baselines_plan.md). Read that zero as a
% baseline outcome, not a broken file: this law is small-signal by construction
% -- the tilt inversion below is the near-hover map theta_d = -ax_b/g, and the
% z3 clamp exists because a hard deploy saturates every channel ~1000x -- so a
% tumbling deploy starts well outside the regime it is parameterized for.
% Re-score any change with test_baseline_single.m -- same seeds, same oracle as
% the envelope.
%
% Architecture -- Gao bandwidth-parameterized cascade: six 3rd-order discrete
% Extended State Observers (ESO) + PD, all forward-Euler at dt = 1/50 s. Each
% ESO estimates [output; rate; total-disturbance f]; the PD forms a virtual
% control and the disturbance estimate is actively cancelled
% (u = (u0 - f_hat)/b0). That lumped-disturbance estimator -- not the I/O map
% -- is the family-distinct feature vs the existing PID (see caveat, bottom).
%
%   OUTER  X (state 1) -> ax_des ,  Y (state 2) -> ay_des        [b0 = 1]
%     |-- yaw-rotate (ax_des,ay_des) by -psi into the heading frame, then
%         invert the near-hover tilt map:  theta_d = -ax_b/g ,  phi_d = +ay_b/g
%   INNER  phi   (state 4, track phi_d)   -> Mx   [b0 = 1/Jxx]
%          theta (state 5, track theta_d) -> My   [b0 = 1/Jyy]
%   ALT    Z     (state 3, track xd(3))   -> T    [b0 = 1/m_uncertain, + m*g FF]
%   YAW    psi   (state 6, track xd(6))   -> Mz   [b0 = 1/Jzz]
%
% ANTI-WINDUP -- ALL channels use POST-SATURATION FEEDBACK (dsmc_constraints.m
% item 4 philosophy): each ESO is driven by the ACHIEVED command, not the
% commanded one. The outer loops use the achieved (post-tilt-clamp) accel; the
% inner/alt/yaw loops use the achieved post-rotor-clip command (control is formed
% first, saturated, THEN the observers update). Every z3 disturbance estimate is
% additionally clamped to its actuator authority. Without this a hard ballistic
% deploy saturates every channel ~1000x and the z3 states wind up to ~1e7,
% driving the loop effectively open-loop (diagnosed 2026-07-20). Exact no-op in
% the unsaturated region, so the small-signal response is unchanged.
%
% A, B unused (Simulink-interface parity with dsmc_constraints.m).
% state, xd : 6x1 [X; Y; Z; phi; theta; psi] -- WORLD position then Euler
%   angles, ROLL (phi) BEFORE PITCH (theta): the in-model Selector
%   [10 11 12 8 7 9] basis, NOT the constants.m C-matrix order.
% x0 : initial state, read on the first tick only (ESO seeding, like dSMC).
% Returns the 4x1 POST-clip absolute command u = [T; Mx; My; Mz].
%
% PD/ESO gains below are bandwidth-parameterized LITERALS
% (kp = wc^2, kd = 2*wc, L = [3*wo; 3*wo^2; wo^3]); only b0 reads constants --
% m_uncertain carries the mass-uncertainty injection (as in dsmc_constraints.m);
% Jxx/Jyy/Jzz have no uncertain variants. Bandwidths (rad/s, all << the
% pi/dt ~ 157 rad/s Nyquist, wo*dt <= 0.72 so the forward-Euler ESOs stay
% inside the unit circle):
%   outer X,Y  wc=1.5 wo=10    inner phi,theta  wc=12 wo=36
%   altitude Z wc=4   wo=16    yaw psi          wc=6  wo=24
% The outer loop is kept ~8x slower than the inner attitude loop for cascade
% timescale separation (wc=2 was found underdamped against the 12 rad/s inner).
%
% Forward-Euler discretization margin (reviewed 2026-07-17, kept as-is): the
% inner attitude channel (wo=36) is the tightest, wo*dt=0.72. It is stable
% (triple discrete ESO pole at 1-wo*dt=0.28, well inside the unit circle; the
% Euler bound is wo*dt<2) and it has been run end-to-end. That is a claim about
% the discretization only. It says nothing about whether this arm recovers a
% ballistic deploy, which is a separate question. Forward Euler does realize the
% observer slightly more aggressively than a matched Tustin/ZOH would,
% and the innovation gain La3=46656 would amplify measurement noise. Benign in
% THIS harness: the controller is fed exact, noise-free plant state and the
% inner reference is slow (outer wc=1.5). If sensor noise or a faster inner
% reference is later introduced (e.g. swarm parity), re-tune with wo*dt<=0.5
% while keeping wo>=3*wc (so the ESO stays faster than its loop) -- which means
% also slowing wc, hence re-deriving the whole cascade -- or Tustin-discretize
% the ESO; then re-check. Not done here: it perturbs the tuning the recorded
% results were produced with, for zero benefit in the current noise-free
% single-drone case.

%% Persistent ESO state: one [z1; z2; z3] per channel (z1~output estimate,
%% z2~rate estimate, z3~lumped-disturbance estimate). Seeded from x0.
persistent eso_x eso_y eso_z eso_phi eso_th eso_psi

g   = constants.g;
mc  = constants.m_uncertain;   % controller-side mass (uncertainty-injection path)
Jxx = constants.Jxx;
Jyy = constants.Jyy;
Jzz = constants.Jzz;
dt  = constants.dt;

if isempty(eso_x)
    eso_x   = [x0(1); 0; 0];
    eso_y   = [x0(2); 0; 0];
    eso_z   = [x0(3); 0; 0];
    eso_phi = [x0(4); 0; 0];
    eso_th  = [x0(5); 0; 0];
    eso_psi = [x0(6); 0; 0];
end

%% Input gains b0 (PLANT FACTS: moments are DIRECT inputs -- no arm length l).
b0_z   = 1.0 / mc;    % world-Z accel ~ (T/m) - g
b0_phi = 1.0 / Jxx;   % phi_ddot   ~ Mx/Jxx
b0_th  = 1.0 / Jyy;   % theta_ddot ~ My/Jyy
b0_psi = 1.0 / Jzz;   % psi_ddot   ~ Mz/Jzz
% Outer loops live directly in world-accel space (X_ddot = ax_des) -> b0 = 1.

%% Bandwidth-parameterized gain literals (kp=wc^2, kd=2*wc, L=[3wo;3wo^2;wo^3]).
kp_o = 2.25; kd_o = 3;    Lo1 = 30;  Lo2 = 300;  Lo3 = 1000;   % outer  wc=1.5 wo=10
kp_a = 144;  kd_a = 24;   La1 = 108; La2 = 3888; La3 = 46656;  % inner  wc=12  wo=36 (wo*dt=0.72, tightest FE margin; header NOTE)
kp_z = 16;   kd_z = 8;    Lz1 = 48;  Lz2 = 768;  Lz3 = 4096;   % alt    wc=4   wo=16
kp_p = 36;   kd_p = 12;   Lp1 = 72;  Lp2 = 1728; Lp3 = 13824;  % yaw    wc=6   wo=24

%% OUTER horizontal loops -- PD (pre-clamp) -> desired world accel.
ax_des = ladrc_ctrl(eso_x, xd(1), 1.0, kp_o, kd_o);
ay_des = ladrc_ctrl(eso_y, xd(2), 1.0, kp_o, kd_o);

%% Yaw decoupling: rotate the world-accel command into the heading frame by
%% -psi (psi = state(6)), then invert the near-hover tilt map. Signs from the
%% PLANT FACTS at psi=0: X_ddot ~ -g*theta (theta<0 -> +X); Y_ddot ~ +g*phi
%% (phi>0 -> +Y). Ballistic recovery carries large yaw, so the rotation matters.
cpsi = cos(state(6));
spsi = sin(state(6));
ax_b =  cpsi * ax_des + spsi * ay_des;
ay_b = -spsi * ax_des + cpsi * ay_des;
theta_d = -ax_b / g;
phi_d   =  ay_b / g;

% Tilt-command safety clamp (~34 deg). Keeps the small-angle inversion sane and
% away from the theta=+-90 deg Euler singularity when the outer loop demands a
% large accel (e.g. arresting ballistic velocity). A hard limit, not a knob.
tilt_max = 0.6;
theta_d  = min(max(theta_d, -tilt_max), tilt_max);
phi_d    = min(max(phi_d,   -tilt_max), tilt_max);

%% Disturbance-estimate (z3) anti-windup bounds, set to the accel / angular-accel
%% each actuator can actually produce (|Mx|,|My| <= 10, |Mz| <= 8 Nm from
%% apply_rotor_clip; horizontal accel via tilt <= g*tan(tilt_max)). Beyond the
%% authority a larger z3 only demands an unachievable actuator effort.
%% These moment numbers are frozen at the DEFAULT cap: 10 = b*Omega2_max and
%% 8 = 2*d*Omega2_max evaluated at Omega2_max = 2. They do not track
%% constants.Omega2_max, so a swept cap would leave them mismatched to the real
%% authority. That does no harm today, since the cap campaign only sweeps the two
%% dSMC arms and SE(3). Derive them from constants.Omega2_max before putting ADRC
%% in a cap sweep.
z3lim_o   = g * tan(tilt_max);
z3lim_z   = 2 * g;
z3lim_att = 10 / Jxx;   % Jxx = Jyy
z3lim_psi = 8  / Jzz;

%% OUTER observer update with post-saturation feedback: feed the ESOs the
%% ACHIEVED (post-clamp) horizontal accel rotated back to world by +psi, so
%% z3 tracks the true residual instead of winding up under a saturated clamp.
%% Reduces to the commanded accel exactly when the clamp is inactive.
axb_ach = -g * theta_d;
ayb_ach =  g * phi_d;
ax_app  = cpsi * axb_ach - spsi * ayb_ach;
ay_app  = spsi * axb_ach + cpsi * ayb_ach;
eso_x = clip3(ladrc_obs(eso_x, state(1), ax_app, 1.0, Lo1, Lo2, Lo3, dt), z3lim_o);
eso_y = clip3(ladrc_obs(eso_y, state(2), ay_app, 1.0, Lo1, Lo2, Lo3, dt), z3lim_o);

%% CONTROL LAW -- form every inner/alt/yaw command from the CURRENT ESO estimate;
%% observer updates are DEFERRED to after saturation (anti-windup, below).
u_Mx    = ladrc_ctrl(eso_phi, phi_d,   b0_phi, kp_a, kd_a);   % roll  -> Mx
u_My    = ladrc_ctrl(eso_th,  theta_d, b0_th,  kp_a, kd_a);   % pitch -> My
T_delta = ladrc_ctrl(eso_z,   xd(3),   b0_z,   kp_z, kd_z);   % alt   -> thrust delta
u_T     = mc * g + T_delta;                                   % + m*g gravity feedforward
u_Mz    = ladrc_ctrl(eso_psi, xd(6),   b0_psi, kp_p, kd_p);   % yaw   -> Mz

%% Assemble and apply the shared naive per-rotor saturation clip (mandatory
%% last step -- identical actuator limit across all comparison arms).
u_pre = [u_T; u_Mx; u_My; u_Mz];
u     = apply_rotor_clip(u_pre, constants);

%% POST-saturation observer updates (anti-windup): feed each inner/alt/yaw ESO
%% the ACHIEVED post-clip command, so its z3 tracks the true residual instead of
%% the (up to ~1000x) saturation deficit. Altitude sees the achieved thrust DELTA
%% (gravity feedforward removed). Each z3 is then clamped to actuator authority.
Mx_ach     = u(2);
My_ach     = u(3);
Mz_ach     = u(4);
Tdelta_ach = u(1) - mc * g;
eso_phi = clip3(ladrc_obs(eso_phi, state(4), Mx_ach,     b0_phi, La1, La2, La3, dt), z3lim_att);
eso_th  = clip3(ladrc_obs(eso_th,  state(5), My_ach,     b0_th,  La1, La2, La3, dt), z3lim_att);
eso_z   = clip3(ladrc_obs(eso_z,   state(3), Tdelta_ach, b0_z,   Lz1, Lz2, Lz3, dt), z3lim_z);
eso_psi = clip3(ladrc_obs(eso_psi, state(6), Mz_ach,     b0_psi, Lp1, Lp2, Lp3, dt), z3lim_psi);

end

% Distinctness caveat: bandwidth-tuned linear ADRC is input-output close to a
% 2-DOF PID (set-point weighting + filtered derivative). The family-distinct
% claim rests on the ESO -- active estimation and cancellation of the lumped
% disturbance -- NOT on a different I/O map. Do not overclaim it as unrelated
% to the framework's existing PID.  (docs/controller_baselines_plan.md 5.2.)


%% ------------------------------------------------------------------------
function uc = ladrc_ctrl(z, r, b0, kp, kd)
% LADRC channel control law (codegen-safe, pure): PD on the ESO estimates plus
% lumped-disturbance cancellation. Reference rate assumed zero (regulator).
%   z = [z1; z2; z3] current ESO estimate (z1~output, z2~rate, z3~disturbance)
%   r = reference setpoint   b0 = input gain   kp,kd = PD gains
    u0 = kp * (r - z(1)) - kd * z(2);
    uc = (u0 - z(3)) / b0;
end

function z_out = ladrc_obs(z, y, u_applied, b0, L1, L2, L3, dt)
% Forward-Euler 3rd-order ESO update (codegen-safe, pure). u_applied is the
% ACTUALLY-applied control so the disturbance estimate tracks the true residual
% (post-saturation feedback) rather than a windup artifact.
%   y = measured output   L1,L2,L3 = ESO gains   innovation err = y - y_hat
    err = y - z(1);
    z_out = [z(1) + dt * (z(2)                  + L1 * err);
             z(2) + dt * (z(3) + b0 * u_applied + L2 * err);
             z(3) + dt * (                        L3 * err)];
end

function z = clip3(z, lim)
% Anti-windup clamp on the ESO disturbance state z3 (3rd element) to +-lim.
    z(3) = min(max(z(3), -lim), lim);
end
