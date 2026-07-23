function u = se3_controller(A, B, state, xd, x0, constants) %#ok<INUSL>
% Geometric SE(3) tracking controller (Lee-Leok-McClamroch, CDC 2010,
% arXiv:1003.2005), specialised to THIS plant (system_dynamics.m) -- NOT the
% textbook. Runs in a Simulink MATLAB-Function block (rapid-accelerator);
% codegen-safe. One of the baseline peers to the custom dSMC
% (docs/controller_baselines_plan.md sec 5.3).
%
% Signature matches dsmc_constraints.m. A, B unused (Simulink interface).
%   state, xd : 6x1 [X; Y; Z; phi; theta; psi]  (WORLD position, then Euler
%               angles ROLL phi BEFORE PITCH theta -- the in-model Selector
%               [10 11 12 8 7 9], NOT the constants.m C order).
%   x0        : 6x1 initial state, read on the first tick only (FD seeding).
% Returns u = [f; Mx; My; Mz] (4x1) -- ABSOLUTE thrust f (~ m*g at hover) and
% absolute body moments, driving system_dynamics.m directly. The final
% computational step is the shared naive per-rotor clip apply_rotor_clip.m,
% identical to every other baseline arm.
%
% No velocity or body-rate is measured: world velocity is finite-differenced
% from state(1:3), and body rates Omega are recovered by finite-differencing
% the Euler angles state(4:6) and inverting the plant's ZYX kinematic map --
% the geometric analogue of the dSMC's dxk = (xkp1 - xk)/dt. Feedforward
% (v_d, a_d, Omega_d) is zero: the ballistic setpoint is a fixed hover pose,
% so this is pure-feedback SE(3) (valid).
%
% ========================================================================
% CONVENTION DERIVATION  (everything below is read straight off the plant
% system_dynamics.m, so the operator can verify each line by eye)
% ========================================================================
% Plant state z = [vx vy vz | wx wy wz | theta phi psi | X Y Z]:
%   BODY-frame linear velocities, BODY rates Omega=[wx;wy;wz], ZYX Euler
%   angles (phi roll / theta pitch / psi yaw), WORLD position. Input
%   u=[T Mx My Mz], T ABSOLUTE thrust, absolute body moments. Jmp=0.
%
% (1) BODY->WORLD ROTATION R.  The plant's world-velocity rows (position
%     kinematics, system_dynamics.m xdot(10:12)) give [Xdot;Ydot;Zdot]=R*[vx;vy;vz]:
%       R = [ cpsi*cth,  -spsi*cphi + cpsi*sth*sphi,   spsi*sphi + cpsi*sth*cphi ;
%             spsi*cth,   cpsi*cphi + spsi*sth*sphi,  -cpsi*sphi + spsi*sth*cphi ;
%             sth,       -cth*sphi,                   -cth*cphi                  ]
%     Rows 1,2 ARE the standard ZYX body->world rotation. Row 3 is the
%     NEGATIVE of standard ZYX (standard row 3 = [-sth, cth*sphi, cth*cphi]).
%     => R is ORTHOGONAL but IMPROPER: det(R) = -1 (this is the #1 SE(3)
%     trap here; world Z is up while body z is down). Consequences, all
%     verified: R still obeys Rdot = R*hat(Omega) (S=diag(1,1,-1) is constant),
%     so the geometric attitude kinematics carry over unchanged; and because
%     R_d is built with the SAME improper convention below, R_d.'*R is PROPER
%     (the two reflections cancel), so e_R and the trace-error Psi are valid
%     SO(3) quantities and the standard Lyapunov proof goes through.
%
% (2) TRANSLATIONAL DYNAMICS in WORLD frame. Thrust enters ONLY vz_dot as
%     -T/m; the body-frame thrust acceleration [0;0;-T/m] maps to world as
%     R*[0;0;-T/m] = -(T/m)*R(:,3). Gravity, mapped likewise, is -g*e3
%     (e3=[0;0;1] world-up). Hence
%           m*A_world = -m*g*e3  +  T*n_world ,   n_world = -R(:,3).
%     Check at hover (phi=theta=0): R(:,3)=[0;0;-1] so n_world=[0;0;1] (up),
%     and A_world=0 requires T=m*g. => gravity feedforward is +m*g*e3 and the
%     thrust axis is n_world=-R(:,3).
%
% (3) DESIRED THRUST FORCE + THRUST MAGNITUDE SIGN. Choose the world force
%           F_des = -k_x*e_x - k_v*e_v + m*g*e3
%     (at hover F_des = m*g*e3, up, magnitude m*g). To realise m*A_world=F_des
%     we need T*n_world = F_des, so the thrust magnitude is the projection of
%     F_des onto the ACTUAL thrust axis n_world:
%           f = F_des . n_world = F_des . (-R(:,3)) = -F_des.'*R(:,3).
%     At hover f = -[0;0;m*g].'*[0;0;-1] = m*g > 0. Sign taken from the plant,
%     NOT from Lee et al. (their NED z-down form would flip it). With perfect
%     attitude the world closed loop collapses to m*e_x_ddot = -k_x*e_x - k_v*e_v.
%
% (4) DESIRED ATTITUDE R_d (same improper convention as R). Desired thrust
%     axis = b3_d = F_des/||F_des||, so the desired body-z column is
%     r3_d = -b3_d (because n_world=-R(:,3) => R_d(:,3)=-b3_d). Complete the
%     frame from a desired heading b1c=[cos psi_d; sin psi_d; 0]:
%           r2_d = unit( b1c x r3_d ),   r1_d = r3_d x r2_d,   R_d=[r1_d r2_d r3_d].
%     At level hover this returns [cpsi_d -spsi_d 0; spsi_d cpsi_d 0; 0 0 -1]
%     (det -1), matching R's convention so R_d.'*R is proper. Hand-checked:
%     a pure +roll gives e_R=[+sin phi;0;0] => Mx=-k_R*sin phi<0 (restoring);
%     +pitch => My<0; +yaw => Mz<0; a +X position error tilts to theta_d>0
%     (plant needs theta<0 for +X accel, theta>0 for -X) -- all signs correct.
%
% (5) ROTATIONAL DYNAMICS are the STANDARD rigid body J*Omega_dot = M - Omega x J*Omega
%     with BODY rates (system_dynamics.m xdot(4:6); Jxx=Jyy so the Mz coupling
%     term vanishes). Geometric law
%           M = -k_R*e_R - k_Omega*e_Omega + Omega x (J*Omega)
%     cancels the gyroscopic term and imposes J*Omega_dot = -k_R*e_R - k_Omega*e_Omega.
%
% (6) BODY RATES FROM EULER RATES. The plant's ZYX map (xdot(7:9)) inverts to
%           wx = phi_dot - sth*psi_dot
%           wy = cphi*theta_dot + cth*sphi*psi_dot
%           wz = -sphi*theta_dot + cth*cphi*psi_dot
%     i.e. Omega = Winv*[phi_dot; theta_dot; psi_dot], Winv coded below.
%
% ------------------------------------------------------------------------
% OFFLINE NUMERICAL VALIDATION (do before trusting the gains; NOT run here):
%   seed the 12-state plant at a perturbed attitude via ballistic_deploy_state.m
%   (e.g. |Omega|~a few rad/s, 30-60 deg roll/pitch), close the loop with
%   system_dynamics.m integrated at 50 Hz, and confirm (a) e_R and e_x decay
%   monotonically-ish to ~0, (b) it clears the 1e5 numerical-blowup guard, and
%   (c) f stays >0 through the transient. Cross-check R against so3(...).rotm
%   OFFLINE only (never inside this block). Then score with ballistic_success.m.
% ========================================================================

%% Persistent finite-difference buffers (mirrors dSMC xk/xkp1 exactly)
persistent xk xkp1

s   = reshape(state, [6, 1]);
xdc = reshape(xd,    [6, 1]);

if isempty(xk)
    xk   = reshape(x0, [6, 1]);   % first tick: seed both from x0 -> zero rates
    xkp1 = xk;
end

% Shift only on a genuinely new sample, so repeated same-timestep calls in
% rapid-accelerator mode do not collapse the finite differences (dSMC pattern).
if abs(norm(xkp1 - s)) > 1e-9
    xk   = xkp1;
    xkp1 = s;
end

%% Parameters (all gains are LITERALS; controller-side mass = m_uncertain)
g   = constants.g;
Jxx = constants.Jxx;
Jyy = constants.Jyy;
Jzz = constants.Jzz;
dt  = constants.dt;
m   = constants.m_uncertain;   % mass-uncertainty injection flows through here
Jvec = [Jxx; Jyy; Jzz];

% Gains, TUNED against the 17-point ballistic-stabilization envelope (a surrogate
% (k_Omega, k_v) sweep, then confirmed in se3_swarm_single.slx). Cascade
% separation is preserved: the attitude loop stays faster than the position loop.
% The dominant lever is the velocity-damping k_v: raising it brakes the deploy
% velocity (satisfies the speed-ratio criterion) AND gentles the position chase,
% which indirectly calms the attitude loop -- so it improves the rotation-rate
% criterion where raising k_Omega DIRECTLY cannot. k_Omega has a hard stability
% ceiling ~0.12: it multiplies the finite-differenced body-rate estimate, so
% past that the 50 Hz ZOH attitude loop rings up and every trial diverges (the
% whole k_Omega>=0.20 region scores 0/17). k_x and k_R are kept at their gentle
% baseline; k_v=5.0 / k_Omega=0.09 sits centrally on a robust score ridge.
%   position:  k_x=1.0,  k_v=5.0
%   attitude:  k_R=0.26, k_Omega=0.09
k_x     = 1.0;
k_v     = 5.0;
k_R     = 0.26;
k_Omega = 0.09;

%% Evaluate at the delayed sample xk with forward-difference rates (dSMC pairing)
phi   = xk(4);
theta = xk(5);
psi   = xk(6);

cphi = cos(phi);  sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cpsi = cos(psi);  spsi = sin(psi);

% Body->world rotation R (improper, det=-1; rows 1,2 standard ZYX, row 3 negated)
R = [ cpsi*cth,  -spsi*cphi + cpsi*sth*sphi,   spsi*sphi + cpsi*sth*cphi;
      spsi*cth,   cpsi*cphi + spsi*sth*sphi,  -cpsi*sphi + spsi*sth*cphi;
      sth,       -cth*sphi,                   -cth*cphi                 ];

% World velocity by finite difference of world position
v_world = (xkp1(1:3) - xk(1:3)) / dt;

% Euler rates by finite difference (psi wraps at +-pi, but over one 1/50 s tick
% the raw difference is acceptable -- matches dSMC dxk(6), no unwrap applied)
phi_dot   = (xkp1(4) - xk(4)) / dt;
theta_dot = (xkp1(5) - xk(5)) / dt;
psi_dot   = (xkp1(6) - xk(6)) / dt;

% Body rates Omega = Winv * [phi_dot; theta_dot; psi_dot] (inverse ZYX map)
Winv = [1,     0,        -sth;
        0,  cphi,   cth*sphi;
        0, -sphi,   cth*cphi];
Omega = Winv * [phi_dot; theta_dot; psi_dot];

%% Position control -> desired world force F_des
e_x = xk(1:3) - xdc(1:3);     % position error (world)
e_v = v_world;                % velocity error (v_d = 0)
e3  = [0; 0; 1];              % world up
F_des = -k_x * e_x - k_v * e_v + m * g * e3;

% Thrust magnitude = projection of F_des onto the actual thrust axis -R(:,3)
f = -(F_des.') * R(:, 3);

%% Desired attitude R_d (same improper convention as R)
% Guard the thrust-axis normalisation. If F_des underflows to (near-)zero the
% desired thrust AXIS is undefined: b3_d/r3_d would collapse to [0;0;0], both
% the primary and the world-up cvec cross products would vanish, and the
% r2_d = cvec/norm(cvec) step below would divide 0/0 -> a NaN command. That NaN
% would NOT trip the plant's norm(vel/rotvel) >= 1e5 blow-up guard (NaN >= 1e5
% is false), so it would silently corrupt a trajectory. Floor F_des ITSELF (not
% just its norm) to the gravity feedforward so the axis defaults to level hover
% at the desired heading. Reachability is measure-zero (needs the PD term to
% exactly cancel m*g*e3 in all three axes in floating point). f (already
% computed above from the TRUE F_des) stays ~0, so this only fixes the attitude
% branch and does not fabricate thrust.
nF = norm(F_des);
if nF < 1e-9
    F_des = m * g * e3;
    nF    = m * g;
end
b3_d = F_des / nF;
r3_d = -b3_d;                 % desired body-z column (=-thrust axis)

psi_d = xdc(6);
b1c   = [cos(psi_d); sin(psi_d); 0];
cvec  = hat(b1c) * r3_d;      % cross(b1c, r3_d)
if norm(cvec) < 1e-6
    % degenerate (desired thrust axis ~ horizontal & aligned with heading):
    % yaw is ill-defined in this recovery regime -- complete with world-up.
    b1c  = [0; 0; 1];
    cvec = hat(b1c) * r3_d;
end
r2_d = cvec / norm(cvec);
r1_d = hat(r3_d) * r2_d;      % cross(r3_d, r2_d); unit (r3_d _|_ r2_d)
R_d  = [r1_d, r2_d, r3_d];

%% Attitude control -> body moment M
% e_R = 0.5*(R_d.'*R - R.'*R_d)^vee ; e_Omega = Omega - R.'*R_d*Omega_d, Omega_d=0
E   = 0.5 * (R_d.' * R - R.' * R_d);
e_R = vee(E);
e_Omega = Omega;             % Omega_d = 0

M = -k_R * e_R - k_Omega * e_Omega + hat(Omega) * (Jvec .* Omega);

%% Assemble and apply the shared naive per-rotor clip (identical to all arms)
u = [f; M];
u = apply_rotor_clip(u, constants);

end


%% Codegen-safe local helpers -----------------------------------------------
function S = hat(v)
% so(3) hat: 3-vector -> skew matrix, with hat(a)*b = cross(a, b).
S = [    0, -v(3),  v(2);
      v(3),     0, -v(1);
     -v(2),  v(1),     0];
end

function v = vee(S)
% Inverse of hat: skew matrix -> 3-vector.
v = [S(3, 2); S(1, 3); S(2, 1)];
end
