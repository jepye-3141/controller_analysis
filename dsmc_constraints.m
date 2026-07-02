function u = dsmc_constraints(A, B, state, xd, x0, constants) %#ok<INUSL>
% MATLAB_Function — Plan A+ augmented dSMC controller for symmetric quadrotor.
%
% Augments the user's existing discrete sliding mode controller (eqs 21,
% 22, 38, 39 of the original derivation) with two structural additions:
%
%   1. VECTOR AUXILIARY STATE  xi in R^4, with deficit-coupling matrix D
%      that captures both same-channel and cross-channel deficit response.
%      Modified sliding variable: tilde_s = s - D*xi (Plan A+ §4.1, §4.2).
%
%   2. CLOSED-FORM PRIORITY-WEIGHTED ALLOCATION in Omega^2-space, with
%      operational priority M_x, M_y > T > M_z (Plan A+ §3).
%
% Inputs:
%   A, B     kept for Simulink interface compatibility (unused in body)
%   state    6x1 vehicle state [x; y; z; phi; theta; psi]
%   xd       6x1 desired state
%   x0       6x1 initial state (used on first tick only)
%
% Output:
%   u        4x1 POST-SATURATION command [u_T; u_Mx; u_My; u_Mz].
%            This is u_bar in Plan A+ notation: the command that, when
%            applied through the rotor mixer T_M^{-1}, produces motor
%            speeds within [Omega2_min, Omega2_max] bounds.
%
% Plan A+ design variables (require user input — see "Design variables"
% block below):
%   Omega2_min, Omega2_max   per-rotor saturation bounds in Omega^2 space
%
% Plan A+ placeholders (Lyapunov certificate weights — affect runtime
% certificate logging only, not closed-loop dynamics; not used in this
% implementation, listed for completeness):
%   rho = 1; W_s = W_xi = (1/2)*I  (tune per Plan A+ §11.2 if needed)

%% ============== Persistent state ==================================
persistent xk xkp1 xk_d xkp1_d ukm1 xi TM_inv
g   = constants.g;
l   = constants.l;
Jmp = constants.Jmp;
Jxx = constants.Jxx;
Jyy = constants.Jyy;
Jzz = constants.Jzz;
dt  = constants.dt;
m   = constants.m_uncertain;
saturation_on = constants.saturation_on;

if isempty(xk)
    xk      = x0;
    xkp1    = xk;
    xk_d    = xd;
    xkp1_d  = xd;
    ukm1    = zeros(4, 1);   % first tick: no prior command
    xi      = zeros(4, 1);   % Plan A+: zero auxiliary state at t=0
end

%% ============== State buffer updates (user's existing logic) ======
if abs(norm(xkp1 - state)) > 1e-9
    xk   = xkp1;
    xkp1 = state;
end
if abs(norm(xkp1_d - xk_d)) > 1e-9
    xk_d   = xkp1_d;
    xkp1_d = xd;
end
if abs(norm(xkp1_d - xd)) > 1e-9
    xk_d   = xkp1_d;
    xkp1_d = xd;
end

%% ============== Parameters (Table I, Table IV) ====================
% Drag coefficients (Table IV)
K1 = 0.0001;  K2 = 0.0001;  K3 = 0.0001;
K4 = 0.0012;  K5 = 0.0012;  K6 = 0.0012;

% Reaching-law gains (Table IV)
nuz   = 7;
nupsi = 7;
nu3   = 14;
nu4   = 14;

% Sliding-surface coefficients (eq 32, constant-valued ones)
az   = 6;
apsi = 1;
a3   = 2;
a4   = 8;
a7   = 2;
a8   = 8;

% Mixing matrix coefficients (Omega^2 -> u)
b = 5;   % thrust coefficient
d = 2;   % yaw torque coefficient

%% ============== Plan A+ design variables ==========================
% Per-rotor saturation in Omega^2 (rotor-speed-squared) space.
%
%   MAGIC NUMBERS — chosen for the user's quadrotor (m = 0.8 kg):
%     Omega2_min = 0.0  : motors can spin to zero (typical ESC convention)
%     Omega2_max = 2.0  : 4*b*Omega2_max = 40 N total thrust  =>  T/W = 5.10
%
%   Hover-trim feasibility (Plan A+ condition C2):
%     4*b*Omega2_max = 40 > m*g = 7.848  (PASSES; T/W > 1)
%   Hover-trim Omega^2 per rotor:
%     Omega2_hover = m*g / (4*b) = 0.392  (inside [0, 2.0])
%
% Adjust these to match actual motor specifications when known.
Omega2_min = 0.0;
Omega2_max = 2;

% Matched auxiliary-state contraction rate (Plan A+ §4.4):
%   k_xi(alpha) = eta_alpha * dt   per channel
% Ordering follows xi indexing: [thrust/z; roll/Mx; pitch/My; yaw/Mz]
k_xi = [nuz * dt; nu3 * dt; nu4 * dt; nupsi * dt];

%% ============== Mixing matrix (Omega^2 -> u) =======================
TM = [b   b   b   b;
      0  -b   0   b;
     -b   0   b   0;
      d  -d   d  -d];
if isempty(TM_inv)
    TM_inv = inv(TM);
end

%% ============== Recover Omega_r from previous (POST-SATURATION) u ==
% ukm1 is the POST-SATURATION command from the previous tick (Plan A+
% convention), so Omegas correspond to the actually-applied rotor speeds
% — which is what's required for the gyroscopic-precession term.
Omegas = TM_inv * ukm1;
Omegas = real(sqrt(complex(Omegas)));
Omegar = real(Omegas(1) - Omegas(2) + Omegas(3) - Omegas(4));

%% ============== Velocity estimates =================================
dxk   = (xkp1 - xk)     / dt;
dxk_d = (xkp1_d - xk_d) / dt;   % zero for constant references

%% ============== STEP 1: Fully-actuated sliding variables (eqs 16, 17) ====
szk   = az   * (xk_d(3) - xk(3)) + (dxk_d(3) - dxk(3));
spsik = apsi * (xk_d(6) - xk(6)) + (dxk_d(6) - dxk(6));

% Clipping retained. Plan A+ theory says the auxiliary state xi should
% absorb sustained tracking deficits, making this clip redundant; in
% practice (2026-05-15 sweep diagnostic) disabling it caused the dSMC
% deploy x cross-track x neighbor sweep to land zero successful
% trajectories at any deploy point, even at dt=1/200. The transient
% growth of s_z and s_psi before xi converges still needs the clip.
szk   = min(szk,   2);  szk   = max(szk,   -2);
spsik = min(spsik, 2);  spsik = max(spsik, -2);

%% ============== STEP 2: D-matrix entries (state-only rows) ==========
% Plan A+ §4.1, with corrected sign (D = -A = +dt*[...] overall).
%   D(1,1) = +dt * cos(phi)*cos(theta)/m   (thrust deficit -> z sliding)
%   D(4,4) = +dt / Jzz                      (yaw moment deficit -> psi sliding)
D11 = dt * cos(xk(4)) * cos(xk(5)) / m;
D44 = dt / Jzz;

%% ============== STEP 3: Modified sliding variables (z, psi) =========
% Plan A+: tilde_s = s - D*xi  (anti-windup substitution)
tilde_szk   = szk   - D11 * xi(1);
tilde_spsik = spsik - D44 * xi(4);

%% ============== STEP 4: Synthesize u_T, u_Mz (eqs 21, 22) ===========
% Same form as user's existing controller, with s -> tilde_s.
u_T = (m / (cos(xk(4)) * cos(xk(5)))) * ...
      (-az * dxk(3) + (K3/m) * dxk(3) + g + nuz * tilde_szk);

u_Mz = Jzz * ((-apsi * dxk(6) + (K6/Jzz) * dxk(6)) + nupsi * tilde_spsik);

%% ============== STEP 5: a_i from CURRENT-TICK COMMANDED u_T (eq 32) ====
% Same convention as user's existing code (single-pass).
% Plan A+ deficit-cancellation derivation requires this evaluation point.
%
% Caveat: a_i are evaluated at the COMMANDED u_T, not the post-saturation
% bar u_T. Under deep thrust saturation, commanded u_T can be far from
% feasible, making a_1, a_2, a_5, a_6 numerically erratic and degrading
% the slowly-varying-D assumption that underpins the ISS bound
% (dsmc_saturation.tex, "Slowly varying D_k" paragraph).
a1 =  6 * m / (u_T * cos(xk(6)));
a2 =  2 * m / (u_T * cos(xk(6)));
a5 = -6 * m / (u_T * cos(xk(4)) * cos(xk(6)));
a6 = -2 * m / (u_T * cos(xk(4)) * cos(xk(6)));

%% ============== STEP 6: D-matrix entries (underactuated rows) =======
% Plan A+ §4.1; positive overall sign per corrected convention.
%   D(2,1) = +dt * a1 * g1_k        (thrust deficit -> roll sliding;   sign of g1)
%   D(2,2) = +dt * a3 * l / Jxx     (roll-moment deficit -> roll sliding)
%   D(3,1) = +dt * a5 * g2_k        (thrust deficit -> pitch sliding;  a5<0)
%   D(3,3) = +dt * a7 * l / Jyy     (pitch-moment deficit -> pitch sliding)
g1_k = (cos(xk(4)) * sin(xk(5)) * sin(xk(6)) - sin(xk(4)) * cos(xk(6))) / m;
g2_k = (cos(xk(4)) * sin(xk(5)) * cos(xk(6)) + sin(xk(4)) * sin(xk(6))) / m;

D21 = dt * a1 * g1_k;
D22 = dt * a3 * l / Jxx;
D31 = dt * a5 * g2_k;
D33 = dt * a7 * l / Jyy;

%% ============== STEP 7: Underactuated sliding variables (eq 27) =====
sphik   = a1*(dxk_d(2) - dxk(2)) + a2*(xk_d(2) - xk(2)) + ...
          a3*(dxk_d(4) - dxk(4)) + a4*(xk_d(4) - xk(4));
sthetak = a5*(dxk_d(1) - dxk(1)) + a6*(xk_d(1) - xk(1)) + ...
          a7*(dxk_d(5) - dxk(5)) + a8*(xk_d(5) - xk(5));

%% ============== STEP 8: Modified sliding variables (phi, theta) =====
% Plan A+: tilde_s_phi includes cross-coupling from xi(1) (thrust deficit)
% plus same-channel from xi(2) (Mx deficit). Same structure for pitch.
tilde_sphik   = sphik   - D21 * xi(1) - D22 * xi(2);
tilde_sthetak = sthetak - D31 * xi(1) - D33 * xi(3);

%% ============== STEP 9: Drift terms f_1, f_2 (eqs 31, 32) ===========
f1_k = (dxk(5)*dxk(6)*(Jyy - Jzz) + Jmp*dxk(5)*Omegar - K4*l*dxk(4)) / Jxx;
f2_k = (dxk(6)*dxk(4)*(Jzz - Jxx) - Jmp*dxk(4)*Omegar - K5*l*dxk(5)) / Jyy;

%% ============== STEP 10: Synthesize u_Mx, u_My (eqs 38, 39) =========
% Same form as user's existing controller, with s -> tilde_s.
u_Mx = (Jxx / (l * a3)) * (-a1 * (g1_k * u_T - K2 * dxk(2)/m) - a2 * dxk(2) - ...
                            a3 * f1_k - a4 * dxk(4) + nu3 * tilde_sphik);
u_My = (Jyy / (l * a7)) * (-a5 * (g2_k * u_T - K1 * dxk(1)/m) - a6 * dxk(1) - ...
                            a7 * f2_k - a8 * dxk(5) + nu4 * tilde_sthetak);

%% ============== STEP 11: Form unconstrained u =======================
u_unc = [u_T; u_Mx; u_My; u_Mz];

%% ============== STEP 12: Priority-weighted allocation (optional) ====
% When saturation_on, run Plan A+ §3 closed-form allocation in Omega^2
% space and feed the resulting deficit into the auxiliary state. When
% off, bypass allocation entirely: u_bar = u_unc, delta_u = 0, and xi
% stays at its initialized zero. With xi == 0 the modified sliding
% variables tilde_s collapse to s, making the OFF branch numerically
% equivalent to dsmc_no_constraints.m.
if saturation_on
    [~, u_bar] = priority_weighted_allocate(u_unc, Omega2_min, Omega2_max, b, d);
    delta_u    = u_unc - u_bar;
else
    u_bar   = u_unc;
    delta_u = zeros(4, 1);
end

%% ============== STEP 13: Auxiliary-state update =====================
% Per-channel scalar contraction:
%   xi_alpha_{k+1} = (1 - k_xi_alpha) * xi_alpha_k + delta_u_alpha
xi = (1 - k_xi) .* xi + delta_u;

%% ============== STEP 14: Update persistent variables, return ========
ukm1 = u_bar;     % POST-SATURATION: actually-applied command
u    = u_bar;     % return POST-SATURATION command per user's pipeline

end


%% ====================================================================
%% Helper: closed-form priority-weighted allocation (Plan A+ §3.2)
%% ====================================================================
function [Omega2_star, u_bar] = priority_weighted_allocate(u, Omega2_min, Omega2_max, b, d)
    % Project u onto the per-rotor polytope [Omega2_min, Omega2_max]^4
    % in Omega^2-space, with operational priority M_x, M_y > T > M_z.
    %
    % Inputs:
    %   u                   4x1 unconstrained dSMC command [T; Mx; My; Mz]
    %   Omega2_min, Omega2_max  per-rotor saturation bounds (Omega^2 units)
    %   b, d                mixing-matrix coefficients
    %
    % Returns:
    %   Omega2_star  4x1  post-allocation Omega^2 values (in [min, max]^4)
    %   u_bar        4x1  feasible thrust/moment command corresponding
    %                     to Omega2_star

    T  = u(1);
    Mx = u(2);
    My = u(3);
    Mz = u(4);

    % Channel contributions in Omega^2-space.
    % These are mutually orthogonal (T_M T_M^T = diag(4b^2, 2b^2, 2b^2, 4d^2)),
    % so scaling any one channel by alpha in [0,1] leaves the others unchanged.
    one4 = [ 1;  1;  1;  1];
    e_Mx = [ 0; -1;  0;  1];
    e_My = [-1;  0;  1;  0];
    e_Mz = [ 1; -1;  1; -1];

    c_T  = (T  / (4*b)) * one4;
    c_Mx = (Mx / (2*b)) * e_Mx;
    c_My = (My / (2*b)) * e_My;
    c_Mz = (Mz / (4*d)) * e_Mz;

    % FAST PATH: try unmodified allocation
    Om2 = c_T + c_Mx + c_My + c_Mz;
    if all(Om2 >= Omega2_min - 1e-12) && all(Om2 <= Omega2_max + 1e-12)
        Omega2_star = max(Omega2_min, min(Om2, Omega2_max));
        u_bar = u;
        return;
    end

    % STAGE 1: scale yaw toward zero (lowest priority)
    base = c_T + c_Mx + c_My;
    alpha_Mz = max_feasible_scale_local(base, c_Mz, Omega2_min, Omega2_max);
    Om2 = base + alpha_Mz * c_Mz;
    if all(Om2 >= Omega2_min - 1e-12) && all(Om2 <= Omega2_max + 1e-12)
        Omega2_star = max(Omega2_min, min(Om2, Omega2_max));
        u_bar = [T; Mx; My; alpha_Mz * Mz];
        return;
    end

    % STAGE 2: scale collective thrust toward zero (middle priority)
    base = c_Mx + c_My;
    beta_T = max_feasible_scale_local(base, c_T, Omega2_min, Omega2_max);
    Om2 = beta_T * c_T + base;
    if all(Om2 >= Omega2_min - 1e-12) && all(Om2 <= Omega2_max + 1e-12)
        Omega2_star = max(Omega2_min, min(Om2, Omega2_max));
        u_bar = [beta_T * T; Mx; My; 0];
        return;
    end

    % STAGE 3: scale roll/pitch jointly (deep saturation, last resort)
    %   Reached only when even zero thrust + zero yaw cannot satisfy the
    %   demanded (Mx, My) — measure-zero set of inputs in practice.
    gamma = max_feasible_scale_local(zeros(4,1), c_Mx + c_My, Omega2_min, Omega2_max);
    Om2 = gamma * (c_Mx + c_My);
    Omega2_star = max(Omega2_min, min(Om2, Omega2_max));
    u_bar = [0; gamma * Mx; gamma * My; 0];
end


function alpha = max_feasible_scale_local(base, direction, lo, hi)
    % Largest alpha in [0, 1] such that  base + alpha * direction  lies in
    % [lo, hi]^4 component-wise. Returns 0 if no feasible alpha exists.
    %
    % For each component i:
    %   if direction(i) > 0:  alpha in [(lo - base(i))/dir(i), (hi - base(i))/dir(i)]
    %   if direction(i) < 0:  alpha in [(hi - base(i))/dir(i), (lo - base(i))/dir(i)]
    %                         (interval flipped due to division by negative)
    %   if direction(i) ~= 0: base(i) must already be in [lo, hi]

    alpha_upper = 1.0;
    alpha_lower = 0.0;
    for i = 1:4
        if abs(direction(i)) < 1e-12
            % Component i unaffected by alpha — base alone must be feasible
            if base(i) < lo - 1e-9 || base(i) > hi + 1e-9
                alpha = 0;
                return;
            end
        else
            bound_hi = (hi - base(i)) / direction(i);
            bound_lo = (lo - base(i)) / direction(i);
            if direction(i) > 0
                alpha_upper = min(alpha_upper, bound_hi);
                alpha_lower = max(alpha_lower, bound_lo);
            else
                alpha_upper = min(alpha_upper, bound_lo);
                alpha_lower = max(alpha_lower, bound_hi);
            end
        end
    end

    if alpha_upper < alpha_lower
        alpha = 0;     % infeasible
    else
        alpha = max(alpha_upper, 0);
    end
end