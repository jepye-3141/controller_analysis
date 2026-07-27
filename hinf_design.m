% hinf_design.m -- OFFLINE H-infinity mixed-sensitivity design (run ONCE).
% ---------------------------------------------------------------------------
% Full MATLAB (NOT codegen). The operator runs this, reads the printed
% Ad_K/Bd_K/Cd_K/Dd_K literals, and pastes them into hinf_controller.m (the
% codegen-safe runtime law). Requires Control System + Robust Control toolboxes.
%
% Pipeline:
%   1. hover-linearized 12-state model (mirror of constants.m).
%   2. build the measurement-basis output matrix C_meas (THE roll/pitch swap).
%   3. mixed-sensitivity synthesis  [K,CL,gamma] = mixsyn(G, W1, W2, W3).
%   4. validate closed-loop stability + feedback sign on the TRUE plant
%      (FAIL CLOSED: errors out rather than emitting a destabilizing controller).
%   5. dominant-mode settling vs the 30 s envelope horizon (position-reach check).
%   6. discretize K at 50 Hz (Tustin) and print paste-ready numeric literals
%      behind an emit gate that rejects a non-finite / excessive gamma.
%
% Design source of truth for the H-inf arm: docs/controller_baselines_plan.md
% section 5.1. Runtime law: hinf_controller.m.
% ---------------------------------------------------------------------------

clear; clc;

%% 1. Hover-linearized plant (mirror of constants.m; kept in sync by hand)
% Replicated here (not `run constants.m`) so this design script has no Simulink
% bus side-effects and does not clobber constants.m's LQR `Kd`. If the physical
% parameters or A/B/C in constants.m change, mirror the change here.
g   = 9.81;      % m/s^2
l   = 0.2;       % m   (NOTE: moments are DIRECT inputs -- no length arm in B)
m   = 0.8;       % kg  (nominal; controller-side mass is constants.m_uncertain)
Jxx = 1.8e-3;    % kg m^2
Jyy = 1.8e-3;    % kg m^2
Jzz = 1.5e-3;    % kg m^2

% State z = [vx vy vz  wx wy wz  theta phi psi  X Y Z]; input u = [dT Mx My Mz]
% (u1 = thrust DELTA from hover). Identical to the A/B/C block in constants.m.
A = [0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 -1 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0;
     0 0 0 0;
     -1/m 0 0 0;
     0 1/Jxx 0 0;
     0 0 1/Jyy 0;
     0 0 0 1/Jzz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];
% constants.m C -- observability-check artifact, ordered [X Y Z theta phi psi]
% (theta BEFORE phi). This is NOT the runtime measurement order.
C = [0 0 0 0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 1 0 0 0];
D = zeros(6, 4);

dt = 1/50;   % 50 Hz discrete control rate (constants.dt); Nyquist ~ pi/dt = 157 rad/s

%% 2. Measurement-basis output matrix  === THE ROLL/PITCH SWAP (LOAD-BEARING) ===
% ---------------------------------------------------------------------------
% The runtime Selector [10 11 12 8 7 9] hands the controller
%     state = [X Y Z  phi theta psi]   (phi=roll BEFORE theta=pitch),
% whereas constants.m's C outputs [X Y Z theta phi psi] (theta before phi).
% They differ ONLY by swapping output rows 4 and 5. We MUST synthesize K in the
% runtime measurement basis, so swap rows 4<->5 of C here. If this swap is
% omitted, roll and pitch are transposed at runtime and the loop is UNSTABLE.
% ---------------------------------------------------------------------------
C_meas = C([1 2 3 5 4 6], :);   % -> outputs [X Y Z phi theta psi]

G = ss(A, B, C_meas, D);        % 4 inputs [dT Mx My Mz], 6 outputs [X Y Z phi theta psi]
fprintf('Plant G: %d states, %d inputs, %d outputs.\n', ...
        size(A,1), size(B,2), size(C_meas,1));

%% 3. Mixed-sensitivity weights (STARTING POINT -- tune per the notes)
% This plant is UNDERACTUATED (4 inputs, 6 outputs): world X,Y are reached only
% by tilting (theta,phi), so the 6 outputs are not independently reachable at
% every frequency. mixsyn still returns an internally-stabilizing K; gamma may
% sit a little above 1 (that is acceptable here -- the pass criterion is a
% stable loop that clears ballistic_success.m, NOT gamma<1). Tuning philosophy:
%   * position (X,Y,Z) loops are SLOW  (each is a ~1/s^4 chain moment->tilt->pos)
%   * attitude (phi,theta,psi) loops are FASTER (inner loop; ~1/s^2)
%
% --- W1 : performance weight on S = inv(I+GK) (tracking / low-freq rejection).
%     CRITICAL: performance (integral action) is demanded only on the FOUR
%     independently-reachable outputs -- X,Y,Z (thrust + tilt) and psi (yaw).
%     Roll/pitch (phi,theta) are the INTERNAL tilt DOFs used to translate X,Y;
%     they cannot be independently held at a reference while ALSO placing X,Y
%     (the plant is underactuated: 4 inputs, 6 outputs). Demanding S->0 on
%     phi,theta makes the mixed-sensitivity problem near-infeasible and pins
%     gamma ~ 1/eps1 ~ 1e4. So weight phi,theta only LIGHTLY (bounded
%     regulation, NO integral demand) and let the position loop drive them.
%     Pseudo-integrator: ~1/eps low-freq gain (=> S small => good DC tracking),
%     corner at wb, high-freq gain 1/Ms (=> S peak <= Ms). Raise wb* for speed,
%     raise Ms toward 2 for robustness/feasibility, lower eps for tighter DC.
Ms    = 2.0;     % max sensitivity peak (1.5-2.0 typical)
eps1  = 1e-4;    % DC sensitivity floor -> steady-state tracking error
wbP   = 1.0;     % position (X,Y,Z) performance bandwidth [rad/s]  (keep modest)
wbPsi = 4.0;     % yaw (psi) performance bandwidth [rad/s]
w1RP  = 0.5;     % roll/pitch (phi,theta): LIGHT CONSTANT weight, no integral demand
w1P   = tf([1/Ms  wbP],   [1  wbP*eps1]);
w1Y   = tf([1/Ms  wbPsi], [1  wbPsi*eps1]);
W1    = append(w1P, w1P, w1P, w1RP, w1RP, w1Y);   % diag, order [X Y Z phi theta psi]

% --- W2 : control-effort weight on KS (keeps demanded thrust/moments within
%     rotor authority so apply_rotor_clip.m does not dominate). Constant diagonal
%     (0 extra controller states), scaled by inverse per-channel authority.
%     Authority (see apply_rotor_clip.m; Omega2 in [0,2], b=5, d=2):
%       thrust-delta dT: T in [0,40] N, hover m*g=7.85 => usable |dT| ~ 30 N
%       Mx,My = b*dOmega2 in [-10,10] N*m ;  Mz = d*(...) in [-8,8] N*m
%     Raise ru if the loop demands infeasible authority (clip saturates); lower
%     it if the response is sluggish.
ru   = 0.10;                       % overall control penalty knob
au   = [1/30; 1/10; 1/10; 1/8];    % inverse authority [dT; Mx; My; Mz]
W2   = ru * diag(au);              % constant 4x4  (for explicit KS roll-off make
                                   % this a high-pass tf per channel instead)

% --- W3 : robustness weight on T = I-S (high-freq roll-off -> gain/phase margin,
%     sensor-noise & unmodeled-dynamics rejection). High-pass: DC gain 1/Mt
%     (allow T~1 for tracking), high-freq gain 1/eps3 (force T->0). Lower wbT to
%     buy robustness (slower loop); keep wbT well under Nyquist 157 rad/s.
Mt   = 2.0;      % max complementary-sensitivity peak
eps3 = 1e-3;     % (1/eps3) = HF weight -> how hard T is pushed to 0
wbT  = 30;       % T roll-off corner [rad/s]  (<< Nyquist)
w3   = tf([1  wbT/Mt], [eps3  wbT]);
W3   = append(w3, w3, w3, w3, w3, w3);

%% 4. Synthesis
% This plant has 12 poles at the origin (pure integrator chains). Modern
% mixsyn/hinfsyn handles jw-axis plant poles, but if it errors on numerical
% conditioning we retry once with a tiny 1e-4 LHP pole nudge (physically
% negligible) purely to move the poles off the axis for the solver.
Gsyn = G;
try
    [K, CL, gamma] = mixsyn(Gsyn, W1, W2, W3);
catch ME
    warning('hinf_design:nudge', ...
        ['mixsyn failed on the raw integrator plant (%s).\n', ...
         'Retrying with a 1e-4 LHP pole nudge for conditioning.'], ME.message);
    Gsyn = ss(A - 1e-4*eye(12), B, C_meas, D);
    [K, CL, gamma] = mixsyn(Gsyn, W1, W2, W3);
end
fprintf('\nmixsyn gamma = %.4f  (aim <~ 1; a little above is OK on this non-square plant)\n', gamma);

%% 5. Validate on the TRUE plant + resolve the feedback sign
% mixsyn's K is built for the standard NEGATIVE-feedback loop with the controller
% in the FORWARD path: u = K*(r - y), i.e. u = K*(xd - state). That is EXACTLY
% the convention hinf_controller.m uses -- so normally no sign flip is needed.
% We still verify internal stability on the un-nudged plant, and (defensively)
% try a global sign flip if the loop is unstable; any flip is folded into the
% printed matrices so the runtime code stays a plain u = K*(xd - state).
loops = loopsens(G, K);
signFlipped = false;
if ~loops.Stable
    Kf = -K;
    loopsf = loopsens(G, Kf);
    if loopsf.Stable
        K = Kf;  loops = loopsf;  signFlipped = true;
        warning('hinf_design:signflip', ...
            'Applied a global sign flip to K to satisfy u=K*(xd-state).');
    else
        % FAIL CLOSED: never fall through to print/save destabilizing matrices.
        % (mixsyn normally returns a stabilizing K, so this is a defensive guard;
        % erroring here stops execution before Section 6 emits anything.)
        error('hinf_design:unstable', ...
            ['Closed loop is UNSTABLE on the true plant with BOTH sign conventions.\n', ...
             'NO matrices were emitted. Retune weights (raise ru, lower wbP) and re-run.']);
    end
end
clpoles = pole(loops.To);
fprintf('Internally stable on true plant : %d   (sign flipped: %d)\n', loops.Stable, signFlipped);
fprintf('Max closed-loop pole real part  : %.4g   (must be < 0)\n', max(real(clpoles)));

% Feedback-sign sanity check: with integral action, To(0) ~ I, so each diagonal
% DC gain should be ~ +1. The cleanly-actuated channels Z (idx 3) and psi (idx 6)
% are the reliable tell; the underactuated X,Y,phi,theta may read less cleanly.
dcTo = diag(dcgain(loops.To));
lbl  = {'X','Y','Z','phi','theta','psi'};
fprintf('DC tracking gains diag(To(0)) -- want ~ +1 (esp. Z, psi):\n');
for i = 1:6
    fprintf('   %-6s % .4f\n', lbl{i}, dcTo(i));
end
if dcTo(3) < 0 || dcTo(6) < 0
    warning('hinf_design:tracksign', ...
        ['Z and/or psi DC tracking gain is NEGATIVE -- feedback sense looks wrong.\n', ...
         'Inspect the C_meas swap and the u=K*(xd-state) convention before use.']);
end

%% 5b. Dominant-mode settling vs the envelope trial horizon (position-reach check)
% The world X/Y loop is a long moment->tilt->accel->position integrator chain, so
% the dominant closed-loop pole is slow. This QUANTIFIES whether the design can
% actually reach the 10 m tolerance inside a 30 s envelope trial (the ballistic
% harness horizon) -- otherwise the H-inf arm may clear the "bounded/stabilize"
% ballistic_success criterion yet still MISS the reach tolerance. Uses only the
% already-computed closed-loop poles (no extra toolbox dependency).
domReal = max(real(clpoles));       % least-negative stable pole = dominant mode
tau_dom = -1 / domReal;             % dominant time constant [s]  (domReal < 0 here)
ts_est  = 4 * tau_dom;              % ~2% settling time [s]
Ttrial  = 30;                       % envelope trial StopTime [s] (analysis.m /
                                    % sweep_ballistic_envelope; the landing sweep
                                    % in sweep_landing_centroid runs 60 s, so this
                                    % is the tighter of the two horizons)
fprintf('Dominant CL time constant       : %.2f s   (~%.0f s 2%%-settling)\n', tau_dom, ts_est);
if ts_est > Ttrial
    warning('hinf_design:slowpos', ...
        ['Dominant closed-loop settling ~%.0f s EXCEEDS the %g s envelope trial horizon.\n', ...
         'The H-inf arm will likely MISS the 10 m reach tolerance within a trial\n', ...
         '(it may still pass the bounded/stabilize ballistic_success criterion).\n', ...
         'To speed the position loop up: raise wbP (currently %.2f) toward 3-6 and/or\n', ...
         'lower ru (currently %.2f) toward 0.03-0.05, then RE-RUN and confirm gamma\n', ...
         'stays O(1-10) and the loop stays stable. Final tuning must be validated in\n', ...
         'the real harness (system_dynamics + ballistic_success at 30 s), not just here.'], ...
         ts_est, Ttrial, wbP, ru);
end

%% 6. Discretize (Tustin @ 50 Hz) and emit paste-ready literals
% EMIT GATE (fail closed): never print/save matrices from a broken synthesis.
% Internal stability is already enforced above (the both-signs-unstable branch
% errors out); here we additionally reject a non-finite or excessive gamma, which
% signals near-infeasible weights (e.g. demanding integral tracking on the
% underactuated phi/theta pins gamma ~ 1e4). A well-posed design on this
% non-square plant sits at gamma ~ O(1-10).
if ~isfinite(gamma) || gamma > 1e3
    error('hinf_design:badgamma', ...
        ['mixsyn gamma = %.4g is non-finite or excessive (weights near-infeasible).\n', ...
         'NO matrices were emitted. Retune the weights before re-running.'], gamma);
end
% NB: this `Kd` shadows constants.m's LQR gain name -- irrelevant here (this
% script never uses the LQR Kd).
Kd = c2d(K, dt, 'tustin');
[Ad_K, Bd_K, Cd_K, Dd_K] = ssdata(Kd);
nK = size(Ad_K, 1);

% (Optional) order reduction if nK is inconveniently large for the block:
%   Kr = balred(Kd, 14);  [Ad_K,Bd_K,Cd_K,Dd_K] = ssdata(Kr); nK = size(Ad_K,1);
%   -- then RE-RUN loopsens(G, d2c(Kr,'tustin')) to confirm it still stabilizes.

fprintf('\n==================================================================\n');
fprintf(' PASTE THE FOLLOWING INTO hinf_controller.m  (nK = %d)\n', nK);
if signFlipped
    fprintf(' (a global sign flip is already folded into these matrices)\n');
end
fprintf('==================================================================\n');
fprintf('Ad = %s;\n', mat2str(Ad_K, 16));
fprintf('Bd = %s;\n', mat2str(Bd_K, 16));
fprintf('Cd = %s;\n', mat2str(Cd_K, 16));
fprintf('Dd = %s;\n', mat2str(Dd_K, 16));
fprintf('==================================================================\n');

%% 7. Save
if ~exist('logs', 'dir'); mkdir('logs'); end
save(fullfile('logs', 'hinf_design.mat'), ...
     'Ad_K', 'Bd_K', 'Cd_K', 'Dd_K', 'gamma', 'nK', 'signFlipped', ...
     'W1', 'W2', 'W3', 'K', 'Kd');
fprintf('Saved logs/hinf_design.mat (Ad_K,Bd_K,Cd_K,Dd_K,gamma,nK,signFlipped).\n');
