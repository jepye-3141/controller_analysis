function forensic = forensic_envelope(traj_params)
% forensic_envelope  Envelope-wide dSMC-sat vs SE(3) divergence forensic.
%
% Runs the ballistic envelope (sweep_ballistic_envelope) with full traces for
% BOTH the dSMC-sat arm and the SE(3) baseline across ALL stations, then pins
% the first irreversible-divergence step t* per station/arm and classifies the
% trigger.
%
% Hypothesis under test (memory project_baseline_controllers_2026-07-17): dSMC
% is feedback-linearization with no switching term, so it should gimbal-lock
% FLIP on the hard ascending stations while SE(3) degrades gracefully and
% survives there.
%
% Verified 2026-07-24 (Thread F): the discriminant is deploy pitch SIGN, not
% station difficulty. The failing nose-up deploys are gentle (|rate_0| about
% 0.8 rad/s), the onset order is rotation-rate first (t* 1.2-4.7 s) with the
% Omega2_min=0 thrust wall downstream (4-28 s), and |theta| passes pi/2 in only
% 3 of 8. So the flip is real but it is not the first thing to go.
%
% traj_params : struct with the operational_launch launch fields
%   (.Vo .el .az .w_z0 .w_y0 .p .alpha_0 .beta_0 .x_0 .y_0 .z_0 .t_max).
%   Defaults to operational_launch() when omitted.
%
% forensic : struct
%   .table     n_points-row envelope-indexed table (one row per station):
%              {station, row, deploy_alt, dsmc_stable, se3_stable,
%               tstar_dsmc, trigger_dsmc, tstar_se3, trigger_se3,
%               divergence_mode, notes}.
%   .out_dsmc  raw sweep_ballistic_envelope out (model "dsmc", both sub-arms).
%   .out_se3   raw sweep_ballistic_envelope out (model "se3").
%
% Trigger classes (per arm, earliest irreversible onset wins):
%   thrust-sign-wall       a rotor pinned at the lower bound Omega2_min=0
%                          through end-of-trace (motors cannot reverse): in the
%                          dSMC the STAGE-3 gamma=0 collapse, in SE(3) the naive
%                          clip's lower clamp. Reconstructed from ctrlout via the
%                          mixer inverse.
%   attitude-gimbal-90deg  integrated pitch |theta| crosses the ZYX singularity
%                          pi/2 and does not return (the dSMC flip).
%   velocity-gate          ||vel|| past the 2*v0 success bound through end
%                          (frame-invariant norm). This is the criterion's soft
%                          landing gate, not a numerical blowup -- SE(3) trips it
%                          while still landing on target.
%   rotation-rate          post-grace ||Euler-rate|| past the 2*RotRefNorm bound
%                          through end -- the clause-(d) flip a bounded, purely
%                          oscillatory (never-past-pi/2) attitude divergence trips.
%   stable                 no trigger, run satisfied the success criterion.
%   drift/finite           failed a success clause but stayed bounded (no wall
%                          / gimbal / speed onset sustained to end-of-trace).
%
% "Irreversible" = the first sample of the final contiguous violating run that
% reaches the end of the logged trace, so a guard-stopped run (trace ends
% diverged) pins its onset, a recovered transient does not, and the front
% zero-pad on ctrlout is ignored (real command follows it).

if nargin < 1
    traj_params = operational_launch();
end

%% Run the envelope with full traces for both arms (one shared ballistic arc)
% model "dsmc" expands to the nosat+sat sub-arm pair; the forensic reads the sat
% sub-arm (Plan A+ + STEP-12b clip, 9/17 -- best of the dSMC variants, below
% SE(3)'s 14/17). The sub-arm pair forces its own saturation_on / unconstrained
% inside sweep_ballistic_envelope, so there is nothing to set here.
tp_dsmc = traj_params;
tp_dsmc.model         = "dsmc";
tp_dsmc.retain_traces = true;
out_dsmc = sweep_ballistic_envelope(tp_dsmc);

tp_se3 = traj_params;
tp_se3.model         = "se3";
tp_se3.retain_traces = true;
out_se3 = sweep_ballistic_envelope(tp_se3);

assert(isequal(out_dsmc.test_points, out_se3.test_points), ...
    'forensic_envelope: dSMC and SE(3) envelopes disagree on station rows');

%% Shared geometry: rebuild the per-station deploy seed for theta0 / v0
% The envelope reads EXACT trajectory rows (no interp), so seed straight from
% test_points; go through ballistic_deploy_state (never the angle cols 13:15).
ballistic_solution = out_dsmc.ballistic_solution;
test_points = out_dsmc.test_points;
n_points    = out_dsmc.n_points;

arm_dsmc = out_dsmc.dsmc_sat;
arm_se3  = out_se3.se3;

%% Per-rotor Omega^2 mixer inverse (same basis as dSMC STEP-12b / apply_rotor_clip)
b_mix = 5; d_mix = 2;
TM_mix = [b_mix b_mix b_mix b_mix; 0 -b_mix 0 b_mix; -b_mix 0 b_mix 0; d_mix -d_mix d_mix -d_mix];
TM_mix_inv = inv(TM_mix); %#ok<MINV>
Omega2_min = 0.0;
clip_tol   = 1e-6;

%% Per-station forensic
station = (1:n_points).';
row     = test_points(:);
[deploy_alt, tstar_dsmc, tstar_se3] = deal(nan(n_points, 1));
[dsmc_stable, se3_stable]           = deal(false(n_points, 1));
[trigger_dsmc, trigger_se3, divergence_mode, notes] = deal(strings(n_points, 1));

for i = 1:n_points
    dp = ballistic_solution.trajectory(test_points(i), :).';   % 15x1 NWU row
    xi = ballistic_deploy_state(dp(1:3), dp(4:6), dp(7:9), dp(10:12));
    theta0        = xi(7);           % deploy pitch, integration seed
    v0_norm       = norm(xi(1:3));   % body-frame deploy speed (frame-invariant)
    deploy_alt(i) = dp(12);          % z position (NWU up)
    ascending     = dp(3) > 0;       % NWU vertical velocity: ascending vs descending

    dsmc_stable(i) = arm_dsmc.mask(i);
    se3_stable(i)  = arm_se3.mask(i);
    [tstar_dsmc(i), trigger_dsmc(i)] = classify_trigger(arm_dsmc.traces(i), ...
        theta0, v0_norm, TM_mix_inv, Omega2_min, clip_tol, dsmc_stable(i));
    [tstar_se3(i), trigger_se3(i)]   = classify_trigger(arm_se3.traces(i), ...
        theta0, v0_norm, TM_mix_inv, Omega2_min, clip_tol, se3_stable(i));

    regime = "(descending)";
    if ascending, regime = "(ascending)"; end
    if dsmc_stable(i) && se3_stable(i)
        divergence_mode(i) = "both-stable";
        notes(i) = regime;
    elseif dsmc_stable(i) && ~se3_stable(i)
        divergence_mode(i) = "dsmc-only";
        notes(i) = "SE(3) " + trigger_se3(i) + " " + regime;
    elseif ~dsmc_stable(i) && se3_stable(i)
        divergence_mode(i) = "se3-survives-crossover";
        notes(i) = "CROSSOVER: dSMC " + trigger_dsmc(i) + "; SE(3) bounded " + regime;
    else
        divergence_mode(i) = "both-fail";
        notes(i) = "dSMC " + trigger_dsmc(i) + " / SE(3) " + trigger_se3(i) + " " + regime;
    end
end

tbl = table(station, row, deploy_alt, dsmc_stable, se3_stable, ...
    tstar_dsmc, trigger_dsmc, tstar_se3, trigger_se3, divergence_mode, notes);

%% Compact printed summary
fprintf('\n== forensic_envelope: dSMC-sat vs SE(3) divergence contrast ==\n');
fprintf('stations: %d   dSMC-sat: %d/%d   SE(3): %d/%d\n', n_points, ...
    arm_dsmc.n_success, n_points, arm_se3.n_success, n_points);
disp(tbl);

% Rank the real crossover: dSMC diverges but SE(3) survives, earliest (most
% severe) dSMC onset first.
xover = find(divergence_mode == "se3-survives-crossover");
[~, ord] = sort(tstar_dsmc(xover));
xover = xover(ord);
fprintf('crossover stations (dSMC diverges, SE(3) survives): %d\n', numel(xover));
for j = 1:numel(xover)
    i = xover(j);
    fprintf('  station %2d (row %3d, alt %6.1f m): dSMC t*=%5.2fs %-22s | %s\n', ...
        station(i), row(i), deploy_alt(i), tstar_dsmc(i), trigger_dsmc(i), notes(i));
end

% dSMC failure-trigger tally (why the failing stations fail).
failed = ~dsmc_stable;
fprintf('dSMC failure triggers: gimbal %d, thrust-wall %d, velocity %d, rotation-rate %d, drift %d\n', ...
    sum(failed & trigger_dsmc == "attitude-gimbal-90deg"), ...
    sum(failed & trigger_dsmc == "thrust-sign-wall"), ...
    sum(failed & trigger_dsmc == "velocity-gate"), ...
    sum(failed & trigger_dsmc == "rotation-rate"), ...
    sum(failed & trigger_dsmc == "drift/finite"));

forensic = struct('table', tbl, 'out_dsmc', out_dsmc, 'out_se3', out_se3);

end

% ------------------------------------------------------------------------
function tstar = onset_time(mask, t)
% Time of the first sample of the FINAL contiguous run of `mask` that reaches
% the end of the trace -- the irreversible-onset convention (a violation that
% never recovers). NaN when the trace ends with mask false (recovered / never
% triggered), which also discards the leading ctrlout zero-pad.
mask = logical(mask(:));
if isempty(mask) || ~mask(end)
    tstar = NaN;
    return
end
last_false = find(~mask, 1, 'last');
if isempty(last_false)
    tstar = t(1);
else
    tstar = t(last_false + 1);
end
end

% ------------------------------------------------------------------------
function [tstar, trigger] = classify_trigger(tr, theta0, v0_norm, TM_inv, Omega2_min, clip_tol, stable)
% Pin t* (first irreversible-divergence step) for one station's trace and label
% the trigger. tr fields (b0_interface): t, pos, vel (NED components, norm-safe),
% rotvel (Tx3 Euler-angle RATES [thetadot phidot psidot]), cmd (Tx4 ctrlout
% [T/f; Mx; My; Mz], absolute thrust, front-zero-padded). A guard-stopped run
% (short trace, ended at the 1e5 chart) ends diverged, so its wall/gimbal/speed
% mask reaches the end and pins the onset; a run that completes but fails only
% the position clause stays bounded and lands in drift/finite.
%
% A station the runner scored stable met every success clause, so no divergence
% trigger applies -- short-circuit (skips the reconstruction on good runs and
% removes the pitch-transient mislabel risk, since a stable station may
% legitimately end with |theta| a touch past pi/2).
if stable
    tstar   = NaN;
    trigger = "stable";
    return
end
t = tr.t;

% Each onset mask coalesces NaN -> violation: a hard blowup Infs then NaNs
% before/at the 1e5 guard, and a bare > comparison would send NaN to false and
% hide the most violent failures (ballistic_success likewise counts NaN as fail).

% (a) thrust-sign wall: reconstruct per-rotor Omega^2 and flag a rotor pinned
%     at the lower bound (motor cutoff / loss of attitude authority).
Om2 = (TM_inv * tr.cmd.').';
tstar_wall = onset_time(any(Om2 <= Omega2_min + clip_tol | isnan(Om2), 2), t);

% (b) gimbal (pitch-primary): integrate pitch rate (col 1) from the deploy
%     pitch; |theta| through the ZYX singularity pi/2 is the dSMC flip. Roll/
%     yaw-driven flips surface via the rotation-rate onset (d) instead.
theta = theta0 + cumtrapz(t, tr.rotvel(:, 1));
tstar_gimbal = onset_time(abs(theta) >= pi/2 | isnan(theta), t);

% (c) velocity gate: ||vel|| past the 2*v0 success bound (ballistic_success
%     VelRatioMax=2), norm frame-invariant despite NED components.
speed = vecnorm(tr.vel(:, 1:3), 2, 2);
tstar_vel = onset_time(speed > 2 * v0_norm | isnan(speed), t);

% (d) rotation-rate blowup: post-grace ||[thetadot;phidot;psidot]|| past the
%     2*RotRefNorm success bound (ballistic_success RotRatioMax=2, RotGraceT=1s).
%     Catches a sustained attitude divergence that never nets pitch past pi/2.
RotRefNorm = norm([pi; pi; pi]);
rot_norm   = vecnorm(tr.rotvel(:, 1:3), 2, 2);
tstar_rot  = onset_time((rot_norm > 2 * RotRefNorm | isnan(rot_norm)) & t > 1, t);

cands  = [tstar_wall, tstar_gimbal, tstar_vel, tstar_rot];
labels = ["thrust-sign-wall", "attitude-gimbal-90deg", "velocity-gate", "rotation-rate"];
if all(isnan(cands))
    tstar   = NaN;
    trigger = "drift/finite";   % failed a clause but never crossed a wall
else
    [tstar, which] = min(cands, [], 'omitnan');
    trigger = labels(which);
end
end
