function [success, info] = ballistic_success(t, pos, vel, target_xy, v0_norm, expected_T, opts)
% ballistic_success  Post-processing success criterion for STABILIZE trials.
%
% The paper's stabilization/landing success criterion (eq:ballistic-success),
% applied to logged sim data after the run. The in-model termination chart is
% NOT the criterion -- a numerical-blowup guard (norms >= 1e5) that stops
% destabilized sims early; a guard-stopped trial fails condition (a) below.
% Callers: analysis.m envelope (30 s, all five arms) and
% sweep_landing_centroid (60 s sweep trials).
%
%   success = (a) && (b) && (c) && (d), where
%     (a) duration:  t(end) >= expected_T - 1e-6   (guard never tripped)
%     (b) position:  ||pos(end,1:2)' - target_xy|| <= ReachTol  (final time)
%     (c) velocity:  max_t ||vel(t,:)|| <= VelRatioMax * v0_norm
%                    (every logged sample; skipped when vel is empty)
%     (d) rotation:  max_{t > RotGraceT} ||RotVel(t,:)|| <= RotRatioMax * RotRefNorm
%                    (skipped when RotVel is empty. RotVel is the models'
%                    rotvelout: plant Euler-angle rates [thetadot phidot
%                    psidot], not body rates -- the same signal the
%                    termination charts norm. The grace window exempts the
%                    ballistic-separation arrest transient (deployment
%                    hands the vehicle over tumbling); empirically
%                    (2026-07-03 sat_on sweep, pre-bugsweep ballistics,
%                    Vo=100/el=45 launch) sweep violations started by
%                    t~=0.06 s and none persisted past 3.8 s, so 1 s
%                    excludes the transient only.)
%
% Inputs:
%   t          Tx1 time vector of the logged series
%   pos        Tx3 (or Tx>=2) leader position trace
%   vel        Tx3 leader velocity trace; pass [] to skip condition (c)
%   target_xy  target horizontal position (first two elements used)
%   v0_norm    norm of the deployment velocity (frame-invariant), > 0
%   expected_T configured StopTime of the trial (s)
%
% Options (name-value):
%   ReachTol    = 10                  final-position tolerance (m)
%   VelRatioMax = 2                   bound on ||v_t|| / ||v_0||
%   RotRatioMax = 2                   bound on ||w_t|| / RotRefNorm
%   RotRefNorm  = norm([pi; pi; pi])  reference rate magnitude (rad/s)
%   RotGraceT   = 1                   grace window (s): rotation bound is
%                                     evaluated only for t > RotGraceT
%   RotVel      = []                  rate trace on the same time grid as t,
%                                     when available. Pass the raw logged
%                                     rotvelout.Data: both the 3-D [3x1xT]
%                                     (lqr/dsmc) and 2-D [Tx3] (pid/smc)
%                                     shapes are normalized internally
%
% Outputs:
%   success    logical
%   info       struct with per-condition flags (duration_ok, position_ok,
%              velocity_ok, rotation_ok), *_checked flags marking skipped
%              conditions, and measured extrema (t_end, final_miss,
%              max_speed[_ratio]; max_rot[_ratio] is whole-trace,
%              max_rot_post[_ratio] is the criterion's post-grace value).

arguments
    t (:,1) double
    pos (:,:) double
    vel double
    target_xy (:,1) double
    v0_norm (1,1) double {mustBePositive}
    expected_T (1,1) double {mustBePositive}
    opts.ReachTol (1,1) double = 10
    opts.VelRatioMax (1,1) double = 2
    opts.RotRatioMax (1,1) double = 2
    opts.RotRefNorm (1,1) double = norm([pi; pi; pi])
    opts.RotGraceT (1,1) double {mustBeNonnegative} = 1
    opts.RotVel double = []
end

info = struct();

% (a) full duration -- an early stop means the blowup guard fired
info.t_end = t(end);
info.duration_ok = t(end) >= expected_T - 1e-6;

% (b) final horizontal position within tolerance
info.final_miss = norm(pos(end, 1:2).' - target_xy(1:2));
info.position_ok = info.final_miss <= opts.ReachTol;

% (c) velocity ratio over the whole trace
info.velocity_checked = ~isempty(vel);
if info.velocity_checked
    info.max_speed = max(vecnorm(vel(:, 1:3), 2, 2), [], 'includenan');  % NaN sample must fail, not vanish (audit C4)
    info.max_speed_ratio = info.max_speed / v0_norm;
    info.velocity_ok = info.max_speed_ratio <= opts.VelRatioMax;
else
    info.max_speed = NaN;
    info.max_speed_ratio = NaN;
    info.velocity_ok = true;
end

% (d) rotation-rate ratio after the grace window (when rates are logged).
% max_rot[_ratio] is whole-trace diagnostics; the criterion uses the
% post-window extremum. No post-window samples -> (d) passes vacuously
% (such a trial fails (a) anyway).
info.rotation_checked = ~isempty(opts.RotVel);
info.rot_grace_T = opts.RotGraceT;
if info.rotation_checked
    % Accept the raw logged rotvelout array: lqr/dsmc log 3-D [3x1xT],
    % pid/smc 2-D [Tx3]. Orient by raw ndims, NOT a size==3 heuristic
    % (mis-orients [3x1xT] when T==3 -- bugsweep 2026-07-10 finding 12).
    if ndims(opts.RotVel) == 3
        opts.RotVel = squeeze(opts.RotVel).';
    end
    assert(size(opts.RotVel, 1) == numel(t), ...
        'ballistic_success:rotvel_grid', ...
        'RotVel has %d rows but t has %d samples -- traces must share a grid', ...
        size(opts.RotVel, 1), numel(t));
    rot_norms = vecnorm(opts.RotVel(:, 1:3), 2, 2);
    info.max_rot = max(rot_norms, [], 'includenan');
    info.max_rot_ratio = info.max_rot / opts.RotRefNorm;
    post = rot_norms(t > opts.RotGraceT);
    if isempty(post)
        info.max_rot_post = NaN;
        info.max_rot_post_ratio = NaN;
        info.rotation_ok = true;
    else
        info.max_rot_post = max(post, [], 'includenan');  % NaN sample must fail clause (d) (audit C4)
        info.max_rot_post_ratio = info.max_rot_post / opts.RotRefNorm;
        info.rotation_ok = info.max_rot_post_ratio <= opts.RotRatioMax;
    end
else
    info.max_rot = NaN;
    info.max_rot_ratio = NaN;
    info.max_rot_post = NaN;
    info.max_rot_post_ratio = NaN;
    info.rotation_ok = true;
end

success = info.duration_ok && info.position_ok && info.velocity_ok && info.rotation_ok;
info.success = success;

end
