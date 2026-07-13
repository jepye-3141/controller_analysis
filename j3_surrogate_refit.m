% j3_surrogate_refit.m  GP refit on the regenerated lookup log (plan J3 W0/W1).
%
% Requires the j3_lut_regen.m log (needs per-sample radial_profile). Does NOT
% overwrite logs/surrogate_optimize_log.mat: results go to
% logs/surrogate_refit_j3_<hr_mode>.mat. The author promotes a winner to the
% canonical name manually (replot_SO_01.m can then be pointed at it).
%
% Controls (set before running; headless example:
%   matlab -batch "hr_mode='area_proxy'; j3_surrogate_refit"):
%   hr_mode   "clamp"      historical surrogate_optimize.m half_radius
%                          treatment, plus the zero-success correction below
%             "area_proxy" trapz(radial_profile.r_bins, mean_ratio) --
%                          integrated reachability, finite for every sample
%   do_verify false (default) | true: one ground-truth sweep at theta*
%             (~6-16 min), appended to the output log as out_v
%
% Deviations from surrogate_optimize.m, both forced by the new data regime
% (stricter criterion + widened p box can produce zero-success samples):
%   1) zero-success samples (reach==0 => NaN centroid) are masked out of the
%      cx/cy GP training sets (kept for the reach and hr GPs) instead of
%      tripping surrogate_optimize's isfinite assert;
%   2) in clamp mode, a NaN half_radius on a ZERO-success sample is set to 0,
%      not 1.2*max(finite): the historical clamp read every NaN as "wider
%      than the grid", which is exactly wrong when the NaN means "nothing
%      landed at all". (In the 2026-05-09 log, 52/59 samples were NaN of the
%      wide kind -- the clamp dominated the hr training signal, one of the
%      reasons this refit exists.)
% Adds: leave-one-out RMSE per GP (~41 refits per target, a few minutes
% total), a lambda-sweep table tracing the accuracy-vs-spread trade, and a
% held-out-style prediction check at the reference operating point.
% Note: lambda values are NOT comparable across hr_mode (half_radius is
% meters; area_proxy is a ratio-weighted length) -- compare within a mode.

if ~exist('hr_mode', 'var'),   hr_mode   = "clamp"; end
if ~exist('do_verify', 'var'), do_verify = false;   end
hr_mode = string(hr_mode);
assert(any(hr_mode == ["clamp", "area_proxy"]), ...
    "j3_surrogate_refit: hr_mode must be ""clamp"" or ""area_proxy""");
clearvars -except hr_mode do_verify
clc
assert(isfile("constants.m"), "j3_surrogate_refit: run from the ATLIS Sims project root");

%% Load the regenerated lookup log
L = load("logs/centroid_lookup_log.mat");   % lookup, ranges, X, N, field_names (+ N_LHS, wall_s)
lookup      = L.lookup;
ranges      = L.ranges;
field_names = L.field_names;
assert(isfield(lookup, 'radial_profile') && ~isempty(lookup(1).radial_profile), ...
    "j3_surrogate_refit: log lacks radial_profile -- regenerate with j3_lut_regen.m first");

Nl = numel(lookup);
d  = numel(field_names);
Xn = zeros(Nl, d);
for k = 1:d
    fk = field_names{k};
    v  = arrayfun(@(s) s.params.(fk), lookup);
    Xn(:, k) = (v(:) - ranges.(fk)(1)) / diff(ranges.(fk));
end
assert(all(Xn(:) > -1e-9 & Xn(:) < 1 + 1e-9), ...
    "j3_surrogate_refit: sample outside the ranges box -- log/ranges mismatch");

y_cx     = arrayfun(@(s) s.p_centroid(1), lookup); y_cx = y_cx(:);
y_cy     = arrayfun(@(s) s.p_centroid(2), lookup); y_cy = y_cy(:);
y_reach  = [lookup.reachability_pct].';
y_hr_raw = [lookup.half_radius].';

ok_centroid = isfinite(y_cx) & isfinite(y_cy);   % false exactly on zero-success samples
fprintf("samples: %d total, %d zero-success (masked from cx/cy GPs only)\n", ...
    Nl, nnz(~ok_centroid));

switch hr_mode
    case "clamp"
        y_hr   = y_hr_raw;
        hr_max = max(y_hr, [], 'omitnan');
        assert(isfinite(hr_max), ...
            "j3_surrogate_refit: every half_radius is NaN -- use hr_mode=""area_proxy""");
        wide = isnan(y_hr) & y_reach >  0;   % profile never decayed below half peak
        dead = isnan(y_hr) & y_reach == 0;   % nothing landed at all
        y_hr(wide) = 1.2 * hr_max;
        y_hr(dead) = 0;
        fprintf("clamp mode: %d wide-NaN -> %.1f m, %d zero-success-NaN -> 0\n", ...
            nnz(wide), 1.2 * hr_max, nnz(dead));
    case "area_proxy"
        y_hr = arrayfun(@(s) area_from_profile(s.radial_profile), lookup);
        y_hr = y_hr(:);
        assert(all(isfinite(y_hr)), "j3_surrogate_refit: area_proxy produced non-finite values");
end

%% Fit GPs (same settings as surrogate_optimize.m)
gp_opts  = {'KernelFunction', 'ardmatern52', 'Standardize', true, 'BasisFunction', 'constant'};
gp_cx    = fitrgp(Xn(ok_centroid, :), y_cx(ok_centroid), gp_opts{:});
gp_cy    = fitrgp(Xn(ok_centroid, :), y_cy(ok_centroid), gp_opts{:});
gp_reach = fitrgp(Xn, y_reach, gp_opts{:});
gp_hr    = fitrgp(Xn, y_hr,    gp_opts{:});

%% Leave-one-out RMSE (and per-point LOO predictions)
loo = struct();
[loo.cx,    pred_cx]    = loo_rmse(Xn(ok_centroid, :), y_cx(ok_centroid), gp_opts);
[loo.cy,    pred_cy]    = loo_rmse(Xn(ok_centroid, :), y_cy(ok_centroid), gp_opts);
[loo.reach, pred_reach] = loo_rmse(Xn, y_reach, gp_opts);
[loo.hr,    pred_hr]    = loo_rmse(Xn, y_hr,    gp_opts);
fprintf("LOO-RMSE: cx %.1f m, cy %.1f m, reach %.3f, hr[%s] %.2f\n", ...
    loo.cx, loo.cy, loo.reach, hr_mode, loo.hr);

%% Multi-start SQP over a lambda sweep (headline lambda = 50)
p_target = [600; 0; 0];
lambdas  = [0 10 25 50 100 200];
rng(0)
n_starts  = 8;
starts    = lhsdesign(n_starts, d);
fmin_opts = optimoptions('fmincon', 'Display', 'off', ...
                         'Algorithm', 'sqp', 'MaxFunctionEvaluations', 500);

lam_template            = struct();
lam_template.lambda     = NaN;
lam_template.theta      = [];
lam_template.theta_phys = [];
lam_template.J          = NaN;
lam_template.cx         = NaN;
lam_template.cy         = NaN;
lam_template.miss       = NaN;
lam_template.hr         = NaN;
lam_template.reach      = NaN;
lam_results = repmat(lam_template, numel(lambdas), 1);
for li = 1:numel(lambdas)
    lam = lambdas(li);
    Jf  = @(t) (predict(gp_cx, t) - p_target(1))^2 + ...
               (predict(gp_cy, t) - p_target(2))^2 - lam * predict(gp_hr, t);
    J_b = inf;  t_b = [];
    for s = 1:n_starts
        [t_s, J_s] = fmincon(Jf, starts(s, :), [], [], [], [], ...
                             zeros(1, d), ones(1, d), [], fmin_opts);
        if J_s < J_b, J_b = J_s; t_b = t_s; end
    end
    tp = struct();
    for k = 1:d
        tp.(field_names{k}) = ranges.(field_names{k})(1) + t_b(k) * diff(ranges.(field_names{k}));
    end
    cx_p = predict(gp_cx, t_b);  cy_p = predict(gp_cy, t_b);
    % per-field assignment avoids the dissimilar-structures error that
    % lam_results(li) = struct(...) risks if field order ever drifts
    lam_results(li).lambda     = lam;
    lam_results(li).theta      = t_b;
    lam_results(li).theta_phys = tp;
    lam_results(li).J          = J_b;
    lam_results(li).cx         = cx_p;
    lam_results(li).cy         = cy_p;
    lam_results(li).miss       = norm([cx_p; cy_p] - p_target(1:2));
    lam_results(li).hr         = predict(gp_hr, t_b);
    lam_results(li).reach      = predict(gp_reach, t_b);
    fprintf("lambda=%5.0f: miss=%6.1f m  hr=%7.1f  reach=%.2f | Vo=%.1f el=%.1f az=%.1f w_z0=%.2f w_y0=%.2f p=%.2f\n", ...
        lam, lam_results(li).miss, lam_results(li).hr, lam_results(li).reach, ...
        tp.Vo, tp.el, tp.az, tp.w_z0, tp.w_y0, tp.p);
end
i50        = find(lambdas == 50, 1);
theta_best = lam_results(i50).theta;
theta_phys = lam_results(i50).theta_phys;
J_best     = lam_results(i50).J;

%% Honest check at the reference operating point (LOO prediction vs truth)
i_ref = find([lookup.is_reference], 1);
if ~isempty(i_ref)
    % index of the reference row inside the centroid-masked set
    idx_masked = cumsum(ok_centroid);
    rp_ = lookup(i_ref).params;   % print the actual reference launch, not a stale literal
    fprintf("\n--- reference operating point (Vo=%g, el=%g, az=%g, p=%g) ---\n", ...
        rp_.Vo, rp_.el, rp_.az, rp_.p);
    if ok_centroid(i_ref)
        fprintf("  LOO pred (cx,cy) = (%.1f, %.1f)  vs actual (%.1f, %.1f)\n", ...
            pred_cx(idx_masked(i_ref)), pred_cy(idx_masked(i_ref)), y_cx(i_ref), y_cy(i_ref));
    end
    fprintf("  LOO pred reach   = %.3f  vs actual %.3f\n", pred_reach(i_ref), y_reach(i_ref));
    fprintf("  LOO pred hr[%s] = %.1f  vs actual %.1f\n", hr_mode, pred_hr(i_ref), y_hr(i_ref));
end

%% Save (non-canonical -- author promotes a winner manually)
out_path = "logs/surrogate_refit_j3_" + hr_mode + ".mat";
save(out_path, "gp_cx", "gp_cy", "gp_reach", "gp_hr", ...
     "theta_best", "theta_phys", "J_best", "p_target", ...
     "lambdas", "lam_results", "loo", "ranges", "field_names", "Xn", ...
     "y_cx", "y_cy", "y_reach", "y_hr", "ok_centroid", "hr_mode");
fprintf("wrote %s\n", out_path);

%% Optional ground-truth verification at theta_best (lambda = 50)
if do_verify
    load_system("discrete_smc_swarm_single")
    addpath("Ballistics_Simulation-master/")
    vp         = theta_phys;
    vp.alpha_0 = 2;  vp.beta_0 = -0.5;
    vp.x_0     = 0;  vp.y_0    = 0;   vp.z_0 = 0;
    vp.t_max   = 300;
    out_v = sweep_landing_centroid(vp, false);
    fprintf("\n--- ground-truth verification at theta* ---\n");
    fprintf("  actual (cx, cy) = (%.2f, %.2f)   pred (%.2f, %.2f)   err %.1f m\n", ...
        out_v.p_centroid(1), out_v.p_centroid(2), lam_results(i50).cx, lam_results(i50).cy, ...
        norm(out_v.p_centroid(1:2) - [lam_results(i50).cx; lam_results(i50).cy]));
    fprintf("  actual reach    = %.3f   pred %.3f\n", out_v.reachability_pct, lam_results(i50).reach);
    save(out_path, "out_v", "-append");
end

%% ---- local functions ----
function a = area_from_profile(rp)
    % NaN-safe integrated reachability of a radial profile.
    m = isfinite(rp.r_bins(:)) & isfinite(rp.mean_ratio(:));
    if nnz(m) < 2
        a = 0;
    else
        a = trapz(rp.r_bins(m), rp.mean_ratio(m));
    end
end

function [r, pred] = loo_rmse(X, y, gp_opts)
    % Leave-one-out RMSE with per-fold hyperparameter refit.
    n    = numel(y);
    pred = nan(n, 1);
    for i = 1:n
        m    = true(n, 1);  m(i) = false;
        g    = fitrgp(X(m, :), y(m), gp_opts{:});
        pred(i) = predict(g, X(i, :));
    end
    r = sqrt(mean((pred - y).^2));
end
