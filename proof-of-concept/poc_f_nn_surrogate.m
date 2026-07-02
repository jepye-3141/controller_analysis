function [theta_star, J_star, meta] = poc_f_nn_surrogate(p_target, lambda, max_new_sweeps, verbose, use_nn)
%POC_F_NN_SURROGATE  GP/NN surrogate + grid argmax (V2 — distributional target).
%   p_target       1x3 numeric (legacy) OR info_target struct (see V2 brief).
%   lambda         weight on reachability_pct (default 0).
%   max_new_sweeps integer budget for verification sweeps (default 0).
%   verbose        logical (default true).
%   use_nn         logical; true => fitrnet([64 64]); false (default) => fitrgp.

if nargin < 1 || isempty(p_target),       p_target = [50 0 0]; end
if nargin < 2 || isempty(lambda),         lambda = 0;          end
if nargin < 3 || isempty(max_new_sweeps), max_new_sweeps = 0;  end
if nargin < 4 || isempty(verbose),        verbose = true;      end
if nargin < 5 || isempty(use_nn),         use_nn = false;      end

info_target = wrap_info_target(p_target);

t_start = tic;

S = load('centroid_lookup_log.mat');
lookup = S.lookup;
ranges = S.ranges;
N = numel(lookup);

X = zeros(N, 6);
J = zeros(N, 1);
valid = false(N, 1);
for n = 1:N
    pp = lookup(n).params;
    X(n,:) = [pp.Vo, pp.el, pp.az, pp.w_z0, pp.w_y0, pp.p];
    pc = lookup(n).p_centroid(:).';
    r  = lookup(n).reachability_pct;
    if isnan(r) || r <= 0 || any(isnan(pc))
        valid(n) = false;
    else
        valid(n) = true;
        J(n) = compute_J(pc, info_target, 100*r, lambda);
    end
end
max_J = max(J(valid));
penalty = 50;
J(~valid) = max_J + penalty;

gate_n = 100;
if use_nn && N < gate_n && verbose
    fprintf('[poc_f] WARNING: N=%d < %d gate; fitrnet expected to underperform.\n', N, gate_n);
end

% HP tuning hooks
n_grid = 11;
kernel = 'ardsquaredexponential';
if isstruct(p_target)
    if isfield(p_target, 'hp_n_grid') && ~isempty(p_target.hp_n_grid)
        n_grid = p_target.hp_n_grid;
    end
    if isfield(p_target, 'hp_kernel') && ~isempty(p_target.hp_kernel)
        kernel = p_target.hp_kernel;
    end
end

rng(0)
if use_nn
    surrogate_type = 'fitrnet';
    mdl = fitrnet(X, J, 'LayerSizes', [64 64], 'Activations', 'relu', ...
                  'Standardize', true);
    cv = crossval(mdl, 'KFold', 5);
else
    surrogate_type = 'fitrgp';
    mdl = fitrgp(X, J, 'KernelFunction', kernel, 'Standardize', true);
    cv = crossval(mdl, 'KFold', 5);
end
J_pred_cv = kfoldPredict(cv);
resid = J - J_pred_cv;
cv_rmse = sqrt(mean(resid.^2));
ss_res = sum(resid.^2);
ss_tot = sum((J - mean(J)).^2);
cv_R2 = 1 - ss_res / max(ss_tot, eps);

Vo_g = linspace(ranges.Vo(1),  ranges.Vo(2),  n_grid);
el_g = linspace(ranges.el(1),  ranges.el(2),  n_grid);
az_g = linspace(ranges.az(1),  ranges.az(2),  n_grid);
p_g  = linspace(ranges.p(1),   ranges.p(2),   n_grid);
wz_mid = mean(ranges.w_z0);
wy_mid = mean(ranges.w_y0);

[VG, EG, AG, PG] = ndgrid(Vo_g, el_g, az_g, p_g);
Xq = [VG(:), EG(:), AG(:), repmat(wz_mid, numel(VG), 1), ...
      repmat(wy_mid, numel(VG), 1), PG(:)];
Jq = predict(mdl, Xq);
[J_pred_min, idx] = min(Jq);
theta_star = Xq(idx, [1 2 3 6]);

if max_new_sweeps > 0
    p_live = build_params(theta_star, wz_mid, wy_mid);
    out = sweep_landing_centroid(p_live, false);
    pc = out.p_centroid(:).';
    r  = out.reachability_pct;
    J_live = compute_J(pc, info_target, 100*r, lambda);
    J_star = J_live;
    n_evals_live = 1;
    J_live_at_theta_star = J_live;
else
    J_star = J_pred_min;
    n_evals_live = 0;
    J_live_at_theta_star = NaN;
end

J_eval = compute_J_eval(theta_star, info_target, lookup);

if cv_R2 < 0.5
    drift_warning = sprintf('low cv_R2=%.3f (<0.5): surrogate underfit at N=%d', cv_R2, N);
else
    drift_warning = '';
end

opts = struct('lambda', lambda, ...
              'max_new_sweeps', max_new_sweeps, 'use_nn', use_nn, ...
              'n_grid', n_grid, 'gate_n', gate_n, ...
              'mu_target', info_target.mu, 'kernel', kernel);

meta = struct( ...
    'n_evals_cached',          N, ...
    'n_evals_live',            n_evals_live, ...
    'wall_time',               toc(t_start), ...
    'surrogate_type',          surrogate_type, ...
    'cv_rmse',                 cv_rmse, ...
    'cv_R2',                   cv_R2, ...
    'J_pred_at_theta_star',    J_pred_min, ...
    'J_live_at_theta_star',    J_live_at_theta_star, ...
    'surrogate_drift_warning', drift_warning, ...
    'J_eval',                  J_eval, ...
    'opts',                    opts);

surrogate = mdl; %#ok<NASGU>
save('poc_f_log.mat', 'theta_star', 'J_star', 'meta', 'surrogate');

if verbose
    fprintf('[poc_f] type=%s N=%d cv_RMSE=%.3f cv_R2=%.3f J*=%.3f J_eval=%.3f wall=%.2fs\n', ...
            surrogate_type, N, cv_rmse, cv_R2, J_star, J_eval, meta.wall_time);
end
end

function J = compute_J(p_centroid, info_target, reach_pct, lambda)
if any(~isfinite(p_centroid)) || reach_pct <= 0
    J = 1000;
    return
end
p_train = info_target.samples_train;
diffs = p_train - p_centroid(:).';
distances = sqrt(sum(diffs.^2, 2));
J = mean(distances) - lambda * (reach_pct / 100);
end

function J_eval = compute_J_eval(theta_star, info_target, lookup)
n = numel(lookup);
theta_mat = zeros(n, 4);
for i = 1:n
    pr = lookup(i).params;
    theta_mat(i, :) = [pr.Vo, pr.el, pr.az, pr.p];
end
S = load('centroid_lookup_log.mat', 'ranges');
range_vec = [diff(S.ranges.Vo), diff(S.ranges.el), diff(S.ranges.az), diff(S.ranges.p)];
d = sqrt(sum(((theta_mat - theta_star) ./ range_vec).^2, 2));
[~, ix] = min(d);
p_c = lookup(ix).p_centroid(:).';
p_eval = info_target.samples_eval;
diffs = p_eval - p_c;
J_eval = mean(sqrt(sum(diffs.^2, 2)));
end

function p = build_params(theta, wz_mid, wy_mid)
p = struct('Vo', theta(1), 'el', theta(2), 'az', theta(3), ...
           'w_z0', wz_mid, 'w_y0', wy_mid, 'p', theta(4), ...
           'alpha_0', 2, 'beta_0', -0.5, ...
           'x_0', 0, 'y_0', 0, 'z_0', 0, 't_max', 300);
end

function info = wrap_info_target(p_target)
if isnumeric(p_target) && numel(p_target) == 3
    p = p_target(:).';
    info = struct('mu', p, 'Sigma', zeros(3), ...
                  'samples_train', p, 'samples_eval', p);
elseif isstruct(p_target)
    info = p_target;
    if ~isfield(info, 'samples_train') || isempty(info.samples_train)
        info.samples_train = info.mu;
    end
    if ~isfield(info, 'samples_eval') || isempty(info.samples_eval)
        info.samples_eval = info.mu;
    end
else
    error('p_target must be 1x3 numeric or info_target struct');
end
end
