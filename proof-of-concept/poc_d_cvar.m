function [theta_star, J_star, meta] = poc_d_cvar(p_target, lambda, max_new_sweeps, verbose, mu)
%POC_D_CVAR  Risk-aware re-ranking of cached samples (V2 — distributional target).
%   p_target       1x3 numeric (legacy) OR info_target struct (see V2 brief).
%   lambda         weight on reachability_pct (default 0).
%   max_new_sweeps ignored.
%   verbose        logical (default true).
%   mu             weight on half_radius spread penalty (default 0).

if nargin < 1 || isempty(p_target),       p_target = [50 0 0]; end
if nargin < 2 || isempty(lambda),         lambda = 0;          end
if nargin < 3 || isempty(max_new_sweeps), max_new_sweeps = 0;  end %#ok<NASGU>
if nargin < 4 || isempty(verbose),        verbose = true;      end
if nargin < 5 || isempty(mu),             mu = 0;              end

info_target = wrap_info_target(p_target);

t0 = tic;

S = load('centroid_lookup_log.mat');
lookup = S.lookup;
N = numel(lookup);

theta_mat = zeros(N, 4);
p_cent    = zeros(N, 3);
reach     = zeros(N, 1);
hr        = zeros(N, 1);
J_dist    = zeros(N, 1);
for i = 1:N
    pr = lookup(i).params;
    theta_mat(i, :) = [pr.Vo, pr.el, pr.az, pr.p];
    p_cent(i, :)    = lookup(i).p_centroid(:).';
    reach(i)        = lookup(i).reachability_pct;
    if isnan(lookup(i).half_radius)
        hr(i) = 0;
    else
        hr(i) = lookup(i).half_radius;
    end
    J_dist(i) = compute_J_pure(p_cent(i,:), info_target);
end

J_cmp  = J_dist - lambda * reach;
J_risk = J_dist + mu * hr - lambda * reach;

% HP tuning hook for mu_grid
mu_grid = [0, 0.5, 1, 2];
if isstruct(p_target) && isfield(p_target, 'hp_mu_grid') && ~isempty(p_target.hp_mu_grid)
    mu_grid = p_target.hp_mu_grid;
end

B = 1000;
top3_count = zeros(N, 1);
rng(0, 'twister');
for b = 1:B
    idx_b = randi(N, N, 1);
    Jb = J_risk(idx_b);
    [~, ord] = sort(Jb);
    top3_idx = unique(idx_b(ord(1:min(3, N))));
    top3_count(top3_idx) = top3_count(top3_idx) + 1;
end
rank_stab = top3_count / B;

[J_risk_min, i_star] = min(J_risk);
theta_star = theta_mat(i_star, :);
J_star = J_cmp(i_star);

n_mu = numel(mu_grid);
theta_at_mu     = zeros(n_mu, 4);
J_distance_at_mu = zeros(n_mu, 1);
half_radius_at_mu = zeros(n_mu, 1);
for m = 1:n_mu
    Jm = J_dist + mu_grid(m) * hr - lambda * reach;
    [~, im] = min(Jm);
    theta_at_mu(m, :)      = theta_mat(im, :);
    J_distance_at_mu(m)    = J_dist(im);
    half_radius_at_mu(m)   = hr(im);
end

J_eval = compute_J_eval(theta_star, info_target, lookup);

idx_col          = (1:N).';
theta_Vo         = theta_mat(:, 1);
theta_el         = theta_mat(:, 2);
theta_az         = theta_mat(:, 3);
theta_p          = theta_mat(:, 4);
J_distance       = J_dist;
half_radius      = hr;
J_risk_col       = J_risk;
rank_stability   = rank_stab;
history = table(idx_col, theta_Vo, theta_el, theta_az, theta_p, ...
                J_distance, half_radius, J_risk_col, rank_stability, ...
                'VariableNames', {'idx', 'theta_Vo', 'theta_el', ...
                'theta_az', 'theta_p', 'J_distance', 'half_radius', ...
                'J_risk', 'rank_stability'});

pareto = struct( ...
    'mu_grid',           mu_grid, ...
    'theta_at_mu',       theta_at_mu, ...
    'J_distance_at_mu',  J_distance_at_mu, ...
    'half_radius_at_mu', half_radius_at_mu);

opts = struct('lambda', lambda, 'max_new_sweeps', 0, ...
              'verbose', verbose, 'mu', mu, ...
              'mu_target', info_target.mu);

meta = struct( ...
    'n_evals_cached', N, ...
    'n_evals_live',   0, ...
    'wall_time',      toc(t0), ...
    'history',        history, ...
    'pareto',         pareto, ...
    'J_eval',         J_eval, ...
    'opts',           opts);

save('poc_d_log.mat', 'theta_star', 'J_star', 'meta');

if verbose
    fprintf('[poc_d_cvar] N=%d, mu=%.2f, J*=%.3f, J_eval=%.3f, J_risk_min=%.3f, wall=%.2fs\n', ...
            N, mu, J_star, J_eval, J_risk_min, meta.wall_time);
end

end

function J = compute_J_pure(p_centroid, info_target)
% MC-mean distance only (no reach penalty here; reach is added by caller)
if any(~isfinite(p_centroid))
    J = 1000;
    return
end
p_train = info_target.samples_train;
diffs = p_train - p_centroid(:).';
distances = sqrt(sum(diffs.^2, 2));
J = mean(distances);
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
