function [theta_star, J_star, meta] = poc_b_cem(p_target, lambda, max_new_sweeps, verbose)
%POC_B_CEM  Cross-Entropy Method outer loop for ballistic targeting (V2 — distributional target).
%   p_target       1x3 numeric (legacy) OR info_target struct (see docs/poc V2 brief).
%   lambda         scalar weight on reachability_pct (default 0).
%   max_new_sweeps integer budget of live sweep_landing_centroid calls (default 0).
%   verbose        logical (default true).

if nargin < 1 || isempty(p_target),       p_target = [50 0 0]; end
if nargin < 2 || isempty(lambda),         lambda = 0;          end
if nargin < 3 || isempty(max_new_sweeps), max_new_sweeps = 0;  end
if nargin < 4 || isempty(verbose),        verbose = true;      end

info_target = wrap_info_target(p_target);

t_start = tic;
rng(0);

S = load('centroid_lookup_log.mat');
lookup = S.lookup;
ranges = S.ranges;

dim_fields = {'Vo', 'el', 'az', 'p'};
lo = zeros(1, 4); hi = zeros(1, 4);
for k = 1:4
    lo(k) = ranges.(dim_fields{k})(1);
    hi(k) = ranges.(dim_fields{k})(2);
end
sigma_min = 0.05 * (hi - lo);

N_cached = numel(lookup);
theta_cached = zeros(N_cached, 4);
J_cached = zeros(N_cached, 1);
for i = 1:N_cached
    pa = lookup(i).params;
    theta_cached(i, :) = [pa.Vo, pa.el, pa.az, pa.p];
    J_cached(i) = compute_J(lookup(i).p_centroid(:).', info_target, ...
                            100*lookup(i).reachability_pct, lambda);
end
zero_reach = arrayfun(@(s) s.reachability_pct == 0, lookup(:));
J_cached(zero_reach) = max(J_cached) + 1e3;

% HP tuning hook for rho
rho = 0.20;
if isstruct(p_target) && isfield(p_target, 'hp_rho') && ~isempty(p_target.hp_rho)
    rho = p_target.hp_rho;
end
K_elite = max(2, round(rho * N_cached));
[J_sorted, idx_sorted] = sort(J_cached, 'ascend');
elite_thetas = theta_cached(idx_sorted(1:K_elite), :);
elite_Js = J_sorted(1:K_elite);

mu_cem = mean(elite_thetas, 1);
Sigma = enforce_floor(cov(elite_thetas), sigma_min);

history = struct('mu', mu_cem, 'Sigma', Sigma, ...
                 'elite_thetas', elite_thetas, 'elite_Js', elite_Js, ...
                 'n_pop', N_cached);

theta_best = theta_cached(idx_sorted(1), :);
J_best = J_sorted(1);

if verbose
    fprintf('[CEM gen 0] cached pool: N=%d, K_elite=%d, J_best=%.4f\n', ...
            N_cached, K_elite, J_best);
end

n_evals_live = 0;
gen = 0;
budget_left = max_new_sweeps;
while budget_left > 0
    gen = gen + 1;
    N_pop = min(6, budget_left);

    samples = zeros(N_pop, 4);
    samples(1, :) = theta_best;
    if N_pop > 1
        samples(2:end, :) = min(max(mvnrnd(mu_cem, Sigma, N_pop - 1), lo), hi);
    end

    J_pop = zeros(N_pop, 1);
    for i = 1:N_pop
        params = build_params(samples(i, :));
        out = sweep_landing_centroid(params, false);
        J_pop(i) = compute_J(out.p_centroid(:).', info_target, ...
                             100*out.reachability_pct, lambda);
        n_evals_live = n_evals_live + 1;
    end

    [J_pop_sorted, ix] = sort(J_pop, 'ascend');
    K_e = max(1, min(N_pop, round(rho * N_pop)));
    elite_thetas = samples(ix(1:K_e), :);
    elite_Js = J_pop_sorted(1:K_e);

    if K_e >= 2
        mu_cem = mean(elite_thetas, 1);
        Sigma = enforce_floor(cov(elite_thetas), sigma_min);
    end

    if J_pop_sorted(1) < J_best
        J_best = J_pop_sorted(1);
        theta_best = elite_thetas(1, :);
    end

    history(end+1) = struct('mu', mu_cem, 'Sigma', Sigma, ...
                            'elite_thetas', elite_thetas, 'elite_Js', elite_Js, ...
                            'n_pop', N_pop); %#ok<AGROW>
    budget_left = budget_left - N_pop;
end

theta_star = theta_best;
J_star = J_best;

J_eval = compute_J_eval(theta_star, info_target, lookup);

meta = struct( ...
    'n_evals_cached', N_cached, ...
    'n_evals_live',   n_evals_live, ...
    'wall_time',      toc(t_start), ...
    'history',        history, ...
    'J_eval',         J_eval, ...
    'opts', struct('lambda', lambda, ...
                   'max_new_sweeps', max_new_sweeps, 'verbose', verbose, ...
                   'mu_target', info_target.mu, 'rho', rho));

save('poc_b_log.mat', 'theta_star', 'J_star', 'meta');

if verbose
    fprintf('[CEM done] theta*=[%.2f %.2f %.2f %.2f] J*=%.4f J_eval=%.4f wall=%.2fs live=%d\n', ...
            theta_star(1), theta_star(2), theta_star(3), theta_star(4), ...
            J_star, J_eval, meta.wall_time, n_evals_live);
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

function S2 = enforce_floor(S, sigma_min)
S2 = (S + S.') / 2;
d = diag(S2);
floor_d = sigma_min(:).^2;
d = max(d, floor_d);
S2(1:size(S2,1)+1:end) = d;
[~, p] = chol(S2);
if p ~= 0
    S2 = diag(d);
end
end

function params = build_params(theta)
params = struct('Vo', theta(1), 'el', theta(2), 'az', theta(3), ...
                'w_z0', 0, 'w_y0', 0, 'p', theta(4), ...
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
