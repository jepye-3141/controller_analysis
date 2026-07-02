function [theta_star, J_star, meta] = poc_e_nn_prescreen_cem(p_target, lambda, max_new_sweeps, verbose)
%POC_E_NN_PRESCREEN_CEM  GP prescreen + live CEM commits (V2 — distributional target).
%   p_target       1x3 numeric (legacy) OR info_target struct (see V2 brief).
%   lambda         weight on reachability_pct (default 0).
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

theta_fields = {'Vo', 'el', 'az', 'p'};
nuis_fields  = {'w_z0', 'w_y0'};
lo = zeros(1, 4); hi = zeros(1, 4);
for k = 1:4
    lo(k) = ranges.(theta_fields{k})(1);
    hi(k) = ranges.(theta_fields{k})(2);
end
nuis_med = zeros(1, 2);
for k = 1:2
    nuis_med(k) = mean(ranges.(nuis_fields{k}));
end
sigma_min = 0.05 * (hi - lo);

N_cached = numel(lookup);
X_train = zeros(N_cached, 6);
J_train = zeros(N_cached, 1);
for i = 1:N_cached
    pa = lookup(i).params;
    X_train(i, :) = [pa.Vo, pa.el, pa.az, pa.p, pa.w_z0, pa.w_y0];
    J_train(i) = compute_J(lookup(i).p_centroid(:).', info_target, ...
                           100*lookup(i).reachability_pct, lambda);
end
J_max = max(J_train);
zero_reach = arrayfun(@(s) s.reachability_pct == 0, lookup(:));
J_train(zero_reach) = J_max + 1e3;

% HP tuning hook for kernel
kernel = 'ardsquaredexponential';
if isstruct(p_target) && isfield(p_target, 'hp_kernel') && ~isempty(p_target.hp_kernel)
    kernel = p_target.hp_kernel;
end

surrogate = fitrgp(X_train, J_train, ...
    'KernelFunction', kernel, ...
    'BasisFunction', 'constant', ...
    'Standardize', true, ...
    'FitMethod', 'exact', ...
    'PredictMethod', 'exact');
J_pred_train = predict(surrogate, X_train);
ss_res = sum((J_train - J_pred_train).^2);
ss_tot = sum((J_train - mean(J_train)).^2);
surrogate_R2_train = 1 - ss_res / max(ss_tot, eps);

[J_best_cached, ix_best_cached] = min(J_train);
theta_best = X_train(ix_best_cached, 1:4);
J_best = J_best_cached;
J_best_is_live = false;

N_pop = 40;
rho = 0.20;
K_elite = max(2, round(rho * N_pop));
max_iter = 3;
top_commit = 3;

mu_cem = theta_best;
Sigma = diag((0.25 * (hi - lo)).^2);

history = repmat(struct('gen', 0, 'mu', mu_cem, 'Sigma', Sigma, ...
    'n_surrogate_evals', 0, 'n_live_evals', 0, ...
    'best_theta', theta_best, 'best_J', J_best), 1, 0);

n_evals_live = 0;
budget_left = max_new_sweeps;
live_thetas = zeros(0, 4);
live_Js = zeros(0, 1);
live_J_surrogate = zeros(0, 1);

for gen = 1:max_iter
    samples = mvnrnd(mu_cem, Sigma, N_pop);
    samples = min(max(samples, lo), hi);
    X_query = [samples, repmat(nuis_med, N_pop, 1)];
    J_surr = predict(surrogate, X_query);

    [J_surr_sorted, ix_surr] = sort(J_surr, 'ascend');

    n_live_this_gen = 0;
    if budget_left > 0
        n_commit = min(top_commit, budget_left);
        for c = 1:n_commit
            theta_c = samples(ix_surr(c), :);
            params = build_params(theta_c);
            out = sweep_landing_centroid(params, false);
            J_live = compute_J(out.p_centroid(:).', info_target, ...
                               100*out.reachability_pct, lambda);
            n_evals_live = n_evals_live + 1;
            n_live_this_gen = n_live_this_gen + 1;
            budget_left = budget_left - 1;
            live_thetas(end+1, :) = theta_c; %#ok<AGROW>
            live_Js(end+1, 1) = J_live; %#ok<AGROW>
            live_J_surrogate(end+1, 1) = J_surr_sorted(c); %#ok<AGROW>
            if J_live < J_best || ~J_best_is_live
                J_best = J_live;
                theta_best = theta_c;
                J_best_is_live = true;
            end
            if budget_left <= 0, break; end
        end
    end

    elite_thetas = samples(ix_surr(1:K_elite), :);
    mu_cem = mean(elite_thetas, 1);
    if K_elite > 1
        Sigma = cov(elite_thetas);
    else
        Sigma = diag(sigma_min.^2);
    end
    Sigma = enforce_floor(Sigma, sigma_min);

    if ~J_best_is_live && J_surr_sorted(1) < J_best
        J_best = J_surr_sorted(1);
        theta_best = samples(ix_surr(1), :);
    end

    history(end+1) = struct('gen', gen, 'mu', mu_cem, 'Sigma', Sigma, ...
        'n_surrogate_evals', N_pop, 'n_live_evals', n_live_this_gen, ...
        'best_theta', theta_best, 'best_J', J_best); %#ok<AGROW>
end

if size(live_thetas, 1) >= 2
    rho_S = corr(live_J_surrogate, live_Js, 'Type', 'Spearman');
else
    rho_S = NaN;
end

theta_star = theta_best;
J_star = J_best;

J_eval = compute_J_eval(theta_star, info_target, lookup);

meta = struct( ...
    'n_evals_cached',     N_cached, ...
    'n_evals_live',       n_evals_live, ...
    'wall_time',          toc(t_start), ...
    'history',            history, ...
    'surrogate',          surrogate, ...
    'surrogate_R2_train', surrogate_R2_train, ...
    'rho_S',              rho_S, ...
    'J_eval',             J_eval, ...
    'opts', struct('lambda', lambda, ...
                   'max_new_sweeps', max_new_sweeps, 'verbose', verbose, ...
                   'N_pop', N_pop, 'rho', rho, 'max_iter', max_iter, ...
                   'top_commit', top_commit, 'nuis_med', nuis_med, ...
                   'mu_target', info_target.mu, 'kernel', kernel));

save('poc_e_log.mat', 'theta_star', 'J_star', 'meta');

if verbose
    fprintf('[POC_E done] theta*=[%.2f %.2f %.2f %.2f] J*=%.4f J_eval=%.4f wall=%.2fs live=%d\n', ...
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
