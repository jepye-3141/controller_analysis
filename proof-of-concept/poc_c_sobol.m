function [theta_star, J_star, meta] = poc_c_sobol(p_target, lambda, max_new_sweeps, verbose)
%POC_C_SOBOL  Sobol' sensitivity over cached LHS for ballistic targeting (V2 — distributional target).
%   p_target       1x3 numeric (legacy) OR info_target struct (see V2 brief).
%   lambda         weight on reachability_pct (default 0).
%   max_new_sweeps integer; THIS POC IGNORES max_new_sweeps.
%   verbose        logical (default true).

if nargin < 1 || isempty(p_target),       p_target = [50 0 0]; end
if nargin < 2 || isempty(lambda),         lambda = 0;          end
if nargin < 3 || isempty(max_new_sweeps), max_new_sweeps = 0;  end %#ok<NASGU>
if nargin < 4 || isempty(verbose),        verbose = true;      end

info_target = wrap_info_target(p_target);

t0 = tic;
S = load('centroid_lookup_log.mat');
lookup = S.lookup;
ranges = S.ranges;
dims   = {'Vo','el','az','w_z0','w_y0','p'};
N      = numel(lookup);
d      = numel(dims);

X_phys = zeros(N, d);
J      = zeros(N, 1);
reach  = zeros(N, 1);
p_cent = zeros(N, 3);
for n = 1:N
    for k = 1:d
        X_phys(n,k) = lookup(n).params.(dims{k});
    end
    p_cent(n,:)  = lookup(n).p_centroid(:).';
    reach(n)     = lookup(n).reachability_pct;
    J(n)         = compute_J(p_cent(n,:), info_target, 100*reach(n), lambda);
end

unreachable = reach == 0;
if any(unreachable)
    valid_J = J(~unreachable);
    if ~isempty(valid_J)
        J(unreachable) = max(valid_J) + 1e-3;
    end
end

[J_star, idx_star] = min(J);
ps = lookup(idx_star).params;
theta_star = [ps.Vo, ps.el, ps.az, ps.p];

% HP tuning hook for regression order
reg_order = 2;
if isstruct(p_target) && isfield(p_target, 'hp_reg_order') && ~isempty(p_target.hp_reg_order)
    reg_order = p_target.hp_reg_order;
end

X_unit = zeros(N, d);
for k = 1:d
    lo = ranges.(dims{k})(1);
    hi = ranges.(dims{k})(2);
    X_unit(:,k) = (X_phys(:,k) - lo) / (hi - lo);
end

[Phi, ~] = build_features(X_unit, reg_order);
P = size(Phi, 2);
ridge = 1e-6;
beta  = (Phi.' * Phi + ridge * eye(P)) \ (Phi.' * J);
J_hat = Phi * beta;
ss_res = sum((J - J_hat).^2);
ss_tot = sum((J - mean(J)).^2);
R2 = 1 - ss_res / max(ss_tot, eps);

rng(0);
M = 20000;
A_mat = rand(M, d);
B_mat = rand(M, d);
fA = eval_surrogate(A_mat, beta, reg_order);
fB = eval_surrogate(B_mat, beta, reg_order);
varY = var([fA; fB]);
S_first = zeros(d, 1);
S_total = zeros(d, 1);
for k = 1:d
    AB = A_mat;  AB(:,k) = B_mat(:,k);
    fAB = eval_surrogate(AB, beta, reg_order);
    S_first(k) = mean(fB .* (fAB - fA)) / max(varY, eps);
    S_total(k) = 0.5 * mean((fA - fAB).^2) / max(varY, eps);
end

sobol_indices = table(S_first, S_total, 'RowNames', dims, ...
                      'VariableNames', {'S_first','S_total'});

J_eval = compute_J_eval(theta_star, info_target, lookup);

opts = struct('lambda', lambda, 'max_new_sweeps', 0, 'verbose', verbose, ...
              'mu_target', info_target.mu, 'reg_order', reg_order);

meta = struct();
meta.n_evals_cached = N;
meta.n_evals_live   = 0;
meta.wall_time      = toc(t0);
meta.sobol_indices  = sobol_indices;
meta.surrogate_R2   = R2;
meta.J_eval         = J_eval;
meta.opts           = opts;
if R2 < 0.5
    meta.warning = sprintf('surrogate_R2 = %.3f < 0.5; Sobol indices unreliable', R2);
else
    meta.warning = '';
end

save('poc_c_log.mat', 'theta_star', 'J_star', 'meta');

if verbose
    fprintf('POC C: cached=%d, R^2=%.3f, J_eval=%.3f, wall=%.2fs\n', ...
            N, R2, J_eval, meta.wall_time);
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

function [Phi, names] = build_features(X, reg_order)
[N, d] = size(X);
if reg_order == 1
    Phi = [ones(N, 1), X];
    names = [{'1'}, arrayfun(@(k) sprintf('x%d',k), 1:d, 'UniformOutput', false)];
else
    n_quad = d * (d + 1) / 2;
    Phi = ones(N, 1 + d + n_quad);
    names = cell(1, 1 + d + n_quad);
    names{1} = '1';
    Phi(:, 2:1+d) = X;
    for i = 1:d, names{1+i} = sprintf('x%d', i); end
    col = 1 + d + 1;
    for i = 1:d
        for j = i:d
            Phi(:, col) = X(:,i) .* X(:,j);
            names{col}  = sprintf('x%dx%d', i, j);
            col = col + 1;
        end
    end
end
end

function y = eval_surrogate(X, beta, reg_order)
[Phi, ~] = build_features(X, reg_order);
y = Phi * beta;
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
