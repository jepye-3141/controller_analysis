function [theta_star, J_star, meta] = poc_a_bo_gp(p_target, lambda, max_new_sweeps, verbose)
%POC_A_BO_GP  MATLAB-native BO/GP optimizer for ballistic targeting (V2 — distributional target).
%   p_target       1x3 numeric (legacy) OR info_target struct with fields
%                  .mu (1x3), .Sigma (3x3), .samples_train (N_train x 3),
%                  .samples_eval (N_eval x 3). Default [50 0 0].
%   lambda         scalar weight on reachability_pct (default 0).
%   max_new_sweeps integer budget of live sweep_landing_centroid calls (default 0).
%   verbose        logical (default true).

if nargin < 1 || isempty(p_target),       p_target = [50 0 0];      end
if nargin < 2 || isempty(lambda),         lambda = 0;               end
if nargin < 3 || isempty(max_new_sweeps), max_new_sweeps = 0;       end
if nargin < 4 || isempty(verbose),        verbose = true;           end

info_target = wrap_info_target(p_target);

t0 = tic;
S = load('centroid_lookup_log.mat');
lookup = S.lookup;
ranges = S.ranges;

n_cached = numel(lookup);
theta_cached = zeros(n_cached, 4);
J_cached = zeros(n_cached, 1);
for i = 1:n_cached
    pr = lookup(i).params;
    theta_cached(i, :) = [pr.Vo, pr.el, pr.az, pr.p];
    J_cached(i) = compute_J(lookup(i).p_centroid(:).', info_target, ...
                            100*lookup(i).reachability_pct, lambda);
end
% Penalize zero-reachability cached entries
zero_reach = arrayfun(@(s) s.reachability_pct == 0, lookup(:));
if any(zero_reach)
    J_cached(zero_reach) = max(J_cached(~zero_reach)) + 1e3;
end

w_z0_med = median(arrayfun(@(L) L.params.w_z0, lookup));
w_y0_med = median(arrayfun(@(L) L.params.w_y0, lookup));

vars = [
    optimizableVariable('Vo', ranges.Vo, 'Type', 'real')
    optimizableVariable('el', ranges.el, 'Type', 'real')
    optimizableVariable('az', ranges.az, 'Type', 'real')
    optimizableVariable('p',  ranges.p,  'Type', 'real')
];

InitialX = array2table(theta_cached, 'VariableNames', {'Vo','el','az','p'});
InitialObjective = J_cached;

history = struct('theta', num2cell(theta_cached, 2), ...
                 'J', num2cell(J_cached), ...
                 'source', repmat({'cached'}, n_cached, 1));

% HP tuning hook
acq = 'expected-improvement-plus';
if isstruct(p_target) && isfield(p_target, 'hp_acq') && ~isempty(p_target.hp_acq)
    acq = p_target.hp_acq;
end

n_live = 0;
if max_new_sweeps > 0
    static_params = struct('alpha_0', 2, 'beta_0', -0.5, ...
                           'x_0', 0, 'y_0', 0, 'z_0', 0, 't_max', 300, ...
                           'w_z0', w_z0_med, 'w_y0', w_y0_med);
    obj_fn = @(t) live_objective(t, static_params, info_target, lambda);
    gp_results = bayesopt(obj_fn, vars, ...
        'AcquisitionFunctionName', acq, ...
        'InitialX', InitialX, 'InitialObjective', InitialObjective, ...
        'MaxObjectiveEvaluations', n_cached + max_new_sweeps, ...
        'IsObjectiveDeterministic', false, ...
        'Verbose', double(verbose), 'PlotFcn', {});
    new_rows = (n_cached + 1):height(gp_results.XTrace);
    for r = new_rows
        n_live = n_live + 1;
        history(end+1).theta = table2array(gp_results.XTrace(r, :)); %#ok<AGROW>
        history(end).J = gp_results.ObjectiveTrace(r);
        history(end).source = 'live';
    end
    theta_star = table2array(gp_results.XAtMinObjective);
    J_star = gp_results.MinObjective;
else
    gp_results = bayesopt(@(t) noop_objective(t), vars, ...
        'AcquisitionFunctionName', acq, ...
        'InitialX', InitialX, 'InitialObjective', InitialObjective, ...
        'MaxObjectiveEvaluations', n_cached, ...
        'IsObjectiveDeterministic', false, ...
        'Verbose', double(verbose), 'PlotFcn', {});
    grid_n = 8;
    [V, E, A, P] = ndgrid( ...
        linspace(ranges.Vo(1), ranges.Vo(2), grid_n), ...
        linspace(ranges.el(1), ranges.el(2), grid_n), ...
        linspace(ranges.az(1), ranges.az(2), grid_n), ...
        linspace(ranges.p(1),  ranges.p(2),  grid_n));
    grid_tbl = array2table([V(:) E(:) A(:) P(:)], ...
                           'VariableNames', {'Vo','el','az','p'});
    [mu_pred, ~] = predictObjective(gp_results, grid_tbl);
    [J_star, idx] = min(mu_pred);
    theta_star = table2array(grid_tbl(idx, :));
end

% Eval-set metric: mean Euclidean distance from p_centroid_at_theta_star to samples_eval
J_eval = compute_J_eval(theta_star, info_target, lookup);

meta = struct( ...
    'n_evals_cached', n_cached, ...
    'n_evals_live',   n_live, ...
    'wall_time',      toc(t0), ...
    'history',        history, ...
    'J_eval',         J_eval, ...
    'opts', struct('lambda', lambda, ...
                   'max_new_sweeps', max_new_sweeps, 'verbose', verbose, ...
                   'mu_target', info_target.mu, 'acq', acq));

if verbose
    fprintf('poc_a_bo_gp: theta_star=[%.2f %.2f %.2f %.2f]  J*=%.3f  J_eval=%.3f  wall=%.2fs\n', ...
            theta_star(1), theta_star(2), theta_star(3), theta_star(4), ...
            J_star, J_eval, meta.wall_time);
end

save('poc_a_log.mat', 'theta_star', 'J_star', 'meta', 'gp_results');

end

function J = compute_J(p_centroid, info_target, reach_pct, lambda)
% MC-mean over samples_train of ||p_centroid - p|| - lambda*(reach_pct/100)
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
% Find p_centroid at theta_star: nearest cached entry by 4-D Euclidean (normalized).
% This is the "predicted" eval since no live sweep is required.
n = numel(lookup);
theta_mat = zeros(n, 4);
for i = 1:n
    pr = lookup(i).params;
    theta_mat(i, :) = [pr.Vo, pr.el, pr.az, pr.p];
end
% normalized distance
S = load('centroid_lookup_log.mat', 'ranges');
range_vec = [diff(S.ranges.Vo), diff(S.ranges.el), diff(S.ranges.az), diff(S.ranges.p)];
d = sqrt(sum(((theta_mat - theta_star) ./ range_vec).^2, 2));
[~, ix] = min(d);
p_c = lookup(ix).p_centroid(:).';
p_eval = info_target.samples_eval;
diffs = p_eval - p_c;
J_eval = mean(sqrt(sum(diffs.^2, 2)));
end

function J = live_objective(t, static_params, info_target, lambda)
params = static_params;
params.Vo = t.Vo; params.el = t.el; params.az = t.az; params.p = t.p;
out = sweep_landing_centroid(params, false);
J = compute_J(out.p_centroid(:).', info_target, ...
              100*out.reachability_pct, lambda);
end

function J = noop_objective(~)
J = 0;
end

function info = wrap_info_target(p_target)
% Backward-compat: 1x3 numeric -> info_target struct.
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
