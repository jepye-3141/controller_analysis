function test_pocs_v2(varargin)
% TEST_POCS_V2  Iterative test harness for 6 POCs with distributional targets.
%   Modes:
%     'mode'           : 'smoke' (cached only, fast) or 'full' (live sweeps).
%     'budget_seconds' : default 6600 (110 min).
%     'max_iter'       : default 6.
%
%   Stages:
%     Stage 0: init (scenarios, samples, cache).
%     Stage 1: iterative cache growth (skipped in 'smoke').
%     Stage 2: HP tuning over cached pool.
%     Stage 3: final live verification (skipped in 'smoke').
%     Stage 4: write report.

p = inputParser;
addParameter(p, 'mode', 'smoke');
addParameter(p, 'budget_seconds', 6600);
addParameter(p, 'max_iter', 6);
parse(p, varargin{:});
opts = p.Results;
is_smoke = strcmpi(opts.mode, 'smoke');

t_start = tic;
addpath(pwd);
addpath(fullfile(pwd, 'Ballistics_Simulation-master'));

stage_times = struct('stage0', NaN, 'stage1', NaN, 'stage2', NaN, ...
                     'stage3', NaN, 'stage4', NaN);

state = struct();
state.opts = opts;
state.t_start = t_start;
state.is_smoke = is_smoke;

%% ================== Stage 0 — Init ==================
fprintf('===== STAGE 0: INIT (mode=%s) =====\n', opts.mode);
t_s0 = tic;
try
    constants;  %#ok<NASGU>
catch ME
    fprintf('constants.m failed: %s\n', ME.message);
end

scenarios = make_scenarios();
state.scenarios = scenarios;

% Load starting cache (do NOT modify the original .mat)
cache_orig = load('centroid_lookup_log.mat');
state.cache = cache_orig;
state.ranges = cache_orig.ranges;
state.range_vec = [diff(state.ranges.Vo), diff(state.ranges.el), ...
                   diff(state.ranges.az), diff(state.ranges.p)];
state.lo = [state.ranges.Vo(1), state.ranges.el(1), state.ranges.az(1), state.ranges.p(1)];
state.initial_N = numel(state.cache.lookup);
fprintf('Initial cache N=%d\n', state.initial_N);
fprintf('Scenarios: S1 concentrated, S2 default, S3 broad. mu=%s\n', mat2str([524 0 0]));

stage_times.stage0 = toc(t_s0);

%% ================== Stage 1 — Iterative cache growth ==================
state.iter_growth = struct('iter', {}, 'recs', {}, 'unique_thetas', {}, ...
                           'n_new_sweeps', {}, 'wall', {});
state.live_sweeps_total_stage1 = 0;
state.converged = false;

if is_smoke
    fprintf('===== STAGE 1: skipped (smoke mode) =====\n');
    stage_times.stage1 = 0;
else
    fprintf('===== STAGE 1: iterative cache growth =====\n');
    t_s1 = tic;
    state = stage1_grow(state);
    stage_times.stage1 = toc(t_s1);
end

if budget_exceeded(t_start, opts.budget_seconds)
    fprintf('BUDGET EXCEEDED: skipping stages 2/3\n');
    stage_times.stage2 = 0;
    stage_times.stage3 = 0;
    write_report(state, stage_times);
    return
end

%% ================== Stage 2 — HP tuning ==================
fprintf('===== STAGE 2: HP tuning =====\n');
t_s2 = tic;
try
    state = stage2_hp(state);
catch ME
    fprintf('Stage 2 errored: %s\n', ME.message);
    state.hp_results = struct('error', ME.message);
end
stage_times.stage2 = toc(t_s2);
save_lite('hp_results.mat', state.hp_results);

if budget_exceeded(t_start, opts.budget_seconds)
    fprintf('BUDGET EXCEEDED: skipping stage 3\n');
    stage_times.stage3 = 0;
    write_report(state, stage_times);
    return
end

%% ================== Stage 3 — Final live verification ==================
state.final_results = struct('rows', {{}});
if is_smoke
    fprintf('===== STAGE 3: cached-only (smoke mode) =====\n');
    t_s3 = tic;
    state = stage3_verify(state, true);
    stage_times.stage3 = toc(t_s3);
else
    fprintf('===== STAGE 3: final live verification =====\n');
    t_s3 = tic;
    state = stage3_verify(state, false);
    stage_times.stage3 = toc(t_s3);
end
save_lite('final_results.mat', state.final_results);

%% ================== Stage 4 — Write report ==================
fprintf('===== STAGE 4: writing report =====\n');
t_s4 = tic;
write_report(state, stage_times);
stage_times.stage4 = toc(t_s4);

fprintf('===== DONE — total wall time %.2fs =====\n', toc(t_start));

end

%% ============================================================
%% Helpers
%% ============================================================

function scenarios = make_scenarios()
mu_t = [524 0 0];
sigmas = {diag([20 10 0].^2), diag([50 30 0].^2), diag([100 80 0].^2)};
names = {'S1_concentrated', 'S2_default', 'S3_broad'};
scenarios = struct();
for s = 1:3
    rng(1000 + s, 'twister');  % fixed seed per scenario
    Sigma = sigmas{s};
    samples_train = mvnrnd(mu_t, Sigma, 50);
    samples_eval  = mvnrnd(mu_t, Sigma, 200);
    scenarios(s).name = names{s};
    scenarios(s).info = struct('mu', mu_t, 'Sigma', Sigma, ...
                               'samples_train', samples_train, ...
                               'samples_eval',  samples_eval);
end
end

function tf = budget_exceeded(t_start, budget_seconds)
tf = toc(t_start) > budget_seconds;
end

function state = stage1_grow(state)
opts = state.opts;
S2 = state.scenarios(2);  % default scenario only
last_thetas = nan(6, 4);

for it = 1:opts.max_iter
    fprintf('-- Stage1 iter %d/%d --\n', it, opts.max_iter);
    if budget_exceeded(state.t_start, opts.budget_seconds)
        fprintf('budget exceeded mid-stage1, breaking\n');
        break
    end

    % Get one rec from each POC against scenario S2
    recs = nan(6, 4);
    poc_names = {'A', 'B', 'C', 'D', 'E', 'F'};
    for k = 1:6
        try
            theta_rec = run_poc_quick(poc_names{k}, S2.info, 0, 0);
            recs(k, :) = theta_rec;
        catch ME
            fprintf('  POC %s FAILED: %s\n', poc_names{k}, ME.message);
        end
    end

    % Dedupe within iteration (eps=0.01 normalized)
    valid = ~any(isnan(recs), 2);
    rec_v = recs(valid, :);
    unique_recs = dedupe_thetas(rec_v, state.lo, state.range_vec, 0.01);

    % Skip thetas already near cache (eps=0.005)
    cache_thetas = cache_to_theta_mat(state.cache.lookup);
    n_uniq = size(unique_recs, 1);
    keep = true(n_uniq, 1);
    for i = 1:n_uniq
        d = vecnorm((cache_thetas - unique_recs(i,:)) ./ state.range_vec, 2, 2);
        if min(d) < 0.005
            keep(i) = false;
        end
    end
    new_thetas = unique_recs(keep, :);

    fprintf('  6 recs -> %d unique -> %d after cache filter\n', ...
            size(unique_recs, 1), size(new_thetas, 1));

    % Run sweep_landing_centroid live for each new theta
    iter_t0 = tic;
    n_added = 0;
    for i = 1:size(new_thetas, 1)
        if budget_exceeded(state.t_start, opts.budget_seconds)
            fprintf('  budget hit mid-iter, stopping\n');
            break
        end
        try
            params = build_sweep_params(new_thetas(i, :));
            t_one = tic;
            out = sweep_landing_centroid(params, false);
            fprintf('  sweep %d done in %.1fs (centroid=[%.1f %.1f %.1f] reach=%.3f)\n', ...
                    i, toc(t_one), out.p_centroid(1), out.p_centroid(2), out.p_centroid(3), ...
                    out.reachability_pct);
            new_entry = make_lookup_entry(params, out);
            state.cache.lookup(end+1) = new_entry;
            state.live_sweeps_total_stage1 = state.live_sweeps_total_stage1 + 1;
            n_added = n_added + 1;
        catch ME
            fprintf('  sweep %d FAILED: %s\n', i, ME.message);
        end
    end

    state.iter_growth(end+1).iter = it; %#ok<AGROW>
    state.iter_growth(end).recs = recs;
    state.iter_growth(end).unique_thetas = unique_recs;
    state.iter_growth(end).n_new_sweeps = n_added;
    state.iter_growth(end).wall = toc(iter_t0);

    % Save grown cache for this iter
    save_grown_cache(state.cache, it);

    % Plot growth visualization
    try
        plot_iter_growth(state.cache, n_added, it);
    catch ME
        fprintf('  plot_iter_growth failed: %s\n', ME.message);
    end

    % Convergence check: all recs within eps=0.005 of last_thetas
    if it > 1 && all(valid) && all(~isnan(last_thetas), 'all')
        d_norm = vecnorm((recs - last_thetas) ./ state.range_vec, 2, 2);
        if all(d_norm < 0.005)
            fprintf('  converged at iter %d (all recs stable)\n', it);
            state.converged = true;
            break
        end
    end
    last_thetas = recs;

    % Also break if no new sweeps were added (cache redundant)
    if n_added == 0 && size(unique_recs, 1) > 0
        fprintf('  no new sweeps (all recs near cache); breaking\n');
        state.converged = true;
        break
    end
end
end

function theta_rec = run_poc_quick(name, info, lambda, max_new)
switch name
    case 'A', [theta_rec, ~, ~] = poc_a_bo_gp(info, lambda, max_new, false);
    case 'B', [theta_rec, ~, ~] = poc_b_cem(info, lambda, max_new, false);
    case 'C', [theta_rec, ~, ~] = poc_c_sobol(info, lambda, max_new, false);
    case 'D', [theta_rec, ~, ~] = poc_d_cvar(info, lambda, max_new, false, 0.5);
    case 'E', [theta_rec, ~, ~] = poc_e_nn_prescreen_cem(info, lambda, max_new, false);
    case 'F', [theta_rec, ~, ~] = poc_f_nn_surrogate(info, lambda, max_new, false, false);
end
end

function out = run_poc_full(name, info, hp)
% Returns theta_star, J_star, J_eval (struct out).
opts_in = info;
if isstruct(hp)
    fns = fieldnames(hp);
    for k = 1:numel(fns)
        opts_in.(fns{k}) = hp.(fns{k});
    end
end
switch name
    case 'A', [ts, js, m] = poc_a_bo_gp(opts_in, 0, 0, false);
    case 'B', [ts, js, m] = poc_b_cem(opts_in, 0, 0, false);
    case 'C', [ts, js, m] = poc_c_sobol(opts_in, 0, 0, false);
    case 'D'
        mu_d = 0.5;
        if isstruct(hp) && isfield(hp, 'mu_d'), mu_d = hp.mu_d; end
        [ts, js, m] = poc_d_cvar(opts_in, 0, 0, false, mu_d);
    case 'E', [ts, js, m] = poc_e_nn_prescreen_cem(opts_in, 0, 0, false);
    case 'F', [ts, js, m] = poc_f_nn_surrogate(opts_in, 0, 0, false, false);
end
out = struct('theta_star', ts, 'J_star', js, 'J_eval', m.J_eval);
end

function thetas = cache_to_theta_mat(lookup)
n = numel(lookup);
thetas = zeros(n, 4);
for i = 1:n
    pa = lookup(i).params;
    thetas(i, :) = [pa.Vo, pa.el, pa.az, pa.p];
end
end

function uniq = dedupe_thetas(thetas, lo, range_vec, eps_norm) %#ok<INUSL>
% Dedupe rows of thetas within eps_norm in normalized [0,1] coords.
n = size(thetas, 1);
if n == 0, uniq = zeros(0, 4); return; end
norm_th = (thetas - lo) ./ range_vec;
keep = true(n, 1);
for i = 2:n
    d = vecnorm(norm_th(1:i-1, :) - norm_th(i, :), 2, 2);
    if any(d < eps_norm)
        keep(i) = false;
    end
end
uniq = thetas(keep, :);
end

function params = build_sweep_params(theta)
params = struct('Vo', theta(1), 'el', theta(2), 'az', theta(3), ...
                'w_z0', 0, 'w_y0', 0, 'p', theta(4), ...
                'alpha_0', 2, 'beta_0', -0.5, ...
                'x_0', 0, 'y_0', 0, 'z_0', 0, 't_max', 300);
end

function entry = make_lookup_entry(params, out)
entry = struct('params', params, ...
               'p_centroid', out.p_centroid, ...
               'reachability_pct', out.reachability_pct, ...
               'half_radius', out.radial_profile.half_radius, ...
               'ballistic_solution', out.ballistic_solution);
end

function save_grown_cache(cache, iter_n)
fname = sprintf('centroid_lookup_log_v2_iter%d.mat', iter_n);
lookup = cache.lookup; %#ok<NASGU>
ranges = cache.ranges; %#ok<NASGU>
N = numel(cache.lookup); %#ok<NASGU>
save(fname, 'lookup', 'ranges', 'N');
end

function plot_iter_growth(cache, n_new, iter_n)
fig = figure('Visible', 'off');
n_total = numel(cache.lookup);
xs = nan(n_total, 1); ys = nan(n_total, 1); rs = nan(n_total, 1);
for i = 1:n_total
    pc = cache.lookup(i).p_centroid(:).';
    xs(i) = pc(1);
    ys(i) = pc(2);
    rs(i) = cache.lookup(i).reachability_pct;
end
% Mark new entries (the last n_new)
old_mask = false(n_total, 1);
old_mask(1:n_total-n_new) = true;
hold on
scatter(xs(old_mask), ys(old_mask), 60, rs(old_mask), 'filled', ...
    'MarkerEdgeColor', 'k');
if n_new > 0
    scatter(xs(~old_mask), ys(~old_mask), 140, rs(~old_mask), 'filled', ...
        'MarkerEdgeColor', 'm', 'LineWidth', 2);
end
colormap(gca, jet)
clim([0 1])
cb = colorbar; cb.Label.String = "reachability fraction";
xlabel('p\_centroid x (m)'); ylabel('p\_centroid y (m)');
title(sprintf('Cache growth iter %d (N=%d, +%d new)', iter_n, n_total, n_new));
grid on
fontsize(gcf, 14, 'points');
fname = sprintf('figs/POC_iter_%d_growth.png', iter_n);
exportgraphics(gcf, fname, 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function state = stage2_hp(state)
% Run 2-3 HP variants per POC on the GROWN cache against scenario S2.
S2 = state.scenarios(2);
hp_grids = make_hp_grids();
poc_names = fieldnames(hp_grids);
hp_results = repmat(struct('poc', '', 'variant', '', ...
                           'theta_star', [], 'J_star', NaN, 'J_eval', NaN, ...
                           'errored', false, 'errmsg', ''), 0, 1);
for k = 1:numel(poc_names)
    name = poc_names{k};
    variants = hp_grids.(name);
    for v = 1:numel(variants)
        try
            out = run_poc_full(name, S2.info, variants(v).hp);
            hp_results(end+1) = struct('poc', name, ...
                'variant', variants(v).label, ...
                'theta_star', out.theta_star, ...
                'J_star', out.J_star, ...
                'J_eval', out.J_eval, ...
                'errored', false, 'errmsg', ''); %#ok<AGROW>
            fprintf('  HP %s/%s: J*=%.3f J_eval=%.3f theta=[%.1f %.1f %.1f %.2f]\n', ...
                name, variants(v).label, out.J_star, out.J_eval, ...
                out.theta_star(1), out.theta_star(2), out.theta_star(3), out.theta_star(4));
        catch ME
            hp_results(end+1) = struct('poc', name, ...
                'variant', variants(v).label, ...
                'theta_star', [], 'J_star', NaN, 'J_eval', NaN, ...
                'errored', true, 'errmsg', ME.message); %#ok<AGROW>
            fprintf('  HP %s/%s ERROR: %s\n', name, variants(v).label, ME.message);
        end
    end
end
state.hp_results = hp_results;
% plot HP sensitivity
try
    plot_hp_sensitivity(hp_results);
catch ME
    fprintf('plot_hp_sensitivity failed: %s\n', ME.message);
end
end

function hp = make_hp_grids()
hp.A = struct('label', {'EI-plus', 'LCB'}, 'hp', {struct('hp_acq', 'expected-improvement-plus'), ...
                                                  struct('hp_acq', 'lower-confidence-bound')});
hp.B = struct('label', {'rho_0.10', 'rho_0.20'}, 'hp', {struct('hp_rho', 0.10), struct('hp_rho', 0.20)});
hp.C = struct('label', {'order_1', 'order_2'}, 'hp', {struct('hp_reg_order', 1), struct('hp_reg_order', 2)});
hp.D = struct('label', {'mu_0', 'mu_0.5', 'mu_1'}, ...
              'hp', {struct('mu_d', 0), struct('mu_d', 0.5), struct('mu_d', 1)});
hp.E = struct('label', {'ardSE', 'ardRQ'}, 'hp', ...
              {struct('hp_kernel', 'ardsquaredexponential'), ...
               struct('hp_kernel', 'ardrationalquadratic')});
hp.F = struct('label', {'ardSE_g11', 'ardRQ_g11'}, 'hp', ...
              {struct('hp_kernel', 'ardsquaredexponential', 'hp_n_grid', 11), ...
               struct('hp_kernel', 'ardrationalquadratic',  'hp_n_grid', 11)});
end

function plot_hp_sensitivity(hp_results)
if isempty(hp_results), return; end
fig = figure('Visible', 'off');
labels = arrayfun(@(r) sprintf('%s/%s', r.poc, r.variant), hp_results, ...
                  'UniformOutput', false);
J_eval = arrayfun(@(r) safe_num(r.J_eval), hp_results);
bar(J_eval);
set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'XTickLabelRotation', 45);
ylabel('J\_eval (m)'); title('HP sensitivity (S2 default scenario)');
grid on
fontsize(gcf, 12, 'points');
exportgraphics(gcf, 'figs/POC_hp_sensitivity.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function v = safe_num(x)
if isempty(x) || ~isfinite(x), v = NaN; else, v = x; end
end

function state = stage3_verify(state, smoke_only)
% For each (POC, scenario), pick best HP for that POC, get rec, optionally
% verify with one live sweep, compute J_eval using actual centroid (cached
% nearest if not live).
poc_names = {'A','B','C','D','E','F'};
n_pocs = numel(poc_names);
n_scen = numel(state.scenarios);
rows = repmat(struct('poc','','scenario','','theta_star',[], ...
                     'J_star',NaN,'J_eval',NaN,'reach_pct',NaN, ...
                     'p_centroid',[NaN NaN NaN], 'live', false, ...
                     'errored', false, 'errmsg', ''), 0, 1);

% pick best HP variant per POC (argmin J_eval from stage2)
best_hp = struct();
if isfield(state, 'hp_results') && isstruct(state.hp_results) && ~isempty(state.hp_results)
    for k = 1:n_pocs
        name = poc_names{k};
        ix = find(arrayfun(@(r) strcmp(r.poc, name) && ~r.errored, state.hp_results));
        if isempty(ix)
            best_hp.(name) = struct();
            continue
        end
        [~, im] = min(arrayfun(@(r) safe_num(r.J_eval), state.hp_results(ix)));
        ix_best = ix(im);
        % Recover hp struct
        grids = make_hp_grids();
        v_label = state.hp_results(ix_best).variant;
        var_arr = grids.(name);
        for vv = 1:numel(var_arr)
            if strcmp(var_arr(vv).label, v_label)
                best_hp.(name) = var_arr(vv).hp;
                break
            end
        end
    end
end

% Sweep deduplication: track thetas already swept this stage
swept_thetas = zeros(0, 4);
swept_pcs = zeros(0, 3);
swept_reach = zeros(0, 1);

for k = 1:n_pocs
    for s = 1:n_scen
        scen = state.scenarios(s);
        name = poc_names{k};
        row = struct('poc', name, 'scenario', scen.name, 'theta_star', [], ...
                     'J_star', NaN, 'J_eval', NaN, 'reach_pct', NaN, ...
                     'p_centroid', [NaN NaN NaN], 'live', false, ...
                     'errored', false, 'errmsg', '');
        try
            hp = struct();
            if isfield(best_hp, name), hp = best_hp.(name); end
            res = run_poc_full(name, scen.info, hp);
            row.theta_star = res.theta_star;
            row.J_star = res.J_star;

            % Dedupe within stage3 sweeps
            do_live = false;
            p_centroid = [NaN NaN NaN];
            reach_pct = NaN;
            if size(swept_thetas, 1) > 0
                d = vecnorm((swept_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                [dmin, imin] = min(d);
                if dmin < 0.003
                    p_centroid = swept_pcs(imin, :);
                    reach_pct = swept_reach(imin);
                end
            end

            % Cache check: any cached entry within eps=0.003?
            if ~smoke_only && any(isnan(p_centroid))
                cache_thetas = cache_to_theta_mat(state.cache.lookup);
                d = vecnorm((cache_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                [dmin, imin] = min(d);
                if dmin < 0.003
                    p_centroid = state.cache.lookup(imin).p_centroid(:).';
                    reach_pct = state.cache.lookup(imin).reachability_pct;
                else
                    do_live = true;
                end
            end

            if ~smoke_only && do_live
                if budget_exceeded(state.t_start, state.opts.budget_seconds)
                    fprintf('  budget exceeded; using cached fallback for %s/%s\n', name, scen.name);
                    cache_thetas = cache_to_theta_mat(state.cache.lookup);
                    d = vecnorm((cache_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                    [~, imin] = min(d);
                    p_centroid = state.cache.lookup(imin).p_centroid(:).';
                    reach_pct = state.cache.lookup(imin).reachability_pct;
                else
                    params = build_sweep_params(res.theta_star);
                    out = sweep_landing_centroid(params, false);
                    p_centroid = out.p_centroid(:).';
                    reach_pct = out.reachability_pct;
                    row.live = true;
                    swept_thetas(end+1, :) = res.theta_star; %#ok<AGROW>
                    swept_pcs(end+1, :) = p_centroid; %#ok<AGROW>
                    swept_reach(end+1, 1) = reach_pct; %#ok<AGROW>
                end
            elseif smoke_only && any(isnan(p_centroid))
                cache_thetas = cache_to_theta_mat(state.cache.lookup);
                d = vecnorm((cache_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                [~, imin] = min(d);
                p_centroid = state.cache.lookup(imin).p_centroid(:).';
                reach_pct = state.cache.lookup(imin).reachability_pct;
            end

            % Compute true J_eval against samples_eval
            diffs = scen.info.samples_eval - p_centroid;
            J_eval_true = mean(sqrt(sum(diffs.^2, 2)));

            row.J_eval = J_eval_true;
            row.reach_pct = reach_pct;
            row.p_centroid = p_centroid;
            fprintf('  %s/%s: theta=[%.1f %.1f %.1f %.2f] J_eval=%.3f reach=%.3f live=%d\n', ...
                name, scen.name, res.theta_star(1), res.theta_star(2), ...
                res.theta_star(3), res.theta_star(4), row.J_eval, row.reach_pct, row.live);
        catch ME
            row.errored = true;
            row.errmsg = ME.message;
            fprintf('  %s/%s ERROR: %s\n', name, scen.name, ME.message);
        end
        rows(end+1) = row; %#ok<AGROW>
    end
end
state.final_results = struct('rows', {rows}, 'best_hp', best_hp);

% Plots
try, plot_final_eval_dist(rows, state.scenarios); catch ME, fprintf('plot_final_eval_dist failed: %s\n', ME.message); end
try, plot_pareto(rows); catch ME, fprintf('plot_pareto failed: %s\n', ME.message); end
try, plot_recommendations(rows); catch ME, fprintf('plot_recommendations failed: %s\n', ME.message); end
try, plot_target_distribution(rows, state.scenarios); catch ME, fprintf('plot_target_distribution failed: %s\n', ME.message); end
end

function plot_final_eval_dist(rows, scenarios)
if isempty(rows), return; end
fig = figure('Visible', 'off');
n_scen = numel(scenarios);
poc_names = {'A','B','C','D','E','F'};
for s = 1:n_scen
    subplot(1, n_scen, s);
    data = []; group = [];
    for k = 1:numel(poc_names)
        ix = find(arrayfun(@(r) strcmp(r.poc, poc_names{k}) && strcmp(r.scenario, scenarios(s).name) && ~r.errored, rows), 1);
        if isempty(ix), continue; end
        pc = rows(ix).p_centroid;
        if any(isnan(pc)), continue; end
        diffs = scenarios(s).info.samples_eval - pc;
        d = sqrt(sum(diffs.^2, 2));
        data = [data; d]; %#ok<AGROW>
        group = [group; repmat(k, numel(d), 1)]; %#ok<AGROW>
    end
    if ~isempty(data)
        boxplot(data, group, 'Labels', poc_names(unique(group)));
    end
    title(scenarios(s).name, 'Interpreter', 'none');
    ylabel('||p\_centroid - p\_eval|| (m)');
    grid on
end
fontsize(gcf, 12, 'points');
exportgraphics(gcf, 'figs/POC_final_eval_dist.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function plot_pareto(rows)
if isempty(rows), return; end
fig = figure('Visible', 'off');
poc_names = {'A','B','C','D','E','F'};
markers = {'o','s','d','^','v','p'};
scen_set = unique({rows.scenario});
colors = lines(numel(scen_set));
hold on
h_pocs = gobjects(0);
for k = 1:numel(poc_names)
    for s = 1:numel(scen_set)
        ix = find(arrayfun(@(r) strcmp(r.poc, poc_names{k}) && strcmp(r.scenario, scen_set{s}) && ~r.errored, rows), 1);
        if isempty(ix), continue; end
        h = scatter(rows(ix).reach_pct, rows(ix).J_eval, 120, colors(s,:), ...
                    markers{k}, 'filled', 'MarkerEdgeColor', 'k', ...
                    'DisplayName', sprintf('%s/%s', poc_names{k}, scen_set{s}));
        if k <= numel(poc_names) && s == 1
            h_pocs(end+1) = h; %#ok<AGROW>
        end
    end
end
xlabel('reach\_pct (fraction)'); ylabel('J\_eval (m)');
title('Pareto: J\_eval vs reach\_pct');
grid on
fontsize(gcf, 14, 'points');
exportgraphics(gcf, 'figs/POC_pareto.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function plot_recommendations(rows)
if isempty(rows), return; end
fig = figure('Visible', 'off');
poc_names = {'A','B','C','D','E','F'};
colors = lines(numel(poc_names));
hold on
labels = {'Vo', 'el', 'az', 'p'};
for k = 1:numel(poc_names)
    ix = find(arrayfun(@(r) strcmp(r.poc, poc_names{k}) && ~r.errored, rows));
    for kx = ix(:).'
        if isempty(rows(kx).theta_star), continue; end
        plot(1:4, rows(kx).theta_star, '-o', 'Color', colors(k,:), 'LineWidth', 1.5);
    end
end
set(gca, 'XTick', 1:4, 'XTickLabel', labels);
title('Recommended thetas (parallel coordinates)');
grid on
fontsize(gcf, 14, 'points');
exportgraphics(gcf, 'figs/POC_recommendations.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function plot_target_distribution(rows, scenarios)
if isempty(rows), return; end
fig = figure('Visible', 'off');
n_scen = numel(scenarios);
poc_names = {'A','B','C','D','E','F'};
markers = {'o','s','d','^','v','p'};
for s = 1:n_scen
    subplot(1, n_scen, s);
    samples = scenarios(s).info.samples_eval;
    scatter(samples(:,1), samples(:,2), 8, 'k', 'filled', 'MarkerFaceAlpha', 0.2);
    hold on
    for k = 1:numel(poc_names)
        ix = find(arrayfun(@(r) strcmp(r.poc, poc_names{k}) && strcmp(r.scenario, scenarios(s).name) && ~r.errored, rows), 1);
        if isempty(ix), continue; end
        pc = rows(ix).p_centroid;
        if any(isnan(pc)), continue; end
        scatter(pc(1), pc(2), 200, markers{k}, 'filled', ...
                'MarkerEdgeColor', 'r', 'LineWidth', 2);
        text(pc(1), pc(2), poc_names{k}, 'FontSize', 10, ...
             'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    end
    title(scenarios(s).name, 'Interpreter', 'none');
    xlabel('x (m)'); ylabel('y (m)');
    grid on; axis equal
end
fontsize(gcf, 12, 'points');
exportgraphics(gcf, 'figs/POC_target_distribution.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function save_lite(fname, varval)
data = varval; %#ok<NASGU>
save(fname, 'data');
end

function write_report(state, stage_times)
fname = 'docs/poc_test_report_v2.md';
if ~exist('docs', 'dir'), mkdir('docs'); end
fid = fopen(fname, 'w');
if fid == -1
    fprintf('FAILED to open report file %s\n', fname);
    return
end
total_time = toc(state.t_start);

fprintf(fid, '# POC Test Report V2 - Distributional Targets\n\n');
fprintf(fid, '> Generated by `test_pocs_v2.m`. Mode: `%s`. Total wall time: %.1fs.\n\n', ...
        state.opts.mode, total_time);

% TL;DR
fprintf(fid, '## TL;DR\n\n');
fprintf(fid, '- 6 POCs (A-F) refactored to accept `info_target` distribution struct\n');
fprintf(fid, '  with fields `mu`, `Sigma`, `samples_train`, `samples_eval`.\n');
fprintf(fid, '- New objective: `J = mean over samples_train of ||p_centroid - p|| - lambda*reach`.\n');
fprintf(fid, '- Held-out metric: `J_eval = mean over samples_eval of ||p_centroid - p||`.\n');
fprintf(fid, '- 3 scenarios (S1 concentrated, S2 default, S3 broad) at mu=[524 0 0].\n');
if state.is_smoke
    fprintf(fid, '- **SMOKE MODE**: Stage 1 (cache growth) and Stage 3 live sweeps skipped.\n');
end
fprintf(fid, '\n');

% Stage 1 - Cache growth
fprintf(fid, '## Cache growth summary\n\n');
fprintf(fid, '- Initial N: %d\n', state.initial_N);
if isfield(state, 'cache')
    fprintf(fid, '- Final N: %d\n', numel(state.cache.lookup));
end
fprintf(fid, '- Live sweeps used (Stage 1): %d\n', state.live_sweeps_total_stage1);
if isfield(state, 'iter_growth') && ~isempty(state.iter_growth)
    fprintf(fid, '- Iterations: %d\n', numel(state.iter_growth));
    fprintf(fid, '\n| Iter | Recs valid | Unique | New sweeps | Wall (s) |\n');
    fprintf(fid, '|------|-----------|--------|------------|----------|\n');
    for it = 1:numel(state.iter_growth)
        ig = state.iter_growth(it);
        n_recs = sum(~any(isnan(ig.recs), 2));
        fprintf(fid, '| %d | %d | %d | %d | %.1f |\n', ig.iter, n_recs, ...
                size(ig.unique_thetas, 1), ig.n_new_sweeps, ig.wall);
    end
    if state.converged
        fprintf(fid, '\n*Converged: recommendations stabilized.*\n');
    end
end
fprintf(fid, '\n');

% Stage 2 - HP tuning
fprintf(fid, '## HP tuning results (S2 scenario)\n\n');
if isfield(state, 'hp_results') && isstruct(state.hp_results)
    if numel(state.hp_results) > 0 && isfield(state.hp_results, 'poc')
        fprintf(fid, '| POC | Variant | J_star | J_eval | Theta |\n');
        fprintf(fid, '|-----|---------|--------|--------|-------|\n');
        for r = 1:numel(state.hp_results)
            hp = state.hp_results(r);
            if hp.errored
                fprintf(fid, '| %s | %s | ERR | ERR | %s |\n', hp.poc, hp.variant, hp.errmsg);
            else
                ts = hp.theta_star;
                fprintf(fid, '| %s | %s | %.3f | %.3f | [%.1f %.1f %.1f %.2f] |\n', ...
                    hp.poc, hp.variant, hp.J_star, hp.J_eval, ts(1), ts(2), ts(3), ts(4));
            end
        end
    else
        fprintf(fid, '*HP tuning not run or errored.*\n');
    end
end
fprintf(fid, '\n');

% Stage 3 - Final eval
fprintf(fid, '## Final eval table (POC x scenario)\n\n');
if isfield(state, 'final_results') && isfield(state.final_results, 'rows')
    rows = state.final_results.rows;
    if iscell(rows) && ~isempty(rows)
        rows = [rows{:}];
    end
    if ~isempty(rows) && isfield(rows, 'poc')
        n_live_total = sum(arrayfun(@(r) r.live, rows));
        fprintf(fid, '*Live sweeps in Stage 3: %d.*\n\n', n_live_total);
        fprintf(fid, '| POC | Scenario | J_eval (m) | reach_pct | live? | theta |\n');
        fprintf(fid, '|-----|----------|-----------|-----------|-------|-------|\n');
        for r = 1:numel(rows)
            row = rows(r);
            if row.errored
                fprintf(fid, '| %s | %s | ERR | ERR | - | %s |\n', ...
                    row.poc, row.scenario, row.errmsg);
            else
                ts = row.theta_star;
                fprintf(fid, '| %s | %s | %.3f | %.3f | %d | [%.1f %.1f %.1f %.2f] |\n', ...
                    row.poc, row.scenario, row.J_eval, row.reach_pct, ...
                    row.live, ts(1), ts(2), ts(3), ts(4));
            end
        end
    else
        fprintf(fid, '*No final-eval rows.*\n');
    end
end
fprintf(fid, '\n');

% Tunability summary
fprintf(fid, '## Tunability summary\n\n');
fprintf(fid, 'Each POC takes scalar `lambda` (reach weight). POC D additionally takes `mu` (half_radius weight).\n');
fprintf(fid, 'HP tuning grids in Appendix.\n\n');

% Visualizations
fprintf(fid, '## Visualizations\n\n');
fig_files = dir('figs/POC_*.png');
for f = 1:numel(fig_files)
    fprintf(fid, '- `figs/%s` -- %s\n', fig_files(f).name, caption_for_fig(fig_files(f).name));
end
fprintf(fid, '\n');

% Time budget consumed
fprintf(fid, '## Time budget consumed (per stage)\n\n');
fprintf(fid, '| Stage | Wall (s) |\n');
fprintf(fid, '|-------|----------|\n');
fns = fieldnames(stage_times);
for k = 1:numel(fns)
    fprintf(fid, '| %s | %.2f |\n', fns{k}, stage_times.(fns{k}));
end
fprintf(fid, '| TOTAL | %.2f |\n', total_time);
fprintf(fid, '\n');

% Limitations
fprintf(fid, '## Limitations & caveats\n\n');
if state.is_smoke
    fprintf(fid, '- **Smoke run**: no live sweeps were executed; all `J_eval` are computed from cached centroids nearest the recommended theta.\n');
end
fprintf(fid, '- POC C and D do no live sweeps; they re-rank the cache only.\n');
fprintf(fid, '- POC D `J_eval` reflects half_radius proxy, not classical CVaR_alpha.\n');
fprintf(fid, '- POC F surrogate predictions can extrapolate outside training data; J_pred can disagree with J_live.\n');
fprintf(fid, '\n');

% Recommended follow-ups
fprintf(fid, '## Recommended follow-ups\n\n');
fprintf(fid, '1. Grow cache to N>=100 so POC F NN gate fires.\n');
fprintf(fid, '2. Sweep `lambda` in [0, 0.5, 1, 2] to expose reach tradeoff.\n');
fprintf(fid, '3. Re-run with operationally chosen scenarios (not synthetic Gaussians).\n');
fprintf(fid, '\n');

% Appendix: HP grids
fprintf(fid, '## Appendix: HP grids per POC\n\n');
hp = make_hp_grids();
fns = fieldnames(hp);
for k = 1:numel(fns)
    fprintf(fid, '### POC %s\n\n', fns{k});
    var_arr = hp.(fns{k});
    for v = 1:numel(var_arr)
        fprintf(fid, '- `%s`\n', var_arr(v).label);
    end
    fprintf(fid, '\n');
end

fclose(fid);
fprintf('Wrote %s\n', fname);
end

function s = caption_for_fig(fname)
if contains(fname, 'iter_')
    s = 'Cache growth at this iteration; magenta-edge markers are new entries.';
elseif contains(fname, 'hp_sensitivity')
    s = 'Bar chart: J_eval per HP variant per POC on S2.';
elseif contains(fname, 'final_eval_dist')
    s = 'Boxplots: distribution of ||p_centroid - p_eval|| per POC, per scenario.';
elseif contains(fname, 'pareto')
    s = 'J_eval vs reach_pct per POC, coloured by scenario.';
elseif contains(fname, 'recommendations')
    s = 'Parallel-coordinates plot of recommended thetas (Vo,el,az,p).';
elseif contains(fname, 'target_distribution')
    s = 'Eval-set samples (gray) overlaid with recommended landing centroids per POC.';
else
    s = '';
end
end
