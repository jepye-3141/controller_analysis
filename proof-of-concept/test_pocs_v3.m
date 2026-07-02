function test_pocs_v3(varargin)
% TEST_POCS_V3  Iterative POC test harness with forced exploration + live HP tuning.
%   Modes:
%     'mode'           : 'smoke' (no live sweeps, fast) or 'full' (live sweeps).
%     'budget_seconds' : default 6300 (~105 min).
%     'max_iter'       : default 6.
%     'cache_seed'     : path to seed .mat. Default 'centroid_lookup_log_v2_iter2.mat'.
%
%   Diffs from v2:
%     - Stage 1 forces 2 exploration thetas per POC per iter (Gaussian noise).
%     - Stage 2 runs each HP variant with a live-eval budget (A,B,E,F).
%     - Stage 4 writes to docs/poc_test_report_v3.md with delta-from-v2 section.
%     - All figures live under figs/POC_v3_*.png.
%
%   Cache management:
%     - At Stage 0 the seed file's contents are written to centroid_lookup_log.mat
%       so the POCs (which hardcode that path) see the grown pool. The original
%       cache is backed up to centroid_lookup_log_v3_orig_backup.mat (only the
%       first time) and restored at end via onCleanup. This is explicit and
%       reversible — the original .mat is preserved bit-for-bit.

p = inputParser;
addParameter(p, 'mode', 'smoke');
addParameter(p, 'budget_seconds', 6300);
addParameter(p, 'max_iter', 6);
addParameter(p, 'cache_seed', 'centroid_lookup_log_v2_iter2.mat');
addParameter(p, 'skip_stage1', false);
addParameter(p, 'report_path', 'docs/poc_test_report_v3.md');
parse(p, varargin{:});
opts = p.Results;
is_smoke = strcmpi(opts.mode, 'smoke');

t_start = tic;
addpath(pwd);
addpath(fullfile(pwd, 'Ballistics_Simulation-master'));

% Constants for forced exploration / dedup / per-iter budget
EXPLORATION_NOISE_NORM = 0.05;
STAGE2_LIVE_BUDGET_PER_VARIANT = 3;
MAX_SWEEPS_PER_ITER = 6;
DEDUP_EPS_NORM = 0.005;
CACHE_DEDUP_EPS = 0.003;

stage_times = struct('stage0', NaN, 'stage1', NaN, 'stage2', NaN, ...
                     'stage3', NaN, 'stage4', NaN);

state = struct();
state.opts = opts;
state.t_start = t_start;
state.is_smoke = is_smoke;
state.constants = struct('EXPLORATION_NOISE_NORM', EXPLORATION_NOISE_NORM, ...
                         'STAGE2_LIVE_BUDGET_PER_VARIANT', STAGE2_LIVE_BUDGET_PER_VARIANT, ...
                         'MAX_SWEEPS_PER_ITER', MAX_SWEEPS_PER_ITER, ...
                         'DEDUP_EPS_NORM', DEDUP_EPS_NORM, ...
                         'CACHE_DEDUP_EPS', CACHE_DEDUP_EPS);

%% ================== Stage 0 — Init + cache seeding ==================
fprintf('===== STAGE 0: INIT (mode=%s) =====\n', opts.mode);
t_s0 = tic;
try
    constants;  %#ok<NASGU>
catch ME
    fprintf('constants.m failed: %s\n', ME.message);
end

% Prewarm parpool — v3 launches with 4 workers hung for 10+ min when parsim
% reported "Connected to 3 of 4 parallel pool workers". The 4th worker loses an
% online-licensing race intermittently. Start with 3 workers to sidestep it;
% sweep_landing_centroid.m will reuse this pool via gcp('nocreate').
try
    delete(gcp('nocreate'));
catch
end
pool = [];
for nw = [3, 2]
    try
        t_pp = tic;
        pool = parpool('Processes', nw, 'IdleTimeout', 240);
        fprintf('parpool: %d workers, started in %.1fs\n', pool.NumWorkers, toc(t_pp));
        % Verify pool is actually responsive — issue a trivial parfor to flush
        % any stuck connection state. If a worker is dead this will fail fast.
        t_v = tic;
        parfor wi = 1:pool.NumWorkers
            pause(0.01);  %#ok<PFBNS>
        end
        fprintf('parpool verified (parfor roundtrip %.2fs)\n', toc(t_v));
        break
    catch ME
        fprintf('parpool(Processes, %d) failed: %s\n', nw, ME.message);
        try, delete(gcp('nocreate')); catch, end
        pool = [];
    end
end
if isempty(pool)
    fprintf('All parpool attempts failed; proceeding serially (sweeps will be SLOW)\n');
end

scenarios = make_scenarios();
state.scenarios = scenarios;

% Seed the working cache from cache_seed (back up original first).
backup_path = 'centroid_lookup_log_v3_orig_backup.mat';
if ~isfile(backup_path) && isfile('centroid_lookup_log.mat')
    copyfile('centroid_lookup_log.mat', backup_path);
    fprintf('Backed up original centroid_lookup_log.mat -> %s\n', backup_path);
end
if isfile(opts.cache_seed)
    copyfile(opts.cache_seed, 'centroid_lookup_log.mat');
    fprintf('Seeded centroid_lookup_log.mat <- %s\n', opts.cache_seed);
else
    warning('cache_seed %s missing; using existing centroid_lookup_log.mat', opts.cache_seed);
end

% Register cleanup to restore the original on any exit.
restorer = onCleanup(@() restore_cache(backup_path));  %#ok<NASGU>

cache_orig = load('centroid_lookup_log.mat');
state.cache = cache_orig;
state.ranges = cache_orig.ranges;
state.range_vec = [diff(state.ranges.Vo), diff(state.ranges.el), ...
                   diff(state.ranges.az), diff(state.ranges.p)];
state.lo = [state.ranges.Vo(1), state.ranges.el(1), ...
            state.ranges.az(1), state.ranges.p(1)];
state.initial_N = numel(state.cache.lookup);
fprintf('Initial cache N=%d (seeded)\n', state.initial_N);
fprintf('Scenarios: S1 concentrated, S2 default, S3 broad. mu=%s\n', mat2str([524 0 0]));

if ~exist('figs', 'dir'), mkdir('figs'); end
if ~exist('docs', 'dir'), mkdir('docs'); end

stage_times.stage0 = toc(t_s0);

%% ================== Stage 1 — Iterative cache growth (forced exploration) ==================
state.iter_growth = struct('iter', {}, 'recs', {}, 'explore_thetas', {}, ...
                           'unique_thetas', {}, 'after_cache_filter', {}, ...
                           'n_new_sweeps', {}, 'wall', {});
state.live_sweeps_total_stage1 = 0;
state.converged = false;

if opts.skip_stage1
    fprintf('===== STAGE 1: SKIPPED (skip_stage1=true) =====\n');
    stage_times.stage1 = 0;
elseif is_smoke
    fprintf('===== STAGE 1: smoke (1 iter, no live sweeps) =====\n');
    t_s1 = tic;
    state = stage1_grow(state, true);  % smoke=true
    stage_times.stage1 = toc(t_s1);
else
    fprintf('===== STAGE 1: iterative cache growth =====\n');
    t_s1 = tic;
    state = stage1_grow(state, false);
    stage_times.stage1 = toc(t_s1);
end

if budget_exceeded(t_start, opts.budget_seconds)
    fprintf('BUDGET EXCEEDED: skipping stages 2/3\n');
    stage_times.stage2 = 0;
    stage_times.stage3 = 0;
    write_report(state, stage_times);
    return
end

%% ================== Stage 2 — HP tuning with live evals ==================
fprintf('===== STAGE 2: HP tuning with live evals =====\n');
t_s2 = tic;
try
    state = stage2_hp(state);
catch ME
    fprintf('Stage 2 errored: %s\n', ME.message);
    state.hp_results = struct('error', ME.message);
end
stage_times.stage2 = toc(t_s2);
save_lite('hp_results_v3.mat', state.hp_results);

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
save_lite('final_results_v3.mat', state.final_results);

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
    rng(1000 + s, 'twister');
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

function restore_cache(backup_path)
% onCleanup callback: restore original centroid_lookup_log.mat.
try
    if isfile(backup_path)
        copyfile(backup_path, 'centroid_lookup_log.mat');
        fprintf('Restored centroid_lookup_log.mat from %s\n', backup_path);
    end
catch ME
    fprintf('restore_cache failed: %s\n', ME.message);
end
end

function state = stage1_grow(state, smoke)
if nargin < 2, smoke = false; end
opts = state.opts;
S2 = state.scenarios(2);  % default scenario only for Stage 1
last_thetas = nan(6, 4);
poc_names = {'A', 'B', 'C', 'D', 'E', 'F'};
EXPLORATION_NOISE_NORM = state.constants.EXPLORATION_NOISE_NORM;
DEDUP_EPS_NORM = state.constants.DEDUP_EPS_NORM;
CACHE_DEDUP_EPS = state.constants.CACHE_DEDUP_EPS;
MAX_SWEEPS_PER_ITER = state.constants.MAX_SWEEPS_PER_ITER;
n_iters = opts.max_iter;
if smoke, n_iters = 1; end

for it = 1:n_iters
    fprintf('-- Stage1 iter %d/%d --\n', it, opts.max_iter);
    if budget_exceeded(state.t_start, opts.budget_seconds)
        fprintf('budget exceeded mid-stage1, breaking\n');
        break
    end

    % 1) Get one theta_best from each POC against scenario S2
    recs = nan(6, 4);
    for k = 1:6
        try
            theta_rec = run_poc_quick(poc_names{k}, S2.info, 0, 0);
            recs(k, :) = theta_rec;
        catch ME
            fprintf('  POC %s FAILED: %s\n', poc_names{k}, ME.message);
        end
    end

    % 2) Generate 2 exploration thetas per POC: theta_best + 5%*range*randn
    explore_thetas = nan(12, 4);
    rg = state.range_vec;
    lo = state.lo;
    hi = lo + rg;
    for k = 1:6
        if any(isnan(recs(k, :))), continue; end
        for kk = 1:2
            % Deterministic seed per (poc_idx, iter, k)
            seed = 7000 + 100*it + 10*k + kk;
            rng(seed, 'twister');
            noise = EXPLORATION_NOISE_NORM * randn(1, 4) .* rg;
            theta_e = recs(k, :) + noise;
            theta_e = max(min(theta_e, hi), lo);
            explore_thetas(2*(k-1) + kk, :) = theta_e;
        end
    end

    % 3) Aggregate candidates: 6 best + 12 exploration = 18
    all_cands = [recs; explore_thetas];
    valid = ~any(isnan(all_cands), 2);
    cand_v = all_cands(valid, :);

    n_total = size(cand_v, 1);
    unique_recs = dedupe_thetas(cand_v, state.lo, state.range_vec, DEDUP_EPS_NORM);
    n_uniq = size(unique_recs, 1);

    % 4) Skip thetas already in cache (CACHE_DEDUP_EPS normalized)
    cache_thetas = cache_to_theta_mat(state.cache.lookup);
    keep = true(n_uniq, 1);
    for i = 1:n_uniq
        d = vecnorm((cache_thetas - unique_recs(i,:)) ./ state.range_vec, 2, 2);
        if min(d) < CACHE_DEDUP_EPS
            keep(i) = false;
        end
    end
    new_thetas = unique_recs(keep, :);
    n_after_cache = size(new_thetas, 1);

    % 5) Cap at MAX_SWEEPS_PER_ITER (subsample randomly with iter-deterministic seed)
    if n_after_cache > MAX_SWEEPS_PER_ITER
        rng(8000 + it, 'twister');
        ix = randperm(n_after_cache, MAX_SWEEPS_PER_ITER);
        new_thetas = new_thetas(ix, :);
    end

    fprintf('  18 cands -> %d valid -> %d unique -> %d after cache filter -> %d to sweep\n', ...
            sum(valid), n_uniq, n_after_cache, size(new_thetas, 1));

    % 6) Run sweep_landing_centroid live for each new theta (or stub in smoke mode)
    iter_t0 = tic;
    n_added = 0;
    for i = 1:size(new_thetas, 1)
        if budget_exceeded(state.t_start, opts.budget_seconds)
            fprintf('  budget hit mid-iter, stopping\n');
            break
        end
        if smoke
            fprintf('  smoke: would sweep theta %d=[%.1f %.1f %.1f %.2f]\n', ...
                    i, new_thetas(i,1), new_thetas(i,2), new_thetas(i,3), new_thetas(i,4));
            continue
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
            % Persist cache so subsequent POC calls see new entry
            persist_cache(state.cache);
        catch ME
            fprintf('  sweep %d FAILED: %s\n', i, ME.message);
        end
    end

    state.iter_growth(end+1).iter = it; %#ok<AGROW>
    state.iter_growth(end).recs = recs;
    state.iter_growth(end).explore_thetas = explore_thetas;
    state.iter_growth(end).unique_thetas = unique_recs;
    state.iter_growth(end).after_cache_filter = n_after_cache;
    state.iter_growth(end).n_new_sweeps = n_added;
    state.iter_growth(end).wall = toc(iter_t0);

    save_grown_cache(state.cache, it);

    try
        plot_iter_growth(state.cache, n_added, it, new_thetas, smoke);
    catch ME
        fprintf('  plot_iter_growth failed: %s\n', ME.message);
    end

    % Convergence: only theta_best matters (not explore thetas)
    valid_recs = ~any(isnan(recs), 2);
    if it > 1 && all(valid_recs) && all(~isnan(last_thetas), 'all')
        d_norm = vecnorm((recs - last_thetas) ./ state.range_vec, 2, 2);
        if all(d_norm < DEDUP_EPS_NORM)
            fprintf('  converged at iter %d (theta_best stable)\n', it);
            state.converged = true;
            break
        end
    end
    last_thetas = recs;
end
end

function persist_cache(cache)
lookup = cache.lookup; %#ok<NASGU>
ranges = cache.ranges; %#ok<NASGU>
N = numel(cache.lookup); %#ok<NASGU>
save('centroid_lookup_log.mat', 'lookup', 'ranges', 'N');
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

function out = run_poc_full(name, info, hp, max_new_sweeps)
% Returns theta_star, J_star, J_eval, n_evals_live.
if nargin < 4, max_new_sweeps = 0; end
opts_in = info;
if isstruct(hp)
    fns = fieldnames(hp);
    for k = 1:numel(fns)
        opts_in.(fns{k}) = hp.(fns{k});
    end
end
switch name
    case 'A', [ts, js, m] = poc_a_bo_gp(opts_in, 0, max_new_sweeps, false);
    case 'B', [ts, js, m] = poc_b_cem(opts_in, 0, max_new_sweeps, false);
    case 'C', [ts, js, m] = poc_c_sobol(opts_in, 0, max_new_sweeps, false);
    case 'D'
        mu_d = 0.5;
        if isstruct(hp) && isfield(hp, 'mu_d'), mu_d = hp.mu_d; end
        [ts, js, m] = poc_d_cvar(opts_in, 0, max_new_sweeps, false, mu_d);
    case 'E', [ts, js, m] = poc_e_nn_prescreen_cem(opts_in, 0, max_new_sweeps, false);
    case 'F', [ts, js, m] = poc_f_nn_surrogate(opts_in, 0, max_new_sweeps, false, false);
end
n_live = 0;
if isstruct(m) && isfield(m, 'n_evals_live'), n_live = m.n_evals_live; end
out = struct('theta_star', ts, 'J_star', js, 'J_eval', m.J_eval, ...
             'n_evals_live', n_live);
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
fname = sprintf('centroid_lookup_log_v3_iter%d.mat', iter_n);
lookup = cache.lookup; %#ok<NASGU>
ranges = cache.ranges; %#ok<NASGU>
N = numel(cache.lookup); %#ok<NASGU>
save(fname, 'lookup', 'ranges', 'N');
end

function plot_iter_growth(cache, n_new, iter_n, proposed_thetas, smoke)
if nargin < 4, proposed_thetas = []; end
if nargin < 5, smoke = false; end
fig = figure('Visible', 'off');
n_total = numel(cache.lookup);
xs = nan(n_total, 1); ys = nan(n_total, 1); rs = nan(n_total, 1);
for i = 1:n_total
    pc = cache.lookup(i).p_centroid(:).';
    xs(i) = pc(1);
    ys(i) = pc(2);
    rs(i) = cache.lookup(i).reachability_pct;
end
old_mask = false(n_total, 1);
old_mask(1:n_total-n_new) = true;
hold on
scatter(xs(old_mask), ys(old_mask), 60, rs(old_mask), 'filled', ...
    'MarkerEdgeColor', 'k');
if n_new > 0
    scatter(xs(~old_mask), ys(~old_mask), 140, rs(~old_mask), 'filled', ...
        'MarkerEdgeColor', 'm', 'LineWidth', 2);
end
% In smoke mode, show proposed thetas (Vo, el) as projection markers since
% no centroid is available yet.
if smoke && ~isempty(proposed_thetas)
    text(0.02, 0.96, sprintf('smoke: %d proposed thetas (no sweep)', ...
        size(proposed_thetas, 1)), 'Units', 'normalized', ...
        'BackgroundColor', [1 1 0.8], 'EdgeColor', 'k');
end
colormap(gca, jet)
clim([0 1])
cb = colorbar; cb.Label.String = "reachability fraction";
xlabel('p\_centroid x (m)'); ylabel('p\_centroid y (m)');
title(sprintf('Cache growth iter %d (N=%d, +%d new)', iter_n, n_total, n_new));
grid on
fontsize(gcf, 14, 'points');
fname = sprintf('figs/POC_v3_iter_%d_growth.png', iter_n);
exportgraphics(gcf, fname, 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function state = stage2_hp(state)
% Run HP variants per POC. A/B/E/F get live-eval budget per variant, sweeping
% max_new_sweeps from 0..STAGE2_LIVE_BUDGET_PER_VARIANT to build a J trace.
% C/D stay cached-only (max_new_sweeps=0). In smoke mode, all variants run
% cached-only (max_new_sweeps=0).
S2 = state.scenarios(2);
hp_grids = make_hp_grids();
poc_names = fieldnames(hp_grids);
budget_per_var = state.constants.STAGE2_LIVE_BUDGET_PER_VARIANT;
if state.is_smoke
    budget_per_var = 0;
end
live_pocs = {'A', 'B', 'E', 'F'};

hp_results = repmat(struct('poc', '', 'variant', '', ...
                           'theta_star', [], 'J_star', NaN, 'J_eval', NaN, ...
                           'n_evals_live', 0, 'errored', false, 'errmsg', ''), 0, 1);
hp_traces = repmat(struct('poc', '', 'variant', '', ...
                          'eval_counts', [], 'J_star_trace', [], ...
                          'J_eval_trace', [], 'n_evals_actual', []), 0, 1);

for k = 1:numel(poc_names)
    name = poc_names{k};
    variants = hp_grids.(name);
    is_live_poc = ismember(name, live_pocs);
    for v = 1:numel(variants)
        if budget_exceeded(state.t_start, state.opts.budget_seconds)
            fprintf('  budget exceeded mid-stage2; skipping remaining variants\n');
            break
        end
        try
            if is_live_poc
                % Sweep max_new_sweeps from 0..budget_per_var; record traces.
                eval_counts = 0:budget_per_var;
                J_star_trace = nan(1, numel(eval_counts));
                J_eval_trace = nan(1, numel(eval_counts));
                n_actual_trace = zeros(1, numel(eval_counts));
                last = struct('theta_star', [], 'J_star', NaN, ...
                              'J_eval', NaN, 'n_evals_live', 0);
                for ec_i = 1:numel(eval_counts)
                    if budget_exceeded(state.t_start, state.opts.budget_seconds)
                        fprintf('  budget exceeded mid-trace at ec=%d\n', eval_counts(ec_i));
                        break
                    end
                    out = run_poc_full(name, S2.info, variants(v).hp, eval_counts(ec_i));
                    J_star_trace(ec_i) = out.J_star;
                    J_eval_trace(ec_i) = out.J_eval;
                    n_actual_trace(ec_i) = out.n_evals_live;
                    last = out;
                    fprintf('  HP %s/%s ec=%d J*=%.3f J_eval=%.3f n_live=%d\n', ...
                        name, variants(v).label, eval_counts(ec_i), ...
                        out.J_star, out.J_eval, out.n_evals_live);
                    % Refresh cache state from disk (POC may have appended entries)
                    state = refresh_cache_from_disk(state);
                end
                hp_traces(end+1) = struct('poc', name, ...
                    'variant', variants(v).label, ...
                    'eval_counts', eval_counts, ...
                    'J_star_trace', J_star_trace, ...
                    'J_eval_trace', J_eval_trace, ...
                    'n_evals_actual', n_actual_trace); %#ok<AGROW>
                hp_results(end+1) = struct('poc', name, ...
                    'variant', variants(v).label, ...
                    'theta_star', last.theta_star, ...
                    'J_star', last.J_star, ...
                    'J_eval', last.J_eval, ...
                    'n_evals_live', last.n_evals_live, ...
                    'errored', false, 'errmsg', ''); %#ok<AGROW>
            else
                out = run_poc_full(name, S2.info, variants(v).hp, 0);
                hp_results(end+1) = struct('poc', name, ...
                    'variant', variants(v).label, ...
                    'theta_star', out.theta_star, ...
                    'J_star', out.J_star, ...
                    'J_eval', out.J_eval, ...
                    'n_evals_live', out.n_evals_live, ...
                    'errored', false, 'errmsg', ''); %#ok<AGROW>
                fprintf('  HP %s/%s (cached): J*=%.3f J_eval=%.3f theta=[%.1f %.1f %.1f %.2f]\n', ...
                    name, variants(v).label, out.J_star, out.J_eval, ...
                    out.theta_star(1), out.theta_star(2), out.theta_star(3), out.theta_star(4));
            end
        catch ME
            hp_results(end+1) = struct('poc', name, ...
                'variant', variants(v).label, ...
                'theta_star', [], 'J_star', NaN, 'J_eval', NaN, ...
                'n_evals_live', 0, ...
                'errored', true, 'errmsg', ME.message); %#ok<AGROW>
            fprintf('  HP %s/%s ERROR: %s\n', name, variants(v).label, ME.message);
        end
    end
end
state.hp_results = hp_results;
state.hp_traces = hp_traces;
% Plots
try
    plot_hp_sensitivity(hp_results);
catch ME
    fprintf('plot_hp_sensitivity failed: %s\n', ME.message);
end
try
    plot_hp_live_traces(hp_traces);
catch ME
    fprintf('plot_hp_live_traces failed: %s\n', ME.message);
end
end

function state = refresh_cache_from_disk(state)
try
    S = load('centroid_lookup_log.mat');
    state.cache.lookup = S.lookup;
catch
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
ylabel('J\_eval (m)'); title('HP sensitivity (S2 default scenario, V3)');
grid on
fontsize(gcf, 12, 'points');
exportgraphics(gcf, 'figs/POC_v3_hp_sensitivity.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function plot_hp_live_traces(hp_traces)
if isempty(hp_traces), return; end
fig = figure('Visible', 'off');
pocs = unique({hp_traces.poc});
n_pocs = numel(pocs);
for p_i = 1:n_pocs
    subplot(2, n_pocs, p_i);
    poc = pocs{p_i};
    rows = hp_traces(strcmp({hp_traces.poc}, poc));
    hold on
    cmap = lines(numel(rows));
    for r = 1:numel(rows)
        ec = rows(r).eval_counts;
        plot(ec, rows(r).J_star_trace, '-o', 'Color', cmap(r,:), 'LineWidth', 1.6, ...
             'DisplayName', rows(r).variant);
    end
    title(sprintf('POC %s J\\_star', poc));
    xlabel('live-eval count');
    ylabel('J\_star');
    legend('Location', 'best', 'Interpreter', 'none');
    grid on

    subplot(2, n_pocs, n_pocs + p_i);
    hold on
    for r = 1:numel(rows)
        ec = rows(r).eval_counts;
        plot(ec, rows(r).J_eval_trace, '-s', 'Color', cmap(r,:), 'LineWidth', 1.6, ...
             'DisplayName', rows(r).variant);
    end
    title(sprintf('POC %s J\\_eval', poc));
    xlabel('live-eval count');
    ylabel('J\_eval (m)');
    legend('Location', 'best', 'Interpreter', 'none');
    grid on
end
sgtitle('HP live-eval traces (V3)');
fontsize(gcf, 11, 'points');
exportgraphics(gcf, 'figs/POC_v3_hp_live_traces.png', 'ContentType', 'image', ...
    'Width', 2400, 'Height', 1600, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function v = safe_num(x)
if isempty(x) || ~isfinite(x), v = NaN; else, v = x; end
end

function state = stage3_verify(state, smoke_only)
poc_names = {'A','B','C','D','E','F'};
n_pocs = numel(poc_names);
n_scen = numel(state.scenarios);
rows = repmat(struct('poc','','scenario','','theta_star',[], ...
                     'J_star',NaN,'J_eval',NaN,'reach_pct',NaN, ...
                     'p_centroid',[NaN NaN NaN], 'live', false, ...
                     'errored', false, 'errmsg', ''), 0, 1);

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
            res = run_poc_full(name, scen.info, hp, 0);
            row.theta_star = res.theta_star;
            row.J_star = res.J_star;

            do_live = false;
            p_centroid = [NaN NaN NaN];
            reach_pct = NaN;
            if size(swept_thetas, 1) > 0
                d = vecnorm((swept_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                [dmin, imin] = min(d);
                if dmin < state.constants.CACHE_DEDUP_EPS
                    p_centroid = swept_pcs(imin, :);
                    reach_pct = swept_reach(imin);
                end
            end

            if ~smoke_only && any(isnan(p_centroid))
                state = refresh_cache_from_disk(state);
                cache_thetas = cache_to_theta_mat(state.cache.lookup);
                d = vecnorm((cache_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                [dmin, imin] = min(d);
                if dmin < state.constants.CACHE_DEDUP_EPS
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
                    % Append to disk cache
                    new_entry = make_lookup_entry(params, out);
                    state.cache.lookup(end+1) = new_entry;
                    persist_cache(state.cache);
                end
            elseif smoke_only && any(isnan(p_centroid))
                cache_thetas = cache_to_theta_mat(state.cache.lookup);
                d = vecnorm((cache_thetas - res.theta_star) ./ state.range_vec, 2, 2);
                [~, imin] = min(d);
                p_centroid = state.cache.lookup(imin).p_centroid(:).';
                reach_pct = state.cache.lookup(imin).reachability_pct;
            end

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
exportgraphics(gcf, 'figs/POC_v3_final_eval_dist.png', 'ContentType', 'image', ...
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
for k = 1:numel(poc_names)
    for s = 1:numel(scen_set)
        ix = find(arrayfun(@(r) strcmp(r.poc, poc_names{k}) && strcmp(r.scenario, scen_set{s}) && ~r.errored, rows), 1);
        if isempty(ix), continue; end
        scatter(rows(ix).reach_pct, rows(ix).J_eval, 120, colors(s,:), ...
                markers{k}, 'filled', 'MarkerEdgeColor', 'k', ...
                'DisplayName', sprintf('%s/%s', poc_names{k}, scen_set{s}));
    end
end
xlabel('reach\_pct (fraction)'); ylabel('J\_eval (m)');
title('Pareto: J\_eval vs reach\_pct (V3)');
grid on
fontsize(gcf, 14, 'points');
exportgraphics(gcf, 'figs/POC_v3_pareto.png', 'ContentType', 'image', ...
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
title('Recommended thetas (parallel coordinates, V3)');
grid on
fontsize(gcf, 14, 'points');
exportgraphics(gcf, 'figs/POC_v3_recommendations.png', 'ContentType', 'image', ...
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
exportgraphics(gcf, 'figs/POC_v3_target_distribution.png', 'ContentType', 'image', ...
    'Width', 2000, 'Height', 1400, 'Resolution', 300, 'Padding', 40);
close(fig);
end

function save_lite(fname, varval)
data = varval; %#ok<NASGU>
save(fname, 'data');
end

function write_report(state, stage_times)
if isfield(state.opts, 'report_path') && ~isempty(state.opts.report_path)
    fname = state.opts.report_path;
else
    fname = 'docs/poc_test_report_v3.md';
end
if ~exist('docs', 'dir'), mkdir('docs'); end
fid = fopen(fname, 'w');
if fid == -1
    fprintf('FAILED to open report file %s\n', fname);
    return
end
total_time = toc(state.t_start);

fprintf(fid, '# POC Test Report V3 - Forced Exploration + Live HP Tuning\n\n');
fprintf(fid, '> Generated by `test_pocs_v3.m`. Mode: `%s`. Total wall time: %.1fs.\n\n', ...
        state.opts.mode, total_time);

% TL;DR
fprintf(fid, '## TL;DR\n\n');
fprintf(fid, '- Stage 1 forces 2 exploration thetas/POC/iter (Gaussian, 5%% of normalized box).\n');
fprintf(fid, '- Stage 2 runs A/B/E/F with `max_new_sweeps=%d` per HP variant; C/D stay cached.\n', ...
        state.constants.STAGE2_LIVE_BUDGET_PER_VARIANT);
fprintf(fid, '- Cache seed: `%s`.\n', state.opts.cache_seed);
if state.is_smoke
    fprintf(fid, '- **SMOKE MODE**: Stage 1 + Stage 3 live sweeps skipped.\n');
end
fprintf(fid, '\n');

% Cache growth
fprintf(fid, '## Cache growth summary\n\n');
fprintf(fid, '- Initial N (after seeding): %d\n', state.initial_N);
if isfield(state, 'cache')
    fprintf(fid, '- Final N: %d\n', numel(state.cache.lookup));
end
fprintf(fid, '- Live sweeps used (Stage 1): %d\n', state.live_sweeps_total_stage1);
if isfield(state, 'iter_growth') && ~isempty(state.iter_growth)
    fprintf(fid, '- Iterations: %d\n', numel(state.iter_growth));
    fprintf(fid, '\n| Iter | Best recs | Explore | Unique | After cache filter | New sweeps | Wall (s) |\n');
    fprintf(fid, '|------|-----------|---------|--------|--------------------|-----------|----------|\n');
    for it = 1:numel(state.iter_growth)
        ig = state.iter_growth(it);
        n_recs = sum(~any(isnan(ig.recs), 2));
        n_explore = sum(~any(isnan(ig.explore_thetas), 2));
        fprintf(fid, '| %d | %d | %d | %d | %d | %d | %.1f |\n', ig.iter, n_recs, ...
                n_explore, size(ig.unique_thetas, 1), ig.after_cache_filter, ...
                ig.n_new_sweeps, ig.wall);
    end
    if state.converged
        fprintf(fid, '\n*Converged: theta_best stabilized.*\n');
    end
end
fprintf(fid, '\n');

% Stage 2
fprintf(fid, '## HP tuning results (S2 scenario)\n\n');
if isfield(state, 'hp_results') && isstruct(state.hp_results)
    if numel(state.hp_results) > 0 && isfield(state.hp_results, 'poc')
        fprintf(fid, '| POC | Variant | J_star | J_eval | n_live | Theta |\n');
        fprintf(fid, '|-----|---------|--------|--------|--------|-------|\n');
        for r = 1:numel(state.hp_results)
            hp = state.hp_results(r);
            if hp.errored
                fprintf(fid, '| %s | %s | ERR | ERR | - | %s |\n', hp.poc, hp.variant, hp.errmsg);
            else
                ts = hp.theta_star;
                fprintf(fid, '| %s | %s | %.3f | %.3f | %d | [%.1f %.1f %.1f %.2f] |\n', ...
                    hp.poc, hp.variant, hp.J_star, hp.J_eval, hp.n_evals_live, ...
                    ts(1), ts(2), ts(3), ts(4));
            end
        end
    else
        fprintf(fid, '*HP tuning not run or errored.*\n');
    end
end
fprintf(fid, '\n');

% Stage 3
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

% Delta from v2
fprintf(fid, '## Delta from V2\n\n');
write_delta_v2(fid, state);
fprintf(fid, '\n');

fprintf(fid, '## Tunability summary\n\n');
fprintf(fid, 'Each POC takes scalar `lambda` (reach weight). POC D additionally takes `mu`.\n');
fprintf(fid, 'V3 enables `max_new_sweeps=%d` for A/B/E/F so HP variants differ live.\n\n', ...
        state.constants.STAGE2_LIVE_BUDGET_PER_VARIANT);

% Visualizations
fprintf(fid, '## Visualizations\n\n');
fig_files = dir('figs/POC_v3_*.png');
for f = 1:numel(fig_files)
    fprintf(fid, '- `figs/%s` -- %s\n', fig_files(f).name, caption_for_fig(fig_files(f).name));
end
fprintf(fid, '\n');

% Time budget
fprintf(fid, '## Time budget consumed (per stage)\n\n');
fprintf(fid, '| Stage | Wall (s) |\n');
fprintf(fid, '|-------|----------|\n');
fns = fieldnames(stage_times);
for k = 1:numel(fns)
    fprintf(fid, '| %s | %.2f |\n', fns{k}, stage_times.(fns{k}));
end
fprintf(fid, '| TOTAL | %.2f |\n\n', total_time);

% Limitations
fprintf(fid, '## Limitations & caveats\n\n');
if state.is_smoke
    fprintf(fid, '- **Smoke run**: no live sweeps; J_eval is from cached centroid nearest theta.\n');
end
fprintf(fid, '- POC C and D do no live sweeps; HP variants only differ in cache re-ranking.\n');
fprintf(fid, '- POC D `J_eval` reflects half_radius proxy, not classical CVaR.\n');
fprintf(fid, '- POC F surrogate predictions can extrapolate outside training data.\n\n');

% Appendix
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

function write_delta_v2(fid, state)
% Quantify cache growth, HP differentiation, and winner flips vs v2.
v2_path = 'centroid_lookup_log_v2_iter2.mat';
v2_N = NaN;
if isfile(v2_path)
    Sv2 = load(v2_path);
    v2_N = numel(Sv2.lookup);
end
final_N = numel(state.cache.lookup);
delta_N = final_N - v2_N;
fprintf(fid, '- Cache growth: V2 final N=%d -> V3 final N=%d (delta=%+d)\n', ...
        v2_N, final_N, delta_N);

% HP differentiation: count distinct (theta_star) per POC
if isfield(state, 'hp_results') && isstruct(state.hp_results) && ...
   ~isempty(state.hp_results) && isfield(state.hp_results, 'poc')
    poc_names = {'A','B','C','D','E','F'};
    fprintf(fid, '- HP variant differentiation (count of distinct theta_star per POC):\n');
    for k = 1:numel(poc_names)
        name = poc_names{k};
        ix = arrayfun(@(r) strcmp(r.poc, name) && ~r.errored, state.hp_results);
        if ~any(ix), continue; end
        ts_mat = vertcat(state.hp_results(ix).theta_star);
        if isempty(ts_mat), continue; end
        n_unique = size(unique(round(ts_mat, 2), 'rows'), 1);
        fprintf(fid, '  - POC %s: %d variants, %d distinct theta_star\n', ...
                name, sum(ix), n_unique);
    end
else
    fprintf(fid, '- HP differentiation: not measurable (no hp_results).\n');
end
end

function s = caption_for_fig(fname)
if contains(fname, 'iter_')
    s = 'Cache growth at this iteration; magenta-edge markers are new entries.';
elseif contains(fname, 'hp_sensitivity')
    s = 'Bar chart: J_eval per HP variant per POC on S2.';
elseif contains(fname, 'hp_live_traces')
    s = 'J_star and J_eval as a function of live-eval count for A/B/E/F variants.';
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
