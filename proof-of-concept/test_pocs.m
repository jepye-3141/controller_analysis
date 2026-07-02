%TEST_POCS  Common test harness across poc_a..poc_f, cached-only.
%   Produces a comparable results table for downstream reporting.
%   Run from project root with: matlab -batch "addpath(pwd); test_pocs"

%% Pick realistic p_targets from the cache distribution.
S = load('centroid_lookup_log.mat');
lookup = S.lookup;
pcs = nan(numel(lookup), 3);
for i = 1:numel(lookup)
    if numel(lookup(i).p_centroid) == 3
        pcs(i, :) = lookup(i).p_centroid(:).';
    end
end
reach = arrayfun(@(s) s.reachability_pct, lookup);
valid = reach > 0 & all(~isnan(pcs), 2);
x_med = median(pcs(valid, 1));
x_q85 = quantile(pcs(valid, 1), 0.85);

scenarios = struct( ...
    'name',     {'T1_on_axis_lam0', 'T2_on_axis_lam1', 'T3_off_axis', 'T4_long_range'}, ...
    'p_target', {[x_med 0 0],       [x_med 0 0],       [x_med 200 0], [x_q85 0 0]},     ...
    'lambda',   {0,                 1,                 0,             0});

fprintf('Cache valid samples: %d/%d, x_median=%.1f m, x_q85=%.1f m\n', ...
        sum(valid), numel(lookup), x_med, x_q85);
fprintf('\nScenarios:\n');
for s = 1:numel(scenarios)
    fprintf('  %s: p=[%.0f %.0f %.0f]  lambda=%g\n', scenarios(s).name, ...
            scenarios(s).p_target, scenarios(s).lambda);
end

%% Run all 6 POCs across all 4 scenarios + 1 NN tunability run on POC F.
poc_names = {'A_bo_gp', 'B_cem', 'C_sobol', 'D_cvar', 'E_nn_prescreen_cem', 'F_gp_default'};
n_pocs = numel(poc_names);
n_scen = numel(scenarios);

results = repmat(struct('poc_name', '', 'scenario', '', 'theta_star', [], ...
                        'J_star', NaN, 'wall_time', NaN, ...
                        'n_evals_cached', NaN, 'n_evals_live', NaN, ...
                        'extra', struct(), 'errored', false, 'errmsg', ''), ...
                 n_pocs * n_scen + n_scen, 1);

row = 0;
for s = 1:n_scen
    pt = scenarios(s).p_target;
    lam = scenarios(s).lambda;
    sn = scenarios(s).name;

    invocations = { ...
        'A_bo_gp',             @() poc_a_bo_gp(pt, lam, 0, false);
        'B_cem',               @() poc_b_cem(pt, lam, 0, false);
        'C_sobol',             @() poc_c_sobol(pt, lam, 0, false);
        'D_cvar',              @() poc_d_cvar(pt, lam, 0, false, 1);
        'E_nn_prescreen_cem',  @() poc_e_nn_prescreen_cem(pt, lam, 0, false);
        'F_gp_default',        @() poc_f_nn_surrogate(pt, lam, 0, false, false)};

    for k = 1:size(invocations, 1)
        row = row + 1;
        results(row).poc_name = invocations{k, 1};
        results(row).scenario = sn;
        try
            [theta_star, J_star, meta] = invocations{k, 2}();
            results(row).theta_star = theta_star; results(row).J_star = J_star;
            results(row).wall_time = meta.wall_time;
            results(row).n_evals_cached = meta.n_evals_cached;
            results(row).n_evals_live = meta.n_evals_live;
            results(row).extra = extract_extra(invocations{k, 1}, meta);
            fprintf('OK  %-22s @ %-20s  J*=%8.3f  theta*=[%.1f %.1f %.1f %.2f]  wall=%.2fs\n', ...
                    results(row).poc_name, sn, J_star, theta_star, meta.wall_time);
        catch ME
            results(row).errored = true; results(row).errmsg = ME.message;
            fprintf('ERROR  %s @ %s : %s\n', invocations{k, 1}, sn, ME.message);
        end
    end
end

%% Tunability sub-test: POC F with use_nn=true on T1 (GP vs NN comparison).
T1 = scenarios(1);
row = row + 1;
results(row).poc_name = 'F_nn_fitrnet';
results(row).scenario = T1.name;
try
    [theta_star, J_star, meta] = poc_f_nn_surrogate(T1.p_target, T1.lambda, 0, false, true);
    results(row).theta_star = theta_star; results(row).J_star = J_star;
    results(row).wall_time = meta.wall_time;
    results(row).n_evals_cached = meta.n_evals_cached;
    results(row).n_evals_live = meta.n_evals_live;
    results(row).extra = extract_extra('F_nn_fitrnet', meta);
    fprintf('OK  %-22s @ %-20s  J*=%8.3f  theta*=[%.1f %.1f %.1f %.2f]  wall=%.2fs\n', ...
            results(row).poc_name, T1.name, J_star, theta_star, meta.wall_time);
catch ME
    results(row).errored = true; results(row).errmsg = ME.message;
    fprintf('ERROR  F_nn_fitrnet @ %s : %s\n', T1.name, ME.message);
end
results = results(1:row);

%% Print results table.
fprintf('\n=== Results table ===\n');
T = struct2table(rmfield(results, {'extra', 'errored', 'errmsg'}));
T.theta_Vo = arrayfun(@(r) safe_idx(r.theta_star, 1), results);
T.theta_el = arrayfun(@(r) safe_idx(r.theta_star, 2), results);
T.theta_az = arrayfun(@(r) safe_idx(r.theta_star, 3), results);
T.theta_p  = arrayfun(@(r) safe_idx(r.theta_star, 4), results);
T.theta_star = [];
disp(T(:, {'poc_name', 'scenario', 'theta_Vo', 'theta_el', 'theta_az', 'theta_p', ...
           'J_star', 'wall_time', 'n_evals_cached', 'n_evals_live'}));

%% Tunability deltas (normalized by ranges) for the six "core" POCs.
ranges = S.ranges;
range_vec = [diff(ranges.Vo), diff(ranges.el), diff(ranges.az), diff(ranges.p)];

fprintf('\n=== Tunability deltas |Theta(scenario) - Theta(T1)| / range ===\n');
fprintf('%-22s  %-12s  %-12s  %-12s\n', 'POC', 'lam0->lam1', 'on->off_axis', 'short->long');
tunability_deltas = struct();
for k = 1:numel(poc_names)
    p = poc_names{k};
    th = nan(4, 4);
    for s = 1:n_scen
        ix = find(strcmp({results.poc_name}, p) & strcmp({results.scenario}, scenarios(s).name), 1);
        if ~isempty(ix) && ~isempty(results(ix).theta_star)
            th(s, :) = results(ix).theta_star;
        end
    end
    d_lambda = mean(abs(th(2, :) - th(1, :)) ./ range_vec);
    d_offax  = mean(abs(th(3, :) - th(1, :)) ./ range_vec);
    d_long   = mean(abs(th(4, :) - th(1, :)) ./ range_vec);
    tunability_deltas.(p) = struct('d_lambda', d_lambda, 'd_offax', d_offax, 'd_long', d_long);
    fprintf('%-22s  %-12.4f  %-12.4f  %-12.4f\n', p, d_lambda, d_offax, d_long);
end

%% File sizes for "simplicity of execution" axis.
fprintf('\n=== POC file sizes (bytes) ===\n');
files = {'poc_a_bo_gp.m', 'poc_b_cem.m', 'poc_c_sobol.m', 'poc_d_cvar.m', ...
         'poc_e_nn_prescreen_cem.m', 'poc_f_nn_surrogate.m'};
for f = 1:numel(files)
    fi = dir(files{f});
    fprintf('  %-30s  %6d bytes\n', files{f}, fi.bytes);
end

%% Save consolidated results.
save('poc_test_results.mat', 'results', 'scenarios', 'tunability_deltas');
fprintf('\nSaved poc_test_results.mat (%d rows).\n', numel(results));

%% --- helpers ---
function v = safe_idx(arr, k)
if numel(arr) >= k, v = arr(k); else, v = NaN; end
end

function ex = extract_extra(name, meta)
ex = struct();
switch name
    case 'A_bo_gp'
        if isfield(meta, 'history'), ex.history_n = numel(meta.history); end
    case 'B_cem'
        if isfield(meta, 'history'), ex.history_n = numel(meta.history); end
    case 'C_sobol'
        if isfield(meta, 'sobol_indices'), ex.sobol_indices = meta.sobol_indices; end
        if isfield(meta, 'surrogate_R2'), ex.surrogate_R2 = meta.surrogate_R2; end
    case 'D_cvar'
        if isfield(meta, 'pareto'), ex.pareto = meta.pareto; end
    case 'E_nn_prescreen_cem'
        if isfield(meta, 'surrogate_R2_train'), ex.surrogate_R2_train = meta.surrogate_R2_train; end
        if isfield(meta, 'rho_S'), ex.rho_S = meta.rho_S; end
    case {'F_gp_default', 'F_nn_fitrnet'}
        if isfield(meta, 'cv_R2'),   ex.cv_R2   = meta.cv_R2; end
        if isfield(meta, 'cv_rmse'), ex.cv_rmse = meta.cv_rmse; end
        if isfield(meta, 'surrogate_type'), ex.surrogate_type = meta.surrogate_type; end
end
end
