close all; clear all; clc
Simulink.sdi.clear()
set_param(0, 'CacheFolder', '');

set_default_fonts();

%% Operational inputs (edit per mission)
p_target = [600; 0; 0];   % target landing centroid (m)
lambda   = 50;            % weight on half_radius (m); J = ||c - p_t||^2 - lambda*half_r
verify   = true;          % run one ground-truth sweep at the surrogate optimum (~16 min)

%% Load LHS training data (canonical log regenerated 2026-07-11 by j3_lut_regen.m, N=41)
% WARNING -- do NOT re-run this script as-is. The canonical
% logs/surrogate_optimize_log.mat is the PROMOTED area_proxy refit
% (re-promoted 2026-07-11 on the retuned N=41 LUT, plan J3); this script's clamp-mode fit would overwrite it and
% regress the promotion (it also overwrites figs/SO_01). Use
% j3_surrogate_refit.m for fitting, replot_SO_01.m to re-render SO_01, or
% port the area_proxy y_hr block + J here first (plan J3 delta W1.1).
load("logs/centroid_lookup_log.mat")   % lookup, ranges, N, X, field_names

%% Build training matrix in normalised [0,1]^d coordinates
d  = length(field_names);
Xn = zeros(N, d);
for k = 1:d
    fk      = field_names{k};
    Xn(:,k) = (arrayfun(@(s) s.params.(fk), lookup) - ranges.(fk)(1)) / diff(ranges.(fk));
end
y_cx    = arrayfun(@(s) s.p_centroid(1), lookup);
y_cy    = arrayfun(@(s) s.p_centroid(2), lookup);
y_reach = [lookup.reachability_pct].';
y_hr    = [lookup.half_radius].';

assert(all(isfinite(y_cx)) && all(isfinite(y_cy)) && all(isfinite(y_reach)), ...
    'surrogate_optimize: lookup table contains non-finite cx/cy/reach values; LHS sweep produced degenerate trials');

% NaN half_radius = radial profile never decayed below 0.5*peak inside the grid;
% clamp above the max finite value so the surrogate reads "at least this wide".
hr_max = max(y_hr, [], 'omitnan');
assert(~isnan(hr_max), 'surrogate_optimize: all half_radius samples are NaN; lookup table degenerate');
y_hr(isnan(y_hr)) = 1.2 * hr_max;

%% Fit GP surrogates
% fitrgp (Stats & ML Toolbox); hyperparameters by marginal likelihood.
% Sacks et al., Statistical Science 4(4), 1989; Rasmussen & Williams 2006.
gp_opts = {'KernelFunction', 'ardmatern52', ...
           'Standardize', true, ...
           'BasisFunction', 'constant'};
gp_cx    = fitrgp(Xn, y_cx,    gp_opts{:});
gp_cy    = fitrgp(Xn, y_cy,    gp_opts{:});
gp_reach = fitrgp(Xn, y_reach, gp_opts{:});
gp_hr    = fitrgp(Xn, y_hr,    gp_opts{:});

%% Multi-start surrogate optimisation (fmincon-SQP on the GP mean)
% Marler & Arora, SMO 26, 2004 (scalarised multi-objective).
% Only (cx,cy) and half_radius enter J: gp_reach is fit/reported but unused; p_target(3) ignored.
J_obj = @(t) (predict(gp_cx, t) - p_target(1))^2 + ...
             (predict(gp_cy, t) - p_target(2))^2 - ...
             lambda * predict(gp_hr, t);

rng(0)
n_starts  = 8;
starts    = lhsdesign(n_starts, d);
fmin_opts = optimoptions('fmincon', 'Display', 'off', ...
                         'Algorithm', 'sqp', ...
                         'MaxFunctionEvaluations', 500);

theta_best = [];
J_best     = inf;
for s = 1:n_starts
    [theta_s, J_s] = fmincon(J_obj, starts(s,:), [], [], [], [], ...
                             zeros(1,d), ones(1,d), [], fmin_opts);
    if J_s < J_best
        J_best     = J_s;
        theta_best = theta_s;
    end
end

theta_phys = struct();
for k = 1:d
    theta_phys.(field_names{k}) = ranges.(field_names{k})(1) + theta_best(k) * diff(ranges.(field_names{k}));
end

cx_pred = predict(gp_cx,    theta_best);
cy_pred = predict(gp_cy,    theta_best);
r_pred  = predict(gp_reach, theta_best);
hr_pred = predict(gp_hr,    theta_best);

fprintf('\n--- Surrogate optimum (J* = %.3f) ---\n', J_best);
for k = 1:d
    fprintf('  %-6s = %8.3f\n', field_names{k}, theta_phys.(field_names{k}));
end
fprintf('  pred  (cx, cy) = (%.2f, %.2f) m\n', cx_pred, cy_pred);
fprintf('  pred  reach    = %.3f\n', r_pred);
fprintf('  pred  half_r   = %.1f m\n', hr_pred);

%% Predicted-J contour slices over the four primary launch knobs
% w_z0, w_y0 held at theta_best (later promotable to disturbance variables).
cmap_ryg = ryg_cmap(32);

primary  = {'Vo', 'el', 'az', 'p'};
prim_idx = cellfun(@(f) find(strcmp(field_names, f)), primary);
pairs    = nchoosek(prim_idx, 2);
n_grid   = 40;

figure
tl = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
for pp = 1:size(pairs, 1)
    i = pairs(pp, 1);
    j = pairs(pp, 2);
    [Gi, Gj] = meshgrid(linspace(0, 1, n_grid), linspace(0, 1, n_grid));
    G        = repmat(theta_best, n_grid^2, 1);
    G(:, i)  = Gi(:);
    G(:, j)  = Gj(:);
    JJ = (predict(gp_cx, G) - p_target(1)).^2 + ...
         (predict(gp_cy, G) - p_target(2)).^2 - ...
         lambda * predict(gp_hr, G);
    JJ = reshape(JJ, n_grid, n_grid);

    X_phys = Gi * diff(ranges.(field_names{i})) + ranges.(field_names{i})(1);
    Y_phys = Gj * diff(ranges.(field_names{j})) + ranges.(field_names{j})(1);

    nexttile
    contourf(X_phys, Y_phys, JJ, 20, 'LineColor', 'none')
    hold on
    plot(theta_phys.(field_names{i}), theta_phys.(field_names{j}), 'p', ...
         'MarkerSize', 14, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k')
    xlabel(field_names{i})
    ylabel(field_names{j})
end
colormap(flipud(cmap_ryg))
cb = colorbar;
cb.Layout.Tile = 'east';
cb.Label.String = "J(\theta) — lower is better";
title(tl, sprintf('Surrogate J slices, p_{target} = (%g, %g) m, \\lambda = %g', ...
                  p_target(1), p_target(2), lambda))
export_figure("figs/SO_01_predicted_J_slices")

%% Save log
% WARNING: overwrites the promoted area_proxy canonical log -- see header note.
save("logs/surrogate_optimize_log.mat", ...
     "gp_cx", "gp_cy", "gp_reach", "gp_hr", ...
     "theta_best", "theta_phys", "J_best", ...
     "p_target", "lambda", "ranges", "field_names", "Xn", ...
     "y_cx", "y_cy", "y_reach", "y_hr");

%% Ground-truth verification (~16 min)
if verify
    load_system("discrete_smc_swarm_single")
    addpath("Ballistics_Simulation-master/")

    verify_params         = theta_phys;
    verify_params.alpha_0 = 2;
    verify_params.beta_0  = -0.5;
    verify_params.x_0     = 0;
    verify_params.y_0     = 0;
    verify_params.z_0     = 0;
    verify_params.t_max   = 300;

    out_v = sweep_landing_centroid(verify_params, false);
    fprintf('\n--- Ground-truth verification ---\n');
    fprintf('  actual (cx, cy) = (%.2f, %.2f) m   pred (%.2f, %.2f)\n', ...
            out_v.p_centroid(1), out_v.p_centroid(2), cx_pred, cy_pred);
    fprintf('  centroid error  = %.2f m\n', ...
            norm(out_v.p_centroid(1:2) - [cx_pred; cy_pred]));
    fprintf('  actual reach    = %.3f                pred %.3f\n', ...
            out_v.reachability_pct, r_pred);
    fprintf('  actual half_r   = %.1f m              pred %.1f\n', ...
            out_v.radial_profile.half_radius, hr_pred);

    centroid_err = norm(out_v.p_centroid(1:2) - [cx_pred; cy_pred]);
    assert(centroid_err < 100, ...
        'surrogate_optimize: verify centroid error %.2f m exceeds 100 m tolerance', centroid_err);
    assert(abs(out_v.reachability_pct - r_pred) < 0.15, ...
        'surrogate_optimize: verify reachability error %.3f exceeds 0.15', ...
        abs(out_v.reachability_pct - r_pred));
    if ~isnan(out_v.radial_profile.half_radius) && ~isnan(hr_pred)
        assert(abs(out_v.radial_profile.half_radius - hr_pred) < 100, ...
            'surrogate_optimize: verify half_r error %.1f m exceeds 100 m', ...
            abs(out_v.radial_profile.half_radius - hr_pred));
    end
end
