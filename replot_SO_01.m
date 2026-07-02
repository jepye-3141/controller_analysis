% Reload surrogate_optimize workspace and re-render SO_01 with current style.
% Used after style-helper updates without re-fitting the GPs or re-running
% the ground-truth verification sweep.
clear; clc; close all;
load('logs/surrogate_optimize_log.mat');
% Provides: gp_cx, gp_cy, gp_reach, gp_hr, theta_best, theta_phys, J_best,
%           p_target, lambda, ranges, field_names, Xn, y_cx, y_cy, y_reach, y_hr

set_default_fonts();

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
cb.Label.String = "J(\theta) -- lower is better";
title(tl, sprintf('Surrogate J slices, p_{target} = (%g, %g) m, \\lambda = %g', ...
                  p_target(1), p_target(2), lambda))
export_figure("figs/SO_01_predicted_J_slices")

fprintf('Wrote figs/SO_01_predicted_J_slices.png\n');
