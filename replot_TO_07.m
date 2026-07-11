% Reload trajectory_optimization workspace and re-render only TO_07.
% Continuous landing-reachability heatmap with power-weighted centroid.
% Source log is pre-criterion (May 2026): reproduces the historical figure; the current paper figure is TO_07_sat_on.
clear; clc; close all;
load('logs/trajectory_optimization_log.mat');
cx_saved = cx;   % stash before the recompute reassigns them
cy_saved = cy;

set_default_fonts();

stable_arr = reshape([results.stable], n_deploy, n_ct, n_nb);

cmap_ryg = ryg_cmap(32);

%% Landing-point reachability (recompute from results)
landing_success = zeros(n_deploy, n_ct);
landing_total   = zeros(n_deploy, n_ct);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            tgt = results(i, k, nb).target_deploy_idx;
            if isnan(tgt)
                continue
            end
            landing_total(tgt, k) = landing_total(tgt, k) + 1;
            if stable_arr(i, k, nb)
                landing_success(tgt, k) = landing_success(tgt, k) + 1;
            end
        end
    end
end
landing_ratio = landing_success ./ max(landing_total, 1);

land_x_flat = reshape(land_pos(1,:,:), [], 1);
land_y_flat = reshape(land_pos(2,:,:), [], 1);
ratio_flat  = landing_ratio(:);

%% Continuous heatmap via scattered interpolation
F = scatteredInterpolant(land_x_flat, land_y_flat, ratio_flat, 'natural', 'none');
n_grid = 200;
[Xg, Yg] = meshgrid(linspace(min(land_x_flat), max(land_x_flat), n_grid), ...
                    linspace(min(land_y_flat), max(land_y_flat), n_grid));
Rg = F(Xg, Yg);
Zg = zeros(size(Xg));

%% Power-weighted centroid (p = 2): pulled toward the high-reachability core
p_centroid = 2;
W = max(Rg, 0).^p_centroid;
W(isnan(Rg)) = 0;
tot_w = sum(W(:));
assert(tot_w > 0, 'TO_07: heatmap has no positive reachability; centroid undefined');
cx = sum(Xg(:) .* W(:)) / tot_w;
cy = sum(Yg(:) .* W(:)) / tot_w;
cz = 0;
assert(abs(cx - cx_saved) < 0.5 && abs(cy - cy_saved) < 0.5, ...
    'replot_TO_07: recomputed centroid (%.2f, %.2f) drifted from saved (%.2f, %.2f); check scatteredInterpolant method matches sweep_landing_centroid.m', ...
    cx, cy, cx_saved, cy_saved);
fprintf('TO_07 centroid (p=%d): x = %.2f m, y = %.2f m\n', p_centroid, cx, cy);

%% Render
figure
axis equal
grid on
hold on

h_surf = surf(Xg, Yg, Zg, Rg, 'EdgeColor', 'none', 'FaceColor', 'interp', ...
              'HandleVisibility', 'off');

h_traj = plot3(ballistic_solution.trajectory(:,10), ...
               ballistic_solution.trajectory(:,11), ...
               ballistic_solution.trajectory(:,12), '-b', 'LineWidth', 2);

h_dep = scatter3(deploy_pos(1,:), deploy_pos(2,:), deploy_pos(3,:), ...
                 40, 'k', 'filled');

h_cent = plot3(cx, cy, cz, 'p', 'MarkerSize', 28, 'MarkerFaceColor', 'm', ...
               'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

colormap(gca, cmap_ryg)
clim([0 1])
cb = colorbar;
cb.Label.String = "Landing-point success ratio";
view(3)
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC landing-reachability heatmap and centroid")
legend([h_traj h_dep h_cent], ...
       ["Mission trajectory", "Deploy points", ...
        sprintf("Centroid (p=%d)", p_centroid)], 'Location', 'best')
export_figure("figs/TO_07_landing_heatmap", EPSContentType="image")

fprintf('Wrote figs/TO_07_landing_heatmap.png\n');
