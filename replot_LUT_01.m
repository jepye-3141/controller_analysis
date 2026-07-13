% Reload centroid_lookup workspace and re-render LUT_01 with current style.
% Used after style-helper updates without re-running the LHS sweep.
% Source log is the 2026-07-11 j3_lut_regen output (N=41; post-bugsweep
% physics + retuned launch Vo=94/el=64, el box [35,70]; row 41 is the
% re-simulated reference operating point, flagged is_reference).
clear; clc; close all;
load('logs/centroid_lookup_log.mat');   % lookup, ranges, X, N, field_names

set_default_fonts();

cmap_ryg = ryg_cmap(32);
reach = [lookup.reachability_pct];

figure
axis equal
grid on
hold on

h_traj = gobjects(0);
for n = 1:N
    tr = lookup(n).ballistic_solution.trajectory;
    c_idx = max(1, min(size(cmap_ryg,1), round(reach(n) * size(cmap_ryg,1))));
    c     = cmap_ryg(c_idx, :);
    h = plot3(tr(:,10), tr(:,11), tr(:,12), '-', ...
              'Color', [c 0.4], 'LineWidth', 1.2);
    if isempty(h_traj)
        h_traj = h;
    end
end

h_cent = gobjects(0);
for n = 1:N
    pc = lookup(n).p_centroid;
    h = plot3(pc(1), pc(2), pc(3), 'p', 'MarkerSize', 18, ...
              'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
    if isempty(h_cent)
        h_cent = h;
    end
end

view(3)
colormap(gca, cmap_ryg)
clim([0 1])
cb = colorbar;
cb.Label.String = "Reachability (fraction of trials stable)";
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title(sprintf("Landing centroids over LHS sample of N=%d trajectories", N))
legend([h_traj h_cent], ["Mortar trajectory", "Landing centroid"], 'Location', 'best')
export_figure("figs/LUT_01_trajectories_and_centroids")

fprintf('Wrote figs/LUT_01_trajectories_and_centroids.png\n');
