close all; clear all; clc
Simulink.sdi.clear()
set_param(0, 'CacheFolder', '');

load_system("discrete_smc_swarm_single")
addpath("Ballistics_Simulation-master/")

ranges = struct( ...
    'Vo',   [ 80, 120], ...
    'el',   [ 35,  55], ...
    'az',   [  0,  30], ...
    'w_z0', [ -1,   2], ...
    'w_y0', [ -1,   1], ...
    'p',    [ -2,   2]);
field_names = fieldnames(ranges);
N = 20;
rng(0)
X = lhsdesign(N, length(field_names));

lookup = repmat(struct( ...
    'params',             [], ...
    'p_centroid',         [], ...
    'reachability_pct',   NaN, ...
    'half_radius',        NaN, ...
    'ballistic_solution', []), N, 1);

for n = 1:N
    params = struct('alpha_0',  2, 'beta_0', -0.5, ...
                    'x_0',      0, 'y_0',     0, 'z_0',   0, ...
                    't_max',  300);
    for k = 1:length(field_names)
        f = field_names{k};
        params.(f) = ranges.(f)(1) + X(n,k) * diff(ranges.(f));
    end
    out = sweep_landing_centroid(params, false);
    lookup(n).params             = params;
    lookup(n).p_centroid         = out.p_centroid;
    lookup(n).reachability_pct   = out.reachability_pct;
    lookup(n).half_radius        = out.radial_profile.half_radius;
    lookup(n).ballistic_solution = out.ballistic_solution;
    fprintf('LHS %d/%d: cx=%.1f cy=%.1f reach=%.2f half_r=%.1f\n', ...
        n, N, out.p_centroid(1), out.p_centroid(2), ...
        out.reachability_pct, out.radial_profile.half_radius);
end
save("logs/centroid_lookup_log.mat", "lookup", "ranges", "X", "N", "field_names")

%% 3D overlay: every sampled mortar trajectory + its landing centroid
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
