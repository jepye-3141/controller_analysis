% Reload trajectory_optimization workspace and re-render only TO_04.
% Used for visualization tweaks without re-running the dSMC sweep.
% Source log is pre-criterion (May 2026): reproduces the historical figure; the current paper figure is TO_04_sat_on.
clear; clc; close all;
load('logs/trajectory_optimization_log.mat');

set_default_fonts();

stable_arr = reshape([results.stable], n_deploy, n_ct, n_nb);

n_half   = 32;
cmap_ryg = ryg_cmap(n_half);

%% Landing-point reachability
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

% first_reach_deploy(tgt,k): earliest deploy index to reach each landing point (drawn bold below).
first_reach_deploy = nan(n_deploy, n_ct);
for tgt = 1:n_deploy
    for k = 1:n_ct
        candidates = [];
        for nb = 1:n_nb
            i_src = tgt - neighbor_offsets(nb);
            if i_src < 1 || i_src > n_deploy
                continue
            end
            if stable_arr(i_src, k, nb)
                candidates(end+1) = i_src; %#ok<SAGROW>
            end
        end
        if ~isempty(candidates)
            first_reach_deploy(tgt, k) = min(candidates);
        end
    end
end

figure
axis equal
grid on
hold on
h_traj = plot3(ballistic_solution.trajectory(:,10), ...
               ballistic_solution.trajectory(:,11), ...
               ballistic_solution.trajectory(:,12), '-b', 'LineWidth', 2);
h_first = gobjects(0);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            tgt = results(i, k, nb).target_deploy_idx;
            if isnan(tgt)
                continue
            end
            is_first = stable_arr(i, k, nb) && (i == first_reach_deploy(tgt, k));
            if is_first
                line_color = [0 0 0];
                lw = 2.5;
                ls = '-';
            elseif stable_arr(i, k, nb)
                line_color = [0 0.6 0 0.5];
                lw = 0.8;
                ls = ':';
            else
                line_color = [0.8 0 0 0.5];
                lw = 0.8;
                ls = ':';
            end
            h_line = plot3([deploy_pos(1,i) land_pos(1,tgt,k)], ...
                  [deploy_pos(2,i) land_pos(2,tgt,k)], ...
                  [deploy_pos(3,i) land_pos(3,tgt,k)], ...
                  ls, 'Color', line_color, 'LineWidth', lw, 'HandleVisibility', 'off');
            if is_first && isempty(h_first)
                set(h_line, 'HandleVisibility', 'on');
                h_first = h_line;
            end
        end
    end
end
land_x_flat = reshape(land_pos(1,:,:), [], 1);
land_y_flat = reshape(land_pos(2,:,:), [], 1);
land_z_flat = reshape(land_pos(3,:,:), [], 1);
ratio_flat  = landing_ratio(:);

% Magenta band below the ryg ramp marks ratio == 0 (no deploy reached that point); zeros remap into the band.
n_ryg     = size(cmap_ryg, 1);
n_band    = 4;
zero_band = repmat([1 0 1], n_band, 1);
ryg_with_zero = [zero_band; cmap_ryg];
band_frac = n_band / n_ryg;

cdata_flat = ratio_flat;
cdata_flat(ratio_flat == 0) = -band_frac/2;

scatter3(land_x_flat, land_y_flat, land_z_flat, 160, cdata_flat, ...
    'filled', 'MarkerEdgeColor', 'k')
colormap(gca, ryg_with_zero)
clim([-band_frac, 1])
cb = colorbar;
cb.Ticks      = [-band_frac/2, 1/n_ryg, 0.5, 1];
cb.TickLabels = {'0', '0+', '0.5', '1'};
cb.Label.String = "Landing-point success ratio";
view(3)
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC landing reachability (each target attempted by neighboring deploys)")
if isgraphics(h_first)
    legend([h_traj h_first], ["Mission trajectory", "First reaching deploy"], ...
           'Location', 'best')
else
    legend(h_traj, "Mission trajectory", 'Location', 'best')
end
export_figure("figs/TO_04_landing_envelope")

fprintf('Wrote figs/TO_04_landing_envelope.png\n');
