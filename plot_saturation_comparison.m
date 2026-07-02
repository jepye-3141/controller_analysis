function plot_saturation_comparison(out_on, out_off)
% plot_saturation_comparison  Side-by-side summary of saturation ON vs OFF
% sweep results.
%
% out_on, out_off : structs returned by sweep_landing_centroid for the
%                   saturation-ON and saturation-OFF runs respectively.
%
% Writes figs/SAT_CMP_summary.{png,eps}.

set_default_fonts();

c_on  = [0.10 0.30 0.85];   % blue
c_off = [0.85 0.20 0.20];   % red

figure
tl = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
title(tl, "Plan A+ dSMC: saturation ON vs OFF", 'FontName', 'Times', 'FontSize', 18);

%% Tile (1,1): radial reachability profile overlay
nexttile
hold on; grid on
plot(out_on.radial_profile.r_bins,  out_on.radial_profile.mean_ratio, ...
     'Color', c_on,  'LineWidth', 2.0, 'DisplayName', 'sat ON');
plot(out_off.radial_profile.r_bins, out_off.radial_profile.mean_ratio, ...
     'Color', c_off, 'LineWidth', 2.0, 'DisplayName', 'sat OFF');

% Half-radius markers
yl = ylim;
if ~isnan(out_on.radial_profile.half_radius)
    xline(out_on.radial_profile.half_radius,  '--', ...
        'Color', c_on,  'LineWidth', 1.2, 'HandleVisibility', 'off');
end
if ~isnan(out_off.radial_profile.half_radius)
    xline(out_off.radial_profile.half_radius, '--', ...
        'Color', c_off, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
ylim(yl);

xlabel("Radius from centroid (m)")
ylabel("Mean success ratio")
title("Radial reachability profile")
legend('Location', 'best')
fontsize(gca, 14, 'points')

%% Tile (1,2): XY centroid scatter on planned ballistic groundtrack
nexttile
hold on; grid on; axis equal

gt = out_on.ballistic_solution.trajectory(:, 10:11);
plot(gt(:, 1), gt(:, 2), 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, ...
     'DisplayName', 'planned ballistic groundtrack');
plot(gt(end, 1), gt(end, 2), 'kp', 'MarkerSize', 14, ...
     'MarkerFaceColor', [1 1 0.4], 'DisplayName', 'planned impact');

plot(out_on.p_centroid(1),  out_on.p_centroid(2),  'o', ...
     'MarkerSize', 14, 'MarkerEdgeColor', c_on, ...
     'MarkerFaceColor', c_on,  'DisplayName', 'centroid (sat ON)');
plot(out_off.p_centroid(1), out_off.p_centroid(2), 'o', ...
     'MarkerSize', 14, 'MarkerEdgeColor', c_off, ...
     'MarkerFaceColor', c_off, 'DisplayName', 'centroid (sat OFF)');

xlabel("X (m)")
ylabel("Y (m)")
title("Landing centroids vs planned impact")
legend('Location', 'best')
fontsize(gca, 14, 'points')

%% Tile (2,1): stats panel — sat ON
nexttile
axis off
text(0.02, 0.95, "Saturation ON", ...
    'FontName', 'Times', 'FontSize', 18, 'FontWeight', 'bold', ...
    'Color', c_on);
text(0.02, 0.55, stats_block(out_on), ...
    'FontName', 'Times', 'FontSize', 14, 'VerticalAlignment', 'top', ...
    'Interpreter', 'none');

%% Tile (2,2): stats panel — sat OFF
nexttile
axis off
text(0.02, 0.95, "Saturation OFF", ...
    'FontName', 'Times', 'FontSize', 18, 'FontWeight', 'bold', ...
    'Color', c_off);
text(0.02, 0.55, stats_block(out_off), ...
    'FontName', 'Times', 'FontSize', 14, 'VerticalAlignment', 'top', ...
    'Interpreter', 'none');

export_figure("figs/SAT_CMP_summary")

end


function s = stats_block(out)
    p  = out.p_centroid;
    hr = out.radial_profile.half_radius;
    if isnan(hr)
        hr_str = 'n/a';
    else
        hr_str = sprintf('%.2f m', hr);
    end
    s = sprintf([ ...
        'reachability_pct  = %.3f\n' ...
        'centroid (x, y)   = (%.2f, %.2f) m\n' ...
        'peak_ratio        = %.3f\n' ...
        'half_radius       = %s'], ...
        out.reachability_pct, p(1), p(2), ...
        out.radial_profile.peak_ratio, hr_str);
end
