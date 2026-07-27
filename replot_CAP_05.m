% replot_CAP_05.m -- re-render figs/CAP_05_se3_crossover_map from the saved
% merged log (logs/cap_grid_full_log.mat), no re-simulation.
%
% This started life as a hotfix: run_cap_grid_stage2.m's ylines were leaking into
% the legend as "data1"/"data2", and re-running the two-stage sweep to fix a
% legend would have cost hours. Stage 2 now sets HandleVisibility off itself, so
% this is a plain replot of the same figure -- keep it if a cheap re-render is
% wanted after a font/label tweak, drop it if the duplication annoys.
cd(fileparts(mfilename('fullpath')));   % project root = this script's folder
set_default_fonts();
L = load('logs/cap_grid_full_log.mat');
TW = @(cap) 4*5*cap / (0.8*9.81);   % T/W axis label: b=5 mixer thrust coeff, m=0.8 kg, g=9.81

col_se3 = [0.85 0.33 0.10]; col_dss = [0.00 0.30 0.70]; col_nos = [0.40 0.65 0.90];
figure('Position', [100 100 1500 560]);

% (L) Envelope
subplot(1,2,1); hold on; grid on;
plot(TW(L.ecap), L.efrc, '-o', 'Color', col_se3, 'LineWidth', 1.8, 'MarkerFaceColor', col_se3, 'DisplayName', 'SE(3)');
yline(L.dsmc_sat_env, '--', 'dSMC-sat (flat)',   'Color', col_dss, 'LineWidth', 1.5, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
yline(L.dsmc_nos_env, ':',  'dSMC-nosat (flat)', 'Color', col_nos, 'LineWidth', 1.5, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
[pk, ipk] = max(L.efrc); plot(TW(L.ecap(ipk)), pk, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.84 0], 'MarkerEdgeColor','k', 'HandleVisibility','off');
set(gca, 'XScale', 'log'); ylim([0 1]);
xlabel('Thrust-to-weight ceiling  T/W = 4b\Omega^2_{max}/(mg)');
ylabel('Envelope success fraction (/17)');
title('Envelope: SE(3) vs actuator cap'); legend('Location', 'southeast');

% (R) Landing
subplot(1,2,2); hold on; grid on;
plot(TW(L.lcap), L.lrch, '-o', 'Color', col_se3, 'LineWidth', 1.8, 'MarkerFaceColor', col_se3, 'DisplayName', 'SE(3)');
yline(L.dsmc_sat_lnd_flat, '--', 'dSMC-sat (flat, cap\geq1)', 'Color', col_dss, 'LineWidth', 1.5, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
[pk, ipk] = max(L.lrch); plot(TW(L.lcap(ipk)), pk, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.84 0], 'MarkerEdgeColor','k', 'HandleVisibility','off');
set(gca, 'XScale', 'log'); ylim([0 0.8]);
xlabel('Thrust-to-weight ceiling  T/W');
ylabel('Landing reachability (/140)');
title('Landing sweep: SE(3) vs actuator cap'); legend('Location', 'southeast');

sgtitle('SE(3) landing/envelope success vs per-rotor authority (dSMC-sat reference lines)');
export_figure('figs/CAP_05_se3_crossover_map');
fprintf('replotted figs/CAP_05_se3_crossover_map\n');
