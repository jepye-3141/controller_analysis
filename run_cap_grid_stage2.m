% run_cap_grid_stage2.m -- Thread C, Stage 2: refine near the Stage-1 knees.
%
% Stage 1 revealed (SE(3), the only cap-dependent arm):
%   envelope: crossover ~cap1.5, PEAK ~cap5 (15/17), plateau 10/17 for cap>=16
%   landing : crossover  cap1.8 (=dSMC-sat 0.607), PEAK cap3 (0.743), plateau 0.636
% This pass localizes the two peaks and the envelope SE(3) turn-on (cap1->1.5 is
% 0->9/17, a near-cliff), then renders the final CAP_05 crossover-map figure over
% the full merged grid (committed anchors + Stage-1 + Stage-2).
%
% Read the peaks as scores, not as flight behavior. The 2026-07-25 criterion
% sweep (run_criterion_sensitivity.m) showed both are artifacts of
% ballistic_success's VelRatioMax=2: relax the bound and SE(3) is monotone in cap
% with no peak on either metric. The crossover IS real -- it sits below the
% criterion-sensitivity onset and reproduces at every bound -- so this script's
% actual deliverable, the crossover map, stands. Section G of
% docs/plan_saturation_cap_campaign.md has the numbers and the validity bound.

cd(fileparts(mfilename('fullpath')));   % project root = this script's folder
addpath('Ballistics_Simulation-master');
base = operational_launch();
TW   = @(cap) 4*5*cap / (0.8*9.81);   % T/W axis label: b=5 mixer thrust coeff, m=0.8 kg, g=9.81

env2_caps = [1.25 3 4 6];   % envelope: turn-on sharpness + peak localization (2<->8)
lnd2_caps = [2.5 4];        % landing : peak shape around cap3

ckpt = 'logs/cap_grid_stage2_ckpt.mat';
if isfile(ckpt)
    S = load(ckpt); env2 = S.env2; lnd2 = S.lnd2;
    fprintf('[stage2] resumed (%d env, %d lnd caps done)\n', numel(env2.cap), numel(lnd2.cap));
else
    env2 = struct('cap', [], 'nsucc', [], 'npts', []);
    lnd2 = struct('cap', [], 'reach', [], 'asc', [], 'fea', [], 'nerr', []);
end

fprintf('\n===== STAGE 2a: ENVELOPE SE(3) peak/turn-on =====\n');
for cap = env2_caps
    if any(abs(env2.cap - cap) < 1e-9), fprintf('  env cap %.3g cached, skip\n', cap); continue; end
    t0 = tic;
    tp = base; tp.model = 'se3'; tp.Omega2_max = cap;
    o  = sweep_ballistic_envelope(tp);
    env2.cap(end+1) = cap; env2.nsucc(end+1) = o.se3.n_success; env2.npts(end+1) = o.n_points;
    fprintf('  env SE(3) cap=%-5.3g (T/W=%5.1f): %2d/%d  (%.1f min)\n', ...
        cap, TW(cap), o.se3.n_success, o.n_points, toc(t0)/60);
    save(ckpt, 'env2', 'lnd2');
end

fprintf('\n===== STAGE 2b: LANDING SE(3) peak shape =====\n');
for cap = lnd2_caps
    if any(abs(lnd2.cap - cap) < 1e-9), fprintf('  lnd cap %.3g cached, skip\n', cap); continue; end
    t0 = tic;
    tp = base; tp.model = 'se3_swarm_single'; tp.saturation_on = true; tp.Omega2_max = cap;
    o  = sweep_landing_centroid(tp, false);
    lnd2.cap(end+1) = cap; lnd2.reach(end+1) = o.reachability_pct;
    lnd2.asc(end+1) = sum(o.deploy_success(1:3))/max(sum(o.deploy_total(1:3)),1);
    lnd2.fea(end+1) = sum(o.deploy_success(4:8))/max(sum(o.deploy_total(4:8)),1);
    lnd2.nerr(end+1) = o.n_errored;
    fprintf('  lnd SE(3) cap=%-5.3g (T/W=%5.1f): reach=%.4f (~%d/140) asc=%.3f fea=%.3f err=%d (%.1f min)\n', ...
        cap, TW(cap), o.reachability_pct, round(o.reachability_pct*140), lnd2.asc(end), lnd2.fea(end), o.n_errored, toc(t0)/60);
    save(ckpt, 'env2', 'lnd2');
end

%% ---- Merge everything: committed anchors + Stage-1 + Stage-2 ----
E  = load('logs/envelope_cap_campaign_log.mat');   % envelope anchors caps [1 2 5]
S1 = load('logs/cap_grid_stage1_log.mat');         % env, lnd (Stage 1)
C  = load('logs/saturation_campaign_log.mat'); C = C.C;   % landing anchors
se3E = find(strcmp(E.arm_names, 'se3'));
se3L = find(strcmp(C.arm_names, 'SE(3)'));
dssL = find(strcmp(C.arm_names, 'dSMC-sat'));

% Envelope SE(3) fraction over the full union grid
ecap = [E.caps(:); S1.env.cap(:); env2.cap(:)];
efrc = [E.nsucc(se3E,:).'./E.npts(:); S1.env.nsucc(:)./S1.env.npts(:); env2.nsucc(:)./env2.npts(:)];
[ecap, k] = unique(round(ecap,4)); efrc = efrc(k);
% dSMC reference lines (flat 9/17 and 6/17). Both arms are cap-flat -- dSMC-sat
% empirically for cap>=1, dSMC-nosat structurally, since its unclipped code path
% never reads Omega2_max -- so any cap's value is THE value. Assert that instead
% of averaging on faith: a mean over a non-flat arm draws a reference line at a
% cap nothing was ever run at. Rows looked up by name, like se3E/se3L above.
sat_frac_env = E.nsucc(strcmp(E.arm_names, 'dsmc_sat'),   :) ./ E.npts;
nos_frac_env = E.nsucc(strcmp(E.arm_names, 'dsmc_nosat'), :) ./ E.npts;
assert(max(sat_frac_env) - min(sat_frac_env) < 1e-9 && ...
       max(nos_frac_env) - min(nos_frac_env) < 1e-9, ...
    'run_cap_grid_stage2:notFlat', ...
    'dSMC envelope arms vary with cap (sat %s, nosat %s) -- one reference line misrepresents them', ...
    mat2str(sat_frac_env, 3), mat2str(nos_frac_env, 3));
dsmc_sat_env = sat_frac_env(1);
dsmc_nos_env = nos_frac_env(1);

% Landing SE(3) reachability over the full union grid
lcap = [C.cap_grid(:); S1.lnd.cap(:); lnd2.cap(:)];
lrch = [C.reach(se3L,:).'; S1.lnd.reach(:); lnd2.reach(:)];
[lcap, k] = unique(round(lcap,4)); lrch = lrch(k);
dsmc_sat_lnd_flat = C.reach(dssL, find(abs(C.cap_grid-2)<1e-9));   % 0.607 (cap>=1 plateau)

save('logs/cap_grid_full_log.mat', 'ecap','efrc','dsmc_sat_env','dsmc_nos_env', ...
     'lcap','lrch','dsmc_sat_lnd_flat', 'env2','lnd2');

%% ---- Figure CAP_05: SE(3)-vs-actuator-cap crossover map (both metrics) ----
set_default_fonts();
col_se3 = [0.85 0.33 0.10];  col_dss = [0.00 0.30 0.70];  col_nos = [0.40 0.65 0.90];
figure('Position', [100 100 1500 560]);

% (L) Envelope. The two ylines are HandleVisibility off: they carry inline text
% labels already, and without this they enter the legend as "data1"/"data2".
subplot(1,2,1); hold on; grid on;
plot(TW(ecap), efrc, '-o', 'Color', col_se3, 'LineWidth', 1.8, 'MarkerFaceColor', col_se3, 'DisplayName', 'SE(3)');
yline(dsmc_sat_env, '--', 'dSMC-sat (flat)', 'Color', col_dss, 'LineWidth', 1.5, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
yline(dsmc_nos_env, ':',  'dSMC-nosat (flat)', 'Color', col_nos, 'LineWidth', 1.5, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
[pk, ipk] = max(efrc); plot(TW(ecap(ipk)), pk, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.84 0], 'MarkerEdgeColor','k', 'HandleVisibility','off');
set(gca, 'XScale', 'log'); ylim([0 1]);
xlabel('Thrust-to-weight ceiling  T/W = 4b\Omega^2_{max}/(mg)');
ylabel('Envelope success fraction (/17)');
title('Envelope: SE(3) vs actuator cap');
legend('Location', 'southeast');

% (R) Landing
subplot(1,2,2); hold on; grid on;
plot(TW(lcap), lrch, '-o', 'Color', col_se3, 'LineWidth', 1.8, 'MarkerFaceColor', col_se3, 'DisplayName', 'SE(3)');
yline(dsmc_sat_lnd_flat, '--', 'dSMC-sat (flat, cap\geq1)', 'Color', col_dss, 'LineWidth', 1.5, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
[pk, ipk] = max(lrch); plot(TW(lcap(ipk)), pk, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.84 0], 'MarkerEdgeColor','k', 'HandleVisibility','off');
set(gca, 'XScale', 'log'); ylim([0 0.8]);
xlabel('Thrust-to-weight ceiling  T/W');
ylabel('Landing reachability (/140)');
title('Landing sweep: SE(3) vs actuator cap');
legend('Location', 'southeast');

sgtitle('SE(3) landing/envelope success vs per-rotor authority (dSMC-sat reference lines)');
export_figure('figs/CAP_05_se3_crossover_map');

%% ---- Final merged tables ----
fprintf('\n===== FULL MERGED PICTURE (committed + Stage-1 + Stage-2) =====\n');
fprintf('\n-- ENVELOPE SE(3) frac (dSMC-sat %.3f, dSMC-nosat %.3f) --\n', dsmc_sat_env, dsmc_nos_env);
fprintf('%-8s %-8s %-8s\n', 'cap','T/W','SE3');
for i=1:numel(ecap), fprintf('%-8.3g %-8.1f %-8.3f\n', ecap(i), TW(ecap(i)), efrc(i)); end
fprintf('\n-- LANDING SE(3) reach (dSMC-sat flat %.3f) --\n', dsmc_sat_lnd_flat);
fprintf('%-8s %-8s %-8s\n', 'cap','T/W','reach');
for i=1:numel(lcap), fprintf('%-8.3g %-8.1f %-8.4f\n', lcap(i), TW(lcap(i)), lrch(i)); end
fprintf('\n===== STAGE 2 COMPLETE -- wrote figs/CAP_05_se3_crossover_map + logs/cap_grid_full_log.mat =====\n');
