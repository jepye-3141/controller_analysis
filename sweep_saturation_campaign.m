function C = sweep_saturation_campaign(visualize, cap_grid)
% sweep_saturation_campaign  Phase 1 of the saturation-cap campaign.
%
% Sweeps the per-rotor Omega^2 cap (constants_struct.Omega2_max) across a grid
% for three arms -- dSMC with priority-weighted allocation (saturation_on=true),
% dSMC with the naive per-rotor clip (saturation_on=false), and the SE(3)
% geometric baseline -- and records at each cap: the 140-trial landing
% reachability, the ascending(1-3)/feasible(4-8) per-station stratification, the
% landing centroid + half-radius, and the H3 clip-active fractions.
%
% Pre-registered hypotheses this sweep adjudicates:
%   H1  a tighter cap WIDENS dSMC's feasible-regime lead over SE(3) (allocation
%       matters more; SE(3)'s naive clip degrades most).
%   H2  the ascending cliff is CAP-INVARIANT (dSMC ~0, SE(3) ~31/55 across the
%       whole grid) -- it lives at Omega2_min=0 (thrust-sign wall), not the cap.
%   H3  SE(3) is cap-insensitive until its geometric law starts saturating --
%       distinguished by clip_hi (ceiling) vs clip_lo (motor cutoff) fractions.
%
% visualize : bool (default true) -> render figs/CAP_01..CAP_04 + save log.
% cap_grid  : optional override (default [0.6 1 1.5 2 3 5 100]).
%
% C : struct of n_arm x n_cap result matrices + axes/metadata, also saved to
%     logs/saturation_campaign_log.mat (checkpoint: saturation_campaign_ckpt.mat).

if nargin < 1 || isempty(visualize), visualize = true; end
if nargin < 2 || isempty(cap_grid),  cap_grid  = [0.6 1.0 1.5 2.0 3.0 5.0 100]; end

% Thrust-to-weight axis: total thrust ceiling = 4*b*Omega2_max; b=5 is the mixer
% thrust coefficient, m,g from constants.m (T/W is an axis label only).
m = 0.8; g = 9.81; b_thrust = 5;
tw_grid = 4*b_thrust*cap_grid / (m*g);

arms = struct( ...
    'name',          {'dSMC-sat', 'dSMC-naive', 'SE(3)'}, ...
    'model',         {'discrete_smc_swarm_single', 'discrete_smc_swarm_single', 'se3_swarm_single'}, ...
    'saturation_on', {true, false, true});
n_arm = numel(arms);
n_cap = numel(cap_grid);

% Committed anchors at cap=2: dSMC-sat 85/140, dSMC-naive 31/140, SE(3) 88/140.
anchor_at2 = [85 31 88];

base = operational_launch();

reach    = nan(n_arm, n_cap);
asc      = nan(n_arm, n_cap);
fea      = nan(n_arm, n_cap);
clip_hi  = nan(n_arm, n_cap);
clip_lo  = nan(n_arm, n_cap);
n_err    = nan(n_arm, n_cap);
half_r   = nan(n_arm, n_cap);
cent     = nan(n_arm, n_cap, 2);
dep_succ = nan(n_arm, n_cap, 8);
dep_tot  = nan(n_arm, n_cap, 8);

ckpt  = 'logs/saturation_campaign_ckpt.mat';
t_all = tic;
for a = 1:n_arm
    for c = 1:n_cap
        tp = base;
        tp.model         = arms(a).model;
        tp.saturation_on = arms(a).saturation_on;
        tp.Omega2_max    = cap_grid(c);
        fprintf('\n[campaign] arm=%s cap=%.3g (T/W=%.2f) ...\n', arms(a).name, cap_grid(c), tw_grid(c));
        t0 = tic;
        o  = sweep_landing_centroid(tp, false);

        reach(a,c)      = o.reachability_pct;
        n_err(a,c)      = o.n_errored;
        half_r(a,c)     = o.radial_profile.half_radius;
        clip_hi(a,c)    = o.clip_hi_frac;
        clip_lo(a,c)    = o.clip_lo_frac;
        cent(a,c,:)     = o.p_centroid(1:2);
        dep_succ(a,c,:) = o.deploy_success(:);
        dep_tot(a,c,:)  = o.deploy_total(:);
        asc(a,c) = sum(o.deploy_success(1:3)) / max(sum(o.deploy_total(1:3)), 1);
        fea(a,c) = sum(o.deploy_success(4:8)) / max(sum(o.deploy_total(4:8)), 1);

        fprintf(['[campaign] %s cap=%.3g: reach=%.4f (~%d/140)  asc=%.3f  fea=%.3f  ' ...
                 'clip_hi=%.3f clip_lo=%.3f  errs=%d  (%.1f min)\n'], ...
            arms(a).name, cap_grid(c), reach(a,c), round(reach(a,c)*140), ...
            asc(a,c), fea(a,c), clip_hi(a,c), clip_lo(a,c), n_err(a,c), toc(t0)/60);
        if abs(cap_grid(c) - 2) < 1e-9
            got = round(reach(a,c)*140);
            tag = "MATCH"; if got ~= anchor_at2(a), tag = "MISMATCH"; end
            fprintf('[campaign] ANCHOR cap=2 %s: got %d/140, expected %d/140 -- %s\n', ...
                arms(a).name, got, anchor_at2(a), tag);
        end
        save(ckpt, 'reach','asc','fea','clip_hi','clip_lo','n_err','half_r', ...
             'cent','dep_succ','dep_tot','cap_grid','tw_grid','arms','a','c');
    end
end
fprintf('\n[campaign] all %d sweeps done in %.1f min\n', n_arm*n_cap, toc(t_all)/60);

C = struct('cap_grid', cap_grid, 'tw_grid', tw_grid, 'arm_names', {{arms.name}}, ...
    'reach', reach, 'asc', asc, 'fea', fea, 'clip_hi', clip_hi, 'clip_lo', clip_lo, ...
    'n_errored', n_err, 'half_radius', half_r, 'centroid', cent, ...
    'deploy_success', dep_succ, 'deploy_total', dep_tot);
save('logs/saturation_campaign_log.mat', 'C');   % saved BEFORE figures: data is safe

if ~visualize, return; end

%% Figures
close all;
set_default_fonts();
arm_col = [0.00 0.30 0.70;   % dSMC-sat   (blue)
           0.40 0.65 0.90;   % dSMC-naive (light blue)
           0.85 0.33 0.10];  % SE(3)      (orange-red)
i_cap2 = find(abs(cap_grid - 2) < 1e-9, 1);

% CAP_01: overall reachability vs T/W
figure; hold on; grid on;
for a = 1:n_arm
    plot(tw_grid, reach(a,:), '-o', 'Color', arm_col(a,:), 'LineWidth', 1.8, ...
         'MarkerFaceColor', arm_col(a,:));
end
set(gca, 'XScale', 'log'); ylim([0 1]);
if ~isempty(i_cap2), xline(tw_grid(i_cap2), '--', 'operating cap = 2', 'Color', [.4 .4 .4]); end
xlabel('Thrust-to-weight ceiling  T/W = 4b\Omega^2_{max}/(mg)');
ylabel('Landing reachability (fraction of 140 trials)');
title('Saturation-cap sweep: reachability vs actuator authority');
legend({arms.name}, 'Location', 'southeast');
export_figure('figs/CAP_01_reach_vs_tw');

% CAP_02: stratified by deploy regime
figure; tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; hold on; grid on;
for a = 1:n_arm
    plot(tw_grid, asc(a,:), '-o', 'Color', arm_col(a,:), 'LineWidth', 1.8, 'MarkerFaceColor', arm_col(a,:));
end
set(gca, 'XScale', 'log'); ylim([0 1]);
xlabel('T/W'); ylabel('Reachability'); title('Ascending stations 1-3 (steep climb)');
legend({arms.name}, 'Location', 'best');
nexttile; hold on; grid on;
for a = 1:n_arm
    plot(tw_grid, fea(a,:), '-o', 'Color', arm_col(a,:), 'LineWidth', 1.8, 'MarkerFaceColor', arm_col(a,:));
end
set(gca, 'XScale', 'log'); ylim([0 1]);
xlabel('T/W'); title('Feasible/descending stations 4-8');
title(tl, 'Cap sweep stratified by deploy regime (H1/H2)');
export_figure('figs/CAP_02_reach_stratified');

% CAP_03: H3 clip-active fractions (solid = upper/ceiling, dashed = lower/cutoff)
figure; hold on; grid on;
h = gobjects(1, n_arm);
for a = 1:n_arm
    h(a) = plot(tw_grid, clip_hi(a,:), '-o', 'Color', arm_col(a,:), 'LineWidth', 1.8, 'MarkerFaceColor', arm_col(a,:));
    plot(tw_grid, clip_lo(a,:), '--s', 'Color', arm_col(a,:), 'LineWidth', 1.2);
end
set(gca, 'XScale', 'log');
xlabel('T/W'); ylabel('Clip-active fraction of timesteps');
title('H3: saturation activity vs cap (solid = upper/ceiling, dashed = lower/cutoff)');
legend(h, {arms.name}, 'Location', 'best');
export_figure('figs/CAP_03_clip_activity');

% CAP_04: value of the priority-weighted allocation vs cap
figure; hold on; grid on;
plot(tw_grid, reach(1,:) - reach(2,:), '-o', 'Color', [0 0.5 0], 'LineWidth', 1.8, 'MarkerFaceColor', [0 0.5 0]);
yline(0, 'k:');
set(gca, 'XScale', 'log');
xlabel('T/W'); ylabel('\Delta reachability  (dSMC-sat - dSMC-naive)');
title('Value of priority-weighted allocation vs actuator cap');
export_figure('figs/CAP_04_allocation_gap');

end
