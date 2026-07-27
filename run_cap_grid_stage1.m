% run_cap_grid_stage1.m -- Thread C, Stage 1: coarse exponential cap extension.
%
% Optional-analysis follow-up to the saturation-cap campaign: (1) sample the
% ENVELOPE metric far above cap5 (the big gap -- Phase 2 stopped at cap5 with
% SE(3) still rising 14->15/17), and (2) put a FINER cap grid on the landing
% sweep to pin the dSMC-sat<->SE(3) crossover T/W (bracketed cap1.5->2) and trace
% the SE(3) high-cap decline (only cap5 + cap100 sampled). That decline later
% turned out to be a scoring artifact rather than a controller property -- see
% section G of docs/plan_saturation_cap_campaign.md. The crossover is real.
%
% Economy: the only cap-DEPENDENT-and-interesting arm is SE(3). dSMC-sat is flat
% for cap>=1 (Phase-1/2 proven) and dSMC-nosat is structurally cap-invariant (its
% unconstrained code path never reads Omega2_max). So both are carried as
% reference lines from the committed logs; all new compute goes to SE(3).
%
% Checkpointed per cap (logs/cap_grid_stage1_ckpt.mat) -> resumable. Writes
% logs/cap_grid_stage1_log.mat and prints merged (committed + new) tables so the
% Stage-2 refinement caps can be chosen from the knees this pass reveals.

cd(fileparts(mfilename('fullpath')));   % project root = this script's folder
addpath('Ballistics_Simulation-master');
base = operational_launch();
TW   = @(cap) 4*5*cap / (0.8*9.81);   % T/W axis label: b=5 mixer thrust coeff, m=0.8 kg, g=9.81

env_caps = [0.5 1.5 8 16 32 64 100];   % envelope SE(3): extend above cap5 + crossover bisect
lnd_caps = [1.6 1.7 1.8 1.9 10 30];     % landing  SE(3): crossover pin (1.5->2) + high-end fill

ckpt = 'logs/cap_grid_stage1_ckpt.mat';
if isfile(ckpt)
    S = load(ckpt); env = S.env; lnd = S.lnd;
    fprintf('[stage1] resumed from checkpoint (%d env, %d lnd caps done)\n', ...
        numel(env.cap), numel(lnd.cap));
else
    env = struct('cap', [], 'nsucc', [], 'npts', []);
    lnd = struct('cap', [], 'reach', [], 'asc', [], 'fea', [], 'nerr', []);
end

%% ---- ENVELOPE SE(3): serial single-drone, 17 stations/cap (the priority) ----
fprintf('\n===== STAGE 1a: ENVELOPE SE(3) extension =====\n');
for cap = env_caps
    if any(abs(env.cap - cap) < 1e-9), fprintf('  env cap %.3g cached, skip\n', cap); continue; end
    t0 = tic;
    % .model here is sweep_ballistic_envelope's arm nickname; the landing loop
    % below sets the same field to a full Simulink model name. The two functions
    % read .model differently, so the values are not interchangeable.
    tp = base; tp.model = 'se3'; tp.Omega2_max = cap;
    o  = sweep_ballistic_envelope(tp);
    env.cap(end+1)   = cap;
    env.nsucc(end+1) = o.se3.n_success;
    env.npts(end+1)  = o.n_points;
    fprintf('  env SE(3) cap=%-5.3g (T/W=%5.1f): %2d/%d  (%.1f min)\n', ...
        cap, TW(cap), o.se3.n_success, o.n_points, toc(t0)/60);
    save(ckpt, 'env', 'lnd');
end

%% ---- LANDING SE(3): parsim 140-trial, crossover refine + high-end fill ----
fprintf('\n===== STAGE 1b: LANDING SE(3) crossover/fill =====\n');
for cap = lnd_caps
    if any(abs(lnd.cap - cap) < 1e-9), fprintf('  lnd cap %.3g cached, skip\n', cap); continue; end
    t0 = tic;
    tp = base; tp.model = 'se3_swarm_single'; tp.saturation_on = true; tp.Omega2_max = cap;
    o  = sweep_landing_centroid(tp, false);
    lnd.cap(end+1)   = cap;
    lnd.reach(end+1) = o.reachability_pct;
    lnd.asc(end+1)   = sum(o.deploy_success(1:3)) / max(sum(o.deploy_total(1:3)), 1);
    lnd.fea(end+1)   = sum(o.deploy_success(4:8)) / max(sum(o.deploy_total(4:8)), 1);
    lnd.nerr(end+1)  = o.n_errored;
    fprintf('  lnd SE(3) cap=%-5.3g (T/W=%5.1f): reach=%.4f (~%d/140) asc=%.3f fea=%.3f err=%d (%.1f min)\n', ...
        cap, TW(cap), o.reachability_pct, round(o.reachability_pct*140), ...
        lnd.asc(end), lnd.fea(end), o.n_errored, toc(t0)/60);
    save(ckpt, 'env', 'lnd');
end

save('logs/cap_grid_stage1_log.mat', 'env', 'lnd', 'env_caps', 'lnd_caps');

%% ---- Merge with committed anchors + print unified sorted tables ----
fprintf('\n===== MERGED PICTURE (committed anchors + Stage-1) =====\n');

% Envelope: committed run_envelope_cap_campaign.m log -> se3 row of nsucc
E = load('logs/envelope_cap_campaign_log.mat');           % caps, arm_names, nsucc, npts
se3_row = find(strcmp(E.arm_names, 'se3'));
ec = [E.caps(:); env.cap(:)];  es = [E.nsucc(se3_row,:).'; env.nsucc(:)];  ep = [E.npts(:); env.npts(:)];
[ec, k] = sort(ec); es = es(k); ep = ep(k);
fprintf('\n-- ENVELOPE SE(3) success/17 vs cap (dSMC-sat flat 9/17, dSMC-nosat flat 6/17) --\n');
fprintf('%-8s %-8s %-10s %-8s\n', 'cap', 'T/W', 'SE3', 'frac');
for i = 1:numel(ec)
    fprintf('%-8.3g %-8.1f %2d/%-7d %-8.3f\n', ec(i), TW(ec(i)), es(i), ep(i), es(i)/ep(i));
end

% Landing: committed sweep_saturation_campaign.m log -> C.reach SE(3) row (arm 3)
L = load('logs/saturation_campaign_log.mat'); C = L.C;   % C.cap_grid, C.reach [3 x n], C.arm_names
se3_l = find(strcmp(C.arm_names, 'SE(3)'));
dss_l = find(strcmp(C.arm_names, 'dSMC-sat'));
lc = [C.cap_grid(:); lnd.cap(:)];  lr = [C.reach(se3_l,:).'; lnd.reach(:)];
la = [C.asc(se3_l,:).'; lnd.asc(:)];  lf = [C.fea(se3_l,:).'; lnd.fea(:)];
[lc, k] = sort(lc); lr = lr(k); la = la(k); lf = lf(k);
dsmc_sat_flat = C.reach(dss_l, :);   % reference line
fprintf('\n-- LANDING SE(3) reachability vs cap (dSMC-sat ref: %s) --\n', num2str(dsmc_sat_flat, '%.3f '));
fprintf('%-8s %-8s %-8s %-8s %-8s\n', 'cap', 'T/W', 'reach', 'asc', 'fea');
for i = 1:numel(lc)
    fprintf('%-8.3g %-8.1f %-8.4f %-8.3f %-8.3f\n', lc(i), TW(lc(i)), lr(i), la(i), lf(i));
end

fprintf('\n===== STAGE 1 COMPLETE -- inspect knees, then choose Stage-2 refine caps =====\n');
