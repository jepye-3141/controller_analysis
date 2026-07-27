% run_crit_cap_sweep.m -- re-fly the cap campaign keeping the per-trial criteria.
%
% The saturation-cap campaign concluded that SE(3)'s high-cap decline is an
% interaction with ballistic_success's velocity clause rather than instability
% (docs/plan_saturation_cap_campaign.md; sweep_saturation_campaign.m header).
% That is a claim ABOUT the success criterion, so it invites the obvious
% reviewer attack -- the headline depends on an arbitrary VelRatioMax = 2 --
% and answering it needs per-trial criteria, which no campaign log kept.
%
% What is already settled, from logs that DID keep them (2026-07-25):
% at the operating cap 2 the velocity clause is entirely non-binding. Every
% trial it rejects also fails at least one other clause, on both metrics and
% every arm, so the landing headline (85/140 vs 31/140) and the envelope masks
% (6/9/14 of 17) are identical at VelRatioMax = 2, 3, 5, 10 and Inf. The open
% question is the HIGH-cap end, where SE(3) has the authority to fly fast
% enough for the bound to start biting -- its cap-2 speed ratios already reach
% 1.72 against dSMC's 1.01. This sweep supplies that end.
%
% Why re-simulate at all. Re-scoring at another VelRatioMax is free wherever
% crit_table survived, since velocity_ok is exactly
% (max_speed_ratio <= VelRatioMax) and the table stores that column
% (rescore_criterion.m). Every cap-campaign driver discarded it --
% sweep_saturation_campaign keeps only aggregates, run_envelope_cap_campaign
% per-clause counts, the two grid stages scalars -- so the cap axis has to be
% flown again. Keeping the table this time also settles every FUTURE criterion
% question on this grid without another sweep, which is the standing backlog
% item "persist crit_table per (arm, cap)".
%
% Cost: ~1.5 h. Landing sweeps run ~1.2 min each (140-trial parsim, 12 workers),
% envelope runs ~2 min (17 serial stations); the first job of each kind pays an
% extra few minutes of pool start and rapid-accelerator build.
%
% Checkpointed per job and resumable -- delete logs/crit_cap_sweep_ckpt.mat for
% a clean restart. Writes logs/crit_cap_sweep_log.mat; interpret it with
% run_criterion_sensitivity.m.

cd(fileparts(mfilename('fullpath')));   % project root = this script's folder
addpath('Ballistics_Simulation-master');
base = operational_launch();
TW   = @(cap) 4*5*cap / (0.8*9.81);   % T/W axis label: b=5 mixer thrust coeff, m=0.8 kg, g=9.81

% The committed campaign grid [0.6 1 1.5 2 3 5 100] extended with 8/16/32,
% where stage 1 found the SE(3) envelope decline and plateau (12/17 at cap 8,
% 10/17 from cap 16 on). Same grid for both metrics so the two curves are read
% against each other without interpolation.
caps = [0.6 1 1.5 2 3 5 8 16 32 100];

% Committed cap-2 values, re-checked as each job lands. A mismatch means this
% re-fly is not the run the campaign reported and nothing downstream is
% comparable -- reported, not asserted, so one bad cell cannot kill the sweep.
anchor_env = struct('se3', 14, 'dsmc_sat', 9, 'dsmc_nosat', 6);          % of 17
anchor_lnd = struct('se3', 88, 'dsmc_sat', 85, 'dsmc_naive', 31);        % of 140

ckpt = 'logs/crit_cap_sweep_ckpt.mat';
if isfile(ckpt)
    S = load(ckpt); env = S.env; lnd = S.lnd;
    fprintf('[crit] resumed from checkpoint (%d env rows, %d lnd rows done)\n', ...
        numel(env.cap), numel(lnd.cap));
else
    env = struct('arm', strings(1,0), 'cap', [], 'nsucc', [], 'npts', [], 'crit', {{}});
    lnd = struct('arm', strings(1,0), 'cap', [], 'reach', [], 'nerr', [], 'crit', {{}});
end

t_all = tic;

%% ---- ENVELOPE: 17 serial stations per job ----
% Two jobs per cap, not three: the "dsmc" nickname expands to the nosat+sat
% sub-arm PAIR inside sweep_ballistic_envelope, so one run yields both tables.
fprintf('\n===== CRIT SWEEP 1a: ENVELOPE (%d caps x 2 jobs) =====\n', numel(caps));
for cap = caps
    for a = ["se3", "dsmc"]
        probe = "se3"; if a == "dsmc", probe = "dsmc_sat"; end
        if any(env.arm == probe & abs(env.cap - cap) < 1e-9)
            fprintf('  env %-5s cap %-5.3g cached, skip\n', a, cap); continue
        end
        t0 = tic;
        % .model is sweep_ballistic_envelope's arm NICKNAME here; the landing
        % loop below sets the same field to a full Simulink model name. The two
        % functions read .model differently -- the values are not interchangeable.
        tp = base; tp.model = char(a); tp.Omega2_max = cap;
        o  = sweep_ballistic_envelope(tp);

        got = "se3"; if a == "dsmc", got = ["dsmc_nosat", "dsmc_sat"]; end
        for g = got
            r = o.(char(g));
            env.arm(end+1)   = g;
            env.cap(end+1)   = cap;
            env.nsucc(end+1) = r.n_success;
            env.npts(end+1)  = o.n_points;
            env.crit{end+1}  = r.crit_table;
            fprintf('  env %-11s cap=%-5.3g (T/W=%6.1f): %2d/%d%s\n', ...
                g, cap, TW(cap), r.n_success, o.n_points, ...
                anchor_note(cap, r.n_success, anchor_env, g));
        end
        fprintf('      (%.1f min)\n', toc(t0)/60);
        save(ckpt, 'env', 'lnd');
    end
end

%% ---- LANDING: 140-trial parsim per job ----
lnd_arms = ["se3", "dsmc_sat", "dsmc_naive"];
lnd_model = struct('se3', 'se3_swarm_single', ...
                   'dsmc_sat', 'discrete_smc_swarm_single', ...
                   'dsmc_naive', 'discrete_smc_swarm_single');
lnd_sat = struct('se3', true, 'dsmc_sat', true, 'dsmc_naive', false);

fprintf('\n===== CRIT SWEEP 1b: LANDING (%d caps x %d arms) =====\n', numel(caps), numel(lnd_arms));
for cap = caps
    for a = lnd_arms
        if any(lnd.arm == a & abs(lnd.cap - cap) < 1e-9)
            fprintf('  lnd %-11s cap %-5.3g cached, skip\n', a, cap); continue
        end
        t0 = tic;
        tp = base;
        tp.model         = lnd_model.(char(a));
        tp.saturation_on = lnd_sat.(char(a));
        tp.Omega2_max    = cap;
        o = sweep_landing_centroid(tp, false);

        lnd.arm(end+1)   = a;
        lnd.cap(end+1)   = cap;
        lnd.reach(end+1) = o.reachability_pct;
        lnd.nerr(end+1)  = o.n_errored;
        lnd.crit{end+1}  = o.crit_table;
        fprintf('  lnd %-11s cap=%-5.3g (T/W=%6.1f): reach=%.4f (~%d/140) err=%d  (%.1f min)%s\n', ...
            a, cap, TW(cap), o.reachability_pct, round(o.reachability_pct*140), ...
            o.n_errored, toc(t0)/60, ...
            anchor_note(cap, round(o.reachability_pct*140), anchor_lnd, a));
        save(ckpt, 'env', 'lnd');
    end
end

save('logs/crit_cap_sweep_log.mat', 'env', 'lnd', 'caps');
fprintf('\n===== CRIT SWEEP COMPLETE in %.1f min -- run run_criterion_sensitivity.m =====\n', toc(t_all)/60);

function s = anchor_note(cap, got, anchors, arm)
% Compare a cap-2 job against the committed campaign value; silent elsewhere.
s = "";
if abs(cap - 2) > 1e-9 || ~isfield(anchors, char(arm)), return; end
want = anchors.(char(arm));
tag = "MATCH"; if got ~= want, tag = "MISMATCH"; end
s = sprintf('   [ANCHOR cap=2: got %d, committed %d -- %s]', got, want, tag);
end
