% j3_lut_regen.m  Step-zero LUT regeneration (plan J3 W0), checkpointed + resumable.
%
% Regenerates logs/centroid_lookup_log.mat on the current eom2 (post 2026-07-07
% h0 fix) and the current ballistic_success criterion, replacing the
% pre-criterion 2026-05-09 log (which is archived into logs/archive/ first, by
% copy -- nothing is destroyed).
%
% Decisions locked 2026-07-10 (see docs/plan_J3_runbook.md):
%   N_LHS = 40, rng(0); ranges.p widened [-2 2] -> [-12 12] so the box brackets
%   the operating spin p = -8.379; other ranges unchanged from
%   centroid_lookup_table.m. The operating point itself is appended as sample
%   N_LHS+1 by reusing the 2026-07-08 sat_on sweep output (zero extra
%   simulation; flagged is_reference=true).
%
% vs centroid_lookup_table.m this driver adds: per-sample checkpointing (safe
% to re-launch after any abort -- completed samples are never re-simulated),
% the full radial_profile stored per sample (required by j3_surrogate_refit.m
% area_proxy mode), and no inline figure -- render with replot_LUT_01.m after.
%
% Run from the project root, desktop (`j3_lut_regen`) or headless
% (`matlab -batch "j3_lut_regen"` -- close any other MATLAB first; the sweep's
% pgrep guard refuses to run alongside a second MATLAB_maca64 process).
% Wall: ~6-8 min/sample warm, ~15 min for the first (rapid-accel build),
% => roughly 4.5-5.5 h for N_LHS = 40. Re-launch the same command to resume.

clearvars; clc
assert(isfile("constants.m") && isfolder("Ballistics_Simulation-master"), ...
    "j3_lut_regen: run from the ATLIS Sims project root");
Simulink.sdi.clear()
set_param(0, 'CacheFolder', '');

load_system("discrete_smc_swarm_single")
addpath("Ballistics_Simulation-master/")

%% Locked configuration (2026-07-10) -- changing any of this invalidates the checkpoint
N_LHS  = 40;
ranges = struct( ...
    'Vo',   [ 80, 120], ...
    'el',   [ 35,  55], ...
    'az',   [  0,  30], ...
    'w_z0', [ -1,   2], ...
    'w_y0', [ -1,   1], ...
    'p',    [-12,  12]);          % widened from [-2 2]: brackets operating p = -8.379
field_names = fieldnames(ranges);
d           = numel(field_names);

ckpt_path  = "logs/centroid_lookup_ckpt_j3.mat";
final_path = "logs/centroid_lookup_log.mat";
ref_log    = "logs/trajectory_optimization_log_sat_on.mat";  % 2026-07-08 operating-point sweep

%% Archive the pre-criterion logs (copy, idempotent) before any overwrite
if ~isfolder("logs/archive"), mkdir("logs/archive"); end
archive_src = ["logs/centroid_lookup_log.mat"; ...
               "logs/surrogate_optimize_log.mat"];
archive_dst = ["logs/archive/centroid_lookup_log_pre_criterion_2026-05-09.mat"; ...
               "logs/archive/surrogate_optimize_log_pre_criterion_2026-05-12.mat"];
for r = 1:numel(archive_src)
    if isfile(archive_src(r)) && ~isfile(archive_dst(r))
        copyfile(archive_src(r), archive_dst(r));
        fprintf("archived %s -> %s\n", archive_src(r), archive_dst(r));
    end
end

%% Checkpoint init / resume
% Template built field-by-field so every repmat'ed row carries a concrete
% is_reference=false (downstream code concatenates [lookup.is_reference]).
template                    = struct();
template.params             = [];
template.p_centroid         = [];
template.reachability_pct   = NaN;
template.half_radius        = NaN;
template.radial_profile     = [];
template.ballistic_solution = [];
template.is_reference       = false;

if isfile(ckpt_path)
    ck = load(ckpt_path);
    assert(ck.N_LHS == N_LHS && isequal(ck.field_names, field_names), ...
        "j3_lut_regen: checkpoint config mismatch (N_LHS or fields) -- delete %s to restart", ckpt_path);
    for k = 1:d
        assert(isequal(ck.ranges.(field_names{k}), ranges.(field_names{k})), ...
            "j3_lut_regen: checkpoint range mismatch on %s -- delete %s to restart", ...
            field_names{k}, ckpt_path);
    end
    X      = ck.X;
    lookup = ck.lookup;
    done   = ck.done;
    wall_s = ck.wall_s;
    fprintf("resuming from checkpoint: %d/%d samples complete\n", nnz(done), numel(done));
else
    rng(0)
    X      = lhsdesign(N_LHS, d);
    lookup = repmat(template, N_LHS + 1, 1);   % +1: reference operating point
    done   = false(N_LHS + 1, 1);
    wall_s = nan(N_LHS + 1, 1);
end

%% Sample N_LHS+1: reference operating point, reused from the 2026-07-08 log
i_ref = N_LHS + 1;
if ~done(i_ref)
    S = load(ref_log, "out");
    assert(abs(S.out.reachability_pct - 0.55) < 1e-9, ...
        "j3_lut_regen: %s does not hold the expected 2026-07-08 result (reach 0.550) -- wrong log?", ref_log);
    lookup(i_ref).params = struct( ...
        'Vo', 100, 'el', 45, 'az', 15, 'w_z0', 1, 'w_y0', 0.5, 'p', -8.379, ...
        'alpha_0', 2, 'beta_0', -0.5, 'x_0', 0, 'y_0', 0, 'z_0', 0, 't_max', 300);
    lookup(i_ref).p_centroid         = S.out.p_centroid;
    lookup(i_ref).reachability_pct   = S.out.reachability_pct;
    lookup(i_ref).half_radius        = S.out.radial_profile.half_radius;
    lookup(i_ref).radial_profile     = S.out.radial_profile;
    lookup(i_ref).ballistic_solution = S.out.ballistic_solution;
    lookup(i_ref).is_reference       = true;
    done(i_ref) = true;
    save_ckpt(ckpt_path, X, ranges, N_LHS, field_names, lookup, done, wall_s);
    fprintf("reference sample stored from %s (reach=%.3f, cx=%.1f, cy=%.1f)\n", ...
        ref_log, lookup(i_ref).reachability_pct, ...
        lookup(i_ref).p_centroid(1), lookup(i_ref).p_centroid(2));
    clear S
end

%% LHS samples (each ~6-8 min warm; checkpoint written after every sample)
for n = 1:N_LHS
    if done(n), continue; end
    params = struct('alpha_0', 2, 'beta_0', -0.5, ...
                    'x_0', 0, 'y_0', 0, 'z_0', 0, 't_max', 300);
    for k = 1:d
        f = field_names{k};
        params.(f) = ranges.(f)(1) + X(n, k) * diff(ranges.(f));
    end
    t0  = tic;
    out = sweep_landing_centroid(params, false);
    wall_s(n) = toc(t0);
    lookup(n).params             = params;
    lookup(n).p_centroid         = out.p_centroid;
    lookup(n).reachability_pct   = out.reachability_pct;
    lookup(n).half_radius        = out.radial_profile.half_radius;
    lookup(n).radial_profile     = out.radial_profile;
    lookup(n).ballistic_solution = out.ballistic_solution;
    lookup(n).is_reference       = false;
    done(n) = true;
    save_ckpt(ckpt_path, X, ranges, N_LHS, field_names, lookup, done, wall_s);
    fprintf("LHS %d/%d: cx=%.1f cy=%.1f reach=%.2f half_r=%.1f errored=%d  (%.1f min; %d/%d done)\n", ...
        n, N_LHS, out.p_centroid(1), out.p_centroid(2), out.reachability_pct, ...
        out.radial_profile.half_radius, out.n_errored, wall_s(n)/60, nnz(done), numel(done));
end

%% Finalize canonical log
assert(all(done), "j3_lut_regen: incomplete (%d/%d) -- re-launch to resume", nnz(done), numel(done));
N = numel(lookup);   % consumer-facing count: surrogate/replot iterate over all lookup entries
save(final_path, "lookup", "ranges", "X", "N", "field_names", "N_LHS", "wall_s")
fprintf("wrote %s: N=%d entries (%d LHS + 1 reference), total sim wall %.1f h\n", ...
    final_path, N, N_LHS, sum(wall_s, 'omitnan')/3600);
fprintf("checkpoint retained at %s -- delete only after verifying the canonical log\n", ckpt_path);

%% ---- local functions ----
function save_ckpt(path, X, ranges, N_LHS, field_names, lookup, done, wall_s)
    % Atomic-ish checkpoint: write to a temp name, then move over the target.
    tmp = replace(path, ".mat", "_tmp.mat");
    save(tmp, "X", "ranges", "N_LHS", "field_names", "lookup", "done", "wall_s");
    movefile(tmp, path, "f");
end
