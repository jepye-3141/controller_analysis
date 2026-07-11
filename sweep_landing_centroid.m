function out = sweep_landing_centroid(traj_params, visualize)
% sweep_landing_centroid  dSMC deploy x crosstrack x neighbor sweep over a
% mortar trajectory; returns the power-weighted landing centroid plus a
% radial reachability profile.
%
% traj_params : struct, all fields required:
%   .Vo .el .az .w_z0 .w_y0 .p .alpha_0 .beta_0 .x_0 .y_0 .z_0 .t_max
%   Optional: .saturation_on -> constants_struct.saturation_on; .label
%   suffixes figure names and the saved log ("_sat_on" etc.).
% visualize   : bool, default false. When true, render TO_01..TO_07 figures
%               and save logs/trajectory_optimization_log<label>.mat
%               (whole-workspace save; reload 'out' via load(log, "out")).
%
% out : struct
%   .p_centroid          [cx; cy; 0]; NaN when zero trials succeed
%   .reachability_pct    stable fraction of attempted trials (cells with
%                        a valid target_deploy_idx; boundary excluded)
%   .radial_profile      r_bins, mean_ratio, peak_ratio, half_radius
%   .ballistic_solution  full mortar struct
%   .n_errored           parsim trials whose ErrorMessage was non-empty
%
% Per-trial success ('stable') is scored post-hoc by ballistic_success:
% full duration (blowup guard never tripped) + final XY within reach_tol
% + velocity ratio <= 2 (whole trace) + rotation-rate ratio <= 2 for
% t > 1 s (grace window). The rotation trace is the model's rotvelout
% (plant Euler-angle rates -- the same signal the termination chart
% norms). Per-trial diagnostics land in results(i,k,nb).criteria.

if nargin < 2
    visualize = false;
end

%% Guard: refuse to run if another MATLAB session is open (macOS).
% A second MATLAB process holding Parallel Computing Toolbox seats makes
% parpool fail ("worker shut down unexpectedly with status 1"). >1
% MATLAB_maca64 process means a peer (usually a GUI session) is contending.
if ismac
    % Count MATLAB_maca64 processes (self included, so the baseline is 1).
    % BSD pgrep has no -c flag, so the old `pgrep -c` always failed open to 0
    % -- this guard never fired (bugsweep 2026-07-10 finding 6). `-x` matches
    % the exact process name and pipes to wc for a portable count.
    [~, n_str] = system("pgrep -x MATLAB_maca64 | wc -l");
    n_matlab = str2double(strtrim(n_str));
    if n_matlab > 1
        error('sweep_landing_centroid:matlab_contention', ...
            ['Detected %d concurrent MATLAB sessions. Close other ' ...
             'MATLAB instances (GUI included) before launching this ' ...
             'sweep -- parpool license seats will otherwise collide ' ...
             'and worker startup will fail.'], n_matlab);
    end
end

%% Constants
constants;
if isfield(traj_params, 'saturation_on')
    constants_struct.saturation_on = traj_params.saturation_on;
elseif isfield(traj_params, 'label')
    warning('sweep_landing_centroid:saturation_on_default', ...
        'traj_params has label "%s" but no saturation_on field; using default %d. Possible typo?', ...
        traj_params.label, constants_struct.saturation_on);
end
label_suffix = "";
if isfield(traj_params, 'label')
    label_suffix = "_" + traj_params.label;
end
STABILIZE = 4;

%% Sweep configuration
n_deploy   = 8;             % deployment points sampled along 3D mission trajectory
% Cross-track landing offsets (m); positive = left of travel (CCW).
ct_offsets = [0, 100, -100, 200, -200];
ct_names   = ["center", "left100", "right100", "left200", "right200"];
n_ct       = length(ct_offsets);
neighbor_offsets = [-1, 0, 1, 2];         % target deploy-index offsets
nb_names         = ["behind1", "inplace", "ahead1", "ahead2"];
n_nb             = length(neighbor_offsets);
sim_time   = 60;            % per-trial sim stop time (s)
reach_tol  = 10;            % tolerance (m): final XY for success, 3D for time_to_land
vel_tol    = 5;             % velocity magnitude threshold for "landed" (m/s)

%% Mission trajectory
env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');

ballistic_solution = eom2(traj_params, env, false);

%% Arc-length sampling of deployment points
traj_xyz = ballistic_solution.trajectory(:, 10:12);
seg_3d   = vecnorm(diff(traj_xyz), 2, 2);
arc_3d   = [0; cumsum(seg_3d)];
deploy_arc = linspace(0.20 * arc_3d(end), 0.80 * arc_3d(end), n_deploy);

deploy_pct_vec = linspace(20, 80, n_deploy);

% Trajectory columns: 1:3 vel (earth), 4:6 ang rates (earth), 7:9 pointing
% unit vector, 10:12 pos. Deploy attitude derives from the pointing vector
% (interpolate r, then call ballistic_deploy_state). Cols 13:15 now carry the
% same derived attitude, backfilled post-hoc by eom2 -- valid at exact rows,
% but interpolated stations must still interpolate r, never the angle columns.
deploy_vel_earth  = interp1(arc_3d, ballistic_solution.trajectory(:, 1:3),   deploy_arc).';
deploy_rvel_earth = interp1(arc_3d, ballistic_solution.trajectory(:, 4:6),   deploy_arc).';
deploy_r          = interp1(arc_3d, ballistic_solution.trajectory(:, 7:9),   deploy_arc).';
deploy_pos        = interp1(arc_3d, ballistic_solution.trajectory(:, 10:12), deploy_arc).';

% Per-deploy cross-track unit vector (perpendicular to local horizontal velocity).
% Falls back to global launch->impact direction when horizontal speed is ~0.
ct_dir = zeros(2, n_deploy);
gt_end = ballistic_solution.trajectory(end, 10:11).';
gt_dir = gt_end / max(norm(gt_end), eps);
fallback_ct = [-gt_dir(2); gt_dir(1)];
for i = 1:n_deploy
    horiz = deploy_vel_earth(1:2, i);
    if norm(horiz) > eps
        t_hat        = horiz / norm(horiz);
        ct_dir(:, i) = [-t_hat(2); t_hat(1)];   % CCW (left of direction of travel)
    else
        ct_dir(:, i) = fallback_ct;
    end
end

% Landing positions: 3 x n_deploy x n_ct
land_pos = zeros(3, n_deploy, n_ct);
for k = 1:n_ct
    land_pos(1, :, k) = deploy_pos(1, :) + ct_offsets(k) * ct_dir(1, :);
    land_pos(2, :, k) = deploy_pos(2, :) + ct_offsets(k) * ct_dir(2, :);
    land_pos(3, :, k) = 0;
end

% Placeholder workspace vars for Step block parameters (evaluated at compile
% time even in STABILIZE mode, where they're unused logically)
x0_step = [2; 4; 10];
xf      = [0; 0; 0];

%% Sweep
results = repmat(struct( ...
    'deploy_pct',        NaN, ...
    'ct_offset',         NaN, ...
    'ct_name',           "", ...
    'nb_offset',         NaN, ...
    'nb_name',           "", ...
    'target_deploy_idx', NaN, ...
    'time_to_land',      NaN, ...
    'stable',            false, ...
    'criteria',          [], ...
    'trajectory',        []), n_deploy, n_ct, n_nb);

% Pre-pass: one SimulationInput + metadata entry per valid (i,k,nb) trial,
% flat-indexed so the parallel run is pure (flat inputs -> flat outputs).
% Boundary trials (target_idx out of range) go straight into results and
% skip the queue.
n_trials_max = n_deploy * n_ct * n_nb;
sim_inputs = repmat(Simulink.SimulationInput("discrete_smc_swarm_single"), 1, n_trials_max);
trial_meta = repmat(struct('i', 0, 'k', 0, 'nb', 0, 'xf', zeros(3,1)), 1, n_trials_max);
trial_ptr  = 0;

% Template SimulationInput carries the 7 statics the model reads from base
% workspace (Simulink.findVars: A, B, constants_struct, constants_struct_bus,
% dt, x0_step, xf); per-trial copies setVariable only xi, xf_ballistic,
% simcase. TransferBaseWorkspaceVariables is off, so any new model dependency
% MUST be added to this static block or trials silently fail/error.
simIn_template = Simulink.SimulationInput("discrete_smc_swarm_single");
simIn_template = simIn_template.setModelParameter( ...
    'StopTime', num2str(sim_time), 'SimulationMode', 'rapid-accelerator');
simIn_template = simIn_template.setVariable('A', A);
simIn_template = simIn_template.setVariable('B', B);
simIn_template = simIn_template.setVariable('constants_struct', constants_struct);
simIn_template = simIn_template.setVariable('constants_struct_bus', constants_struct_bus);
simIn_template = simIn_template.setVariable('dt', dt);
simIn_template = simIn_template.setVariable('x0_step', x0_step);
simIn_template = simIn_template.setVariable('xf', xf);

% (No fail-fast findVars audit here: it evaluates block parameters and
% errors on per-trial vars not yet in base workspace. The n_errored assert
% below catches a missing setVariable name, ~30 s into parsim startup.)

for i = 1:n_deploy
    % Deploy seed: attitude from the pointing vector, velocities/rates
    % mapped world->body consistently with system_dynamics.m (2026-07-10
    % fix; see ballistic_deploy_state.m for the convention).
    xi = ballistic_deploy_state(deploy_vel_earth(:, i), ...
        deploy_rvel_earth(:, i), deploy_r(:, i), deploy_pos(:, i));

    for k = 1:n_ct
        for nb = 1:n_nb
            target_idx = i + neighbor_offsets(nb);

            results(i, k, nb).deploy_pct = deploy_pct_vec(i);
            results(i, k, nb).ct_offset  = ct_offsets(k);
            results(i, k, nb).ct_name    = ct_names(k);
            results(i, k, nb).nb_offset  = neighbor_offsets(nb);
            results(i, k, nb).nb_name    = nb_names(nb);

            if target_idx < 1 || target_idx > n_deploy
                results(i, k, nb).target_deploy_idx = NaN;
                continue
            end
            results(i, k, nb).target_deploy_idx = target_idx;

            xf_ballistic = land_pos(:, target_idx, k);
            simcase = STABILIZE;

            simIn = simIn_template;
            simIn = simIn.setVariable('xi', xi);
            simIn = simIn.setVariable('xf_ballistic', xf_ballistic);
            simIn = simIn.setVariable('simcase', simcase);

            trial_ptr = trial_ptr + 1;
            sim_inputs(trial_ptr) = simIn;
            trial_meta(trial_ptr) = struct('i', i, 'k', k, 'nb', nb, 'xf', xf_ballistic);
        end
    end
end
sim_inputs = sim_inputs(1:trial_ptr);
trial_meta = trial_meta(1:trial_ptr);

n_workers = 12;
if isempty(gcp('nocreate'))
    parpool('Processes', n_workers);
end
pool = gcp();
fprintf('Running %d trials on %d workers...\n', length(sim_inputs), pool.NumWorkers);
sim_outputs = parsim(sim_inputs, 'ShowProgress', 'on', 'StopOnError', 'off', ...
                     'TransferBaseWorkspaceVariables', 'off');

% Post-pass: score each SimulationOutput into results(i,k,nb)
n_errored = 0;
for ptr = 1:length(sim_outputs)
    meta = trial_meta(ptr);
    out_sim = sim_outputs(ptr);

    if ~isempty(out_sim.ErrorMessage)
        warning('Trial (i=%d, k=%d, nb=%d) errored: %s', ...
                meta.i, meta.k, meta.nb, out_sim.ErrorMessage);
        n_errored = n_errored + 1;
        continue   % leave defaults (stable=false, time_to_land=NaN, trajectory=[])
    end

    t_trial    = out_sim.posout.Time;
    pos_trial  = squeeze(out_sim.posout.Data).';
    vel_trial  = squeeze(out_sim.velout.Data).';
    % rotvelout = plant Euler-angle rates [thetadot phidot psidot], the
    % signal the termination chart norms; logged shape varies ([3x1xT]
    % vs [Tx3]), hence the squeeze+transpose normalization.
    rot_trial  = squeeze(out_sim.rotvelout.Data);
    % Orient to Tx3 by raw ndims (3-D [3x1xT] -> transpose; 2-D [Tx3] -> keep),
    % robust to the T==3 case a size==3 heuristic mis-orients (finding 12).
    if ndims(out_sim.rotvelout.Data) == 3, rot_trial = rot_trial.'; end
    ctrl_raw   = squeeze(out_sim.ctrlout.Data).';
    % Zero-pad ctrl to t_trial length (sim delay: ctrlout may lag posout).
    n_pad      = length(t_trial) - size(ctrl_raw, 1);
    if n_pad > 0
        ctrl_trial = [zeros(n_pad, size(ctrl_raw, 2)); ctrl_raw];
    else
        ctrl_trial = ctrl_raw;
    end

    % Success criterion (paper eq:ballistic-success): full duration +
    % final-XY tol + velocity ratio (whole trace) + rotation ratio (t > 1 s).
    [stable, crit] = ballistic_success(t_trial, pos_trial, vel_trial, ...
        meta.xf(1:2), norm(deploy_vel_earth(:, meta.i)), sim_time, ...
        ReachTol=reach_tol, RotVel=rot_trial);

    dist_to_target = vecnorm(pos_trial - meta.xf.', 2, 2);
    speed          = vecnorm(vel_trial, 2, 2);
    reach_mask     = (dist_to_target < reach_tol) & (speed < vel_tol);
    reach_idx      = find(reach_mask, 1, 'first');
    if isempty(reach_idx)
        time_to_land = NaN;
    else
        time_to_land = t_trial(reach_idx);
    end

    results(meta.i, meta.k, meta.nb).time_to_land = time_to_land;
    results(meta.i, meta.k, meta.nb).stable       = stable;
    results(meta.i, meta.k, meta.nb).criteria     = crit;
    results(meta.i, meta.k, meta.nb).trajectory   = struct( ...
        'time',   t_trial, ...
        'pos',    pos_trial, ...
        'vel',    vel_trial, ...
        'rotvel', rot_trial, ...
        'ctrl',   ctrl_trial);
end

err_frac = n_errored / max(length(sim_outputs), 1);
fprintf('parsim error tally: %d of %d trials errored (%.1f%%)\n', ...
        n_errored, length(sim_outputs), 100*err_frac);
assert(err_frac < 0.05, ...
    'sweep_landing_centroid: %d of %d trials errored (>5%%); sweep results untrustworthy', ...
    n_errored, length(sim_outputs));

%% Aggregations needed for centroid + figures
stable_arr = reshape([results.stable], n_deploy, n_ct, n_nb);
tgt_arr    = reshape([results.target_deploy_idx], n_deploy, n_ct, n_nb);
n_trials   = numel(results);
[I_idx, K_idx, NB_idx] = ndgrid(1:n_deploy, 1:n_ct, 1:n_nb);
I_idx = I_idx(:); K_idx = K_idx(:); NB_idx = NB_idx(:);

% Landing-point reachability flatten (used by both centroid and TO_04).
tgt_flat    = tgt_arr(:);
stable_flat = stable_arr(:);
valid_flat  = ~isnan(tgt_flat);
subs        = [tgt_flat(valid_flat), K_idx(valid_flat)];
landing_total   = accumarray(subs, 1, [n_deploy n_ct]);
landing_success = accumarray(subs, double(stable_flat(valid_flat)), [n_deploy n_ct]);
landing_ratio   = landing_success ./ max(landing_total, 1);

land_x_flat = reshape(land_pos(1,:,:), [], 1);
land_y_flat = reshape(land_pos(2,:,:), [], 1);
land_z_flat = reshape(land_pos(3,:,:), [], 1);
ratio_flat  = landing_ratio(:);

%% Power-weighted centroid
% 'natural' interpolant; the centroid is method-sensitive ('linear' shifts it ~1 m).
F_heat = scatteredInterpolant(land_x_flat, land_y_flat, ratio_flat, 'natural', 'none');
n_grid = 200;
[Xg, Yg] = meshgrid(linspace(min(land_x_flat), max(land_x_flat), n_grid), ...
                    linspace(min(land_y_flat), max(land_y_flat), n_grid));
Rg = F_heat(Xg, Yg);
Zg = zeros(size(Xg));

p_centroid = 2;
W = max(Rg, 0).^p_centroid;
W(isnan(Rg)) = 0;
tot_w = sum(W(:));
if tot_w > 0
    cx = sum(Xg(:) .* W(:)) / tot_w;
    cy = sum(Yg(:) .* W(:)) / tot_w;
    assert(cx >= min(land_x_flat) - 1e-6 && cx <= max(land_x_flat) + 1e-6, ...
        'sweep_landing_centroid: centroid x = %.2f outside grid [%.2f, %.2f]', ...
        cx, min(land_x_flat), max(land_x_flat));
    assert(cy >= min(land_y_flat) - 1e-6 && cy <= max(land_y_flat) + 1e-6, ...
        'sweep_landing_centroid: centroid y = %.2f outside grid [%.2f, %.2f]', ...
        cy, min(land_y_flat), max(land_y_flat));
else
    % Zero-reachability sweep (legitimate: the per-rotor-clipped baseline
    % lands 0/140) -- centroid undefined. Keep NaN, still save the log and
    % render figures (plot skips NaN points).
    warning('sweep_landing_centroid:zero_reachability', ...
        'no trial satisfied the success criterion; centroid and half_radius are NaN');
    cx = NaN; cy = NaN;
end
cz = 0;
fprintf('centroid (p=%d): x = %.2f m, y = %.2f m\n', p_centroid, cx, cy);

%% Radial reachability profile around centroid
n_bins  = 20;
if tot_w > 0
    r_grid = sqrt((Xg(:) - cx).^2 + (Yg(:) - cy).^2);
else
    % All-zero profile: bin about the grid center so r_bins stay finite.
    r_grid = sqrt((Xg(:) - mean(Xg(:))).^2 + (Yg(:) - mean(Yg(:))).^2);
end
r_edges = linspace(0, max(r_grid), n_bins + 1);
r_bins_c = (r_edges(1:end-1) + r_edges(2:end)) / 2;
mean_ratio = nan(1, n_bins);
Rg_flat = Rg(:);
for b = 1:n_bins
    in_bin = r_grid >= r_edges(b) & r_grid < r_edges(b+1);
    vals   = Rg_flat(in_bin);
    vals   = vals(~isnan(vals));
    if ~isempty(vals)
        mean_ratio(b) = mean(vals);
    end
end
[peak_ratio, peak_b] = max(mean_ratio, [], 'omitnan');
if isnan(peak_ratio) || peak_ratio <= 0
    half_radius = NaN;   % no reachable region -> half-reach radius undefined
else
    half_offset = find(mean_ratio(peak_b:end) <= 0.5*peak_ratio, 1, 'first');
    if isempty(half_offset)
        half_radius = NaN;
    else
        half_radius = r_bins_c(peak_b - 1 + half_offset);
    end
end

radial_profile = struct( ...
    'r_bins',      r_bins_c, ...
    'mean_ratio',  mean_ratio, ...
    'peak_ratio',  peak_ratio, ...
    'half_radius', half_radius);

% Normalize over attempted trials only -- boundary cells with target_idx
% out of [1, n_deploy] never ran and would otherwise dilute the metric.
attempted_mask  = ~isnan([results.target_deploy_idx]);
reachability_pct = sum(stable_arr(:)) / nnz(attempted_mask);

out = struct( ...
    'p_centroid',         [cx; cy; cz], ...
    'reachability_pct',   reachability_pct, ...
    'radial_profile',     radial_profile, ...
    'ballistic_solution', ballistic_solution, ...
    'n_errored',          n_errored);

if ~visualize
    return
end

%% Figures
close all;
set_default_fonts();

%% Kinetic energy overlay
% Nebula-style colormap: blue -> purple -> magenta, indexed by deploy number.
nebula_line = [linspace(0, 1, n_deploy).', zeros(n_deploy, 1), ones(n_deploy, 1)];
nebula_full = [linspace(0, 1, 256).',      zeros(256, 1),      ones(256, 1)];

ts = 0:0.1:sim_time;
figure
hold on
grid on
for r = 1:n_trials
    if ~stable_arr(r)
        continue
    end
    tr = results(r).trajectory;
    if isempty(tr) || isempty(tr.time)
        continue
    end
    v_interp = interp1(tr.time, tr.vel, ts);
    ke       = 0.5*constants_struct.m*sum(v_interp.^2, 2);
    h = plot(ts, ke);
    h.Color = [nebula_line(I_idx(r), :) 0.5];
end
colormap(gca, nebula_full)
clim([deploy_pct_vec(1) deploy_pct_vec(end)])
cb = colorbar;
cb.Label.String = "Deployment % along mission trajectory";
title("dSMC kinetic energy: successful landings only")
xlabel("Time (s)")
ylabel("Kinetic energy (J)")
xlim([0 10])
ylim([0 5000])
export_figure("figs/TO_01_kinetic_energy" + label_suffix)

%% Control effort norm overlay
figure
hold on
grid on
for r = 1:n_trials
    if ~stable_arr(r)
        continue
    end
    tr = results(r).trajectory;
    if isempty(tr) || isempty(tr.time)
        continue
    end
    u_interp = interp1(tr.time, tr.ctrl, ts);
    u_norm   = vecnorm(u_interp, 2, 2);
    h = plot(ts, u_norm);
    h.Color = [nebula_line(I_idx(r), :) 0.5];
end
colormap(gca, nebula_full)
clim([deploy_pct_vec(1) deploy_pct_vec(end)])
cb = colorbar;
cb.Label.String = "Deployment % along mission trajectory";
title("dSMC control effort norm: successful landings only")
xlabel("Time (s)")
ylabel("\|ctrlout\|")
xlim([0 10])
export_figure("figs/TO_06_control_norm" + label_suffix)

%% Settling time: first XY pass within reach_tol of target
settle_t_arr = nan(n_deploy, n_ct, n_nb);
for r = 1:n_trials
    tr  = results(r).trajectory;
    tgt = tgt_arr(r);
    if isempty(tr) || isempty(tr.time) || isnan(tgt)
        continue
    end
    xf_ik = squeeze(land_pos(:, tgt, K_idx(r)));
    dist  = vecnorm(tr.pos(:, 1:2) - xf_ik(1:2).', 2, 2);   % horizontal, matches stable flag
    idx   = find(dist <= reach_tol, 1, 'first');
    if ~isempty(idx)
        settle_t_arr(r) = tr.time(idx);
    end
end

figure
hold on
grid on
markers   = {'o', 's', 'd', '^', 'v'};
ct_labels = ["center", "left +100 m", "right -100 m", "left +200 m", "right -200 m"];
h_handles = gobjects(1, n_ct);
for k = 1:n_ct
    valid_k = ~isnan(tgt_arr(:, k, :));
    xs = repmat(deploy_pct_vec(:), 1, n_nb);
    ys = squeeze(settle_t_arr(:, k, :));
    sm = squeeze(stable_arr(:, k, :));
    valid_k = squeeze(valid_k);
    xs = xs(valid_k);
    ys = ys(valid_k);
    sm = logical(sm(valid_k));
    scatter(xs(sm),  ys(sm),  120, 'g', markers{k}, 'filled', 'MarkerFaceAlpha', 0.7)
    scatter(xs(~sm), ys(~sm), 120, 'r', markers{k}, 'filled', 'MarkerFaceAlpha', 0.7)
    h_handles(k) = scatter(NaN, NaN, 120, 'k', markers{k}, 'filled');
end
xlabel("Deployment % along mission trajectory")
ylabel(sprintf("Time to reach within %g m horizontal of target (s)", reach_tol))
title("dSMC settling time (first XY pass within reach\_tol)")
legend(h_handles, ct_labels, 'Location', 'best')
xlim([deploy_pct_vec(1) deploy_pct_vec(end)])
ylim([0 sim_time])
export_figure("figs/TO_02_settling_time" + label_suffix)

%% Deployment-point success rate (averaged over all valid ct x neighbor trials)
% Trials with isnan(target_deploy_idx) never ran (parsim was skipped), so
% stable defaults to false and contributes 0 to the success sum.
deploy_total   = squeeze(sum(~isnan(tgt_arr), [2 3]));
deploy_success = squeeze(sum(stable_arr, [2 3]));
deploy_ratio   = deploy_success ./ max(deploy_total, 1);

n_half   = 32;
cmap_ryg = ryg_cmap(n_half);

figure
axis equal
grid on
hold on
plot3(ballistic_solution.trajectory(:,10), ...
      ballistic_solution.trajectory(:,11), ...
      ballistic_solution.trajectory(:,12), '-b', 'LineWidth', 2)
scatter3(deploy_pos(1,:), deploy_pos(2,:), deploy_pos(3,:), ...
    160, deploy_ratio, 'filled', 'MarkerEdgeColor', 'k')
view(3)
colormap(gca, cmap_ryg)
clim([0 1])
cb = colorbar;
cb.Label.String = "Deployment success ratio (all ct \times neighbor targets)";
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC deployment-point success rate")
export_figure("figs/TO_03_stabilization_envelope" + label_suffix)

%% Landing-point reachability
% Each landing point (target deploy idx j, ct k) is attempted by up to 4
% deploys (j-2..j+1, per neighbor_offsets). Landing markers colored by
% success ratio; dotted deploy->target lines by that trial's outcome.

% Earliest deploy to successfully reach each (tgt, k) landing point; its trace is bolded.
first_reach_deploy = nan(n_deploy, n_ct);
for tgt = 1:n_deploy
    for k = 1:n_ct
        candidates = nan(1, n_nb);
        for nb = 1:n_nb
            i_src = tgt - neighbor_offsets(nb);
            if i_src < 1 || i_src > n_deploy
                continue
            end
            if stable_arr(i_src, k, nb)
                candidates(nb) = i_src;
            end
        end
        first_reach_deploy(tgt, k) = min(candidates, [], 'omitnan');
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
for r = 1:n_trials
    tgt = tgt_arr(r);
    if isnan(tgt)
        continue
    end
    i = I_idx(r);
    k = K_idx(r);
    is_first = stable_arr(r) && (i == first_reach_deploy(tgt, k));
    if is_first
        line_color = [0 0 0];
        lw = 2.5;
        ls = '-';
    elseif stable_arr(r)
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

% Magenta step marks ratio == 0 ("no deploy reached this point"); the ryg
% gradient covers >0. The band extends the colormap below 0 in clim space;
% exact-zero data is remapped into it so marker color matches the colorbar.
n_ryg     = size(cmap_ryg, 1);
n_band    = 4;                            % visible magenta band height
zero_band = repmat([1 0 1], n_band, 1);
ryg_with_zero = [zero_band; cmap_ryg];
band_frac = n_band / n_ryg;

cdata_flat = ratio_flat;
cdata_flat(ratio_flat == 0) = -band_frac/2;   % center of magenta band

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
if ~isempty(h_first)
    legend([h_traj h_first], ["Mission trajectory", "First reaching deploy"], ...
           'Location', 'best')
else
    legend(h_traj, "Mission trajectory", 'Location', 'best')
end
export_figure("figs/TO_04_landing_envelope" + label_suffix)

%% Landing-reachability heatmap and centroid (TO_07)
% Same landing-point (x, y, ratio) samples as TO_04, interpolated into a
% surface at z=0 so the trajectory plots above it. The p=2 weighting pulls
% the centroid toward the dense core; at p=1, isolated 1.0 cells skew it.
figure
axis equal
grid on
hold on
h_surf = surf(Xg, Yg, Zg, Rg, 'EdgeColor', 'none', 'FaceColor', 'interp', ...
              'HandleVisibility', 'off'); %#ok<NASGU>
h_traj = plot3(ballistic_solution.trajectory(:,10), ...
               ballistic_solution.trajectory(:,11), ...
               ballistic_solution.trajectory(:,12), '-b', 'LineWidth', 2);
h_dep = scatter3(deploy_pos(1,:), deploy_pos(2,:), deploy_pos(3,:), ...
                 40, 'k', 'filled');
h_cent = plot3(cx, cy, cz, 'p', 'MarkerSize', 28, 'MarkerFaceColor', 'm', ...
               'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
colormap(gca, cmap_ryg)
clim([0 1])
cb = colorbar;
cb.Label.String = "Landing-point success ratio";
view(3)
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC landing-reachability heatmap and centroid")
legend([h_traj h_dep h_cent], ...
       ["Mission trajectory", "Deploy points", ...
        sprintf("Centroid (p=%d)", p_centroid)], 'Location', 'best')
% surf at z=0 with FaceColor=interp tessellates poorly as vector (scan lines,
% ~5 MB). Embed a 300 DPI raster of the figure in EPS instead.
export_figure("figs/TO_07_landing_heatmap" + label_suffix, EPSContentType="image")

%% Unique reachability (Shapley-weighted credit, per-success normalized)
% reachers{j,k} = list of deploy indices that successfully stabilized at landing point (j,k).
reachers = cell(n_deploy, n_ct);
for r = 1:n_trials
    tgt = tgt_arr(r);
    if isnan(tgt) || ~stable_arr(r)
        continue
    end
    reachers{tgt, K_idx(r)}(end+1) = I_idx(r);
end

unique_credit = zeros(n_deploy, 1);
n_successes   = zeros(n_deploy, 1);
for r = 1:n_trials
    if ~stable_arr(r)
        continue
    end
    tgt = tgt_arr(r);
    if isnan(tgt)
        continue
    end
    i = I_idx(r);
    n_successes(i)   = n_successes(i) + 1;
    unique_credit(i) = unique_credit(i) + 1 / numel(reachers{tgt, K_idx(r)});
end
unique_score = unique_credit ./ max(n_successes, 1);

figure
axis equal
grid on
hold on
plot3(ballistic_solution.trajectory(:,10), ...
      ballistic_solution.trajectory(:,11), ...
      ballistic_solution.trajectory(:,12), '-b', 'LineWidth', 2)
scatter3(deploy_pos(1,:), deploy_pos(2,:), deploy_pos(3,:), ...
    160, unique_score, 'filled', 'MarkerEdgeColor', 'k')
view(3)
colormap(gca, cmap_ryg)
clim([0 1])
cb = colorbar;
cb.Label.String = "Unique reachability (Shapley credit per successful hit)";
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC deployment-point unique reachability")
export_figure("figs/TO_05_unique_reachability" + label_suffix)

save("logs/trajectory_optimization_log" + label_suffix + ".mat")

end
