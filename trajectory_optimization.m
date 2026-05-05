close all; clear all;
clc
Simulink.sdi.clear()
set_param(0, 'CacheFolder', '');

%% Constants
constants;
STEP = 1;
F8 = 2;
SPIRAL = 3;
STABILIZE = 4;

load_system("discrete_smc_swarm_single")
addpath("Ballistics_Simulation-master/")

%% Sweep configuration
n_deploy   = 12;            % deployment points sampled along 3D mission trajectory
% Cross-track landing offsets (m): center + inner(+-50) + middle(+-100) + outer(+-150).
% Positive = left of travel (CCW), negative = right.
ct_offsets = [0, 50, -50, 100, -100, 150, -150];
ct_names   = ["center", "left50", "right50", "left100", "right100", "left150", "right150"];
n_ct       = length(ct_offsets);
neighbor_offsets = [-2, -1, 0, 1, 2];     % target deploy-index offsets
nb_names         = ["behind2", "behind1", "inplace", "ahead1", "ahead2"];
n_nb             = length(neighbor_offsets);
sim_time   = 60;            % per-trial sim stop time (s)
reach_tol  = 10;            % absolute 3D position tolerance for "reached"/settled (m)
vel_tol    = 5;             % velocity magnitude threshold for "landed" (m/s)

%% Mission trajectory (same firing parameters as analysis.m ballistic envelope)
env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');

Vo_set = 100; % initial vel at muzzle exit in m/s
el_0_set = 45; % vertical angle of departure in deg (pos up)
az_0_set = 15;  % horizontal angle of departure in deg(pos to right)

w_z0_set=1; % initial pitch rate in rad/s (pos nose up)
w_y0_set=0.5; % initial transverse yaw rate in rad/s (pos for left yaw)

alpha_0_set = 2; % exit elevation (deg)
beta_0_set= -0.5; % exit azimuth (deg)

x_0 = 0; % x-axis (m) - range direction
y_0 = 0; % y-axis (m) - altitude
z_0 = 0; % z-axis (m) - cross-range direction

t_max = 300; % sim end time
p = 0; % initial spin rate in rad/s

ballistic_solution = eom2(t_max, Vo_set, el_0_set, az_0_set, w_z0_set, w_y0_set, ...
    alpha_0_set, beta_0_set, p, x_0, y_0, z_0, env, false);

%% Arc-length sampling of deployment points
traj_xyz = ballistic_solution.trajectory(:, 10:12);
seg_3d   = vecnorm(diff(traj_xyz), 2, 2);
arc_3d   = [0; cumsum(seg_3d)];
deploy_arc = linspace(0, 0.9 * arc_3d(end), n_deploy);

deploy_pct_vec = linspace(0, 90, n_deploy);

% Interpolate full trajectory state at each deployment arc length
% Columns 1:3 vel (earth), 4:6 ang rates (earth), 13:15 Euler deg, 10:12 pos
deploy_vel_earth  = interp1(arc_3d, ballistic_solution.trajectory(:, 1:3),   deploy_arc).';
deploy_rvel_earth = interp1(arc_3d, ballistic_solution.trajectory(:, 4:6),   deploy_arc).';
deploy_rot_deg    = interp1(arc_3d, ballistic_solution.trajectory(:, 13:15), deploy_arc).';
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

% Landing positions per deploy point, per cross-track variant: 3 x n_deploy x n_ct
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
    'trajectory',        []), n_deploy, n_ct, n_nb);

for i = 1:n_deploy
    rot0 = deg2rad(deploy_rot_deg(:, i));
    x0   = deploy_pos(:, i);
    rot  = eul2rotm([rot0(3) rot0(2) rot0(1)], "ZYX");

    vel0    = rot*deploy_vel_earth(:, i);  % earth -> body
    rotvel0 = rot*deploy_rvel_earth(:, i); % earth -> body

    xi = [vel0; rotvel0; rot0; x0];

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
            set_param("discrete_smc_swarm_single","StopTime",num2str(sim_time), 'SimulationMode','Rapid')
            dsmc_trial = sim("discrete_smc_swarm_single");

            t_trial    = dsmc_trial.posout.Time;
            pos_trial  = squeeze(dsmc_trial.posout.Data).';
            vel_trial  = squeeze(dsmc_trial.velout.Data).';
            ctrl_raw   = squeeze(dsmc_trial.ctrlout.Data).';
            % Zero-pad ctrl at the start to match t_trial length (sim delay
            % means ctrlout may have fewer samples than posout).
            n_pad      = length(t_trial) - size(ctrl_raw, 1);
            if n_pad > 0
                ctrl_trial = [zeros(n_pad, size(ctrl_raw, 2)); ctrl_raw];
            else
                ctrl_trial = ctrl_raw;
            end

            stable = false;
            norm(xf_ballistic(1:2) - pos_trial(end, 1:2).')
            if t_trial(end) == sim_time
                if norm(xf_ballistic(1:2) - pos_trial(end, 1:2).') < reach_tol
                    stable = true;
                end
            end

            dist_to_target = vecnorm(pos_trial - xf_ballistic.', 2, 2);
            speed          = vecnorm(vel_trial, 2, 2);
            reach_mask     = (dist_to_target < reach_tol) & (speed < vel_tol);
            reach_idx      = find(reach_mask, 1, 'first');
            if isempty(reach_idx)
                time_to_land = NaN;
            else
                time_to_land = t_trial(reach_idx);
            end

            results(i, k, nb).time_to_land = time_to_land;
            results(i, k, nb).stable       = stable;
            results(i, k, nb).trajectory   = struct( ...
                'time', t_trial, ...
                'pos',  pos_trial, ...
                'vel',  vel_trial, ...
                'ctrl', ctrl_trial);
        end
    end
end

save("trajectory_optimization_log.mat")

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
set(0, 'DefaultAxesFontName', 'Times');
set(0, 'defaultUicontrolFontName', 'Times');

%% Flatten results
stable_arr = reshape([results.stable], n_deploy, n_ct, n_nb);

%% Kinetic energy overlay
% Nebula-style colormap: blue -> purple -> magenta, indexed by deploy number.
nebula_line = [linspace(0, 1, n_deploy).', zeros(n_deploy, 1), ones(n_deploy, 1)];
nebula_full = [linspace(0, 1, 256).',      zeros(256, 1),      ones(256, 1)];

ts = 0:0.1:sim_time;
figure
hold on
grid on
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            if ~stable_arr(i, k, nb)
                continue
            end
            tr = results(i, k, nb).trajectory;
            if isempty(tr) || isempty(tr.time)
                continue
            end
            v_interp = interp1(tr.time, tr.vel, ts);
            ke       = 0.5*m*sum(v_interp.^2, 2);
            h = plot(ts, ke);
            h.Color = [nebula_line(i, :) 0.5];
        end
    end
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
fontsize(gcf, 18, "points")
exportgraphics(gcf, "figs/TO_01_kinetic_energy.png",ContentType="image", ...
     Width=1000,Height=700)

%% Control effort norm overlay (same format as TO_01)
figure
hold on
grid on
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            if ~stable_arr(i, k, nb)
                continue
            end
            tr = results(i, k, nb).trajectory;
            if isempty(tr) || isempty(tr.time)
                continue
            end
            u_interp = interp1(tr.time, tr.ctrl, ts);
            u_norm   = vecnorm(u_interp, 2, 2);
            h = plot(ts, u_norm);
            h.Color = [nebula_line(i, :) 0.5];
        end
    end
end
colormap(gca, nebula_full)
clim([deploy_pct_vec(1) deploy_pct_vec(end)])
cb = colorbar;
cb.Label.String = "Deployment % along mission trajectory";
title("dSMC control effort norm: successful landings only")
xlabel("Time (s)")
ylabel("\|ctrlout\|")
xlim([0 10])
fontsize(gcf, 18, "points")
exportgraphics(gcf, "figs/TO_06_control_norm.png",ContentType="image", ...
     Width=1000,Height=700)

%% Settling time: first time the trajectory comes within reach_tol (absolute) of target
settle_t_arr = nan(n_deploy, n_ct, n_nb);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            tr  = results(i, k, nb).trajectory;
            tgt = results(i, k, nb).target_deploy_idx;
            if isempty(tr) || isempty(tr.time) || isnan(tgt)
                continue
            end
            xf_ik = squeeze(land_pos(:, tgt, k));
            dist  = vecnorm(tr.pos(:, 1:2) - xf_ik(1:2).', 2, 2);   % horizontal, matches stable flag
            idx   = find(dist <= reach_tol, 1, 'first');
            if ~isempty(idx)
                settle_t_arr(i, k, nb) = tr.time(idx);
            end
        end
    end
end

figure
hold on
grid on
markers   = {'o', 's', 'd', '^', 'v', '>', '<'};
ct_labels = ["center", "left +50 m", "right -50 m", "left +100 m", "right -100 m", ...
             "left +150 m", "right -150 m"];
h_handles = gobjects(1, n_ct);
for k = 1:n_ct
    xs = [];
    ys = [];
    sm = [];
    for i = 1:n_deploy
        for nb = 1:n_nb
            if isnan(results(i, k, nb).target_deploy_idx)
                continue
            end
            xs(end+1) = deploy_pct_vec(i);          %#ok<SAGROW>
            ys(end+1) = settle_t_arr(i, k, nb);     %#ok<SAGROW>
            sm(end+1) = stable_arr(i, k, nb);       %#ok<SAGROW>
        end
    end
    sm = logical(sm);
    scatter(xs(sm),  ys(sm),  120, 'g', markers{k}, 'filled', 'MarkerFaceAlpha', 0.7)
    scatter(xs(~sm), ys(~sm), 120, 'r', markers{k}, 'filled', 'MarkerFaceAlpha', 0.7)
    h_handles(k) = scatter(NaN, NaN, 120, 'k', markers{k}, 'filled');
end
xlabel("Deployment % along mission trajectory")
ylabel(sprintf("Time to reach within %g m horizontal of target (s)", reach_tol))
title("dSMC settling time (first XY pass within reach\_tol)")
legend(h_handles, ct_labels, 'Location', 'best')
xlim([0 90])
ylim([0 sim_time])
fontsize(gcf, 18, "points")
exportgraphics(gcf, "figs/TO_02_settling_time.png",ContentType="image", ...
     Width=1000,Height=700)

%% Deployment-point success rate (averaged over all valid ct x neighbor trials)
deploy_success = zeros(n_deploy, 1);
deploy_total   = zeros(n_deploy, 1);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            if isnan(results(i, k, nb).target_deploy_idx)
                continue
            end
            deploy_total(i) = deploy_total(i) + 1;
            if stable_arr(i, k, nb)
                deploy_success(i) = deploy_success(i) + 1;
            end
        end
    end
end
deploy_ratio = deploy_success ./ max(deploy_total, 1);

% Red-yellow-green diverging colormap (0 = red, 0.5 = yellow, 1 = green)
n_half   = 32;
ryg_cmap = [[ones(n_half, 1); linspace(1, 0, n_half).'], ...
            [linspace(0, 1, n_half).'; ones(n_half, 1)], ...
            zeros(2*n_half, 1)];

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
colormap(gca, ryg_cmap)
clim([0 1])
cb = colorbar;
cb.Label.String = "Deployment success ratio (all ct \times neighbor targets)";
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC deployment-point success rate")
fontsize(gcf, 18, "points")
exportgraphics(gcf, "figs/TO_03_stabilization_envelope.png",ContentType="image", ...
     Width=1000,Height=700)

%% Landing-point reachability
% Each landing point (target deploy idx j, ct k) is attempted by up to 3
% deploys (j-1, j, j+1). Color the landing marker by success ratio and the
% dotted deploy->target line by that specific trial's success/failure.
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

figure
axis equal
grid on
hold on
h_traj = plot3(ballistic_solution.trajectory(:,10), ...
               ballistic_solution.trajectory(:,11), ...
               ballistic_solution.trajectory(:,12), '-b', 'LineWidth', 2);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            tgt = results(i, k, nb).target_deploy_idx;
            if isnan(tgt)
                continue
            end
            if stable_arr(i, k, nb)
                line_color = [0 0.6 0 0.5];
            else
                line_color = [0.8 0 0 0.5];
            end
            plot3([deploy_pos(1,i) land_pos(1,tgt,k)], ...
                  [deploy_pos(2,i) land_pos(2,tgt,k)], ...
                  [deploy_pos(3,i) land_pos(3,tgt,k)], ...
                  ':', 'Color', line_color, 'LineWidth', 0.8, 'HandleVisibility', 'off')
        end
    end
end
land_x_flat = reshape(land_pos(1,:,:), [], 1);
land_y_flat = reshape(land_pos(2,:,:), [], 1);
land_z_flat = reshape(land_pos(3,:,:), [], 1);
ratio_flat  = landing_ratio(:);
scatter3(land_x_flat, land_y_flat, land_z_flat, 160, ratio_flat, 'filled', 'MarkerEdgeColor', 'k')
colormap(gca, ryg_cmap)
clim([0 1])
cb = colorbar;
cb.Label.String = "Landing-point success ratio";
view(3)
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC landing reachability (each target attempted by neighboring deploys)")
legend(h_traj, "Mission trajectory", 'Location', 'best')
fontsize(gcf, 18, "points")
exportgraphics(gcf, "figs/TO_04_landing_envelope.png",ContentType="image", ...
     Width=1000,Height=700)

%% Unique reachability (Shapley-weighted credit, per-success normalized)
% reachers{j,k} = list of deploy indices that successfully stabilized at landing point (j,k).
reachers = cell(n_deploy, n_ct);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            tgt = results(i, k, nb).target_deploy_idx;
            if isnan(tgt) || ~stable_arr(i, k, nb)
                continue
            end
            reachers{tgt, k}(end+1) = i;
        end
    end
end

unique_credit = zeros(n_deploy, 1);
n_successes   = zeros(n_deploy, 1);
for i = 1:n_deploy
    for k = 1:n_ct
        for nb = 1:n_nb
            if ~stable_arr(i, k, nb)
                continue
            end
            tgt = results(i, k, nb).target_deploy_idx;
            if isnan(tgt)
                continue
            end
            n_successes(i)   = n_successes(i) + 1;
            unique_credit(i) = unique_credit(i) + 1 / numel(reachers{tgt, k});
        end
    end
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
colormap(gca, ryg_cmap)
clim([0 1])
cb = colorbar;
cb.Label.String = "Unique reachability (Shapley credit per successful hit)";
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("dSMC deployment-point unique reachability")
fontsize(gcf, 18, "points")
exportgraphics(gcf, "figs/TO_05_unique_reachability.png",ContentType="image", ...
     Width=1000,Height=700)
