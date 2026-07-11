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


load_system("lqr_swarm")
load_system("pid_redux")
load_system("smc_swarm")
load_system("discrete_smc_swarm")

%% Parametric Uncertainty
mL = 1.6;
m1 = 0.6;
m2 = 1.2;
m3 = 1.0;

KL = 0.8;
K1 = 0.7;
K2 = 0.6;
K3 = 0.5;

A = [0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 -1 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0;
     0 0 0 0;
     -KL/mL 0 0 0;
     0 KL/Jxx 0 0;
     0 0 KL/Jyy 0;
     0 0 0 KL/Jzz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];
A1 = [0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 -1 0 0 0 0 0 0 0 0 0];
B1 = [0 0 0 0;
     0 0 0 0;
     -K1/m1 0 0 0;
     0 K1/Jxx 0 0;
     0 0 K1/Jyy 0;
     0 0 0 K1/Jzz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];
A2 = [0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 -1 0 0 0 0 0 0 0 0 0];
B2 = [0 0 0 0;
     0 0 0 0;
     -K2/m2 0 0 0;
     0 K2/Jxx 0 0;
     0 0 K2/Jyy 0;
     0 0 0 K2/Jzz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];
A3 = [0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 -1 0 0 0 0 0 0 0 0 0];
B3 = [0 0 0 0;
     0 0 0 0;
     -K3/m3 0 0 0;
     0 K3/Jxx 0 0;
     0 0 K3/Jyy 0;
     0 0 0 K3/Jzz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

vel0 = [0; 0; 0];
rotvel0 = [1; 1; 0];
rot0 = [0; 0; 0];
x0_step = [2; 4; 10];
xf = [0; 0; 0];
xi = [vel0; rotvel0; rot0; x0_step];

simcase = STEP;
set_param("lqr_swarm","StopTime","50", 'SimulationMode','Rapid')
step_out_lqr_uncertainty = sim("lqr_swarm")
set_param("pid_redux","StopTime","50", 'SimulationMode','Rapid')
step_out_pid_uncertainty = sim("pid_redux")
set_param("smc_swarm","StopTime","50", 'SimulationMode','Rapid')
step_out_smc_uncertainty = sim("smc_swarm")
set_param("discrete_smc_swarm","StopTime","50", 'SimulationMode','Rapid')
step_out_discrete_smc_uncertainty = sim("discrete_smc_swarm")

%% Reset to nominal
% No parametric uncertainty
m = 0.8; % kg

% rebuild linearized A, B at nominal mass
A = [0 0 0 0 0 0 -g 0 0 0 0 0;
     0 0 0 0 0 0 0 g 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     1 0 0 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0 0 0;
     0 0 -1 0 0 0 0 0 0 0 0 0];
B = [0 0 0 0;
     0 0 0 0;
     -1/m 0 0 0;
     0 1/Jxx 0 0;
     0 0 1/Jyy 0;
     0 0 0 1/Jzz;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0];

A1 = A; A2 = A; A3 = A;
B1 = B; B2 = B; B3 = B;

%% Step decrease trajectory
vel0 = [0; 0; 0];
rotvel0 = [1; 1; 0];
rot0 = [0; 0; 0];
x0_step = [2; 4; 10];
xf = [0; 0; 0];
xi = [vel0; rotvel0; rot0; x0_step];
simcase = STEP;

set_param("lqr_swarm","StopTime","50", 'SimulationMode','Rapid')
step_out_lqr = sim("lqr_swarm")
set_param("pid_redux","StopTime","50", 'SimulationMode','Rapid')
step_out_pid = sim("pid_redux")
set_param("smc_swarm","StopTime","50", 'SimulationMode','Rapid')
step_out_smc = sim("smc_swarm")
set_param("discrete_smc_swarm","StopTime","50", 'SimulationMode','Rapid')
step_out_discrete_smc = sim("discrete_smc_swarm")

%% Figure-8 and spiral trajectories
vel0 = [0; 0; 0];
rotvel0 = [0; 0; 0];
rot0 = [0; 0; 0];
x0 = [0; 0; 0];
xf = [2; 4; 10];
xi = [vel0; rotvel0; rot0; x0];

simcase = F8;
set_param("lqr_swarm","StopTime","150", 'SimulationMode','Rapid')
f8_out_lqr = sim("lqr_swarm")
set_param("pid_redux","StopTime","150", 'SimulationMode','Rapid')
f8_out_pid = sim("pid_redux")
set_param("smc_swarm","StopTime","150", 'SimulationMode','Rapid')
f8_out_smc = sim("smc_swarm")
set_param("discrete_smc_swarm","StopTime","150", 'SimulationMode','Rapid')
f8_out_discrete_smc = sim("discrete_smc_swarm")

simcase = SPIRAL;
set_param("lqr_swarm","StopTime","250", 'SimulationMode','Rapid')
spiral_out_lqr = sim("lqr_swarm")
set_param("pid_redux","StopTime","250", 'SimulationMode','Rapid')
spiral_out_pid = sim("pid_redux")
set_param("smc_swarm","StopTime","250", 'SimulationMode','Rapid')
spiral_out_smc = sim("smc_swarm")
set_param("discrete_smc_swarm","StopTime","250", 'SimulationMode','Rapid')
spiral_out_discrete_smc = sim("discrete_smc_swarm")

%% Ballistic case
env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');

launch.Vo      = 94;   % muzzle velocity (m/s); operational launch, gentle apogee deploy
launch.el      = 64;   % departure elevation (deg); recovers reachability optimum
launch.az      = 15;   % horizontal angle of departure in deg (pos to right)
launch.w_z0    = 1;    % initial pitch rate in rad/s (pos nose up)
launch.w_y0    = 0.5;  % initial transverse yaw rate in rad/s (pos for left yaw)
launch.alpha_0 = 2;    % exit elevation (deg)
launch.beta_0  = -0.5; % exit azimuth (deg)
% munition CG initial position wrt inertial frame
launch.x_0     = 0;    % x-axis (m) - range direction
launch.y_0     = 0;    % y-axis (m) - altitude
launch.z_0     = 0;    % z-axis (m) - cross-range direction
launch.t_max   = 300;  % sim end time (s)
launch.p       = -8.379; % axial launch spin (rad/s); restores prior deploy rate (apogee h.r~-0.81)

ballistic_solution = eom2(launch, env, false);

load_system("lqr_swarm_single")
load_system("pid_redux_single")
load_system("smc_swarm_single")
load_system("discrete_smc_swarm_single")

deploy_point = ballistic_solution.trajectory(ballistic_solution.apogee_idx, :).';
% Deploy seed: attitude from the pointing vector (cols 7:9), velocities and
% rates mapped world->body consistently (2026-07-10 fix). Trajectory cols
% 13:15 now carry the same derived attitude, backfilled by eom2 -- still
% interpolate r and call the helper rather than reading the angle columns.
xi = ballistic_deploy_state(deploy_point(1:3), deploy_point(4:6), ...
    deploy_point(7:9), deploy_point(10:12));
xf_ballistic = [xi(10:11,:);0];
% Snapshot for the fig 38 ballistic settling block: the envelope loop below
% overwrites xf_ballistic per test point.
xf_ballistic_apogee = xf_ballistic;

simcase = STABILIZE;
set_param("lqr_swarm_single","StopTime","100", 'SimulationMode','Rapid')
step_out_lqr_ballistic = sim("lqr_swarm_single")
set_param("pid_redux_single","StopTime","100", 'SimulationMode','Rapid')
step_out_pid_ballistic = sim("pid_redux_single")
set_param("smc_swarm_single","StopTime","100", 'SimulationMode','Rapid')
step_out_smc_ballistic = sim("smc_swarm_single")
set_param("discrete_smc_swarm_single","StopTime","100", 'SimulationMode','Rapid')
step_out_discrete_smc_ballistic = sim("discrete_smc_swarm_single")

% The four apogee runs are figure-only (never scored by ballistic_success),
% so an early stop (in-model 1e5 blowup guard) would otherwise flow silently
% into figs 36-38 as an interp1 NaN tail (bugsweep 2026-07-10, finding 17).
apogee_runs  = {step_out_lqr_ballistic, step_out_pid_ballistic, ...
                step_out_smc_ballistic, step_out_discrete_smc_ballistic};
apogee_names = ["LQR", "PID", "cSMC", "dSMC"];
for a = 1:numel(apogee_runs)
    t_end_a = apogee_runs{a}.posout.Time(end);
    if t_end_a < 100 - 1e-6
        warning("analysis:apogee_early_stop", ...
            "Apogee %s run stopped at t=%.2f s < 100 s (1e5 blowup guard); " + ...
            "its fig 36-38 ballistic cells will be blank/truncated.", ...
            apogee_names(a), t_end_a);
    end
end

%% Ballistic envelope testing
env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');

launch.Vo      = 94;   % muzzle velocity (m/s); operational launch, gentle apogee deploy
launch.el      = 64;   % departure elevation (deg); recovers reachability optimum
launch.az      = 15;   % horizontal angle of departure in deg (pos to right)
launch.w_z0    = 1;    % initial pitch rate in rad/s (pos nose up)
launch.w_y0    = 0.5;  % initial transverse yaw rate in rad/s (pos for left yaw)
launch.alpha_0 = 2;    % exit elevation (deg)
launch.beta_0  = -0.5; % exit azimuth (deg)
% munition CG initial position wrt inertial frame
launch.x_0     = 0;    % x-axis (m) - range direction
launch.y_0     = 0;    % y-axis (m) - altitude
launch.z_0     = 0;    % z-axis (m) - cross-range direction
launch.t_max   = 300;  % sim end time (s)
launch.p       = -8.379; % axial launch spin (rad/s); restores prior deploy rate (apogee h.r~-0.81)

ballistic_solution = eom2(launch, env, false);

test_points = 1:10:size(ballistic_solution.trajectory, 1);
% Never test the appended ground-impact row (altitude 0, full impact
% velocity): with the fixed 0.1 s output grid, 1:10:rows lands on it whenever
% rows == 1 (mod 10), and the criterion never checks z (bugsweep 2026-07-10
% finding 11).
test_points(test_points == size(ballistic_solution.trajectory, 1)) = [];
n_test = size(test_points, 2);
envelope_lqr_mask = false(1, n_test);
envelope_pid_mask = false(1, n_test);
envelope_smc_mask = false(1, n_test);
envelope_dsmc_nosat_mask = false(1, n_test);
envelope_dsmc_sat_mask   = false(1, n_test);
for i = 1:size(test_points,2)
    idx = test_points(i);
    deploy_point = ballistic_solution.trajectory(idx, :).';
    xi = ballistic_deploy_state(deploy_point(1:3), deploy_point(4:6), ...
        deploy_point(7:9), deploy_point(10:12));
    xf_ballistic = [xi(10:11,:);0];
    
    simcase = STABILIZE;
    set_param("lqr_swarm_single","StopTime","30", 'SimulationMode','Rapid')
    lqr_trial = sim("lqr_swarm_single")
    set_param("pid_redux_single","StopTime","30", 'SimulationMode','Rapid')
    pid_trial = sim("pid_redux_single")
    set_param("smc_swarm_single","StopTime","30", 'SimulationMode','Rapid')
    smc_trial = sim("smc_swarm_single")
    set_param("discrete_smc_swarm_single","StopTime","30", 'SimulationMode','Rapid')
    constants_struct.saturation_on = false;
    constants_struct.unconstrained = true;    % nosat arm: canonical dsmc_no_constraints (no motor limits)
    discrete_smc_trial_nosat = sim("discrete_smc_swarm_single")
    constants_struct.saturation_on = true;
    constants_struct.unconstrained = false;   % sat arm: Plan A+ handler (+ per-rotor clip)
    discrete_smc_trial_sat   = sim("discrete_smc_swarm_single")

    % Success scored post-hoc by ballistic_success (eq:ballistic-success):
    % full 30 s (blowup guard never tripped) + final XY within 10 m of
    % deploy + velocity ratio <= 2 whole-trace + rotation-rate ratio <= 2
    % for t > 1 s (grace window, default RotGraceT). rotvelout is the
    % plant's Euler-angle rates [thetadot phidot psidot]
    % (system_dynamics.m xdot(7:9)) -- the signal the termination charts norm.
    % lqr/dsmc log a 3-D [3x1xT] array, pid/smc a 2-D [Tx3]; orient to Tx3 by
    % raw ndims, NOT a size==3 heuristic (which mis-orients [3x1xT] when T==3
    % exactly -- bugsweep 2026-07-10 finding 12).
    v0_norm = norm(xi(1:3));
    rot_lqr = squeeze(lqr_trial.rotvelout.Data);
    if ndims(lqr_trial.rotvelout.Data) == 3, rot_lqr = rot_lqr.'; end
    rot_pid = squeeze(pid_trial.rotvelout.Data);
    if ndims(pid_trial.rotvelout.Data) == 3, rot_pid = rot_pid.'; end
    rot_smc = squeeze(smc_trial.rotvelout.Data);
    if ndims(smc_trial.rotvelout.Data) == 3, rot_smc = rot_smc.'; end
    rot_dsmc_nosat = squeeze(discrete_smc_trial_nosat.rotvelout.Data);
    if ndims(discrete_smc_trial_nosat.rotvelout.Data) == 3, rot_dsmc_nosat = rot_dsmc_nosat.'; end
    rot_dsmc_sat = squeeze(discrete_smc_trial_sat.rotvelout.Data);
    if ndims(discrete_smc_trial_sat.rotvelout.Data) == 3, rot_dsmc_sat = rot_dsmc_sat.'; end
    envelope_lqr_mask(i) = ballistic_success(lqr_trial.posout.Time, ...
        squeeze(lqr_trial.posout.Data(1,:,:)).', squeeze(lqr_trial.velout.Data(1,:,:)).', ...
        xi(10:11), v0_norm, 30, RotVel=rot_lqr);
    envelope_pid_mask(i) = ballistic_success(pid_trial.posout.Time, ...
        squeeze(pid_trial.posout.Data(1,:,:)).', squeeze(pid_trial.velout.Data(1,:,:)).', ...
        xi(10:11), v0_norm, 30, RotVel=rot_pid);
    envelope_smc_mask(i) = ballistic_success(smc_trial.posout.Time, ...
        squeeze(smc_trial.posout.Data(1,:,:)).', squeeze(smc_trial.velout.Data(1,:,:)).', ...
        xi(10:11), v0_norm, 30, RotVel=rot_smc);
    envelope_dsmc_nosat_mask(i) = ballistic_success(discrete_smc_trial_nosat.posout.Time, ...
        squeeze(discrete_smc_trial_nosat.posout.Data(1,:,:)).', squeeze(discrete_smc_trial_nosat.velout.Data(1,:,:)).', ...
        xi(10:11), v0_norm, 30, RotVel=rot_dsmc_nosat);
    envelope_dsmc_sat_mask(i) = ballistic_success(discrete_smc_trial_sat.posout.Time, ...
        squeeze(discrete_smc_trial_sat.posout.Data(1,:,:)).', squeeze(discrete_smc_trial_sat.velout.Data(1,:,:)).', ...
        xi(10:11), v0_norm, 30, RotVel=rot_dsmc_sat);
end
ballistic_envelope_lqr = test_points(envelope_lqr_mask);
ballistic_envelope_pid = test_points(envelope_pid_mask);
ballistic_envelope_smc = test_points(envelope_smc_mask);
ballistic_envelope_dsmc_nosat = test_points(envelope_dsmc_nosat_mask);
ballistic_envelope_dsmc_sat   = test_points(envelope_dsmc_sat_mask);

%% Plot setup
close all;
set_default_fonts();

set(0,'DefaultFigureVisible','on')
set(gcf,'visible','on')
%% Positional Plots


subplot(5,4,1)
hold on
for i=1:10
    plot3(squeeze(step_out_lqr.posout.Data(i,1,:)), ...
        squeeze(step_out_lqr.posout.Data(i,2,:)), ...
        squeeze(step_out_lqr.posout.Data(i,3,:)))
    title("Step, LQR")
    view(-37.5, 30)
end

subplot(5,4,2)
hold on
for i=1:10
    plot3(squeeze(step_out_smc.posout.Data(i,1,:)), ...
        squeeze(step_out_smc.posout.Data(i,2,:)), ...
        squeeze(step_out_smc.posout.Data(i,3,:)))
    title("Step, cSMC")
    view(-37.5, 30)
end

subplot(5,4,3)
hold on
for i=1:10
    plot3(squeeze(step_out_pid.posout.Data(i,1,:)), ...
        squeeze(step_out_pid.posout.Data(i,2,:)), ...
        squeeze(step_out_pid.posout.Data(i,3,:)))
    title("Step, PID")
    view(-37.5, 30)
end

subplot(5,4,4)
hold on
for i=1:10
    plot3(squeeze(step_out_discrete_smc.posout.Data(i,1,:)), ...
        squeeze(step_out_discrete_smc.posout.Data(i,2,:)), ...
        squeeze(step_out_discrete_smc.posout.Data(i,3,:)))
    title("Step, dSMC")
    view(-37.5, 30)
end

subplot(5,4,5)
hold on
for i=1:10
    plot3(squeeze(step_out_lqr_uncertainty.posout.Data(i,1,:)), ...
        squeeze(step_out_lqr_uncertainty.posout.Data(i,2,:)), ...
        squeeze(step_out_lqr_uncertainty.posout.Data(i,3,:)))
    title("Step, LQR, uncertainty")
    view(-37.5, 30)
end

subplot(5,4,6)
hold on
for i=1:10
    plot3(squeeze(step_out_smc_uncertainty.posout.Data(i,1,:)), ...
        squeeze(step_out_smc_uncertainty.posout.Data(i,2,:)), ...
        squeeze(step_out_smc_uncertainty.posout.Data(i,3,:)))
    title("Step, SMC, uncertainty")
    view(-37.5, 30)
end

subplot(5,4,7)
hold on
for i=1:10
    plot3(squeeze(step_out_pid_uncertainty.posout.Data(i,1,:)), ...
        squeeze(step_out_pid_uncertainty.posout.Data(i,2,:)), ...
        squeeze(step_out_pid_uncertainty.posout.Data(i,3,:)))
    title("Step, PID, uncertainty")
    view(-37.5, 30)
end

subplot(5,4,8)
hold on
for i=1:10
    plot3(squeeze(step_out_discrete_smc_uncertainty.posout.Data(i,1,:)), ...
          squeeze(step_out_discrete_smc_uncertainty.posout.Data(i,2,:)), ...
          squeeze(step_out_discrete_smc_uncertainty.posout.Data(i,3,:)))
    title("Step, dSMC, uncertainty")
    view(-37.5, 30)
end

subplot(5,4,9)
hold on
for i=1:10
    plot3(squeeze(f8_out_lqr.posout.Data(i,1,:)), ...
        squeeze(f8_out_lqr.posout.Data(i,2,:)), ...
        squeeze(f8_out_lqr.posout.Data(i,3,:)))
    title("Figure 8, LQR")
    view(-37.5, 30)
end

subplot(5,4,10)
hold on
for i=1:10
    plot3(squeeze(f8_out_smc.posout.Data(i,1,:)), ...
        squeeze(f8_out_smc.posout.Data(i,2,:)), ...
        squeeze(f8_out_smc.posout.Data(i,3,:)))
    title("Figure 8, cSMC")
    view(-37.5, 30)
end

subplot(5,4,11)
hold on
for i=1:10
    plot3(squeeze(f8_out_pid.posout.Data(i,1,:)), ...
        squeeze(f8_out_pid.posout.Data(i,2,:)), ...
        squeeze(f8_out_pid.posout.Data(i,3,:)))
    title("Figure 8, PID")
    view(-37.5, 30)
end

subplot(5,4,12)
hold on
for i=1:10
    plot3(squeeze(f8_out_discrete_smc.posout.Data(i,1,:)), ...
          squeeze(f8_out_discrete_smc.posout.Data(i,2,:)), ...
          squeeze(f8_out_discrete_smc.posout.Data(i,3,:)))
    title("Figure 8, dSMC")
    view(-37.5, 30)
end

subplot(5,4,13)
hold on
for i=1:10
    plot3(squeeze(spiral_out_lqr.posout.Data(i,1,:)), ...
        squeeze(spiral_out_lqr.posout.Data(i,2,:)), ...
        squeeze(spiral_out_lqr.posout.Data(i,3,:)))
    title("Arithmetic Spiral, LQR")
    view(-37.5, 30)
end

subplot(5,4,14)
hold on
for i=1:10
    plot3(squeeze(spiral_out_smc.posout.Data(i,1,:)), ...
        squeeze(spiral_out_smc.posout.Data(i,2,:)), ...
        squeeze(spiral_out_smc.posout.Data(i,3,:)))
    title("Arithmetic Spiral, cSMC")
    view(-37.5, 30)
end

subplot(5,4,15)
hold on
for i=1:10
    plot3(squeeze(spiral_out_pid.posout.Data(i,1,:)), ...
        squeeze(spiral_out_pid.posout.Data(i,2,:)), ...
        squeeze(spiral_out_pid.posout.Data(i,3,:)))
    title("Arithmetic Spiral, PID")
    view(-37.5, 30)
end

subplot(5,4,16)
hold on
for i=1:10
    plot3(squeeze(spiral_out_discrete_smc.posout.Data(i,1,:)), ...
          squeeze(spiral_out_discrete_smc.posout.Data(i,2,:)), ...
          squeeze(spiral_out_discrete_smc.posout.Data(i,3,:)))
    title("Arithmetic Spiral, dSMC")
    view(-37.5, 30)
end

subplot(5,4,17)
hold on
for i=1:1
    plot3(squeeze(step_out_lqr_ballistic.posout.Data(i,1,:)), ...
        squeeze(step_out_lqr_ballistic.posout.Data(i,2,:)), ...
        squeeze(step_out_lqr_ballistic.posout.Data(i,3,:)))
    title("Ballistic Stabilization, LQR")
    view(-37.5, 30)
end

subplot(5,4,18)
hold on
for i=1:1
    plot3(squeeze(step_out_smc_ballistic.posout.Data(i,1,:)), ...
        squeeze(step_out_smc_ballistic.posout.Data(i,2,:)), ...
        squeeze(step_out_smc_ballistic.posout.Data(i,3,:)))
    title("Ballistic Stabilization, cSMC")
    view(-37.5, 30)
end

subplot(5,4,19)
hold on
for i=1:1
    plot3(squeeze(step_out_pid_ballistic.posout.Data(i,1,:)), ...
        squeeze(step_out_pid_ballistic.posout.Data(i,2,:)), ...
        squeeze(step_out_pid_ballistic.posout.Data(i,3,:)))
    title("Ballistic Stabilization, PID")
    view(-37.5, 30)
end

subplot(5,4,20)
hold on
for i=1:1
    plot3(squeeze(step_out_discrete_smc_ballistic.posout.Data(i,1,:)), ...
          squeeze(step_out_discrete_smc_ballistic.posout.Data(i,2,:)), ...
          squeeze(step_out_discrete_smc_ballistic.posout.Data(i,3,:)))
    title("Ballistic Stabilization, dSMC")
    view(-37.5, 30)
end


fontsize(gcf, 12, "points")
export_figure("figs/1_flight_path", Width=2400, Height=1800)

%% Desired Flight Paths
step_cmd = step_out_smc.cmdout.Data;
uncertain_cmd = step_out_smc_uncertainty.cmdout.Data;
f8_cmd = f8_out_smc.cmdout.Data;
spiral_cmd = spiral_out_smc.cmdout.Data;

step_cmd = cat(3, x0_step, step_cmd);
uncertain_cmd = cat(3, x0_step, uncertain_cmd);
f8_cmd = cat(3, [0;0;0], f8_cmd);
spiral_cmd = cat(3, [0;0;0], spiral_cmd);

% Formation X-offsets in model (posout) drone order; only col 1 (leader) is plotted below
offsets = [0 1 2 3 4 5 -1 -2 -4 -3;
           0 0 0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 0 0];

figure
subplot(2,2,1)
hold on
for i=1:1
    plot3(squeeze(step_cmd(1,:,:)) + offsets(1,i), ...
        squeeze(step_cmd(2,:,:)) + offsets(2,i), ...
        squeeze(step_cmd(3,:,:)) + offsets(3,i))
    title("Step Commands")
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("alt (m)")
    grid on
    view(-37.5, 30)
end
pbaspect([1 1 1])

subplot(2,2,2)
hold on
for i=1:1
    plot3(squeeze(uncertain_cmd(1,:)) + offsets(1,i), ...
        squeeze(uncertain_cmd(2,:)) + offsets(2,i), ...
        squeeze(uncertain_cmd(3,:)) + offsets(3,i))
    title("Uncertain Step Commands")
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("alt (m)")
    grid on
    view(-37.5, 30)
end
pbaspect([1 1 1])

subplot(2,2,3)
hold on
for i=1:1
    plot3(squeeze(f8_cmd(1,:)) + offsets(1,i), ...
        squeeze(f8_cmd(2,:)) + offsets(2,i), ...
        squeeze(f8_cmd(3,:)) + offsets(3,i))
    title("Figure-8 Commands")
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("alt (m)")
    grid on
    view(-37.5, 30)
end
pbaspect([1 1 1])

subplot(2,2,4)
hold on
for i=1:1
    plot3(squeeze(spiral_cmd(1,:)) + offsets(1,i), ...
        squeeze(spiral_cmd(2,:)) + offsets(2,i), ...
        squeeze(spiral_cmd(3,:)) + offsets(3,i))
    title("Arithmetic Spiral Commands")
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("alt (m)")
    grid on
    view(-37.5, 30)
end
pbaspect([1 1 1])

export_figure("figs/2_flight_path_cmds", Width=2000, Height=1600)

%% Time Series Comparisons
step_time = 0:0.1:50;
ballistic_time = 0:0.1:100;
f8_time = 0:0.1:150;
spiral_time = 0:0.1:250;

% Step
interp_step_lqr_pos = interp1(step_out_lqr.posout.Time, permute(step_out_lqr.posout.Data, [3 1 2]), step_time);
interp_step_smc_pos = interp1(step_out_smc.posout.Time, permute(step_out_smc.posout.Data, [3 1 2]), step_time);
interp_step_discrete_smc_pos = interp1(step_out_discrete_smc.posout.Time, permute(step_out_discrete_smc.posout.Data, [3 1 2]), step_time);
interp_step_pid_pos = interp1(step_out_pid.posout.Time, permute(step_out_pid.posout.Data, [3 1 2]), step_time);
interp_step_lqr_vel = interp1(step_out_lqr.velout.Time, permute(step_out_lqr.velout.Data, [3 1 2]), step_time);
interp_step_smc_vel = interp1(step_out_smc.velout.Time, permute(step_out_smc.velout.Data, [3 1 2]), step_time);
interp_step_discrete_smc_vel = interp1(step_out_discrete_smc.velout.Time, permute(step_out_discrete_smc.velout.Data, [3 1 2]), step_time);
interp_step_pid_vel = interp1(step_out_pid.velout.Time, permute(step_out_pid.velout.Data, [3 1 2]), step_time);

temp = sum(interp_step_lqr_vel.^2, 3);
step_lqr_ke = 0.5*m*temp;
temp = sum(interp_step_smc_vel.^2, 3);
step_smc_ke = 0.5*m*temp;
temp = sum(interp_step_discrete_smc_vel.^2, 3);
step_discrete_smc_ke = 0.5*m*temp;
temp = sum(interp_step_pid_vel.^2, 3);
step_pid_ke = 0.5*m*temp;

step_lqr_min_distances = [];
step_lqr_avg_distances = [];
step_lqr_max_distances = [];
for i=1:10
    temp = repmat(interp_step_lqr_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_lqr_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_lqr_min_distances = [step_lqr_min_distances; min_distance];
    step_lqr_max_distances = [step_lqr_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_lqr_avg_distances = [step_lqr_avg_distances; avg_distance];
end

step_smc_min_distances = [];
step_smc_avg_distances = [];
step_smc_max_distances = [];
for i=1:10
    temp = repmat(interp_step_smc_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_smc_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_smc_min_distances = [step_smc_min_distances; min_distance];
    step_smc_max_distances = [step_smc_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_smc_avg_distances = [step_smc_avg_distances; avg_distance];
end

step_discrete_smc_min_distances = [];
step_discrete_smc_avg_distances = [];
step_discrete_smc_max_distances = [];
for i=1:10
    temp = repmat(interp_step_discrete_smc_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_discrete_smc_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_discrete_smc_min_distances = [step_discrete_smc_min_distances; min_distance];
    step_discrete_smc_max_distances = [step_discrete_smc_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_discrete_smc_avg_distances = [step_discrete_smc_avg_distances; avg_distance];
end

step_pid_min_distances = [];
step_pid_avg_distances = [];
step_pid_max_distances = [];
for i=1:10
    temp = repmat(interp_step_pid_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_pid_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_pid_min_distances = [step_pid_min_distances; min_distance];
    step_pid_max_distances = [step_pid_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_pid_avg_distances = [step_pid_avg_distances; avg_distance];
end

% Ballistic
interp_step_lqr_ballistic_pos = interp1(step_out_lqr_ballistic.posout.Time, permute(step_out_lqr_ballistic.posout.Data, [3 1 2]), ballistic_time);
interp_step_smc_ballistic_pos = interp1(step_out_smc_ballistic.posout.Time, permute(step_out_smc_ballistic.posout.Data, [3 1 2]), ballistic_time);
interp_step_discrete_smc_ballistic_pos = interp1(step_out_discrete_smc_ballistic.posout.Time, permute(step_out_discrete_smc_ballistic.posout.Data, [3 1 2]), ballistic_time);
interp_step_pid_ballistic_pos = interp1(step_out_pid_ballistic.posout.Time, permute(step_out_pid_ballistic.posout.Data, [3 1 2]), ballistic_time);
interp_step_lqr_ballistic_vel = interp1(step_out_lqr_ballistic.velout.Time, permute(step_out_lqr_ballistic.velout.Data, [3 1 2]), ballistic_time);
interp_step_smc_ballistic_vel = interp1(step_out_smc_ballistic.velout.Time, permute(step_out_smc_ballistic.velout.Data, [3 1 2]), ballistic_time);
interp_step_discrete_smc_ballistic_vel = interp1(step_out_discrete_smc_ballistic.velout.Time, permute(step_out_discrete_smc_ballistic.velout.Data, [3 1 2]), ballistic_time);
interp_step_pid_ballistic_vel = interp1(step_out_pid_ballistic.velout.Time, permute(step_out_pid_ballistic.velout.Data, [3 1 2]), ballistic_time);

temp = sum(interp_step_lqr_ballistic_vel.^2, 3);
step_lqr_ballistic_ke = 0.5*m*temp;
temp = sum(interp_step_smc_ballistic_vel.^2, 3);
step_smc_ballistic_ke = 0.5*m*temp;
temp = sum(interp_step_discrete_smc_ballistic_vel.^2, 3);
step_discrete_smc_ballistic_ke = 0.5*m*temp;
temp = sum(interp_step_pid_ballistic_vel.^2, 3);
step_pid_ballistic_ke = 0.5*m*temp;

% (Removed 2026-07-10, bugsweep finding 19: the four single-drone ballistic
% self-distance blocks computed identically-zero 1x10 rows -- one drone's
% trajectory minus 10 repmat copies of itself -- had zero consumers repo-
% wide, and were persisted into logs/analysis_log.mat every run.)

% Uncertainty
interp_step_lqr_uncertainty_pos = interp1(step_out_lqr_uncertainty.posout.Time, permute(step_out_lqr_uncertainty.posout.Data, [3 1 2]), step_time);
interp_step_smc_uncertainty_pos = interp1(step_out_smc_uncertainty.posout.Time, permute(step_out_smc_uncertainty.posout.Data, [3 1 2]), step_time);
interp_step_discrete_smc_uncertainty_pos = interp1(step_out_discrete_smc_uncertainty.posout.Time, permute(step_out_discrete_smc_uncertainty.posout.Data, [3 1 2]), step_time);
interp_step_pid_uncertainty_pos = interp1(step_out_pid_uncertainty.posout.Time, permute(step_out_pid_uncertainty.posout.Data, [3 1 2]), step_time);
interp_step_lqr_uncertainty_vel = interp1(step_out_lqr_uncertainty.velout.Time, permute(step_out_lqr_uncertainty.velout.Data, [3 1 2]), step_time);
interp_step_smc_uncertainty_vel = interp1(step_out_smc_uncertainty.velout.Time, permute(step_out_smc_uncertainty.velout.Data, [3 1 2]), step_time);
interp_step_discrete_smc_uncertainty_vel = interp1(step_out_discrete_smc_uncertainty.velout.Time, permute(step_out_discrete_smc_uncertainty.velout.Data, [3 1 2]), step_time);
interp_step_pid_uncertainty_vel = interp1(step_out_pid_uncertainty.velout.Time, permute(step_out_pid_uncertainty.velout.Data, [3 1 2]), step_time);

mass_uncertain = [mL; m1; m1; m2; m2; m2; m2; m2; m3; m3];

temp = sum(interp_step_lqr_uncertainty_vel.^2, 3);
step_lqr_uncertainty_ke = 0.5*(temp.*(mass_uncertain'));

temp = sum(interp_step_smc_uncertainty_vel.^2, 3);
step_smc_uncertainty_ke = 0.5*(temp.*(mass_uncertain'));

temp = sum(interp_step_discrete_smc_uncertainty_vel.^2, 3);
step_discrete_smc_uncertainty_ke = 0.5*(temp.*(mass_uncertain'));

temp = sum(interp_step_pid_uncertainty_vel.^2, 3);
step_pid_uncertainty_ke = 0.5*(temp.*(mass_uncertain'));

step_lqr_uncertainty_min_distances = [];
step_lqr_uncertainty_avg_distances = [];
step_lqr_uncertainty_max_distances = [];
for i=1:10
    temp = repmat(interp_step_lqr_uncertainty_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_lqr_uncertainty_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_lqr_uncertainty_min_distances = [step_lqr_uncertainty_min_distances; min_distance];
    step_lqr_uncertainty_max_distances = [step_lqr_uncertainty_max_distances; max_distance];
    avg_distance = mean(temp, 1);
    step_lqr_uncertainty_avg_distances = [step_lqr_uncertainty_avg_distances; avg_distance];
end

step_smc_uncertainty_min_distances = [];
step_smc_uncertainty_avg_distances = [];
step_smc_uncertainty_max_distances = [];
for i=1:10
    temp = repmat(interp_step_smc_uncertainty_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_smc_uncertainty_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_smc_uncertainty_min_distances = [step_smc_uncertainty_min_distances; min_distance];
    step_smc_uncertainty_max_distances = [step_smc_uncertainty_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_smc_uncertainty_avg_distances = [step_smc_uncertainty_avg_distances; avg_distance];
end

step_discrete_smc_uncertainty_min_distances = [];
step_discrete_smc_uncertainty_avg_distances = [];
step_discrete_smc_uncertainty_max_distances = [];
for i=1:10
    temp = repmat(interp_step_discrete_smc_uncertainty_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_discrete_smc_uncertainty_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_discrete_smc_uncertainty_min_distances = [step_discrete_smc_uncertainty_min_distances; min_distance];
    step_discrete_smc_uncertainty_max_distances = [step_discrete_smc_uncertainty_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_discrete_smc_uncertainty_avg_distances = [step_discrete_smc_uncertainty_avg_distances; avg_distance];
end

step_pid_uncertainty_min_distances = [];
step_pid_uncertainty_avg_distances = [];
step_pid_uncertainty_max_distances = [];
for i=1:10
    temp = repmat(interp_step_pid_uncertainty_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_step_pid_uncertainty_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    step_pid_uncertainty_min_distances = [step_pid_uncertainty_min_distances; min_distance];
    step_pid_uncertainty_max_distances = [step_pid_uncertainty_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    step_pid_uncertainty_avg_distances = [step_pid_uncertainty_avg_distances; avg_distance];
end

interp_f8_lqr_pos = interp1(f8_out_lqr.posout.Time, permute(f8_out_lqr.posout.Data, [3 1 2]), f8_time);
interp_f8_smc_pos = interp1(f8_out_smc.posout.Time, permute(f8_out_smc.posout.Data, [3 1 2]), f8_time);
interp_f8_discrete_smc_pos = interp1(f8_out_discrete_smc.posout.Time, permute(f8_out_discrete_smc.posout.Data, [3 1 2]), f8_time);
interp_f8_pid_pos = interp1(f8_out_pid.posout.Time, permute(f8_out_pid.posout.Data, [3 1 2]), f8_time);
interp_f8_lqr_vel = interp1(f8_out_lqr.velout.Time, permute(f8_out_lqr.velout.Data, [3 1 2]), f8_time);
interp_f8_smc_vel = interp1(f8_out_smc.velout.Time, permute(f8_out_smc.velout.Data, [3 1 2]), f8_time);
interp_f8_discrete_smc_vel = interp1(f8_out_discrete_smc.velout.Time, permute(f8_out_discrete_smc.velout.Data, [3 1 2]), f8_time);
interp_f8_pid_vel = interp1(f8_out_pid.velout.Time, permute(f8_out_pid.velout.Data, [3 1 2]), f8_time);

temp = sum(interp_f8_lqr_vel.^2, 3);
f8_lqr_ke = 0.5*m*temp;

temp = sum(interp_f8_smc_vel.^2, 3);
f8_smc_ke = 0.5*m*temp;

temp = sum(interp_f8_discrete_smc_vel.^2, 3);
f8_discrete_smc_ke = 0.5*m*temp;

temp = sum(interp_f8_pid_vel.^2, 3);
f8_pid_ke = 0.5*m*temp;

f8_lqr_min_distances = [];
f8_lqr_avg_distances = [];
f8_lqr_max_distances = [];
for i=1:10
    temp = repmat(interp_f8_lqr_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_f8_lqr_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    f8_lqr_min_distances = [f8_lqr_min_distances; min_distance];
    f8_lqr_max_distances = [f8_lqr_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    f8_lqr_avg_distances = [f8_lqr_avg_distances; avg_distance];
end

f8_smc_min_distances = [];
f8_smc_avg_distances = [];
f8_smc_max_distances = [];
for i=1:10
    temp = repmat(interp_f8_smc_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_f8_smc_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    f8_smc_min_distances = [f8_smc_min_distances; min_distance];
    f8_smc_max_distances = [f8_smc_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    f8_smc_avg_distances = [f8_smc_avg_distances; avg_distance];
end

f8_discrete_smc_min_distances = [];
f8_discrete_smc_avg_distances = [];
f8_discrete_smc_max_distances = [];
for i=1:10
    temp = repmat(interp_f8_discrete_smc_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_f8_discrete_smc_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    f8_discrete_smc_min_distances = [f8_discrete_smc_min_distances; min_distance];
    f8_discrete_smc_max_distances = [f8_discrete_smc_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    f8_discrete_smc_avg_distances = [f8_discrete_smc_avg_distances; avg_distance];
end

f8_pid_min_distances = [];
f8_pid_avg_distances = [];
f8_pid_max_distances = [];
for i=1:10
    temp = repmat(interp_f8_pid_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_f8_pid_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    f8_pid_min_distances = [f8_pid_min_distances; min_distance];
    f8_pid_max_distances = [f8_pid_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    f8_pid_avg_distances = [f8_pid_avg_distances; avg_distance];
end

interp_spiral_out_lqr_pos = interp1(spiral_out_lqr.posout.Time, permute(spiral_out_lqr.posout.Data, [3 1 2]), spiral_time);
interp_spiral_out_smc_pos = interp1(spiral_out_smc.posout.Time, permute(spiral_out_smc.posout.Data, [3 1 2]), spiral_time);
interp_spiral_out_discrete_smc_pos = interp1(spiral_out_discrete_smc.posout.Time, permute(spiral_out_discrete_smc.posout.Data, [3 1 2]), spiral_time);
interp_spiral_out_pid_pos = interp1(spiral_out_pid.posout.Time, permute(spiral_out_pid.posout.Data, [3 1 2]), spiral_time);
interp_spiral_out_lqr_vel = interp1(spiral_out_lqr.velout.Time, permute(spiral_out_lqr.velout.Data, [3 1 2]), spiral_time);
interp_spiral_out_smc_vel = interp1(spiral_out_smc.velout.Time, permute(spiral_out_smc.velout.Data, [3 1 2]), spiral_time);
interp_spiral_out_discrete_smc_vel = interp1(spiral_out_discrete_smc.velout.Time, permute(spiral_out_discrete_smc.velout.Data, [3 1 2]), spiral_time);
interp_spiral_out_pid_vel = interp1(spiral_out_pid.velout.Time, permute(spiral_out_pid.velout.Data, [3 1 2]), spiral_time);

temp = sum(interp_spiral_out_lqr_vel.^2, 3);
spiral_lqr_ke = 0.5*m*temp;

temp = sum(interp_spiral_out_smc_vel.^2, 3);
spiral_smc_ke = 0.5*m*temp;

temp = sum(interp_spiral_out_discrete_smc_vel.^2, 3);
spiral_discrete_smc_ke = 0.5*m*temp;

temp = sum(interp_spiral_out_pid_vel.^2, 3);
spiral_pid_ke = 0.5*m*temp;

spiral_lqr_min_distances = [];
spiral_lqr_avg_distances = [];
spiral_lqr_max_distances = [];
for i=1:10
    temp  = repmat(interp_spiral_out_lqr_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_spiral_out_lqr_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    spiral_lqr_min_distances = [spiral_lqr_min_distances; min_distance];
    spiral_lqr_max_distances = [spiral_lqr_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    spiral_lqr_avg_distances = [spiral_lqr_avg_distances; avg_distance];
end

spiral_smc_min_distances = [];
spiral_smc_avg_distances = [];
spiral_smc_max_distances = [];
for i=1:10
    temp  = repmat(interp_spiral_out_smc_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_spiral_out_smc_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    spiral_smc_min_distances = [spiral_smc_min_distances; min_distance];
    spiral_smc_max_distances = [spiral_smc_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    spiral_smc_avg_distances = [spiral_smc_avg_distances; avg_distance];
end

spiral_discrete_smc_min_distances = [];
spiral_discrete_smc_avg_distances = [];
spiral_discrete_smc_max_distances = [];
for i=1:10
    temp  = repmat(interp_spiral_out_discrete_smc_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_spiral_out_discrete_smc_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    spiral_discrete_smc_min_distances = [spiral_discrete_smc_min_distances; min_distance];
    spiral_discrete_smc_max_distances = [spiral_discrete_smc_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    spiral_discrete_smc_avg_distances = [spiral_discrete_smc_avg_distances; avg_distance];
end

spiral_pid_min_distances = [];
spiral_pid_avg_distances = [];
spiral_pid_max_distances = [];
for i=1:10
    temp  = repmat(interp_spiral_out_pid_pos(:, i, :), [1, 10, 1]);
    temp_pos = abs(interp_spiral_out_pid_pos - temp);
    temp = sqrt(sum(temp_pos.^2, 3));
    min_distance = min(temp, [], 1);
    max_distance = max(temp, [], 1);

    spiral_pid_min_distances = [spiral_pid_min_distances; min_distance];
    spiral_pid_max_distances = [spiral_pid_max_distances; max_distance];

    avg_distance = mean(temp, 1);
    spiral_pid_avg_distances = [spiral_pid_avg_distances; avg_distance];
end

%% Distance Heatmaps
figure
hold on
subplot(3, 1, 1)
heatmap(step_lqr_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, LQR step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_lqr_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
title("Avg euclidean distance between swarm members, LQR step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
clim([0, 4]);
subplot(3, 1, 3)
heatmap(step_lqr_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Max euclidean distance between swarm members, LQR step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/3_step_lqr_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_smc_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, SMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_smc_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, SMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_smc_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, SMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/4_step_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_discrete_smc_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, dSMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_discrete_smc_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, dSMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_discrete_smc_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, dSMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/5_step_discrete_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_pid_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, PID step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_pid_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, PID step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_pid_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Max euclidean distance between swarm members, PID step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/6_step_pid_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_lqr_uncertainty_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, LQR step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_lqr_uncertainty_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, LQR step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_lqr_uncertainty_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, LQR step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/7_step_lqr_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_smc_uncertainty_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, SMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_smc_uncertainty_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, SMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_smc_uncertainty_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, SMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/8_step_smc_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_discrete_smc_uncertainty_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, dSMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_discrete_smc_uncertainty_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, dSMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_discrete_smc_uncertainty_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, dSMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/9_step_discrete_smc_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_pid_uncertainty_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, PID step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_pid_uncertainty_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, PID step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_pid_uncertainty_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, PID step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/10_step_pid_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_lqr_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, LQR figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_lqr_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, LQR figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_lqr_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, LQR figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/11_f8_lqr_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_smc_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, SMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_smc_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, SMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_smc_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, SMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/12_f8_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_discrete_smc_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, dSMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_discrete_smc_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, dSMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_discrete_smc_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, dSMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/13_f8_discrete_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_pid_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, PID figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_pid_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, PID figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_pid_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, PID figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/14_f8_pid_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_lqr_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, LQR arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_lqr_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, LQR arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_lqr_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, LQR arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/15_spiral_lqr_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_smc_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, SMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_smc_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, SMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_smc_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, SMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/16_spiral_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_discrete_smc_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, dSMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_discrete_smc_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, dSMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_discrete_smc_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, dSMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/17_spiral_discrete_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_pid_min_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Min euclidean distance between swarm members, PID arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_pid_avg_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14);
clim([0, 4]);
title("Avg euclidean distance between swarm members, PID arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_pid_max_distances, "Colormap", hot, 'FontName', 'Times', 'FontSize', 14); 
clim([0, 4]);
title("Max euclidean distance between swarm members, PID arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/18_spiral_pid_distances")

%% Delta-Distance Heatmaps
% Nominal formation, model (posout) drone order [0 1 2 3 4 5 -1 -2 -4 -3]; row order MUST match posout
initial_positions = [0 0 0;
                     1 0 0;
                     2 0 0;
                     3 0 0;
                     4 0 0;
                     5 0 0;
                     -1 0 0;
                     -2 0 0;
                     -4 0 0;
                     -3 0 0];
ideal_distances = zeros(10,10);
for i=1:10
    for j=1:10
        ideal_distances(i, j) = norm(initial_positions(i,:) - initial_positions(j,:));
    end
end

figure
hold on
subplot(3, 1, 1)
heatmap(step_lqr_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, LQR step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_lqr_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, LQR step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_lqr_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, LQR step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/19_heatmap_step_lqr_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_smc_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, SMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_smc_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, SMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_smc_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, SMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/20_heatmap_step_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_discrete_smc_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, dSMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_discrete_smc_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, dSMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_discrete_smc_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, dSMC step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/21_heatmap_step_discrete_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_pid_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, PID step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_pid_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, PID step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_pid_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, PID step response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/22_heatmap_step_pid_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_lqr_uncertainty_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, LQR step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_lqr_uncertainty_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, LQR step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_lqr_uncertainty_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, LQR step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/23_heatmap_step_lqr_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_smc_uncertainty_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, SMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_smc_uncertainty_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, SMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_smc_uncertainty_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, SMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/24_heatmap_step_smc_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_discrete_smc_uncertainty_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, dSMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_discrete_smc_uncertainty_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, dSMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_discrete_smc_uncertainty_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, dSMC step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/25_heatmap_step_discrete_smc_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(step_pid_uncertainty_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, PID step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(step_pid_uncertainty_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, PID step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(step_pid_uncertainty_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, PID step response w/ uncertainty")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/26_heatmap_step_pid_uncertainty_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_lqr_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, LQR figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_lqr_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, LQR figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_lqr_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, LQR figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/27_heatmap_f8_lqr_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_smc_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, SMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_smc_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, SMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_smc_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, SMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/28_heatmap_f8_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_discrete_smc_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, dSMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_discrete_smc_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, dSMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_discrete_smc_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, dSMC figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/29_heatmap_f8_discrete_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(f8_pid_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, PID figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(f8_pid_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, PID figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(f8_pid_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, PID figure-8 response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/30_heatmap_f8_pid_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_lqr_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, LQR arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_lqr_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, LQR arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_lqr_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, LQR arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/31_heatmap_spiral_lqr_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_smc_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, SMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_smc_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, SMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_smc_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, SMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/32_heatmap_spiral_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_discrete_smc_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, dSMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_discrete_smc_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, dSMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_discrete_smc_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, dSMC arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/33_heatmap_spiral_discrete_smc_distances")

figure
hold on
subplot(3, 1, 1)
heatmap(spiral_pid_min_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between min euclidean distance between drones, and nominal separation, PID arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 2)
heatmap(spiral_pid_avg_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times');
clim([-1 1])
title("Difference between avg euclidean distance between drones, and nominal separation, PID arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")
subplot(3, 1, 3)
heatmap(spiral_pid_max_distances-ideal_distances, "Colormap", [flipud(jet);jet], 'FontName', 'Times'); 
clim([-1 1])
title("Difference between max euclidean distance between drones, and nominal separation, PID arithmetic spiral response")
xlabel("Drone index (1 is leader)")
ylabel("Drone index (1 is leader)")

export_figure("figs/34_heatmap_spiral_pid_distances")

%% Score Plots
step_lqr_min_delta_score = norm(step_lqr_min_distances-ideal_distances, 'fro')/norm(step_lqr_min_distances+ideal_distances, 'fro');
step_lqr_avg_delta_score = norm(step_lqr_avg_distances-ideal_distances, 'fro')/norm(step_lqr_avg_distances+ideal_distances, 'fro');
step_lqr_max_delta_score = norm(step_lqr_max_distances-ideal_distances, 'fro')/norm(step_lqr_max_distances+ideal_distances, 'fro');
step_smc_min_delta_score = norm(step_smc_min_distances-ideal_distances, 'fro')/norm(step_smc_min_distances+ideal_distances, 'fro');
step_smc_avg_delta_score = norm(step_smc_avg_distances-ideal_distances, 'fro')/norm(step_smc_avg_distances+ideal_distances, 'fro');
step_smc_max_delta_score = norm(step_smc_max_distances-ideal_distances, 'fro')/norm(step_smc_max_distances+ideal_distances, 'fro');
step_discrete_smc_min_delta_score = norm(step_discrete_smc_min_distances-ideal_distances, 'fro')/norm(step_discrete_smc_min_distances+ideal_distances, 'fro');
step_discrete_smc_avg_delta_score = norm(step_discrete_smc_avg_distances-ideal_distances, 'fro')/norm(step_discrete_smc_avg_distances+ideal_distances, 'fro');
step_discrete_smc_max_delta_score = norm(step_discrete_smc_max_distances-ideal_distances, 'fro')/norm(step_discrete_smc_max_distances+ideal_distances, 'fro');
step_pid_min_delta_score = norm(step_pid_min_distances-ideal_distances, 'fro')/norm(step_pid_min_distances+ideal_distances, 'fro');
step_pid_avg_delta_score = norm(step_pid_avg_distances-ideal_distances, 'fro')/norm(step_pid_avg_distances+ideal_distances, 'fro');
step_pid_max_delta_score = norm(step_pid_max_distances-ideal_distances, 'fro')/norm(step_pid_max_distances+ideal_distances, 'fro');
step_score_mat = [step_lqr_min_delta_score step_smc_min_delta_score step_discrete_smc_min_delta_score step_pid_min_delta_score; 
                  step_lqr_avg_delta_score step_smc_avg_delta_score step_discrete_smc_avg_delta_score step_pid_avg_delta_score;   
                  step_lqr_max_delta_score step_smc_max_delta_score step_discrete_smc_max_delta_score step_pid_max_delta_score];

step_lqr_uncertainty_min_delta_score = norm(step_lqr_uncertainty_min_distances-ideal_distances, 'fro')/norm(step_lqr_uncertainty_min_distances+ideal_distances, 'fro');
step_lqr_uncertainty_avg_delta_score = norm(step_lqr_uncertainty_avg_distances-ideal_distances, 'fro')/norm(step_lqr_uncertainty_avg_distances+ideal_distances, 'fro');
step_lqr_uncertainty_max_delta_score = norm(step_lqr_uncertainty_max_distances-ideal_distances, 'fro')/norm(step_lqr_uncertainty_max_distances+ideal_distances, 'fro');
step_smc_uncertainty_min_delta_score = norm(step_smc_uncertainty_min_distances-ideal_distances, 'fro')/norm(step_smc_uncertainty_min_distances+ideal_distances, 'fro');
step_smc_uncertainty_avg_delta_score = norm(step_smc_uncertainty_avg_distances-ideal_distances, 'fro')/norm(step_smc_uncertainty_avg_distances+ideal_distances, 'fro');
step_smc_uncertainty_max_delta_score = norm(step_smc_uncertainty_max_distances-ideal_distances, 'fro')/norm(step_smc_uncertainty_max_distances+ideal_distances, 'fro');
step_discrete_smc_uncertainty_min_delta_score = norm(step_discrete_smc_uncertainty_min_distances-ideal_distances, 'fro')/norm(step_discrete_smc_uncertainty_min_distances+ideal_distances, 'fro');
step_discrete_smc_uncertainty_avg_delta_score = norm(step_discrete_smc_uncertainty_avg_distances-ideal_distances, 'fro')/norm(step_discrete_smc_uncertainty_avg_distances+ideal_distances, 'fro');
step_discrete_smc_uncertainty_max_delta_score = norm(step_discrete_smc_uncertainty_max_distances-ideal_distances, 'fro')/norm(step_discrete_smc_uncertainty_max_distances+ideal_distances, 'fro');
step_pid_uncertainty_min_delta_score = norm(step_pid_uncertainty_min_distances-ideal_distances, 'fro')/norm(step_pid_uncertainty_min_distances+ideal_distances, 'fro');
step_pid_uncertainty_avg_delta_score = norm(step_pid_uncertainty_avg_distances-ideal_distances, 'fro')/norm(step_pid_uncertainty_avg_distances+ideal_distances, 'fro');
step_pid_uncertainty_max_delta_score = norm(step_pid_uncertainty_max_distances-ideal_distances, 'fro')/norm(step_pid_uncertainty_max_distances+ideal_distances, 'fro');
step_uncertainty_score_mat = [step_lqr_uncertainty_min_delta_score step_smc_uncertainty_min_delta_score step_discrete_smc_uncertainty_min_delta_score step_pid_uncertainty_min_delta_score; 
                              step_lqr_uncertainty_avg_delta_score step_smc_uncertainty_avg_delta_score step_discrete_smc_uncertainty_avg_delta_score step_pid_uncertainty_avg_delta_score;   
                              step_lqr_uncertainty_max_delta_score step_smc_uncertainty_max_delta_score step_discrete_smc_uncertainty_max_delta_score step_pid_uncertainty_max_delta_score];

f8_lqr_min_delta_score = norm(f8_lqr_min_distances-ideal_distances, 'fro')/norm(f8_lqr_min_distances+ideal_distances, 'fro');
f8_lqr_avg_delta_score = norm(f8_lqr_avg_distances-ideal_distances, 'fro')/norm(f8_lqr_avg_distances+ideal_distances, 'fro');
f8_lqr_max_delta_score = norm(f8_lqr_max_distances-ideal_distances, 'fro')/norm(f8_lqr_max_distances+ideal_distances, 'fro');
f8_smc_min_delta_score = norm(f8_smc_min_distances-ideal_distances, 'fro')/norm(f8_smc_min_distances+ideal_distances, 'fro');
f8_smc_avg_delta_score = norm(f8_smc_avg_distances-ideal_distances, 'fro')/norm(f8_smc_avg_distances+ideal_distances, 'fro');
f8_smc_max_delta_score = norm(f8_smc_max_distances-ideal_distances, 'fro')/norm(f8_smc_max_distances+ideal_distances, 'fro');
f8_discrete_smc_min_delta_score = norm(f8_discrete_smc_min_distances-ideal_distances, 'fro')/norm(f8_discrete_smc_min_distances+ideal_distances, 'fro');
f8_discrete_smc_avg_delta_score = norm(f8_discrete_smc_avg_distances-ideal_distances, 'fro')/norm(f8_discrete_smc_avg_distances+ideal_distances, 'fro');
f8_discrete_smc_max_delta_score = norm(f8_discrete_smc_max_distances-ideal_distances, 'fro')/norm(f8_discrete_smc_max_distances+ideal_distances, 'fro');
f8_pid_min_delta_score = norm(f8_pid_min_distances-ideal_distances, 'fro')/norm(f8_pid_min_distances+ideal_distances, 'fro');
f8_pid_avg_delta_score = norm(f8_pid_avg_distances-ideal_distances, 'fro')/norm(f8_pid_avg_distances+ideal_distances, 'fro');
f8_pid_max_delta_score = norm(f8_pid_max_distances-ideal_distances, 'fro')/norm(f8_pid_max_distances+ideal_distances, 'fro');
f8_score_mat = [f8_lqr_min_delta_score f8_smc_min_delta_score f8_discrete_smc_min_delta_score f8_pid_min_delta_score; 
                f8_lqr_avg_delta_score f8_smc_avg_delta_score f8_discrete_smc_avg_delta_score f8_pid_avg_delta_score;   
                f8_lqr_max_delta_score f8_smc_max_delta_score f8_discrete_smc_max_delta_score f8_pid_max_delta_score];

spiral_lqr_min_delta_score = norm(spiral_lqr_min_distances-ideal_distances, 'fro')/norm(spiral_lqr_min_distances+ideal_distances, 'fro');
spiral_lqr_avg_delta_score = norm(spiral_lqr_avg_distances-ideal_distances, 'fro')/norm(spiral_lqr_avg_distances+ideal_distances, 'fro');
spiral_lqr_max_delta_score = norm(spiral_lqr_max_distances-ideal_distances, 'fro')/norm(spiral_lqr_max_distances+ideal_distances, 'fro');
spiral_smc_min_delta_score = norm(spiral_smc_min_distances-ideal_distances, 'fro')/norm(spiral_smc_min_distances+ideal_distances, 'fro');
spiral_smc_avg_delta_score = norm(spiral_smc_avg_distances-ideal_distances, 'fro')/norm(spiral_smc_avg_distances+ideal_distances, 'fro');
spiral_smc_max_delta_score = norm(spiral_smc_max_distances-ideal_distances, 'fro')/norm(spiral_smc_max_distances+ideal_distances, 'fro');
spiral_discrete_smc_min_delta_score = norm(spiral_discrete_smc_min_distances-ideal_distances, 'fro')/norm(spiral_discrete_smc_min_distances+ideal_distances, 'fro');
spiral_discrete_smc_avg_delta_score = norm(spiral_discrete_smc_avg_distances-ideal_distances, 'fro')/norm(spiral_discrete_smc_avg_distances+ideal_distances, 'fro');
spiral_discrete_smc_max_delta_score = norm(spiral_discrete_smc_max_distances-ideal_distances, 'fro')/norm(spiral_discrete_smc_max_distances+ideal_distances, 'fro');
spiral_pid_min_delta_score = norm(spiral_pid_min_distances-ideal_distances, 'fro')/norm(spiral_pid_min_distances+ideal_distances, 'fro');
spiral_pid_avg_delta_score = norm(spiral_pid_avg_distances-ideal_distances, 'fro')/norm(spiral_pid_avg_distances+ideal_distances, 'fro');
spiral_pid_max_delta_score = norm(spiral_pid_max_distances-ideal_distances, 'fro')/norm(spiral_pid_max_distances+ideal_distances, 'fro');
spiral_score_mat = [spiral_lqr_min_delta_score spiral_smc_min_delta_score spiral_discrete_smc_min_delta_score spiral_pid_min_delta_score; 
                    spiral_lqr_avg_delta_score spiral_smc_avg_delta_score spiral_discrete_smc_avg_delta_score spiral_pid_avg_delta_score;   
                    spiral_lqr_max_delta_score spiral_smc_max_delta_score spiral_discrete_smc_max_delta_score spiral_pid_max_delta_score];

rowlabels = {'min separation', 'average separation', 'max separation'};
collabels = {'LQR', 'cSMC', 'dSMC', 'PID'};
figure
subplot(2,1,1)
h1 = heatmap(collabels, rowlabels, 100*(1-step_score_mat), 'FontSize', 16, 'FontName', 'Times')
title("Step impulse nominal vs actual separation % similarity")
h1.CellLabelFormat = '%.2f %%'; 
subplot(2,1,2)
h2 = heatmap(collabels, rowlabels, 100*(1-step_uncertainty_score_mat), 'FontSize', 16, 'FontName', 'Times')
title("Step impulse with uncertainty nominal vs actual separation % similarity")
h2.CellLabelFormat = '%.2f %%'; 
% subplot(2,2,3)
% h3 = heatmap(collabels, rowlabels, 1-f8_score_mat, 'FontSize', 12, 'FontName', 'Times')
% title("Figure-8 nominal vs actual separation % similarity")
% subplot(2,2,4)
% h4 = heatmap(collabels, rowlabels, 1-spiral_score_mat, 'FontSize', 12, 'FontName', 'Times')
% title("Arithmetic spiral nominal vs actual separation % similarity")
export_figure("figs/35_score_table")

%% Kinetic Energy Plots
figure
subplot(5,4,1)
hold on
for i=1:10
    plot(step_time, ...
        step_lqr_ke(:,i))
    title("Step, LQR")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,2)
hold on
for i=1:10
    plot(step_time, ...
        step_smc_ke(:,i))
    title("Step, cSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,3)
hold on
for i=1:10
    plot(step_time, ...
        step_pid_ke(:,i))
    title("Step, PID")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
    ylim([0, max(max(step_smc_ke)*5)])
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,4)
hold on
for i=1:10
    plot(step_time, ...
        step_discrete_smc_ke(:,i))
    title("Step, dSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,5)
hold on
for i=1:10
    plot(step_time, ...
        step_lqr_uncertainty_ke(:,i))
    title("Step, LQR, uncertainty")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,6)
hold on
for i=1:10
    plot(step_time, ...
        step_smc_uncertainty_ke(:,i))
    title("Step, SMC, uncertainty")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,7)
hold on
for i=1:10
    plot(step_time, ...
        step_pid_uncertainty_ke(:,i))
    title("Step, PID, uncertainty")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
    ylim([0, max(max(step_lqr_uncertainty_ke)*5)])
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,8)
hold on
for i=1:10
    plot(step_time, ...
        step_discrete_smc_uncertainty_ke(:,i))
    title("Step, dSMC, uncertainty")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,9)
hold on
for i=1:10
    plot(f8_time, ...
        f8_lqr_ke(:,i))
    title("Figure 8, LQR")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,10)
hold on
for i=1:10
    plot(f8_time, ...
        f8_smc_ke(:,i))
    title("Figure 8, cSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,11)
hold on
for i=1:10
    plot(f8_time, ...
        f8_pid_ke(:,i))
    title("Figure 8, PID")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
    ylim([0, max(max(f8_lqr_ke)*5)])
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,12)
hold on
for i=1:10
    plot(f8_time, ...
        f8_discrete_smc_ke(:,i))
    title("Figure 8, dSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,13)
hold on
for i=1:10
    plot(spiral_time, ...
        spiral_lqr_ke(:,i))
    title("Arithmetic Spiral, LQR")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,14)
hold on
for i=1:10
    plot(spiral_time, ...
        spiral_smc_ke(:,i))
    title("Arithmetic Spiral, cSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,15)
hold on
for i=1:10
    plot(spiral_time, ...
        spiral_pid_ke(:,i))
    title("Arithmetic Spiral, PID")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
    ylim([0, max(max(spiral_lqr_ke)*5)])
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,16)
hold on
for i=1:10
    plot(spiral_time, ...
        spiral_discrete_smc_ke(:,i))
    title("Arithmetic Spiral, dSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader", "Follower 1", "Follower 2", "Follower 3", "Follower 4", ...
    "Follower 5", "Follower 6", "Follower 7", "Follower 8", "Follower 9")

subplot(5,4,17)
hold on
for i=1:1
    plot(ballistic_time, ...
        step_lqr_ballistic_ke(:,i))
    title("Ballistic, LQR")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader")

subplot(5,4,18)
hold on
for i=1:1
    plot(ballistic_time, ...
        step_smc_ballistic_ke(:,i))
    title("Ballistic, cSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader")

subplot(5,4,19)
hold on
for i=1:1
    plot(ballistic_time, ...
        step_pid_ballistic_ke(:,i))
    title("Ballistic, PID")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
    % Autoscale to PID's own trace: slaving to 5x LQR's max (as the other
    % panels do) rendered PID as an unreadable flat line whenever LQR
    % diverged (bugsweep 2026-07-10 finding 18).
end
legend("Leader")

subplot(5,4,20)
hold on
for i=1:1
    plot(ballistic_time, ...
        step_discrete_smc_ballistic_ke(:,i))
    title("Ballistic, dSMC")
    xlabel("Time (s)")
    ylabel("Kinetic energy (J)")
end
legend("Leader")
fontsize(gcf, 9, "points")
export_figure("figs/36_kinetic_energy", Width=2400, Height=1800)

%% Kinetic Energy Scores
avg_step_lqr_ke                         = mean(mean(step_lqr_ke, 2));
avg_step_smc_ke                         = mean(mean(step_smc_ke, 2));
avg_step_discrete_smc_ke                = mean(mean(step_discrete_smc_ke, 2));
avg_step_pid_ke                         = mean(mean(step_pid_ke, 2));
avg_step_lqr_ballistic_ke               = mean(mean(step_lqr_ballistic_ke, 2));
avg_step_smc_ballistic_ke               = mean(mean(step_smc_ballistic_ke, 2));
avg_step_discrete_smc_ballistic_ke      = mean(mean(step_discrete_smc_ballistic_ke, 2));
avg_step_pid_ballistic_ke               = mean(mean(step_pid_ballistic_ke, 2));
avg_step_lqr_uncertainty_ke             = mean(mean(step_lqr_uncertainty_ke, 2));
avg_step_smc_uncertainty_ke             = mean(mean(step_smc_uncertainty_ke, 2));
avg_step_discrete_smc_uncertainty_ke    = mean(mean(step_discrete_smc_uncertainty_ke, 2));
avg_step_pid_uncertainty_ke             = mean(mean(step_pid_uncertainty_ke, 2));
avg_f8_lqr_ke                           = mean(mean(f8_lqr_ke, 2));
avg_f8_smc_ke                           = mean(mean(f8_smc_ke, 2));
avg_f8_discrete_smc_ke                  = mean(mean(f8_discrete_smc_ke, 2));
avg_f8_pid_ke                           = mean(mean(f8_pid_ke, 2));
avg_spiral_lqr_ke                       = mean(mean(spiral_lqr_ke, 2));
avg_spiral_smc_ke                       = mean(mean(spiral_smc_ke, 2));
avg_spiral_discrete_smc_ke              = mean(mean(spiral_discrete_smc_ke, 2));
avg_spiral_pid_ke                       = mean(mean(spiral_pid_ke, 2));

unified_ke_step = [avg_step_lqr_ke avg_step_smc_ke avg_step_discrete_smc_ke avg_step_pid_ke];
unified_ke_step_ballistic = [avg_step_lqr_ballistic_ke avg_step_smc_ballistic_ke avg_step_discrete_smc_ballistic_ke avg_step_pid_ballistic_ke];
unified_ke_uncertainty = [avg_step_lqr_uncertainty_ke avg_step_smc_uncertainty_ke avg_step_discrete_smc_uncertainty_ke ...
    avg_step_pid_uncertainty_ke];
unified_ke_f8 = [avg_f8_lqr_ke avg_f8_smc_ke avg_f8_discrete_smc_ke avg_f8_pid_ke];
unified_ke_spiral = [avg_spiral_lqr_ke avg_spiral_smc_ke avg_spiral_discrete_smc_ke avg_spiral_pid_ke];

ke_scores_step = zeros(4,4);
ke_scores_ballistic = zeros(4,4);
ke_scores_uncertainty = zeros(4,4);
ke_scores_f8 = zeros(4,4);
ke_scores_spiral = zeros(4,4);
for i=1:4
    for j=1:4
        ke_scores_step(i,j) = 100*(unified_ke_step(i) - unified_ke_step(j)) / (unified_ke_step(i) + unified_ke_step(j));
        ke_scores_ballistic(i,j) = 100*(unified_ke_step_ballistic(i) - unified_ke_step_ballistic(j)) / (unified_ke_step_ballistic(i) + unified_ke_step_ballistic(j));
        ke_scores_uncertainty(i,j) = 100*(unified_ke_uncertainty(i) - unified_ke_uncertainty(j)) / (unified_ke_uncertainty(i) + unified_ke_uncertainty(j));
        ke_scores_f8(i,j) = 100*(unified_ke_f8(i) - unified_ke_f8(j)) / (unified_ke_f8(i) + unified_ke_f8(j));
        ke_scores_spiral(i,j) = 100*(unified_ke_spiral(i) - unified_ke_spiral(j)) / (unified_ke_spiral(i) + unified_ke_spiral(j));
    end
end

figure
conlabels = {'LQR', 'cSMC', 'dSMC', 'PID'};
subplot(3,1,1)
h1 = heatmap(conlabels, conlabels, ke_scores_step, "Colormap", jet, 'FontSize', 16, 'FontName', 'Times')
title("Relative % per-timestep average KE, step impulse")
h1.CellLabelFormat = '%.2f %%'; 
clim([-50 50])

subplot(3,1,2)
h2 = heatmap(conlabels, conlabels, ke_scores_uncertainty, "Colormap", jet, 'FontSize', 16, 'FontName', 'Times')
title("Relative % per-timestep average KE, step w/ uncertainty")
h2.CellLabelFormat = '%.2f %%'; 
clim([-50 50])

% subplot(3,2,3)
% h3 = heatmap(conlabels, conlabels, ke_scores_f8, "Colormap", jet, 'FontName', 'Times')
% title("Relative % integrated kinetic energy over time, step impulse figure-8")
% h3.CellLabelFormat = '%.2f %%'; 
% clim([-50 50])
% 
% subplot(3,2,4)
% h4 = heatmap(conlabels, conlabels, ke_scores_spiral, "Colormap", jet, 'FontName', 'Times')
% title("Relative % integrated kinetic energy over time, step impulse arithmetic spiral")
% h4.CellLabelFormat = '%.2f %%'; 
% clim([-50 50])

subplot(3,1,3)
h3 = heatmap(conlabels, conlabels, ke_scores_ballistic, "Colormap", jet, 'FontSize', 16, 'FontName', 'Times')
title("Relative % per-timestep average KE, ballistic step")
h3.CellLabelFormat = '%.2f %%'; 
clim([-50 50])
export_figure("figs/37_kinetic_energy_scores")

%% Difference in uncertain trajectories
norm(interp_step_lqr_uncertainty_pos - interp_step_lqr_pos, 'fro')
norm(interp_step_smc_uncertainty_pos - interp_step_smc_pos, 'fro')
norm(interp_step_discrete_smc_uncertainty_pos - interp_step_discrete_smc_pos, 'fro')
norm(interp_step_pid_uncertainty_pos - interp_step_pid_pos, 'fro')

%% Settling time
% Step: settled at first t where worst-drone deviation from xf <= 5% of its step |xf - xi|
xi_lqr = interp_step_lqr_pos(1,:,:);
xi_smc = interp_step_smc_pos(1,:,:);
xi_discrete_smc = interp_step_discrete_smc_pos(1,:,:);
xi_pid = interp_step_pid_pos(1,:,:);
xf_lqr = interp_step_lqr_pos(end,:,:);
xf_smc = interp_step_smc_pos(end,:,:);
xf_discrete_smc = interp_step_discrete_smc_pos(end,:,:);
xf_pid = interp_step_pid_pos(end,:,:);

ts_xf_lqr = NaN;
ts_xf_smc = NaN;
ts_xf_discrete_smc = NaN;
ts_xf_pid = NaN;

for i=1:size(interp_step_smc_pos, 1)
    delta_lqr = vecnorm(interp_step_lqr_pos(i,:,:) - xf_lqr,2,3);
    delta_smc = vecnorm(interp_step_smc_pos(i,:,:) - xf_smc,2,3);
    delta_discrete_smc = vecnorm(interp_step_discrete_smc_pos(i,:,:) - xf_discrete_smc,2,3);
    delta_pid = vecnorm(interp_step_pid_pos(i,:,:) - xf_pid,2,3);

    [max_delta_lqr,max_delta_lqr_idx] = max(delta_lqr);
    [max_delta_smc,max_delta_smc_idx] = max(delta_smc);
    [max_delta_discrete_smc,max_delta_discrete_smc_idx] = max(delta_discrete_smc);
    [max_delta_pid,max_delta_pid_idx] = max(delta_pid);

    delta_lqr = max_delta_lqr / norm(squeeze(xf_lqr(1,max_delta_lqr_idx,:) - xi_lqr(1,max_delta_lqr_idx,:)));
    delta_smc = max_delta_smc / norm(squeeze(xf_smc(1,max_delta_smc_idx,:) - xi_smc(1,max_delta_smc_idx,:)));
    delta_discrete_smc = max_delta_discrete_smc / norm(squeeze(xf_discrete_smc(1,max_delta_discrete_smc_idx,:) - xi_discrete_smc(1,max_delta_discrete_smc_idx,:)));
    delta_pid = max_delta_pid / norm(squeeze(xf_pid(1,max_delta_pid_idx,:) - xi_pid(1,max_delta_pid_idx,:)));

    if (delta_lqr <= 0.05 && isnan(ts_xf_lqr))
        ts_xf_lqr = step_time(i);
    end
    if (delta_smc <= 0.05 && isnan(ts_xf_smc))
        ts_xf_smc = step_time(i);
    end
    if (delta_discrete_smc <= 0.05 && isnan(ts_xf_discrete_smc))
        ts_xf_discrete_smc = step_time(i);
    end
    if (delta_pid <= 0.05 && isnan(ts_xf_pid))
        ts_xf_pid = step_time(i);
    end
end
ts = [ts_xf_lqr ts_xf_smc ts_xf_discrete_smc ts_xf_pid];
ts_score = zeros(4,4);
for i=1:4
    for j=1:4
        ts_score(i,j) = 100*(ts(i) - ts(j))/(ts(i)+ts(j));
    end
end

% Ballistic Analysis
ts_xf_lqr_ballistic = NaN;
ts_xf_smc_ballistic = NaN;
ts_xf_discrete_smc_ballistic = NaN;
ts_xf_pid_ballistic = NaN;

xi_lqr_ballistic = interp_step_lqr_ballistic_pos(1,:,:);
xi_smc_ballistic = interp_step_smc_ballistic_pos(1,:,:);
xi_discrete_smc_ballistic = interp_step_discrete_smc_ballistic_pos(1,:,:);
xi_pid_ballistic = interp_step_pid_ballistic_pos(1,:,:);
% Settle toward the COMMANDED ballistic target xf_ballistic (the apogee
% deploy XY at ground, L253), not each run's own final position. (Bugsweep
% 2026-07-10 finding 16: referencing the run's own endpoint guarantees any
% diverged run a finite, competitive-looking settling time -- the diverged
% LQR apogee run scored within 3% of the converged controllers. Against the
% fixed command, a run that never reaches the target keeps ts = NaN, which
% the score heatmap renders blank -- the correct "did not settle".)
xf_cmd_ballistic = reshape(xf_ballistic_apogee, [1 1 3]);
xf_lqr_ballistic = xf_cmd_ballistic;
xf_smc_ballistic = xf_cmd_ballistic;
xf_discrete_smc_ballistic = xf_cmd_ballistic;
xf_pid_ballistic = xf_cmd_ballistic;

for i=1:size(interp_step_smc_ballistic_pos, 1)
    delta_lqr = vecnorm(interp_step_lqr_ballistic_pos(i,:,:) - xf_lqr_ballistic,2,3);
    delta_smc = vecnorm(interp_step_smc_ballistic_pos(i,:,:) - xf_smc_ballistic,2,3);
    delta_discrete_smc = vecnorm(interp_step_discrete_smc_ballistic_pos(i,:,:) - xf_discrete_smc_ballistic,2,3);
    delta_pid = vecnorm(interp_step_pid_ballistic_pos(i,:,:) - xf_pid_ballistic,2,3);

    [max_delta_lqr,max_delta_lqr_idx] = max(delta_lqr);
    [max_delta_smc,max_delta_smc_idx] = max(delta_smc);
    [max_delta_discrete_smc,max_delta_discrete_smc_idx] = max(delta_discrete_smc);
    [max_delta_pid,max_delta_pid_idx] = max(delta_pid);

    delta_lqr = max_delta_lqr / norm(squeeze(xf_lqr_ballistic(1,max_delta_lqr_idx,:) - xi_lqr_ballistic(1,max_delta_lqr_idx,:)));
    delta_smc = max_delta_smc / norm(squeeze(xf_smc_ballistic(1,max_delta_smc_idx,:) - xi_smc_ballistic(1,max_delta_smc_idx,:)));
    delta_discrete_smc = max_delta_discrete_smc / norm(squeeze(xf_discrete_smc_ballistic(1,max_delta_discrete_smc_idx,:) - xi_discrete_smc_ballistic(1,max_delta_discrete_smc_idx,:)));
    delta_pid = max_delta_pid / norm(squeeze(xf_pid_ballistic(1,max_delta_pid_idx,:) - xi_pid_ballistic(1,max_delta_pid_idx,:)));

    if (delta_lqr <= 0.05 && isnan(ts_xf_lqr_ballistic))
        ts_xf_lqr_ballistic = ballistic_time(i);
    end
    if (delta_smc <= 0.05 && isnan(ts_xf_smc_ballistic))
        ts_xf_smc_ballistic = ballistic_time(i);
    end
    if (delta_discrete_smc <= 0.05 && isnan(ts_xf_discrete_smc_ballistic))
        ts_xf_discrete_smc_ballistic = ballistic_time(i);
    end
    if (delta_pid <= 0.05 && isnan(ts_xf_pid_ballistic))
        ts_xf_pid_ballistic = ballistic_time(i);
    end
end
ts = [ts_xf_lqr_ballistic ts_xf_smc_ballistic ts_xf_discrete_smc_ballistic ts_xf_pid_ballistic];
ts_score_ballistic = zeros(4,4);
for i=1:4
    for j=1:4
        ts_score_ballistic(i,j) = 100*(ts(i) - ts(j))/(ts(i)+ts(j));
    end
end

% Step uncertainty analysis
xi_lqr_uncertainty = interp_step_lqr_uncertainty_pos(1,:,:);
xi_smc_uncertainty = interp_step_smc_uncertainty_pos(1,:,:);
xi_discrete_smc_uncertainty = interp_step_discrete_smc_uncertainty_pos(1,:,:);
xi_pid_uncertainty = interp_step_pid_uncertainty_pos(1,:,:);
xf_lqr_uncertainty = interp_step_lqr_uncertainty_pos(end,:,:);
xf_smc_uncertainty = interp_step_smc_uncertainty_pos(end,:,:);
xf_discrete_smc_uncertainty = interp_step_discrete_smc_uncertainty_pos(end,:,:);
xf_pid_uncertainty = interp_step_pid_uncertainty_pos(end,:,:);

ts_xf_lqr_uncertainty = NaN;
ts_xf_smc_uncertainty = NaN;
ts_xf_discrete_smc_uncertainty = NaN;
ts_xf_pid_uncertainty = NaN;

for i=1:size(interp_step_smc_uncertainty_pos, 1)
    delta_lqr_uncertainty = vecnorm(interp_step_lqr_uncertainty_pos(i,:,:) - xf_lqr_uncertainty,2,3);
    delta_smc_uncertainty = vecnorm(interp_step_smc_uncertainty_pos(i,:,:) - xf_smc_uncertainty,2,3);
    delta_discrete_smc_uncertainty = vecnorm(interp_step_discrete_smc_uncertainty_pos(i,:,:) - xf_discrete_smc_uncertainty,2,3);
    delta_pid_uncertainty = vecnorm(interp_step_pid_uncertainty_pos(i,:,:) - xf_pid_uncertainty,2,3);

    [max_delta_lqr_uncertainty,max_delta_lqr_idx_uncertainty] = max(delta_lqr_uncertainty);
    [max_delta_smc_uncertainty,max_delta_smc_idx_uncertainty] = max(delta_smc_uncertainty);
    [max_delta_discrete_smc_uncertainty,max_delta_discrete_smc_idx_uncertainty] = max(delta_discrete_smc_uncertainty);
    [max_delta_pid_uncertainty,max_delta_pid_idx_uncertainty] = max(delta_pid_uncertainty);

    delta_lqr_uncertainty = max_delta_lqr_uncertainty / norm(squeeze(xf_lqr_uncertainty(1,max_delta_lqr_idx_uncertainty,:) - xi_lqr_uncertainty(1,max_delta_lqr_idx_uncertainty,:)));
    delta_smc_uncertainty = max_delta_smc_uncertainty / norm(squeeze(xf_smc_uncertainty(1,max_delta_smc_idx_uncertainty,:) - xi_smc_uncertainty(1,max_delta_smc_idx_uncertainty,:)));
    delta_discrete_smc_uncertainty = max_delta_discrete_smc_uncertainty / norm(squeeze(xf_discrete_smc_uncertainty(1,max_delta_discrete_smc_idx_uncertainty,:) - xi_discrete_smc_uncertainty(1,max_delta_discrete_smc_idx_uncertainty,:)));
    delta_pid_uncertainty = max_delta_pid_uncertainty / norm(squeeze(xf_pid_uncertainty(1,max_delta_pid_idx_uncertainty,:) - xi_pid_uncertainty(1,max_delta_pid_idx_uncertainty,:)));

    if (delta_lqr_uncertainty <= 0.05 && isnan(ts_xf_lqr_uncertainty))
        ts_xf_lqr_uncertainty = step_time(i);
    end
    if (delta_smc_uncertainty <= 0.05 && isnan(ts_xf_smc_uncertainty))
        ts_xf_smc_uncertainty = step_time(i);
    end
    if (delta_discrete_smc_uncertainty <= 0.05 && isnan(ts_xf_discrete_smc_uncertainty))
        ts_xf_discrete_smc_uncertainty = step_time(i);
    end
    if (delta_pid_uncertainty <= 0.05 && isnan(ts_xf_pid_uncertainty))
        ts_xf_pid_uncertainty = step_time(i);
    end
end
ts_uncertainty = [ts_xf_lqr_uncertainty ts_xf_smc_uncertainty ts_xf_discrete_smc_uncertainty ts_xf_pid_uncertainty];
ts_score_uncertainty = zeros(4,4);
for i=1:4
    for j=1:4
        ts_score_uncertainty(i,j) = 100*(ts_uncertainty(i) - ts_uncertainty(j))/(ts_uncertainty(i)+ts_uncertainty(j));
    end
end

figure
conlabels = {'LQR', 'cSMC', 'dSMC', 'PID'};
subplot(3,1,1)
h1 = heatmap(conlabels, conlabels, ts_score, "Colormap", jet, 'FontSize', 16, 'FontName', 'Times')
title("Relative % 95% settling time, step impulse")
h1.CellLabelFormat = '%.1f %%'; 
clim([-50 50])

subplot(3,1,2)
h2 = heatmap(conlabels, conlabels, ts_score_uncertainty, "Colormap", jet, 'FontSize', 16, 'FontName', 'Times')
title("Relative % 95% settling time, step impulse with uncertainty")
h2.CellLabelFormat = '%.1f %%'; 
clim([-50 50])

subplot(3,1,3)
h3 = heatmap(conlabels, conlabels, ts_score_ballistic, "Colormap", jet, 'FontSize', 16, 'FontName', 'Times')
title("Relative % 95% settling time, ballistic")
h3.CellLabelFormat = '%.1f %%'; 
clim([-50 50])
export_figure("figs/38_settling_time_scores")

%% Ballistic envelope (fig 39)
figure
ax(1) = subplot(2,3,1)
axis equal
title("LQR")
grid on
hold on
plot3(ballistic_solution.trajectory(1:end,10),ballistic_solution.trajectory(1:end,11),ballistic_solution.trajectory(1:end,12), '-b', 'LineWidth',2)
for i=1:size(test_points,2)
    idx = test_points(i);

    deploy_point = ballistic_solution.trajectory(idx, :).';
    x0 = deploy_point(10:12);
    
    if ismember(idx, ballistic_envelope_lqr)
        plot3(x0(1), x0(2), x0(3), 'og', 'MarkerFaceColor', 'green')
    else
        plot3(x0(1), x0(2), x0(3), 'or', 'MarkerFaceColor', 'red')
    end
end
view(3)
xticks(0:400:800)
yticks(-400:400:0)
zticks(0:400:400)

ax(2) = subplot(2,3,2)
axis equal
title("PID")
grid on
hold on
plot3(ballistic_solution.trajectory(1:end,10),ballistic_solution.trajectory(1:end,11),ballistic_solution.trajectory(1:end,12), '-b', 'LineWidth',2)
for i=1:size(test_points,2)
    idx = test_points(i);

    deploy_point = ballistic_solution.trajectory(idx, :).';
    x0 = deploy_point(10:12);
    
    if ismember(idx, ballistic_envelope_pid)
        plot3(x0(1), x0(2), x0(3), 'og', 'MarkerFaceColor', 'green')
    else
        plot3(x0(1), x0(2), x0(3), 'or', 'MarkerFaceColor', 'red')
    end
end
view(3)
xticks(0:400:800)
yticks(-400:400:0)
zticks(0:400:400)

ax(3) = subplot(2,3,3)
axis equal
title("cSMC")
grid on
hold on
plot3(ballistic_solution.trajectory(1:end,10),ballistic_solution.trajectory(1:end,11),ballistic_solution.trajectory(1:end,12), '-b', 'LineWidth',2)
for i=1:size(test_points,2)
    idx = test_points(i);

    deploy_point = ballistic_solution.trajectory(idx, :).';
    x0 = deploy_point(10:12);
    
    if ismember(idx, ballistic_envelope_smc)
        plot3(x0(1), x0(2), x0(3), 'og', 'MarkerFaceColor', 'green')
    else
        plot3(x0(1), x0(2), x0(3), 'or', 'MarkerFaceColor', 'red')
    end
end
view(3)
xticks(0:400:800)
yticks(-400:400:0)
zticks(0:400:400)

ax(4) = subplot(2,3,4)
axis equal
title("dSMC (no control constraints)")
grid on
hold on
plot3(ballistic_solution.trajectory(1:end,10),ballistic_solution.trajectory(1:end,11),ballistic_solution.trajectory(1:end,12), '-b', 'LineWidth',2)
for i=1:size(test_points,2)
    idx = test_points(i);

    deploy_point = ballistic_solution.trajectory(idx, :).';
    x0 = deploy_point(10:12);

    if ismember(idx, ballistic_envelope_dsmc_nosat)
        plot3(x0(1), x0(2), x0(3), 'og', 'MarkerFaceColor', 'green')
    else
        plot3(x0(1), x0(2), x0(3), 'or', 'MarkerFaceColor', 'red')
    end
end
view(3)
xticks(0:400:800)
yticks(-400:400:0)
zticks(0:400:400)

ax(5) = subplot(2,3,5)
axis equal
title("dSMC (saturated control effort)")
grid on
hold on
plot3(ballistic_solution.trajectory(1:end,10),ballistic_solution.trajectory(1:end,11),ballistic_solution.trajectory(1:end,12), '-b', 'LineWidth',2)
for i=1:size(test_points,2)
    idx = test_points(i);

    deploy_point = ballistic_solution.trajectory(idx, :).';
    x0 = deploy_point(10:12);

    if ismember(idx, ballistic_envelope_dsmc_sat)
        plot3(x0(1), x0(2), x0(3), 'og', 'MarkerFaceColor', 'green')
    else
        plot3(x0(1), x0(2), x0(3), 'or', 'MarkerFaceColor', 'red')
    end
end
view(3)
xticks(0:400:800)
yticks(-400:400:0)
zticks(0:400:400)

% Spread bottom row across full figure width: center ax(4) between top cols 1-2
% and ax(5) between top cols 2-3 so their long titles no longer overlap.
p1 = ax(1).Position; p2 = ax(2).Position; p3 = ax(3).Position;
ax(4).Position(1) = (p1(1) + p2(1))/2;
ax(5).Position(1) = (p2(1) + p3(1))/2;

% Drop the top row so the sgtitle has breathing room above subplot titles.
for k = 1:3
    ax(k).Position(2) = ax(k).Position(2) - 0.04;
end

linkaxes(ax, "xyz")

sgtitle("Ballistic trajectory stabilization envelopes", 'FontSize', 20, 'FontWeight', 'bold')

export_figure("figs/39_ballistic_envelope", Width=3000, Height=1200)

%% Cache workspace (load to skip re-simulating)
save("logs/analysis_log.mat")