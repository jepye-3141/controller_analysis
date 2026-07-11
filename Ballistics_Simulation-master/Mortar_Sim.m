%% Documentation
% Program written by: John Pye
% Date 9 February 2026

% N.b. assume range is oriented northwards, crossrange is east
% Munition parameters, equations of motion, and basic outline from:
% Modern Exterior Ballistics, 2nd edition, McCoy, Chapter 9

%% Program setup
clc

%% Initial conditions
env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');

launch.Vo      = 100;  % initial vel at muzzle exit in m/s
launch.el      = 45;   % vertical angle of departure in deg (pos up)
launch.az      = 15;   % horizontal angle of departure in deg (pos to right)
launch.w_z0    = 1;    % initial pitch rate in rad/s (pos nose up)
launch.w_y0    = 0.5;  % initial transverse yaw rate in rad/s (pos for left yaw)
launch.alpha_0 = 2;    % initial AoA: pointing elevation minus velocity elevation (deg)
launch.beta_0  = -0.5; % initial sideslip: pointing azimuth minus velocity azimuth (deg)
% initial position of munition center of gravity (CG) wrt inertial frame
launch.x_0     = 0;    % x-axis (m) - range direction
launch.y_0     = 0;    % y-axis (m) - altitude
launch.z_0     = 0;    % z-axis (m) - cross-range direction
launch.t_max   = 300;  % sim end time (s)
launch.p       = 0;    % initial spin rate in rad/s

ballistic_solution = eom2(launch, env, true);

save("logs/ballistic_log.mat")