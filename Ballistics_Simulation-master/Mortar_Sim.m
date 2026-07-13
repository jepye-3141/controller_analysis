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

% Single source of the operational launch (project root must be on the
% path, as for every driver here); retune in operational_launch.m only.
launch = operational_launch();

ballistic_solution = eom2(launch, env, true);

save("logs/ballistic_log.mat")