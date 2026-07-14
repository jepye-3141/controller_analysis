function launch = operational_launch()
% OPERATIONAL_LAUNCH  Single source of the operational mortar launch.
% Retune here ONLY -- consumed by trajectory_optimization.m, analysis.m
% (apogee test + envelope), j3_lut_regen.m (reference row), and
% Ballistics_Simulation-master/Mortar_Sim.m. Callers add non-launch fields
% (saturation_on, label) themselves.
launch.Vo      = 94;   % muzzle velocity (m/s); operational launch, gentle apogee deploy
launch.el      = 64;   % departure elevation (deg); recovers corrected-physics reachability optimum (~0.33)
launch.az      = 15;   % departure azimuth off the vertical firing plane (deg,
                       % pos to right; McCoy sec 9.3) -- NOT a compass heading:
                       % at el=64 the true ground track is ~31 deg (audit C10)
launch.w_z0    = 1;    % initial pitch rate in rad/s (pos nose up)
launch.w_y0    = 0.5;  % initial transverse yaw rate in rad/s (pos for left yaw)
launch.alpha_0 = 2;    % initial pitch AoA: pointing elevation minus velocity
                       % elevation (deg); an OFFSET added to el for r0, not an
                       % absolute exit elevation (audit C10)
launch.beta_0  = -0.5; % initial sideslip: pointing azimuth minus velocity
                       % azimuth (deg); an OFFSET added to az for r0
% munition CG initial position wrt inertial frame
launch.x_0     = 0;    % x-axis (m) - range direction
launch.y_0     = 0;    % y-axis (m) - altitude
launch.z_0     = 0;    % z-axis (m) - cross-range direction
launch.t_max   = 300;  % sim end time (s)
launch.p       = -8.379; % axial launch spin (rad/s); sets the deploy tumble (apogee h.r ~ -0.81 at the current launch)
end
