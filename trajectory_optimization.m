% Saturation ON/OFF driver: runs sweep_landing_centroid twice on one launch
% (labels sat_on / sat_off), then plot_saturation_comparison. sat_off is the
% naive per-rotor-clipped baseline (STEP 12b clip, 2026-07-03), not unconstrained.
close all; clear all; clc
Simulink.sdi.clear()
set_param(0, 'CacheFolder', '');

load_system("discrete_smc_swarm_single")
addpath("Ballistics_Simulation-master/")

% Single source of the operational launch; retune in operational_launch.m only.
base_params = operational_launch();

p_on                = base_params;
p_on.saturation_on  = true;
p_on.label          = "sat_on";

p_off               = base_params;
p_off.saturation_on = false;
p_off.label         = "sat_off";

out_on  = sweep_landing_centroid(p_on,  true);
out_off = sweep_landing_centroid(p_off, true);

plot_saturation_comparison(out_on, out_off);
