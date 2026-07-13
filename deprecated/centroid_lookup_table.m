% LHS lookup table over the 6-D launch space (Vo, el, az, w_z0, w_y0, p):
% one sweep_landing_centroid(_, false) per sample; N=20, rng(0), ~5 h total.
% Writes logs/centroid_lookup_log.mat (lookup, ranges, X, N, field_names);
% render figs/LUT_01 afterwards via replot_LUT_01.m.
%
% SUPERSEDED by j3_lut_regen.m (executed 2026-07-10; retuned 2026-07-11):
% this driver keeps the pre-J3 config (p [-2,2] and el [35,55] exclude the
% operating point p=-8.379, el=64; N=20; no radial_profile/is_reference
% fields, which j3_surrogate_refit.m asserts on; no checkpointing) and
% running it overwrites the canonical N=41 logs/centroid_lookup_log.mat
% with no archive step. Kept for reference only -- do NOT run.
close all; clear all; clc
Simulink.sdi.clear()
set_param(0, 'CacheFolder', '');

load_system("discrete_smc_swarm_single")
addpath("Ballistics_Simulation-master/")

ranges = struct( ...
    'Vo',   [ 80, 120], ...
    'el',   [ 35,  55], ...
    'az',   [  0,  30], ...
    'w_z0', [ -1,   2], ...
    'w_y0', [ -1,   1], ...
    'p',    [ -2,   2]);
field_names = fieldnames(ranges);
N = 20;
rng(0)
X = lhsdesign(N, length(field_names));

lookup = repmat(struct( ...
    'params',             [], ...
    'p_centroid',         [], ...
    'reachability_pct',   NaN, ...
    'half_radius',        NaN, ...
    'ballistic_solution', []), N, 1);

for n = 1:N
    params = struct('alpha_0',  2, 'beta_0', -0.5, ...
                    'x_0',      0, 'y_0',     0, 'z_0',   0, ...
                    't_max',  300);
    for k = 1:length(field_names)
        f = field_names{k};
        params.(f) = ranges.(f)(1) + X(n,k) * diff(ranges.(f));
    end
    out = sweep_landing_centroid(params, false);
    lookup(n).params             = params;
    lookup(n).p_centroid         = out.p_centroid;
    lookup(n).reachability_pct   = out.reachability_pct;
    lookup(n).half_radius        = out.radial_profile.half_radius;
    lookup(n).ballistic_solution = out.ballistic_solution;
    fprintf('LHS %d/%d: cx=%.1f cy=%.1f reach=%.2f half_r=%.1f\n', ...
        n, N, out.p_centroid(1), out.p_centroid(2), ...
        out.reachability_pct, out.radial_profile.half_radius);
end
save("logs/centroid_lookup_log.mat", "lookup", "ranges", "X", "N", "field_names")

%% Figure: render via replot_LUT_01.m (single canonical LUT_01 renderer;
%  matches the j3_lut_regen.m convention)
