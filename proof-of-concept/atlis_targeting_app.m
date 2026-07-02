function fig = atlis_targeting_app(varargin)
%ATLIS_TARGETING_APP  Interactive ballistic targeting demo.
%   fig = atlis_targeting_app() opens the GUI with default settings.
%
%   Optional name-value args:
%     'cache_path'  default 'centroid_lookup_log_v3_iter6.mat'
%     'mu0'         default [524 0 0]   (target mean: range, crossrange, alt)
%     'Sigma0'      default diag([50^2 30^2 0])
%     'poc'         default 'E'
%
%   Single-target, single-POC interactive design study tool.
%   Layout: 2D top-down map (left) | 3D flight viewport (right) |
%           right rail with POC selector, readouts, buttons.

p = inputParser;
addParameter(p, 'cache_path', 'centroid_lookup_log_v3_iter6.mat');
addParameter(p, 'mu0',  [524 0 0]);
addParameter(p, 'Sigma0', diag([50^2 30^2 0]));
addParameter(p, 'poc', 'E');
parse(p, varargin{:});
opts = p.Results;

% Ensure the ballistics folder is on path
this_dir = fileparts(mfilename('fullpath'));
ballistics_dir = fullfile(this_dir, 'Ballistics_Simulation-master');
if exist(ballistics_dir, 'dir') && ~contains(path, ballistics_dir)
    addpath(ballistics_dir);
end

% Build initial state
state = init_state(opts);

% Build the GUI (returns handles in state.h)
[fig, state] = build_layout(state);

% Initial compute + render
state = trigger_compute(state, false);
fig.UserData = state;
end

% ============================================================
% State init
% ============================================================
function state = init_state(opts)
state.opts = opts;

% Load cache
S = load(opts.cache_path);
state.lookup = S.lookup;
state.ranges = S.ranges;

% Filter entries with valid p_centroid + nonzero reach (for plotting)
valid = arrayfun(@(L) L.reachability_pct > 0 && all(isfinite(L.p_centroid)), ...
                 state.lookup(:));
state.valid_idx = find(valid);

% Default target
state.mu_target    = opts.mu0(:).';     % [range cross alt]
state.Sigma_target = opts.Sigma0;
state.poc          = opts.poc;

% Pre-sample evaluation cloud for the map
rng(0);
state.n_eval  = 200;
state.n_train = 50;
state = resample_target(state);

% Solution placeholders
state.theta_star = [];
state.J_star     = NaN;
state.J_eval     = NaN;
state.predicted_centroid = [NaN NaN NaN];
state.trajectory = [];          % deterministic flight path
state.dispersion_tube = [];     % MC envelope, struct with .alts, .ellipses
state.cep_radius = NaN;

% Animation state
state.anim.frame = 1;
state.anim.timer = [];
state.anim.playing = false;
state.anim.speed = 1.0;
end

function state = resample_target(state)
mu = state.mu_target;
Sig = state.Sigma_target;
% Drop the zero-altitude row/col for sampling (it's a degenerate dim)
Sig2d = Sig(1:2, 1:2);
mu2d  = mu(1:2);
[U, D] = eig(Sig2d);
L = U * sqrt(max(D, 0));
samp_train = (mu2d + (L * randn(2, state.n_train))').';
samp_train = [samp_train; zeros(1, state.n_train)]';     % add alt=0 column
samp_eval  = (mu2d + (L * randn(2, state.n_eval))').';
samp_eval  = [samp_eval; zeros(1, state.n_eval)]';
state.samples_train = samp_train;     % n_train x 3
state.samples_eval  = samp_eval;      % n_eval x 3
end

% ============================================================
% Layout
% ============================================================
function [fig, state] = build_layout(state)
fig = uifigure('Name', 'ATLIS Targeting Demo', 'Position', [80 80 1500 900], ...
               'Color', [0.97 0.97 0.98]);

% Header strip
h.header = uipanel(fig, 'Position', [0 855 1500 45], ...
                   'BackgroundColor', [0.13 0.18 0.27], 'BorderType', 'none');
uilabel(h.header, 'Position', [16 6 600 32], ...
        'Text', 'ATLIS — Stochastic Ballistic Targeting Demo', ...
        'FontSize', 18, 'FontWeight', 'bold', ...
        'FontColor', [0.95 0.96 0.99]);
h.header_status = uilabel(h.header, 'Position', [620 6 800 32], ...
        'Text', 'Ready', 'FontSize', 13, ...
        'FontColor', [0.78 0.85 0.95]);

% Main grid: [2D map | 3D scene | right rail]
main_y = 60;
main_h = 790;
map_x = 16;     map_w = 540;
scn_x = map_x + map_w + 12;  scn_w = 600;
rail_x = scn_x + scn_w + 12; rail_w = 1500 - rail_x - 16;

% 2D map
h.map_panel = uipanel(fig, 'Position', [map_x main_y map_w main_h], ...
                     'Title', 'Top-down map (range × crossrange, m)', ...
                     'FontWeight', 'bold');
h.ax_map = uiaxes(h.map_panel, 'Position', [8 8 map_w-16 main_h-32]);
h.ax_map.Box = 'on';
h.ax_map.XGrid = 'on'; h.ax_map.YGrid = 'on';
h.ax_map.XLabel.String = 'Range (m, north)';
h.ax_map.YLabel.String = 'Crossrange (m, east)';
h.ax_map.DataAspectRatio = [1 1 1];

% 3D viewport
h.scn_panel = uipanel(fig, 'Position', [scn_x main_y scn_w main_h], ...
                     'Title', 'Flight viewport (3D)', 'FontWeight', 'bold');
h.ax_scn = uiaxes(h.scn_panel, 'Position', [8 8 scn_w-16 main_h-32]);
h.ax_scn.Box = 'on';
h.ax_scn.XLabel.String = 'Range (m)';
h.ax_scn.YLabel.String = 'Crossrange (m)';
h.ax_scn.ZLabel.String = 'Altitude (m)';
h.ax_scn.DataAspectRatioMode = 'manual';
h.ax_scn.DataAspectRatio = [1 1 1];
view(h.ax_scn, 35, 25);

% Right rail
h.rail = uipanel(fig, 'Position', [rail_x main_y rail_w main_h], ...
                 'Title', 'Targeting solution', 'FontWeight', 'bold');

ry = main_h - 60;     % cursor (top down)
gap = 6; row_h = 24; lbl_w = 90; val_w = rail_w - 30 - lbl_w;

% POC selector
uilabel(h.rail, 'Position', [12 ry lbl_w row_h], 'Text', 'POC:');
h.dd_poc = uidropdown(h.rail, 'Position', [12+lbl_w ry val_w row_h], ...
    'Items', {'A — BO/GP', 'B — CEM', 'C — Sobol', 'D — CVaR', ...
              'E — GP prescreen + CEM', 'F — NN surrogate'}, ...
    'ItemsData', {'A','B','C','D','E','F'}, ...
    'Value', state.poc);
ry = ry - row_h - gap;

% Live-refine budget
uilabel(h.rail, 'Position', [12 ry lbl_w row_h], 'Text', 'Live budget:');
h.sp_budget = uispinner(h.rail, 'Position', [12+lbl_w ry val_w row_h], ...
    'Limits', [0 20], 'Value', 0, 'Step', 1);
ry = ry - row_h - gap;

% Buttons row
btn_w = (rail_w - 30) / 2 - 4;
h.btn_compute = uibutton(h.rail, 'Position', [12 ry btn_w row_h+4], ...
    'Text', 'Compute (cache)', 'BackgroundColor', [0.85 0.92 1.0]);
h.btn_refine  = uibutton(h.rail, 'Position', [12+btn_w+8 ry btn_w row_h+4], ...
    'Text', 'Refine live', 'BackgroundColor', [0.92 0.96 0.85]);
ry = ry - (row_h+4) - gap*2;

% Section: Target
h.lbl_target = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'Target distribution', 'FontWeight', 'bold', ...
    'FontColor', [0.13 0.18 0.27]);
ry = ry - row_h;
h.lbl_mu     = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', sprintf('μ = [%.1f  %.1f  %.1f] m', state.mu_target));
ry = ry - row_h;
[ev_principal, ev_minor, theta_deg] = sigma_ellipse_params(state.Sigma_target(1:2,1:2));
h.lbl_sigma  = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', sprintf('σ_a = %.1f, σ_b = %.1f m, θ = %.1f°', ...
                    ev_principal, ev_minor, theta_deg));
ry = ry - row_h - gap;

% Section: Solution
h.lbl_sol_hdr = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'Recommended firing solution', 'FontWeight', 'bold', ...
    'FontColor', [0.13 0.18 0.27]);
ry = ry - row_h;
h.lbl_theta_Vo = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'V₀ = — m/s');
ry = ry - row_h;
h.lbl_theta_el = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'el = — °');
ry = ry - row_h;
h.lbl_theta_az = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'az = — °');
ry = ry - row_h;
h.lbl_theta_p  = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'spin = — rad/s');
ry = ry - row_h - gap;

% Section: Predicted outcome
h.lbl_imp_hdr = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'Predicted outcome', 'FontWeight', 'bold', ...
    'FontColor', [0.13 0.18 0.27]);
ry = ry - row_h;
h.lbl_centroid = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'drone-land μ = — m');
ry = ry - row_h;
h.lbl_cep      = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'drone CEP = — m');
ry = ry - row_h;
h.lbl_carrier  = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'carrier impact = — m');
ry = ry - row_h;
h.lbl_jstar    = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'E[||Δp||] = — m');
ry = ry - row_h;
h.lbl_tof      = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'time-of-flight = — s');
ry = ry - row_h;
h.lbl_apex     = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'max altitude = — m');
ry = ry - row_h - gap*2;

% Animation controls
h.lbl_anim_hdr = uilabel(h.rail, 'Position', [12 ry rail_w-24 row_h], ...
    'Text', 'Animation', 'FontWeight', 'bold', ...
    'FontColor', [0.13 0.18 0.27]);
ry = ry - row_h - 4;

abtn_w = (rail_w - 36) / 3;
h.btn_play  = uibutton(h.rail, 'Position', [12 ry abtn_w row_h], 'Text', '▶ Play');
h.btn_pause = uibutton(h.rail, 'Position', [12+abtn_w+6 ry abtn_w row_h], 'Text', '❚❚ Pause');
h.btn_reset = uibutton(h.rail, 'Position', [12+2*(abtn_w+6) ry abtn_w row_h], 'Text', '⟲ Reset');
ry = ry - row_h - 4;

uilabel(h.rail, 'Position', [12 ry 50 row_h], 'Text', 'Speed:');
h.dd_speed = uidropdown(h.rail, 'Position', [62 ry 80 row_h], ...
    'Items', {'0.25×','0.5×','1×','2×','4×'}, 'ItemsData', [0.25 0.5 1 2 4], ...
    'Value', 1);
ry = ry - row_h - 4;

h.sld_scrub = uislider(h.rail, 'Position', [16 ry rail_w-32 3], ...
    'Limits', [0 1], 'Value', 0);
ry = ry - 30 - gap*2;

% Bottom: utility row
ubtn_w = (rail_w - 36) / 3;
h.btn_export = uibutton(h.rail, 'Position', [12 ry ubtn_w row_h], 'Text', '📷 Export');
h.btn_resamp = uibutton(h.rail, 'Position', [12+ubtn_w+6 ry ubtn_w row_h], 'Text', '↻ Resample');
h.btn_reset_target = uibutton(h.rail, 'Position', [12+2*(ubtn_w+6) ry ubtn_w row_h], ...
                              'Text', '⌖ Default tgt');

state.h = h;

% Wire callbacks (capture fig handle in closures)
h.dd_poc.ValueChangedFcn       = @(s,e) cb_poc_changed(fig, s);
h.btn_compute.ButtonPushedFcn  = @(s,e) cb_compute(fig, false);
h.btn_refine.ButtonPushedFcn   = @(s,e) cb_compute(fig, true);
h.btn_play.ButtonPushedFcn     = @(s,e) cb_play(fig);
h.btn_pause.ButtonPushedFcn    = @(s,e) cb_pause(fig);
h.btn_reset.ButtonPushedFcn    = @(s,e) cb_reset_anim(fig);
h.dd_speed.ValueChangedFcn     = @(s,e) cb_speed_changed(fig, s);
h.sld_scrub.ValueChangingFcn   = @(s,e) cb_scrub(fig, e);
h.btn_export.ButtonPushedFcn   = @(s,e) cb_export(fig);
h.btn_resamp.ButtonPushedFcn   = @(s,e) cb_resample(fig);
h.btn_reset_target.ButtonPushedFcn = @(s,e) cb_reset_target(fig);

% Drag callbacks on the map
fig.WindowButtonDownFcn   = @(s,e) cb_button_down(fig);
fig.WindowButtonUpFcn     = @(s,e) cb_button_up(fig);
fig.WindowButtonMotionFcn = @(s,e) cb_button_motion(fig);

fig.CloseRequestFcn = @(s,e) cb_close(fig);

% Initial render of static elements
state = render_map_static(state);
state = render_3d_static(state);
end

% ============================================================
% Compute pipeline
% ============================================================
function state = trigger_compute(state, refine_live)
h = state.h;
h.header_status.Text = 'Computing…';
drawnow;

% Build info_target
info = struct('mu', state.mu_target, 'Sigma', state.Sigma_target, ...
              'samples_train', state.samples_train, ...
              'samples_eval',  state.samples_eval);

t0 = tic;
budget = 0;
if refine_live
    budget = h.sp_budget.Value;
    if budget == 0; budget = 3; end  % default refine budget
end

try
    [theta_star, J_star, meta] = dispatch_poc(state.poc, info, budget);
catch ME
    h.header_status.Text = sprintf('Compute failed: %s', ME.message);
    return
end

state.theta_star = theta_star;
state.J_star = J_star;
if isfield(meta, 'J_eval'); state.J_eval = meta.J_eval; end

% Predict centroid for these θ via nearest-neighbor on cache
state.predicted_centroid = predict_centroid(theta_star, state.lookup, state.ranges);

% Carrier trajectory (try cached, else live eom2)
[traj, tof, apex] = get_carrier_trajectory(theta_star, state.lookup);
state.trajectory = traj;
state.tof = tof;
state.apex = apex;

% Drone descent cone: from deployment band down to landing centroid
state.dispersion_tube = compute_drone_dispersion( ...
    traj, state.predicted_centroid, state.lookup, theta_star, state.ranges);
state.cep_radius = state.dispersion_tube.cep_radius;

% Reset animation
state.anim.frame = 1;
state.anim.playing = false;

% Update display
update_readouts(state);
state = render_solution_overlay(state);
state = render_3d_solution(state);

dt = toc(t0);
if refine_live
    msg = sprintf('Refined live (%d sweeps) in %.1fs — POC %s', budget, dt, state.poc);
else
    msg = sprintf('Cache compute in %.2fs — POC %s', dt, state.poc);
end
h.header_status.Text = msg;
end

function [theta_star, J_star, meta] = dispatch_poc(poc, info, budget)
switch upper(poc)
    case 'A'; [theta_star, J_star, meta] = poc_a_bo_gp(info, 0, budget, false);
    case 'B'; [theta_star, J_star, meta] = poc_b_cem(info, 0, budget, false);
    case 'C'; [theta_star, J_star, meta] = poc_c_sobol(info, 0, budget, false);
    case 'D'; [theta_star, J_star, meta] = poc_d_cvar(info, 0, budget, false);
    case 'E'; [theta_star, J_star, meta] = poc_e_nn_prescreen_cem(info, 0, budget, false);
    case 'F'; [theta_star, J_star, meta] = poc_f_nn_surrogate(info, 0, budget, false);
    otherwise
        error('Unknown POC: %s', poc);
end
end

function pc = predict_centroid(theta_star, lookup, ranges)
% Nearest-neighbor in normalized θ space among reachable cached entries
n = numel(lookup);
theta_mat = zeros(n, 4);
reachable = false(n,1);
for i = 1:n
    pa = lookup(i).params;
    theta_mat(i,:) = [pa.Vo, pa.el, pa.az, pa.p];
    reachable(i) = lookup(i).reachability_pct > 0 && all(isfinite(lookup(i).p_centroid));
end
range_vec = [diff(ranges.Vo), diff(ranges.el), diff(ranges.az), diff(ranges.p)];
d = sqrt(sum(((theta_mat - theta_star) ./ range_vec).^2, 2));
d(~reachable) = inf;
[~, ix] = min(d);
pc = lookup(ix).p_centroid(:).';
end

function [traj, tof, apex] = get_carrier_trajectory(theta, lookup)
% Returns the carrier mortar trajectory for these θ.
% If an exact-θ entry exists in the cache, use its stored ballistic_solution
% (faster, exactly consistent with what the optimizer saw). Otherwise live eom2.
exact_idx = find_exact_theta(lookup, theta);
if ~isempty(exact_idx) && isfield(lookup(exact_idx).ballistic_solution, 'trajectory')
    bs = lookup(exact_idx).ballistic_solution;
    traj.t = bs.time;
else
    env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');
    launch = struct('Vo', theta(1), 'el', theta(2), 'az', theta(3), ...
                    'w_z0', 0, 'w_y0', 0, 'alpha_0', 0, 'beta_0', 0, ...
                    'p', theta(4), 'x_0', 0, 'y_0', 0, 'z_0', 0, 't_max', 300);
    bs = eom2(launch, env, false);
    traj.t = bs.time;
end
% NWU frame: cols 10 = range, 11 = -cross, 12 = altitude
traj.x =  bs.trajectory(:,10);
traj.y = -bs.trajectory(:,11);
traj.z =  bs.trajectory(:,12);
% Trim at impact
[~, I] = max(traj.z);
post = I:numel(traj.z);
imp_idx = post(find(traj.z(post) <= 0, 1, 'first'));
if isempty(imp_idx); imp_idx = numel(traj.z); end
traj.t = traj.t(1:imp_idx);
traj.x = traj.x(1:imp_idx);
traj.y = traj.y(1:imp_idx);
traj.z = traj.z(1:imp_idx);
% Arc length (3D) for deployment-band markers
seg = sqrt(diff(traj.x).^2 + diff(traj.y).^2 + diff(traj.z).^2);
traj.arc = [0; cumsum(seg)];
% Deployment band: 20-80% arc length (match sweep_landing_centroid convention)
arc_total = traj.arc(end);
traj.deploy_arc_lo = 0.20 * arc_total;
traj.deploy_arc_hi = 0.80 * arc_total;
i_lo = find(traj.arc >= traj.deploy_arc_lo, 1, 'first');
i_hi = find(traj.arc >= traj.deploy_arc_hi, 1, 'first');
traj.deploy_idx_lo = i_lo;
traj.deploy_idx_hi = i_hi;
tof = traj.t(end);
apex = max(traj.z);
end

function idx = find_exact_theta(lookup, theta)
idx = [];
for i = 1:numel(lookup)
    p = lookup(i).params;
    th = [p.Vo p.el p.az p.p];
    if max(abs(th - theta)) < 1e-6
        idx = i; return
    end
end
end

function tube = compute_drone_dispersion(traj, centroid, lookup, theta, ranges)
% Build the altitude-resolved drone landing dispersion: a descent cone
% from the deployment band (top) down to the landing centroid (ground).
%
% Top-of-cone (at deployment-band altitude): width tracks the carrier
% trajectory's spatial extent across the deployment band (drones can
% deploy anywhere from arc 20% to 80%).
% Bottom-of-cone (at ground): drone landing CEP around centroid.

% Ground CEP: prefer cached half_radius; else estimate from reachability.
[cep_radius, hr_source] = estimate_cep(theta, lookup, ranges);

if isempty(centroid) || any(~isfinite(centroid))
    centroid = [traj.x(traj.deploy_idx_lo + ...
                       round(0.5*(traj.deploy_idx_hi - traj.deploy_idx_lo))), ...
                traj.y(traj.deploy_idx_lo + ...
                       round(0.5*(traj.deploy_idx_hi - traj.deploy_idx_lo))), 0];
end

% Top-of-cone footprint: take the carrier path between deploy_idx_lo:deploy_idx_hi
% and fit an ellipse on the (x,y) coords. That's the spread of possible deploy positions.
band_x = traj.x(traj.deploy_idx_lo:traj.deploy_idx_hi);
band_y = traj.y(traj.deploy_idx_lo:traj.deploy_idx_hi);
band_z = traj.z(traj.deploy_idx_lo:traj.deploy_idx_hi);
top_mu = [mean(band_x), mean(band_y)];
top_C  = cov([band_x, band_y]);
% Add an isotropic floor so a degenerate (collinear) band still renders
top_C = top_C + 50^2 * eye(2);

bot_mu = centroid(1:2);
bot_C  = cep_radius^2 * eye(2);

% Linearly interpolate ellipses across altitude levels
alt_max = max(band_z);
alt_min = 0;
n_levels = 18;
alt_levels = linspace(alt_min, alt_max, n_levels);
ellipses = cell(n_levels, 1);
for k = 1:n_levels
    s = (alt_levels(k) - alt_min) / max(alt_max - alt_min, eps);  % 0=ground, 1=top
    mu_k = (1 - s) * bot_mu + s * top_mu;
    C_k  = (1 - s) * bot_C  + s * top_C;
    C_k  = (C_k + C_k.') / 2;
    ellipses{k} = struct('mu', mu_k, 'Sigma', C_k);
end

tube.alt_levels = alt_levels;
tube.ellipses = ellipses;
tube.cep_radius = cep_radius;
tube.cep_source = hr_source;
tube.deploy_band_top = [band_x band_y band_z];
tube.landing_centroid = centroid(:).';
end

function [cep, source] = estimate_cep(theta, lookup, ranges)
% Reachability-scaled heuristic for per-trial drone landing CEP.
% (Cached half_radius reflects the sweep-coverage radius across deploy×offset
%  combinations, not a per-drone landing dispersion — too large to use here.)
n = numel(lookup);
theta_mat = zeros(n,4);
reach = zeros(n,1);
for i = 1:n
    pa = lookup(i).params;
    theta_mat(i,:) = [pa.Vo pa.el pa.az pa.p];
    reach(i) = lookup(i).reachability_pct;
end
range_vec = [diff(ranges.Vo) diff(ranges.el) diff(ranges.az) diff(ranges.p)];
d = sqrt(sum(((theta_mat - theta) ./ range_vec).^2, 2));
[~, ord] = sort(d, 'ascend');
r_near = mean(reach(ord(1:min(5, n))));
r_near = max(r_near, 0.05);
cep = 25 + 60 * (1 - r_near);   % ~25m at full reach, ~85m at zero
source = sprintf('reach=%.2f heuristic', r_near);
end

% ============================================================
% Rendering
% ============================================================
function state = render_map_static(state)
ax = state.h.ax_map;
cla(ax);
hold(ax, 'on');

% Cache p_centroids as light gray markers
pcs = nan(numel(state.lookup), 3);
for i = 1:numel(state.lookup)
    if state.lookup(i).reachability_pct > 0 && all(isfinite(state.lookup(i).p_centroid))
        pcs(i,:) = state.lookup(i).p_centroid(:).';
    end
end
state.h.cache_dots = scatter(ax, pcs(:,1), pcs(:,2), 18, [0.78 0.80 0.85], ...
                             'filled', 'MarkerFaceAlpha', 0.5, ...
                             'PickableParts', 'none');

% Determine plot range from cache extent + target margin
xlims = [-200, 1100];
ylims = [-500, 500];
xlim(ax, xlims); ylim(ax, ylims);

% Compass rose (top-right corner)
state.h.compass = annotate_compass(ax, xlims(2)-80, ylims(2)-60, 35);

% Eval cloud (gets refreshed when target moves)
state.h.eval_dots = scatter(ax, state.samples_eval(:,1), state.samples_eval(:,2), ...
                            12, [0.55 0.55 0.62], 'filled', ...
                            'MarkerFaceAlpha', 0.45, 'PickableParts', 'none');

% Target ellipse
[xe, ye] = sigma_ellipse(state.mu_target(1:2), state.Sigma_target(1:2,1:2), 80);
state.h.target_ellipse = plot(ax, xe, ye, 'Color', [0.78 0.18 0.22], ...
                              'LineWidth', 1.6, 'PickableParts', 'none');

% Target μ crosshair (draggable)
state.h.target_mu = plot(ax, state.mu_target(1), state.mu_target(2), '+', ...
                         'Color', [0.78 0.18 0.22], 'MarkerSize', 18, ...
                         'LineWidth', 2.5, 'Tag', 'target_mu');

% Σ axis handles (two: principal & minor endpoints)
[ev_pr, ev_mn, theta_deg] = sigma_ellipse_params(state.Sigma_target(1:2,1:2));
[hp1, hp2] = ellipse_handle_points(state.mu_target(1:2), ev_pr, ev_mn, theta_deg);
state.h.target_h1 = plot(ax, hp1(1), hp1(2), 'sq', ...
                         'Color', [0.78 0.18 0.22], 'MarkerSize', 10, ...
                         'MarkerFaceColor', [1 1 1], 'LineWidth', 1.5, ...
                         'Tag', 'sigma_h1');
state.h.target_h2 = plot(ax, hp2(1), hp2(2), 'sq', ...
                         'Color', [0.78 0.18 0.22], 'MarkerSize', 10, ...
                         'MarkerFaceColor', [1 1 1], 'LineWidth', 1.5, ...
                         'Tag', 'sigma_h2');

% Predicted centroid (filled later)
state.h.pred_centroid = plot(ax, NaN, NaN, 'o', ...
                             'Color', [0.10 0.55 0.30], 'MarkerSize', 12, ...
                             'MarkerFaceColor', [0.30 0.75 0.45], 'LineWidth', 1.6, ...
                             'PickableParts', 'none');

% Mortar origin
plot(ax, 0, 0, '^', 'Color', [0.13 0.18 0.27], 'MarkerSize', 12, ...
     'MarkerFaceColor', [0.13 0.18 0.27], 'PickableParts', 'none');
text(ax, 8, 8, 'mortar', 'FontSize', 9, 'Color', [0.13 0.18 0.27]);

hold(ax, 'off');
state.drag.active = '';
end

function ax = annotate_compass(ax, x, y, R)
hold(ax, 'on');
plot(ax, x + R*cosd(0:5:360), y + R*sind(0:5:360), '-', ...
     'Color', [0.55 0.55 0.6], 'LineWidth', 0.75, 'PickableParts','none');
plot(ax, [x x], [y-R+2 y+R-2], '-', 'Color', [0.55 0.55 0.6], 'PickableParts','none');
plot(ax, [x-R+2 x+R-2], [y y], '-', 'Color', [0.55 0.55 0.6], 'PickableParts','none');
text(ax, x, y+R+5, 'N', 'HorizontalAlignment','center', 'FontSize',9, 'Color',[0.3 0.3 0.36]);
text(ax, x+R+4, y, 'E', 'VerticalAlignment','middle', 'FontSize',9, 'Color',[0.3 0.3 0.36]);
end

function state = render_solution_overlay(state)
ax = state.h.ax_map;
% Update eval cloud + target ellipse
set(state.h.eval_dots,  'XData', state.samples_eval(:,1), 'YData', state.samples_eval(:,2));
[xe, ye] = sigma_ellipse(state.mu_target(1:2), state.Sigma_target(1:2,1:2), 80);
set(state.h.target_ellipse, 'XData', xe, 'YData', ye);
set(state.h.target_mu, 'XData', state.mu_target(1), 'YData', state.mu_target(2));
[ev_pr, ev_mn, theta_deg] = sigma_ellipse_params(state.Sigma_target(1:2,1:2));
[hp1, hp2] = ellipse_handle_points(state.mu_target(1:2), ev_pr, ev_mn, theta_deg);
set(state.h.target_h1, 'XData', hp1(1), 'YData', hp1(2));
set(state.h.target_h2, 'XData', hp2(1), 'YData', hp2(2));

% Predicted drone landing centroid
if all(isfinite(state.predicted_centroid))
    set(state.h.pred_centroid, 'XData', state.predicted_centroid(1), ...
                                'YData', state.predicted_centroid(2));
end

% Drone landing CEP circle on the ground
if isfield(state.h, 'cep_circle') && isvalid(state.h.cep_circle)
    delete(state.h.cep_circle);
end
if isfinite(state.cep_radius) && all(isfinite(state.predicted_centroid))
    phi = linspace(0, 2*pi, 80);
    cx = state.predicted_centroid(1) + state.cep_radius*cos(phi);
    cy = state.predicted_centroid(2) + state.cep_radius*sin(phi);
    hold(ax, 'on');
    state.h.cep_circle = plot(ax, cx, cy, '--', 'Color', [0.10 0.55 0.30], ...
        'LineWidth', 1.3, 'PickableParts', 'none');
    hold(ax, 'off');
end

% Carrier impact + carrier path on the map
if isfield(state.h, 'carrier_path') && isvalid(state.h.carrier_path)
    delete(state.h.carrier_path);
end
if isfield(state.h, 'carrier_impact') && isvalid(state.h.carrier_impact)
    delete(state.h.carrier_impact);
end
if isfield(state.h, 'deploy_band_2d') && isvalid(state.h.deploy_band_2d)
    delete(state.h.deploy_band_2d);
end
if ~isempty(state.trajectory)
    T = state.trajectory;
    hold(ax, 'on');
    state.h.carrier_path = plot(ax, T.x, T.y, '-', ...
        'Color', [0.13 0.40 0.78 0.55], 'LineWidth', 1.2, 'PickableParts','none');
    state.h.deploy_band_2d = plot(ax, ...
        T.x(T.deploy_idx_lo:T.deploy_idx_hi), ...
        T.y(T.deploy_idx_lo:T.deploy_idx_hi), '-', ...
        'Color', [0.18 0.62 0.85], 'LineWidth', 3.2, 'PickableParts','none');
    state.h.carrier_impact = plot(ax, T.x(end), T.y(end), 'x', ...
        'Color', [0.13 0.40 0.78], 'MarkerSize', 12, 'LineWidth', 2, ...
        'PickableParts','none');
    hold(ax, 'off');
end

% Auto-fit map limits to data extent
fit_map_limits(state);
end

function fit_map_limits(state)
ax = state.h.ax_map;
all_x = [state.mu_target(1), state.predicted_centroid(1)];
all_y = [state.mu_target(2), state.predicted_centroid(2)];
if ~isempty(state.trajectory)
    all_x = [all_x, state.trajectory.x(:).'];
    all_y = [all_y, state.trajectory.y(:).'];
end
all_x = all_x(isfinite(all_x));
all_y = all_y(isfinite(all_y));
if isempty(all_x); return; end
xpad = max(80, 0.10 * (max(all_x) - min(all_x)));
ypad = max(80, 0.10 * (max(all_y) - min(all_y)));
xlim(ax, [min(all_x)-xpad, max(all_x)+xpad]);
ylim(ax, [min(all_y)-ypad, max(all_y)+ypad]);
end

function state = render_3d_static(state)
ax = state.h.ax_scn;
cla(ax);
hold(ax, 'on');
% Ground grid
[xg, yg] = meshgrid(-200:200:1100, -500:200:500);
zg = zeros(size(xg));
mesh(ax, xg, yg, zg, 'EdgeColor', [0.85 0.85 0.88], 'FaceColor', 'none');
% Mortar
plot3(ax, 0, 0, 0, '^', 'Color', [0.13 0.18 0.27], 'MarkerSize', 10, ...
      'MarkerFaceColor', [0.13 0.18 0.27]);
xlim(ax, [-200 1100]); ylim(ax, [-500 500]); zlim(ax, [0 600]);
hold(ax, 'off');
end

function state = render_3d_solution(state)
ax = state.h.ax_scn;
cla(ax);
hold(ax, 'on');

% Compute data-driven extent
T = state.trajectory;
if isempty(T)
    xlims = [-200 1100]; ylims = [-500 500]; zlims = [0 600];
else
    xpad = 80; ypad = 80;
    xlims = [min([T.x(:); 0]) - xpad, max([T.x(:); state.mu_target(1)]) + xpad];
    ylims = [min([T.y(:); state.mu_target(2)]) - ypad, ...
             max([T.y(:); state.mu_target(2)]) + ypad];
    zlims = [0, max(T.z) * 1.15];
end

% Ground grid
xg_step = max(50, round((xlims(2)-xlims(1))/12 / 50) * 50);
yg_step = max(50, round((ylims(2)-ylims(1))/8  / 50) * 50);
[xg, yg] = meshgrid(xlims(1):xg_step:xlims(2), ylims(1):yg_step:ylims(2));
zg = zeros(size(xg));
mesh(ax, xg, yg, zg, 'EdgeColor', [0.88 0.88 0.91], 'FaceColor', 'none', ...
     'PickableParts', 'none');

% Target ellipse (red) on ground
[xe, ye] = sigma_ellipse(state.mu_target(1:2), state.Sigma_target(1:2,1:2), 80);
plot3(ax, xe, ye, zeros(size(xe)), '-', 'Color', [0.78 0.18 0.22], ...
      'LineWidth', 2, 'PickableParts','none');

% Drone descent cone — translucent ellipses from deploy alt down to ground
if ~isempty(state.dispersion_tube) && ~isempty(state.dispersion_tube.alt_levels)
    levels = state.dispersion_tube.alt_levels;
    Es     = state.dispersion_tube.ellipses;
    for k = 1:numel(levels)
        E = Es{k};
        if any(~isfinite(E.Sigma(:))); continue; end
        [xe2, ye2] = sigma_ellipse(E.mu, E.Sigma, 60);
        % Alpha gradient: more opaque near ground (where drones land)
        alpha = 0.18 + 0.32 * (1 - levels(k)/max(levels));
        plot3(ax, xe2, ye2, levels(k)*ones(size(xe2)), '-', ...
              'Color', [0.10 0.55 0.30 alpha], 'LineWidth', 0.7, ...
              'PickableParts','none');
    end
end

% Carrier flight path (mortar shell)
if ~isempty(T)
    plot3(ax, T.x, T.y, T.z, '-', 'Color', [0.13 0.40 0.78], ...
          'LineWidth', 2.2, 'PickableParts','none');
    % Deployment band: thicker cyan segment between 20-80% arc length
    di = T.deploy_idx_lo:T.deploy_idx_hi;
    plot3(ax, T.x(di), T.y(di), T.z(di), '-', ...
          'Color', [0.18 0.62 0.85], 'LineWidth', 4, 'PickableParts','none');
    % Sample 4 deployment markers across the band
    deploy_pts = round(linspace(T.deploy_idx_lo, T.deploy_idx_hi, 4));
    plot3(ax, T.x(deploy_pts), T.y(deploy_pts), T.z(deploy_pts), 'o', ...
          'Color', [0.06 0.45 0.70], 'MarkerSize', 7, ...
          'MarkerFaceColor', [0.50 0.80 0.95], 'LineWidth', 1.5, ...
          'PickableParts','none');
    % Carrier impact (X marker)
    plot3(ax, T.x(end), T.y(end), 0, 'x', ...
          'Color', [0.13 0.40 0.78], 'MarkerSize', 12, 'LineWidth', 2.2, ...
          'PickableParts','none');
    state.h.flight_marker = plot3(ax, T.x(1), T.y(1), T.z(1), 'o', ...
        'Color', [0.13 0.40 0.78], 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.30 0.55 0.95], 'LineWidth', 1.5, ...
        'PickableParts','none');
end

% Drone landing centroid + CEP on the ground
if all(isfinite(state.predicted_centroid))
    cx = state.predicted_centroid(1); cy = state.predicted_centroid(2);
    plot3(ax, cx, cy, 0, 'o', 'Color', [0.10 0.55 0.30], 'MarkerSize', 11, ...
          'MarkerFaceColor', [0.30 0.75 0.45], 'LineWidth', 1.6, ...
          'PickableParts','none');
    if isfinite(state.cep_radius)
        phi = linspace(0, 2*pi, 60);
        plot3(ax, cx + state.cep_radius*cos(phi), cy + state.cep_radius*sin(phi), ...
              zeros(size(phi)), '--', 'Color', [0.10 0.55 0.30], ...
              'LineWidth', 1.2, 'PickableParts','none');
    end
end

% Mortar
plot3(ax, 0, 0, 0, '^', 'Color', [0.13 0.18 0.27], 'MarkerSize', 10, ...
      'MarkerFaceColor', [0.13 0.18 0.27], 'PickableParts','none');

xlim(ax, xlims); ylim(ax, ylims); zlim(ax, zlims);
xlabel(ax, 'Range (m)'); ylabel(ax, 'Crossrange (m)'); zlabel(ax, 'Altitude (m)');
ax.DataAspectRatio = [1 1 1];
hold(ax, 'off');
end

function update_readouts(state)
h = state.h;
h.lbl_mu.Text = sprintf('μ = [%.1f  %.1f  %.1f] m', state.mu_target);
[ev_pr, ev_mn, theta_deg] = sigma_ellipse_params(state.Sigma_target(1:2,1:2));
h.lbl_sigma.Text = sprintf('σ_a = %.1f, σ_b = %.1f m, θ = %.1f°', ev_pr, ev_mn, theta_deg);

if ~isempty(state.theta_star)
    th = state.theta_star;
    h.lbl_theta_Vo.Text = sprintf('V₀ = %.1f m/s', th(1));
    h.lbl_theta_el.Text = sprintf('el = %.1f°', th(2));
    h.lbl_theta_az.Text = sprintf('az = %.1f°', th(3));
    h.lbl_theta_p.Text  = sprintf('spin = %.2f rad/s', th(4));
end
if all(isfinite(state.predicted_centroid))
    h.lbl_centroid.Text = sprintf('drone-land μ = [%.0f  %.0f] m', ...
                                   state.predicted_centroid(1), state.predicted_centroid(2));
end
if isfinite(state.cep_radius)
    h.lbl_cep.Text = sprintf('drone CEP ≈ %.1f m', state.cep_radius);
end
if ~isempty(state.trajectory)
    h.lbl_carrier.Text = sprintf('carrier impact = [%.0f  %.0f] m', ...
        state.trajectory.x(end), state.trajectory.y(end));
end
if isfinite(state.J_star)
    h.lbl_jstar.Text = sprintf('J* = %.2f m,  J_eval = %.2f m', state.J_star, state.J_eval);
end
if isfield(state, 'tof') && isfinite(state.tof)
    h.lbl_tof.Text = sprintf('time-of-flight = %.1f s', state.tof);
end
if isfield(state, 'apex') && isfinite(state.apex)
    h.lbl_apex.Text = sprintf('max altitude = %.1f m', state.apex);
end
end

% ============================================================
% Geometry helpers
% ============================================================
function [xe, ye] = sigma_ellipse(mu, Sigma, npts)
if nargin < 3, npts = 80; end
phi = linspace(0, 2*pi, npts);
% 1-sigma ellipse
[V, D] = eig(Sigma);
L = V * sqrt(max(D, 0));
xy = L * [cos(phi); sin(phi)];
xe = xy(1,:) + mu(1);
ye = xy(2,:) + mu(2);
end

function [a, b, theta_deg] = sigma_ellipse_params(Sigma2d)
[V, D] = eig(Sigma2d);
[d_sorted, idx] = sort(diag(D), 'descend');
V = V(:, idx);
a = sqrt(max(d_sorted(1), 0));
b = sqrt(max(d_sorted(2), 0));
theta_deg = atan2d(V(2,1), V(1,1));
end

function [hp1, hp2] = ellipse_handle_points(mu, a, b, theta_deg)
% End of major axis (handle 1), end of minor axis (handle 2)
ct = cosd(theta_deg); st = sind(theta_deg);
hp1 = mu(:).' + a*[ct st];
hp2 = mu(:).' + b*[-st ct];
end

% ============================================================
% Callbacks
% ============================================================
function cb_poc_changed(fig, src)
state = fig.UserData;
state.poc = src.Value;
fig.UserData = state;
% Light recompute via cache
state = trigger_compute(state, false);
fig.UserData = state;
end

function cb_compute(fig, refine_live)
state = fig.UserData;
state = trigger_compute(state, refine_live);
fig.UserData = state;
end

function cb_resample(fig)
state = fig.UserData;
% New rng seed for variety
rng(randi(1e6));
state = resample_target(state);
state = render_solution_overlay(state);
fig.UserData = state;
end

function cb_reset_target(fig)
state = fig.UserData;
state.mu_target = state.opts.mu0(:).';
state.Sigma_target = state.opts.Sigma0;
state = resample_target(state);
state = trigger_compute(state, false);
fig.UserData = state;
end

function cb_export(fig)
state = fig.UserData;
out_dir = fullfile(fileparts(mfilename('fullpath')), 'figs');
if ~exist(out_dir, 'dir'); mkdir(out_dir); end
ts = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<DATST>
fn_map = fullfile(out_dir, sprintf('demo_map_%s.png', ts));
fn_3d  = fullfile(out_dir, sprintf('demo_3d_%s.png',  ts));
exportgraphics(state.h.ax_map, fn_map, 'Resolution', 300);
exportgraphics(state.h.ax_scn, fn_3d,  'Resolution', 300);
state.h.header_status.Text = sprintf('Exported: %s, %s', fn_map, fn_3d);
end

function cb_close(fig)
state = fig.UserData;
if isfield(state, 'anim') && ~isempty(state.anim.timer)
    try; stop(state.anim.timer); delete(state.anim.timer); catch; end
end
delete(fig);
end

% --- Animation -----
function cb_play(fig)
state = fig.UserData;
if isempty(state.trajectory); return; end
if isfield(state.anim, 'timer') && ~isempty(state.anim.timer) && isvalid(state.anim.timer)
    stop(state.anim.timer); delete(state.anim.timer);
end
period = max(0.04, 0.04 / state.anim.speed);
state.anim.timer = timer('ExecutionMode', 'fixedRate', 'Period', period, ...
    'TimerFcn', @(s,e) anim_tick(fig));
state.anim.playing = true;
fig.UserData = state;
start(state.anim.timer);
end

function cb_pause(fig)
state = fig.UserData;
if isfield(state.anim, 'timer') && ~isempty(state.anim.timer) && isvalid(state.anim.timer)
    stop(state.anim.timer);
end
state.anim.playing = false;
fig.UserData = state;
end

function cb_reset_anim(fig)
state = fig.UserData;
cb_pause(fig);
state.anim.frame = 1;
state.h.sld_scrub.Value = 0;
update_flight_marker(state);
fig.UserData = state;
end

function cb_speed_changed(fig, src)
state = fig.UserData;
state.anim.speed = src.Value;
fig.UserData = state;
if state.anim.playing
    cb_play(fig);
end
end

function cb_scrub(fig, evt)
state = fig.UserData;
if isempty(state.trajectory); return; end
n = numel(state.trajectory.x);
state.anim.frame = max(1, round(evt.Value * (n - 1)) + 1);
fig.UserData = state;
update_flight_marker(state);
end

function anim_tick(fig)
state = fig.UserData;
if isempty(state.trajectory)
    cb_pause(fig); return;
end
n = numel(state.trajectory.x);
state.anim.frame = state.anim.frame + 1;
if state.anim.frame > n
    state.anim.frame = 1;
end
state.h.sld_scrub.Value = (state.anim.frame - 1) / max(1, n - 1);
fig.UserData = state;
update_flight_marker(state);
end

function update_flight_marker(state)
if ~isfield(state.h, 'flight_marker') || ~isvalid(state.h.flight_marker); return; end
T = state.trajectory;
i = state.anim.frame;
i = max(1, min(i, numel(T.x)));
set(state.h.flight_marker, 'XData', T.x(i), 'YData', T.y(i), 'ZData', T.z(i));
end

% --- Drag (map μ + Σ handles) -----
function cb_button_down(fig)
state = fig.UserData;
ax = state.h.ax_map;
cp = ax.CurrentPoint;
mx = cp(1,1); my = cp(1,2);
% If outside the map, ignore
xlims = xlim(ax); ylims = ylim(ax);
if mx < xlims(1) || mx > xlims(2) || my < ylims(1) || my > ylims(2); return; end

% Hit-test in *normalized* axes coords so handle radius is screen-uniform
hit_radius = 0.025 * (xlims(2) - xlims(1));   % ~2.5% of x range
candidates = {
    'mu',  [state.mu_target(1) state.mu_target(2)];
    'h1',  [state.h.target_h1.XData state.h.target_h1.YData];
    'h2',  [state.h.target_h2.XData state.h.target_h2.YData];
};
best = ''; best_d = inf;
for i = 1:size(candidates,1)
    d = hypot(candidates{i,2}(1) - mx, candidates{i,2}(2) - my);
    if d < hit_radius && d < best_d
        best_d = d; best = candidates{i,1};
    end
end
if isempty(best); return; end
state.drag.active = best;
fig.UserData = state;
end

function cb_button_motion(fig)
state = fig.UserData;
if ~isfield(state, 'drag') || isempty(state.drag.active); return; end
ax = state.h.ax_map;
cp = ax.CurrentPoint;
mx = cp(1,1); my = cp(1,2);

switch state.drag.active
    case 'mu'
        state.mu_target(1) = mx;
        state.mu_target(2) = my;
        state = resample_target(state);
    case 'h1'
        % Set principal axis length = distance from μ to mouse;
        % orientation = angle from μ to mouse.
        v = [mx - state.mu_target(1), my - state.mu_target(2)];
        a = max(5, hypot(v(1), v(2)));
        theta_deg = atan2d(v(2), v(1));
        [~, b, ~] = sigma_ellipse_params(state.Sigma_target(1:2,1:2));
        b = max(5, b);
        state.Sigma_target(1:2,1:2) = build_sigma(a, b, theta_deg);
        state = resample_target(state);
    case 'h2'
        v = [mx - state.mu_target(1), my - state.mu_target(2)];
        b = max(5, hypot(v(1), v(2)));
        [a, ~, theta_deg] = sigma_ellipse_params(state.Sigma_target(1:2,1:2));
        a = max(5, a);
        % Minor axis must be perpendicular to principal — ignore mouse angle
        state.Sigma_target(1:2,1:2) = build_sigma(a, b, theta_deg);
        state = resample_target(state);
end
state = render_solution_overlay(state);
update_readouts(state);
fig.UserData = state;
end

function cb_button_up(fig)
state = fig.UserData;
was_dragging = isfield(state, 'drag') && ~isempty(state.drag.active);
state.drag.active = '';
fig.UserData = state;
if was_dragging
    state = trigger_compute(state, false);
    fig.UserData = state;
end
end

function S = build_sigma(a, b, theta_deg)
ct = cosd(theta_deg); st = sind(theta_deg);
R = [ct -st; st ct];
S = R * diag([a^2 b^2]) * R.';
S = (S + S.') / 2;
end
