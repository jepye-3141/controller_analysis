% Exercise the full compute path of atlis_targeting_app without launching the GUI.
addpath(pwd); addpath('Ballistics_Simulation-master');
copyfile('centroid_lookup_log_v3_iter6.mat','centroid_lookup_log.mat');

% Step 1: trigger a compute via the public function-handle path
fprintf('Launching app in headless mode for smoke test...\n');

% Open the app in invisible mode by tweaking the figure right after creation
fig = atlis_targeting_app('poc', 'C');
state = fig.UserData;

% Validate state
assert(~isempty(state.theta_star), 'theta_star not set');
assert(all(isfinite(state.predicted_centroid)), 'predicted_centroid invalid');
assert(~isempty(state.trajectory), 'trajectory empty');
assert(~isempty(state.dispersion_tube), 'dispersion_tube empty');
assert(isfinite(state.cep_radius), 'cep_radius invalid');

fprintf('theta_star = [%.2f %.2f %.2f %.2f]\n', state.theta_star);
fprintf('drone-land μ = [%.1f %.1f %.1f]\n', state.predicted_centroid);
fprintf('cep_radius = %.1f m (%s)\n', state.cep_radius, state.dispersion_tube.cep_source);
fprintf('carrier impact = [%.1f %.1f]\n', state.trajectory.x(end), state.trajectory.y(end));
fprintf('time-of-flight = %.1f s, apex = %.1f m\n', state.tof, state.apex);
fprintf('deploy band: idx %d to %d (alts %.0f to %.0f m)\n', ...
    state.trajectory.deploy_idx_lo, state.trajectory.deploy_idx_hi, ...
    state.trajectory.z(state.trajectory.deploy_idx_lo), ...
    state.trajectory.z(state.trajectory.deploy_idx_hi));
fprintf('dispersion alt levels: %d, top ellipse area scale: %.1f m\n', ...
    numel(state.dispersion_tube.alt_levels), ...
    sqrt(max(eig(state.dispersion_tube.ellipses{end}.Sigma))));

% Try changing POC
fprintf('\nSwitching POC to E (this calls poc_e_nn_prescreen_cem)...\n');
fig.UserData.poc = 'E';  % manual change to avoid invoking dropdown callback
% Re-run trigger_compute by direct call... the function is private. Instead
% re-launch:
delete(fig);
fig2 = atlis_targeting_app('poc', 'E');
state2 = fig2.UserData;
fprintf('POC E theta = [%.2f %.2f %.2f %.2f]\n', state2.theta_star);
fprintf('POC E drone-land μ = [%.1f %.1f]\n', state2.predicted_centroid(1:2));

delete(fig2);
fprintf('SMOKE TEST PASSED\n');
