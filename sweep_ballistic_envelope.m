function out = sweep_ballistic_envelope(traj_params)
% sweep_ballistic_envelope  Reusable ballistic-envelope runner: fly one mortar
% arc, deploy a single drone at every 10th trajectory ROW, and score STABILIZE
% success per controller arm. Factored verbatim from analysis.m's envelope loop
% (analysis.m:167-235) so the shared runner and analysis.m report identical
% masks. analysis.m runs this section SERIALLY (plain sim(), no parsim); so does
% this runner -- it never contends for the shared parpool.
%
% traj_params : struct
%   Launch fields (required, same as operational_launch / sweep_landing_centroid):
%     .Vo .el .az .w_z0 .w_y0 .p .alpha_0 .beta_0 .x_0 .y_0 .z_0 .t_max
%   Optional control fields:
%     .model         "lqr"|"pid"|"smc"|"dsmc"|"se3"|"dsmc_naive". Default runs
%                    the five mask arms (lqr, pid, smc, dsmc_nosat, dsmc_sat) so
%                    a bare call reproduces the envelope mask contract. "dsmc"
%                    expands to the nosat + sat sub-arm pair (one model, two
%                    constants_struct). "dsmc_naive" is a THIRD sub-arm of that
%                    same model -- saturation_on=false with unconstrained=false,
%                    i.e. the Plan A+ allocation and anti-windup bypassed but the
%                    STEP-12b per-rotor clip still applied. That is exactly the
%                    arm sweep_landing_centroid scores as "naive" (the 22.1%
%                    baseline). It is deliberately NOT part of the five-arm mask
%                    contract; it exists to decompose the dsmc_nosat -> dsmc_sat
%                    margin into gains / clip / allocation (added 2026-07-27).
%     .saturation_on -> constants_struct.saturation_on (se3 / single-arm runs;
%                    the two dSMC sub-arms force their own defining values).
%     .unconstrained -> constants_struct.unconstrained (same caveat).
%     .Omega2_max    -> constants_struct.Omega2_max (per-rotor cap, default 2);
%                    reaches the dSMC STEP-12b clip and se3 apply_rotor_clip.
%     .retain_traces bool, default false; keep the full per-point trace.
%
% out : struct
%   .n_points           == n_test (17 at operational_launch defaults)
%   .test_points        1 x n_points trajectory ROW indices scored
%   .ballistic_solution full eom2 struct (arc geometry / apogee_idx)
%   Per arm (out.lqr, out.pid, out.smc, out.dsmc_nosat, out.dsmc_sat, out.se3,
%   out.dsmc_naive -- whichever the .model selection produced):
%     .n_success   nnz(mask)
%     .mask        1 x n_points logical
%     .envelope    test_points(mask) -- the surviving row indices
%     .crit_table  n_points-row table, sweep_landing_centroid's VariableNames
%                  (envelope has no cross-track/neighbor grid, so ct = nb = 0
%                  and deploy = station index i)
%     .traces      1 x n_points struct {t, pos, vel, rotvel, cmd}, only when
%                  retain_traces=true
%
% Success is scored post-hoc by ballistic_success at analysis.m's defaults
% (StopTime 30 s, ReachTol 10 m, RotGraceT 1 s, VelRatioMax/RotRatioMax 2). At
% operational_launch defaults, and at the constants.m default cap
% Omega2_max = 2, out.{lqr,pid,smc,dsmc_nosat,dsmc_sat}.n_success
% == [5, 0, 0, 6, 9] and out.n_points == 17 -- the regression contract. The
% dSMC-sat 9 is on the tuned gains (0813df0); the pre-tune value was 8.
%
% Stations are RAW trajectory ROWS (analysis.m:172-179), NOT the arc-length grid
% sweep_landing_centroid uses: the runner reads exact eom2 rows (1:10:end minus
% the appended ground-impact row), so the station count moves with the
% launch/physics and 17 is the anchor at the operational launch.

%% Constants (A, B, dt, discrete LQR gains, constants_struct + bus)
constants;

retain_traces = isfield(traj_params, 'retain_traces') && traj_params.retain_traces;

%% Controller arms
% Each arm carries its Simulink model and a PRIVATE constants_struct copy: the
% two dSMC sub-arms run ONE model (discrete_smc_swarm_single) differing only in
% saturation_on/unconstrained, so those toggles live on the per-arm struct
% (setVariable below) rather than being mutated in a shared workspace between
% sims (the analysis.m serial pattern). base_cs inherits any traj_params
% override; the two dSMC sub-arms then force their defining configuration.
base_cs = constants_struct;
if isfield(traj_params, 'saturation_on'), base_cs.saturation_on = traj_params.saturation_on; end
if isfield(traj_params, 'unconstrained'), base_cs.unconstrained = traj_params.unconstrained; end
if isfield(traj_params, 'Omega2_max'),    base_cs.Omega2_max    = traj_params.Omega2_max;    end

nosat_cs = base_cs; nosat_cs.saturation_on = false; nosat_cs.unconstrained = true;   % dsmc_no_constraints (unclipped)
sat_cs   = base_cs; sat_cs.saturation_on   = true;  sat_cs.unconstrained   = false;  % Plan A+ + STEP-12b clip
naive_cs = base_cs; naive_cs.saturation_on = false; naive_cs.unconstrained = false;  % allocation bypassed, STEP-12b clip ONLY

% APPEND new arms at the END: the switch below indexes all_arms positionally, so
% inserting an arm mid-list would silently repoint an existing nickname.
all_arms = struct( ...
    'name',  {'lqr', 'pid', 'smc', 'dsmc_nosat', 'dsmc_sat', 'se3', 'dsmc_naive'}, ...
    'model', {"lqr_swarm_single", "pid_redux_single", "smc_swarm_single", ...
              "discrete_smc_swarm_single", "discrete_smc_swarm_single", ...
              "se3_swarm_single", "discrete_smc_swarm_single"}, ...
    'cs',    {base_cs, base_cs, base_cs, nosat_cs, sat_cs, base_cs, naive_cs});

if isfield(traj_params, 'model')
    switch string(traj_params.model)
        case "lqr",  arms = all_arms(1);
        case "pid",  arms = all_arms(2);
        case "smc",  arms = all_arms(3);
        case "dsmc", arms = all_arms([4 5]);   % nosat + sat sub-arm pair
        case "se3",  arms = all_arms(6);
        case "dsmc_naive", arms = all_arms(7);   % decomposition arm, not a mask arm
        otherwise
            error('sweep_ballistic_envelope:bad_model', ...
                'Unknown model "%s" (expected lqr|pid|smc|dsmc|se3|dsmc_naive).', ...
                traj_params.model);
    end
else
    arms = all_arms(1:5);   % the five mask arms (se3 is forensic-only)
end
n_arm = numel(arms);

%% Mission trajectory (same arc analysis.m's apogee test flies)
env = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');
ballistic_solution = eom2(traj_params, env, false);

%% Station sampling -- RAW ROW INDEXING (analysis.m:172-179), not arc length.
% Every 10th trajectory row on eom2's fixed 0.1 s dense-output grid, minus the
% appended ground-impact row (altitude 0 / full impact velocity; the criterion
% never checks z). n_test is physics-dependent -- 17 at the operational launch.
n_rows = size(ballistic_solution.trajectory, 1);
test_points = 1:10:n_rows;
test_points(test_points == n_rows) = [];
n_test = size(test_points, 2);

STABILIZE = 4;
sim_time  = 30;              % per-trial StopTime (s); also ballistic_success expected_T
simcase   = STABILIZE;

% Placeholder workspace vars for the Step-block parameters (evaluated at compile
% time even in STABILIZE mode, where they are logically unused).
x0_step = [2; 4; 10];
xf      = [0; 0; 0];

%% Per-arm SimulationInput templates + result preallocation
% Statics are over-set as a superset (A/B/dt/discrete-LQR gains/bus/Step
% placeholders): serial sim() would also resolve them from the base workspace,
% but setVariable-ing the full constants set keeps the runner self-contained and
% a variable an arm's model does not read is simply ignored. constants_struct is
% NOT shared across arms (the two dSMC arms differ) so it lives on each template;
% per-station xi/xf_ballistic/simcase are copied on in the loop.
for a = 1:n_arm
    t_in = Simulink.SimulationInput(arms(a).model);
    % 'rapid' == analysis.m's 'Rapid' (Rapid Accelerator mode).
    t_in = t_in.setModelParameter('StopTime', num2str(sim_time), 'SimulationMode', 'rapid');
    t_in = t_in.setVariable('A', A);
    t_in = t_in.setVariable('B', B);
    t_in = t_in.setVariable('constants_struct', arms(a).cs);
    t_in = t_in.setVariable('constants_struct_bus', constants_struct_bus);
    t_in = t_in.setVariable('dt', dt);
    t_in = t_in.setVariable('Kp_d', Kp_d);   % 15-state discrete LQR gains (lqr arm)
    t_in = t_in.setVariable('Ki_d', Ki_d);
    t_in = t_in.setVariable('K', K);
    t_in = t_in.setVariable('x0_step', x0_step);
    t_in = t_in.setVariable('xf', xf);
    arms(a).template = t_in;

    arms(a).mask = false(1, n_test);
    arms(a).info = cell(1, n_test);
    if retain_traces
        arms(a).traces = repmat( ...
            struct('t', [], 'pos', [], 'vel', [], 'rotvel', [], 'cmd', []), 1, n_test);
    end
end

%% Envelope loop -- one deploy per station, every arm scored post-hoc
for i = 1:n_test
    idx = test_points(i);
    deploy_point = ballistic_solution.trajectory(idx, :).';   % 15x1 NWU row
    % Deploy seed: attitude from the pointing vector (cols 7:9), velocities and
    % rates mapped world->body (2026-07-10 fix). Never read the angle columns.
    xi = ballistic_deploy_state(deploy_point(1:3), deploy_point(4:6), ...
        deploy_point(7:9), deploy_point(10:12));
    xf_ballistic = [xi(10:11); 0];   % target = the station's own deploy XY, z=0
    v0_norm      = norm(xi(1:3));     % body-frame deploy speed (norm frame-invariant)

    for a = 1:n_arm
        simIn = arms(a).template;
        simIn = simIn.setVariable('xi', xi);
        simIn = simIn.setVariable('xf_ballistic', xf_ballistic);
        simIn = simIn.setVariable('simcase', simcase);
        out_sim = sim(simIn);

        % Leader (index 1) traces: posout world NWU; velout NED components
        % (y,z sign-flipped vs posout -- an uncompensated *_single root gain,
        % norm-safe since every consumer uses ||.||; audit C6).
        t_trial   = out_sim.posout.Time;
        pos_trial = squeeze(out_sim.posout.Data(1,:,:)).';
        vel_trial = squeeze(out_sim.velout.Data(1,:,:)).';

        % Success criterion (eq:ballistic-success) at analysis.m defaults: full
        % 30 s + final XY within 10 m + velocity ratio <= 2 (whole trace) +
        % rotation-rate ratio <= 2 for t > 1 s. Pass RAW rotvelout.Data (plant
        % Euler-angle rates); ballistic_success normalizes its [3x1xT]/[Tx3] shape.
        [ok, info] = ballistic_success(t_trial, pos_trial, vel_trial, ...
            xi(10:11), v0_norm, sim_time, RotVel=out_sim.rotvelout.Data);
        arms(a).mask(i) = ok;
        arms(a).info{i} = info;

        if retain_traces
            % rotvelout shape varies ([3x1xT] lqr/dsmc/se3 vs [Tx3] pid/smc);
            % orient by raw ndims (NOT a size==3 heuristic -- mis-orients T==3).
            rot_trial = squeeze(out_sim.rotvelout.Data);
            if ndims(out_sim.rotvelout.Data) == 3, rot_trial = rot_trial.'; end
            % ctrlout = applied [T/f; Mx; My; Mz], ABSOLUTE thrust (the channel
            % the per-rotor Omega^2 reconstruction operates on downstream);
            % front-zero-pad to t_trial length (ctrlout may lag posout).
            ctrl_raw = squeeze(out_sim.ctrlout.Data).';
            n_pad    = length(t_trial) - size(ctrl_raw, 1);
            if n_pad > 0
                cmd_trial = [zeros(n_pad, size(ctrl_raw, 2)); ctrl_raw];
            else
                cmd_trial = ctrl_raw;
            end
            arms(a).traces(i) = struct('t', t_trial, 'pos', pos_trial, ...
                'vel', vel_trial, 'rotvel', rot_trial, 'cmd', cmd_trial);
        end
    end
end

%% Assemble output
out = struct('n_points', n_test, 'test_points', test_points, ...
             'ballistic_solution', ballistic_solution);
for a = 1:n_arm
    mask = arms(a).mask;
    res  = struct( ...
        'n_success',  nnz(mask), ...
        'mask',       mask, ...
        'envelope',   test_points(mask), ...
        'crit_table', envelope_crit_table(arms(a).info));
    if retain_traces
        res.traces = arms(a).traces;
    end
    out.(arms(a).name) = res;
end

end

function tbl = envelope_crit_table(info_cells)
% Compact per-station criteria, mirroring sweep_landing_centroid.m's crit_table
% VariableNames. The envelope has no cross-track/neighbor grid, so ct = nb = 0
% (placeholders keeping the schema identical) and deploy = station index i.
n = numel(info_cells);
[deploy, ct, nb] = deal(zeros(n, 1));
[stable, duration_ok, position_ok, velocity_ok, rotation_ok] = deal(false(n, 1));
[final_miss, max_speed_ratio, max_rot_post_ratio] = deal(nan(n, 1));
for i = 1:n
    c = info_cells{i};
    deploy(i)             = i;
    stable(i)             = c.success;
    duration_ok(i)        = c.duration_ok;
    position_ok(i)        = c.position_ok;
    velocity_ok(i)        = c.velocity_ok;
    rotation_ok(i)        = c.rotation_ok;
    final_miss(i)         = c.final_miss;
    max_speed_ratio(i)    = c.max_speed_ratio;
    max_rot_post_ratio(i) = c.max_rot_post_ratio;
end
tbl = table(deploy, ct, nb, stable, duration_ok, position_ok, velocity_ok, ...
    rotation_ok, final_miss, max_speed_ratio, max_rot_post_ratio, ...
    'VariableNames', {'deploy','ct','nb','stable','duration_ok','position_ok', ...
    'velocity_ok','rotation_ok','final_miss','max_speed_ratio','max_rot_post_ratio'});
end
