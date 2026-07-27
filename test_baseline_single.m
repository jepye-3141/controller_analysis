function [res, info] = test_baseline_single(model, mode, varargin)
% test_baseline_single  Validate a cloned single-drone controller model on the
% ballistic-stabilization task, using the SAME oracle (ballistic_success) and
% seeds (ballistic_deploy_state on the operational arc) as analysis.m's
% envelope. A controller must truly stabilize -- staying finite is not enough.
%
%   test_baseline_single('adrc_swarm_single')                  % apogee, 100 s
%   test_baseline_single('adrc_swarm_single','apogee')
%   res = test_baseline_single('adrc_swarm_single','envelope') % 17-point sweep
%   test_baseline_single('adrc_swarm_single','envelope','Points',1:3)  % subset
%
% Returns res (logical scalar for apogee; logical mask for envelope) and info
% (the ballistic_success diagnostic struct, or a cell array of them). Prints a
% per-run breakdown of the four success conditions.
%
% There is no cap knob here: this always runs at whatever Omega2_max constants.m
% sets (the default 2), because it calls `constants` itself below. For a peer
% score at a swept cap, use sweep_ballistic_envelope, which overrides the bus
% field per trial.

    if nargin < 2 || isempty(mode), mode = 'apogee'; end
    ip = inputParser;
    ip.addParameter('StopTime', []);
    ip.addParameter('Points', []);       % envelope subset (indices into 1:10:end)
    ip.parse(varargin{:});

    % --- Path + base workspace (mirror analysis.m's setup) ---
    if exist('Ballistics_Simulation-master', 'dir'), addpath('Ballistics_Simulation-master'); end
    evalin('base', 'constants');              % A, B, dt, constants_struct(_bus)
    assignin('base', 'x0_step', [2; 4; 10]);
    assignin('base', 'xf', [0; 0; 0]);
    STABILIZE = 4;

    env    = aero_constants('std_atm.csv', 'Aerodynamic_Char_120mm_Mortar.xlsx');
    launch = operational_launch();
    bsol   = eom2(launch, env, false);

    load_system(model);
    cleaner = onCleanup(@() close_system(model, 0)); %#ok<NASGU>

    switch lower(mode)
      case 'apogee'
        St = 100; if ~isempty(ip.Results.StopTime), St = ip.Results.StopTime; end
        [res, info] = run_one(model, bsol, bsol.apogee_idx, St, STABILIZE);
        print_row(sprintf('APOGEE %s', model), res, info);

      case 'envelope'
        St = 30; if ~isempty(ip.Results.StopTime), St = ip.Results.StopTime; end
        n_rows = size(bsol.trajectory, 1);
        tp = 1:10:n_rows; tp(tp == n_rows) = [];    % drop the ground-impact row
        sel = 1:numel(tp);
        if ~isempty(ip.Results.Points), sel = ip.Results.Points; end
        res = false(1, numel(sel)); info = cell(1, numel(sel));
        for j = 1:numel(sel)
            k = sel(j);
            [res(j), info{j}] = run_one(model, bsol, tp(k), St, STABILIZE);
            print_row(sprintf('  tp%2d row%3d', k, tp(k)), res(j), info{j});
        end
        fprintf('ENVELOPE %s: %d/%d success\n', model, sum(res), numel(res));

      otherwise
        error('test_baseline_single:mode', 'mode must be ''apogee'' or ''envelope''');
    end
end

function [ok, info] = run_one(model, bsol, idx, St, STABILIZE)
    dp = bsol.trajectory(idx, :).';
    xi = ballistic_deploy_state(dp(1:3), dp(4:6), dp(7:9), dp(10:12));
    assignin('base', 'xi', xi);
    assignin('base', 'xf_ballistic', [xi(10:11); 0]);
    assignin('base', 'simcase', STABILIZE);
    set_param(model, 'StopTime', num2str(St), 'SimulationMode', 'Rapid');
    out = sim(model);
    pos = squeeze(out.posout.Data(1, :, :)).';
    vel = squeeze(out.velout.Data(1, :, :)).';
    [ok, info] = ballistic_success(out.posout.Time, pos, vel, xi(10:11), ...
        norm(xi(1:3)), St, RotVel = out.rotvelout.Data);
end

function print_row(tag, ok, info)
    fprintf(['%-24s ok=%d  [dur=%d pos=%d vel=%d rot=%d]  miss=%.2fm  ' ...
             'vRatio=%.2f  rotPost=%.2f  tEnd=%.1fs\n'], ...
        tag, ok, info.duration_ok, info.position_ok, info.velocity_ok, ...
        info.rotation_ok, info.final_miss, info.max_speed_ratio, ...
        info.max_rot_post_ratio, info.t_end);
end
