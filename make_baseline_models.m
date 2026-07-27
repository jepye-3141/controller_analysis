function make_baseline_models(ctrls, overwrite)
% make_baseline_models  Clone the dSMC single-drone ballistic-stabilization
% model into per-controller baseline models, WITHOUT touching any shared file.
%
%   make_baseline_models()                            % all three, skip existing
%   make_baseline_models({'adrc'})                    % just ADRC
%   make_baseline_models({'adrc','hinf','se3'}, true) % force regenerate
%
% The dSMC single model is TWO files:
%   discrete_smc_swarm_single.slx   -- top model
%   actual_discrete_smc_controller.slx -- a Subsystem Reference (the "Leader":
%       plant system_dynamics + control law dsmc_constraints), shared by
%       reference from smc single/Leader.
% Editing the control-law chart in place would mutate that SHARED file and hit
% the real dSMC arm. So for each ctrl this makes a private copy of BOTH files:
%   1. actual_<ctrl>_controller.slx : clone of the referenced subsystem, with
%      its Subsystem/MATLAB Function chart repointed
%      u = dsmc_constraints(...) -> u = <ctrl>_controller(...).
%   2. <ctrl>_swarm_single.slx : clone of the top model, with its
%      smc single/Leader ReferencedSubsystem repointed to actual_<ctrl>_controller.
% The plant, the [10 11 12 8 7 9] measurement selector, and all five ToWorkspace
% outputs (posout, velout, cmdout, ctrlout, rotvelout) come free with the clone.
%
% <ctrl>_controller.m must be on the path to SIMULATE the result, but not to
% GENERATE it (Simulink resolves the function name only at run time).

    if nargin < 1 || isempty(ctrls),     ctrls = {'adrc','hinf','se3'}; end
    if nargin < 2 || isempty(overwrite), overwrite = false;             end
    if ischar(ctrls) || isstring(ctrls), ctrls = cellstr(ctrls);        end

    src_top   = 'discrete_smc_swarm_single';
    src_ref   = 'actual_discrete_smc_controller';
    leader    = '/smc single/Leader';           % Subsystem Reference block (top model)
    chart_sub = '/Subsystem/MATLAB Function';   % control-law chart (referenced file)

    % Build constants_struct + constants_struct_bus in the base workspace so the
    % models' bus ports resolve while we load and re-save them. This is doing the
    % work the source model's PostLoadFcn was supposed to do: that callback is set
    % to the string 'init.m', which is not a valid command (it should be `init`),
    % so it fails and warns on every load. The clones inherit the same broken
    % callback. That does no harm, since anything that simulates them runs
    % constants.m first, but it is why this line has to stay.
    evalin('base', 'constants');

    for i = 1:numel(ctrls)
        c        = ctrls{i};
        dst_top  = [c '_swarm_single'];
        dst_ref  = ['actual_' c '_controller'];
        fn       = [c '_controller'];
        top_file = [dst_top '.slx'];
        ref_file = [dst_ref '.slx'];

        if exist(top_file, 'file') && ~overwrite
            warning('make_baseline_models:exists', ...
                '%s exists; skipping (pass overwrite=true to regenerate).', top_file);
            continue;
        end
        if overwrite
            force_delete(dst_top, top_file);
            force_delete(dst_ref, ref_file);
        end

        % --- 1. Private copy of the referenced subsystem, law swapped. ---
        clone_bd(src_ref, dst_ref);
        newscript = sprintf(['function u = MATLAB_Function(A, B, state, xd, x0, constants)\n' ...
                             'u = %s(A, B, state, xd, x0, constants);'], fn);
        set_chart_script([dst_ref chart_sub], newscript);
        save_system(dst_ref);
        got = get_chart_script([dst_ref chart_sub]);
        assert(contains(got, [fn '(']), 'make_baseline_models:swapFailed', ...
            'Law swap failed in %s (script now: %s)', ref_file, got);
        close_system(dst_ref, 0);

        % --- 2. Private copy of the top model, Leader reference repointed. ---
        clone_bd(src_top, dst_top);
        set_param([dst_top leader], 'ReferencedSubsystem', dst_ref);
        save_system(dst_top);
        ref_now = get_param([dst_top leader], 'ReferencedSubsystem');
        assert(strcmp(ref_now, dst_ref), 'make_baseline_models:repointFailed', ...
            'Leader repoint failed in %s (references "%s")', top_file, ref_now);
        close_system(dst_top, 0);

        fprintf('  OK  %-26s + %-30s ->  u = %s(...)\n', top_file, ref_file, fn);
    end
end

function clone_bd(src, dst)
    % Write src's block diagram to dst.slx, then load dst as its own model.
    load_system(src);
    try, close_system(dst, 0); catch, end
    save_system(src, dst, 'OverwriteIfChangedOnDisk', true);
    % Normalize loaded state across releases: drop both, reload the clone.
    try, close_system(src, 0); catch, end
    try, close_system(dst, 0); catch, end
    load_system(dst);
end

function force_delete(bd, file)
    % Close any in-memory copy and remove the file so a clone starts clean.
    if bdIsLoaded(bd), close_system(bd, 0); end
    if exist(file, 'file'), delete(file); end
end

function tf = bdIsLoaded(bd)
    tf = any(strcmp(bd, find_system('SearchDepth', 0, 'type', 'block_diagram')));
end

function set_chart_script(blk, newscript)
    % Prefer the documented MATLAB Function block configuration object; fall
    % back to the Stateflow chart API on older releases.
    try
        cfg = get_param(blk, 'MATLABFunctionConfiguration');
        cfg.FunctionScript = newscript;
        return;
    catch
    end
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
    if isempty(ch)
        error('set_chart_script:notFound', 'MATLAB Function chart not found: %s', blk);
    end
    ch.Script = newscript;
end

function s = get_chart_script(blk)
    try
        cfg = get_param(blk, 'MATLABFunctionConfiguration');
        s = cfg.FunctionScript;
        return;
    catch
    end
    s = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk).Script;
end
