function set_omega2_max(val)
% set_omega2_max  Rewrite the Omega2_max literal on disk (regexprep + fwrite)
% in EVERY file that carries a private copy of the per-rotor clip bound -- a
% source edit, not a runtime parameter. Saturation-sweep helper; leaves those
% files git-dirty at the last value set.
%
% Two files hold the literal and MUST move in lockstep, or the baseline
% comparison's "identical actuator limits across all arms" invariant breaks:
%   dsmc_constraints.m -- the dSMC STEP-12b clip (canonical source).
%   apply_rotor_clip.m -- the shared clip the H-inf / ADRC / SE(3) baselines call.
% Rewriting only the first (the historical behaviour) would silently give the
% dSMC arm a different rotor ceiling than its peers -- unequal actuator boxes
% with no error (baseline-controllers review, 2026-07-20).
here  = fileparts(mfilename('fullpath'));
files = {'dsmc_constraints.m', 'apply_rotor_clip.m'};
pat   = 'Omega2_max\s*=\s*[\d.eE+\-]+\s*;';
for i = 1:numel(files)
    fp = fullfile(here, files{i});
    s  = fileread(fp);
    if isempty(regexp(s, pat, 'once'))
        error('set_omega2_max: pattern not found in %s', fp);
    end
    s2 = regexprep(s, pat, sprintf('Omega2_max = %g;', val), 'once');
    fid = fopen(fp, 'w');
    fwrite(fid, s2);
    fclose(fid);
    fprintf('set_omega2_max: Omega2_max = %g in %s\n', val, files{i});
end
end
