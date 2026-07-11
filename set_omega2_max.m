function set_omega2_max(val)
% set_omega2_max  Rewrite the Omega2_max literal in dsmc_constraints.m on
% disk (regexprep + fwrite) -- a source edit, not a runtime parameter.
% Saturation-sweep helper; leaves that file git-dirty at the last value set.
fp = fullfile(fileparts(mfilename('fullpath')), 'dsmc_constraints.m');
s  = fileread(fp);
if isempty(regexp(s, 'Omega2_max\s*=\s*[\d.eE+\-]+\s*;', 'once'))
    error('set_omega2_max: pattern not found in %s', fp);
end
s2 = regexprep(s, 'Omega2_max\s*=\s*[\d.eE+\-]+\s*;', sprintf('Omega2_max = %g;', val), 'once');
fid = fopen(fp, 'w');
fwrite(fid, s2);
fclose(fid);
fprintf('set_omega2_max: Omega2_max = %g\n', val);
end
