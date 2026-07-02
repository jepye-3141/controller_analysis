function set_omega2_max(val)
% Programmatically rewrite the Omega2_max line in dsmc_constraints.m.
% Used by the saturation sweep to vary the per-rotor Omega^2 saturation
% bound without manual file editing.
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
