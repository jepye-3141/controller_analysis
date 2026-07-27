function R = rescore_criterion(crit_table, vel_grid)
% rescore_criterion  Re-score ballistic_success outcomes at alternative
% VelRatioMax bounds, with no re-simulation.
%
% ballistic_success is post-hoc: it reads logged traces and never touches
% in-model state, and its four clauses are independent conjuncts,
%
%   success = duration_ok & position_ok & velocity_ok & rotation_ok
%
% with velocity_ok exactly (max_speed_ratio <= VelRatioMax). crit_table stores
% all four flags AND the measured max_speed_ratio per trial, so the whole
% VelRatioMax sensitivity question is arithmetic over a stored column. Varying
% any of the other three bounds WOULD need the traces back; only this one is
% free.
%
% crit_table : table from sweep_landing_centroid or sweep_ballistic_envelope
%              (both emit identical VariableNames). One row per attempted trial.
% vel_grid   : VelRatioMax values to score at. Must include the as-run 2 --
%              that entry is the regression gate, not decoration.
%
% R : struct
%   .vel_grid     the input grid
%   .n_trials     height(crit_table)
%   .n_success    1 x numel(vel_grid) successes at each bound
%   .mask         n_trials x numel(vel_grid) logical per-trial outcome
%   .vr_decidable sorted max_speed_ratio over the trials that pass the OTHER
%                 three clauses -- i.e. every trial the velocity bound is
%                 actually deciding. n_success(V) == nnz(vr_decidable <= V), so
%                 this one vector is the entire sensitivity curve, and where its
%                 values sit says whether a relaxed bound rescues real flights
%                 (ratios bunched just past 2) or nothing at all (ratios in the
%                 tens, i.e. genuine divergence).
%   .n_nan_ratio  rows with max_speed_ratio = NaN (see below)
%
% NaN and Inf. ballistic_success maxes the speed trace with 'includenan', so one
% NaN sample gives max_speed_ratio = NaN and the bare <= sends velocity_ok
% false: NaN is a failure. This keeps that convention at EVERY bound, Inf
% included, where NaN <= Inf is still false. So "VelRatioMax = Inf" here means
% "any finite speed passes", not "delete clause (c)". The readings differ only
% on NaN rows, which .n_nan_ratio counts -- and a trial violent enough to make
% NaN normally trips the 1e5 blowup guard first and fails duration_ok anyway.

arguments
    crit_table table
    vel_grid (1,:) double
end

need = {'stable', 'duration_ok', 'position_ok', 'rotation_ok', 'max_speed_ratio'};
missing = need(~ismember(need, crit_table.Properties.VariableNames));
assert(isempty(missing), 'rescore_criterion:schema', ...
    'crit_table is missing column(s): %s', strjoin(missing, ', '));

vr    = crit_table.max_speed_ratio;
other = crit_table.duration_ok & crit_table.position_ok & crit_table.rotation_ok;
mask  = other & (vr <= vel_grid);   % n_trials x n_grid by implicit expansion

% Regression gate. At the as-run bound the reconstruction has to reproduce the
% stored outcome on every row; if it does not, the conjunction above is wrong
% and every relaxed-bound number downstream is meaningless. Cheap to check,
% fatal to skip.
j2 = find(vel_grid == 2, 1);
assert(~isempty(j2), 'rescore_criterion:noAnchor', ...
    'vel_grid must contain the as-run bound 2 so the reconstruction can be checked.');
% Not an assert: assert() evaluates its message arguments even when the
% condition holds, and bad(1) indexes an empty array on the passing path.
bad = find(mask(:, j2) ~= crit_table.stable);
if ~isempty(bad)
    error('rescore_criterion:mismatch', ...
        'Re-score at VelRatioMax=2 disagrees with the stored stable flag on %d of %d rows (first: row %d).', ...
        numel(bad), numel(vr), bad(1));
end

R = struct('vel_grid', vel_grid, 'n_trials', numel(vr), ...
    'n_success', sum(mask, 1), 'mask', mask, ...
    'vr_decidable', sort(vr(other)), 'n_nan_ratio', sum(isnan(vr)));

end
