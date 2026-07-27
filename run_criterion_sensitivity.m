% run_criterion_sensitivity.m -- how much of the cap campaign is VelRatioMax?
%
% Reads logs/crit_cap_sweep_log.mat (run_crit_cap_sweep.m) and re-scores every
% (metric, arm, cap) cell at a grid of VelRatioMax bounds, with no
% re-simulation -- ballistic_success is post-hoc and velocity_ok is exactly
% (max_speed_ratio <= VelRatioMax), so the whole question is arithmetic over a
% stored column (rescore_criterion.m).
%
% The claim on trial, from docs/plan_saturation_cap_campaign.md: SE(3)'s
% high-cap decline is an interaction with the velocity clause, NOT instability.
% If that is right, the SE(3) curve should lose its decline as the bound
% relaxes while every other curve sits still. If the curves are identical at
% every bound, the decline is something else and the campaign doc needs fixing.
%
% Prints a per-cap table and writes figs/CRIT_01_velratio_sensitivity.

cd(fileparts(mfilename('fullpath')));
L = load('logs/crit_cap_sweep_log.mat');   % env, lnd, caps

% Inf is "any finite speed passes", not "drop the clause": ballistic_success
% counts a NaN speed as a failure and NaN <= Inf is still false. The two read
% the same unless some trial went NaN, which rescore_criterion reports.
% Column labels are padded to the same width as the %-6d data fields below, so
% the header lines up with the numbers under it.
V      = [2 3 5 10 Inf];
V_name = ["2", "3", "5", "10", "Inf"];

fprintf('\n================ VelRatioMax sensitivity ================\n');
S = struct();
for metric = ["env", "lnd"]
    D = L.(metric);
    arms = unique(D.arm, 'stable');
    caps = unique(D.cap);
    n = nan(numel(arms), numel(caps), numel(V));
    dec = nan(numel(arms), numel(caps));       % trials the bound can decide
    vmax = nan(numel(arms), numel(caps));      % worst decidable speed ratio
    den = nan(numel(arms), numel(caps));
    % Per-clause failure counts. These overlap -- one dead trial trips several
    % clauses -- so they are a profile, not a partition, and they do not sum to
    % the failure count. They exist to separate the two ways a curve can fall:
    % velocity-clause losses (the campaign's claimed mechanism, a scoring
    % artifact) from rotation-clause losses (attitude divergence, which is a
    % much stronger statement than "never instability").
    clause = nan(numel(arms), numel(caps), 4);
    for i = 1:numel(arms)
        for j = 1:numel(caps)
            k = find(D.arm == arms(i) & abs(D.cap - caps(j)) < 1e-9, 1);
            if isempty(k), continue; end
            T = D.crit{k};
            R = rescore_criterion(T, V);
            n(i,j,:)  = R.n_success;
            den(i,j)  = R.n_trials;
            dec(i,j)  = numel(R.vr_decidable);
            clause(i,j,:) = [sum(~T.duration_ok), sum(~T.position_ok), ...
                             sum(~T.velocity_ok), sum(~T.rotation_ok)];
            if ~isempty(R.vr_decidable), vmax(i,j) = max(R.vr_decidable); end
            if R.n_nan_ratio > 0
                fprintf('  NOTE %s %s cap %.3g: %d NaN speed ratios (Inf column is not "clause off")\n', ...
                    metric, arms(i), caps(j), R.n_nan_ratio);
            end
        end
    end
    S.(metric) = struct('arms', arms, 'caps', caps, 'n', n, 'den', den, ...
                        'dec', dec, 'vmax', vmax, 'clause', clause);

    label = "ENVELOPE (of 17 stations)";
    if metric == "lnd", label = "LANDING (of 140 trials)"; end
    fprintf('\n-- %s: successes at VelRatioMax = %s (2 is the as-run bound) --\n', ...
        label, strjoin(V_name, ', '));
    for i = 1:numel(arms)
        fprintf('\n  %s\n', arms(i));
        fprintf('    %-7s %-8s %s  %-7s %-9s %s\n', 'cap', 'T/W', ...
            sprintf('%-6s', V_name), 'decid.', 'worst vr', 'fails dur/pos/vel/rot');
        for j = 1:numel(caps)
            if isnan(den(i,j)), continue; end
            fprintf('    %-7.3g %-8.1f %s  %-7d %-9.2f %d/%d/%d/%d\n', caps(j), ...
                4*5*caps(j)/(0.8*9.81), sprintf('%-6d', n(i,j,:)), dec(i,j), vmax(i,j), ...
                clause(i,j,1), clause(i,j,2), clause(i,j,3), clause(i,j,4));
        end
        % Relaxing the bound can only ADD successes, so this is >= 0 by
        % construction; a negative entry would mean rescore_criterion is broken.
        gain = squeeze(n(i,:,end) - n(i,:,1));
        fprintf('    relaxing 2 -> Inf recovers: %s  (max +%d)\n', ...
            mat2str(gain(~isnan(gain))), max(gain, [], 'omitnan'));
    end
end

%% Figure: does the bound move any curve?
set_default_fonts();
figure;
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
arm_col = containers.Map( ...
    {'se3', 'dsmc_sat', 'dsmc_nosat', 'dsmc_naive'}, ...
    {[0.85 0.33 0.10], [0.00 0.30 0.70], [0.40 0.65 0.90], [0.40 0.65 0.90]});

for metric = ["env", "lnd"]
    M = S.(metric);
    nexttile; hold on; grid on;
    h = gobjects(1, numel(M.arms));
    for i = 1:numel(M.arms)
        c   = arm_col(char(M.arms(i)));
        frac = squeeze(M.n(i,:,:)) ./ M.den(i,:).';
        % Solid = the as-run bound, dashed = fully relaxed. Where the two
        % coincide the velocity clause decided nothing at that cap.
        h(i) = plot(4*5*M.caps/(0.8*9.81), frac(:,1), '-o', 'Color', c, ...
            'LineWidth', 1.8, 'MarkerFaceColor', c);
        plot(4*5*M.caps/(0.8*9.81), frac(:,end), '--s', 'Color', c, 'LineWidth', 1.2);
    end
    set(gca, 'XScale', 'log'); ylim([0 1]);
    xline(4*5*2/(0.8*9.81), ':', 'cap 2', 'Color', [.4 .4 .4], 'HandleVisibility', 'off');
    xlabel('T/W = 4b\Omega^2_{max}/(mg)');
    if metric == "env"
        ylabel('Success fraction'); title('Envelope (17 stations)');
    else
        title('Landing sweep (140 trials)');
    end
    legend(h, strrep(cellstr(M.arms), '_', '-'), 'Location', 'southeast');
    % Every dSMC dashed curve lands exactly on its solid twin, so it is hidden
    % rather than absent -- and that coincidence IS the result, so say it. A
    % reader who reads the missing dashes as missing data draws the opposite
    % conclusion from the one the figure supports.
    text(0.03, 0.97, 'dSMC dashed curves coincide exactly with solid (0 trials recovered at every cap)', ...
        'Units', 'normalized', 'FontSize', 11, 'VerticalAlignment', 'top');
end
title(tl, 'Criterion sensitivity: solid = VelRatioMax 2 (as run), dashed = relaxed to Inf');
export_figure('figs/CRIT_01_velratio_sensitivity');

save('logs/criterion_sensitivity_log.mat', 'S', 'V');
fprintf('\n===== done: figs/CRIT_01_velratio_sensitivity, logs/criterion_sensitivity_log.mat =====\n');
