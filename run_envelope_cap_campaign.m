% run_envelope_cap_campaign.m -- Thread B: does the saturation cap move the
% 17-point ballistic ENVELOPE the way it moved the 140-trial landing sweep?
%
% Phase 1 (landing sweep) found dSMC-sat cap-FLAT for cap>=1 and SE(3) carrying
% the cap story (non-monotonic, peak 0.743 @ cap3 via the velocity-ratio gate).
% This cross-checks that on the envelope metric. Pre-registered H4: the envelope
% is LESS cap-sensitive than the landing sweep because it over-weights the
% cap-invariant ascending regime (dSMC 0/anything there).
%
% Arms (envelope-native, = analysis.m fig 39): dSMC-sat (Plan A+, cap-dependent),
% dSMC-nosat (unclipped -> cap-INVARIANT control), SE(3) (naive clip,
% cap-dependent). Phase 1's landing sweep used dSMC-*naive* (clipped, no
% allocation) as its middle arm; swap a naive config in below if strict Phase-1
% arm parity is wanted -- nosat is used here as the free cap-invariant control.
%
% Compute: per cap, model 'dsmc' runs nosat+sat (34 sims) + model 'se3' (17) =
% 51 serial rapid-accel 30 s sims; x3 caps = 153 sims + builds. Serial (single
% MATLAB, no parpool).
%
% Writes figs/ENV_CAP_01.{png,eps} + logs/envelope_cap_campaign_log.mat.

cd(fileparts(mfilename('fullpath')));   % project root = this script's folder
addpath('Ballistics_Simulation-master');
set_default_fonts();

TW   = @(cap) 4*5*cap / (0.8*9.81);   % T/W axis label: b=5 mixer thrust coeff, m=0.8 kg, g=9.81
caps = [1 2 5];
base = operational_launch();
nC   = numel(caps);

arm_names = {'dsmc_sat', 'dsmc_nosat', 'se3'};
arm_disp  = {'dSMC-sat', 'dSMC-nosat', 'SE(3)'};
nA        = numel(arm_names);

nsucc      = nan(nA, nC);        % n_success per arm per cap
npts       = nan(1, nC);         % n_points per cap (physics-dependent)
% among FAILED stations, count per-clause violations [duration position velocity rotation]
failclause = zeros(nA, nC, 4);

fprintf('\n===== ENVELOPE CAP CAMPAIGN (Thread B) =====\n');
for c = 1:nC
    cap = caps(c);
    fprintf('-- cap %.2f (T/W ~ %.1f) --\n', cap, TW(cap));

    tp_d = base; tp_d.Omega2_max = cap; tp_d.model = 'dsmc';   % nosat + sat
    od   = sweep_ballistic_envelope(tp_d);
    tp_s = base; tp_s.Omega2_max = cap; tp_s.model = 'se3';
    os   = sweep_ballistic_envelope(tp_s);

    assert(isequal(od.test_points, os.test_points), 'cap %g: arm station rows disagree', cap);
    npts(c) = od.n_points;
    src = {od.dsmc_sat, od.dsmc_nosat, os.se3};

    for a = 1:nA
        r          = src{a};
        nsucc(a,c) = r.n_success;
        T          = r.crit_table;
        f          = ~T.stable;
        failclause(a,c,:) = [sum(f & ~T.duration_ok), sum(f & ~T.position_ok), ...
                             sum(f & ~T.velocity_ok), sum(f & ~T.rotation_ok)];
        fprintf('   %-11s %2d/%d  fail[dur %d pos %d vel %d rot %d]\n', ...
            arm_names{a}, nsucc(a,c), npts(c), squeeze(failclause(a,c,:)));
    end
end

reach = nsucc ./ npts;

%% Figure ENV_CAP_01 -- masks vs cap (L) + SE(3) failure attribution vs cap (R)
figure('Position', [100 100 1500 560]);

% (L) envelope mask fraction vs cap, per arm
subplot(1,2,1); hold on; grid on;
mk = {'-o','--s',':^'};
for a = 1:nA
    plot(caps, reach(a,:), mk{a}, 'LineWidth', 1.5, 'MarkerSize', 8, ...
        'DisplayName', arm_disp{a});
end
xlabel('per-rotor cap \Omega^2_{max}'); ylabel('envelope success fraction (/n_{pts})');
title('Envelope masks vs saturation cap'); legend('Location','best');
ylim([0 1]); xticks(caps);

% (R) SE(3) per-clause failure attribution vs cap -- does the landing-sweep
% velocity-gate mechanism (SE(3) non-monotonicity) show up on the envelope?
% Read the stack heights as counts per clause, NOT as a partition: a station that
% blows up trips several clauses at once, so the stack total exceeds the number of
% failed stations. Only the per-clause pattern across caps is meaningful.
subplot(1,2,2);
se3_idx = find(strcmp(arm_names,'se3'));
bar(caps, squeeze(failclause(se3_idx,:,:)), 'stacked');
xlabel('per-rotor cap \Omega^2_{max}');
ylabel('SE(3) clause violations (stations may trip >1)');
title('SE(3) failure attribution vs cap');
legend({'duration','position','velocity','rotation'}, 'Location','best');
xticks(caps);

export_figure('figs/ENV_CAP_01');

%% Persist
save('logs/envelope_cap_campaign_log.mat', ...
    'caps', 'arm_names', 'arm_disp', 'nsucc', 'npts', 'reach', 'failclause');

fprintf('\nsummary (success fraction):\n');
for a = 1:nA
    fprintf('  %-11s %s\n', arm_disp{a}, num2str(reach(a,:), '%6.3f'));
end
fprintf('H4 read: compare dSMC-sat + SE(3) cap-spread here vs landing-sweep\n');
fprintf('  (sat flat 0.607 cap>=1; SE(3) peak 0.743 @cap3). Wrote figs/ENV_CAP_01.\n');
fprintf('===== END ENVELOPE CAP CAMPAIGN =====\n');
