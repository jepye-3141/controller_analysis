old = load('trajectory_optimization_log_baseline.mat');
new = load('trajectory_optimization_log.mat');

fprintf('--- structural ---\n');
fprintf('size(results) old=%s new=%s\n', mat2str(size(old.results)), mat2str(size(new.results)));

% Recompute landing_ratio for both
function lr = recompute_landing_ratio(s)
    n_d = size(s.results, 1);
    n_c = size(s.results, 2);
    n_b = size(s.results, 3);
    stable_arr = reshape([s.results.stable], n_d, n_c, n_b);
    landing_success = zeros(n_d, n_c);
    landing_total   = zeros(n_d, n_c);
    for i = 1:n_d
        for k = 1:n_c
            for nb = 1:n_b
                tgt = s.results(i, k, nb).target_deploy_idx;
                if isnan(tgt); continue; end
                landing_total(tgt, k) = landing_total(tgt, k) + 1;
                if stable_arr(i, k, nb)
                    landing_success(tgt, k) = landing_success(tgt, k) + 1;
                end
            end
        end
    end
    lr = landing_success ./ max(landing_total, 1);
end

lr_old = recompute_landing_ratio(old);
lr_new = recompute_landing_ratio(new);

fprintf('--- aggregates ---\n');
fprintf('landing_ratio max abs diff: %.3e\n', max(abs(lr_old(:) - lr_new(:))));

stable_old = reshape([old.results.stable], size(old.results));
stable_new = reshape([new.results.stable], size(new.results));
fprintf('stable mismatch count: %d / %d\n', sum(stable_old(:) ~= stable_new(:)), numel(stable_old));

ttl_old = reshape([old.results.time_to_land], size(old.results));
ttl_new = reshape([new.results.time_to_land], size(new.results));
ttl_diff = ttl_old - ttl_new;
ttl_diff_finite = ttl_diff(isfinite(ttl_diff));
if isempty(ttl_diff_finite)
    fprintf('time_to_land: no finite diffs to compare\n');
else
    fprintf('time_to_land max abs diff (finite): %.3e\n', max(abs(ttl_diff_finite)));
end

fprintf('--- spot-check trajectory(3,4,3) ---\n');
t_old = old.results(3,4,3).trajectory;
t_new = new.results(3,4,3).trajectory;
fprintf('pos size old=%s new=%s\n', mat2str(size(t_old.pos)), mat2str(size(t_new.pos)));
fprintf('pos max abs diff: %.3e\n', max(abs(t_old.pos(:) - t_new.pos(:))));
fprintf('vel max abs diff: %.3e\n', max(abs(t_old.vel(:) - t_new.vel(:))));
fprintf('ctrl max abs diff: %.3e\n', max(abs(t_old.ctrl(:) - t_new.ctrl(:))));
