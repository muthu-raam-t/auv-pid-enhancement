%RUN_FIX1_TUNING Fix 1: automatic PID gain tuning, with before/after table.
%   Tunes on the first 45 s, then scores baseline and tuned gains on
%   the full run, the tuning window, and the held-out last 45 s.

clear; clc;
p = config();
t_train = 45;

g0 = baseline_gains();
fprintf('Tuning on t = 0..%d s (this takes a minute or two)...\n', t_train);
[g1, info] = tune_pid(p, t_train);

base = simulate_pid_gains(p, g0);
tune = simulate_pid_gains(p, g1);

rows = {'Baseline (hand-tuned)', base; 'Fix 1 (auto-tuned)', tune};
fprintf('\n%-24s %10s %12s %12s %10s %10s\n', 'Controller', 'RMSE full', ...
        sprintf('RMSE 0-%ds', t_train), sprintf('RMSE %d-%ds', t_train, p.Tf), ...
        'Mean |u|', 'Sat. %');
for i = 1:size(rows, 1)
    o = rows{i, 2};
    e2 = o.pos_error.^2;
    tr = o.t <= t_train;
    r_tr = sqrt(mean(e2(tr)));
    r_te = sqrt(mean(e2(~tr)));
    effort = mean(abs(o.U(:)));
    lim = repmat(p.u_max, 1, size(o.U, 2));
    sat = 100 * mean(abs(o.U(:)) >= abs(lim(:)) - 1e-9);
    fprintf('%-24s %10.4f %12.4f %12.4f %10.3f %9.1f\n', rows{i, 1}, ...
            o.rmse_pos, r_tr, r_te, effort, sat);
end

fprintf('\nBaseline gains:\n'); disp(reshape(g0, 3, 4));
fprintf('Tuned gains (columns: x, y, z, psi; rows: Kp, Ki, Kd):\n');
disp(reshape(g1, 3, 4));

if ~exist('results', 'dir'), mkdir('results'); end
save('results/fix1_tuned_gains.mat', 'g0', 'g1', 'info', 'base', 'tune');

figure('Color', 'w');
plot(base.t, base.pos_error, 'LineWidth', 1.4); hold on;
plot(tune.t, tune.pos_error, 'LineWidth', 1.4);
xline(t_train, '--k', 'tuning window ends');
legend('Baseline', 'Auto-tuned', 'Location', 'best');
xlabel('Time [s]'); ylabel('Position error [m]');
title('Fix 1: position error, baseline vs auto-tuned PID');
grid on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
