%RUN_FIX1B_NOISE Fix 1b: does auto-tuning survive noise and thruster lag?
%   Compares three gain sets, all scored on noisy runs with seeds the
%   tuner never saw:
%     1. Baseline (hand-tuned)
%     2. Fix 1  (tuned in ideal conditions)   - needs results/fix1_tuned_gains.mat
%     3. Fix 1b (tuned under noise + lag + jitter penalty)

clear; clc;
p = config();

if ~exist('results/fix1_tuned_gains.mat', 'file')
    error('Run run_fix1_tuning first (it saves results/fix1_tuned_gains.mat).');
end
S = load('results/fix1_tuned_gains.mat');
g_base = baseline_gains();
g_ideal = S.g1;

train_seeds = [1 2 3];
test_seeds  = 101:105;

fprintf('Tuning under noise on seeds %s (a few minutes)...\n', mat2str(train_seeds));
[g_noise, info] = tune_pid_noisy(p, train_seeds, 0.02);
fprintf('Done: %d iterations, %d evaluations.\n', info.iterations, info.evals);

sets = {'Baseline (hand-tuned)', g_base;
        'Fix 1  (ideal-tuned)',  g_ideal;
        'Fix 1b (noise-tuned)',  g_noise};

ideal = struct('vel_noise',0,'pos_noise',0,'yaw_noise',0,'tau',0);

fprintf('\n%-24s %16s %12s %10s %14s\n', 'Controller', 'RMSE noisy [m]', ...
        'Jitter', 'Mean |u|', 'RMSE ideal [m]');
R = zeros(numel(test_seeds), size(sets, 1));
for j = 1:size(sets, 1)
    g = sets{j, 2};
    r = zeros(size(test_seeds)); c = r; e = r;
    for i = 1:numel(test_seeds)
        o = simulate_pid_noisy(p, g, struct('seed', test_seeds(i)));
        r(i) = o.rmse_pos; c(i) = o.chatter; e(i) = o.effort;
    end
    R(:, j) = r(:);
    oi = simulate_pid_noisy(p, g, ideal);
    fprintf('%-24s %8.4f +/- %.4f %12.3f %10.3f %14.4f\n', sets{j, 1}, ...
            mean(r), std(r), mean(c), mean(e), oi.rmse_pos);
end

fprintf('\nNoise-tuned gains (columns: x, y, z, psi; rows: Kp, Ki, Kd):\n');
disp(reshape(g_noise, 3, 4));
fprintf('Ideal-tuned gains for comparison:\n');
disp(reshape(g_ideal, 3, 4));

if ~exist('results', 'dir'), mkdir('results'); end
save('results/fix1b_noise_tuned.mat', 'g_base', 'g_ideal', 'g_noise', 'info', 'R');

figure('Color', 'w');
m = mean(R, 1); s = std(R, 0, 1);
bar(m, 'FaceColor', [0.3 0.5 0.8]); hold on;
errorbar(1:3, m, s, 'k.', 'LineWidth', 1.4);
set(gca, 'XTick', 1:3, 'XTickLabel', {'Baseline', 'Ideal-tuned', 'Noise-tuned'}, ...
    'Color', 'w', 'XColor', 'k', 'YColor', 'k');
ylabel('Position RMSE under noise [m]');
title('Fix 1b: PID gains scored on noisy runs (5 unseen seeds)');
grid on;
