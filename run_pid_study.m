%% run_pid_study.m - runs the PID baseline on the ocean-current disturbance
clear; clc; close all;
addpath(genpath(pwd));

p = config();
res = simulate_pid_baseline(p);
fprintf('PID baseline | RMSE pos: %.4f m\n', res.rmse_pos);

if ~exist('results', 'dir'); mkdir('results'); end
save(fullfile('results', 'pid_baseline.mat'), 'res');

fig = figure('Color', 'white');
ax = axes(fig, 'Color', 'white', 'XColor', 'black', 'YColor', 'black');
hold(ax, 'on');
plot(ax, res.Xref(1,:), res.Xref(2,:), '--', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.5);
plot(ax, res.X(1,:), res.X(2,:), 'Color', [0.2 0.45 0.7], 'LineWidth', 1.2);
axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
xlabel(ax, 'x [m]', 'Color', 'black'); ylabel(ax, 'y [m]', 'Color', 'black');
title(ax, 'PID baseline trajectory', 'Color', 'black');
legend(ax, {'Reference', 'PID baseline'}, 'TextColor', 'black', 'Color', 'white', 'EdgeColor', 'black');
