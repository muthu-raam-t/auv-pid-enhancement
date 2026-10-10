function [g_best, info] = tune_pid(p, t_train)
%TUNE_PID Fix 1: replace hand-tuned gains with optimiser-found gains.
%   Minimises position RMSE over the first t_train seconds only
%   (default 45 s). The rest of the run is held out so the result is
%   not just a fit to the same data it is scored on.
%
%   Gains are searched in log-space and limited to roughly 0.14x - 7.4x
%   of the baseline values, so the optimiser cannot return absurd gains.

if nargin < 2 || isempty(t_train), t_train = 45; end

g0 = baseline_gains();
map = @(th) g0 .* exp(2*tanh(th));       % bounded search space

obj = @(th) objective(map(th), p, t_train);

opts = optimset('Display', 'iter', 'MaxFunEvals', 800, 'MaxIter', 400, ...
                'TolX', 1e-3, 'TolFun', 1e-5);
[th_best, f_best, ~, output] = fminsearch(obj, zeros(1, 12), opts);

g_best = map(th_best);
info.train_rmse = f_best;
info.iterations = output.iterations;
info.evals = output.funcCount;
info.t_train = t_train;
end

function f = objective(g, p, t_train)
out = simulate_pid_gains(p, g, t_train);
if ~out.ok
    f = 1e3;                              % unstable run: reject
else
    f = out.rmse_pos;
end
end
